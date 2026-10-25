"""Compiled conservative pixel-coverage proof for first-person depth peeling."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
HEADER = ROOT / "src/openrct2-ui/drawing/engines/opengl/FirstPersonTransparencyCoverage.h"
RENDERER = ROOT / "src/openrct2-ui/drawing/engines/opengl/FirstPersonGLRenderer.cpp"

CXX = r'''
#include "FirstPersonTransparencyCoverage.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <limits>
#include <random>
#include <vector>
using namespace OpenRCT2::Ui;
struct Candidate {
    int32_t x0=0, y0=0, x1=128, y1=128;
    std::array<FirstPersonProjectedScreenPolygon,2> projectedPolygons{};
};
static FirstPersonProjectedScreenPolygon triangle(float ax,float ay,float bx,float by,float cx,float cy)
{
    FirstPersonProjectedScreenPolygon p{};
    p.count=3;
    p.points[0]={ax,ay};p.points[1]={bx,by};p.points[2]={cx,cy};
    return p;
}
int main()
{
    std::mt19937 rng(20261025);
    std::uniform_real_distribution<float> uni(-30,158);
    for (int n=0;n<800;++n)
    {
        std::vector<Candidate> candidates(1+rng()%14);
        for (auto& v:candidates)
            for (auto& p:v.projectedPolygons)
                p=triangle(uni(rng),uni(rng),uni(rng),uni(rng),uni(rng),uni(rng));
        const size_t bounded=FirstPersonMaximumPolygonCellOverlap(candidates,0,0,128,128);
        size_t rasterMaximum=0;
        for(int y=0;y<128;++y)
        for(int x=0;x<128;++x)
        {
            size_t count=0;
            for (const auto& candidate:candidates)
            {
                bool covered=false;
                for(const auto& p:candidate.projectedPolygons)
                {
                    const auto& a=p.points[0];
                    const auto& b=p.points[1];
                    const auto& c=p.points[2];
                    const auto edge=[](const auto& u,const auto& v,float x,float y) {
                        return (v[0]-u[0])*(y-u[1])-(v[1]-u[1])*(x-u[0]);
                    };
                    const float ab=edge(a,b,x+0.5f,y+0.5f);
                    const float bc=edge(b,c,x+0.5f,y+0.5f);
                    const float ca=edge(c,a,x+0.5f,y+0.5f);
                    if((ab>=0&&bc>=0&&ca>=0)||(ab<=0&&bc<=0&&ca<=0))
                        covered=true;
                }
                if(covered) ++count;
            }
            rasterMaximum=std::max(rasterMaximum,count);
        }
        assert(bounded>=rasterMaximum);
    }
    Candidate a,b;
    a.projectedPolygons[0]=triangle(0,0,45,0,0,45);
    a.projectedPolygons[1]=a.projectedPolygons[0];
    b.projectedPolygons[0]=triangle(128,128,83,128,128,83);
    b.projectedPolygons[1]=b.projectedPolygons[0];
    assert(FirstPersonMaximumPolygonCellOverlap(std::array<Candidate,2>{a,b},0,0,128,128)==1);
    FirstPersonProjectedScreenPolygon absent{};
    assert(!FirstPersonProjectedPolygonMayTouchCell(absent,0,0,16,16));
    a.projectedPolygons[0].points[0][0]=std::numeric_limits<float>::quiet_NaN();
    assert(FirstPersonProjectedPolygonMayTouchCell(a.projectedPolygons[0],0,0,16,16));
    std::cout << "conservative polygon coverage verified" << std::endl;
}
'''

class TransparencyPolygonBoundTest(unittest.TestCase):
    def test_render_path_keeps_both_safe_upper_bounds(self):
        src = RENDERER.read_text()
        hdr = HEADER.read_text()
        self.assertIn("FirstPersonMaximumRectangleOverlap(tile.candidates)", src)
        self.assertIn("FirstPersonMaximumPolygonCellOverlap(", src)
        self.assertIn("std::min(rectangleBound, triangleBound)", src)
        self.assertIn("projectedPolygons[triangleStart / 3]", src)
        self.assertIn("kRasterMargin = 2.0f", hdr)
        self.assertIn("return true; // Unknown coverage", hdr)
        self.assertNotIn("waitForPeelCoverageQuery", src)

    @unittest.skipUnless(shutil.which("g++") or shutil.which("clang++"), "C++ compiler unavailable")
    def test_compiled_randomized_pixel_centres_never_exceed_bound(self):
        compiler = shutil.which("g++") or shutil.which("clang++")
        with tempfile.TemporaryDirectory(prefix="first-person-poly-") as temp:
            cpp = Path(temp) / "test.cpp"
            exe = Path(temp) / "test"
            cpp.write_text(CXX)
            subprocess.run([compiler, "-std=c++20", "-O2", "-Wall", "-Wextra", "-Werror",
                            "-I", str(HEADER.parent), str(cpp), "-o", str(exe)],
                           check=True, timeout=30)
            subprocess.run([str(exe)], check=True, timeout=35)

if __name__ == "__main__":
    unittest.main()
