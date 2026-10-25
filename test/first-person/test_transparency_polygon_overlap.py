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
// Independent cell-first reference: candidate-first binning must retain
// exactly the same conservative bounds, even at off-grid cell edges.
template<typename Candidates>
size_t cellFirstReference(const Candidates& candidates,
    int32_t left,int32_t top,int32_t right,int32_t bottom,int32_t cellSize)
{
    if(cellSize<=0||right<=left||bottom<=top) return 0;
    size_t maximum=0;
    for(int32_t y=top;y<bottom;y+=cellSize)
    for(int32_t x=left;x<right;x+=cellSize)
    {
        const int32_t cellRight=std::min(x+cellSize,right);
        const int32_t cellBottom=std::min(y+cellSize,bottom);
        size_t count=0;
        for(const auto& candidate:candidates)
        {
            if(candidate.x1<=x||candidate.x0>=cellRight
                ||candidate.y1<=y||candidate.y0>=cellBottom) continue;
            if(FirstPersonProjectedPolygonMayTouchCell(
                    candidate.projectedPolygons[0],x,y,cellRight,cellBottom)
                || FirstPersonProjectedPolygonMayTouchCell(
                    candidate.projectedPolygons[1],x,y,cellRight,cellBottom))
                ++count;
        }
        maximum=std::max(maximum,count);
    }
    return maximum;
}
int main()
{
    std::mt19937 rng(20261025);
    std::uniform_real_distribution<float> general(-60,190);
    // Full and clipped candidates, arbitrary screen origins, and cells
    // whose final right/bottom edges are not cell-aligned.
    for(int n=0;n<1200;++n)
    {
        std::vector<Candidate> candidates(1+rng()%30);
        for(auto& c:candidates)
        {
            c.x0=int32_t(rng()%220)-55;
            c.y0=int32_t(rng()%220)-55;
            c.x1=c.x0+int32_t(rng()%165);
            c.y1=c.y0+int32_t(rng()%165);
            for(auto& p:c.projectedPolygons)
                p=triangle(general(rng),general(rng),
                           general(rng),general(rng),
                           general(rng),general(rng));
        }
        const int32_t cellSize=std::array<int32_t,3>{8,16,25}[n%3];
        const auto expected=cellFirstReference(
            candidates,-13,7,133,141,cellSize);
        const auto result=FirstPersonMaximumPolygonCellOverlap(
            candidates,-13,7,133,141,cellSize);
        assert(expected==result);
    }

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
        // The adaptive 8px pass is also an exact conservative bound;
        // the minimum of both cannot lose any real raster fragment.
        const size_t refined=FirstPersonMaximumPolygonCellOverlap(
            candidates,0,0,128,128,8);
        assert(refined>=rasterMaximum);
        assert(std::min(bounded,refined)>=rasterMaximum);
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
    // The earlier cell-first bound deliberately overcounted rectangles
    // even with inverted edges when both edges fell inside one cell.
    // Preserve that conservative behavior instead of treating them as empty.
    Candidate malformed{};
    malformed.x0=13;malformed.x1=12;malformed.y0=13;malformed.y1=12;
    malformed.projectedPolygons[0]=triangle(0,0,15,0,0,15);
    assert(cellFirstReference(std::array<Candidate,1>{malformed},0,0,32,32,16)==1);
    assert(FirstPersonMaximumPolygonCellOverlap(
        std::array<Candidate,1>{malformed},0,0,32,32,16)==1);
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
        self.assertIn("tile.candidates.size() <= 48", src)
        self.assertIn("tile.x0, tile.y0, tile.x1, tile.y1, 8", src)
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
