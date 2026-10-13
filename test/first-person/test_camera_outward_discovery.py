"""Camera-nearest discovery must preserve coverage and frame-budget priority."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]

class NearFirstDiscoveryTest(unittest.TestCase):
    def test_cpp_discovery_order_and_coverage(self):
        compiler = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++")
        if not compiler:
            self.skipTest("C++ compiler not available")
        code = r"""
#include "src/openrct2/paint/FirstPersonStreaming.h"
#include "src/openrct2/paint/FirstPersonAsync.h"
#include <cassert>
#include <chrono>
#include <set>
#include <utility>
#include <vector>
using namespace OpenRCT2::Paint;
int main() {
    FirstPersonCamera camera{};
    camera.position = {50.0f*32, 50.0f*32, 10};
    camera.hasExplicitBasis = true;
    camera.explicitBasis = {{1,0,0},{0,1,0},{0,0,1}};
    const FirstPersonFrustum frustum(camera, 90, 1, 0, 100000);
    auto bounds = [](int32_t,int32_t,int32_t,int32_t) {
        return FirstPersonRegionBounds{{0,0,0},{100000,100000,100000},true};
    };
    std::vector<std::pair<int32_t,int32_t>> leaves;
    auto leaf = [&](int32_t x,int32_t y,int32_t xx,int32_t yy) {
        assert(xx-x==16 && yy-y==16);
        leaves.emplace_back(x,y);
    };
    VisitFirstPersonRegions(frustum,0,0,64,64,leaf,bounds);
    assert(leaves.size()==16 && leaves.front()==std::make_pair(48,48));
    assert(std::set<std::pair<int32_t,int32_t>>(leaves.begin(),leaves.end()).size()==16);
    FirstPersonFrameBudget budget(2,std::chrono::seconds(5));
    std::vector<std::pair<int32_t,int32_t>> admitted;
    auto admit = [&](int32_t x,int32_t y,int32_t,int32_t) {
        if(budget.take()) admitted.emplace_back(x,y);
    };
    VisitFirstPersonRegions(frustum,0,0,64,64,admit,bounds);
    assert(admitted.size()==2 && admitted.front()==std::make_pair(48,48));
    std::vector<std::pair<int32_t,int32_t>> tiles;
    auto record = [&](int32_t x,int32_t y){ tiles.emplace_back(x,y); };
    VisitFirstPersonLeafTilesNearEye({40*32.0f,42*32.0f,0},32,32,48,48,record);
    assert(tiles.size()==256 && tiles.front()==std::make_pair(40,42));
    assert(std::set<std::pair<int32_t,int32_t>>(tiles.begin(),tiles.end()).size()==256);
    tiles.clear();
    VisitFirstPersonLeafTilesNearEye({100000,-100000,0},32,32,48,48,record);
    assert(tiles.size()==256 && tiles.front()==std::make_pair(47,32));
    assert(std::set<std::pair<int32_t,int32_t>>(tiles.begin(),tiles.end()).size()==256);
    camera.position = {3*32.0f,3*32.0f,0};
    const FirstPersonFrustum moved(camera,90,1,0,100000);
    leaves.clear();
    VisitFirstPersonRegions(moved,0,0,64,64,leaf,bounds);
    assert(leaves.size()==16 && leaves.front()==std::make_pair(0,0));
}
"""
        with tempfile.TemporaryDirectory() as folder:
            cpp = Path(folder)/"near_first.cpp"
            exe = Path(folder)/"near_first"
            cpp.write_text(code,encoding="utf-8")
            compile_run = subprocess.run(
                [compiler,"-std=c++20","-O0","-I",str(ROOT),str(cpp),"-o",str(exe)],
                capture_output=True,text=True)
            self.assertEqual(compile_run.returncode,0,compile_run.stderr)
            result = subprocess.run([str(exe)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)

if __name__ == "__main__":
    unittest.main()
