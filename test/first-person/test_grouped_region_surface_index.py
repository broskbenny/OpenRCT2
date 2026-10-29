"""Compiled region-index traversal and production registration regression."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2] / "src/openrct2/paint"


class GroupedRegionSurfaceIndexTest(unittest.TestCase):
    def test_registration_and_resumable_assembly_are_connected(self):
        renderer = (ROOT / "FirstPersonRenderer.cpp").read_text()
        core = (ROOT / "FirstPersonRenderer.Core.inc").read_text()
        world = (ROOT / "FirstPersonRenderer.World.inc").read_text()
        self.assertIn('#include "FirstPersonRegionSurfaceIndex.h"', renderer)
        self.assertEqual(core.count("IndexFirstPersonSurfacesByGpuRegion(cached.surfaces)"), 2)
        self.assertEqual(core.count("FirstPersonRegionSurfaceIndex regionSurfaceIndices;"), 2)
        self.assertEqual(world.count("ResumeFirstPersonIndexedRegionSurfaces("), 2)

    def test_compiled_index_preserves_order_and_budget(self):
        compiler = shutil.which("clang++") or shutil.which("g++")
        if not compiler:
            self.skipTest("No C++ compiler")
        code = r"""
#include "FirstPersonRegionSurfaceIndex.h"
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>
using namespace OpenRCT2::Paint;
struct Surface { uint64_t gpuRegion; size_t id; };
struct Cache {
    std::vector<Surface> surfaces;
    FirstPersonRegionSurfaceIndex regionSurfaceIndices;
};
struct Budget { size_t n; bool take() { return n && (--n, true); } };
int main() {
    Cache c;
    for (size_t i = 0; i < 80000; ++i)
        c.surfaces.push_back({i % 7 == 0 ? 0 : 1 + i % 3, i});
    c.regionSurfaceIndices = IndexFirstPersonSurfacesByGpuRegion(c.surfaces);
    assert(c.regionSurfaceIndices.size() == 3);
    assert(!c.regionSurfaceIndices.contains(0));
    for (uint64_t region = 1; region <= 4; ++region) {
        size_t offset = 0;
        std::vector<size_t> observed;
        unsigned yields = 0;
        for (;;) {
            Budget budget{7};
            const bool done = ResumeFirstPersonIndexedRegionSurfaces(
                c, region, offset, budget,
                [&](const Surface& s) {
                    assert(s.gpuRegion == region);
                    observed.push_back(s.id);
                });
            if (done) break;
            ++yields;
        }
        if (region == 4) {
            assert(observed.empty() && offset == 0 && yields == 0);
            continue;
        }
        assert(yields > 100);
        size_t j = 0;
        for (const auto& s : c.surfaces)
            if (s.gpuRegion == region)
                assert(j < observed.size() && observed[j++] == s.id);
        assert(j == observed.size());
    }
    c.surfaces = {{3, 900}};
    c.regionSurfaceIndices = IndexFirstPersonSurfacesByGpuRegion(c.surfaces);
    size_t offset = 0;
    Budget budget{1};
    assert(ResumeFirstPersonIndexedRegionSurfaces(
        c, 3, offset, budget, [](const Surface& s) { assert(s.id == 900); }));
    assert(offset == 1);
}
"""
        with tempfile.TemporaryDirectory() as d:
            src, exe = Path(d) / "index.cpp", Path(d) / "index"
            src.write_text(code)
            result = subprocess.run(
                [compiler, "-std=c++20", "-O2", "-I", str(ROOT),
                 str(src), "-o", str(exe)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
