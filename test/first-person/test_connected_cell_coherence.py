"""Compile the production region key and occupancy resolver against cell seams."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2] / "src/openrct2/paint"


class ConnectedCellCoherenceTest(unittest.TestCase):
    def test_cells_and_diagonal_relationship_context_are_consistent(self):
        header = (ROOT / "FirstPersonRenderer.h").read_text()
        core = (ROOT / "FirstPersonRenderer.Core.inc").read_text()
        world = (ROOT / "FirstPersonRenderer.World.inc").read_text()
        self.assertIn("kFirstPersonConnectedRegionTiles = 8;", header)
        self.assertIn("minTileX / kFirstPersonConnectedRegionTiles", core)
        self.assertIn("regionX * kFirstPersonConnectedRegionTiles", core)
        self.assertIn("((tileX % kCell) + kCell) % kCell", core)
        self.assertIn("if (localX == kCell - 1)", core)
        self.assertIn("if (localY == kCell - 1)", core)
        self.assertIn("dirty(regionX + 1, regionY + 1)", core)
        self.assertIn("kRegionTileCount = kFPCell * kFPCell", world)
        self.assertIn("index / kFPCell", world)
        self.assertIn("index % kFPCell", world)
        self.assertIn("staging.relationshipTile < size_t(4 * kFPCell + 4)", world)
        self.assertIn("std::array<std::array<int32_t, 2>, 8>", world)
        self.assertIn("std::array<std::array<int32_t, 2>, 9>", world)
        self.assertIn("offset & 1 ? x1 : x0 - 1", world)
        self.assertIn("offset & 2 ? y1 : y0 - 1", world)
        self.assertIn("FirstPersonGpuRegionKey(nx * kFPCell, ny * kFPCell)", world)
        self.assertIn("progress.nextTile < kFPCell * kFPCell", world)
        self.assertIn("complete ? size_t(kFPCell * kFPCell)", world)

    def test_compiled_owner_partition_and_cross_cell_suppression(self):
        compiler = shutil.which("clang++") or shutil.which("g++")
        if not compiler:
            self.skipTest("C++ compiler unavailable")
        header = (ROOT / "FirstPersonRenderer.h").read_text()
        relations = (ROOT / "FirstPersonSurfaceRelations.h").read_text()
        keys = header[
            header.index("    inline constexpr int32_t kFirstPersonConnectedRegionTiles"):
            header.index("    struct FirstPersonSurface")]
        owner = relations[
            relations.index("    struct FirstPersonCanonicalSurfaceQuad"):
            relations.index("    [[nodiscard]] inline FirstPersonSurfaceRelationshipKind")]
        resolver = relations[
            relations.index("    struct FirstPersonInternalInterfaceResolution"):
            relations.index("} // namespace OpenRCT2::Paint",
                            relations.index("    struct FirstPersonInternalInterfaceResolution"))]
        cpp = r"""
#include <algorithm>
#include <array>
#include <bit>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <map>
#include <optional>
#include <span>
#include <unordered_map>
#include <utility>
#include <vector>
namespace OpenRCT2::Paint {
struct FirstPersonVec3 { float x{}, y{}, z{}; };
float FpDot(const FirstPersonVec3& a, const FirstPersonVec3& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}
struct FirstPersonVertex { FirstPersonVec3 world{}; };
struct FirstPersonSurface {
    std::array<FirstPersonVertex,6> triangles{};
    uint64_t reconstructionGroup=0, diagnosticSemanticGroup=0, sourceInstanceKey=0;
    bool cameraIndependent=true, reconstructedOccupancyBoundary=true, artworkCarrier=false;
    FirstPersonVec3 outwardNormal{};
};
""" + keys + owner + resolver + r"""
}
using namespace OpenRCT2::Paint;
static_assert(kFirstPersonConnectedRegionTiles == 8);
static_assert(FirstPersonGpuRegionKey(7,7) != FirstPersonGpuRegionKey(8,7));
static_assert(FirstPersonGpuRegionKey(7,7) != FirstPersonGpuRegionKey(8,8));
static_assert(FirstPersonGpuRegionKey(0,0) != 0);
FirstPersonSurface face(uint64_t owner, float normalX) {
    FirstPersonSurface result;
    result.reconstructionGroup=owner;
    result.outwardNormal={normalX,0,0};
    const FirstPersonVec3 a{256,256,0},b{256,288,0},
        c{256,288,32},d{256,256,32};
    result.triangles={
        FirstPersonVertex{a},FirstPersonVertex{b},FirstPersonVertex{c},
        FirstPersonVertex{a},FirstPersonVertex{c},FirstPersonVertex{d}};
    return result;
}
int main() {
    for (int x=0;x<256;++x)
    for (int y=0;y<256;++y) {
        const auto region=FirstPersonGpuRegionKey(x,y);
        assert(region==FirstPersonGpuRegionKey(x/8*8,y/8*8));
        assert(region!=FirstPersonTerrainRegionKey(x,y));
        assert(region!=FirstPersonLocalSceneryRegionKey(x,y));
        assert((region>>62)==0);
    }
    auto a=face(101,1),b=face(202,-1);
    for (auto p : std::array<std::array<int,4>,2>{
            std::array<int,4>{7,8,8,8}, std::array<int,4>{7,7,8,8}}) {
        assert(FirstPersonGpuRegionKey(p[0],p[1])!=FirstPersonGpuRegionKey(p[2],p[3]));
        std::vector<FirstPersonSurface> local{a};
        const std::array<FirstPersonSurface,1> context{b};
        auto r=ResolveFirstPersonInternalOccupancyInterfaces(local,context);
        assert(local.empty() && r.suppressedFaces==1 && r.contextMatches==1);
        local={b};
        const std::array<FirstPersonSurface,1> reverse{a};
        r=ResolveFirstPersonInternalOccupancyInterfaces(local,reverse);
        assert(local.empty() && r.suppressedFaces==1 && r.contextMatches==1);
    }
    std::vector<FirstPersonSurface> retained{a};
    const std::array<FirstPersonSurface,1> sameOwner{face(101,-1)};
    auto r=ResolveFirstPersonInternalOccupancyInterfaces(retained,sameOwner);
    assert(retained.size()==1 && r.suppressedFaces==0);
}
"""
        with tempfile.TemporaryDirectory() as td:
            source, executable = Path(td) / "cells.cpp", Path(td) / "cells"
            source.write_text(cpp, encoding="utf-8")
            build = subprocess.run(
                [compiler, "-std=c++20", "-O2", str(source), "-o", str(executable)],
                capture_output=True, text=True)
            self.assertEqual(build.returncode, 0, build.stderr)
            run = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)


if __name__ == "__main__":
    unittest.main()
