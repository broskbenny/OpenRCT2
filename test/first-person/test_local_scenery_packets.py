"""Incremental 8x8 scenery packets must never fragment connected physical meshes."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
WORLD = ROOT / "src/openrct2/paint/FirstPersonRenderer.World.inc"
CORE = ROOT / "src/openrct2/paint/FirstPersonRenderer.Core.inc"
HEADER = ROOT / "src/openrct2/paint/FirstPersonRenderer.h"
MAIN = ROOT / "src/openrct2/paint/FirstPersonRenderer.cpp"
PAINT = ROOT / "src/openrct2/paint/FirstPersonRenderer.Paint.inc"


class LocalSceneryPacketsTest(unittest.TestCase):
    def test_locality_invalidation_lifetime_and_publication(self):
        world = WORLD.read_text(encoding="utf-8")
        core = CORE.read_text(encoding="utf-8")
        main = MAIN.read_text(encoding="utf-8")
        paint = PAINT.read_text(encoding="utf-8")
        self.assertIn("void SubmitVisibleLocalSceneryRegions(", world)
        self.assertIn("void MarkLocalSceneryDirtyForTile(int32_t tx, int32_t ty)", world)
        self.assertIn("MarkLocalSceneryDirtyForTile(tileX, tileY);", core)
        self.assertIn("MarkLocalSceneryDirtyForTile(tx, ty);", main)
        self.assertIn("MarkTerrainRegionDirtyForTile(tx, ty);", main)
        self.assertIn("_localSceneryPackets.clear();", main)
        self.assertIn("_localSceneryEpoch = _sceneEpoch;", main)
        self.assertIn("FirstPersonFrameBudget budget(4, std::chrono::microseconds(1200));", world)
        self.assertIn("++packet.generation;", world)
        self.assertIn("ResidentStaticSurfaceFingerprint(surfaces)", world)
        self.assertIn("packet.fingerprint == fingerprint", world)
        self.assertIn("packet.textureDependencies == dependencies", world)
        self.assertIn("waitingForSourceRevalidation", world)
        self.assertIn("std::make_shared<const std::vector<FirstPersonSurface>>", world)
        terrain = paint.index("SubmitVisibleTerrainRegions(scene, worldFrustum);")
        local = paint.index("SubmitVisibleLocalSceneryRegions(scene, worldFrustum);")
        connected = paint.index("SubmitVisibleStaticRegions(scene, worldFrustum, frame);")
        self.assertLess(terrain, local)
        self.assertLess(local, connected)

    def test_only_source_tile_independent_surfaces_move_to_local(self):
        world = WORLD.read_text(encoding="utf-8")
        start = world.index("bool RebuildStaticRegionPacket(")
        end = world.index("void SubmitVisibleStaticRegions(", start)
        coherent = world[start:end]
        self.assertIn("if (tileSource && FirstPersonIndependentLocalScenery(surface))", coherent)
        # Only the four source-tile emission sites may set tileSource=true:
        # semantic, camera-independent and rotated source geometry.
        self.assertEqual(coherent.count("addSurface(surface, true);"), 3)
        self.assertEqual(coherent.count("addSurface(presented, true);"), 1)
        self.assertIn("addSurface(sourceSurface);", coherent)
        self.assertIn("addSurface(surfaces[staging.surfaceOffset++]);", coherent)
        self.assertIn("ResolveFirstPersonInternalOccupancyInterfaces(", coherent)
        independent = world[world.index("bool FirstPersonIndependentLocalScenery("):
                            world.index("struct FirstPersonLocalSceneryPacket")]
        for forbidden in (
            "surface.reconstructionGroup == 0",
            "!surface.reconstructedOccupancyBoundary",
            "!surface.coplanarOwner",
            "!surface.artworkCarrier",
            "!surface.exteriorOnly",
            "semanticRole == FirstPersonPaintSemanticRole::pathFixture",
            "semanticRole == FirstPersonPaintSemanticRole::smallSceneryBody",
        ):
            self.assertIn(forbidden, independent)
        self.assertNotIn("semanticRole == FirstPersonPaintSemanticRole::none", independent)

    def test_rotation_only_local_changes_do_not_rebuild_parent(self):
        world = WORLD.read_text(encoding="utf-8")
        paint = PAINT.read_text(encoding="utf-8")
        self.assertIn("bool FirstPersonRotationHasCoherentTileGeometry(", world)
        self.assertIn("&& surface.reconstructionGroup == 0", world)
        self.assertIn("if (FirstPersonRotationHasCoherentTileGeometry(cached))", paint)
        self.assertIn("MarkStaticRegionDirtyForTile(tx, ty);", paint)
        self.assertIn("MarkLocalSceneryDirtyForTile(tx, ty);", paint)
        self.assertLess(paint.index("if (FirstPersonRotationHasCoherentTileGeometry(cached))"),
                        paint.index("cached.selectedRotation = requested;"))

    def test_native_cpp_key_spaces_and_safe_eligibility(self):
        compiler = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++") or shutil.which("cl")
        if not compiler:
            self.skipTest("C++ compiler unavailable")
        header = HEADER.read_text(encoding="utf-8")
        world = WORLD.read_text(encoding="utf-8")
        keys = header[header.index("    [[nodiscard]] constexpr uint64_t FirstPersonGpuRegionKey("):
                      header.index("    struct FirstPersonSurface", header.index("FirstPersonGpuRegionKey("))]
        predicate = world[world.index("        [[nodiscard]] bool FirstPersonIndependentLocalScenery("):
                          world.index("        [[nodiscard]] bool FirstPersonResidentContainsCoherentGeometry(")]
        source = """
#include <cstdint>
#include <cassert>
enum class FirstPersonPaintSemanticRole : uint8_t {
    none=0, support=1, pathDeck=2, railing=3, pathFixture=4,
    stationFloor=5, stationFence=6, stationCover=7, wall=8,
    terrainEdge=9, structureBody=10, structureFloor=11, structureRoof=12,
    pier=13, towerSection=14, movingMachinery=15, seat=16, sign=17,
    smallSceneryBody=18
};
struct FirstPersonSurface {
    uint8_t diagnosticSemanticRole=0;
    bool resident=true;
    uint64_t reconstructionGroup=0;
    bool reconstructedOccupancyBoundary=false;
    bool coplanarOwner=false;
    bool artworkCarrier=false;
    bool exteriorOnly=false;
};
bool IsResidentStaticSurface(const FirstPersonSurface& s) { return s.resident; }
""" + keys + predicate + """
int main() {
    for (int32_t x=0; x<256; x+=3)
        for (int32_t y=0; y<256; y+=5) {
            auto local=FirstPersonLocalSceneryRegionKey(x,y);
            auto terrain=FirstPersonTerrainRegionKey(x,y);
            auto coherent=FirstPersonGpuRegionKey(x,y);
            assert(local!=terrain && local!=coherent && terrain!=coherent);
            assert((local >> 62) == 1u && (terrain >> 62) == 2u);
            assert((coherent >> 62) == 0u);
            assert(local==FirstPersonLocalSceneryRegionKey(x/8*8,y/8*8));
        }
    assert(FirstPersonLocalSceneryRegionKey(7,7)!=FirstPersonLocalSceneryRegionKey(8,7));
    FirstPersonSurface s;
    assert(!FirstPersonIndependentLocalScenery(s));
    s.diagnosticSemanticRole=uint8_t(FirstPersonPaintSemanticRole::smallSceneryBody);
    assert(FirstPersonIndependentLocalScenery(s));
    s.reconstructionGroup=123; assert(!FirstPersonIndependentLocalScenery(s)); s.reconstructionGroup=0;
    s.reconstructedOccupancyBoundary=true; assert(!FirstPersonIndependentLocalScenery(s)); s.reconstructedOccupancyBoundary=false;
    s.coplanarOwner=true; assert(!FirstPersonIndependentLocalScenery(s)); s.coplanarOwner=false;
    s.artworkCarrier=true; assert(!FirstPersonIndependentLocalScenery(s)); s.artworkCarrier=false;
    s.exteriorOnly=true; assert(!FirstPersonIndependentLocalScenery(s)); s.exteriorOnly=false;
    s.diagnosticSemanticRole=uint8_t(FirstPersonPaintSemanticRole::wall);
    assert(!FirstPersonIndependentLocalScenery(s));
    s.diagnosticSemanticRole=uint8_t(FirstPersonPaintSemanticRole::pathDeck);
    assert(!FirstPersonIndependentLocalScenery(s));
    s.diagnosticSemanticRole=uint8_t(FirstPersonPaintSemanticRole::structureBody);
    assert(!FirstPersonIndependentLocalScenery(s));
    s.diagnosticSemanticRole=uint8_t(FirstPersonPaintSemanticRole::pathFixture);
    assert(FirstPersonIndependentLocalScenery(s));
    s.diagnosticSemanticRole=uint8_t(FirstPersonPaintSemanticRole::smallSceneryBody);
    assert(FirstPersonIndependentLocalScenery(s));
    s.resident=false; assert(!FirstPersonIndependentLocalScenery(s));
}
"""
        with tempfile.TemporaryDirectory() as folder:
            cpp = Path(folder) / "local_scenery.cpp"
            cpp.write_text(source, encoding="utf-8")
            is_msvc = Path(compiler).stem.lower() == "cl"
            exe = Path(folder) / ("local_scenery.exe" if is_msvc else "local_scenery")
            cmd = ([compiler, "/nologo", "/std:c++20", "/EHsc", str(cpp), "/Fe:" + str(exe)]
                   if is_msvc else [compiler, "-std=c++20", "-O0", str(cpp), "-o", str(exe)])
            built = subprocess.run(cmd, cwd=folder, capture_output=True, text=True)
            self.assertEqual(built.returncode, 0, built.stdout + built.stderr)
            executed = subprocess.run([str(exe)], cwd=folder, capture_output=True, text=True)
            self.assertEqual(executed.returncode, 0, executed.stdout + executed.stderr)


if __name__ == "__main__":
    unittest.main()
