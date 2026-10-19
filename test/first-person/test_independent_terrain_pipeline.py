"""Regression guard for independent resident terrain and nonblocking transparency."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
HEADER = ROOT / "src/openrct2/paint/FirstPersonRenderer.h"
WORLD = ROOT / "src/openrct2/paint/FirstPersonRenderer.World.inc"
PAINT = ROOT / "src/openrct2/paint/FirstPersonRenderer.Paint.inc"
GL = ROOT / "src/openrct2-ui/drawing/engines/opengl/FirstPersonGLRenderer.cpp"


class TerrainPipelineTest(unittest.TestCase):
    def test_terrain_has_separate_gpu_lifetime(self):
        header = HEADER.read_text(encoding="utf-8")
        world = WORLD.read_text(encoding="utf-8")
        paint = PAINT.read_text(encoding="utf-8")
        self.assertIn("FirstPersonTerrainRegionKey(int32_t tileX, int32_t tileY)", header)
        self.assertIn("void SubmitVisibleTerrainRegions(", world)
        self.assertIn("MarkTerrainRegionDirtyForTile(tx, ty);", world)
        self.assertIn("cache.ground.gpuRegion = FirstPersonTerrainRegionKey(tx,ty);", world)
        self.assertIn("bool haveKnownTerrain = false;", world)
        self.assertIn("if (!terrain.verticalOpening)", world)
        self.assertIn("terrainIt->second.verticalOpening = verticalOpening;", paint)
        self.assertIn("MarkTerrainRegionDirtyForTile(\n                                            item.position.x / kCoordsXYStep,", paint)
        self.assertIn("region.surfaces = std::make_shared<const std::vector<FirstPersonSurface>>", world)
        builder = world[world.index("bool RebuildStaticRegionPacket("):
                        world.index("void SubmitVisibleStaticRegions(")]
        self.assertNotIn("_terrainCache.entries.find(tileKey)", builder)
        self.assertLess(paint.index("SubmitVisibleTerrainRegions(scene, worldFrustum);"),
                        paint.index("SubmitVisibleStaticRegions(scene, worldFrustum, frame);"))

    def test_balanced_native_capture_and_nonblocking_queries(self):
        paint = PAINT.read_text(encoding="utf-8")
        gl = GL.read_text(encoding="utf-8")
        self.assertIn("nativeCapturesRemaining = 2;", paint)
        self.assertIn("captureCursor += captureExamined;", paint)
        self.assertNotIn("objectPriorityPass", paint)
        self.assertNotIn("waitForPeelCoverageQuery", gl)
        self.assertIn("conservativeLayerLimit(tile)", gl)
        self.assertIn("GL_QUERY_RESULT_AVAILABLE, &ready", gl)

    def test_authoritative_structure_and_reusable_assets(self):
        world = WORLD.read_text(encoding="utf-8")
        paint = PAINT.read_text(encoding="utf-8")
        core = (ROOT / "src/openrct2/paint/FirstPersonRenderer.Core.inc").read_text(encoding="utf-8")
        small = (ROOT / "src/openrct2/paint/FirstPersonRenderer.SmallScenery.inc").read_text(encoding="utf-8")
        self.assertIn("void UpdateAuthoritativeLargeScenery(", world)
        self.assertIn("GetReconstructionGroup(tile, element)", world)
        self.assertIn("indexed.largeScenery.push_back(*group);", world)
        self.assertIn("QueueLargeSceneryGeometry(*large, *entry, group, frame);", world)
        self.assertIn("UpdateAuthoritativeLargeScenery(scene, frame);", paint)
        self.assertIn("_largeScenerySourceAssets", core)
        self.assertIn("std::shared_ptr<FirstPersonSpriteSnapshot> sprites;", core)
        self.assertIn("model, job->group, job->group.key, frame, *job->sprites, epoch", core)
        self.assertIn("_smallSceneryJobs.find(key)", small)
        self.assertIn("job->surfaces", small)

    def test_terrain_gpu_keys_do_not_alias_scenery(self):
        compiler = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++") or shutil.which("cl")
        if not compiler:
            self.skipTest("C++ compiler not available")
        header = HEADER.read_text(encoding="utf-8")
        start = header.index("    [[nodiscard]] constexpr uint64_t FirstPersonGpuRegionKey(")
        end = header.index("    struct FirstPersonSurface", start)
        helper = header[start:end]
        source = """
#include <cassert>
#include <cstdint>
""" + helper + """
int main() {
    for (int32_t tx = 0; tx < 128; tx += 7)
        for (int32_t ty = 0; ty < 128; ty += 9) {
            auto terrain = FirstPersonTerrainRegionKey(tx, ty);
            auto scenery = FirstPersonGpuRegionKey(tx, ty);
            assert(terrain != scenery);
            assert((terrain >> 63) == 1);
            assert((scenery >> 63) == 0);
            assert(FirstPersonTerrainRegionKey(tx, ty)
                == FirstPersonTerrainRegionKey(tx / 8 * 8, ty / 8 * 8));
        }
    assert(FirstPersonTerrainRegionKey(0,0) != FirstPersonTerrainRegionKey(8,0));
    assert(FirstPersonTerrainRegionKey(0,0) != FirstPersonTerrainRegionKey(0,8));
}
"""
        with tempfile.TemporaryDirectory() as folder:
            cpp = Path(folder) / "terrain_key.cpp"
            is_msvc = Path(compiler).stem.lower() == "cl"
            exe = Path(folder) / ("terrain_key.exe" if is_msvc else "terrain_key")
            cpp.write_text(source, encoding="utf-8")
            cmd = ([compiler, "/nologo", "/std:c++20", "/EHsc", str(cpp), "/Fe:" + str(exe)]
                   if is_msvc else
                   [compiler, "-std=c++20", "-O0", str(cpp), "-o", str(exe)])
            built = subprocess.run(cmd, cwd=folder, capture_output=True, text=True)
            self.assertEqual(built.returncode, 0, built.stderr)
            result = subprocess.run([str(exe)], cwd=folder, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
