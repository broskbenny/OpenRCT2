"""Only certified simple walls bypass per-placement native paint capture."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
MAP = ROOT / "src/openrct2/paint/FirstPersonRenderer.MapWall.inc"
LOCAL = ROOT / "src/openrct2/paint/FirstPersonRenderer.LocalScenery.inc"
WORLD = ROOT / "src/openrct2/paint/FirstPersonRenderer.World.inc"
NATIVE = ROOT / "src/openrct2/paint/tile_element/Paint.Wall.cpp"
HEADER = ROOT / "src/openrct2/paint/tile_element/Paint.Wall.h"


class MapPlainWallTest(unittest.TestCase):
    def test_simple_wall_eligibility_excludes_special_native_behaviour(self):
        m = MAP.read_text(encoding="utf-8")
        for forbidden in ("WallSceneryFlag::isDoor",
                          "WallSceneryFlag::hasGlass",
                          "WallSceneryFlag::isDoubleSided",
                          "WallSceneryFlag2::isTransparent",
                          "WallSceneryFlag2::isAnimated",
                          "kScrollingModeNone",
                          "wall.getSlope() != 0",
                          "wall.isGhost()",
                          "wall.isInvisible()",
                          "gTrackDesignSaveMode",
                          "VIEWPORT_FLAG_HIDE_SCENERY",
                          "VIEWPORT_FLAG_INVISIBLE_SCENERY",
                          "VIEWPORT_FLAG_CLIP_VIEW",
                          "VIEWPORT_FLAG_UNDERGROUND_INSIDE"):
            self.assertIn(forbidden, m)
        self.assertIn("return count == 1 && eligible;", m)
        self.assertIn("FirstPersonPaintSemanticRole::wall", m)
        self.assertIn("BuildFirstPersonWallPlane(", m)
        self.assertIn("GetFirstPersonWallBodyImageOffset(", m)
        self.assertIn("surface.coplanarOwner = true;", m)
        self.assertIn("ViewportInteractionItem::wall", m)
        self.assertIn("imageAnchor.x - sprite->xOffset", m)
        self.assertIn("imageAnchor.y - sprite->yOffset", m)
        self.assertIn("templateImage.WithIndex(", m)
        self.assertIn("templateImage.WithPrimary(", m)
        self.assertIn("templateImage.WithSecondary(", m)
        self.assertIn("templateImage.WithTertiary(", m)

    def test_native_and_map_ownership_handoff_is_transactional(self):
        local = LOCAL.read_text(encoding="utf-8")
        world = WORLD.read_text(encoding="utf-8")
        self.assertIn('#include "FirstPersonRenderer.MapWall.inc"', world)
        self.assertIn("AppendFirstPersonMapPlainWall(", local)
        self.assertIn("mappedWallTiles.insert(TerrainKey(tx, ty));", local)
        self.assertIn("packet.mappedWallTiles != mappedWallTiles", local)
        self.assertIn("MarkFirstPersonAuthoritativeRegionEdit(coherentKey);", local)
        self.assertIn("MarkStaticRegionPacketDirty(coherentKey);", local)
        self.assertEqual(
            local.count("packet.mappedWallTiles = std::move(mappedWallTiles);"), 2)
        self.assertIn("it->second.mappedWallTiles.contains(TerrainKey(", local)
        coherent = world.split("bool RebuildStaticRegionPacket(", 1)[1].split(
            "if (!staging.progress.tilesDone(kRegionTileCount))", 1)[0]
        self.assertEqual(coherent.count("mappedWallReady"), 4)
        self.assertEqual(coherent.count("FirstPersonPaintSemanticRole::wall"), 3)

    def test_compiled_shared_native_image_offsets_match_original_wall_table(self):
        compiler = shutil.which("g++") or shutil.which("clang++")
        if compiler is None:
            self.skipTest("No C++ compiler")
        native = NATIVE.read_text(encoding="utf-8")
        self.assertIn("GetFirstPersonWallBodyImageOffset(", native)
        cpp = r"""
#include "Paint.Wall.h"
#include <cassert>
#include <cstdint>
int main() {
    const uint32_t old[4][3] = {
        {1,5,3},{0,4,2},{1,3,5},{0,2,4}};
    for (uint8_t direction=0; direction<4; ++direction)
    for (uint8_t slope=0; slope<3; ++slope)
    for (int doubles=0; doubles<2; ++doubles)
    for (int glass=0; glass<2; ++glass) {
        uint32_t expected=old[direction][slope];
        if (doubles && direction==1)
            expected += glass?12:6;
        if (doubles && direction==2)
            expected += 6;
        assert(GetFirstPersonWallBodyImageOffset(
            direction,slope,doubles!=0,glass!=0)==expected);
    }
}
"""
        with tempfile.TemporaryDirectory() as directory:
            src = Path(directory) / "wall.cpp"
            exe = Path(directory) / "wall"
            src.write_text(cpp, encoding="utf-8")
            subprocess.run(
                [compiler, "-std=c++20", "-I", str(HEADER.parent),
                 str(src), "-o", str(exe)],
                check=True, capture_output=True, text=True)
            subprocess.run([str(exe)], check=True, capture_output=True, text=True)


if __name__ == "__main__":
    unittest.main()
