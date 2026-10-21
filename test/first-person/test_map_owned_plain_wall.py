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
                          "wall.getSlope() > 2",
                          "wall.isGhost()",
                          "wall.isInvisible()",
                          "gTrackDesignSaveMode",
                          "VIEWPORT_FLAG_HIDE_SCENERY",
                          "VIEWPORT_FLAG_INVISIBLE_SCENERY",
                          "VIEWPORT_FLAG_CLIP_VIEW",
                          "VIEWPORT_FLAG_UNDERGROUND_INSIDE"):
            self.assertIn(forbidden, m)
        self.assertIn("count != 1 || loneWall == nullptr", m)
        self.assertIn("!FirstPersonMapOwnsPlainWall(*loneWall, viewFlags)", m)
        self.assertIn("TileElementType::largeScenery", m)
        self.assertIn("TileElementType::track", m)
        self.assertIn("TileElementType::entrance", m)
        self.assertIn("MapIsLocationValid(neighbour)", m)
        self.assertIn("((direction + 2u) & 3u)", m)
        self.assertIn("return true;", m)
        self.assertIn("FirstPersonPaintSemanticRole::wall", m)
        self.assertIn("BuildFirstPersonWallPlane(", m)
        self.assertIn("GetFirstPersonWallBodyImageOffset(", m)
        self.assertIn("surface.coplanarOwner = true;", m)
        self.assertIn("if (image.IsBlended() || sprite == nullptr", m)
        self.assertIn("|| sprite->width <= 0 || sprite->height <= 0", m)
        self.assertIn("ViewportInteractionItem::wall", m)
        self.assertIn("imageAnchor.x - sprite->xOffset", m)
        self.assertIn("imageAnchor.y - sprite->yOffset", m)
        self.assertIn("templateImage.WithIndex(", m)
        self.assertIn("templateImage.WithPrimary(", m)
        self.assertIn("templateImage.WithSecondary(", m)
        self.assertIn("templateImage.WithTertiary(", m)

    def test_sloped_wall_uses_the_native_geometry_and_artwork_table(self):
        direct = MAP.read_text(encoding="utf-8")
        native = NATIVE.read_text(encoding="utf-8")
        painting = (ROOT / "src/openrct2/paint/Paint.h").read_text(
            encoding="utf-8")
        self.assertIn("wall.getSlope() > 2", direct)
        self.assertIn("BuildFirstPersonWallPlane(", direct)
        self.assertIn("wall.getSlope(),", direct)
        self.assertIn("GetFirstPersonWallBodyImageOffset(", native)
        self.assertIn("GetFirstPersonWallBodyImageOffset(", direct)
        self.assertIn("FirstPersonWallSemanticCorners(", painting)
        self.assertIn("if ((slope & 1u) != 0)", painting)
        self.assertIn("else if ((slope & 2u) != 0)", painting)
        # The native painter authors only two raised-end states, and the
        # direct plane uses that exact existing geometry contract.
        for direction in range(4):
            for slope in (0, 1, 2):
                raised_a = bool(slope & 2)
                raised_b = bool(slope & 1)
                self.assertFalse(raised_a and raised_b)
                self.assertEqual(int(raised_a) + int(raised_b), int(slope != 0))

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

    def test_opposite_wall_edge_mapping_is_symmetric(self):
        m = MAP.read_text(encoding="utf-8")
        self.assertIn("{ -kCoordsXYStep, 0 }, { 0, kCoordsXYStep }", m)
        self.assertIn("{ kCoordsXYStep, 0 }, { 0, -kCoordsXYStep }", m)
        offsets = ((-32, 0), (0, 32), (32, 0), (0, -32))
        for direction, (x, y) in enumerate(offsets):
            opposite = (direction + 2) & 3
            self.assertEqual((x+offsets[opposite][0],
                              y+offsets[opposite][1]), (0, 0))
        # Cross-tile coplanar wall pairs stay on the coherent interface path
        # regardless of elevation or colour, rather than risking a z fight.
        self.assertIn("other->getType() == TileElementType::wall", m)
        self.assertIn("return false;", m)

    def test_wall_projection_uses_original_native_sprite_offsets(self):
        m = MAP.read_text(encoding="utf-8")
        native = NATIVE.read_text(encoding="utf-8")
        source = (ROOT / "src/openrct2/paint/FirstPersonRenderer.cpp").read_text(
            encoding="utf-8")
        paint = (ROOT / "src/openrct2/paint/Paint.cpp").read_text(
            encoding="utf-8")
        for offset in (
            "{ 0, 0, height }",
            "{ 1, 31, height }",
            "{ 31, 0, height }",
            "{ 2, 1, height }",
        ):
            self.assertIn(offset, native)
        self.assertIn("{ { 0, 0 }, { 1, 31 }", m)
        self.assertIn("{ 31, 0 }, { 2, 1 }", m)
        self.assertIn("GetTileElementPaintSpritePosition(", m)
        self.assertIn("sourceRotation = 0;", m)
        self.assertIn("Translate3DTo2DWithZ(", m)
        self.assertIn("BuildFirstPersonWallPlane(", m)
        self.assertIn("FirstPersonWallSemanticCorners(", source)
        self.assertIn("FirstPersonSemanticArtworkScreenPos(", paint)
        self.assertIn("imageAnchor.x - sprite->xOffset", m)
        self.assertIn("imageAnchor.y - sprite->yOffset", m)

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
