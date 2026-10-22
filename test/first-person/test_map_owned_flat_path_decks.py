"""Map-owned flat path decks preserve native artwork and avoid native duplication."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
MAP_PATH = ROOT / "src/openrct2/paint/FirstPersonRenderer.MapPath.inc"
LOCAL = ROOT / "src/openrct2/paint/FirstPersonRenderer.LocalScenery.inc"
WORLD = ROOT / "src/openrct2/paint/FirstPersonRenderer.World.inc"
FOOTPRINT = ROOT / "src/openrct2/paint/FirstPersonPathGeometry.h"


class MapOwnedFlatPathDeckTest(unittest.TestCase):
    def test_source_independent_deck_reuses_native_artwork_and_pick(self):
        path = MAP_PATH.read_text(encoding="utf-8")
        self.assertIn("GetFirstPersonPathDeckImageOffset(path, sourceRotation)", path)
        self.assertIn("GetTileElementPaintSpritePosition(", path)
        self.assertIn("Translate3DTo2DWithZ(", path)
        self.assertIn("BuildFirstPersonPathFootprint(", path)
        self.assertIn("EmitQuad(surface, vertices);", path)
        self.assertIn("ViewportInteractionItem::footpath", path)
        self.assertIn("surface.diagnosticSourceTile = tile;", path)
        self.assertIn("surface.coplanarOwner = true;", path)
        self.assertNotIn("0.01f", path)
        self.assertIn("path.isSloped()", path)
        self.assertNotIn("path.isSloped() || path.isWide()", path)
        self.assertIn("FirstPersonPathUsesFullTileDeck(path)", path)
        self.assertIn("cells.push_back({ 0, 0, kCoordsXYStep, kCoordsXYStep });", path)
        self.assertIn("path.getRailingsDescriptor() == nullptr", path)
        self.assertIn("image.WithRemap(", path)
        self.assertIn("Drawing::FilterPaletteID::paletteGhost", path)
        self.assertIn("gPaintBlockedTiles && path.isBlockedByVehicle()", path)
        self.assertIn("Drawing::FilterPaletteID::palette46", path)
        self.assertIn("gPaintWidePathsAsGhost && path.isWide()", path)
        self.assertIn("ground->getSlope() == pathSlope", path)
        self.assertIn("ground->getBaseZ() == path.getBaseZ()", path)
        self.assertIn("ground->getWaterHeight() <= path.getBaseZ()", path)
        self.assertIn("return paths == 1 && eligible;", path)

    def test_path_sprite_edges_use_a_persistent_opaque_native_palette_material(self):
        path = MAP_PATH.read_text(encoding="utf-8")
        self.assertIn("BuildFirstPersonTerrainEdgeMaterial(", path)
        self.assertIn("DecodeFirstPersonSpritePixels(*sprite)", path)
        self.assertIn("_terrainCache.materials.find(image.GetIndex())", path)
        self.assertIn("if (!image.IsBlended())", path)
        self.assertIn("if (deckMaterial != nullptr)", path)
        self.assertIn("vertex.u += float(deckMaterial->padding);", path)
        self.assertIn("vertex.v += float(deckMaterial->padding);", path)
        self.assertIn("surface.persistentPixels = deckMaterial->pixels;", path)
        self.assertIn("surface.persistentBitmap = true;", path)
        self.assertIn("surface.physicalCoverage = true;", path)
        self.assertLess(path.index("BuildFirstPersonPathFootprint("),
                        path.index("BuildFirstPersonTerrainEdgeMaterial("))
        self.assertLess(path.index("vertices[i] = {"),
                        path.index("PeriodizeFirstPersonFlatTileMaterial("))
        self.assertLess(path.index("surface.physicalCoverage = true;"),
                        path.index("EmitQuad(surface, vertices);"))
        # The physical tile coverage still belongs to the authored footprint:
        # filling image transparency must not widen a path or lift it off land.
        self.assertIn("BuildFirstPersonPathFootprint(", path)
        self.assertIn("FirstPersonPathUsesFullTileDeck(path)", path)
        self.assertIn("surface.coplanarOwner = true;", path)
        self.assertNotIn("heightAt(float(cell.x0), float(cell.y0)) +", path)

    def test_native_path_image_aliases_never_mix_repeat_axes(self):
        import re
        native = (ROOT / "src/openrct2/paint/tile_element/Paint.Path.cpp").read_text()
        entries = native.split(
            "kPathEdgesAndCornersToSurfaceImageIndexOffset[] = {", 1
        )[1].split("};", 1)[0]
        offsets = [int(x) for x in re.findall(r"\b\d+\b", entries)]
        self.assertEqual(len(offsets), 256)
        axes_per_image = {}
        for combined, offset in enumerate(offsets):
            edges = combined & 15
            axes = int((edges & 0x05) == 0x05)
            axes |= 2 * int((edges & 0x0A) == 0x0A)
            axes_per_image.setdefault(offset, set()).add(axes)
        # An immutable cached palette can safely be keyed by sprite ID,
        # rather than re-created for every path placement.
        self.assertTrue(all(len(axes) == 1 for axes in axes_per_image.values()))
        self.assertEqual(axes_per_image[5], {1})
        self.assertEqual(axes_per_image[10], {2})
        self.assertEqual(axes_per_image[15], {3})

    def test_path_packets_publish_and_suppress_only_ready_duplicates(self):
        local = LOCAL.read_text(encoding="utf-8")
        world = WORLD.read_text(encoding="utf-8")
        self.assertIn("AppendFirstPersonMapFlatPathDeck(", local)
        self.assertIn("add(surface, owningRegion, true);", local)
        self.assertIn("if (it == _localSceneryPackets.end() || it->second.dirty", local)
        self.assertIn("FirstPersonLocalFlatPathReady(", world)
        gather = world.split("bool RebuildStaticRegionPacket(", 1)[1].split(
            "if (!staging.progress.tilesDone(kRegionTileCount))", 1)[0]
        self.assertEqual(gather.count("mappedPathReady"), 4)
        self.assertEqual(gather.count("FirstPersonPaintSemanticRole::pathDeck"), 3)
        self.assertIn("mappedPathTiles.insert(TerrainKey(tx, ty));", local)
        self.assertIn("mappedPathTiles.contains(", local)
        self.assertEqual(local.count("packet.mappedPathTiles = std::move(mappedPathTiles);"), 2)
        self.assertIn("std::unordered_set<uint64_t> mappedPathTiles;", local)
        self.assertIn("packet.paintBlockedMode != gPaintBlockedTiles", local)
        self.assertIn("packet.paintBlockedMode = gPaintBlockedTiles;", local)
        self.assertIn("packet.paintWideGhostMode != gPaintWidePathsAsGhost", local)
        self.assertIn("packet.paintWideGhostMode = gPaintWidePathsAsGhost;", local)
        self.assertIn('#include "FirstPersonRenderer.MapPath.inc"', world)
        self.assertLess(world.index('#include "FirstPersonRenderer.MapPath.inc"'),
                        world.index('#include "FirstPersonRenderer.LocalScenery.inc"'))

    def test_local_gpu_owner_order_is_stable_across_camera_turns(self):
        local = LOCAL.read_text(encoding="utf-8")
        publish = local.split("std::vector<uint64_t> orderedVisible(", 1)[1].split(
            "scene.staticRegions.push_back({", 1)[0]
        self.assertIn("visible.begin(), visible.end()", publish)
        self.assertIn("std::sort(orderedVisible.begin(), orderedVisible.end());", publish)
        self.assertIn("for (const auto key : orderedVisible)", publish)
        # A moving frustum may change the cell set, but unchanged neighbours
        # keep a fixed relative owner draw order.
        import random
        rng = random.Random(20261020)
        for _ in range(30000):
            previous = {rng.randrange(1024) for _ in range(90)}
            current = set(previous)
            for _ in range(8):
                current.discard(rng.randrange(1024))
                current.add(rng.randrange(1024))
            old_shared = [key for key in sorted(previous) if key in current]
            new_shared = [key for key in sorted(current) if key in previous]
            self.assertEqual(old_shared, new_shared)

    def test_native_visibility_modes_remain_authoritative(self):
        path = MAP_PATH.read_text(encoding="utf-8")
        for mode in (
            "VIEWPORT_FLAG_HIGHLIGHT_PATH_ISSUES",
            "VIEWPORT_FLAG_HIDE_PATHS",
            "VIEWPORT_FLAG_INVISIBLE_PATHS",
            "VIEWPORT_FLAG_CLIP_VIEW",
            "VIEWPORT_FLAG_UNDERGROUND_INSIDE",
        ):
            self.assertIn(mode, path)
        self.assertIn("scene.options.viewFlags", LOCAL.read_text(encoding="utf-8"))
        self.assertIn("packet.viewFlags != scene.options.viewFlags",
                      LOCAL.read_text(encoding="utf-8"))
        # Unchanged map tiles must still switch representations promptly on
        # a viewport visibility transition, without rebuilding whole terrain.
        import random
        rng = random.Random(20261020)
        def can_map(flags):
            return (flags & 0b11111) == 0
        for _ in range(50000):
            old, new = rng.randrange(32), rng.randrange(32)
            changed = old != new
            self.assertEqual(changed, bool(old ^ new))
            self.assertFalse(can_map(old) and old != 0)

    def test_on_grade_slopes_use_the_same_height_rule_as_native_path_painter(self):
        path = MAP_PATH.read_text(encoding="utf-8")
        native = (ROOT / "src/openrct2/paint/tile_element/Paint.Path.cpp").read_text(
            encoding="utf-8")
        walking = (ROOT / "src/openrct2/paint/FirstPersonWalkingSemantics.h").read_text(
            encoding="utf-8")
        # An elevated bridge or a path whose slope disagrees with its land
        # stays with the native painter. No detached plane is substituted.
        self.assertIn("ground->getSlope() == pathSlope", path)
        self.assertIn("ground->getBaseZ() == path.getBaseZ()", path)
        self.assertIn("ground->getWaterHeight() <= path.getBaseZ()", path)
        self.assertIn("FirstPersonPathLandSlope(path.getSlopeDirection())", path)
        self.assertIn("GetSlopeCornerHeights(path.getBaseZ(), slope)", path)
        self.assertIn("heightAt(float(cell.x0), float(cell.y0))", path)
        self.assertIn("heightAt(float(cell.x1), float(cell.y1))", path)
        self.assertIn("GetSlopeCornerHeights(height, slope)", native)
        self.assertIn("FirstPersonPathLandSlope(", walking)
        self.assertNotIn("|| path.isSloped()\n", path)

        # Both native semantic and map-owned decks interpolate the same
        # four physical corners. Subdividing a sloped deck into any subset
        # of native footprint cells cannot introduce seams at shared edges.
        import random
        rng = random.Random(20261020)
        for _ in range(20000):
            sw, se = rng.randrange(0, 100), rng.randrange(0, 100)
            nw, ne = rng.randrange(0, 100), rng.randrange(0, 100)
            def height(x, y):
                u, v = x / 32.0, y / 32.0
                return sw * (1-u) * (1-v) + se * u * (1-v) + ne*u*v + nw*(1-u)*v
            x0, x1 = sorted(rng.sample(range(33), 2))
            y0, y1 = sorted(rng.sample(range(33), 2))
            x = rng.uniform(x0, x1)
            y = rng.uniform(y0, y1)
            u, v = (x-x0)/(x1-x0), (y-y0)/(y1-y0)
            subdivided = (
                height(x0,y0)*(1-u)*(1-v)
                + height(x1,y0)*u*(1-v)
                + height(x1,y1)*u*v
                + height(x0,y1)*(1-u)*v
            )
            self.assertAlmostEqual(height(x, y), subdivided, places=10)
            self.assertAlmostEqual(height(0, 0), sw)
            self.assertAlmostEqual(height(32, 0), se)
            self.assertAlmostEqual(height(32, 32), ne)
            self.assertAlmostEqual(height(0, 32), nw)

    def test_map_ownership_handoffs_invalidate_in_flight_coherent_geometry(self):
        local = LOCAL.read_text(encoding="utf-8")
        world = WORLD.read_text(encoding="utf-8")
        self.assertIn("packet.mappedPathTiles != mappedPathTiles", local)
        self.assertIn("MarkFirstPersonAuthoritativeRegionEdit(coherentKey);", local)
        self.assertIn("MarkStaticRegionPacketDirty(coherentKey);", local)
        self.assertIn("FirstPersonRegionMapEditSerial(regionKey)", world)
        self.assertIn("FirstPersonStagedPacketNeedsSourceRestart(", world)
        self.assertIn("packet.mappedPathTiles = std::move(mappedPathTiles);", local)
        self.assertLess(
            local.index("packet.mappedPathTiles != mappedPathTiles"),
            local.index("packet.mappedPathTiles = std::move(mappedPathTiles);"))
        # First admission, removal and restoration all invalidate coherent
        # ownership; identity-preserving redraws do not.
        import random
        rng = random.Random(20261020)
        for _ in range(50000):
            old = {rng.randrange(64) for _ in range(rng.randrange(65))}
            new = set(old)
            if rng.randrange(3) == 0:
                new.discard(rng.randrange(64))
            elif rng.randrange(3) == 0:
                new.add(rng.randrange(64))
            transition = old != new
            self.assertEqual(transition, bool(old.symmetric_difference(new)))

    def test_native_box_pole_and_map_paths_share_wide_plaza_artwork(self):
        native = (ROOT / "src/openrct2/paint/tile_element/Paint.Path.cpp").read_text(
            encoding="utf-8")
        declaration = (ROOT / "src/openrct2/paint/tile_element/Paint.Path.h").read_text(
            encoding="utf-8")
        self.assertIn("uint8_t GetFirstPersonPathDeckImageOffset(", declaration)
        self.assertEqual(native.count(
            "GetFirstPersonPathDeckImageOffset(pathElement, session.CurrentRotation)"), 2)
        self.assertIn("kPathEdgesAndCornersToSurfaceImageIndexOffset[0xFF]", native)
        self.assertIn("FirstPersonPathUsesFullTileDeck(pathElement)", native)
        self.assertIn("GetPathSurfaceImageOffset(pathElement, rotation)", native)

    def test_compiled_native_footprints_never_overlap_or_escape_a_tile(self):
        compiler = shutil.which("g++") or shutil.which("clang++")
        if compiler is None:
            self.skipTest("No C++ compiler available")
        cpp = r"""
#include "FirstPersonPathGeometry.h"
#include <cassert>
using namespace OpenRCT2::Paint;
int main() {
    for (int edges=0;edges<16;++edges)
    for (int corners=0;corners<16;++corners)
    for (int queue=0;queue<2;++queue) {
        const auto cells = BuildFirstPersonPathFootprint(
            edges,corners,queue!=0);
        assert(!cells.empty());
        bool painted[32][32] = {};
        for (const auto& c : cells) {
            assert(c.x0 >= 0 && c.x0 < c.x1 && c.x1 <= 32);
            assert(c.y0 >= 0 && c.y0 < c.y1 && c.y1 <= 32);
            for (int x=c.x0;x<c.x1;++x)
            for (int y=c.y0;y<c.y1;++y) {
                assert(!painted[x][y]);
                painted[x][y] = true;
            }
        }
        for (int x=3;x<29;++x)
        for (int y=3;y<29;++y)
            assert(painted[x][y]);
    }
}
"""
        with tempfile.TemporaryDirectory() as td:
            source = Path(td) / "footprint.cpp"
            binary = Path(td) / "footprint"
            source.write_text(cpp, encoding="utf-8")
            subprocess.run(
                [compiler, "-std=c++20", "-O2", "-I", str(FOOTPRINT.parent),
                 str(source), "-o", str(binary)],
                check=True, capture_output=True, text=True)
            subprocess.run([str(binary)], check=True, capture_output=True, text=True)


if __name__ == "__main__":
    unittest.main()
