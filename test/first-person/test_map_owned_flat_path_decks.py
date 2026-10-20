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
        self.assertIn("GetPathSurfaceImageOffset(path, sourceRotation)", path)
        self.assertIn("GetTileElementPaintSpritePosition(", path)
        self.assertIn("Translate3DTo2DWithZ(", path)
        self.assertIn("BuildFirstPersonPathFootprint(", path)
        self.assertIn("EmitQuad(surface, vertices);", path)
        self.assertIn("ViewportInteractionItem::footpath", path)
        self.assertIn("surface.diagnosticSourceTile = tile;", path)
        self.assertIn("surface.coplanarOwner = true;", path)
        self.assertNotIn("0.01f", path)
        self.assertIn("path.isSloped() || path.isWide()", path)
        self.assertIn("ground->getSlope() == kTileSlopeFlat", path)
        self.assertIn("ground->getBaseZ() == path.getBaseZ()", path)
        self.assertIn("ground->getWaterHeight() <= path.getBaseZ()", path)
        self.assertIn("return paths == 1 && eligible;", path)

    def test_path_packets_publish_and_suppress_only_ready_duplicates(self):
        local = LOCAL.read_text(encoding="utf-8")
        world = WORLD.read_text(encoding="utf-8")
        self.assertIn("AppendFirstPersonMapFlatPathDeck(", local)
        self.assertIn("add(surface, owningRegion, true);", local)
        self.assertIn("if (it == _localSceneryPackets.end() || it->second.dirty", local)
        self.assertIn("FirstPersonLocalFlatPathReady(", world)
        gather = world.split("bool RebuildStaticRegionPacket(", 1)[1].split(
            "if (!staging.progress.tilesDone(kRegionTileCount))", 1)[0]
        self.assertEqual(gather.count("if (mappedPathReady"), 3)
        self.assertEqual(gather.count("FirstPersonPaintSemanticRole::pathDeck"), 3)
        self.assertIn("FirstPersonPaintSemanticRole::pathDeck", local)
        self.assertIn('#include "FirstPersonRenderer.MapPath.inc"', world)
        self.assertLess(world.index('#include "FirstPersonRenderer.MapPath.inc"'),
                        world.index('#include "FirstPersonRenderer.LocalScenery.inc"'))

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
