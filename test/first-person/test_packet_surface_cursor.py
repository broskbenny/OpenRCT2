"""Production surface-level budget and resumable static packet tile traversal."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
CURSOR = ROOT / "src/openrct2/paint/FirstPersonPacketSurfaceCursor.h"
WORLD = ROOT / "src/openrct2/paint/FirstPersonRenderer.World.inc"
RENDERER = ROOT / "src/openrct2/paint/FirstPersonRenderer.cpp"


class PacketSurfaceCursorTest(unittest.TestCase):
    def test_renderer_uses_same_cursor_for_semantic_independent_and_native(self):
        world = WORLD.read_text(encoding="utf-8")
        renderer = RENDERER.read_text(encoding="utf-8")
        body = world[world.index("[[nodiscard]] bool RebuildStaticRegionPacket"):
                     world.index("void SubmitVisibleStaticRegions")]
        self.assertIn("FirstPersonPacketTileSurfaceCursor tileCursor{}", world)
        self.assertIn('#include "FirstPersonPacketSurfaceCursor.h"', renderer)
        self.assertIn("ResumeFirstPersonPacketSurfaces(", body)
        self.assertIn("staging.tileCursor.nextTile();", body)
        self.assertIn("const size_t index = staging.progress.nextTile;", body)
        self.assertIn("const uint8_t phase = staging.tileCursor.phase;", body)
        self.assertIn("phase == 0", body)
        self.assertIn("phase == 1", body)
        self.assertIn("phase - 2", body)
        self.assertLess(body.index("ResumeFirstPersonPacketSurfaces("),
                        body.index("if (!staging.progress.tilesDone(kRegionTileCount))"))

    def test_compiled_cursor_never_replays_or_skips_surfaces_under_budget(self):
        compiler = shutil.which("clang++") or shutil.which("g++")
        if not compiler:
            self.skipTest("C++ compiler unavailable")
        code = r"""
#include "FirstPersonPacketSurfaceCursor.h"
#include <array>
#include <cassert>
#include <cstddef>
#include <numeric>
#include <vector>
using namespace OpenRCT2::Paint;
struct Budget {
    size_t left;
    bool take() { return left && (--left, true); }
};
int main() {
    // A pathological tile with 30,000 faces needs many incremental passes.
    // Every surface is visited exactly once, in canonical phase order.
    std::array<std::vector<int>, 6> faces;
    for (int phase=0;phase<6;++phase) {
        faces[phase].resize(phase % 2 ? 301 : 10001);
        for (size_t i=0;i<faces[phase].size();++i)
            faces[phase][i]=phase*100000 + int(i);
    }
    std::vector<int> seen;
    FirstPersonPacketTileSurfaceCursor cursor;
    size_t retries=0;
    while (cursor.phase < faces.size()) {
        Budget budget{7};
        if (!ResumeFirstPersonPacketSurfaces(
            faces[cursor.phase], cursor, budget, [&](int v){seen.push_back(v);}))
            ++retries;
    }
    assert(retries>1000);
    size_t index=0;
    for (const auto& phase:faces)
        for (int value:phase) assert(seen[index++]==value);
    assert(index==seen.size());
    cursor.nextTile();
    assert(cursor.phase==0 && cursor.surface==0);
    // Empty and budget-exhausted work preserves cursor invariants.
    std::vector<int> empty;
    Budget none{0};
    assert(ResumeFirstPersonPacketSurfaces(empty,cursor,none,[](int){}));
    assert(cursor.phase==1 && cursor.surface==0);
    std::vector<int> one{42};
    assert(!ResumeFirstPersonPacketSurfaces(one,cursor,none,[](int){assert(false);}));
    assert(cursor.phase==1 && cursor.surface==0);
}
"""
        with tempfile.TemporaryDirectory() as td:
            source, exe = Path(td)/"cursor.cpp", Path(td)/"cursor"
            source.write_text(code, encoding="utf-8")
            cmd = [compiler, "-std=c++20", "-O2", "-I", str(CURSOR.parent),
                   str(source), "-o", str(exe)]
            result = subprocess.run(cmd, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            result = subprocess.run([str(exe)], text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
