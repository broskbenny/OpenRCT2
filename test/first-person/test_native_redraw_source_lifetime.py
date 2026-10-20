"""Unchanged native redraws must not invalidate authoritative static 3D geometry."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
STREAM = ROOT / "src/openrct2/paint/FirstPersonStreaming.h"
RENDERER = ROOT / "src/openrct2/paint/FirstPersonRenderer.cpp"


class NativeRedrawSourceLifetimeTest(unittest.TestCase):
    def test_production_redraw_uses_explicit_change_and_dynamic_policy(self):
        header = STREAM.read_text(encoding="utf-8")
        renderer = RENDERER.read_text(encoding="utf-8")
        code = renderer.split("void InvalidateFirstPersonSceneRegion(", 1)[1].split(
            "void InvalidateFirstPersonSceneTile(", 1)[0]
        self.assertIn("FirstPersonNativeRedrawNeedsRevalidation(", header)
        self.assertIn("FirstPersonNativeRedrawNeedsRevalidation(", code)
        self.assertIn("const bool authoritativeChange = changedTiles.contains(key);", code)
        self.assertIn("cached->second.hasCameraDependentFallback", code)
        self.assertIn("cached->second.animated", code)
        self.assertIn("const bool inspectorActive =", code)
        self.assertEqual(code.count("TileInspector::GetSelectedElement()"), 1)
        self.assertIn("cached->second.dirty = true;", code)
        self.assertIn("if (authoritativeChange)\n"
                      "                    cached->second.visibilityDirty = true;", code)
        # Retain the native capture scheduler's periodic semantic probe.
        paint = (ROOT / "src/openrct2/paint/FirstPersonRenderer.Paint.inc").read_text(
            encoding="utf-8")
        self.assertIn("FirstPersonRefreshDue(", paint)
        self.assertIn("MapAnimations::IsTileAnimatedForFirstPerson(", paint)

    def test_real_cpp_gate_exercises_unchanged_shadow_and_dynamic_redraws(self):
        compiler = shutil.which("g++") or shutil.which("clang++")
        if not compiler:
            self.skipTest("C++ compiler unavailable")
        cpp = r"""
#include "FirstPersonStreaming.h"
#include <cassert>
using namespace OpenRCT2::Paint;
int main() {
    for (int mask = 0; mask < 16; ++mask) {
        bool mapChanged = (mask & 1) != 0;
        bool animated = (mask & 2) != 0;
        bool dependent = (mask & 4) != 0;
        bool inspector = (mask & 8) != 0;
        bool needs = FirstPersonNativeRedrawNeedsRevalidation(
            mapChanged, animated, dependent, inspector);
        assert(needs == (mask != 0));
    }
    // A static, already-observed world tile survives a viewport redraw
    // caused by a guest shadow or unrelated 2D animation.
    assert(!FirstPersonNativeRedrawNeedsRevalidation(false,false,false,false));
    // Editing land, operating a ride, or entering an inspector scene
    // still demands reclassification of source appearance.
    assert(FirstPersonNativeRedrawNeedsRevalidation(true,false,false,false));
    assert(FirstPersonNativeRedrawNeedsRevalidation(false,true,false,false));
}
"""
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "native.cpp"
            binary = Path(directory) / "native"
            source.write_text(cpp, encoding="utf-8")
            subprocess.run(
                [compiler, "-std=c++20", "-pthread", "-I", str(STREAM.parent),
                 str(source), "-o", str(binary)],
                check=True, capture_output=True, text=True)
            subprocess.run([str(binary)], check=True, capture_output=True, text=True)


if __name__ == "__main__":
    unittest.main()
