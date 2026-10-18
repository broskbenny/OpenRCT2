"""Object-bearing tiles must receive bounded, fair native source capture priority."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
PAINT = ROOT / "src/openrct2/paint/FirstPersonRenderer.Paint.inc"


class ObjectSourcePriorityTest(unittest.TestCase):
    def test_capture_priority_preserves_terrain_fairness_and_budget(self):
        paint = PAINT.read_text(encoding="utf-8")
        self.assertIn("nativeCapturesRemaining = 2;", paint)
        self.assertIn("const bool objectPriorityPass = (frame & 3u) != 0u;", paint)
        self.assertIn(
            "if (objectPriorityPass && !FirstPersonTileHasNonSurfaceElement(tile))",
            paint,
        )
        self.assertIn(
            "const bool hasObject = FirstPersonTileHasNonSurfaceElement(tile);",
            paint,
        )
        self.assertIn("captureCursor += captureExamined;", paint)
        self.assertIn("if (nativeCapturesRemaining == 2", paint)

    def test_actual_cpp_tile_classification(self):
        compiler = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++") or shutil.which("cl")
        if not compiler:
            self.skipTest("C++ compiler not available")
        paint = PAINT.read_text(encoding="utf-8")
        start = paint.index("        [[nodiscard]] bool FirstPersonTileHasNonSurfaceElement(")
        end = paint.index("        // Reconstructing original scenery sprites", start)
        helper = paint[start:end]

        source = """
#include <cassert>
struct CoordsXY { int x; int y; };
enum class TileElementType { surface, path, track, smallScenery };
struct TileElement {
    TileElementType type;
    bool last;
    TileElementType getType() const { return type; }
    bool isLastForTile() const { return last; }
};
TileElement* currentTile = nullptr;
TileElement* MapGetFirstElementAt(CoordsXY) { return currentTile; }
""" + helper + """
int main()
{
    TileElement surfaces[] = {
        {TileElementType::surface, false}, {TileElementType::surface, true}
    };
    TileElement path[] = {
        {TileElementType::surface, false}, {TileElementType::path, true}
    };
    TileElement scenery[] = {
        {TileElementType::surface, false}, {TileElementType::smallScenery, true}
    };
    assert(!FirstPersonTileHasNonSurfaceElement({0, 0}));
    currentTile = surfaces;
    assert(!FirstPersonTileHasNonSurfaceElement({0, 0}));
    currentTile = path;
    assert(FirstPersonTileHasNonSurfaceElement({0, 0}));
    currentTile = scenery;
    assert(FirstPersonTileHasNonSurfaceElement({0, 0}));
    int objectPriorityFrames = 0;
    int allTileFrames = 0;
    for (unsigned frame = 0; frame < 8; ++frame)
    {
        if ((frame & 3u) != 0u)
            ++objectPriorityFrames;
        else
            ++allTileFrames;
    }
    assert(objectPriorityFrames == 6 && allTileFrames == 2);
}
"""
        with tempfile.TemporaryDirectory() as folder:
            cpp = Path(folder) / "object_priority.cpp"
            is_msvc = Path(compiler).stem.lower() == "cl"
            exe = Path(folder) / ("object_priority.exe" if is_msvc else "object_priority")
            cpp.write_text(source, encoding="utf-8")
            command = (
                [compiler, "/nologo", "/std:c++20", "/EHsc", str(cpp), "/Fe:" + str(exe)]
                if is_msvc else
                [compiler, "-std=c++20", "-O0", str(cpp), "-o", str(exe)]
            )
            result = subprocess.run(command, cwd=folder, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
