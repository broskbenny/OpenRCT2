"""Test the production underside visibility predicate without the game."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]

SOURCE = r'''
#include "src/openrct2/paint/FirstPersonUnknownUnderside.h"
#include <array>
#include <cassert>
struct Point { float x{}, y{}, z{}; };
struct Face { Point normal{}; std::array<Point, 4> corners{}; };
using OpenRCT2::Paint::FirstPersonRevealUnknownUnderside;
int main() {
    Face roof{{0,0,-1}, {{{0,0,8},{8,0,8},{8,8,8},{0,8,8}}}};
    assert(FirstPersonRevealUnknownUnderside(roof,0,2,true,false));
    assert(!FirstPersonRevealUnknownUnderside(roof,0,2,true,true));
    assert(!FirstPersonRevealUnknownUnderside(roof,0,2,false,false));
    assert(!FirstPersonRevealUnknownUnderside(roof,0,0,true,false));
    for (auto& vertex : roof.corners) vertex.z=0;
    assert(!FirstPersonRevealUnknownUnderside(roof,0,2,true,false));
    for (auto& vertex : roof.corners) vertex.z=8;
    roof.normal.z=1;
    assert(!FirstPersonRevealUnknownUnderside(roof,0,2,true,false));
}
'''

class UnknownUndersideTests(unittest.TestCase):
    def test_cpp_predicate(self):
        if not shutil.which("g++"):
            self.skipTest("g++ unavailable")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / "test.cpp").write_text(SOURCE)
            subprocess.run(["g++","-std=c++20","-O2","-Wall","-Wextra","-Werror",
                            "-pedantic","-fsanitize=undefined",
                            "-fno-sanitize-recover=all","-I",str(ROOT),
                            str(path/"test.cpp"),"-o",str(path/"test")],check=True)
            subprocess.run([str(path/"test")],check=True)

    def test_renderer_bypasses_missing_source_for_elevated_undersides(self):
        core = (ROOT/"src/openrct2/paint/FirstPersonRenderer.Core.inc").read_text()
        self.assertIn("if (!fallbackImage.HasValue() && markUnknownUnderside)",core)
        self.assertIn("FirstPersonMissingMaterialPixel(px, py)",core)
        self.assertIn("drawablePixelCount = pixels.size()",core)

if __name__ == "__main__":
    unittest.main()
