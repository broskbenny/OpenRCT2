"""Exact production recovery of source-backed subpixel structural members."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
HEADER = ROOT / "src/openrct2/paint/FirstPersonAssetReconstruction.h"
SMALL = ROOT / "src/openrct2/paint/FirstPersonRenderer.SmallScenery.inc"


class SubpixelSourceMaterialWitnessTest(unittest.TestCase):
    def test_only_certified_wire_faces_use_line_evidence(self):
        small = SMALL.read_text(encoding="utf-8")
        self.assertIn("&& hull->sourceTriangulatedWireSelected", small)
        self.assertIn("visibility.projected.empty()", small)
        self.assertIn("FirstPersonSubpixelFaceSourceWitnesses(", small)
        self.assertIn("source.pixels[size_t(v)", small)
        self.assertIn("owner->second.depth", small)
        self.assertIn("witness.depth + 1.0f", small)
        self.assertIn("visibility.visible.size() >= 2", small)
        self.assertIn("(visibility.projected.size() + 1) / 2", small)

    def test_production_line_witness_is_bounded_and_excludes_real_faces(self):
        compiler = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++")
        if not compiler or Path(compiler).stem.lower() == "cl":
            self.skipTest("GNU/Clang C++ compiler unavailable")
        header = HEADER.read_text(encoding="utf-8")
        start = header.index("    [[nodiscard]] inline std::optional<ScreenCoordsXY>" + chr(10)
                             + "        FirstPersonSubpixelFaceSourceSample(")
        end = header.index("    struct FirstPersonDepthVisibility", start)
        policy = header[start:end]
        source = """
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>
struct ScreenCoordsXY { int32_t x = 0, y = 0; };
""" + policy + """
int main() {
    using Point = ScreenCoordsXY;
    std::array<float, 4> depth{1, 9, 9, 1};
    auto horizontal = FirstPersonSubpixelFaceSourceWitnesses(
        {Point{0, 0}, Point{32, 0}, Point{32, 0}, Point{0, 0}}, depth);
    assert(horizontal.size() == 33);
    assert(horizontal.front().pixel.x == 0);
    assert(horizontal.back().pixel.x == 32);
    assert(horizontal.front().depth == 1);
    assert(horizontal.back().depth == 9);
    assert(horizontal[16].depth > 4 && horizontal[16].depth < 6);
    auto diagonal = FirstPersonSubpixelFaceSourceWitnesses(
        {Point{0, 0}, Point{25, 25}, Point{25, 25}, Point{0, 0}}, depth);
    assert(diagonal.size() == 26);
    assert(diagonal[12].pixel.x == diagonal[12].pixel.y);
    auto thick = FirstPersonSubpixelFaceSourceWitnesses(
        {Point{0, 0}, Point{30, 0}, Point{30, 12}, Point{0, 12}}, depth);
    assert(thick.empty());
    auto tooLong = FirstPersonSubpixelFaceSourceWitnesses(
        {Point{0, 0}, Point{200, 0}, Point{200, 0}, Point{0, 0}}, depth);
    assert(tooLong.empty());
    auto point = FirstPersonSubpixelFaceSourceWitnesses(
        {Point{5, 5}, Point{6, 5}, Point{6, 6}, Point{5, 6}}, depth);
    assert(point.size() == 1);
    auto slantedWide = FirstPersonSubpixelFaceSourceWitnesses(
        {Point{0, 0}, Point{30, 30}, Point{26, 34}, Point{-4, 4}}, depth);
    assert(slantedWide.empty());
}
"""
        with tempfile.TemporaryDirectory() as folder:
            cpp = Path(folder) / "witness.cpp"
            exe = Path(folder) / "witness"
            cpp.write_text(source, encoding="utf-8")
            build = subprocess.run(
                [compiler, "-std=c++20", "-Wall", "-Wextra", "-Werror",
                 str(cpp), "-o", str(exe)],
                capture_output=True, text=True)
            self.assertEqual(build.returncode, 0, build.stderr)
            run = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)


if __name__ == "__main__":
    unittest.main()
