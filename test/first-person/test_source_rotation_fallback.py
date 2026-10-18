"""A camera turn must not withdraw already-captured native source artwork."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class SourceViewFallbackTest(unittest.TestCase):
    def test_resident_and_streamed_reconstruction_use_same_fallback(self):
        world = (ROOT / "src/openrct2/paint/FirstPersonRenderer.World.inc").read_text(encoding="utf-8")
        paint = (ROOT / "src/openrct2/paint/FirstPersonRenderer.Paint.inc").read_text(encoding="utf-8")
        self.assertIn("selected = FirstPersonAvailableSourceRotation(\n                                cached, captured);", world)
        self.assertIn("selected = FirstPersonAvailableSourceRotation(\n                                    cached, group->second.selectedRotation);", paint)
        self.assertIn("FP_SOURCE_COVERAGE visibleTiles=%zu coldTiles=%zu", paint)
        self.assertIn("nativeCapturesRemaining = 2;", paint)

    def test_actual_cpp_rotation_policy(self):
        compiler = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++")
        if not compiler:
            self.skipTest("C++ compiler not available")

        world = (ROOT / "src/openrct2/paint/FirstPersonRenderer.World.inc").read_text(encoding="utf-8")
        start = world.index("        template<typename TTileCache>\n        [[nodiscard]] uint8_t FirstPersonAvailableSourceRotation(")
        end = world.index("        [[nodiscard]] bool RebuildStaticRegionPacket(", start)
        helper = world[start:end]

        source = """
#include <array>
#include <cassert>
#include <cstdint>
#include <cstddef>
struct Rotation { bool valid = false; };
struct Cache {
    std::array<Rotation, 4> rotations{};
    bool hasSelectedRotation = false;
    uint8_t selectedRotation = 0;
};
""" + helper + """
int main()
{
    Cache cache;
    // No observation exists; presenting any artwork would be fabricated.
    assert(FirstPersonAvailableSourceRotation(cache, 2) == 0xffu);
    // A cold preferred angle keeps the previous valid source.
    cache.rotations[1].valid = true;
    cache.hasSelectedRotation = true;
    cache.selectedRotation = 1;
    assert(FirstPersonAvailableSourceRotation(cache, 2) == 1);
    assert(FirstPersonAvailableSourceRotation(cache, 0xffu) == 1);
    // Switch atomically once the preferred observation really exists.
    cache.rotations[2].valid = true;
    assert(FirstPersonAvailableSourceRotation(cache, 2) == 2);
    // If the previous selected view is invalid, use any genuinely ready view.
    cache.rotations[1].valid = false;
    cache.rotations[2].valid = false;
    cache.rotations[3].valid = true;
    assert(FirstPersonAvailableSourceRotation(cache, 2) == 3);
    cache.hasSelectedRotation = false;
    assert(FirstPersonAvailableSourceRotation(cache, 0xffu) == 3);
}
"""
        with tempfile.TemporaryDirectory() as folder:
            cpp = Path(folder) / "source_rotation.cpp"
            exe = Path(folder) / "source_rotation"
            cpp.write_text(source, encoding="utf-8")
            built = subprocess.run(
                [compiler, "-std=c++20", "-O0", str(cpp), "-o", str(exe)],
                capture_output=True, text=True)
            self.assertEqual(built.returncode, 0, built.stderr)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
