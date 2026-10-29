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
        self.assertIn("selected = FirstPersonAvailableSourceRotation(\n                                    cached, captured);", world)
        self.assertIn("selected = FirstPersonAvailableSourceRotation(\n                                    cached, group->second.selectedRotation);", paint)
        self.assertIn("FP_SOURCE_COVERAGE visibleTiles=%zu objectTiles=%zu", paint)
        for field in (
            "coldTiles", "dirtyTiles", "observedTiles",
            "coldObjectTiles", "dirtyObjectTiles", "observedObjectTiles",
            "semanticObservedTiles", "objectViewPending", "presentedRegions"
        ):
            self.assertIn(f"{field}=%zu", paint)
        self.assertIn("nativeCapturesRemaining = 2;", paint)

    def test_nonempty_native_variant_fallback_policy(self):
        compiler = shutil.which("g++") or shutil.which("clang++")
        if not compiler:
            self.skipTest("C++ compiler unavailable")
        core = (ROOT / "src/openrct2/paint/FirstPersonRenderer.Core.inc").read_text()
        start = core.index("        template<typename Cache>" + chr(10) + "        [[nodiscard]] uint8_t FirstPersonReadyRotation(")
        end = core.index("        void RequestFirstPersonNativeSceneryRetry(", start)
        policy = core[start:end]
        program = """
#include <array>
#include <cassert>
#include <cstdint>
#include <vector>
struct View { bool valid = false; std::vector<int> residentSurfaces, streamedSurfaces; };
struct Cache {
    std::array<View, 4> rotations{};
    bool hasSelectedRotation = false;
    uint8_t selectedRotation = 0;
};
""" + policy + """
int main() {
    Cache cache;
    assert(FirstPersonReadyRotation(cache, false) == 255);
    assert(FirstPersonReadyRotation(cache, true) == 255);
    cache.rotations[1].valid = true;
    cache.rotations[1].residentSurfaces.push_back(1);
    cache.rotations[1].streamedSurfaces.push_back(1);
    cache.hasSelectedRotation = true;
    cache.selectedRotation = 2;
    cache.rotations[2].valid = true; // A valid source with no usable geometry.
    assert(FirstPersonReadyRotation(cache, false) == 1);
    assert(FirstPersonReadyRotation(cache, true) == 1);
    cache.rotations[2].residentSurfaces.push_back(2);
    assert(FirstPersonReadyRotation(cache, false) == 2);
    assert(FirstPersonReadyRotation(cache, true) == 1);
    cache.rotations[1].valid = false;
    assert(FirstPersonReadyRotation(cache, true) == 255);
    cache.rotations[2].streamedSurfaces.push_back(2);
    assert(FirstPersonReadyRotation(cache, true) == 2);
}
"""
        with tempfile.TemporaryDirectory() as folder:
            cpp, exe = Path(folder) / "ready.cpp", Path(folder) / "ready"
            cpp.write_text(program, encoding="utf-8")
            built = subprocess.run([compiler, "-std=c++20", "-O0", str(cpp), "-o", str(exe)],
                                   capture_output=True, text=True)
            self.assertEqual(built.returncode, 0, built.stderr)
            ran = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(ran.returncode, 0, ran.stderr)

    def test_actual_cpp_rotation_policy(self):
        compiler = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++") or shutil.which("cl")
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
            is_msvc = Path(compiler).stem.lower() == "cl"
            exe = Path(folder) / ("source_rotation.exe" if is_msvc else "source_rotation")
            cpp.write_text(source, encoding="utf-8")
            args = (
                [compiler, "/nologo", "/std:c++20", "/EHsc", str(cpp), "/Fe:" + str(exe)]
                if is_msvc else
                [compiler, "-std=c++20", "-O0", str(cpp), "-o", str(exe)]
            )
            built = subprocess.run(args, cwd=folder,
                capture_output=True, text=True)
            self.assertEqual(built.returncode, 0, built.stderr)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
