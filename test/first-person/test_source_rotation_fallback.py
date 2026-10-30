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
        self.assertIn("selected = FirstPersonAvailableGroupSourceRotation(\n                                        cached, captured, surface.reconstructionGroup, false);", world)
        self.assertIn("selected = FirstPersonAvailableGroupSourceRotation(\n                                        cached, group->second.selectedRotation,\n                                        staticSurface.reconstructionGroup, true);", paint)
        for source in (world, paint):
            self.assertIn("std::vector<std::pair<uint64_t, uint8_t>> selectedGroupViews;", source)
            self.assertIn("const auto prior = std::find_if(", source)
            self.assertIn("selectedGroupViews.emplace_back(", source)
        self.assertIn("FP_SOURCE_COVERAGE visibleTiles=%zu objectTiles=%zu", paint)
        for field in (
            "coldTiles", "dirtyTiles", "observedTiles",
            "coldObjectTiles", "dirtyObjectTiles", "observedObjectTiles",
            "semanticObservedTiles", "objectViewPending", "presentedRegions"
        ):
            self.assertIn(f"{field}=%zu", paint)
        self.assertIn("nativeCapturesRemaining = 2;", paint)
        local = (ROOT / "src/openrct2/paint/FirstPersonRenderer.LocalScenery.inc").read_text(encoding="utf-8")
        self.assertIn("FirstPersonCachedReadyInstanceRotation(\n                                cached, surface, false, selectedNativeInstances)", world)
        self.assertIn("FirstPersonReadyInstanceRotation(\n                                        cached, staticSurface, true)", paint)
        self.assertIn("FirstPersonReadyInstanceRotation(\n                                                    cached, surface, false)", local)
        self.assertIn("while (staging.tileCursor.phase <= 6)", local)
        self.assertIn("selectedNativeInstances", paint)
        self.assertIn("selectedNativeInstances", local)


    def test_compiled_per_object_rotation_selection(self):
        compiler = shutil.which("g++") or shutil.which("clang++")
        if not compiler:
            self.skipTest("C++20 compiler unavailable")
        core = (ROOT / "src/openrct2/paint/FirstPersonRenderer.Core.inc").read_text(encoding="utf-8")
        header = (ROOT / "src/openrct2/paint/FirstPersonStreaming.h").read_text(encoding="utf-8")
        first = core.index("        template<typename Cache>" + chr(10)
                           + "        [[nodiscard]] uint8_t FirstPersonReadyRotation(")
        last = core.index("        void RequestFirstPersonNativeSceneryRetry(", first)
        ready = core[first:last]
        begin = core.index("        template<typename Cache, typename Surface>" + chr(10)
                           + "        [[nodiscard]] uint8_t FirstPersonReadyInstanceRotation(")
        end = core.index("        [[nodiscard]] std::optional<uint8_t> CachedVerticalTunnelHeight", begin)
        selector = core[begin:end]
        recordStart = header.index("    struct FirstPersonNativeInstanceRecord")
        recordEnd = header.index("    // The caller must certify unchanged tile bytes", recordStart)
        record = header[recordStart:recordEnd]
        code = """
#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>
struct Surface {
    uint64_t sourceInstanceKey=0, reconstructionGroup=0;
    uint8_t diagnosticSemanticRole=0;
};
struct View {
    bool valid=false;
    std::vector<Surface> residentSurfaces, streamedSurfaces;
};
struct Cache {
    std::array<View,4> rotations{};
    bool hasSelectedRotation=false;
    uint8_t selectedRotation=0;
};
""" + record + ready + selector + """
int main() {
    Cache cached;
    cached.hasSelectedRotation=true;
    cached.selectedRotation=2;
    cached.rotations[1].valid=true;
    cached.rotations[2].valid=true;
    Surface a{100,0,1}, b{200,0,1}, support{100,0,2};
    cached.rotations[1].streamedSurfaces={a,b,support};
    cached.rotations[2].streamedSurfaces={a,support};
    // A's current view must not remove B, even though both share a tile.
    assert(FirstPersonReadyInstanceRotation(cached,a,true)==2);
    assert(FirstPersonReadyInstanceRotation(cached,b,true)==1);
    assert(FirstPersonReadyInstanceRotation(cached,support,true)==2);
    // A ready resident face wins only for its own physical component.
    cached.rotations[1].residentSurfaces={a,b};
    cached.rotations[2].residentSurfaces={a};
    assert(FirstPersonReadyInstanceRotation(cached,a,false)==2);
    assert(FirstPersonReadyInstanceRotation(cached,b,false)==1);
    // No valid source never invents a replacement.
    cached.rotations[1].valid=false;
    assert(FirstPersonReadyInstanceRotation(cached,b,true)==255);
    std::unordered_map<FirstPersonNativeInstanceRecord,uint8_t,
        FirstPersonNativeInstanceRecordHash> resolved;
    assert(FirstPersonCachedReadyInstanceRotation(cached,a,true,resolved)==2);
    assert(FirstPersonCachedReadyInstanceRotation(cached,a,true,resolved)==2);
    assert(FirstPersonCachedReadyInstanceRotation(cached,b,true,resolved)==255);
    assert(resolved.size()==2);
    // A non-tile-backed paint surface retains the ordinary fallback.
    Surface unknown{};
    assert(FirstPersonReadyInstanceRotation(cached,unknown,true)==2);
}
"""
        with tempfile.TemporaryDirectory() as folder:
            cpp, exe = Path(folder)/"rotation.cpp", Path(folder)/"rotation"
            cpp.write_text(code, encoding="utf-8")
            result = subprocess.run([compiler, "-std=c++20", "-O2", str(cpp), "-o", str(exe)],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            run = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)

    def test_nonempty_native_variant_fallback_policy(self):
        compiler = shutil.which("g++") or shutil.which("clang++")
        if not compiler:
            self.skipTest("C++ compiler unavailable")
        core = (ROOT / "src/openrct2/paint/FirstPersonRenderer.Core.inc").read_text()
        start = core.index("        template<typename Cache>" + chr(10) + "        [[nodiscard]] uint8_t FirstPersonReadyRotation(")
        end = core.index("        void RequestFirstPersonNativeSceneryRetry(", start)
        policy = core[start:end]
        program = """
#include <algorithm>
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
        compiler = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++")
        if not compiler:
            self.skipTest("C++ compiler not available")
        core = (ROOT / "src/openrct2/paint/FirstPersonRenderer.Core.inc").read_text(encoding="utf-8")
        start = core.index("        template<typename Cache>" + chr(10)
                           + "        [[nodiscard]] uint8_t FirstPersonAvailableGroupSourceRotation(")
        end = core.index("        void RequestFirstPersonNativeSceneryRetry(", start)
        policy = core[start:end]
        source = """
#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>
struct Surface { uint64_t reconstructionGroup = 0; };
struct Rotation {
    bool valid = false;
    std::vector<Surface> residentSurfaces, streamedSurfaces;
};
struct Cache {
    std::array<Rotation, 4> rotations{};
    bool hasSelectedRotation = false;
    uint8_t selectedRotation = 0;
};
""" + policy + """
int main() {
    Cache cache;
    constexpr uint64_t group = 123, other = 456;
    assert(FirstPersonAvailableGroupSourceRotation(cache, 2, group, false) == 255);
    cache.rotations[1].valid = true;
    cache.rotations[1].residentSurfaces.push_back({group});
    cache.rotations[1].streamedSurfaces.push_back({group});
    cache.hasSelectedRotation = true;
    cache.selectedRotation = 1;
    cache.rotations[2].valid = true; // Valid native session, empty group.
    assert(FirstPersonAvailableGroupSourceRotation(cache, 2, group, false) == 1);
    assert(FirstPersonAvailableGroupSourceRotation(cache, 2, group, true) == 1);
    cache.rotations[2].residentSurfaces.push_back({other});
    cache.rotations[2].streamedSurfaces.push_back({other});
    assert(FirstPersonAvailableGroupSourceRotation(cache, 2, group, false) == 1);
    assert(FirstPersonAvailableGroupSourceRotation(cache, 2, group, true) == 1);
    cache.rotations[2].residentSurfaces.push_back({group});
    assert(FirstPersonAvailableGroupSourceRotation(cache, 2, group, false) == 2);
    assert(FirstPersonAvailableGroupSourceRotation(cache, 2, group, true) == 1);
    cache.rotations[2].streamedSurfaces.push_back({group});
    assert(FirstPersonAvailableGroupSourceRotation(cache, 2, group, true) == 2);
    cache.rotations[1].valid = false;
    cache.rotations[2].valid = false;
    cache.rotations[3].valid = true;
    cache.rotations[3].residentSurfaces.push_back({group});
    assert(FirstPersonAvailableGroupSourceRotation(cache, 0xffu, group, false) == 3);
    assert(FirstPersonAvailableGroupSourceRotation(cache, 0xffu, other, false) == 255);
}
"""
        with tempfile.TemporaryDirectory() as folder:
            cpp, exe = Path(folder) / "source_rotation.cpp", Path(folder) / "source_rotation"
            cpp.write_text(source, encoding="utf-8")
            built = subprocess.run([compiler, "-std=c++20", "-O0", str(cpp), "-o", str(exe)],
                                   capture_output=True, text=True)
            self.assertEqual(built.returncode, 0, built.stderr)
            ran = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(ran.returncode, 0, ran.stderr)


if __name__ == "__main__":
    unittest.main()
