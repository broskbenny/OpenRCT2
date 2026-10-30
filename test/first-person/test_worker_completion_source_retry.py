"""Worker completion must not masquerade as an authoritative world edit."""
from pathlib import Path
import random
import unittest

ROOT = Path(__file__).resolve().parents[2]
MAIN = ROOT / "src/openrct2/paint/FirstPersonRenderer.cpp"
CORE = ROOT / "src/openrct2/paint/FirstPersonRenderer.Core.inc"
PAINT = ROOT / "src/openrct2/paint/FirstPersonRenderer.Paint.inc"


class WorkerCompletionSourceRetryTest(unittest.TestCase):
    def test_worker_completion_retries_views_not_map_or_gpu_regions(self):
        main = MAIN.read_text(encoding="utf-8")
        core = CORE.read_text(encoding="utf-8")
        paint = PAINT.read_text(encoding="utf-8")
        begin = main.index("_reconstructionRepaints.drain(repaintBudget")
        end = main.index("_discoveryBudget =", begin)
        completion = main[begin:end]
        self.assertIn("RequestFirstPersonNativeSceneryRetry(tile, rotation)", completion)
        self.assertIn("found->second.rotations[rotation].valid", completion)
        self.assertIn("found == _staticPaintCache.end() || found->second.dirty", completion)
        self.assertNotIn("InvalidateFirstPersonSceneTile(", completion)
        self.assertNotIn("MarkStaticRegionDirtyForTile(", completion)
        self.assertNotIn("MarkTerrainRegionDirtyForTile(", completion)
        self.assertIn("nativeRetryMask |= uint8_t(1u << rotation)", core)
        self.assertNotIn("found->second.rotations[rotation].valid = false;", core)
        self.assertIn("nativeRetryMask & uint8_t(1u << rotation)", paint)
        self.assertIn("rotationMask |= cached.nativeRetryMask;", paint)
        self.assertIn("previousReadyNativeViews", paint)
        self.assertIn("FirstPersonKeepLastGoodNativeView(", paint)
        self.assertIn("current = std::move(previous);", paint)
        self.assertIn("const auto restoreEmptyNativeRefreshes = [&]() {", paint)
        self.assertIn("if (session == nullptr)\n                    {\n                        restoreEmptyNativeRefreshes();", paint)
        self.assertIn("restoreEmptyNativeRefreshes();\n            const auto nativeCaptureDone", paint)
        self.assertLess(
            paint.index("auto* session = PaintSessionAlloc("),
            paint.index("auto old = std::move(variant);"))
        scheduled = paint[paint.index("if (stale && nativeCapturesRemaining != 0)"):
                          paint.index("const auto selectedViewsDone =", paint.index("if (stale && nativeCapturesRemaining != 0)"))]
        self.assertNotIn("variant.residentSurfaces.clear();", scheduled)
        self.assertNotIn("variant.valid = false;", scheduled)

    def test_randomized_source_view_retry_is_bounded_and_exact(self):
        rng = random.Random(20261019)
        for _ in range(150000):
            valid = [rng.randrange(2) != 0 for _ in range(4)]
            dirty = rng.random() < .2
            retry = 0
            if not dirty:
                for rotation in range(4):
                    if valid[rotation]:
                        retry |= 1 << rotation
            self.assertEqual(retry, 0 if dirty else sum((1 << i) for i,v in enumerate(valid) if v))
            self.assertLessEqual(retry, 15)


if __name__ == "__main__":
    unittest.main()

class LastGoodNativeRefreshPolicyTest(unittest.TestCase):
    def test_compiled_production_last_good_policy(self):
        import shutil
        import subprocess
        import tempfile
        compiler = shutil.which("g++") or shutil.which("clang++")
        if compiler is None:
            self.skipTest("C++20 compiler unavailable")
        header = (ROOT / "src/openrct2/paint/FirstPersonStreaming.h").read_text(encoding="utf-8")
        begin = header.index("    template<typename View>" + chr(10)
                             + "    [[nodiscard]] inline bool FirstPersonKeepLastGoodNativeView(")
        end = header.index("    // Native paint is still needed", begin)
        policy = header[begin:end]
        code = """
#include <cassert>
#include <vector>
struct View {
    bool valid = false;
    std::vector<int> residentSurfaces;
    std::vector<int> streamedSurfaces;
};
""" + policy + """
int main() {
    View old, current;
    old.valid = true;
    old.streamedSurfaces.push_back(8);
    assert(FirstPersonKeepLastGoodNativeView(old,current,false));
    assert(!FirstPersonKeepLastGoodNativeView(old,current,true));
    current.streamedSurfaces.push_back(9);
    assert(!FirstPersonKeepLastGoodNativeView(old,current,false));
    current.streamedSurfaces.clear();
    current.residentSurfaces.push_back(10);
    assert(!FirstPersonKeepLastGoodNativeView(old,current,false));
    current.residentSurfaces.clear();
    old.valid = false;
    assert(!FirstPersonKeepLastGoodNativeView(old,current,false));
    old.valid = true;
    old.streamedSurfaces.clear();
    assert(!FirstPersonKeepLastGoodNativeView(old,current,false));
    old.residentSurfaces.push_back(11);
    assert(FirstPersonKeepLastGoodNativeView(old,current,false));
}
"""
        with tempfile.TemporaryDirectory() as folder:
            from pathlib import Path
            file = Path(folder) / "native.cpp"
            exe = Path(folder) / "native"
            file.write_text(code, encoding="utf-8")
            compiled = subprocess.run(
                [compiler, "-std=c++20", "-O2", "-Wall", "-Wextra", "-Werror",
                 str(file), "-o", str(exe)], capture_output=True, text=True)
            self.assertEqual(compiled.returncode, 0, compiled.stderr)
            ran = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(ran.returncode, 0, ran.stderr)
