"""Guard native-paint-independent admission of shared small-scenery assets."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
SCENERY = ROOT / "src/openrct2/paint/FirstPersonRenderer.SmallScenery.inc"
WORLD = ROOT / "src/openrct2/paint/FirstPersonRenderer.World.inc"
MAIN = ROOT / "src/openrct2/paint/FirstPersonRenderer.cpp"


class SmallSceneryPrewarmTest(unittest.TestCase):
    def test_uses_one_worker_path_with_persistent_asset_key(self):
        scenery = SCENERY.read_text(encoding="utf-8")
        world = WORLD.read_text(encoding="utf-8")
        main = MAIN.read_text(encoding="utf-8")
        self.assertEqual(scenery.count("BuildSmallSceneryGeometry(result, job->element"), 1)
        self.assertIn("FirstPersonSmallSceneryReconstructionKey(*entry, small)", scenery)
        self.assertIn("FirstPersonSmallSceneryReconstructionKey(*entry, *small)", scenery)
        self.assertIn("StartFirstPersonSmallSceneryAssetJob(", scenery)
        self.assertIn("PrewarmFirstPersonSmallSceneryAsset(", scenery)
        self.assertIn("StartFirstPersonSmallSceneryAssetJob(\n                    it->second, *entry, tile, 0, false);", scenery)
        self.assertIn("StartFirstPersonSmallSceneryAssetJob(\n                job, *entry, ps.MapPos, rotation, true);", scenery)
        self.assertIn("if (!job->submitted && fromNativePainter)", scenery)
        self.assertIn("void PrewarmVisibleSmallSceneryAssets(", world)
        self.assertIn("FirstPersonFrameBudget scanBudget(", world)
        self.assertIn("PrewarmFirstPersonSmallSceneryAsset(tile, *small);", world)
        self.assertIn("PrewarmVisibleSmallSceneryAssets(scene);", main)
        self.assertLess(main.index("PrewarmVisibleSmallSceneryAssets(scene);"),
                        main.index("void ClearFirstPersonSceneCache()"))

    def test_large_scenery_worker_model_reuse_is_epoch_scoped(self):
        core = (ROOT / "src/openrct2/paint/FirstPersonRenderer.Core.inc").read_text(encoding="utf-8")
        worker = (ROOT / "src/openrct2/paint/FirstPersonAsync.h").read_text(encoding="utf-8")
        self.assertIn("static thread_local uint64_t modelEpoch = 0;", core)
        self.assertIn("std::unordered_map<ImageIndex, LargeSceneryAssetModel>", core)
        self.assertIn("if (modelEpoch != epoch)", core)
        self.assertIn("assetModels.clear();", core)
        self.assertIn("assetModels.try_emplace(metadata.image)", core)
        self.assertIn("if (inserted)\n                    modelIt->second = BuildLargeSceneryAssetModel(metadata);", core)
        self.assertIn("const auto& model = modelIt->second;", core)
        self.assertIn("std::thread _worker;", worker)
        self.assertIn("++_epoch;", worker)

    def test_compiled_asset_key_is_shared_only_when_allowed(self):
        compiler = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++") or shutil.which("cl")
        if not compiler:
            self.skipTest("C++ compiler not found; use Visual Studio developer shell on Windows")
        scenery = SCENERY.read_text(encoding="utf-8")
        start = scenery.index("        [[nodiscard]] uint64_t FirstPersonSmallSceneryReconstructionKey(")
        end = scenery.index("        void StartFirstPersonSmallSceneryAssetJob(", start)
        actual = scenery[start:end]
        source = """
#include <cassert>
#include <cstdint>
struct SmallSceneryEntry { uint32_t image; bool shared; };
struct SmallSceneryElement {
    uint8_t direction, quadrant, occupied, stage;
    uint8_t getDirection() const { return direction; }
    uint8_t getSceneryQuadrant() const { return quadrant; }
    uint8_t getOccupiedQuadrants() const { return occupied; }
};
bool FirstPersonSmallSceneryCanShareRotatedAsset(const SmallSceneryEntry& e,
    const SmallSceneryElement&) { return e.shared; }
uint8_t FirstPersonSmallSceneryWitherStage(const SmallSceneryEntry&,
    const SmallSceneryElement& s) { return s.stage; }
void ExtendStableKey(uint64_t& k, uint64_t v) {
    k ^= v;
    k *= 1099511628211ull;
}
""" + actual + """
int main() {
    SmallSceneryEntry shared{ 122400, true };
    SmallSceneryElement a{ 0, 0, 15, 0 };
    SmallSceneryElement b{ 1, 3, 15, 0 };
    assert(FirstPersonSmallSceneryReconstructionKey(shared, a)
        == FirstPersonSmallSceneryReconstructionKey(shared, b));
    SmallSceneryEntry distinct{ 122400, false };
    assert(FirstPersonSmallSceneryReconstructionKey(distinct, a)
        != FirstPersonSmallSceneryReconstructionKey(distinct, b));
    b = { 0, 0, 7, 0 };
    assert(FirstPersonSmallSceneryReconstructionKey(shared, a)
        != FirstPersonSmallSceneryReconstructionKey(shared, b));
    b = { 0, 0, 15, 1 };
    assert(FirstPersonSmallSceneryReconstructionKey(shared, a)
        != FirstPersonSmallSceneryReconstructionKey(shared, b));
}
"""
        with tempfile.TemporaryDirectory() as folder:
            cpp = Path(folder) / "scenery_key.cpp"
            cpp.write_text(source, encoding="utf-8")
            is_msvc = Path(compiler).stem.lower() == "cl"
            exe = Path(folder) / ("scenery_key.exe" if is_msvc else "scenery_key")
            cmd = ([compiler, "/nologo", "/std:c++20", "/EHsc", str(cpp), "/Fe:" + str(exe)]
                   if is_msvc else
                   [compiler, "-std=c++20", "-O0", str(cpp), "-o", str(exe)])
            result = subprocess.run(cmd, cwd=folder, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run([str(exe)], cwd=folder, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
