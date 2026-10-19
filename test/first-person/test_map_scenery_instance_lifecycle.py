"""Compiler-backed protection against stale and duplicated local scenery.

The actual production instance-key helper is extracted and compiled with small
TileElement stubs. This exercises authoritative changes to pose, withering and
remaps; it does not invent an alternative key-generation implementation.
"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
LOCAL = ROOT / "src/openrct2/paint/FirstPersonRenderer.LocalScenery.inc"
CORE = ROOT / "src/openrct2/paint/FirstPersonRenderer.Core.inc"
SMALL = ROOT / "src/openrct2/paint/FirstPersonRenderer.SmallScenery.inc"


class MapSceneryInstanceLifecycleTest(unittest.TestCase):
    def test_exact_native_instance_suppression_is_scoped_to_its_element(self):
        local = LOCAL.read_text(encoding="utf-8")
        core = CORE.read_text(encoding="utf-8")
        self.assertIn("std::unordered_set<uint64_t> mapOwnedNativeKeys;", local)
        self.assertIn("mapOwnedNativeKeys.contains(source.sourceInstanceKey)", local)
        self.assertIn("mapOwnedNativeKeys.insert(nativeKey == 0 ? 1 : nativeKey)", local)
        self.assertIn("ExtendStableKey(nativeKey, uint32_t(tx));", local)
        self.assertIn("ExtendStableKey(nativeKey, uint32_t(ty));", local)
        self.assertIn("ExtendStableKey(nativeKey, elementIndex);", local)
        self.assertIn("TileElementType::smallScenery", local)
        self.assertIn("FirstPersonPaintSourceInstanceKey(", core)
        self.assertIn("ExtendStableKey(key, index);", core)

    def test_source_exact_cpp_identity_changes_with_appearance_and_geometry(self):
        compiler = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++") or shutil.which("cl")
        if not compiler:
            self.skipTest("C++ compiler unavailable")
        local = LOCAL.read_text(encoding="utf-8")
        start = local.index("        [[nodiscard]] uint64_t FirstPersonLocalSmallSceneryInstanceKey(")
        end = local.index("        void SubmitVisibleLocalSceneryRegions(", start)
        real_function = local[start:end]
        source = r"""
#include <cstdint>
#include <cassert>
#include <random>
struct SmallSceneryEntry { uint32_t id = 0; };
struct SmallSceneryElement {
    uint32_t entryIndex = 1, baseZ = 16, direction = 0, quadrant = 0;
    uint32_t occupied = 15, stage = 0, primary = 1, secondary = 2, tertiary = 3;
    const SmallSceneryEntry* entry = nullptr;
    uint32_t getEntryIndex() const { return entryIndex; }
    uint32_t getBaseZ() const { return baseZ; }
    uint32_t getDirection() const { return direction; }
    uint32_t getSceneryQuadrant() const { return quadrant; }
    uint32_t getOccupiedQuadrants() const { return occupied; }
    uint32_t getPrimaryColour() const { return primary; }
    uint32_t getSecondaryColour() const { return secondary; }
    uint32_t getTertiaryColour() const { return tertiary; }
    const SmallSceneryEntry* getEntry() const { return entry; }
};
uint32_t FirstPersonSmallSceneryWitherStage(
    const SmallSceneryEntry&, const SmallSceneryElement& e) { return e.stage; }
void ExtendStableKey(uint64_t& key, uint64_t value) {
    key ^= value;
    key *= 1099511628211ull;
}
""" + real_function + r"""
int main() {
    SmallSceneryEntry entry;
    SmallSceneryElement base; base.entry = &entry;
    const auto key = FirstPersonLocalSmallSceneryInstanceKey(4, 9, 2, base);
    auto current = base;
    current.direction = 1;
    assert(key != FirstPersonLocalSmallSceneryInstanceKey(4, 9, 2, current));
    current = base; current.quadrant = 2;
    assert(key != FirstPersonLocalSmallSceneryInstanceKey(4, 9, 2, current));
    current = base; current.occupied = 3;
    assert(key != FirstPersonLocalSmallSceneryInstanceKey(4, 9, 2, current));
    current = base; current.stage = 1;
    assert(key != FirstPersonLocalSmallSceneryInstanceKey(4, 9, 2, current));
    current = base; current.primary = 10;
    assert(key != FirstPersonLocalSmallSceneryInstanceKey(4, 9, 2, current));
    current = base; current.secondary = 10;
    assert(key != FirstPersonLocalSmallSceneryInstanceKey(4, 9, 2, current));
    current = base; current.tertiary = 10;
    assert(key != FirstPersonLocalSmallSceneryInstanceKey(4, 9, 2, current));
    current = base; current.entryIndex = 20;
    assert(key != FirstPersonLocalSmallSceneryInstanceKey(4, 9, 2, current));
    current = base; current.baseZ = 40;
    assert(key != FirstPersonLocalSmallSceneryInstanceKey(4, 9, 2, current));
    assert(key != FirstPersonLocalSmallSceneryInstanceKey(5, 9, 2, base));
    assert(key != FirstPersonLocalSmallSceneryInstanceKey(4, 10, 2, base));
    assert(key != FirstPersonLocalSmallSceneryInstanceKey(4, 9, 3, base));
    std::mt19937 rng(20261019);
    for(int i=0; i<50000; ++i) {
        current = base; current.primary = rng()%255 + 100;
        assert(key != FirstPersonLocalSmallSceneryInstanceKey(4, 9, 2, current));
    }
}
"""
        with tempfile.TemporaryDirectory() as td:
            cpp = Path(td) / "instance.cpp"
            exe = Path(td) / ("instance.exe" if Path(compiler).stem.lower() == "cl" else "instance")
            cpp.write_text(source, encoding="utf-8")
            cmd = ([compiler, "/nologo", "/std:c++20", "/EHsc", str(cpp), "/Fe:" + str(exe)]
                   if Path(compiler).stem.lower() == "cl" else
                   [compiler, "-std=c++20", "-O0", str(cpp), "-o", str(exe)])
            result = subprocess.run(cmd, cwd=td, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            result = subprocess.run([str(exe)], cwd=td, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
