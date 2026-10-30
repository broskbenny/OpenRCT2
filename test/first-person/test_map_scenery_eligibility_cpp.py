"""Compile the actual C++ native-versus-map ownership gate with model inputs."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "src/openrct2/paint/FirstPersonRenderer.SmallScenery.inc"


class MapSceneryEligibilityCppTest(unittest.TestCase):
    def test_native_cpp_eligibility(self):
        compiler = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++") or shutil.which("cl")
        if compiler is None:
            self.skipTest("C++ compiler unavailable")
        text = SOURCE.read_text(encoding="utf-8")
        begin = text.index("        [[nodiscard]] bool FirstPersonMapSmallSceneryEligible(")
        end = text.index("        [[nodiscard]] bool FirstPersonMapSmallSceneryCanPublish(", begin)
        actual = text[begin:end]
        fixture = """
#include <cstdint>
#include <cassert>
#include <random>
enum class SmallSceneryFlag {
    isAnimated, hasFrameOffsets, hasGlass, hasOverlayImage,
    isVisibleWhenZoomed, flag17
};
struct SmallSceneryFlags {
    uint32_t bits=0;
    template <typename... T> bool hasAny(T... values) const {
        return ((bits & (1u << uint32_t(values))) || ...);
    }
};
struct SmallSceneryEntry { SmallSceneryFlags flags; };
struct SmallSceneryElement {
    bool ghost=false, invisible=false, supports=false;
    bool isGhost() const { return ghost; }
    bool isInvisible() const { return invisible; }
    bool needsSupports() const { return supports; }
};
using TileElement = SmallSceneryElement;
struct TileInspector {
    static inline const TileElement* selected = nullptr;
    static const TileElement* GetSelectedElement() { return selected; }
};
bool gTrackDesignSaveMode=false;
""" + actual + """
int main() {
    SmallSceneryEntry entry;
    SmallSceneryElement element;
    assert(FirstPersonMapSmallSceneryEligible(entry,element,0));
    assert(!FirstPersonMapSmallSceneryEligible(entry,element,1));
    TileInspector::selected = &element;
    assert(!FirstPersonMapSmallSceneryEligible(entry,element,0));
    TileInspector::selected = nullptr;
    for (int bit=0; bit<5; ++bit) {
        entry.flags.bits=1u<<bit;
        assert(!FirstPersonMapSmallSceneryEligible(entry,element,0));
    }
    // Static legacy flag17 is not a dynamic paint contract.
    entry.flags.bits=1u<<5;
    assert(FirstPersonMapSmallSceneryEligible(entry,element,0));
    entry.flags.bits=(1u<<5)|(1u<<1);
    assert(!FirstPersonMapSmallSceneryEligible(entry,element,0));
    entry.flags.bits=0;
    element.ghost=true;
    assert(!FirstPersonMapSmallSceneryEligible(entry,element,0));
    element.ghost=false;
    element.invisible=true;
    assert(!FirstPersonMapSmallSceneryEligible(entry,element,0));
    element.invisible=false;
    element.supports=true;
    // Static body eligibility is independent of native wood supports.
    assert(FirstPersonMapSmallSceneryEligible(entry,element,0));
    element.supports=false;
    gTrackDesignSaveMode=true;
    assert(!FirstPersonMapSmallSceneryEligible(entry,element,0));
    gTrackDesignSaveMode=false;
    std::mt19937 rng(20261019);
    for(int i=0;i<100000;++i) {
        entry.flags.bits = rng() & 63;
        element.ghost = (rng() & 1)!=0;
        element.invisible = (rng() & 1)!=0;
        element.supports = (rng() & 1)!=0;
        uint32_t viewFlags = rng() & 3;
        gTrackDesignSaveMode = (rng() & 1)!=0;
        const bool expected = (entry.flags.bits & 31u)==0
            && !element.ghost && !element.invisible
            && viewFlags==0 && !gTrackDesignSaveMode;
        assert(FirstPersonMapSmallSceneryEligible(
            entry,element,viewFlags)==expected);
    }
}
"""
        with tempfile.TemporaryDirectory() as directory:
            cpp = Path(directory) / "map_scenery_eligibility.cpp"
            exe = Path(directory) / ("policy.exe" if Path(compiler).stem.lower() == "cl" else "policy")
            cpp.write_text(fixture, encoding="utf-8")
            if Path(compiler).stem.lower() == "cl":
                command = [compiler, "/nologo", "/std:c++20", "/EHsc", str(cpp), "/Fe:" + str(exe)]
            else:
                command = [compiler, "-std=c++20", "-O0", str(cpp), "-o", str(exe)]
            built = subprocess.run(command, cwd=directory, capture_output=True, text=True)
            self.assertEqual(built.returncode, 0, built.stdout + built.stderr)
            ran = subprocess.run([str(exe)], cwd=directory, capture_output=True, text=True)
            self.assertEqual(ran.returncode, 0, ran.stdout + ran.stderr)


if __name__ == "__main__":
    unittest.main()
