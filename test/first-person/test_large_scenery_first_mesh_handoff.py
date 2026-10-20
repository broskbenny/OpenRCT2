"""A cold 3D reconstruction must not erase its native large-scenery body."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
CORE = ROOT / "src/openrct2/paint/FirstPersonRenderer.Core.inc"


class LargeSceneryFirstMeshHandoffTest(unittest.TestCase):
    def test_actual_production_suppression_predicate(self):
        compiler = shutil.which("clang++") or shutil.which("g++")
        if compiler is None:
            self.skipTest("No C++ compiler")
        core = CORE.read_text(encoding="utf-8")
        start = core.index(
            "        [[nodiscard]] bool IsReconstructedLargeSceneryBody(")
        end = core.index(
            "        [[nodiscard]] bool IsResidentStaticSurface(", start)
        implementation = core[start:end]
        cpp = r"""
#include <cstdint>
#include <vector>
#include <unordered_map>
#include <cassert>
struct Image {
    uint32_t index=0;
    bool hasValue=true;
    bool HasValue() const {return hasValue;}
    uint32_t GetIndex() const {return index;}
};
struct FirstPersonSurface {
    uint64_t reconstructionGroup=0;
    Image image{};
};
struct Geometry {
    bool dirty=false;
    uint64_t lastSeen=0;
    std::vector<int> surfaces;
    uint32_t bodyImageFirst=200;
    uint32_t bodyImageLast=220;
};
struct PendingBody {uint32_t first, last;};
bool _largeSceneryGeometryEnabled=true;
std::unordered_map<uint64_t, Geometry> _largeSceneryGeometryCache;
std::unordered_map<uint64_t, PendingBody> _pendingLargeSceneryBodies;
""" + implementation + r"""
int main() {
    FirstPersonSurface native{42, Image{205,true}};
    constexpr uint64_t frame=100;
    _pendingLargeSceneryBodies[42]={200,220};
    // No replacement has ever been published: retain native appearance.
    assert(!IsReconstructedLargeSceneryBody(native,frame));
    _largeSceneryGeometryCache[42]={false,frame,{},200,220};
    // An empty or unusable reconstruction also must not hide the source.
    assert(!IsReconstructedLargeSceneryBody(native,frame));
    _largeSceneryGeometryCache[42]={true,99,{1,2},200,220};
    // Prior 3D output remains last-known-good while a replacement builds.
    assert(IsReconstructedLargeSceneryBody(native,frame));
    _pendingLargeSceneryBodies.erase(42);
    _largeSceneryGeometryCache[42]={false,frame,{1,2},200,220};
    assert(IsReconstructedLargeSceneryBody(native,frame));
    native.image.index=199;
    assert(!IsReconstructedLargeSceneryBody(native,frame));
    native.image.index=220;
    assert(!IsReconstructedLargeSceneryBody(native,frame));
    native.image.index=205;
    _largeSceneryGeometryCache[42].lastSeen=99;
    assert(!IsReconstructedLargeSceneryBody(native,frame));
    _largeSceneryGeometryEnabled=false;
    assert(!IsReconstructedLargeSceneryBody(native,frame));
}
"""
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "handoff.cpp"
            binary = Path(directory) / "handoff"
            source.write_text(cpp, encoding="utf-8")
            subprocess.run([compiler, "-std=c++20", str(source), "-o", str(binary)],
                           check=True, capture_output=True, text=True)
            subprocess.run([str(binary)], check=True, capture_output=True, text=True)


if __name__ == "__main__":
    unittest.main()
