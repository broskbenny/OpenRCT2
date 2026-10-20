"""Entrances and scrolling text must survive cold/failed reconstruction."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
CORE = ROOT / "src/openrct2/paint/FirstPersonRenderer.Core.inc"


class EntranceFirstMeshHandoffTest(unittest.TestCase):
    def test_actual_production_body_and_sign_suppression(self):
        compiler = shutil.which("clang++") or shutil.which("g++")
        if compiler is None:
            self.skipTest("C++ compiler unavailable")
        text = CORE.read_text(encoding="utf-8")
        start = text.index(
            "        [[nodiscard]] bool\n            IsReconstructedParkEntranceBody(")
        end = text.index(
            "        [[nodiscard]] bool\n"
            "            PreserveParkEntranceSemanticSignPlane(", start)
        impl = text[start:end]
        cpp = r"""
#include <cstdint>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cassert>
enum class FirstPersonPaintSemanticRole : uint8_t {
    unknown, structureBody, sign
};
struct FirstPersonSurface {
    uint8_t diagnosticSemanticRole=0;
    uint64_t reconstructionGroup=0;
};
struct CachedGeometry {
    bool dirty=false;
    uint64_t lastSeen=0;
    std::vector<int> surfaces;
};
bool _parkEntranceGeometryEnabled=true;
std::unordered_set<uint64_t> _pendingParkEntranceBodies;
std::unordered_map<uint64_t,CachedGeometry> _parkEntranceGeometryCache;
""" + impl + r"""
int main() {
    constexpr uint64_t frame=100;
    FirstPersonSurface body{uint8_t(FirstPersonPaintSemanticRole::structureBody),42};
    FirstPersonSurface text{uint8_t(FirstPersonPaintSemanticRole::sign),42};
    _pendingParkEntranceBodies.insert(42);
    // Never erase the only first-frame entrance body.
    assert(!IsReconstructedParkEntranceBody(body,frame));
    // Keep the known sign-plane policy while work is pending.
    assert(IsReconstructedParkEntranceBody(text,frame));
    _parkEntranceGeometryCache[42]={true,99,{1}};
    assert(IsReconstructedParkEntranceBody(body,frame));
    _pendingParkEntranceBodies.erase(42);
    _parkEntranceGeometryCache[42]={false,frame,{1}};
    assert(IsReconstructedParkEntranceBody(body,frame));
    assert(IsReconstructedParkEntranceBody(text,frame));
    _parkEntranceGeometryCache[42]={false,frame,{}};
    // Failed geometry must not permanently erase artwork or lettering.
    assert(!IsReconstructedParkEntranceBody(body,frame));
    assert(!IsReconstructedParkEntranceBody(text,frame));
    _parkEntranceGeometryCache[42].lastSeen=99;
    assert(!IsReconstructedParkEntranceBody(body,frame));
    _parkEntranceGeometryEnabled=false;
    assert(!IsReconstructedParkEntranceBody(body,frame));
}
"""
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "entrance.cpp"
            binary = Path(directory) / "entrance"
            source.write_text(cpp, encoding="utf-8")
            subprocess.run([compiler, "-std=c++20", str(source), "-o", str(binary)],
                           check=True, capture_output=True, text=True)
            subprocess.run([str(binary)], check=True, capture_output=True, text=True)


if __name__ == "__main__":
    unittest.main()
