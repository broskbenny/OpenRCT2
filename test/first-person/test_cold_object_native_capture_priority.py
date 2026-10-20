"""Native source admission reaches cold objects without suppressing terrain."""
from pathlib import Path
import random
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
STREAMING = ROOT / "src/openrct2/paint/FirstPersonStreaming.h"
PAINT = ROOT / "src/openrct2/paint/FirstPersonRenderer.Paint.inc"


class ColdObjectNativeCapturePriorityTest(unittest.TestCase):
    def test_authoritative_source_and_independent_fairness_paths(self):
        streaming = STREAMING.read_text(encoding="utf-8")
        paint = PAINT.read_text(encoding="utf-8")
        self.assertIn("FirstPersonColdObjectCaptureOffset(", streaming)
        self.assertIn("std::min<size_t>(visibleCount, 32)", streaming)
        self.assertIn("(frame & 3u) == 0", streaming)
        self.assertIn("static size_t captureCursor = 0;", paint)
        self.assertIn("static size_t coldObjectCursor = 0;", paint)
        self.assertIn("first->getType() == TileElementType::surface", paint)
        self.assertIn("&& first->isLastForTile()", paint)
        self.assertIn("!std::any_of(cached.rotations.begin()", paint)
        lookahead = paint.split("const size_t coldOffset =", 1)[1].split(
            "const bool useColdObjectCursor =", 1)[0]
        self.assertNotIn("|| found->second.dirty", lookahead)
        self.assertIn("if (useColdObjectCursor)", paint)
        self.assertIn("coldObjectCursor += captureExamined;", paint)
        self.assertIn("captureCursor += captureExamined;", paint)
        self.assertIn("size_t nativeCapturesRemaining = 2;", paint)

    def test_actual_cpp_policy_limits_probes_and_gives_general_sweep_a_turn(self):
        compiler = shutil.which("clang++") or shutil.which("g++")
        if compiler is None:
            self.skipTest("C++ compiler unavailable")
        cpp = r"""
#include "FirstPersonStreaming.h"
#include <cassert>
#include <cstdint>
#include <random>
#include <vector>
using namespace OpenRCT2::Paint;
int main() {
    std::mt19937 rng(20261020);
    for (size_t count=0; count<400; ++count) {
        for (size_t trial=0;trial<128;++trial) {
            std::vector<bool> cold(count, false);
            for (size_t i=0;i<count;++i)
                cold[i]=(rng()%7==0);
            size_t cursor=rng()%40000;
            uint64_t frame=trial+1;
            size_t calls=0;
            size_t offset=FirstPersonColdObjectCaptureOffset(
                count,cursor,frame,[&](size_t index){
                    assert(index<count);
                    ++calls;
                    return cold[index];
                });
            assert(calls <= (count<32 ? count:32));
            if (count==0 || frame%4==0) {
                assert(offset==count);
                assert(calls==0);
                continue;
            }
            if (offset!=count) {
                assert(offset<32 && offset<count);
                assert(cold[(cursor%count+offset)%count]);
                for (size_t i=0;i<offset;++i)
                    assert(!cold[(cursor%count+i)%count]);
            } else {
                for (size_t i=0;i<(count<32?count:32);++i)
                    assert(!cold[(cursor%count+i)%count]);
            }
        }
    }
    // A permanently cold object cannot consume the normal sweep's slot.
    size_t general=0, object=0;
    for (uint64_t frame=1; frame<=12000; ++frame) {
        const size_t count=1536;
        size_t offset=FirstPersonColdObjectCaptureOffset(
            count, object, frame, [&](size_t i){return i==33;});
        const bool special=offset<count;
        if (special) object+=offset+1;
        else if (frame%4 != 0) object+=32;
        if (!special) ++general;
    }
    assert(general >= 3000);
}
"""
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "capture.cpp"
            binary = Path(folder) / "capture"
            source.write_text(cpp, encoding="utf-8")
            subprocess.run(
                [compiler, "-std=c++20", "-O2", "-I", str(STREAMING.parent),
                 str(source), "-o", str(binary)],
                check=True, text=True, capture_output=True)
            subprocess.run([str(binary)], check=True, text=True, capture_output=True)


if __name__ == "__main__":
    unittest.main()
