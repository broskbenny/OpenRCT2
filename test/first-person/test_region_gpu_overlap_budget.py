"""Resident GPU replacements must make progress even when geometry shrinks."""
from pathlib import Path
import random
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
HEADER = ROOT / "src/openrct2-ui/drawing/engines/opengl/FirstPersonRegionResidencyBudget.h"
GL = ROOT / "src/openrct2-ui/drawing/engines/opengl/FirstPersonGLRenderer.cpp"


class RegionGpuOverlapBudgetTest(unittest.TestCase):
    def test_production_counts_existing_visible_vbos_before_evicting(self):
        gl = GL.read_text(encoding="utf-8")
        self.assertIn('#include "FirstPersonRegionResidencyBudget.h"', gl)
        start = gl.index("size_t visibleBytes = 0")
        end = gl.index("if (_pendingRegion)", start)
        section = gl[start:end]
        self.assertIn("residentVisibleBytes += it->second.bytes;", section)
        self.assertIn("it->second.lastSeen = _frame;", section)
        self.assertIn("DiscardRegionBuffer(packet.key);", section)
        self.assertIn("it->second.sceneEpoch != packet.sceneEpoch", section)
        self.assertIn("FirstPersonVisibleRegionGpuBudget(", section)
        self.assertIn("visibleBytes, residentVisibleBytes,", section)
        self.assertIn("largestPacketBytes);", section)
        self.assertNotIn("std::max(kRegionGpuLimit, visibleBytes + largestPacketBytes)", section)
        self.assertIn("it->second.lastSeen != _frame", gl)
        self.assertIn("DiscardRegionBuffer(victim->first)", gl)

    def test_compiled_production_budget_handles_shrink_growth_and_overflow(self):
        compiler = shutil.which("g++") or shutil.which("clang++")
        if compiler is None:
            self.skipTest("No C++ compiler available")
        cpp = r"""
#include "FirstPersonRegionResidencyBudget.h"
#include <cassert>
#include <algorithm>
#include <cstdint>
#include <limits>
#include <random>
using namespace OpenRCT2::Ui;
int main() {
    const size_t mib = 1024 * 1024;
    // Two preexisting visible 80-MiB buffers shrink to 12 MiB each.
    // The old desired-only budget was 96 MiB and could never replace either
    // last-known-good resident because both must remain drawable.
    assert(FirstPersonVisibleRegionGpuBudget(96*mib,24*mib,160*mib,12*mib)
        == 172*mib);
    // Growing geometry must likewise accommodate the new working set.
    assert(FirstPersonVisibleRegionGpuBudget(96*mib,160*mib,24*mib,80*mib)
        == 240*mib);
    assert(FirstPersonVisibleRegionGpuBudget(96*mib,4*mib,8*mib,2*mib)
        == 96*mib);
    assert(FirstPersonVisibleRegionGpuBudget(0,0,0,0) == 0);
    assert(FirstPersonVisibleRegionGpuBudget(0,100,90,30) == 130);
    assert(FirstPersonVisibleRegionGpuBudget(0,90,100,30) == 130);
    assert(FirstPersonVisibleRegionGpuBudget(0,
        std::numeric_limits<size_t>::max()-10, 20, 40)
        == std::numeric_limits<size_t>::max());
    std::mt19937_64 rng(20261020);
    for (int i=0; i<250000; ++i) {
        size_t desired = rng() % 10000000;
        size_t resident = rng() % 10000000;
        size_t largest = rng() % 1000000;
        size_t baseline = rng() % 15000000;
        size_t budget = FirstPersonVisibleRegionGpuBudget(
            baseline, desired, resident, largest);
        assert(budget >= baseline);
        assert(budget >= desired + largest);
        assert(budget >= resident + largest);
    }
}
"""
        with tempfile.TemporaryDirectory() as td:
            source = Path(td) / "budget.cpp"
            binary = Path(td) / "budget"
            source.write_text(cpp, encoding="utf-8")
            subprocess.run(
                [compiler, "-std=c++20", "-O2", "-I",
                 str(HEADER.parent), str(source), "-o", str(binary)],
                check=True, capture_output=True, text=True)
            subprocess.run([str(binary)], check=True, capture_output=True, text=True)


if __name__ == "__main__":
    unittest.main()
