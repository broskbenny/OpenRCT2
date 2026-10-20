"""Native observation churn cannot starve 32x32 packets; real map edits still restart."""
from pathlib import Path
import random
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
WORLD = ROOT / "src/openrct2/paint/FirstPersonRenderer.World.inc"
MAIN = ROOT / "src/openrct2/paint/FirstPersonRenderer.cpp"
PROGRESS = ROOT / "src/openrct2/paint/FirstPersonPacketBuildProgress.h"


class NativeSourcePacketEditSerialTest(unittest.TestCase):
    def test_production_map_edits_are_isolated_from_native_source_arrivals(self):
        world = WORLD.read_text(encoding="utf-8")
        main = MAIN.read_text(encoding="utf-8")
        self.assertIn("uint64_t initialMapEditSerial = 0;", world)
        self.assertIn("static std::unordered_map<uint64_t, uint64_t>\n"
                      "            _authoritativeRegionEditSerials;", world)
        self.assertIn("void MarkFirstPersonAuthoritativeRegionEdit(", world)
        self.assertIn("FirstPersonStagedPacketNeedsSourceRestart(", world)
        self.assertIn("staging.initialMapEditSerial =\n"
                      "                    FirstPersonRegionMapEditSerial(regionKey);", world)
        self.assertIn("MarkFirstPersonAuthoritativeRegionEdit(regionKey);", main)
        self.assertIn("_authoritativeRegionEditSerials.clear();", main)
        # Completed native captures still invalidate the full packet for
        # later reconciliation, but they are not authoritative map edits.
        paint = (ROOT / "src/openrct2/paint/FirstPersonRenderer.Paint.inc").read_text(
            encoding="utf-8")
        self.assertIn("MarkStaticRegionDirtyForTile(tx, ty);", paint)
        self.assertNotIn("MarkFirstPersonAuthoritativeRegionEdit(", paint)
        self.assertIn("packet.dirty = missedSourceChanges;", world)

    def test_worker_survives_benign_source_revision_and_rejects_structural_edits(self):
        world = WORLD.read_text(encoding="utf-8")
        worker = world.split("const auto serial = staging.progress.invalidationSerial;", 1)[1].split(
            "if (!accepted)", 1)[0]
        self.assertIn("const auto mapEditSerial = staging.initialMapEditSerial;", worker)
        self.assertIn("FirstPersonRegionMapEditSerial(regionKey)", worker)
        self.assertIn("pending->second.initialMapEditSerial != mapEditSerial", worker)
        self.assertIn("pending->second.buildToken != token", worker)
        self.assertNotIn("live->second.invalidationSerial != serial", worker)
        self.assertIn("pending->second.progress.invalidationSerial == serial", worker)
        self.assertIn("const bool missedSourceChanges =", world)
        self.assertIn("staging.progress.invalidationSerial != published.invalidationSerial", world)

    def test_compiled_restart_policy_and_randomized_capture_storm(self):
        compiler = shutil.which("clang++") or shutil.which("g++")
        if compiler is None:
            self.skipTest("C++ compiler unavailable")
        cpp = r"""
#include "FirstPersonPacketBuildProgress.h"
#include <cassert>
#include <cstdint>
#include <limits>
#include <random>
using namespace OpenRCT2::Paint;
int main() {
    const auto invalid = std::numeric_limits<uint64_t>::max();
    assert(FirstPersonStagedPacketNeedsSourceRestart(invalid, false, 0, 0));
    assert(!FirstPersonStagedPacketNeedsSourceRestart(5, false, 1, 2));
    assert(!FirstPersonStagedPacketNeedsSourceRestart(5, true, 10, 10));
    assert(FirstPersonStagedPacketNeedsSourceRestart(5, true, 10, 11));
    // 1000 new sprite observations must not restart the partially built
    // packet; only a map edit or a failed worker snapshot is allowed to.
    uint64_t sourceSerial = 5, stagedSerial = 5;
    uint64_t stagedMap = 10, liveMap = 10;
    for (int i=0;i<1000;++i) {
        ++sourceSerial;
        assert(!FirstPersonStagedPacketNeedsSourceRestart(
            stagedSerial, true, stagedMap, liveMap));
    }
    assert(sourceSerial != stagedSerial);
    ++liveMap;
    assert(FirstPersonStagedPacketNeedsSourceRestart(
        stagedSerial, true, stagedMap, liveMap));
    std::mt19937_64 rng(20261020);
    for (int i=0;i<200000;++i) {
        bool published = bool(rng()&1);
        uint64_t stage = rng()%1000;
        uint64_t live = rng()%1000;
        uint64_t source = rng()%1000;
        if (rng()%1000==0) source=invalid;
        bool shouldRestart = FirstPersonStagedPacketNeedsSourceRestart(
            source, published, stage, live);
        assert(shouldRestart == (source == invalid ||
            (published && stage != live)));
    }
}
"""
        with tempfile.TemporaryDirectory() as td:
            src = Path(td) / "staging.cpp"
            binary = Path(td) / "staging"
            src.write_text(cpp, encoding="utf-8")
            subprocess.run([compiler, "-std=c++20", "-O2", "-I",
                            str(PROGRESS.parent), str(src), "-o", str(binary)],
                           check=True, capture_output=True, text=True)
            subprocess.run([str(binary)], check=True, capture_output=True, text=True)


if __name__ == "__main__":
    unittest.main()
