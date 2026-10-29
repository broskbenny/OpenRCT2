"""Equal-depth ownership must not change solely from frustum hash iteration."""
from pathlib import Path
import random
import unittest

ROOT = Path(__file__).resolve().parents[2]
WORLD = ROOT / "src/openrct2/paint/FirstPersonRenderer.World.inc"
LOCAL = ROOT / "src/openrct2/paint/FirstPersonRenderer.LocalScenery.inc"


class StaticRegionPublicationOrderTest(unittest.TestCase):
    def test_coherent_regions_are_published_in_key_order(self):
        world = WORLD.read_text(encoding="utf-8")
        publish = world.split("void SubmitVisibleStaticRegions(", 1)[1]
        self.assertIn("std::vector<uint64_t> orderedRegions(", publish)
        self.assertIn("visibleRegions.begin(), visibleRegions.end()", publish)
        self.assertIn("std::sort(orderedRegions.begin(), orderedRegions.end());", publish)
        self.assertIn("for (const auto key : orderedRegions)", publish)
        self.assertLess(
            publish.index("std::sort(orderedRegions.begin(), orderedRegions.end());"),
            publish.index("scene.staticRegions.push_back({"))
        self.assertIn("std::sort(rebuildCandidates.begin(), rebuildCandidates.end(),",
                      publish)
        self.assertLess(
            publish.index("FirstPersonStaticRegionRebuildBefore);"),
            publish.index("std::vector<uint64_t> orderedRegions("))

    def test_incomplete_unpublished_packets_keep_their_build_slots(self):
        streaming = (ROOT/"src/openrct2/paint/FirstPersonStreaming.h").read_text()
        world = WORLD.read_text()
        self.assertIn("bool hasStagedWork = false;", streaming)
        self.assertIn("!a.hasPublishedGeometry && a.hasStagedWork != b.hasStagedWork", streaming)
        self.assertIn("frustum.eye, _staticRegionPacketBuilds.contains(key)", world)
        self.assertLess(streaming.index("a.hasPublishedGeometry != b.hasPublishedGeometry"),
                        streaming.index("a.hasStagedWork != b.hasStagedWork"))
        self.assertLess(streaming.index("a.hasStagedWork != b.hasStagedWork"),
                        streaming.index("a.distanceSquared != b.distanceSquared"))

    def test_compiled_rebuild_priority_survives_camera_turns(self):
        import shutil
        import subprocess
        import tempfile
        compiler = shutil.which("clang++") or shutil.which("g++")
        if compiler is None:
            self.skipTest("C++ compiler unavailable")
        program = r"""
#include "FirstPersonStreaming.h"
#include <algorithm>
#include <cassert>
#include <vector>
using namespace OpenRCT2::Paint;
int main() {
    FirstPersonVec3 eye{0, 0, 0};
    auto active = MakeFirstPersonStaticRegionRebuildCandidate(
        (2ull<<32)|2u, false, eye, true);
    auto cold = MakeFirstPersonStaticRegionRebuildCandidate(
        (1ull<<32)|1u, false, eye, false);
    auto resident = MakeFirstPersonStaticRegionRebuildCandidate(
        (1ull<<32)|1u, true, eye, true);
    assert(FirstPersonStaticRegionRebuildBefore(active, cold));
    assert(FirstPersonStaticRegionRebuildBefore(cold, resident));
    for (int n=0;n<600;++n) {
        // Turning the camera makes the new cold region nearer than the
        // staged one. Continuing progress still wins; visible resident
        // replacement remains behind the unpublished cold region.
        eye={float(n % 2 ? 5000 : -5000), float(n%3 ? -4000 : 4000), 0};
        auto a=MakeFirstPersonStaticRegionRebuildCandidate(
            (2ull<<32)|2u, false, eye, true);
        auto b=MakeFirstPersonStaticRegionRebuildCandidate(
            (1ull<<32)|1u, false, eye, false);
        auto c=MakeFirstPersonStaticRegionRebuildCandidate(
            (1ull<<32)|1u, true, eye, false);
        std::vector<FirstPersonStaticRegionRebuildCandidate> jobs{c,b,a};
        std::sort(jobs.begin(),jobs.end(),FirstPersonStaticRegionRebuildBefore);
        assert(jobs[0].key==a.key);
        assert(jobs[1].key==b.key);
    }
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            from pathlib import Path
            src, exe = Path(tmp)/"priority.cpp", Path(tmp)/"priority"
            src.write_text(program, encoding="utf-8")
            run = subprocess.run([compiler, "-std=c++20", "-O2", "-I",
                str(ROOT/"src/openrct2/paint"), str(src), "-o", str(exe)],
                text=True, capture_output=True)
            self.assertEqual(run.returncode, 0, run.stderr)
            run = subprocess.run([str(exe)], text=True, capture_output=True)
            self.assertEqual(run.returncode, 0, run.stderr)

    def test_local_and_coherent_publication_stay_stable_across_view_turns(self):
        local = LOCAL.read_text(encoding="utf-8")
        self.assertIn("std::sort(orderedVisible.begin(), orderedVisible.end());", local)
        rng = random.Random(20261020)
        for _ in range(40000):
            base = {rng.randrange(1, 16000) for _ in range(60)}
            view = set(base)
            for _ in range(12):
                view.discard(rng.randrange(1, 16000))
                view.add(rng.randrange(1, 16000))
            preserved = sorted(base & view)
            self.assertEqual([key for key in sorted(base) if key in view],
                             preserved)
            self.assertEqual([key for key in sorted(view) if key in base],
                             preserved)


if __name__ == "__main__":
    unittest.main()
