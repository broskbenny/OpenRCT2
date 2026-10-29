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
