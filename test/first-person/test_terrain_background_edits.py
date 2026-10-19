"""Authoritative offscreen terrain edits remain finite, deduplicated and fair."""
from collections import deque
from pathlib import Path
import random
import unittest

ROOT = Path(__file__).resolve().parents[2]
WORLD = ROOT / "src/openrct2/paint/FirstPersonRenderer.World.inc"
MAIN = ROOT / "src/openrct2/paint/FirstPersonRenderer.cpp"


def drain_edits(keys, unavailable, budget_on_every_step=True):
    queue = deque()
    membership = set()
    for key in keys:
        if key not in membership:
            membership.add(key)
            queue.append(key)
    attempts = {}
    limit = len(membership) * 2 + 10
    for i in range(limit):
        if not queue:
            break
        if not budget_on_every_step and (i % 3 == 0):
            continue  # an interrupted frame cannot remove a queued edit
        key = queue.popleft()
        attempts[key] = attempts.get(key, 0) + 1
        if key in unavailable and attempts[key] <= unavailable[key]:
            queue.append(key)
        else:
            membership.remove(key)
    return membership, queue, attempts


class TerrainBackgroundEditsTest(unittest.TestCase):
    def test_queued_edits_are_distinct_from_renderer_packet_invalidations(self):
        world = WORLD.read_text(encoding="utf-8")
        main = MAIN.read_text(encoding="utf-8")
        self.assertIn("void QueueFirstPersonTerrainBackgroundEdit(int32_t tx, int32_t ty)", world)
        self.assertIn("_terrainBackgroundEditedSet.insert(key).second", world)
        self.assertIn("_terrainBackgroundEditedTiles.push_back(key)", world)
        self.assertIn("backgroundCursor >= size_t(map.x) * size_t(map.y)", world)
        self.assertIn("if (terrainIndex != editedIndex)", world)
        self.assertIn("if (attemptedEdit && editedTile.has_value()", world)
        self.assertIn("_terrainBackgroundEditedTiles.pop_front();", world)
        self.assertIn("_terrainBackgroundEditedTiles.push_back(key);", world)
        self.assertIn("QueueFirstPersonTerrainBackgroundEdit(tx, ty);", main)
        self.assertIn("_terrainBackgroundEditedTiles.clear();", main)
        self.assertIn("_terrainBackgroundEditedSet.clear();", main)
        packet_dirty = world[world.index("void MarkTerrainRegionDirtyForTile("):
                             world.index("void SubmitVisibleTerrainRegions(")]
        self.assertNotIn("QueueFirstPersonTerrainBackgroundEdit(", packet_dirty)

    def test_duplicate_edits_and_failed_resource_attempts_do_not_starve(self):
        rng = random.Random(20261019)
        for _ in range(2000):
            edits = [rng.randrange(80) for _ in range(rng.randrange(1, 200))]
            distinct = set(edits)
            unavailable = {key: rng.randrange(0, 3) for key in distinct}
            remaining, queue, attempts = drain_edits(edits, unavailable)
            self.assertFalse(remaining)
            self.assertFalse(queue)
            self.assertEqual(set(attempts), distinct)
            self.assertTrue(all(attempts[key] >= 1 for key in distinct))

    def test_zero_build_budget_keeps_pending_edits(self):
        rng = random.Random(20261019)
        edits = [rng.randrange(1000) for _ in range(10000)]
        remaining, queue, attempts = drain_edits(edits, {}, budget_on_every_step=False)
        self.assertFalse(remaining)
        self.assertFalse(queue)
        self.assertEqual(set(attempts), set(edits))


if __name__ == "__main__":
    unittest.main()
