"""Persistent offscreen asset admission is finite, bounded and starvation-free."""
from pathlib import Path
import random
import unittest

ROOT = Path(__file__).resolve().parents[2]
SCAN = ROOT / "src/openrct2/paint/FirstPersonRenderer.ParkSceneryPrewarm.inc"
MAIN = ROOT / "src/openrct2/paint/FirstPersonRenderer.cpp"
SMALL = ROOT / "src/openrct2/paint/FirstPersonRenderer.SmallScenery.inc"


class Model:
    def __init__(self, count):
        self.count = count
        self.cursor = 0
        self.dirty = set()
        self.visited = set()

    def frame(self, admit, limit=32):
        postponed = set()
        examined = 0
        while examined < limit:
            more_initial = self.cursor < self.count
            use_dirty = bool(self.dirty) and (not more_initial or examined % 2 == 0)
            if use_dirty:
                tile = self.dirty.pop()
            elif more_initial:
                tile = self.cursor
                self.cursor += 1
            else:
                break
            examined += 1
            self.visited.add(tile)
            if not admit(tile):
                postponed.add(tile)
        self.dirty.update(postponed)
        return examined


class ParkSceneryDependencyTest(unittest.TestCase):
    def test_code_connects_map_invalidation_and_stops_after_first_sweep(self):
        scan = SCAN.read_text(encoding="utf-8")
        main = MAIN.read_text(encoding="utf-8")
        small = SMALL.read_text(encoding="utf-8")
        self.assertIn("void QueueFirstPersonParkSceneryTile(", scan)
        self.assertIn("QueueFirstPersonParkSceneryTile(tx, ty);", main)
        self.assertIn("_parkSceneryScanCursor < total", scan)
        self.assertIn("_parkSceneryChangedTiles.insert(key)", scan)
        self.assertIn("const bool takeDirty =", scan)
        self.assertIn("(examined & 1u) == 0", scan)
        self.assertIn("deferred.push_back(", scan)
        self.assertIn("for (const uint64_t key : deferred)", scan)
        self.assertIn("!job->second->submitted", scan)
        self.assertIn("FirstPersonMapSmallSceneryEligible(", scan)
        self.assertIn("InvalidateReadyFirstPersonSmallSceneryCells(", small)
        self.assertIn("job.waitingMapCells.clear();", small)

    def test_complete_world_traversal_terminates_and_edits_reactivate(self):
        rng = random.Random(20261019)
        for _ in range(250):
            tiles = rng.randrange(1, 3000)
            model = Model(tiles)
            for frame in range(10000):
                if frame % 3 == 0:
                    model.dirty.add(rng.randrange(tiles))
                model.frame(lambda tile: True)
                if model.cursor == tiles and not model.dirty:
                    break
            else:
                self.fail("initial map sweep starved")
            self.assertEqual(len(model.visited), tiles)
            for _ in range(10):
                self.assertEqual(model.frame(lambda tile: True), 0)
            changed = rng.randrange(tiles)
            model.dirty.add(changed)
            self.assertEqual(model.frame(lambda tile: True), 1)
            self.assertEqual(model.frame(lambda tile: True), 0)

    def test_sparse_map_scan_uses_existing_time_slice_not_32_tile_cap(self):
        source = SCAN.read_text(encoding="utf-8")
        self.assertIn("256, std::chrono::microseconds(300)", source)
        self.assertNotIn("32, std::chrono::microseconds(300)", source)
        # With a cheap empty tile, the work count must not impose several
        # thousand extra foreground frames before the rest of the park
        # can even reach its immutable reconstruction worker.
        model = Model(256 * 256)
        for _ in range(256):
            model.frame(lambda tile: True, limit=256)
        self.assertEqual(model.cursor, 256 * 256)
        self.assertEqual(len(model.visited), 256 * 256)

    def test_unadmitted_worker_inputs_are_not_lost(self):
        rng = random.Random(20261019)
        for _ in range(2000):
            tiles = rng.randrange(1, 300)
            held = rng.randrange(tiles)
            failures = rng.randrange(1, 15)
            model = Model(tiles)
            attempts = [0]
            def admit(tile):
                if tile != held:
                    return True
                attempts[0] += 1
                return attempts[0] > failures
            for _frame in range(500):
                model.frame(admit)
                if model.cursor == tiles and not model.dirty:
                    break
            self.assertEqual(model.cursor, tiles)
            self.assertEqual(len(model.visited), tiles)
            self.assertEqual(attempts[0], failures + 1)
            self.assertFalse(model.dirty)


if __name__ == "__main__":
    unittest.main()
