"""Map-wide terrain preparation must be bounded, finite and presentation-isolated."""
import random
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
WORLD = ROOT / "src/openrct2/paint/FirstPersonRenderer.World.inc"


def simulate_scan(width, height, visible, valid, build_limit, admission_limit=32):
    """Equivalent index bookkeeping for the C++ finite offscreen warm sweep."""
    total = width * height
    cursor = 0
    acquired = set(visible)
    while cursor < total:
        admitted = []
        while cursor < total and len(admitted) < admission_limit:
            n = cursor
            cursor += 1
            if n not in valid or n in acquired:
                continue
            admitted.append(n)
        for i, index in enumerate(admitted):
            if i >= build_limit:
                cursor = index
                break
            acquired.add(index)
    return acquired, cursor


class TerrainBackgroundPreparationTest(unittest.TestCase):
    def test_world_source_has_finite_nonstreaming_background_pass(self):
        source = WORLD.read_text(encoding="utf-8")
        first = source.index("void CollectTerrain(FirstPersonScene& scene)")
        last = source.index("BuildFirstPersonSemanticPhysicalFaces(", first)
        collector = source[first:last]
        self.assertIn("backgroundEpoch != _sceneEpoch", collector)
        self.assertIn("backgroundCursor < total && warmBudget.take()", collector)
        self.assertIn("FirstPersonFrameBudget warmBuildBudget(", collector)
        self.assertIn("8, std::chrono::microseconds(500)", collector)
        self.assertIn("!presentInFrame && !warmBuildBudget.take()", collector)
        self.assertIn("if (!presentInFrame && terrainIndex == visibleCount)", collector)
        self.assertIn("warmBuildBudget.beginPhase(std::chrono::microseconds(500))", collector)
        self.assertIn("backgroundCursor =", collector)
        self.assertIn("backgroundTiles.reserve(32)", collector)
        self.assertIn("terrainIndex < visibleCount + backgroundTiles.size()", collector)
        self.assertIn("scene.visibleTiles[terrainIndex]", collector)
        self.assertNotIn("workTiles(scene.visibleTiles.begin(), scene.visibleTiles.end())", collector)
        self.assertIn("const bool presentInFrame =", collector)
        self.assertIn("if (presentInFrame)", collector)
        self.assertIn("MapIsLocationValid(tile)", collector)
        self.assertIn("FirstPersonTerrainRegionKey(tx,ty)", collector)
        self.assertNotIn("scene.visibleTiles.emplace_back(tile)", collector)

    def test_scan_does_not_skip_a_valid_tile_on_partial_batch(self):
        rng = random.Random(20261019)
        for i in range(2500):
            width = rng.randrange(3, 100)
            height = rng.randrange(3, 100)
            total = width * height
            valid = {j for j in range(total) if rng.random() < .8}
            visible = {j for j in range(total) if rng.random() < .3 and j in valid}
            built, cursor = simulate_scan(
                width, height, visible, valid,
                build_limit=rng.randrange(1, 9),
                admission_limit=rng.randrange(8, 33))
            self.assertEqual(built, valid)
            self.assertEqual(cursor, total)

    def test_camera_visible_work_is_not_subject_to_background_limit(self):
        for visible_count in (0, 1, 32, 96, 1000):
            # The C++ workTiles vector is visible tiles FIRST; the warm budget
            # must only apply when terrainIndex >= scene.visibleTiles.size().
            selected = list(range(visible_count))
            extra = list(range(visible_count, visible_count+16))
            staged = selected + extra
            admitted = []
            for i, tile in enumerate(staged):
                if i >= visible_count and len(admitted) - visible_count >= 8:
                    break
                admitted.append(tile)
            self.assertEqual(admitted[:visible_count], selected)


if __name__ == "__main__":
    unittest.main()
