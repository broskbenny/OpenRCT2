"""Camera turns must not restart every incomplete static region packet."""
import random
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
WORLD = ROOT / "src/openrct2/paint/FirstPersonRenderer.World.inc"


def retain(stages, visible, frame, grace=120, limit=4):
    """Mirror the selection/lifetime policy, preserving each stage's progress."""
    for key in list(stages):
        if key in visible:
            stages[key]["last_visible"] = frame
        elif frame - stages[key]["last_visible"] > grace:
            del stages[key]
    paused = sorted(
        ((state["last_visible"], key) for key, state in stages.items()
         if key not in visible), key=lambda pair: (-pair[0], pair[1]))
    for _, key in paused[limit:]:
        del stages[key]


class PausedRegionPacketBuildsTest(unittest.TestCase):
    def test_renderer_bounds_paused_staging_without_forcing_discard_on_turn(self):
        world = WORLD.read_text(encoding="utf-8")
        lifetime = world.split("constexpr uint64_t kPausedBuildGraceFrames", 1)[1].split(
            "_snapshotBudget = FirstPersonFrameBudget(", 1)[0]
        self.assertIn("constexpr size_t kPausedBuildLimit = 4;", lifetime)
        self.assertIn("it->second.lastVisibleFrame = frame;", lifetime)
        self.assertIn("frame - it->second.lastVisibleFrame", lifetime)
        self.assertIn("pausedBuilds.emplace_back(", lifetime)
        self.assertIn("_staticRegionPacketBuilds.erase(pausedBuilds[i].second)", lifetime)
        self.assertIn("staging.lastVisibleFrame = frame;", world)
        self.assertIn("staging.progress.restart(", world)
        self.assertIn("FirstPersonStagedPacketSelectionChanged(staging)", world)

    def test_short_offscreen_turn_preserves_progress(self):
        stages = {10: {"last_visible": 100, "next_tile": 900}}
        retain(stages, set(), 115)
        self.assertEqual(stages[10]["next_tile"], 900)
        retain(stages, {10}, 116)
        self.assertEqual(stages[10]["next_tile"], 900)
        retain(stages, set(), 237)
        self.assertNotIn(10, stages)

    def test_randomized_grace_and_memory_limits(self):
        rng = random.Random(20261020)
        stages = {}
        for frame in range(1, 10000):
            visible = set(rng.sample(range(50), rng.randrange(0, 8)))
            retain(stages, visible, frame)
            for key in visible:
                state = stages.setdefault(
                    key, {"last_visible": frame, "next_tile": 0})
                state["next_tile"] += 1
            paused = [key for key in stages if key not in visible]
            self.assertLessEqual(len(paused), 4)
            self.assertTrue(all(frame - stages[key]["last_visible"] <= 120
                                for key in paused))
            self.assertTrue(visible.issubset(stages))


if __name__ == "__main__":
    unittest.main()
