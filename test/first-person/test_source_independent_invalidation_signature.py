"""Map redraws use a durable baseline even before native source is captured."""
from pathlib import Path
import random
import unittest

ROOT = Path(__file__).resolve().parents[2]
MAIN = ROOT / "src/openrct2/paint/FirstPersonRenderer.cpp"
WORLD = ROOT / "src/openrct2/paint/FirstPersonRenderer.World.inc"
CORE = ROOT / "src/openrct2/paint/FirstPersonRenderer.Core.inc"


class InvalidationState:
    def __init__(self):
        self.baseline = None
        self.native_signature = None
        self.visibility = None
        self.changed = 0

    def discover(self, live):
        self.visibility = live
        if self.baseline is None:
            self.baseline = (
                self.native_signature if self.native_signature is not None
                else live)

    def invalidate(self, live):
        known = self.baseline if self.baseline is not None else self.native_signature
        if known == live:
            return False
        self.baseline = live
        self.changed += 1
        return True


class SourceIndependentInvalidationSignatureTest(unittest.TestCase):
    def test_production_seeds_once_and_updates_only_after_verified_change(self):
        core = CORE.read_text(encoding="utf-8")
        world = WORLD.read_text(encoding="utf-8")
        main = MAIN.read_text(encoding="utf-8")
        self.assertIn("uint64_t lastInvalidatedSignature = 0;", core)
        self.assertIn("bool hasInvalidationSignature = false;", core)
        self.assertEqual(world.count("if (!tileCache.hasInvalidationSignature)"), 1)
        self.assertEqual(world.count("if (!cached.hasInvalidationSignature)"), 1)
        self.assertIn("tileCache.valid ? tileCache.signature : snapshot.signature", world)
        self.assertIn("cached.valid ? cached.signature : snapshot.signature", world)
        self.assertIn("tileCache.lastInvalidatedSignature == signature", main)
        invalidation = main.split("void InvalidateFirstPersonSceneRegion(", 1)[1].split(
            "void InvalidateFirstPersonSceneTile(", 1)[0]
        self.assertIn("auto& tileCache = _staticPaintCache[key];", invalidation)
        self.assertLess(invalidation.index("auto& tileCache = _staticPaintCache[key];"),
                        invalidation.index("if (unchanged)"))
        self.assertNotIn("if (cached != _staticPaintCache.end())", invalidation.split(
            "for (int32_t ty = y0;", 1)[1].split("changedTiles.insert(key);", 1)[0])
        # A cold tile incurs one conservative invalidation, not an edit
        # every time the native window redraws an unchanged object.
        cold = InvalidationState()
        self.assertTrue(cold.invalidate(123))
        self.assertFalse(cold.invalidate(123))
        self.assertTrue(cold.invalidate(124))
        self.assertIn("tileCache.lastInvalidatedSignature = signature;", main)
        invalidation = main.split("void InvalidateFirstPersonSceneRegion(", 1)[1].split(
            "void InvalidateFirstPersonSceneTile(", 1)[0]
        self.assertLess(
            invalidation.index("if (unchanged)"),
            invalidation.index("tileCache.lastInvalidatedSignature = signature;"))
        self.assertLess(
            invalidation.index("tileCache.lastInvalidatedSignature = signature;"),
            invalidation.index("changedTiles.insert(key);"))

    def test_camera_visibility_observations_do_not_hide_authoritative_edits(self):
        state = InvalidationState()
        state.discover(55)
        for _ in range(1000):
            self.assertFalse(state.invalidate(55))
            state.discover(55)
        self.assertEqual(state.changed, 0)

        # Discovery may see newer bytes before a native redraw callback.
        state.discover(56)
        self.assertEqual(state.baseline, 55)
        self.assertTrue(state.invalidate(56))
        self.assertEqual(state.changed, 1)
        # Multiple equivalent redraw notifications do not rebuild packets.
        for _ in range(100):
            self.assertFalse(state.invalidate(56))
        state.discover(57)
        self.assertTrue(state.invalidate(57))
        self.assertEqual(state.changed, 2)

    def test_randomized_edit_sequences_and_uncaptured_native_source(self):
        rng = random.Random(20261020)
        for trial in range(100):
            state = InvalidationState()
            live = rng.getrandbits(64)
            state.discover(live)
            changes = 0
            for _ in range(1500):
                if rng.randrange(8) == 0:
                    live = rng.getrandbits(64)
                    changes += 1
                # Make visibility observations arbitrarily frequent, while
                # native source remains entirely uncaptured for the tile.
                if rng.randrange(3) != 0:
                    state.discover(live)
                if rng.randrange(2) == 0:
                    state.invalidate(live)
            state.invalidate(live)
            self.assertLessEqual(state.changed, changes)
            self.assertEqual(state.baseline, live)


if __name__ == "__main__":
    unittest.main()
