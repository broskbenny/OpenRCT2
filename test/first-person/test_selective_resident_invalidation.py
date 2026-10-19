"""Resident source-refresh invalidation: local only when old and new are independent."""
from pathlib import Path
import random
import unittest

ROOT = Path(__file__).resolve().parents[2]
PAINT = ROOT / "src/openrct2/paint/FirstPersonRenderer.Paint.inc"
LOCAL = ROOT / "src/openrct2/paint/FirstPersonRenderer.LocalScenery.inc"
CORE = ROOT / "src/openrct2/paint/FirstPersonRenderer.Core.inc"


def route(previous_coherent, current_coherent, changed, animated=False):
    if not changed or animated:
        return "none"
    return "coherent" if previous_coherent or current_coherent else "local"


class SelectiveResidentInvalidationTest(unittest.TestCase):
    def test_previous_and_current_ownership_are_preserved(self):
        paint = PAINT.read_text(encoding="utf-8")
        core = CORE.read_text(encoding="utf-8")
        local = LOCAL.read_text(encoding="utf-8")
        self.assertIn("bool hadCoherentResident = false;", core)
        self.assertIn("bool hadCoherentCameraIndependent = false;", core)
        self.assertIn("bool FirstPersonResidentContainsCoherentGeometry(", local)
        self.assertIn("IsResidentStaticSurface(surface)", world)
        self.assertIn("!FirstPersonIndependentLocalScenery(surface)", world)
        self.assertIn("cached.hadCoherentCameraIndependent || coherent", paint)
        self.assertIn("variant.hadCoherentResident || coherent", paint)
        self.assertIn("MarkLocalSceneryDirtyForTile(tx, ty);", paint)
        self.assertIn("MarkStaticRegionDirtyForTile(tx, ty);", paint)
        self.assertIn("cached.hadCoherentCameraIndependent = coherent;", paint)
        self.assertIn("variant.hadCoherentResident = coherent;", paint)

    def test_ownership_transition_never_skips_parent(self):
        for prev in (False, True):
            for now in (False, True):
                self.assertEqual(route(prev, now, True),
                                 "coherent" if (prev or now) else "local")
                self.assertEqual(route(prev, now, False), "none")
                self.assertEqual(route(prev, now, True, animated=True), "none")

    def test_randomized_lifetimes_and_output_changes(self):
        rng = random.Random(20261019)
        for _ in range(100000):
            prev = rng.random() < .3
            now = rng.random() < .3
            changed = rng.random() < .6
            animated = rng.random() < .1
            result = route(prev, now, changed, animated)
            if result == "local":
                self.assertFalse(prev)
                self.assertFalse(now)
                self.assertTrue(changed)
                self.assertFalse(animated)
            if changed and not animated and (prev or now):
                self.assertEqual(result, "coherent")


if __name__ == "__main__":
    unittest.main()
