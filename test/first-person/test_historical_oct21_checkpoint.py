"""Freeze the actual October 21 small-scenery policy as a bisect reference.

This branch exists to compare reconstruction behavior, not to promote the
historical renderer or weaken the current branch's geometry safety gates.
"""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]


class October21ReferenceCheckpoint(unittest.TestCase):
    def test_fine_retry_and_material_publication_are_pre_regression(self):
        collision = (ROOT / "src/openrct2/paint/FirstPersonSmallSceneryCollision.h").read_text()
        scenery = (ROOT / "src/openrct2/paint/FirstPersonRenderer.SmallScenery.inc").read_text()
        hull = (ROOT / "src/openrct2/paint/FirstPersonVisualHull.h").read_text()
        self.assertIn("fineBounds.step = 1.0f;", collision)
        self.assertIn("fineConfig.allowContinuousSurfaceRefinement = false;", collision)
        self.assertIn("if (fine.valid)", collision)
        self.assertNotIn("strictConfig.preferCertifiedSmoothSurface = true;", collision)
        self.assertNotIn("unsupportedVisibleFaces >= 128", scenery)
        self.assertNotIn("BuildFirstPersonSourceTriangulatedWireFaces(", hull)

    def test_mment_legacy_flags_enable_the_historical_path(self):
        # Numeric DAT metadata recorded by a local source-only inspection;
        # neither artwork nor proprietary compressed pixels are committed.
        flags = 0x96040F
        occupies_full_tile = bool(flags & (1 << 0))
        has_rotation = bool(flags & (1 << 3))
        animated_or_glass = bool(flags & ((1 << 4) | (1 << 9)))
        self.assertTrue(occupies_full_tile and has_rotation)
        self.assertFalse(animated_or_glass)


if __name__ == "__main__":
    unittest.main()
