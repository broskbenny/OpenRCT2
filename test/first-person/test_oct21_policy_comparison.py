"""Guard the isolated October 21 rendering-policy comparison.

This test does not assert that historic geometry is visually better.
It checks that the policy difference is deliberate, auditable, and local
to this experimental branch.
"""
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[2]
COLLISION=ROOT/"src/openrct2/paint/FirstPersonSmallSceneryCollision.h"
RENDER=ROOT/"src/openrct2/paint/FirstPersonRenderer.SmallScenery.inc"


class Oct21ComparisonPolicyTest(unittest.TestCase):
    def test_only_historic_geometry_gates_are_bypassed(self):
        collision=COLLISION.read_text()
        renderer=RENDER.read_text()
        self.assertIn("kFirstPersonHistoricalOct21SceneryPolicy = true;", collision)
        self.assertIn("config.allowStructuralBeamReconstruction = false;", collision)
        self.assertIn("config.allowSourceTriangulatedWireFallback = false;", collision)
        self.assertIn("if (!kFirstPersonHistoricalOct21SceneryPolicy)", collision)
        self.assertIn("fineConfig.allowContinuousSurfaceRefinement = false;", collision)
        self.assertIn("&& !kFirstPersonHistoricalOct21SceneryPolicy", renderer)
        self.assertIn("scene.surfaces.resize(surfaceStart);", renderer)
        self.assertIn("config.minimumCandidateCoverage = 0.55f", collision)
        self.assertIn("config.minimumObservedCoverage = 0.30f", collision)
        self.assertIn("fineBounds.step = 1.0f;", collision)


if __name__=="__main__":
    unittest.main()
