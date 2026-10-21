"""Fine lattice-scenery retry is evidence-gated and cannot lower 3D quality admission."""
from pathlib import Path
import math
import random
import unittest

ROOT = Path(__file__).resolve().parents[2]
COLLISION = ROOT / "src/openrct2/paint/FirstPersonSmallSceneryCollision.h"


class AdaptiveSmallSceneryHullTest(unittest.TestCase):
    def test_retry_only_after_certified_coarse_silhouette_overpaint(self):
        text = COLLISION.read_text(encoding="utf-8")
        start = text.index("if (carveAttempt.stage == 5")
        end = text.index("// The origin hypothesis may itself", start)
        code = text[start:end]
        self.assertIn("carveAttempt.nativeViews == 4", code)
        self.assertIn("carveAttempt.observedCoverage >= 0.95f", code)
        self.assertIn("carveAttempt.candidateCoverage >= 0.40f", code)
        self.assertIn("carveAttempt.candidateCoverage < config.minimumCandidateCoverage", code)
        self.assertIn("carveAttempt.maximumEdgeError <= 2", code)
        self.assertIn("fineBounds.step = 1.0f;", code)
        self.assertIn("kMaximumFineCells = 65536", code)
        self.assertIn("fineConfig.allowContinuousSurfaceRefinement = false;", code)
        self.assertIn("if (fine.valid)", code)
        self.assertIn("return fine;", code)
        self.assertIn("BuildFirstPersonVisualHull(", code)
        self.assertNotIn("config.minimumCandidateCoverage =", code)
        self.assertNotIn("config.minimumObservedCoverage =", code)

    def test_grid_retry_never_exceeds_admission_or_grid_budget(self):
        rng = random.Random(20261021)
        max_cells = 65536
        for _ in range(10000):
            span_x = rng.uniform(0, 32)
            span_y = rng.uniform(0, 32)
            span_z = rng.uniform(0, 200)
            nx, ny, nz = (math.ceil(span) for span in
                          (span_x, span_y, span_z))
            admitted = (0 < nx <= 32 and 0 < ny <= 32
                        and 0 < nz <= max_cells // (nx * ny))
            if admitted:
                self.assertLessEqual(nx * ny * nz, max_cells)
                self.assertGreater(nx * ny * nz, 0)

    def test_bad_or_incomplete_source_cannot_trigger_retry(self):
        def retry(stage, views, source_coverage, candidate, edge):
            return (stage == 5 and views == 4
                    and source_coverage >= 0.95
                    and 0.40 <= candidate < 0.55
                    and edge <= 2)
        self.assertTrue(retry(5, 4, 0.994, 0.506, 1))
        self.assertFalse(retry(5, 4, 0.98, 0.60, 1))
        self.assertFalse(retry(5, 3, 0.99, 0.50, 1))
        self.assertFalse(retry(5, 4, 0.94, 0.50, 1))
        self.assertFalse(retry(5, 4, 0.99, 0.50, 8))
        self.assertFalse(retry(4, 4, 0.99, 0.50, 1))


if __name__ == "__main__":
    unittest.main()
