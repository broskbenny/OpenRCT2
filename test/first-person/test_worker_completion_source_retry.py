"""Worker completion must not masquerade as an authoritative world edit."""
from pathlib import Path
import random
import unittest

ROOT = Path(__file__).resolve().parents[2]
MAIN = ROOT / "src/openrct2/paint/FirstPersonRenderer.cpp"
CORE = ROOT / "src/openrct2/paint/FirstPersonRenderer.Core.inc"
PAINT = ROOT / "src/openrct2/paint/FirstPersonRenderer.Paint.inc"


class WorkerCompletionSourceRetryTest(unittest.TestCase):
    def test_worker_completion_retries_views_not_map_or_gpu_regions(self):
        main = MAIN.read_text(encoding="utf-8")
        core = CORE.read_text(encoding="utf-8")
        paint = PAINT.read_text(encoding="utf-8")
        begin = main.index("_reconstructionRepaints.drain(repaintBudget")
        end = main.index("_discoveryBudget =", begin)
        completion = main[begin:end]
        self.assertIn("RequestFirstPersonNativeSceneryRetry(tile, rotation)", completion)
        self.assertIn("found->second.rotations[rotation].valid", completion)
        self.assertIn("found == _staticPaintCache.end() || found->second.dirty", completion)
        self.assertNotIn("InvalidateFirstPersonSceneTile(", completion)
        self.assertNotIn("MarkStaticRegionDirtyForTile(", completion)
        self.assertNotIn("MarkTerrainRegionDirtyForTile(", completion)
        self.assertIn("nativeRetryMask |= uint8_t(1u << rotation)", core)
        self.assertNotIn("found->second.rotations[rotation].valid = false;", core)
        self.assertIn("nativeRetryMask & uint8_t(1u << rotation)", paint)
        self.assertIn("rotationMask |= cached.nativeRetryMask;", paint)

    def test_randomized_source_view_retry_is_bounded_and_exact(self):
        rng = random.Random(20261019)
        for _ in range(150000):
            valid = [rng.randrange(2) != 0 for _ in range(4)]
            dirty = rng.random() < .2
            retry = 0
            if not dirty:
                for rotation in range(4):
                    if valid[rotation]:
                        retry |= 1 << rotation
            self.assertEqual(retry, 0 if dirty else sum((1 << i) for i,v in enumerate(valid) if v))
            self.assertLessEqual(retry, 15)


if __name__ == "__main__":
    unittest.main()
