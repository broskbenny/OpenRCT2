"""Structural discovery is independent of native-paint readiness.

The cached native visibility flag can be dirty for a perfectly unchanged
TileElement. Only a real authoritative signature mismatch may prevent use
of the structure index; the source element is revalidated before activation.
"""
from pathlib import Path
import random
import unittest

ROOT = Path(__file__).resolve().parents[2]
WORLD = ROOT / "src/openrct2/paint/FirstPersonRenderer.World.inc"


def may_use_index(index_signature, paint_known, paint_dirty, paint_signature,
                  current_authoritative_signature):
    uncertain = paint_dirty or (paint_known and paint_signature != index_signature)
    return not uncertain or current_authoritative_signature == index_signature


class AuthoritativeLargeSceneryDiscoveryTest(unittest.TestCase):
    def test_no_native_source_capture_requirement(self):
        world = WORLD.read_text(encoding="utf-8")
        start = world.index("void UpdateAuthoritativeLargeScenery(")
        end = world.index("void PrewarmVisibleSmallSceneryAssets(", start)
        discovery = world[start:end]
        self.assertIn("const uint64_t signature = NativeTileSignature(tile);", discovery)
        self.assertIn("&& NativeTileSignature(tile) != index->second.signature", discovery)
        self.assertIn("GetReconstructionGroup(tile, element)", discovery)
        self.assertIn("GetReconstructionGroup(\n                                group.sourceTile, element)", discovery)
        self.assertIn("if (large == nullptr)", discovery)
        self.assertIn("LargeSceneryInstanceMetadataMatches(*large, *entry)", discovery)
        self.assertNotIn("!paint->second.visibilityBoundValid", discovery)
        self.assertNotIn("if (paint == _staticPaintCache.end()", discovery)

    def test_racy_native_cache_invalidation_cannot_hide_unchanged_map(self):
        rng = random.Random(20261019)
        for _ in range(100000):
            index_signature = rng.getrandbits(64)
            changed = rng.randrange(2) != 0
            live_signature = rng.getrandbits(64) if changed else index_signature
            paint_known = rng.randrange(2) != 0
            paint_dirty = rng.randrange(2) != 0
            paint_signature = rng.getrandbits(64) if rng.randrange(2) else index_signature
            allowed = may_use_index(
                index_signature, paint_known, paint_dirty, paint_signature,
                live_signature)
            if not changed:
                self.assertTrue(allowed)
            if changed and (paint_dirty or (paint_known and paint_signature != index_signature)):
                self.assertFalse(allowed)


if __name__ == "__main__":
    unittest.main()
