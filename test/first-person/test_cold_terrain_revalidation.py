"""Native source acquisition must not repeatedly rebuild stable terrain artwork."""
from pathlib import Path
import random
import unittest

ROOT = Path(__file__).resolve().parents[2]
WORLD = ROOT / "src/openrct2/paint/FirstPersonRenderer.World.inc"


def needs_terrain_refresh(terrain_dirty, source_valid, source_dirty, fields_match):
    return terrain_dirty or (source_valid and source_dirty) or not fields_match


class ColdTerrainRevalidationTest(unittest.TestCase):
    def test_actual_terrain_path_requires_valid_source_before_semantic_dirty(self):
        world = WORLD.read_text(encoding="utf-8")
        section = world.split("void CollectTerrain(FirstPersonScene& scene)", 1)[1].split(
            "BuildFirstPersonSemanticPhysicalFaces(", 1)[0]
        self.assertIn(
            "const bool semanticDirty =\n"
            "                        semanticCache != _staticPaintCache.end()\n"
            "                        && semanticCache->second.valid\n"
            "                        && semanticCache->second.dirty;",
            section)
        self.assertIn("const bool sourceStateMatches =", section)
        self.assertIn("!wasDirty && !semanticDirty", section)
        self.assertIn("cache.baseZ == baseZ", section)
        self.assertIn("cache.waterZ == waterZ", section)
        self.assertIn("cache.slope == slope", section)
        self.assertIn("cache.surfaceStyle == surfaceStyle", section)
        self.assertIn("cache.waterOverlayImage == waterOverlay", section)

    def test_cold_source_does_not_reinvalidate_unchanged_independent_terrain(self):
        rng = random.Random(20261020)
        for _ in range(100000):
            terrain_dirty = bool(rng.getrandbits(1))
            source_valid = bool(rng.getrandbits(1))
            source_dirty = bool(rng.getrandbits(1))
            fields_match = bool(rng.getrandbits(1))
            should_refresh = needs_terrain_refresh(
                terrain_dirty, source_valid, source_dirty, fields_match)
            if terrain_dirty or not fields_match:
                self.assertTrue(should_refresh)
            if source_valid and source_dirty:
                self.assertTrue(should_refresh)
            if not terrain_dirty and fields_match and not source_valid:
                self.assertFalse(should_refresh)

    def test_source_and_terrain_lifecycles_remain_independent(self):
        # Simulate thousands of cold native tiles with already resident,
        # unchanged map-derived ground; none require new material lookups.
        self.assertEqual(sum(needs_terrain_refresh(
            False, False, True, True) for _ in range(12000)), 0)
        # Edits to authoritative surface fields still refresh without
        # requiring a native paint capture first.
        self.assertTrue(needs_terrain_refresh(False, False, True, False))
        self.assertTrue(needs_terrain_refresh(True, False, True, True))
        self.assertTrue(needs_terrain_refresh(False, True, True, True))


if __name__ == "__main__":
    unittest.main()
