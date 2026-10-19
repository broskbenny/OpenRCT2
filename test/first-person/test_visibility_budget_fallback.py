"""Unknown authoritative tiles must remain drawable when metadata budget expires."""
import math
import random
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
WORLD = ROOT / "src/openrct2/paint/FirstPersonRenderer.World.inc"


class VisibilityBudgetFallbackTest(unittest.TestCase):
    def test_unknown_never_means_empty(self):
        source = WORLD.read_text(encoding="utf-8")
        region = source[source.index("FirstPersonRegionBounds CachedRegionBounds("):
                        source.index("std::optional<FirstPersonSemanticSphere> CachedTileVisibilityBounds(")]
        tile = source[source.index("std::optional<FirstPersonSemanticSphere> CachedTileVisibilityBounds("):
                      source.index("void DiscoverVisibleTiles(")]
        self.assertIn("if (!known && !_discoveryBudget.available())", region)
        before_entry = region[:region.index("auto& entry = _regionBounds[key]")]
        self.assertNotIn("return {{}, {}, false}", before_entry)
        self.assertIn("2304.0f + kHalo", before_entry)
        self.assertIn("true", before_entry)
        self.assertIn("if (!_staticPaintCache.contains(key) && !_discoveryBudget.available())", tile)
        self.assertIn("return unknown();", tile)
        self.assertIn("cached.visibilityKnown ? std::nullopt", tile)
        self.assertIn("ReadTileSemanticSnapshot(world)", tile)

    def test_conservative_tile_bounds_contain_normal_valid_height_range(self):
        rng = random.Random(20261019)
        k_half_tile, halo = 16, 256
        k_low, k_high = -512, 4096
        z_middle = (k_low + k_high) / 2
        radius = math.sqrt(2 * k_half_tile * k_half_tile +
                           ((k_high-k_low)/2)**2) + halo
        for _ in range(100000):
            x, y = rng.uniform(-16, 16), rng.uniform(-16, 16)
            z = rng.uniform(k_low, k_high)
            self.assertLessEqual(
                math.sqrt(x*x+y*y+(z-z_middle)**2), radius)
        self.assertGreater(radius, (k_high-k_low)/2)
        self.assertGreater(halo, 0)

    def test_region_fallback_encloses_known_world_tiles(self):
        rng = random.Random(20261019)
        for _ in range(50000):
            w, h = rng.randrange(1, 17), rng.randrange(1, 17)
            dx, dy = (w*32)/2 + 512, (h*32)/2 + 512
            dz = 2304 + 512
            x = rng.uniform(-w*16, w*16)
            y = rng.uniform(-h*16, h*16)
            z = rng.uniform(-512, 4096)
            self.assertLessEqual(abs(x), dx)
            self.assertLessEqual(abs(y), dy)
            self.assertLessEqual(abs(z - 1792), dz)


if __name__ == "__main__":
    unittest.main()
