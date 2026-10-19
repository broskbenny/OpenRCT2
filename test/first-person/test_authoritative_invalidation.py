"""Distinguish native redraw notifications from changes to map-owned geometry."""
from pathlib import Path
import random
import unittest

ROOT = Path(__file__).resolve().parents[2]
MAIN = ROOT / "src/openrct2/paint/FirstPersonRenderer.cpp"
CORE = ROOT / "src/openrct2/paint/FirstPersonRenderer.Core.inc"


def dirty_regions(tiles, cached_signatures, live_signatures):
    changed = {(x, y) for x, y in tiles
               if (x, y) not in cached_signatures
               or cached_signatures[(x, y)] != live_signatures[(x, y)]}
    regions = {(x // 32, y // 32) for x, y in changed}
    return changed, regions


def request_affected(request, changed):
    rx, ry = request
    return any(abs(rx - tx) <= 16 and abs(ry - ty) <= 16
               for tx, ty in changed)


def group_changed(known_signatures, live_signatures):
    return not known_signatures or any(
        live_signatures[tile] != signature
        for tile, signature in known_signatures.items())


class AuthoritativeInvalidationTest(unittest.TestCase):
    def test_source_only_changes_trigger_structural_residency(self):
        main = MAIN.read_text(encoding="utf-8")
        core = CORE.read_text(encoding="utf-8")
        begin = main.index("void InvalidateFirstPersonSceneRegion(")
        end = main.index("void InvalidateFirstPersonSceneTile(", begin)
        implementation = main[begin:end]
        self.assertIn("std::unordered_set<uint64_t> changedTiles;", implementation)
        self.assertIn("std::unordered_set<uint64_t> changedRegions;", implementation)
        self.assertIn("NativeTileSignature({tx * kCoordsXYStep,", implementation)
        self.assertIn("for (const uint64_t regionKey : changedRegions)", implementation)
        self.assertIn("const bool authoritativeChange = changedTiles.contains(key);", implementation)
        self.assertIn("if (authoritativeChange)", implementation)
        self.assertIn("QueueFirstPersonParkSceneryTile(tx, ty);", implementation)
        self.assertIn("const bool affected = std::any_of(", implementation)
        self.assertIn("changedTiles.begin(), changedTiles.end()", implementation)
        self.assertIn("const bool objectChanged =", implementation)
        self.assertIn("geometry.tileSignatures.empty()", implementation)
        self.assertIn("WithdrawFirstPersonLargeSceneryPhysicalProxies(groupKey);", implementation)
        self.assertNotIn("ClearFirstPersonLargeSceneryPhysicalProxies();", implementation)
        self.assertIn("std::vector<std::pair<CoordsXY, uint64_t>> tileSignatures;", core)
        self.assertIn("result.tileSignatures = job->tileSignatures;", core)

    def test_generic_redraws_preserve_gpu_and_collision_work(self):
        rng = random.Random(20261019)
        for _ in range(10000):
            count = rng.randrange(1, 50)
            tiles = {(rng.randrange(256), rng.randrange(256)) for _ in range(count)}
            signatures = {tile: rng.getrandbits(64) for tile in tiles}
            changed, regions = dirty_regions(tiles, signatures, signatures)
            self.assertFalse(changed)
            self.assertFalse(regions)
            self.assertFalse(request_affected((rng.randrange(256), rng.randrange(256)), changed))
            self.assertFalse(group_changed(signatures, signatures))

    def test_missing_or_changed_tile_invalidates_only_dependent_region(self):
        rng = random.Random(20261019)
        for _ in range(40000):
            count = rng.randrange(1, 100)
            tiles = {(rng.randrange(256), rng.randrange(256)) for _ in range(count)}
            signatures = {tile: rng.getrandbits(64) for tile in tiles}
            changed_item = rng.choice(tuple(tiles))
            live = dict(signatures)
            live[changed_item] ^= 1
            changed, regions = dirty_regions(tiles, signatures, live)
            self.assertEqual(changed, {changed_item})
            self.assertEqual(regions, {(changed_item[0] // 32, changed_item[1] // 32)})
            self.assertTrue(request_affected(changed_item, changed))
            self.assertTrue(group_changed({changed_item: signatures[changed_item]}, live))
            missing = dict(signatures)
            missing.pop(changed_item)
            changed, regions = dirty_regions(tiles, missing, signatures)
            self.assertEqual(changed, {changed_item})
            self.assertEqual(len(regions), 1)


if __name__ == "__main__":
    unittest.main()
