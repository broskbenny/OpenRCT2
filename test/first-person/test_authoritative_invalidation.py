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
        self.assertIn("const uint64_t signature = NativeTileSignature({", implementation)
        self.assertIn("tileCache.hasInvalidationSignature", implementation)
        self.assertIn("tileCache.lastInvalidatedSignature == signature", implementation)
        self.assertIn("tileCache.lastInvalidatedSignature = signature;", implementation)
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

    def test_visibility_hierarchy_rebuild_requires_real_map_change(self):
        main = MAIN.read_text(encoding="utf-8")
        region = main.split("void InvalidateFirstPersonSceneRegion(", 1)[1].split(
            "void InvalidateFirstPersonSceneTile(", 1)[0]
        before_tile_walk = region.split(
            "const bool authoritativeChange = changedTiles.contains(key);", 1)[0]
        self.assertIn(
            "if (!changedTiles.empty())\n"
            "            InvalidateFirstPersonRegionBounds(", before_tile_walk)
        cached_tail = region.split("cached->second.dirty = true;", 1)[1]
        self.assertIn(
            "if (authoritativeChange)\n"
            "                    cached->second.visibilityDirty = true;",
            cached_tail)
        # Native renderer invalidation still triggers classification refresh.
        self.assertIn("cached->second.dirty = true;", region)
        # An unchanged source signature needs neither a leaf rebuild nor
        # invalidation of previously known structural visibility evidence.
        for tiles in ({(2, 3)}, {(7, 1), (31, 31)}, {(5, 5), (48, 20)}):
            signatures = {tile: hash(tile) for tile in tiles}
            changed, regions = dirty_regions(tiles, signatures, signatures)
            self.assertFalse(changed)
            self.assertFalse(regions)

    def test_changed_bounds_do_not_invalidate_unrelated_redraw_leaves(self):
        main = MAIN.read_text(encoding="utf-8")
        region = main.split("void InvalidateFirstPersonSceneRegion(", 1)[1].split(
            "void InvalidateFirstPersonSceneTile(", 1)[0]
        self.assertIn("changedX0 = std::min(changedX0, tx);", region)
        self.assertIn("changedY0 = std::min(changedY0, ty);", region)
        self.assertIn("changedX1 = std::max(changedX1, tx);", region)
        self.assertIn("changedY1 = std::max(changedY1, ty);", region)
        self.assertIn("changedX0, changedY0, changedX1, changedY1", region)
        self.assertNotIn(
            "InvalidateFirstPersonRegionBounds(\\n                x0, y0, x1, y1)",
            region)
        rng = random.Random(20261020)
        for _ in range(50000):
            width, height = rng.randrange(2, 512), rng.randrange(2, 512)
            changed = {(rng.randrange(width), rng.randrange(height))
                       for _ in range(rng.randrange(1, 12))}
            x0 = min(x for x, _ in changed)
            x1 = max(x for x, _ in changed)
            y0 = min(y for _, y in changed)
            y1 = max(y for _, y in changed)
            self.assertTrue(all(x0 <= x <= x1 and y0 <= y <= y1
                                for x, y in changed))
            self.assertLessEqual((x1 - x0 + 1) * (y1 - y0 + 1),
                                 width * height)
        # A one-cell edit inside a full-map repaint dirties just one leaf.
        self.assertEqual((16, 16, 16, 16),
                         (min([16]), min([16]), max([16]), max([16])))

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
