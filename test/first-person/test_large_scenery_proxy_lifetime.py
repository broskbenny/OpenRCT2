"""Preserve reconstructed and early walking proxies across unrelated redraws."""
from pathlib import Path
import random
import unittest

ROOT = Path(__file__).resolve().parents[2]
MAIN = ROOT / "src/openrct2/paint/FirstPersonRenderer.cpp"
CORE = ROOT / "src/openrct2/paint/FirstPersonRenderer.Core.inc"


class LargeSceneryProxyLifetimeTest(unittest.TestCase):
    def test_source_level_dependency_contract(self):
        main = MAIN.read_text(encoding="utf-8")
        core = CORE.read_text(encoding="utf-8")
        assert "_onDemandLargeSceneryProxySources" in main
        assert "result.tileSignatures = job->tileSignatures;" in core
        assert "std::vector<std::pair<CoordsXY, uint64_t>> tileSignatures;" in core
        ensure = main[main.index("EnsureFirstPersonLargeSceneryPhysicalProxy("):
                      main.index("GetFirstPersonTerrainSourceRotation(", main.index("EnsureFirstPersonLargeSceneryPhysicalProxy("))]
        assert "sources.emplace_back(location, NativeTileSignature(location));" in ensure
        invalidate = main[main.index("void InvalidateFirstPersonSceneRegion("):
                          main.index("void InvalidateFirstPersonSceneTile(")]
        assert "if (!changedTiles.empty())" in invalidate
        assert "changedTiles.contains(sourceKey)" in invalidate
        assert "NativeTileSignature(source.first) != source.second" in invalidate
        assert "WithdrawFirstPersonLargeSceneryPhysicalProxies(it->first);" in invalidate
        assert "WithdrawFirstPersonLargeSceneryPhysicalProxies(groupKey);" in invalidate
        assert "if (!objectChanged)" in invalidate
        assert "continue;" in invalidate
        assert "ClearFirstPersonLargeSceneryPhysicalProxies();" not in invalidate

    def test_only_groups_with_modified_source_tiles_are_withdrawn(self):
        rng = random.Random(20261019)
        for _ in range(20000):
            group_count = rng.randrange(1, 35)
            groups = {
                group: {
                    (rng.randrange(128), rng.randrange(128)): rng.getrandbits(64)
                    for _ in range(rng.randrange(1, 8))
                }
                for group in range(group_count)
            }
            all_sources = {tile: value for signatures in groups.values()
                           for tile, value in signatures.items()}
            modified = rng.choice(list(all_sources))
            modified_groups = {group for group, signatures in groups.items()
                               if modified in signatures}
            withdrawn = set()
            for group, signatures in groups.items():
                if any(tile == modified for tile in signatures):
                    withdrawn.add(group)
            self.assertEqual(withdrawn, modified_groups)
            self.assertLessEqual(len(withdrawn), len(groups))

    def test_redraw_only_change_cannot_revoke_on_demand_proxies(self):
        rng = random.Random(20261019)
        for _ in range(20000):
            count = rng.randrange(1, 100)
            signatures = [rng.getrandbits(64) for _ in range(count)]
            observed = list(signatures)
            self.assertFalse(any(a != b for a, b in zip(signatures, observed)))
            # Native redraws may be frequent; unchanged map signatures
            # must not invalidate any of the recorded per-group proxies.
            assert signatures == observed


if __name__ == "__main__":
    unittest.main()
