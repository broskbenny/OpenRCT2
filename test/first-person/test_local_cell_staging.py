"""8x8 local source-independent cells advance and publish atomically."""
from pathlib import Path
import random
import unittest

ROOT = Path(__file__).resolve().parents[2]
LOCAL = ROOT / "src/openrct2/paint/FirstPersonRenderer.LocalScenery.inc"
MAIN = ROOT / "src/openrct2/paint/FirstPersonRenderer.cpp"


class LocalCellStagingTests(unittest.TestCase):
    def test_production_staging_has_no_partial_cell_publication(self):
        source = LOCAL.read_text(encoding="utf-8")
        main = MAIN.read_text(encoding="utf-8")
        self.assertIn("struct FirstPersonLocalSceneryStaging", source)
        self.assertIn("std::array<uint64_t, 64> sourceSignatures{}", source)
        self.assertIn("while (staging.nextTile < 64 && tileBudget.take())", source)
        self.assertIn("const size_t cellIndex = staging.nextTile;", source)
        self.assertIn("FirstPersonPacketTileSurfaceCursor tileCursor{}", source)
        self.assertIn("if (staging.tileCursor.phase == 0)", source)
        self.assertIn("ResumeFirstPersonPacketSurfaces(", source)
        self.assertIn("staging.tileCursor.nextTile();", source)
        self.assertIn("++staging.nextTile;", source)
        self.assertLess(source.index("staging.tileCursor.nextPhase();"),
                        source.index("const auto it = _staticPaintCache.find(TerrainKey(tx, ty));"))
        self.assertIn("staging.sourceSignatures[cellIndex] =", source)
        self.assertIn("if (staging.nextTile != 64)\n                    continue;", source)
        self.assertIn("liveSignature != staging.sourceSignatures[index]", source)
        self.assertIn("if (editedDuringBuild)", source)
        self.assertIn("_localSceneryBuilds.erase(key);", source)
        self.assertLess(source.index("if (staging.nextTile != 64)"),
                        source.index("ResidentStaticSurfaceFingerprint(surfaces)"))
        self.assertLess(source.index("if (editedDuringBuild)"),
                        source.index("packet.fingerprint = fingerprint;"))
        self.assertIn("packet.dirty = packet.invalidationSerial != staging.sourceInvalidationSerial;", source)
        self.assertIn("staging.epoch != _sceneEpoch", source)
        self.assertIn("staging.selectionGeneration != oldPacket.selectionGeneration", source)
        self.assertIn("staging.viewFlags != scene.options.viewFlags", source)
        self.assertIn("_localSceneryBuilds.clear();", main)
        self.assertIn("_localSceneryBuilds.clear();", source)

    def test_current_gpu_cell_is_retained_during_replacement(self):
        source = LOCAL.read_text(encoding="utf-8")
        stage = source[source.index("FirstPersonFrameBudget budget(4,"):
                       source.index("std::vector<uint64_t> orderedVisible(")]
        self.assertIn("auto& oldPacket = _localSceneryPackets[key];", stage)
        self.assertNotIn("oldPacket.surfaces.reset()", stage)
        self.assertNotIn("oldPacket.surfaces.clear()", stage)
        self.assertIn("const auto previousCell = _localSceneryPackets.find(key);", stage)
        self.assertIn("packet.surfaces =", stage)
        self.assertIn("std::make_shared<const std::vector<FirstPersonSurface>>", stage)
        self.assertIn("kPausedLocalBuildLimit = 8", source)
        self.assertIn("kLocalBuildGrace = 120", source)
        self.assertIn("if (aStaged != bStaged)", source)

    def test_randomized_partial_snapshot_never_commits_mixed_map_epochs(self):
        rng = random.Random(20261029)
        source = [rng.getrandbits(64) for _ in range(64)]
        committed = []
        rejected = 0
        for _ in range(1200):
            start = [None] * 64
            next_tile = 0
            # An epoch can change while this cell is incomplete.
            while next_tile < 64:
                take = rng.randrange(1, 16)
                for i in range(next_tile, min(64, next_tile + take)):
                    start[i] = source[i]
                next_tile += take
                if rng.randrange(5) == 0:
                    source[rng.randrange(64)] = rng.getrandbits(64)
            if any(stored != live for stored, live in zip(start, source)):
                rejected += 1
            else:
                committed.append(tuple(start))
                self.assertEqual(committed[-1], tuple(source))
        self.assertGreater(rejected, 0)
        self.assertGreater(len(committed), 0)


if __name__ == "__main__":
    unittest.main()
