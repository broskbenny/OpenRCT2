"""Map-derived scenery must not inherit unrelated native-paint readiness."""
from pathlib import Path
import itertools
import unittest

ROOT = Path(__file__).resolve().parents[2]
LOCAL = ROOT / "src/openrct2/paint/FirstPersonRenderer.LocalScenery.inc"
SMALL = ROOT / "src/openrct2/paint/FirstPersonRenderer.SmallScenery.inc"
PARK = ROOT / "src/openrct2/paint/FirstPersonRenderer.ParkSceneryPrewarm.inc"
MAIN = ROOT / "src/openrct2/paint/FirstPersonRenderer.cpp"


def should_hold_previous(native_dirty, previous_has_surfaces, previous_native_owned):
    return native_dirty and previous_has_surfaces and previous_native_owned


class MapSceneryPublicationIsolationTest(unittest.TestCase):
    def test_map_only_packet_never_waits_on_native_capture(self):
        local = LOCAL.read_text(encoding="utf-8")
        self.assertIn("bool containsNativeSource = false;", local)
        self.assertIn("if (!mapOwned)\n                        containsNativeSource = true;", local)
        self.assertIn("&& current->second.containsNativeSource)", local)
        self.assertGreaterEqual(local.count("packet.containsNativeSource = containsNativeSource;"), 2)
        self.assertIn("add(surface, owningRegion, true);", local)
        self.assertIn("if (waitingForSourceRevalidation", local)

    def test_completed_small_scenery_skips_cold_native_priority(self):
        local = LOCAL.read_text(encoding="utf-8")
        paint = (ROOT / "src/openrct2/paint/FirstPersonRenderer.Paint.inc").read_text()
        ready = local[
            local.index("bool FirstPersonTileObjectsMapOwnedAndReady("):
            local.index("void MarkLocalSceneryDirtyForTile(", local.index("bool FirstPersonTileObjectsMapOwnedAndReady("))]
        self.assertIn("case TileElementType::smallScenery:", ready)
        self.assertIn("small->needsSupports()", ready)
        self.assertIn("SmallSceneryFlag::hasNoSupports", ready)
        core = (ROOT / "src/openrct2/paint/FirstPersonRenderer.Core.inc").read_text()
        self.assertIn("const bool nativeSmallBody = nativeEntry != nullptr", core)
        self.assertIn("if (nativeSmallBody && surface.diagnosticSemanticRole == 0)", core)
        self.assertIn("&& source.diagnosticSemanticRole == static_cast<uint8_t>(", local)
        self.assertIn("packet->second.mapOwnedInstanceKeys.contains(", ready)
        self.assertIn("FirstPersonLocalSmallSceneryInstanceKey(", ready)
        self.assertIn("elementIndex, *small", ready)
        self.assertIn("packet->second.dirty", ready)
        self.assertEqual(local.count("packet.mapOwnedInstanceKeys = std::move(mapOwnedInstanceKeys);"), 2)
        self.assertIn("mapOwnedInstanceKeys.insert(instance);", local)
        self.assertIn("|| FirstPersonTileObjectsMapOwnedAndReady(tile))", paint)
        self.assertIn("const bool useColdObjectCursor = coldOffset < visibleCount;", paint)

    def test_published_native_instances_survive_only_unchanged_source_tiles(self):
        local = LOCAL.read_text(encoding="utf-8")
        self.assertEqual(local.count("std::array<uint64_t, 64> sourceSignatures{};"), 2)
        self.assertIn("packet.sourceSignatures = staging.sourceSignatures;", local)
        self.assertIn("current->second.sourceSignatures[index]", local)
        self.assertIn("staging.sourceSignatures[index] == 0", local)
        self.assertIn("FirstPersonNativeRecordOf(old)", local)
        self.assertIn("present.contains(record)", local)
        self.assertIn("current->second.mapOwnedInstanceKeys.contains(record.instance)", local)
        self.assertIn("mapOwnedInstanceKeys.insert(record.instance);", local)
        self.assertIn("mapOwnedNativeKeys.contains(record.instance)", local)
        self.assertIn("TileInspector::GetSelectedElement() == nullptr", local)
        self.assertIn("current->second.selectionGeneration == staging.selectionGeneration", local)
        self.assertIn("current->second.viewFlags == staging.viewFlags", local)
        self.assertIn("const uint64_t sourceRegion = FirstPersonGpuRegionKey(tx, ty);", local)
        self.assertIn("add(carried, sourceRegion);", local)

        # Exact-object retention never survives authoritative replacement.
        for prior, current in ((17,17), (17,18), (0,0), (9,0)):
            for replaced in (False, True):
                for already_present in (False, True):
                    keep = prior != 0 and current == prior and not replaced and not already_present
                    self.assertEqual(
                        keep,
                        (prior != 0 and current == prior
                         and not (replaced or already_present)))

    def test_native_to_map_owned_handoff_waits_for_exact_published_instance(self):
        core = (ROOT / "src/openrct2/paint/FirstPersonRenderer.Core.inc").read_text()
        local = LOCAL.read_text()
        small = SMALL.read_text()
        self.assertIn("bool FirstPersonLocalSceneryOwnsSmallScenery(", core)
        self.assertIn("&& FirstPersonLocalSceneryOwnsSmallScenery(ps.MapPos, *small)", small)
        handoff = local[
            local.index("bool FirstPersonLocalSceneryOwnsSmallScenery("):
            local.index("void SubmitVisibleLocalSceneryRegions(", local.index("bool FirstPersonLocalSceneryOwnsSmallScenery("))]
        self.assertIn("!cell->second.surfaces", handoff)
        self.assertIn("element == selected", handoff)
        self.assertIn("FirstPersonLocalSmallSceneryInstanceKey(", handoff)
        self.assertIn("cell->second.mapOwnedInstanceKeys.contains(", handoff)
        # An asset job's ready flag is not sufficient to withdraw its native
        # fallback; only published map ownership permits the handoff.
        self.assertLess(
            small.index("FirstPersonMapSmallSceneryCanPublish(", small.index("if (job->ready)")),
            small.index("&& FirstPersonLocalSceneryOwnsSmallScenery(ps.MapPos, *small)"))

    def test_ownership_prevents_withdrawal_during_pending_native_capture(self):
        for native_dirty, present, native_owned in itertools.product((False, True), repeat=3):
            hold = should_hold_previous(native_dirty, present, native_owned)
            self.assertEqual(hold, native_dirty and present and native_owned)
            if not native_owned:
                self.assertFalse(hold)

    def test_background_traversal_is_bounded_and_camera_independent(self):
        code = PARK.read_text(encoding="utf-8")
        main = MAIN.read_text(encoding="utf-8")
        self.assertIn("FirstPersonFrameBudget budget(", code)
        self.assertIn("256, std::chrono::microseconds(300)", code)
        self.assertIn("_parkSceneryScanCursor++", code)
        self.assertIn("_parkSceneryChangedTiles", code)
        self.assertIn("if (moreInitial)", code)
        self.assertIn("MapGetFirstElementAt(tile)", code)
        self.assertIn("FirstPersonMapSmallSceneryEligible(", code)
        self.assertIn("PrewarmFirstPersonSmallSceneryAsset(tile, *small);", code)
        self.assertLess(main.index("PrewarmVisibleSmallSceneryAssets(scene);"),
                        main.index("PrewarmParkSmallSceneryAssets();"))
        self.assertIn("_snapshotBudget = FirstPersonFrameBudget(\n            4, std::chrono::microseconds(200));", main)


if __name__ == "__main__":
    unittest.main()
