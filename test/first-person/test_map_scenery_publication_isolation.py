"""Map-derived scenery must not inherit unrelated native-paint readiness."""
from pathlib import Path
import itertools
import unittest

ROOT = Path(__file__).resolve().parents[2]
LOCAL = ROOT / "src/openrct2/paint/FirstPersonRenderer.LocalScenery.inc"
SMALL = ROOT / "src/openrct2/paint/FirstPersonRenderer.SmallScenery.inc"
PARK = ROOT / "src/openrct2/paint/FirstPersonRenderer.ParkSceneryPrewarm.inc"
MAIN = ROOT / "src/openrct2/paint/FirstPersonRenderer.cpp"
CONVERGENCE = ROOT / "src/openrct2/paint/FirstPersonRenderer.WorldConvergence.inc"


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
        self.assertIn("surface.diagnosticSourceTile = mapTile;", local)
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

    def test_streamed_native_fallback_respects_exact_gpu_publication(self):
        paint = (ROOT / "src/openrct2/paint/FirstPersonRenderer.Paint.inc").read_text()
        streamed = paint[paint.index("// Native group source selection examines all faces"):]
        self.assertIn("mapOwnedNativeInstances.try_emplace(record, false)", paint)
        self.assertIn("FirstPersonNativeInstanceMapOwned(tile, record)", paint)
        self.assertIn("record.role != static_cast<uint8_t>(", paint)
        self.assertIn("FirstPersonPaintSemanticRole::smallSceneryBody", paint)
        self.assertIn("if (hasPublishedMapBody(staticSurface))", streamed)
        self.assertIn("if (hasPublishedMapBody(staticSurface))\n                                continue;", streamed)
        self.assertLess(
            paint.index("if (hasPublishedMapBody(staticSurface))"),
            paint.index("ReorientBillboard(surface, opt.camera.position, basis.right)"))
        self.assertLess(
            paint.index("hasPublishedMapBody ="),
            paint.index("cached.cameraIndependentStreamedSurfaces)"))

    def test_pending_map_mesh_remains_published_owner(self):
        local = LOCAL.read_text(encoding="utf-8")
        pending = local[local.index("else if (FirstPersonMapSmallSceneryPending("):
                        local.index("++elementIndex;", local.index("else if (FirstPersonMapSmallSceneryPending("))]
        self.assertIn("staging.sourceSignatures[cellIndex] != 0", pending)
        self.assertIn("previousCell->second.sourceSignatures[cellIndex]", pending)
        self.assertIn("previousCell->second.viewFlags == staging.viewFlags", pending)
        self.assertIn("previousCell->second.paintWideGhostMode", pending)
        self.assertIn("bool carriedBody = false;", pending)
        self.assertIn("if (carriedBody)\n                                            markPublishedBody();", pending)
        self.assertIn("mapOwnedInstanceKeys.insert(instance);", local)
        self.assertIn("mapOwnedNativeKeys.insert(nativeKey == 0 ? 1 : nativeKey);", local)

    def test_ownership_prevents_withdrawal_during_pending_native_capture(self):
        for native_dirty, present, native_owned in itertools.product((False, True), repeat=3):
            hold = should_hold_previous(native_dirty, present, native_owned)
            self.assertEqual(hold, native_dirty and present and native_owned)
            if not native_owned:
                self.assertFalse(hold)


    def test_world_inventory_is_authoritative_and_edit_reconciled(self):
        convergence = CONVERGENCE.read_text(encoding="utf-8")
        park = PARK.read_text(encoding="utf-8")
        main = MAIN.read_text(encoding="utf-8")
        self.assertIn('#include "FirstPersonRenderer.WorldConvergence.inc"', main)
        self.assertLess(
            main.index('#include "FirstPersonRenderer.World.inc"'),
            main.index('#include "FirstPersonRenderer.WorldConvergence.inc"'))
        self.assertLess(
            main.index('#include "FirstPersonRenderer.WorldConvergence.inc"'),
            main.index('#include "FirstPersonRenderer.ParkSceneryPrewarm.inc"'))
        self.assertIn("ClearFirstPersonWorldInventory();", main)
        self.assertIn("EnsureFirstPersonWorldInventoryEpoch()", convergence)
        self.assertIn("_worldInventoryObjectCount -= it->second.objects.size();", convergence)
        self.assertIn("object.nativeSourceKey = FirstPersonWorldNativeSourceKey(", convergence)
        self.assertIn("object.identity = FirstPersonLocalSmallSceneryInstanceKey(", convergence)
        self.assertIn("ExtendStableKey(identity, next.sourceSignature);", convergence)
        self.assertIn("!element->isGhost() && !element->isInvisible()", convergence)
        self.assertIn("InvalidateFirstPersonWorldInventoryTile(tx, ty);", park)
        self.assertIn("ObserveFirstPersonWorldInventoryTile(tile);", park)
        self.assertLess(
            park.index("ObserveFirstPersonWorldInventoryTile(tile);"),
            park.index("PrewarmFirstPersonSmallSceneryAsset(tile, *small);"))

    def test_unrepresented_exact_small_object_gets_only_untried_capture(self):
        convergence = CONVERGENCE.read_text(encoding="utf-8")
        paint = (ROOT / "src/openrct2/paint/FirstPersonRenderer.Paint.inc").read_text(
            encoding="utf-8")
        self.assertIn("FirstPersonLocalSceneryOwnsSmallScenery(tile, *small)", convergence)
        self.assertIn("inventory->second.sourceSignature == cached.signature", convergence)
        self.assertIn("object.nativeSourceKey", convergence)
        self.assertIn("object.identity", convergence)
        self.assertIn("return chooseUnobserved();", convergence)
        self.assertIn("A changed or not-yet-inventoried tile falls back", convergence)
        self.assertIn("FirstPersonNativeSourceContainsInstance(", convergence)
        self.assertIn("source.sourceInstanceKey == nativeSourceKey", convergence)
        self.assertIn("FirstPersonPaintSemanticRole::smallSceneryBody", convergence)
        self.assertIn("InspectFirstPersonWorldCoverage(", convergence)
        self.assertIn("sampleBudget.take()", convergence)
        self.assertIn("worldCoverage.missingSmall", paint)
        self.assertIn("worldCoverage.exhaustedSmall", paint)
        self.assertIn("!cached.rotations[preferredRotation].valid", convergence)
        self.assertIn("!cached.rotations[rotation].valid", convergence)
        self.assertIn("return std::nullopt;", convergence)
        self.assertGreaterEqual(
            paint.count("FirstPersonMissingSmallSceneryCaptureRotation("), 2)
        self.assertIn("rotationMask |= uint8_t(1u << *missingRotation);", paint)
        self.assertIn("rotationMask |= cached.nativeRetryMask;", paint)
        self.assertIn("size_t nativeCapturesRemaining = 2;", paint)
        self.assertIn("inventoriedObjects=%zu parkScannedTiles=%zu", paint)

        # A new capture is allowed only for an unobserved quarter turn.
        for available in ((False, False, False, False),
                          (True, False, True, True),
                          (True, True, True, True)):
            for preferred in range(4):
                selected = (preferred if not available[preferred] else
                            next((r for r, observed in enumerate(available)
                                  if not observed), None))
                self.assertEqual(selected is None, all(available))

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
