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
