"""Structural object discovery is independent of native artwork readiness.

An authoritative map snapshot drives large-scenery identity and existence.
Stale cached instances must be validated against live TileElements before
activating their old GPU geometry.
"""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
WORLD = ROOT / "src/openrct2/paint/FirstPersonRenderer.World.inc"


class StructuralDiscoveryTest(unittest.TestCase):
    def test_authoritative_tiles_admit_before_native_visibility(self):
        source = WORLD.read_text(encoding="utf-8")
        begin = source.index("void UpdateAuthoritativeLargeScenery(")
        end = source.index("// Terrain publication is deliberately independent", begin)
        block = source[begin:end]
        self.assertIn("NativeTileSignature(tile)", block)
        self.assertIn("MapGetFirstElementAt(tile)", block)
        self.assertIn("GetReconstructionGroup(tile, element)", block)
        self.assertIn("FirstPersonFrameBudget scanBudget(", block)
        self.assertIn("256, std::chrono::microseconds(1000)", block)
        scan = block[:block.index("cursor += examined;")]
        self.assertNotIn("paint->second.visibilityBoundValid", scan)
        self.assertNotIn("paint == _staticPaintCache.end()", scan)

    def test_stale_group_is_validated_before_cached_geometry_activation(self):
        source = WORLD.read_text(encoding="utf-8")
        block = source[source.index("void UpdateAuthoritativeLargeScenery("):
                       source.index("// Terrain publication is deliberately independent",
                                    source.index("void UpdateAuthoritativeLargeScenery("))]
        validate = block.index("MapGetFirstElementAt(group.sourceTile)")
        resolve = block.index("current->key == group.key")
        reject = block.index("if (large == nullptr)")
        activate = block.index("ActivateLargeSceneryRegions(ready->second)")
        self.assertLess(validate, resolve)
        self.assertLess(resolve, reject)
        self.assertLess(reject, activate)
        self.assertIn("paint->second.visibilityDirty", block)
        self.assertIn("paint->second.visibilitySignature", block)


if __name__ == "__main__":
    unittest.main()
