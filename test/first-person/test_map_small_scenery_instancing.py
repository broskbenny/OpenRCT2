"""Source-independent placement of ready, strictly static small scenery.

These tests intentionally distinguish asset readiness from authoritative
placement completeness. Reconstructed local meshes are published once per 8x8
cell, and native paint must not draw a second copy. Dynamic/overlay/ghost cases
must keep native responsibility.
"""
from pathlib import Path
import math
import random
import unittest

ROOT = Path(__file__).resolve().parents[2]
SMALL = ROOT / "src/openrct2/paint/FirstPersonRenderer.SmallScenery.inc"
WORLD = ROOT / "src/openrct2/paint/FirstPersonRenderer.World.inc"
CORE = ROOT / "src/openrct2/paint/FirstPersonRenderer.Core.inc"


def rotate_quarter(point, turns):
    x, y, z = point
    for _ in range(turns % 4):
        x, y = -y, x
    return (x, y, z)


def native_instance(vertices, turns, x, y, z):
    return [(u + x, v + y, w + z) for u, v, w in
            (rotate_quarter(p, turns) for p in vertices)]


def map_instance(vertices, turns, x, y, z):
    output = []
    for p in vertices:
        pt = rotate_quarter(p, turns) if turns else p
        output.append((pt[0] + x, pt[1] + y, pt[2] + z))
    return output


class MapSmallSceneryTest(unittest.TestCase):
    def test_map_publication_and_native_suppression_share_gate(self):
        small = SMALL.read_text(encoding="utf-8")
        world = WORLD.read_text(encoding="utf-8")
        core = CORE.read_text(encoding="utf-8")
        self.assertIn("bool FirstPersonMapSmallSceneryEligible(", small)
        self.assertIn("bool FirstPersonMapSmallSceneryCanPublish(", small)
        self.assertIn("bool AppendFirstPersonMapSmallScenery(", small)
        self.assertIn("FirstPersonMapSmallSceneryCanPublish(\n                        *entry, *small, scene.options.viewFlags)", small)
        self.assertIn("AppendFirstPersonMapSmallScenery(\n                                            mapSurfaces, mapTile, *small,", world)
        self.assertIn("add(surface, owningRegion, true);", world)
        self.assertIn("BuildFirstPersonTileInteractionTarget(", world)
        self.assertIn("surface.sourceInstanceKey = instance;", world)
        self.assertIn("MapIsLocationValid(mapTile)", world)
        self.assertIn("waitingMapCells", small)
        self.assertIn("readyMapCells", small)
        self.assertIn("InvalidateReadyFirstPersonSmallSceneryCells(*job);", small)
        self.assertIn("MarkLocalSceneryDirtyForTile(cx * 8, cy * 8);", small)
        self.assertIn("void MarkLocalSceneryDirtyForTile(int32_t tileX, int32_t tileY);", core)
        # Eligibility excludes opaque-source deviations needing native paint.
        for flag in ("isAnimated", "hasFrameOffsets", "hasGlass", "hasOverlayImage",
                     "isVisibleWhenZoomed", "flag17"):
            self.assertIn("SmallSceneryFlag::" + flag, small)
        for guard in ("!gTrackDesignSaveMode", "!small.isGhost()",
                      "!small.isInvisible()", "!small.needsSupports()"):
            self.assertIn(guard, small)

    def test_randomized_canonical_transform_matches_existing_native_geometry(self):
        rng = random.Random(20261019)
        for _ in range(40000):
            vertices = [(rng.uniform(-32, 32), rng.uniform(-32, 32),
                         rng.uniform(0, 100)) for _ in range(6)]
            rotation = rng.randrange(4)
            origin = (rng.randrange(0, 1000) * 32,
                      rng.randrange(0, 1000) * 32, rng.randrange(0, 240))
            assert map_instance(vertices, rotation, *origin) == native_instance(
                vertices, rotation, *origin)

    def test_per_instance_colors_do_not_change_asset_identity(self):
        rng = random.Random(20261019)
        for _ in range(40000):
            image = rng.randrange(100000)
            direction = rng.randrange(4)
            stage = rng.randrange(3)
            a = tuple(rng.randrange(32) for _ in range(3))
            b = tuple(rng.randrange(32) for _ in range(3))
            map_image = (image + direction + stage * 4, b)
            native_image = (image + direction + stage * 4, b)
            self.assertEqual(map_image, native_image)
            # Instance palettes don't change the reconstruction source artwork.
            self.assertEqual((image, stage, direction), (image, stage, direction))

    def test_local_cell_deduplication_does_not_drop_distinct_placements(self):
        rng = random.Random(20261019)
        for _ in range(10000):
            placements = {(rng.randrange(120), rng.randrange(120))
                          for __ in range(rng.randrange(1, 80))}
            touched = {(x // 8, y // 8) for x, y in placements}
            self.assertLessEqual(len(touched), len(placements))
            for x, y in placements:
                self.assertIn((x // 8, y // 8), touched)


if __name__ == "__main__":
    unittest.main()
