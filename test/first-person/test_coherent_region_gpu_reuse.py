"""Guard no-op GPU publication of resolved coherent first-person packets.

A new authoritative source revision can produce identical resolved geometry:
the published GPU buffer must remain reusable, and its selected-reconstruction
dependencies must still advance. Changed geometry or dependencies must upload.
"""
from pathlib import Path
import unittest
import random

ROOT = Path(__file__).resolve().parents[2]
WORLD = ROOT / "src/openrct2/paint/FirstPersonRenderer.World.inc"
CORE = ROOT / "src/openrct2/paint/FirstPersonRenderer.Core.inc"


def publication(current, built, missed):
    """Model the immutable publication decision using the C++ predicates."""
    if (current is not None and current["fingerprint"] == built["fingerprint"] and
            current["materials"] == built["materials"]):
        return {
            "generation": current["generation"],
            "gpu_revision": current["gpu_revision"],
            "published_source_revision": built["source_revision"],
            "picking_target": built["picking_target"],
            "fingerprint": current["fingerprint"],
            "materials": current["materials"],
            "selection_stamp": built["selection_stamp"],
            "dirty": missed,
        }
    return {
        "generation": current["generation"] + 1 if current else 1,
        "gpu_revision": built["source_revision"],
        "published_source_revision": built["source_revision"],
        "picking_target": built["picking_target"],
        "fingerprint": built["fingerprint"],
        "materials": built["materials"],
        "selection_stamp": built["selection_stamp"],
        "dirty": missed,
    }


class CoherentRegionReuseTest(unittest.TestCase):
    def test_renderer_publishes_unchanged_buffers_without_generation_bump(self):
        world = WORLD.read_text(encoding="utf-8")
        core = CORE.read_text(encoding="utf-8")
        self.assertIn("uint64_t surfaceFingerprint{};", core)
        self.assertIn("fingerprint, surface.nativePaintOrdinal", core)
        self.assertIn("ResidentStaticSurfaceFingerprint(packet.surfaces)", world)
        self.assertIn("published.surfaceFingerprint == fingerprint", world)
        self.assertIn("published.textureDependencies == packet.textureDependencies", world)
        self.assertIn("if (sameGpuOutput)", world)
        no_op = world[world.index("if (sameGpuOutput)"):
                      world.index("packet.surfaceFingerprint = fingerprint;")]
        self.assertNotIn("++published.generation", no_op)
        self.assertIn("published.surfaceStorage =", no_op)
        self.assertIn("published.publishedSourceRevision =", no_op)
        self.assertNotIn("published.generation =", no_op)
        self.assertIn("published.reconstructionSelectionStamp", no_op)
        self.assertIn("published.dirty = missedSourceChanges;", no_op)
        self.assertIn("_staticRegionPacketBuilds.erase(regionKey)", no_op)
        self.assertIn("packet.generation = published.generation + 1;", world)

    def test_randomized_publication_identity_and_invalidation(self):
        rng = random.Random(20261019)
        for _ in range(75000):
            old = {
                "generation": rng.randrange(1, 100000),
                "gpu_revision": rng.randrange(1, 5000),
                "fingerprint": rng.getrandbits(64),
                "materials": tuple(sorted({rng.randrange(200) for _ in range(6)})),
                "selection_stamp": rng.getrandbits(64),
                "picking_target": rng.getrandbits(64),
            }
            same = rng.random() < 0.75
            built = {
                "source_revision": rng.randrange(1, 5000),
                "fingerprint": old["fingerprint"] if same else rng.getrandbits(64),
                "materials": old["materials"],
                "selection_stamp": rng.getrandbits(64),
                "picking_target": rng.getrandbits(64),
            }
            if rng.random() < 0.1:
                built["materials"] = old["materials"] + (201,)
                same = False
            missed = rng.choice((True, False))
            out = publication(old, built, missed)
            self.assertEqual(out["generation"], old["generation"] + (0 if same else 1))
            self.assertEqual(out["gpu_revision"],
                             old["gpu_revision"] if same else built["source_revision"])
            self.assertEqual(out["selection_stamp"], built["selection_stamp"])
            self.assertEqual(out["picking_target"], built["picking_target"])
            self.assertEqual(out["published_source_revision"], built["source_revision"])
            self.assertEqual(out["dirty"], missed)


if __name__ == "__main__":
    unittest.main()
