"""Regression: resident interface context is copied in bounded geometric slices."""
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


class StagedRelationshipContextTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source=(ROOT/"src/openrct2/paint/FirstPersonRenderer.World.inc").read_text()
        cls.stage=source[source.index("struct FirstPersonStaticPacketStaging"):
                         source.index("void SubmitVisibleStaticRegions(")]

    def test_every_neighbour_tile_and_group_surface_is_resumable(self):
        s=self.stage
        self.assertIn("staging.relationshipTile < 128",s)
        self.assertIn("staging.relationshipTileSurface < surfaces.size()",s)
        self.assertIn("staging.neighbourSource < staging.neighbourSources.size()",s)
        self.assertIn("staging.neighbourSourceSurface < surfaces->size()",s)
        self.assertGreaterEqual(s.count("if (!gatherBudget.take())"),3)
        self.assertEqual(s.count("ResumeFirstPersonIndexedRegionSurfaces("), 2)
        self.assertIn("staging.neighbourSourcesInitialized = true;",s)

    def test_interface_evidence_copy_does_not_duplicate_textures(self):
        s=self.stage
        context=s[s.index("auto& relationshipContext = staging.relationshipContext;"):
                  s.index("// Keep nontrivial interface resolution off the frame thread.")]
        self.assertIn("FirstPersonSurface geometry{};",context)
        for field in ["cameraIndependent","reconstructedOccupancyBoundary",
                      "artworkCarrier","reconstructionGroup","diagnosticSemanticGroup",
                      "sourceInstanceKey","outwardNormal","triangles"]:
            self.assertIn("geometry."+field+" = source."+field,context)
        self.assertNotIn("geometry.immutablePixels =",context)
        self.assertNotIn("geometry.persistentPixels =",context)

    def test_unrelated_view_changes_preserve_packet_and_worker(self):
        s=self.stage
        self.assertIn("FirstPersonStagedPacketSelectionChanged(", s)
        self.assertIn("staging.selectedGroups.emplace(", s)
        self.assertIn("relevantSelectionChanged", s)
        self.assertIn("staging.progress.selectionSerial = _reconstructionSelectionSerial", s)
        self.assertIn("FirstPersonStagedPacketSelectionChanged(pending->second)", s)
        world = (ROOT/"src/openrct2/paint/FirstPersonRenderer.World.inc").read_text()
        self.assertIn("FP_REGION_HEALTH", world)
        self.assertNotIn("_reconstructionSelectionSerial != selection", s)

    def test_worker_fingerprints_resolved_mesh_exactly_once(self):
        s = self.stage
        fingerprint = "ResidentStaticSurfaceFingerprint(input->surfaces)"
        self.assertEqual(s.count(fingerprint), 1)
        self.assertLess(s.index("ResolveFirstPersonInternalOccupancyInterfaces(\n"
                                "                            input->surfaces, input->context);"),
                        s.index(fingerprint))
        self.assertLess(s.index(fingerprint),
                        s.index("pendingPacket.surfaces = std::move(input->surfaces);"))
        self.assertIn("pending->second.workerSurfaceFingerprint = fingerprint;", s)
        self.assertIn("pending->second.workerFingerprintReady = true;", s)
        self.assertIn("const uint64_t fingerprint = staging.workerFingerprintReady", s)
        self.assertIn("staging.workerSurfaceFingerprint", s)
        self.assertIn(": ResidentStaticSurfaceFingerprint(packet.surfaces);", s)
        # A map edit, failed source or worker cancellation resets the staging
        # struct, including its fingerprint and all other resume cursors.
        self.assertIn("staging = {};", s)
        self.assertIn("bool workerFingerprintReady = false;", s)

    def test_failed_submission_restores_both_snapshots(self):
        s=self.stage
        self.assertIn("input->source = staging.provisionalSurfaces;",s)
        self.assertIn("_regionRelationshipWorker.hasCapacity()", s)
        self.assertIn("_regionRelationshipWorker.submit(", s)
        renderer = (ROOT/"src/openrct2/paint/FirstPersonRenderer.cpp").read_text()
        self.assertIn("_regionRelationshipWorker.publish(", renderer)
        self.assertIn("_regionRelationshipWorker.cancel();", renderer)
        self.assertIn("pendingPacket.surfaces = std::move(input->surfaces);",s)
        self.assertIn("relationshipContext = std::move(input->context);",s)
        self.assertIn("published.surfaceStorage = staging.provisionalSurfaces;",s)
        self.assertIn("pending->second.buildToken != token",s)
        self.assertIn("published = std::move(packet);",s)
        self.assertIn("staging.boundaryCandidateCount >= 16", s)
        self.assertNotIn("relationshipContext.size() >= 128", s)
        self.assertLess(s.index("published.surfaceStorage = staging.provisionalSurfaces;"),
                        s.index("_regionRelationshipWorker.submit("))
        self.assertIn("input->surfaces = std::move(packet.surfaces);", s)
        self.assertIn("if (input->source)", s)
        self.assertIn("packet.surfaces = std::move(input->surfaces);", s)


if __name__ == "__main__":
    unittest.main()
