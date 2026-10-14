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
        self.assertGreaterEqual(s.count("if (!gatherBudget.take())\n                                return false;"),2)
        self.assertIn("staging.neighbourSourcesInitialized = true;",s)

    def test_interface_evidence_copy_does_not_duplicate_textures(self):
        s=self.stage
        context=s[s.index("auto& relationshipContext = staging.relationshipContext;"):
                  s.index("// Large occupancy/interface meshes")]
        self.assertIn("FirstPersonSurface geometry{};",context)
        for field in ["cameraIndependent","reconstructedOccupancyBoundary",
                      "artworkCarrier","reconstructionGroup","diagnosticSemanticGroup",
                      "sourceInstanceKey","outwardNormal","triangles"]:
            self.assertIn("geometry."+field+" = source."+field,context)
        self.assertNotIn("geometry.immutablePixels =",context)
        self.assertNotIn("geometry.persistentPixels =",context)

    def test_failed_submission_restores_both_snapshots(self):
        s=self.stage
        self.assertIn("packet.surfaces = std::move(input->surfaces);",s)
        self.assertIn("relationshipContext = std::move(input->context);",s)
        self.assertIn("pending->second.buildToken != token",s)
        self.assertIn("published = std::move(packet);",s)


if __name__ == "__main__":
    unittest.main()
