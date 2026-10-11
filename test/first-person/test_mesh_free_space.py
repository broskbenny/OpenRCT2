"""Synthetic checks: a closed mesh must not invent matter in real openings."""
import unittest
import numpy as np
import trimesh
from check_mesh_free_space import (
    measure, classify_closed_mesh, exterior_connected_empty,
)


class MeshFreeSpaceTests(unittest.TestCase):
    def setUp(self):
        self.source = np.zeros((8, 8, 8), dtype=np.uint8)
        self.source[1:7, 1:7, 1:7] = 1
        self.mesh = trimesh.creation.box(extents=(12, 12, 12))
        self.mesh.apply_translation((8, 8, 8))

    def test_closed_cube_exact(self):
        report = measure(self.mesh, self.source)
        self.assertEqual(report['false_filled_empty_cells'], 0)
        self.assertEqual(report['lost_source_solid_cells'], 0)
        self.assertEqual(report['ambiguous_vertical_rays'], 0)

    def test_sealing_authentic_open_passage_is_rejected(self):
        source = self.source.copy()
        source[3:5, 3:5, 1:5] = 0
        report = measure(self.mesh, source)
        self.assertEqual(report['sealed_exterior_connected_empty_cells'], 16)
        self.assertEqual(report['lost_source_solid_cells'], 0)

    def test_wrong_grid_origin_is_not_a_successful_geometry_match(self):
        report = measure(self.mesh, self.source, origin=(2, 0, 0))
        self.assertGreater(report['false_filled_empty_cells'] +
                           report['lost_source_solid_cells'], 0)

    def test_known_cavity_is_not_exterior_connected(self):
        source = self.source.copy()
        source[3, 3, 3] = 0
        exterior = exterior_connected_empty(source.astype(bool))
        self.assertFalse(exterior[3, 3, 3])
        self.assertTrue(exterior[0, 0, 0])

    def test_open_mesh_cannot_make_volume_claims(self):
        open_mesh = self.mesh.copy()
        open_mesh.update_faces(np.arange(len(open_mesh.faces)-2))
        with self.assertRaises(ValueError):
            classify_closed_mesh(open_mesh, self.source.shape)


if __name__ == '__main__':
    unittest.main()
