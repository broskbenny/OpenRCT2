"""Portable regression tests for exact planar-patch retessellation."""
import os
import unittest

import numpy as np
import trimesh
from retessellate_planar_patches import retessellate


class CoplanarPatchTests(unittest.TestCase):
    def check_invariants(self, original, candidate):
        self.assertTrue(candidate.is_watertight)
        self.assertTrue(candidate.is_winding_consistent)
        self.assertEqual(original.euler_number, candidate.euler_number)
        self.assertAlmostEqual(original.volume, candidate.volume, places=6)
        self.assertLessEqual(len(candidate.faces), len(original.faces))
        self.assertEqual(
            len(original.split(only_watertight=False)),
            len(candidate.split(only_watertight=False)))

    def test_subdivided_box_reduces_coplanar_faces(self):
        mesh = trimesh.creation.box(extents=(10, 12, 8))
        for _ in range(3):
            mesh = mesh.subdivide()
        candidate, stats = retessellate(mesh)
        self.check_invariants(mesh, candidate)
        self.assertLess(len(candidate.faces), len(mesh.faces) * 0.5)
        self.assertGreater(stats["removed_triangles"], 0)

    def test_through_opening_remains_open(self):
        # A connected building with a real vertical through-opening:
        # merging planes must not bridge this hole.
        from skimage.measure import marching_cubes

        occupied = np.zeros((18, 18, 10), dtype=bool)
        occupied[2:16, 2:16, 2:9] = True
        occupied[7:11, 7:11, :] = False
        vertices, faces, _, _ = marching_cubes(
            np.pad(occupied, 1).astype(float), 0.5)
        mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
        self.assertEqual(mesh.euler_number, 0)
        candidate, stats = retessellate(mesh)
        self.check_invariants(mesh, candidate)
        self.assertEqual(candidate.euler_number, 0)
        self.assertLess(len(candidate.faces), len(mesh.faces))

    def test_simple_box_no_false_reduction(self):
        mesh = trimesh.creation.box(extents=(1, 1, 1))
        candidate, _ = retessellate(mesh)
        self.check_invariants(mesh, candidate)
        self.assertEqual(len(candidate.faces), len(mesh.faces))

    def test_non_coplanar_sphere_left_alone(self):
        mesh = trimesh.creation.icosphere(subdivisions=1)
        candidate, _ = retessellate(mesh)
        self.check_invariants(mesh, candidate)
        self.assertEqual(len(candidate.faces), len(mesh.faces))

    def test_private_real_asset_when_provided(self):
        source = os.getenv("SMH2_RAW_HULL_MESH")
        if not source:
            self.skipTest("Private fixture: set SMH2_RAW_HULL_MESH")
        mesh = trimesh.load(source, force="mesh")
        candidate, stats = retessellate(mesh)
        self.check_invariants(mesh, candidate)
        self.assertLess(len(candidate.faces), len(mesh.faces) * 0.7)


if __name__ == "__main__":
    unittest.main()
