"""Execute production C++ planar geometry, not a Python reimplementation."""
import shutil
import tempfile
import unittest
from pathlib import Path

import numpy as np
from run_production_planar import compile_production, execute, as_mesh


REPO = Path(__file__).resolve().parents[2]


def roof(heights, width=6):
    shape = np.zeros((len(heights), width, 12), dtype=np.uint8)
    for x, h in enumerate(heights):
        shape[x, :, :h] = 1
    return shape


class ProductionPlanarGeometryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if shutil.which('g++') is None:
            raise unittest.SkipTest('C++ compiler unavailable')
        cls.tmp = tempfile.TemporaryDirectory(prefix='production-planar-test-')
        cls.addClassCleanup(cls.tmp.cleanup)
        cls.executable = Path(cls.tmp.name) / 'production'
        compile_production(REPO, cls.executable)

    def test_simple_roofs_form_closed_meshes(self):
        for heights, expected in [([4]*8, 6), ([3,3,4,4,5,5,6,6], 6),
                                  ([2,3,4,5,6,5,4,3], 10)]:
            result = execute(self.executable, roof(heights), step=2)
            self.assertEqual(len(result['faces']), expected)
            mesh = as_mesh(result)
            self.assertTrue(mesh.is_watertight)
            self.assertTrue(mesh.is_winding_consistent)
            self.assertTrue(np.isfinite(mesh.vertices).all())

    def test_genuine_opening_is_never_sealed_by_solid_fitter(self):
        hollow = roof([8]*8)
        hollow[2:6, 2:4, :5] = 0
        self.assertEqual(execute(self.executable, hollow)['faces'], [])

    def test_missing_footprint_preserves_void(self):
        missing = roof([8]*8)
        missing[2, 2, :] = 0
        self.assertEqual(execute(self.executable, missing)['faces'], [])

    def test_irregular_roof_is_not_forced_into_solid_plane(self):
        ornamental = roof([2, 8, 2, 8, 2, 8, 2, 8])
        self.assertEqual(execute(self.executable, ornamental)['faces'], [])

    def test_nonbinary_occupancy_rejected_before_native_call(self):
        with self.assertRaises(ValueError):
            execute(self.executable, np.array([[[2]]], dtype=np.uint8))

    def test_real_smh2_occupancy_if_explicitly_available(self):
        # Private artwork and reconstructed volumes never enter the repository.
        # Set SMH2_OCCUPANCY_NPY to run this meaningful smoke test locally.
        import os
        path = os.getenv('SMH2_OCCUPANCY_NPY')
        if not path:
            self.skipTest('Set SMH2_OCCUPANCY_NPY for private SMH2 gate')
        occupancy = np.load(path, allow_pickle=False)
        result = execute(self.executable, occupancy)
        self.assertEqual(result['faces'], [],
                         'SMH2 cannot be converted into a simple closed solid')


if __name__ == '__main__':
    unittest.main()
