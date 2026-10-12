"""Exercises exact production-compatible C++ connected planar retessellation.

The proprietary DAT/mesh is an optional local-only fixture.
"""
import os
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import unittest

import numpy as np
import trimesh
from skimage.measure import marching_cubes


ROOT = Path(__file__).resolve().parents[2]


def run_cpp(source_mesh, executable, scratch):
    before = Path(scratch) / 'mesh.bin'
    after = Path(scratch) / 'faces.bin'
    vertices = np.asarray(source_mesh.vertices, dtype='<f4')
    faces = np.asarray(source_mesh.faces, dtype='<u4')
    with before.open('wb') as f:
        f.write(struct.pack('<II', len(vertices), len(faces)))
        f.write(vertices.tobytes())
        f.write(faces.tobytes())
    subprocess.run([str(executable), str(before), str(after)], check=True)
    data = np.fromfile(after, dtype='<u4')
    if len(data) != 1 + int(data[0]) * 3:
        raise AssertionError('Malformed C++ output')
    output_faces = data[1:].reshape(-1, 3)
    return trimesh.Trimesh(vertices=source_mesh.vertices.copy(), faces=output_faces, process=True)


class NativePlanarTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if shutil.which('g++') is None:
            raise unittest.SkipTest('g++ unavailable')
        cls.directory = tempfile.TemporaryDirectory(prefix='fp-native-planar-')
        cls.addClassCleanup(cls.directory.cleanup)
        cls.scratch = Path(cls.directory.name)
        cls.executable = cls.scratch / 'native'
        subprocess.run([
            'g++', '-std=c++20', '-O2', '-Wall', '-Wextra', '-Werror',
            '-pedantic', '-fsanitize=undefined', '-fno-sanitize-recover=all',
            '-I', str(ROOT / 'src/openrct2/paint'),
            str(ROOT / 'test/first-person/ConnectedPlanarMeshRunner.cpp'),
            '-o', str(cls.executable)], check=True)

    def verify(self, before, after):
        self.assertTrue(before.is_watertight)
        self.assertTrue(after.is_watertight)
        self.assertTrue(after.is_winding_consistent)
        self.assertEqual(before.euler_number, after.euler_number)
        self.assertAlmostEqual(before.volume, after.volume, places=5)
        self.assertLessEqual(len(after.faces), len(before.faces))
        self.assertEqual(len(after.split(only_watertight=False)),
                         len(before.split(only_watertight=False)))

    def test_dense_subdivided_box(self):
        source = trimesh.creation.box(extents=(16, 12, 8))
        for _ in range(3):
            source = source.subdivide()
        result = run_cpp(source, self.executable, self.scratch)
        self.verify(source, result)
        self.assertLess(len(result.faces), len(source.faces) * 0.6)

    def test_true_open_tunnel_preserved(self):
        occupied = np.zeros((18, 18, 10), dtype=np.uint8)
        occupied[2:16, 2:16, 2:9] = 1
        occupied[7:11, 7:11, :] = 0
        vertices, faces, _, _ = marching_cubes(np.pad(occupied, 1).astype(float), 0.5)
        source = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
        self.assertEqual(source.euler_number, 0)
        result = run_cpp(source, self.executable, self.scratch)
        self.verify(source, result)
        self.assertLess(len(result.faces), len(source.faces))

    def test_rounded_surface_not_falsely_collapsed(self):
        source = trimesh.creation.icosphere(subdivisions=2)
        result = run_cpp(source, self.executable, self.scratch)
        self.verify(source, result)
        self.assertEqual(len(result.faces), len(source.faces))

    def test_real_smh2_when_private_fixture_is_available(self):
        fixture = os.getenv('SMH2_RAW_HULL_MESH')
        if not fixture:
            self.skipTest('Private SMH2 source mesh not installed')
        source = trimesh.load(fixture, force='mesh')
        result = run_cpp(source, self.executable, self.scratch)
        self.verify(source, result)
        self.assertEqual(len(source.faces), 6128)
        self.assertEqual(len(result.faces), 3660)


if __name__ == '__main__':
    unittest.main()
