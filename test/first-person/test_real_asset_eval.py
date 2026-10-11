"""Synthetic, distributable regression tests for the real-DAT evaluation tool."""
import struct
import tempfile
import unittest
from pathlib import Path
import numpy as np

import real_asset_eval as lab


def pack_dat(images):
    payload = bytearray(22)
    payload += b'\xff' + bytes(16)  # language end + scenery-group reference
    for x, y in ((0, 0), (32, 0), (0, 32), (32, 32)):
        payload += struct.pack('<hhhBH', x, y, 0, 32, 0)
    payload += b'\xff\xff'
    binary = bytearray()
    records = bytearray()
    for width, height, x, y, pixels, rle in images:
        start = len(binary)
        if rle:
            # A single end-of-line RLE segment per row.
            binary += struct.pack('<' + 'H' * height,
                                  *(height * 2 + row * (2 + width) for row in range(height)))
            for row in range(height):
                binary += bytes((0x80 | width, 0))
                binary += pixels[row * width:(row + 1) * width]
        else:
            binary += pixels
        records += struct.pack('<IhhhhHH', start, width, height, x, y,
                               4 if rle else 0, 0)
    payload += struct.pack('<II', len(images), len(binary)) + records + binary
    # Use literal Sawyer chunks to avoid tying the test to the production decoder.
    chunks = b''.join(bytes((len(payload[i:i + 128]) - 1,)) + payload[i:i + 128]
                      for i in range(0, len(payload), 128))
    return struct.pack('<I8sI', 0, b'TEST    ', 0) + struct.pack('<BI', 1, len(chunks)) + chunks


class DecoderTests(unittest.TestCase):
    def test_raw_and_row_rle_preserve_palette_indices(self):
        pix = bytes([0, 12, 13, 0, 44, 45, 0, 9])
        fixture = pack_dat([(4, 2, -2, -3, pix, i == 1) for i in range(4)])
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / 'TEST.DAT'
            path.write_bytes(fixture)
            obj = lab.read_large_scenery(path)
        self.assertEqual(obj['name'], 'TEST')
        self.assertEqual(len(obj['tiles']), 4)
        self.assertEqual(len(obj['images']), 4)
        self.assertEqual([im['pixels'] for im in obj['images']], [pix] * 4)
        self.assertEqual([im['opaque'] for im in obj['images']], [5] * 4)
        self.assertEqual(obj['images'][1]['flags'], 4)

    def test_truncated_rle_and_images_raise(self):
        base = pack_dat([(4, 2, 0, 0, bytes([1] * 8), False) for _ in range(4)])
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / 'corrupt.dat'
            path.write_bytes(base[:-1])
            with self.assertRaises(ValueError):
                lab.read_large_scenery(path)
            path.write_bytes(base[:16] + bytes((9,)) + base[17:])
            with self.assertRaises(ValueError):
                lab.read_large_scenery(path)

    def test_project_rotates_native_coordinates(self):
        self.assertEqual(lab.projection(0, 5, 7, 3, 0, 0, 0), (2, 3.0))
        self.assertEqual(lab.projection(1, 5, 7, 3, 0, 0, 0), (-12, -2.0))
        self.assertEqual(lab.projection(2, 5, 7, 3, 0, 0, 0), (-2, -9.0))
        self.assertEqual(lab.projection(3, 5, 7, 3, 0, 0, 0), (12, -4.0))


class GeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import trimesh
            import cv2
        except ImportError as error:
            raise unittest.SkipTest(f'Optional comparison dependency missing: {error}')
        cls.mesh = trimesh.creation.box(extents=(8, 8, 8))
        cls.images = [dict(width=48, height=48, x=-24, y=-24) for _ in range(4)]

    def test_self_projection_is_exact_across_four_rotations(self):
        images = []
        for view, im in enumerate(self.images):
            mask = lab.project_mesh_mask(self.mesh, view, im, 0, 0, 0)
            images.append({**im, 'pixels':mask.astype('uint8').tobytes()})
        dat = dict(images=images, name='CUBE', sha256='synthetic')
        report = lab.evaluate(dat, self.mesh, 0, 0, 0)
        self.assertEqual([view['iou'] for view in report['views']], [1.0] * 4)
        self.assertEqual(report['gates']['maximum_interior_missing_pixels'], 0)
        displaced = self.mesh.copy()
        displaced.apply_translation((6, 0, 0))
        bad = lab.evaluate(dat, displaced, 0, 0, 0)
        self.assertLess(bad['gates']['min_view_iou'], 1.0)

    def test_reference_interior_misses_are_not_hidden_by_precision(self):
        reference = np.ones((15, 15), bool)
        model = reference.copy()
        model[7, 7] = False
        score = lab.mask_metrics(reference, model)
        self.assertEqual(score['interior_missing_pixels'], 1)
        self.assertEqual(score['missing_reference_pixels'], 1)
        self.assertLess(score['iou'], 1.0)


if __name__ == '__main__':
    unittest.main()
