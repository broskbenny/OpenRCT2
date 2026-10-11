"""Synthetic checks for native indexed-palette large-scenery compositing."""
import unittest
import numpy as np
from audit_native_artwork import (
    rotation_xy, tile_composite, audit_registration, audit_asset,
)


def image(pixels, x=0, y=0):
    a = np.asarray(pixels, dtype=np.uint8)
    h, w = a.shape
    return dict(width=w, height=h, x=x, y=y, pixels=a.tobytes())


class NativeArtworkAuditTest(unittest.TestCase):
    def test_four_direction_rotation_matches_authoritative_coordinate_contract(self):
        self.assertEqual([rotation_xy(7, 3, i) for i in range(4)],
                         [(7, 3), (3, -7), (-7, -3), (-3, 7)])

    def test_native_sequence_layers_are_depth_sorted(self):
        blank = image([[0]])
        sprites = [blank.copy() for _ in range(12)]
        sprites[4] = image([[51]])
        sprites[8] = image([[86]])
        dat = dict(tiles=[dict(x=0, y=0, z=0), dict(x=0, y=0, z=0)],
                   images=sprites)
        composite, _ = tile_composite(dat, 0)
        self.assertEqual(composite.tolist(), [[86]])

    def test_exact_palette_matches_after_translation(self):
        reference = np.array([[0, 0, 0, 0],
                              [0, 5, 6, 0],
                              [0, 7, 9, 0],
                              [0, 0, 0, 0]], dtype=np.uint8)
        canvas = np.zeros((9, 11), dtype=np.uint8)
        canvas[3:7, 4:8] = reference
        got = audit_registration(reference, canvas)
        self.assertEqual(got['matched_palette_pixels'], 4)
        self.assertEqual(got['exact_reference_fraction'], 1.0)
        self.assertFalse(got['independent_geometry_certificate'])

    def test_incorrect_palette_cannot_be_disguised_as_opaque_support(self):
        reference = np.array([[0, 5, 6, 0], [0, 7, 9, 0]], dtype=np.uint8)
        canvas = reference.copy()
        canvas[0, 2] = 99
        got = audit_registration(reference, canvas)
        self.assertEqual(got['matched_palette_pixels'], 3)
        self.assertEqual(got['different_palette_pixels'], 1)
        self.assertEqual(got['exact_reference_fraction'], 0.75)

    def test_incomplete_image_table_is_rejected(self):
        dat = dict(tiles=[dict(x=0, y=0, z=0)] * 4,
                   images=[image([[1]])] * 5)
        with self.assertRaises(ValueError):
            tile_composite(dat, 0)

    def test_preview_identical_to_tile_source_does_not_count_as_independent(self):
        pictures = [image([[0, 5], [7, 9]]) for _ in range(4)]
        sprites = [image([[0]]) for _ in range(16)]
        for direction in range(4):
            sprites[direction] = pictures[direction]
        dat = dict(tiles=[dict(x=0, y=0, z=0)] * 4,
                   images=pictures + sprites, name='SYNTH', sha256='synthetic')
        result = audit_asset(dat)
        self.assertEqual(len(result['native_preview_comparisons']), 4)
        for row in result['native_preview_comparisons']:
            self.assertEqual(row['exact_reference_fraction'], 1.0)
            self.assertFalse(row['independent_geometry_certificate'])


if __name__ == '__main__':
    unittest.main()
