"""Counterexamples for the initial planar-solid occupancy gate."""
import unittest
import numpy as np
from check_occupancy_contract import assess


class OccupancyContractTests(unittest.TestCase):
    def test_continuous_roof_passes_initial_gate(self):
        occupancy = np.zeros((8, 6, 12), dtype=np.uint8)
        for x in range(8):
            occupancy[x, :, :2 + min(x, 7-x)] = 1
        self.assertTrue(assess(occupancy)['passes_initial_solid_occupancy_gate'])

    def test_open_walkway_cannot_be_sealed_as_solid(self):
        occupancy = np.ones((8, 6, 12), dtype=np.uint8)
        occupancy[2:6, 2:4, :7] = 0
        result = assess(occupancy)
        self.assertFalse(result['passes_initial_solid_occupancy_gate'])
        self.assertGreater(result['columns_with_internal_gaps_or_floating_sections'], 0)

    def test_empty_footprint_rejected(self):
        occupancy = np.ones((8, 6, 12), dtype=np.uint8)
        occupancy[1, 1, :] = 0
        result = assess(occupancy)
        self.assertEqual(result['empty_horizontal_columns'], 1)
        self.assertFalse(result['passes_initial_solid_occupancy_gate'])

    def test_full_columns_are_not_sufficient_for_a_roof_certificate(self):
        occupancy = np.zeros((8, 6, 12), dtype=np.uint8)
        for x in range(8):
            occupancy[x, :, :2 + (x % 2) * 8] = 1
        result = assess(occupancy)
        self.assertTrue(result['passes_initial_solid_occupancy_gate'])
        self.assertFalse(result['production_candidate_executed'])

    def test_reject_nonbinary_and_non_xyz_inputs(self):
        with self.assertRaises(ValueError):
            assess(np.array([[[2]]], dtype=np.uint8))
        with self.assertRaises(ValueError):
            assess(np.zeros((5, 5), dtype=np.uint8))


if __name__ == '__main__':
    unittest.main()
