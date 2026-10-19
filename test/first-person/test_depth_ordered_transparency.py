"""Exactness contract for the depth-separated transparency fast path.

The shader writes a palette-filter token for every surface and the compositor
applies the native palette rows far-to-near. A strict camera-z interval
separation guarantees the physical order everywhere and permits skipping
both general depth selection and native-ordinal arbitration.
"""
from pathlib import Path
import math
import random
import unittest

GL = Path(__file__).resolve().parents[2] / (
    "src/openrct2-ui/drawing/engines/opengl/FirstPersonGLRenderer.cpp")


def order_nonoverlapping_intervals(candidates):
    """Return far-to-near indices only for provably nonoverlapping intervals."""
    if not 2 <= len(candidates) <= 64:
        return None
    order = sorted(range(len(candidates)), key=lambda i: (-candidates[i][0], i))
    for n, index in enumerate(order):
        lo, hi, eligible = candidates[index]
        if not eligible or not math.isfinite(lo) or not math.isfinite(hi) or lo <= 0 or lo > hi:
            return None
        if n and candidates[order[n - 1]][0] <= hi + 0.01:
            return None
    return order


def physical_native_depth_peel(fragments):
    """Exact shader contract: farthest physical depth, earliest ordinal at that depth."""
    remaining = list(fragments)
    out = []
    while remaining:
        farthest = max(f[0] for f in remaining)
        selected = min((f for f in remaining if f[0] == farthest),
                       key=lambda f: f[1])
        out.append(selected[2])
        remaining.remove(selected)
    return out


class DepthOrderedTransparencyTest(unittest.TestCase):
    def test_renderer_has_exact_guards_and_one_draw_per_candidate(self):
        source = GL.read_text(encoding="utf-8")
        self.assertIn("float depthMin{}, depthMax{};", source)
        self.assertIn("depthMin = std::min(depthMin, clipped[i].z);", source)
        self.assertIn("depthMax = std::max(depthMax, clipped[i].z);", source)
        self.assertIn("!surface->coplanarOwner", source)
        self.assertIn("candidate.depthMin > candidate.depthMax", source)
        self.assertIn("kSeparationMargin = 0.01f", source)
        self.assertIn("std::array<size_t, 64> depthOrder{}", source)
        self.assertIn("std::array<size_t, 64> offsets{}", source)
        self.assertIn("std::isfinite(candidate.depthMin)", source)
        self.assertIn('glUniform1i, Uniform(_program, "uPeelStage"), 3', source)
        self.assertIn("GLsizei(candidate.vertexCount)", source)
        self.assertIn("composeLayer(logicalLayer, tile);", source)
        self.assertIn("depthOrderedTiles=%zu", source)
        self.assertIn("conservativeLayerLimit(tile)", source)
        self.assertNotIn("waitForPeelCoverageQuery", source)

    def test_nonintersecting_ranges_preserve_exact_palette_sequence(self):
        rng = random.Random(20261019)
        for _ in range(50000):
            count = rng.randrange(2, 24)
            intervals = []
            fragments = []
            depth = 1.0
            for _ in range(count):
                depth += rng.uniform(0.02, 2)
                minimum = depth
                depth += rng.uniform(0.01, 1)
                maximum = depth
                intervals.append((minimum, maximum, True))
                fragments.append((
                    rng.uniform(minimum, maximum),
                    rng.randrange(0, 1 << 24),
                    rng.randrange(1, 255)
                ))
                depth += rng.uniform(0.02, 0.5)
            indices = order_nonoverlapping_intervals(intervals)
            self.assertIsNotNone(indices)
            self.assertEqual(
                physical_native_depth_peel(fragments),
                [fragments[i][2] for i in indices])

    def test_intersections_and_equal_depth_fall_back(self):
        candidates = [
            (1.0, 2.0, True),
            (1.5, 3.0, True)
        ]
        self.assertIsNone(order_nonoverlapping_intervals(candidates))
        self.assertIsNone(order_nonoverlapping_intervals([
            (1.0, 2.0, True), (2.0, 3.0, True)]))
        self.assertIsNone(order_nonoverlapping_intervals([
            (1.0, 1.2, True), (2.0, 2.4, False)]))
        self.assertIsNone(order_nonoverlapping_intervals([
            (0.0, 0.0, True), (2.0, 3.0, True)]))
        self.assertIsNone(order_nonoverlapping_intervals([
            (float('nan'), 2.0, True), (3.0, 4.0, True)]))
        self.assertIsNone(order_nonoverlapping_intervals([
            (1.0, float('inf'), True), (3.0, 4.0, True)]))

    def test_palette_order_noncommutative(self):
        # Synthetic noncommutative palette filters. A mistaken near-to-far
        # ordering must fail, whereas depth-separated far-to-near matches exact.
        rows = [(i * 73 + 17) % 256 for i in range(256)]
        def apply(v, row):
            return (v * 31 + rows[row]) % 256
        far = (10., 0, 10)
        near = (2., 0, 40)
        reference = physical_native_depth_peel([near, far])
        value = 128
        for row in reference:
            value = apply(value, row)
        near_first = apply(apply(128, near[2]), far[2])
        self.assertNotEqual(value, near_first)


if __name__ == "__main__":
    unittest.main()
