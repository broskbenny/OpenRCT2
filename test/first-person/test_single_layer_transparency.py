"""Single-layer fast path must preserve exact palette-filter selection.

Rectangular overlap <= 1 is a sufficient condition: different physical
surfaces cannot cover the same pixel, even when actual sprite opacity is less
than the bounding rectangle. The existing two-pass depth/ordinal selector
therefore reduces to the same one-pass depth test and encoded token.
"""
from pathlib import Path
import random
import unittest

ROOT = Path(__file__).resolve().parents[2]
RENDERER = ROOT / "src/openrct2-ui/drawing/engines/opengl/FirstPersonGLRenderer.cpp"


def exact_two_pass_fragments(fragments):
    """Mimic stage 1 (maximum physical depth), stage 2 (minimum ordinal)."""
    depth = max((f[0] for f in fragments), default=None)
    if depth is None:
        return 0
    ordinal = min(f[1] for f in fragments if f[0] == depth)
    for f_depth, f_ordinal, row in fragments:
        if f_depth == depth and f_ordinal == ordinal:
            return (ordinal << 8) | (row + 1)
    raise AssertionError("unreachable")


def direct_single_layer_fragments(fragments):
    """Mimic GL_GREATER depth test from zero; equal depth keeps first."""
    best_depth = 0.0
    encoded = 0
    for depth, ordinal, row in fragments:
        if depth > best_depth:
            best_depth = depth
            encoded = (ordinal << 8) | (row + 1)
    return encoded


class SingleLayerTransparencyTest(unittest.TestCase):
    def test_single_layer_source_is_conservative_and_exact(self):
        gl = RENDERER.read_text(encoding="utf-8")
        self.assertIn("conservativeLayerLimit(tile)", gl)
        self.assertIn("if (logicalPasses == 1)", gl)
        self.assertIn('glUniform1i, Uniform(_program, "uPeelStage"), 3', gl)
        self.assertIn("glCall(glDepthFunc, GL_GREATER);", gl)
        self.assertIn("oIndex = (ordinal << 8u) | (row + 1u);", gl)
        self.assertIn("composeLayer(logicalLayer, tile);", gl)
        self.assertIn("singleLayerTiles=%zu", gl)
        self.assertNotIn("waitForPeelCoverageQuery", gl)

    def test_single_surface_fragments_match_depth_and_native_order(self):
        rng = random.Random(20261018)
        # A single material surface may generate fragments from both quad
        # triangles. Their native paint ordinal remains identical.
        for _ in range(10000):
            ordinal = rng.randint(1, (1 << 24) - 1)
            fragments = []
            for _ in range(rng.randrange(0, 5)):
                # Include equal-depth ties and arbitrary palette-filter rows.
                depth = rng.choice([0.01, 0.17, 0.5, 0.9, 0.9999])
                fragments.append((depth, ordinal, rng.randrange(255)))
            self.assertEqual(exact_two_pass_fragments(fragments),
                             direct_single_layer_fragments(fragments))

    def test_nonoverlapping_rectangles_cannot_need_multiple_peels(self):
        rects = [(0, 0, 10, 10), (10, 0, 20, 10),
                 (0, 10, 20, 20), (20, 0, 30, 30)]
        for y in range(0, 31):
            for x in range(0, 31):
                touching = sum(x0 <= x < x1 and y0 <= y < y1
                               for x0, y0, x1, y1 in rects)
                self.assertLessEqual(touching, 1)


if __name__ == "__main__":
    unittest.main()
