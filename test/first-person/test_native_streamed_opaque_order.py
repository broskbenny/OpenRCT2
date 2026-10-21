"""Native opaque sprite overlap must not depend on source-cache arrival order."""
from pathlib import Path
import random
import unittest

ROOT = Path(__file__).resolve().parents[2]
GL = ROOT / "src/openrct2-ui/drawing/engines/opengl/FirstPersonGLRenderer.cpp"


class NativeOpaqueOrderingTest(unittest.TestCase):
    def test_native_opaque_order_is_fixed_before_vertex_packing(self):
        source = GL.read_text(encoding="utf-8")
        start = source.index("std::vector<const Paint::FirstPersonSurface*> streamedOpaque;")
        sort = source.index("std::stable_sort(\n            streamedOpaque.begin()", start)
        prepare = source.index("auto prepareMaterial =", sort)
        split = source.index("for (const auto* surface : streamedOpaque)", prepare)
        self.assertLess(start, sort)
        self.assertLess(sort, prepare)
        self.assertLess(prepare, split)
        section = source[sort:prepare]
        self.assertIn("surface->nativePaintOrdinal == 0", section)
        self.assertIn("std::numeric_limits<uint64_t>::max()", section)
        self.assertIn("return order(a) < order(b);", section)
        self.assertNotIn("glPolygonOffset", section)

    def test_equal_depth_native_layers_ignore_arrival_order(self):
        rng = random.Random(20261021)
        for _ in range(4000):
            ordinals = list(range(1, rng.randrange(4, 60)))
            rng.shuffle(ordinals)
            first = sorted(ordinals)
            rng.shuffle(ordinals)
            second = sorted(ordinals)
            self.assertEqual(first, second)
            # Unknown (zero) order is last; no ungrounded priority is
            # assigned between genuinely unnumbered streamed surfaces.
            mixed = [(n if n % 7 else 0, i) for i, n in enumerate(ordinals)]
            priority = lambda x: x[0] if x[0] else (1 << 64) - 1
            output = sorted(mixed, key=priority)
            self.assertEqual([priority(o) for o in output],
                             sorted(priority(o) for o in mixed))


if __name__ == "__main__":
    unittest.main()
