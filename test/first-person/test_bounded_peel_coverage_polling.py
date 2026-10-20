"""Coverage probes may reduce work but must never omit an unproven layer."""
from pathlib import Path
import random
import unittest

ROOT = Path(__file__).resolve().parents[2]
GL = ROOT / "src/openrct2-ui/drawing/engines/opengl/FirstPersonGLRenderer.cpp"


def process_layers(coverage, ready_at, stride=4):
    """GPU probe model: only a completed zero-coverage pass can terminate."""
    executed = []
    for index, samples in enumerate(coverage):
        if index % stride == 0:
            for earlier in executed:
                if ready_at[earlier] <= index and coverage[earlier] == 0:
                    return executed
        executed.append(index)
    return executed


class BoundedPeelCoveragePollingTest(unittest.TestCase):
    def test_actual_peel_path_polls_once_at_bounded_intervals(self):
        gl = GL.read_text(encoding="utf-8")
        start = gl.index("constexpr size_t kCoveragePollInterval = 4;")
        end = gl.index("// Restore the physical viewport/scissor", start)
        path = gl[start:end]
        self.assertIn("if (pass % kCoveragePollInterval == 0", path)
        self.assertEqual(path.count("pollPeelCoverageQueries(coverageToken)"), 1)
        self.assertIn("GL_ANY_SAMPLES_PASSED", path)
        self.assertIn("glDrawElements,GL_TRIANGLES", path)
        self.assertIn("composeLayer(logicalLayer,tile);", path)
        self.assertNotIn("glFinish", path)
        self.assertNotIn("GL_QUERY_RESULT", path)
        self.assertIn("if (coverageSlot", path)
        self.assertIn("&& _peelCoverageTokens[i] == token", gl.replace(
            "if (_peelCoverageTokens[i] == token", "if (&& _peelCoverageTokens[i] == token"))
        # Guard against reintroducing a second poll after stage one or two.
        after_geometry = path.split("glDrawElements,GL_TRIANGLES", 1)[1]
        self.assertEqual(after_geometry.count("pollPeelCoverageQueries("), 0)

    def test_randomized_delayed_queries_never_truncate_nonempty_layers(self):
        rng = random.Random(20261020)
        for _ in range(70000):
            length = rng.randrange(0, 180)
            nonempty = rng.randrange(length + 1)
            coverage = [1] * nonempty + [0] * (length - nonempty)
            ready_at = [i + rng.randrange(0, 32) for i in range(length)]
            executed = process_layers(coverage, ready_at)
            self.assertEqual(executed, list(range(len(executed))))
            self.assertGreaterEqual(len(executed), nonempty)

    def test_zero_latency_can_stop_only_after_a_completed_empty_pass(self):
        self.assertEqual(
            process_layers([1] * 3 + [0] * 20, list(range(23))), [0, 1, 2, 3])
        self.assertEqual(
            process_layers([1] * 6 + [0] * 20, list(range(26))), list(range(8)))


if __name__ == "__main__":
    unittest.main()
