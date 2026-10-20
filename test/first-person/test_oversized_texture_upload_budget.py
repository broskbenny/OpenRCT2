"""An indivisible large texture must not permanently stall GPU region admission."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
GL = ROOT / "src/openrct2-ui/drawing/engines/opengl/FirstPersonGLRenderer.cpp"
ASYNC = ROOT / "src/openrct2/paint/FirstPersonAsync.h"


class OversizedTextureBudgetTest(unittest.TestCase):
    def test_both_texture_sources_use_oversized_admission(self):
        gl = GL.read_text(encoding="utf-8")
        self.assertIn("uploadBudget.takeOversized(size_t(source->width) * size_t(source->height))", gl)
        self.assertIn("uploadBudget.takeOversized(size_t(surface.immutableWidth) * size_t(surface.immutableHeight))", gl)
        self.assertIn("6 * sizeof(GPUVertex)", gl)

    def test_region_materials_precede_vertex_charges_in_both_passes(self):
        gl = GL.read_text(encoding="utf-8")
        region_upload = gl.split("while (uploadBudget.available())", 1)[1]
        # Use full source-order matches to cover each resident upload phase.
        first_vertex = region_upload.index("if (!uploadBudget.take(")
        first_material = region_upload.index("if (!prepareMaterial(surface))")
        self.assertLess(first_material, first_vertex)
        second_vertex = region_upload.index(
            "if (!uploadBudget.take(", first_vertex + 1)
        second_material = region_upload.index(
            "if (!prepareMaterial(surface))", first_material + 1)
        self.assertLess(first_vertex, second_material)
        self.assertLess(second_material, second_vertex)

    def test_actual_budget_compiles_and_guards_indivisible_uploads(self):
        compiler = shutil.which("g++") or shutil.which("clang++")
        if compiler is None:
            self.skipTest("No C++ compiler for executable budget test")
        program = r"""
#include "FirstPersonAsync.h"
#include <cassert>
#include <chrono>
using namespace OpenRCT2::Paint;
int main() {
    using namespace std::chrono;
    const auto generous = hours(1);
    FirstPersonFrameBudget fresh(512, duration_cast<microseconds>(generous));
    assert(fresh.takeOversized(513));
    assert(!fresh.available());
    assert(!fresh.takeOversized(514));

    FirstPersonFrameBudget consumed(512, duration_cast<microseconds>(generous));
    assert(consumed.take(1));
    assert(!consumed.takeOversized(513));
    assert(consumed.take(511));
    assert(!consumed.available());

    FirstPersonFrameBudget ordinary(512, duration_cast<microseconds>(generous));
    assert(ordinary.takeOversized(300));
    assert(ordinary.take(212));
    assert(!ordinary.take(1));

    FirstPersonFrameBudget phased(512, duration_cast<microseconds>(generous));
    assert(phased.take(1));
    phased.beginPhase(duration_cast<microseconds>(generous));
    assert(!phased.takeOversized(1000));

    // A cold, oversized resident material consumes this frame. Its six
    // vertices are safely deferred; the next frame can append them.
    FirstPersonFrameBudget firstRegionFrame(512, duration_cast<microseconds>(generous));
    assert(firstRegionFrame.takeOversized(1024));
    assert(!firstRegionFrame.take(6));
    FirstPersonFrameBudget nextRegionFrame(512, duration_cast<microseconds>(generous));
    assert(nextRegionFrame.take(6));

    FirstPersonFrameBudget zero(0, duration_cast<microseconds>(generous));
    assert(!zero.takeOversized(1000));
}
"""
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "budget.cpp"
            binary = Path(directory) / "budget"
            source.write_text(program, encoding="utf-8")
            subprocess.run(
                [compiler, "-std=c++20", "-pthread", "-I", str(ASYNC.parent),
                 str(source), "-o", str(binary)],
                check=True, capture_output=True, text=True)
            subprocess.run([str(binary)], check=True, capture_output=True, text=True)


if __name__ == "__main__":
    unittest.main()
