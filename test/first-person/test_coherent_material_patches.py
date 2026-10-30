"""Exercise the real production material-patch component admission in C++.

The test extracts the implementation verbatim. No approximate Python shadow
algorithm and no proprietary native sprite fixture is required.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
VISUAL = ROOT / "src/openrct2/paint/FirstPersonVisualHull.h"
SMALL = ROOT / "src/openrct2/paint/FirstPersonRenderer.SmallScenery.inc"


class CoherentMaterialPatchesTest(unittest.TestCase):
    def test_bake_uses_component_filter_only_after_heavy_material_rejection(self):
        s = SMALL.read_text(encoding="utf-8")
        begin = s.index("            // A high unsupported-face count")
        end = s.index("            if (appearance.certifiesHullMaterial)", begin + 80)
        block = s[begin:end + 120]
        self.assertIn("unsupportedVisibleFaces >= 128", block)
        self.assertIn("unsupportedVisibleFaces > emittedFaces / 2", block)
        self.assertIn("SelectFirstPersonCoherentMaterialPatches(", s)
        self.assertIn("coherent.retainedFaces == 0", s)
        self.assertIn("materialCoverage.clearFace(faceIndex);", s)
        self.assertIn("emittedFaces = coherent.retainedFaces;", s)

    def test_real_cpp_rejects_glitter_but_retains_physical_patches(self):
        compiler = shutil.which("g++") or shutil.which("clang++")
        if not compiler:
            self.skipTest("C++20 compiler unavailable")
        h = VISUAL.read_text(encoding="utf-8")
        start = h.index("    // Material admission is independent")
        end = h.index("    // A closed occupancy hull does not", start)
        selection = h[start:end]
        code = r"""
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <iostream>
#include <unordered_map>
#include <vector>
struct FirstPersonVec3 { float x, y, z; };
struct FirstPersonVisualHullFace {
    std::array<FirstPersonVec3, 4> corners{};
};
""" + selection + r"""
static FirstPersonVisualHullFace square(float x, float y, float width=2) {
    return { { {{x,y,0}, {x+width,y,0},
                 {x+width,y+width,0}, {x,y+width,0}} } };
}
int main() {
    std::vector<FirstPersonVisualHullFace> faces;
    std::vector<size_t> admitted;
    auto push = [&](float x,float y,float width=2) {
        admitted.push_back(faces.size());
        faces.push_back(square(x,y,width));
    };

    // Eight connected 2x2 quads are a physical patch, not glitter.
    for (int i=0; i<8; ++i) push(float(i*2),0);
    auto a=SelectFirstPersonCoherentMaterialPatches(faces,admitted);
    assert(a.retainedFaces==8);
    for (size_t i=0; i<8; ++i) assert(a.keep[i]);

    // Separate flakes should not be rescued by the accepted source mask.
    for (int i=0; i<8; ++i) push(float(i*7),20);
    auto b=SelectFirstPersonCoherentMaterialPatches(faces,admitted);
    assert(b.retainedFaces==8);
    for (size_t i=8; i<16; ++i) assert(!b.keep[i]);

    // An otherwise valid strip cannot rescue a hull dominated by glints.
    for (int i=0; i<32; ++i) push(float(i*7),40);
    auto c=SelectFirstPersonCoherentMaterialPatches(faces,admitted);
    assert(c.retainedFaces==0);

    // Connectedness requires a common *edge*, not a common vertex.
    faces.clear(); admitted.clear();
    for (int i=0; i<8; ++i) push(float(i*2),float(i*2));
    auto d=SelectFirstPersonCoherentMaterialPatches(faces,admitted);
    assert(d.retainedFaces==0);

    // A real broad textured panel does not need eight artificial facets.
    faces.clear(); admitted.clear();
    push(0,0,16);
    auto e=SelectFirstPersonCoherentMaterialPatches(faces,admitted);
    assert(e.retainedFaces==1);

    // Unsupported geometry cannot connect distinct textured islands.
    faces.clear(); admitted.clear();
    push(0,0);
    faces.push_back(square(2,0)); // no source-backed material
    push(4,0);
    auto f=SelectFirstPersonCoherentMaterialPatches(faces,admitted);
    assert(f.retainedFaces==0);

    // Source geometry and material do not imply a manufactured backing.
    faces.clear(); admitted.clear();
    assert(SelectFirstPersonCoherentMaterialPatches(faces,admitted)
        .retainedFaces==0);
    std::cout << "connected source-backed material: PASS\\n";
}
""";
        with tempfile.TemporaryDirectory(prefix="fp-material-patches-") as folder:
            src = Path(folder) / "component.cpp"
            exe = Path(folder) / "component"
            src.write_text(code, encoding="utf-8")
            build = subprocess.run(
                [compiler, "-std=c++20", "-O2", "-Wall", "-Wextra",
                 "-Werror", "-pedantic", str(src), "-o", str(exe)],
                text=True, capture_output=True)
            self.assertEqual(build.returncode, 0, build.stderr)
            check = subprocess.run([str(exe)], text=True, capture_output=True)
            self.assertEqual(check.returncode, 0, check.stderr)


if __name__ == "__main__":
    unittest.main()
