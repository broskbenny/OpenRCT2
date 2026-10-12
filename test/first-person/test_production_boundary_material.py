"""Compile the exact production alpha-versus-geometry policy without the game."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]

SOURCE = r'''
#include "src/openrct2/paint/FirstPersonBoundaryMaterialPolicy.h"
#include <cassert>
#include <cstdint>
#include <iostream>
using namespace OpenRCT2::Paint;
enum class Kind : uint8_t { unknown, transparent, colour };
struct Decision { Kind kind{}; uint8_t pixel{}; int observedViews{}; int transparentViews{}; };

int main()
{
    for (int geometry = 0; geometry < 2; ++geometry)
    {
        auto colour = ResolveFirstPersonBoundaryMaterial(
            Decision{Kind::colour, 42, 4, 0}, geometry != 0);
        assert(colour.kind == Kind::colour && colour.pixel == 42);
        assert(colour.observedViews == 4);

        auto absent = ResolveFirstPersonBoundaryMaterial(
            Decision{Kind::unknown, 0, 1, 0}, geometry != 0);
        assert(absent.kind == Kind::unknown);

        auto alpha = ResolveFirstPersonBoundaryMaterial(
            Decision{Kind::transparent, 0, 1, 1}, geometry != 0);
        assert(alpha.kind == (geometry ? Kind::unknown : Kind::transparent));
        assert(alpha.observedViews == 1);
        auto consensus = ResolveFirstPersonBoundaryMaterial(
            Decision{Kind::transparent, 0, 2, 2}, geometry != 0);
        assert(consensus.kind == Kind::transparent);
    }
    constexpr auto alpha4 = ResolveFirstPersonBoundaryMaterial(
        Decision{Kind::transparent, 0, 4, 1}, true);
    static_assert(alpha4.kind == Kind::unknown);
    std::cout << "production boundary material policy: PASS\n";
}
'''

class ProductionBoundaryMaterialTest(unittest.TestCase):
    def test_real_policy_cannot_turn_solid_geometry_into_sprite_alpha(self):
        if shutil.which("g++") is None:
            self.skipTest("Native compiler unavailable")
        with tempfile.TemporaryDirectory(prefix="fp-boundary-material-") as folder:
            source = Path(folder) / "test.cpp"
            binary = Path(folder) / "test"
            source.write_text(SOURCE)
            subprocess.run([
                "g++", "-std=c++20", "-O2", "-Wall", "-Wextra", "-Werror",
                "-pedantic", "-fsanitize=undefined", "-fno-sanitize-recover=all",
                "-I", str(ROOT), str(source), "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)

    def test_real_material_bake_uses_boundary_policy(self):
        renderer = (ROOT / "src/openrct2/paint/FirstPersonRenderer.Core.inc").read_text()
        self.assertIn("ResolveFirstPersonBoundaryMaterial(", renderer)
        self.assertIn("materialEvidence.finish(), hull.valid", renderer)


if __name__ == "__main__":
    unittest.main()
