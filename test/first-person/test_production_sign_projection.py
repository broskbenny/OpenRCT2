"""Compile and execute the exact C++ glyph-to-world registration used by the game.

Unlike a reimplementation in Python, this actually checks the production
ReprojectFirstPersonAttachedArtwork function and its failure conditions.
"""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]


class ProductionSignProjectionTest(unittest.TestCase):
    def test_cpp_sign_registration_and_rejection(self):
        compiler = shutil.which("c++")
        self.assertIsNotNone(compiler, "A C++ compiler is required for this check")
        source = r"""
#include "src/openrct2/paint/FirstPersonAttachedArtworkRegistration.h"
#include <array>
#include <cassert>
#include <cmath>
using namespace OpenRCT2::Paint;
int main() {
    const std::array<FirstPersonAttachedArtworkPoint3,4> face{{
        {3008,4096,152}, {3072,4096,152},
        {3072,4096,168}, {3008,4096,168}
    }};
    // OpenRCT2's rotation-zero native projection:
    // (screenX, screenY) = (worldY-worldX,
    //                         (worldX+worldY)/2-worldZ).
    const std::array<FirstPersonAttachedArtworkPoint2,3> projected{{
        {1088,3400}, {1024,3432}, {1088,3384}
    }};
    const auto glyph = ReprojectFirstPersonAttachedArtwork(
        face, projected, 1052,3404, 8,8);
    assert(glyph.has_value());
    assert(std::abs(glyph->centerS - 0.5f) < 0.001f);
    assert(std::abs(glyph->centerT - 0.5f) < 0.001f);
    for (const auto& p : glyph->corners)
        assert(std::abs(p.y - 4096.0f) < 0.001f);

    // The complete two-tile sign's 3D-font line must lie on the same
    // world plane at its source-image positions.
    for (const float s : {0.125f, 0.25f, 0.5f, 0.75f, 0.875f}) {
        const float screenX = 1088.0f - 64.0f * s;
        const float screenY = 3400.0f + 32.0f*s - 8.0f;
        auto letter = ReprojectFirstPersonAttachedArtwork(
            face, projected, screenX-1, screenY-1, 2,2);
        assert(letter.has_value());
        assert(std::abs(letter->centerS - s) < 0.001f);
        for (const auto& corner : letter->corners)
            assert(std::abs(corner.y - 4096.0f) < 0.001f);
    }

    const auto outOfBounds = ReprojectFirstPersonAttachedArtwork(
        face, projected, 900,3404, 8,8);
    assert(!outOfBounds.has_value());

    auto degenerate = projected;
    degenerate[2] = degenerate[0];
    assert(!ReprojectFirstPersonAttachedArtwork(
        face, degenerate, 1052,3404,8,8).has_value());

    auto warped = face;
    warped[2].z += 2.0f;
    assert(!ReprojectFirstPersonAttachedArtwork(
        warped, projected, 1052,3404,8,8).has_value());

    assert(!ReprojectFirstPersonAttachedArtwork(
        face,projected,1052,3404,0,8).has_value());
}
"""
        with tempfile.TemporaryDirectory(prefix="fp-sign-reg-") as directory:
            executable = Path(directory) / "sign_registration"
            command = [
                compiler, "-std=c++20", "-O1", "-Wall", "-Wextra",
                "-I", str(ROOT), "-x", "c++", "-", "-o", str(executable),
            ]
            compiled = subprocess.run(
                command, input=source, text=True, capture_output=True
            )
            self.assertEqual(
                compiled.returncode, 0,
                f"production C++ registration compilation failed:\n{compiled.stderr}"
            )
            executed = subprocess.run(
                [str(executable)], text=True, capture_output=True
            )
            self.assertEqual(
                executed.returncode, 0,
                f"production C++ registration assertions failed:\n{executed.stderr}"
            )


if __name__ == "__main__":
    unittest.main()
