"""Regression contract for native attached glyphs projected onto a real 3D panel.

This is a geometry/projection oracle using OpenRCT2's integer isometric
projection convention, not a claim that every 3D-sign asset has been solved.
The in-game projector is compiled by the separate Windows Release job.
"""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
PAINT = (ROOT / "src/openrct2/paint/FirstPersonRenderer.Paint.inc").read_text(encoding="utf-8")
CORE = (ROOT / "src/openrct2/paint/FirstPersonRenderer.Core.inc").read_text(encoding="utf-8")


def project(rotation, point):
    x, y, z = point
    if rotation == 1:
        x, y = y, -x
    elif rotation == 2:
        x, y = -x, -y
    elif rotation == 3:
        x, y = -y, x
    return y - x, (x + y) // 2 - z


def inverse_on_quad(face, rotation, screen):
    p0, p1, _, p3 = face
    a, b, d = [project(rotation, p) for p in (p0, p1, p3)]
    ux, uy = b[0] - a[0], b[1] - a[1]
    vx, vy = d[0] - a[0], d[1] - a[1]
    det = ux * vy - uy * vx
    if abs(det) < 0.5:
        return None
    dx, dy = screen[0] - a[0], screen[1] - a[1]
    s = (dx * vy - dy * vx) / det
    t = (ux * dy - uy * dx) / det
    return tuple(
        p0[i] + s * (p1[i] - p0[i]) + t * (p3[i] - p0[i])
        for i in range(3)
    )


class NativeSignAttachedTextProjectionTest(unittest.TestCase):
    def test_source_is_attached_glyph_not_sign_body(self):
        self.assertIn("attached.diagnosticReconstructionKind = 4;", CORE)
        self.assertIn("LargeSceneryFlag::is3DText", CORE)

    def test_glyph_reprojection_is_world_fixed(self):
        # The clicked ssig2 hull contains a vertical face on world y=4096.
        # Map its visible glyph corners through the native source projection.
        face = (
            (3008, 4096, 152),
            (3016, 4096, 152),
            (3016, 4096, 160),
            (3008, 4096, 160),
        )
        for rotation in range(4):
            with self.subTest(rotation=rotation):
                # Distinct screen points must return distinct locations on the
                # same physical plane, independent of the first-person camera.
                for s, t in ((0.125, 0.25), (0.5, 0.5), (0.75, 0.75)):
                    world = tuple(
                        face[0][i]
                        + s * (face[1][i] - face[0][i])
                        + t * (face[3][i] - face[0][i])
                        for i in range(3)
                    )
                    image = project(rotation, world)
                    inverse = inverse_on_quad(face, rotation, image)
                    self.assertIsNotNone(inverse)
                    self.assertAlmostEqual(inverse[1], 4096, places=4)
                    # Integer screen projection is quantised: allow one
                    # source pixel, but never a perpendicular billboard.
                    self.assertLessEqual(abs(inverse[0] - world[0]), 2.0)
                    self.assertLessEqual(abs(inverse[2] - world[2]), 2.0)

    def test_two_tile_three_d_text_line_has_one_world_plane(self):
        # Native ssig2 has two lengthwise tiles at offsets (0,0) and
        # (0,32), a 62-pixel-wide 3D font line, and glyphs up to 10px high.
        # This is a representative full-panel world frame, not an assumption
        # that each reconstructed 4-unit face owns a complete glyph.
        face = (
            (3008, 4096, 152),
            (3072, 4096, 152),
            (3072, 4096, 168),
            (3008, 4096, 168),
        )
        for rotation in range(4):
            world_text_centers = []
            for glyph_x in (0, 8, 16, 24, 40, 54):
                s = (glyph_x + 4.0) / 64.0
                t = 8.0 / 16.0
                world = (
                    face[0][0] + s * (face[1][0] - face[0][0]),
                    4096.0,
                    face[0][2] + t * (face[3][2] - face[0][2]),
                )
                mapped = inverse_on_quad(face, rotation, project(rotation, world))
                self.assertIsNotNone(mapped)
                self.assertAlmostEqual(mapped[1], 4096.0, places=4)
                self.assertLessEqual(abs(mapped[0] - world[0]), 2.0)
                world_text_centers.append(mapped)
            # A row of native 3D glyphs must progress along the same
            # tangent, not turn perpendicular between reconstructed faces.
            self.assertTrue(all(
                a[0] < b[0]
                for a, b in zip(world_text_centers, world_text_centers[1:])
            ), f"text orientation reversed at rotation {rotation}")

    def test_native_compiled_helper_accepts_cross_patch_text_and_rejects_unobserved(self):
        # A glyph can extend across two perfectly coplanar 4-unit hull
        # patches. It must not be restricted to a single tiny parent face.
        # Conversely, an infinite extrapolated plane is NOT body support.
        compiler = shutil.which("c++")
        self.assertIsNotNone(compiler, "C++ compiler required for native glyph contract")
        code = r"""
#include <array>
#include <cassert>
#include "src/openrct2/paint/FirstPersonAttachedArtworkRegistration.h"
using namespace OpenRCT2::Paint;
int main() {
    const std::array<FirstPersonAttachedArtworkPoint3, 4> small{{
        {0,0,0}, {4,0,0}, {4,0,4}, {0,0,4}
    }};
    const std::array<FirstPersonAttachedArtworkPoint3, 4> neighbour{{
        {4,0,0}, {8,0,0}, {8,0,4}, {4,0,4}
    }};
    const std::array<FirstPersonAttachedArtworkPoint2, 3> view{{
        {0,0}, {4,0}, {0,4}
    }};
    const auto glyph = ReprojectFirstPersonAttachedArtwork(
        small, view, 2.0f, 1.0f, 4.0f, 2.0f);
    assert(glyph.has_value());
    bool crossesSeam = false;
    for (const float x : {1.0f/6.0f, 0.5f, 5.0f/6.0f})
    for (const float y : {1.0f/6.0f, 0.5f, 5.0f/6.0f}) {
        const auto& p = glyph->corners[0];
        const auto& q = glyph->corners[1];
        const auto& r = glyph->corners[3];
        const FirstPersonAttachedArtworkPoint3 sample{
            p.x + x*(q.x-p.x) + y*(r.x-p.x),
            p.y + x*(q.y-p.y) + y*(r.y-p.y),
            p.z + x*(q.z-p.z) + y*(r.z-p.z)
        };
        const bool first = FirstPersonAttachedArtworkPointOnFace(sample, small);
        const bool second = FirstPersonAttachedArtworkPointOnFace(sample, neighbour);
        assert(first || second);
        crossesSeam |= second && !first;
    }
    assert(crossesSeam);
    assert(!FirstPersonAttachedArtworkPointOnFace({9,0,2}, small));
    assert(!FirstPersonAttachedArtworkPointOnFace({9,0,2}, neighbour));
    assert(!FirstPersonAttachedArtworkPointOnFace({2,1,2}, small));
    assert(!FirstPersonAttachedArtworkPointOnFace({2,0,2}, neighbour));
}
"""
        with tempfile.TemporaryDirectory(prefix="fp-native-sign-") as tmp:
            src = Path(tmp) / "glyph_test.cpp"
            exe = Path(tmp) / "glyph_test"
            src.write_text(code, encoding="utf-8")
            subprocess.run(
                [compiler, "-std=c++20", "-I", str(ROOT), str(src),
                 "-o", str(exe)],
                check=True, capture_output=True, text=True
            )
            subprocess.run([str(exe)], check=True, capture_output=True)

    def test_render_path_never_reorients_successfully_attached_text(self):
        self.assertIn("AttachFirstPersonLargeTextToWorld(", PAINT)
        self.assertIn("ReprojectFirstPersonAttachedArtwork(", PAINT)
        self.assertIn("FirstPersonAttachedArtworkPointOnFace(", PAINT)
        self.assertIn("if (!glyphSupported())", PAINT)
        self.assertIn("if (!worldAttached)", PAINT)
        self.assertIn("glyph.viewFacing = false;", PAINT)
        self.assertIn("glyph.exteriorOnly = true;", PAINT)
        self.assertIn("glyph.gpuRegion = 0;", PAINT)
        self.assertIn("FirstPersonSurfaceVisibleFromNativeView(", PAINT)
        self.assertIn("body.surfaces", PAINT)
        self.assertIn("glyph.billboardLeft", PAINT)
        self.assertIn("glyph.billboardTop", PAINT)
        self.assertIn("glyph.triangles[i].world = bestWorld[corners[i]];", PAINT)


if __name__ == "__main__":
    unittest.main()
