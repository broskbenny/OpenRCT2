"""Regression contract for native attached glyphs projected onto a real 3D panel.

This is a geometry/projection oracle using OpenRCT2's integer isometric
projection convention, not a claim that every 3D-sign asset has been solved.
The in-game projector is compiled by the separate Windows Release job.
"""

from pathlib import Path
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

    def test_render_path_never_reorients_successfully_attached_text(self):
        self.assertIn("AttachFirstPersonLargeTextToWorld(", PAINT)
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
