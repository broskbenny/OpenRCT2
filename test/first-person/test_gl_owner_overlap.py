"""Offscreen pixel regression for adjacent coplanar native wall materials.

Uses the REAL production OpenGL scene shaders with llvmpipe/EGL. A successful
C++ build or glslang syntax check alone cannot detect depth-buffer flicker.
The picked wall pair occupies y=4544 and overlaps at z=120..128, with the
later native paint ordinal belonging to wallcf16.

This is a surgical shader/owner replay test, not a substitute for rendering
the entire game or checking sprite texture quality.
"""

from pathlib import Path
import math
import re
import sys
import unittest

import moderngl
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
RENDERER = ROOT / "src/openrct2-ui/drawing/engines/opengl/FirstPersonGLRenderer.cpp"
EXTRACT = re.compile(
    r'constexpr\s+char\s+(k\w+Shader)\[\]\s*=\s*R"GLSL\((.*?)\)GLSL";',
    re.DOTALL,
)
WIDTH = HEIGHT = 96
VERTEX_FORMAT = np.dtype([
    ("world", "<f4", (3,)),
    ("uv", "<f4", (2,)),
    ("atlas", "<f4", (4,)),
    ("size", "<f4", (2,)),
    ("atlasLayer", "<i4"),
    ("palettes", "<i4", (4,)),
    ("flags", "<i4"),
    ("maskAtlas", "<f4", (4,)),
    ("maskSize", "<f4", (2,)),
    ("maskLayer", "<i4"),
    ("paintOrdinal", "<u4"),
    ("normal", "<f4", (3,)),
], align=False)
ATTRIBUTES = (
    "aWorld", "aUV", "aAtlas", "aSize", "aAtlasLayer",
    "aPalettes", "aFlags", "aMaskAtlas", "aMaskSize", "aMaskLayer",
    "aPaintOrdinal", "aOutwardNormal",
)
FORMAT = "3f 2f 4f 2f 1i 4i 1i 4f 2f 1i 1u 3f"


def quad(z0, z1, colour, opposite_diagonal, y=4544.0):
    x0, x1 = 2624.0, 2656.0
    if opposite_diagonal:
        points = [
            (x0, y, z0), (x1, y, z0), (x1, y, z1),
            (x0, y, z0), (x1, y, z1), (x0, y, z1),
        ]
    else:
        points = [
            (x1, y, z0), (x0, y, z0), (x0, y, z1),
            (x1, y, z0), (x0, y, z1), (x1, y, z1),
        ]
    vertices = np.zeros(6, dtype=VERTEX_FORMAT)
    vertices["world"] = points
    # Actual clicked walls sample native indexed atlas artwork.
    vertices["flags"] = 4
    vertices["size"][:] = (2.0, 2.0)
    vertices["atlas"][:] = (0.0, 0.0, 2.0, 2.0)
    vertices["atlasLayer"] = 0 if colour == 6 else 1
    vertices["palettes"][:, 3] = colour
    vertices["maskLayer"] = -1
    vertices["paintOrdinal"] = colour
    vertices["normal"][:] = (2.0, y, 17.0)
    return vertices


def quad_x(x, y0, y1, z0, z1, colour, outward_sign):
    """Opposite authored semantic wall sides at one EXACT physical plane."""
    if outward_sign > 0:
        points = [
            (x, y0, z0), (x, y1, z0), (x, y1, z1),
            (x, y0, z0), (x, y1, z1), (x, y0, z1),
        ]
    else:
        points = [
            (x, y1, z0), (x, y0, z0), (x, y0, z1),
            (x, y1, z0), (x, y0, z1), (x, y1, z1),
        ]
    vertices = np.zeros(6, dtype=VERTEX_FORMAT)
    vertices["world"] = points
    vertices["uv"] = [
        ((p[1] - y0) * 2.0 / (y1 - y0),
         (p[2] - z0) * 2.0 / (z1 - z0))
        for p in points
    ]
    vertices["flags"] = 4
    vertices["size"][:] = (2.0, 2.0)
    vertices["atlas"][:] = (0.0, 0.0, 2.0, 2.0)
    vertices["atlasLayer"] = 0 if colour == 6 else 1
    vertices["maskLayer"] = -1
    vertices["paintOrdinal"] = colour
    # Use the identical edge-cross-product rule as the C++ GPU packer,
    # instead of feeding the expected facing sign straight into the shader.
    edge_u = vertices["world"][1] - vertices["world"][0]
    edge_v = vertices["world"][2] - vertices["world"][0]
    winding = np.cross(edge_u, edge_v)[0]
    assert winding * outward_sign > 0
    vertices["normal"][:] = (
        math.copysign(1.0, winding), x, 17.0)
    return vertices


class ProductionGLOwnerOverlapTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source = RENDERER.read_text(encoding="utf-8")
        shaders = dict(EXTRACT.findall(source))
        cls.ctx = moderngl.create_standalone_context(require=330, backend="egl")
        cls.shader = cls.ctx.program(
            vertex_shader=shaders["kVertexShader"],
            fragment_shader=shaders["kFragmentShader"],
        )
        print("GL_TEST_CONTEXT", cls.ctx.info.get("GL_RENDERER"),
              cls.ctx.info.get("GL_VERSION"), file=sys.stderr)
        cls.colour = cls.ctx.texture((WIDTH, HEIGHT), components=1, dtype="u4")
        cls.depth = cls.ctx.depth_renderbuffer((WIDTH, HEIGHT))
        cls.target = cls.ctx.framebuffer(
            color_attachments=[cls.colour], depth_attachment=cls.depth)
        cls.opaque_depth = cls.ctx.texture(
            (WIDTH, HEIGHT), components=1,
            data=np.ones((HEIGHT, WIDTH), dtype=np.float32).tobytes(),
            dtype="f4")
        cls.sprites = cls.ctx.texture_array(
            (2, 2, 2), components=1,
            data=bytes([6] * 4 + [9] * 4), dtype="u1")
        cls.palettes = cls.ctx.texture(
            (256, 256), components=1, data=bytes(256 * 256), dtype="u1")
        cls.previous_layer = cls.ctx.texture(
            (WIDTH, HEIGHT), components=1,
            data=bytes(WIDTH * HEIGHT * 4), dtype="u4")
        cls.ctx.enable(moderngl.DEPTH_TEST)
        cls.ctx.disable(moderngl.CULL_FACE)
        cls.ctx.disable(moderngl.BLEND)
        cls.ctx.depth_func = "<="

    @classmethod
    def tearDownClass(cls):
        cls.ctx.release()

    def render_overlap(self, yaw, eye, surfaces=None, side_passes=(0,),
                       forward_override=None, right_override=None):
        ctx, program = self.ctx, self.shader
        self.target.use()
        ctx.viewport = (0, 0, WIDTH, HEIGHT)
        self.colour.write(bytes(WIDTH * HEIGHT * 4))
        self.target.clear(depth=1.0)
        ctx.enable(moderngl.DEPTH_TEST)
        ctx.depth_func = "<="
        self.sprites.use(location=0)
        self.palettes.use(location=1)
        self.opaque_depth.use(location=2)
        self.opaque_depth.use(location=3)
        self.previous_layer.use(location=4)
        self.opaque_depth.use(location=5)
        # Every active GLSL sampler must have a compatible texture unit,
        # including those used only by inactive peel branches. Leaving a
        # sampler2D and usampler2DArray aliased on unit zero invalidates
        # the entire draw on core-profile OpenGL.
        for name, location in (
            ("uSprites", 0), ("uPalettes", 1), ("uOpaqueDepth", 2),
            ("uPreviousDepth", 3), ("uPreviousLayer", 4),
            ("uSelectedPhysicalDepth", 5)):
            if name in program:
                program[name].value = location
        forward = forward_override or (math.sin(yaw), math.cos(yaw), 0)
        right = right_override or (math.cos(yaw), -math.sin(yaw), 0)
        for name, value in (
            ("uEye", eye),
            ("uForward", forward), ("uRight", right),
            ("uUp", (0.0, 0.0, 1.0)),
            ("uNearFar", (1.0, 10000.0)), ("uFocal", (1.0, 1.0)),
            ("uViewport", (0.0, 0.0, float(WIDTH), float(HEIGHT))),
            ("uPeeling", False), ("uPeelStage", 0)):
            if name in program:
                program[name].value = value
        # The lower paint ordinal rail material comes first. Its top eight
        # units overlap the primary wall, whose greater ordinal must win.
        if surfaces is None:
            surfaces = (quad(112, 128, 6, True), quad(120, 136, 9, False))
        for side_pass in side_passes:
            if "uOwnerSidePass" in program:
                program["uOwnerSidePass"].value = side_pass
            for vertices in surfaces:
                buf = ctx.buffer(vertices.tobytes())
                vao = ctx.vertex_array(
                    program, [(buf, FORMAT, *ATTRIBUTES)])
                vao.render(mode=moderngl.TRIANGLES)
                vao.release()
                buf.release()
        if "uOwnerSidePass" in program:
            program["uOwnerSidePass"].value = 0
        return np.frombuffer(self.colour.read(), dtype=np.uint32).reshape(HEIGHT, WIDTH)

    def test_production_vertex_shader_preserves_owner_flags_and_palette(self):
        shaders = dict(EXTRACT.findall(RENDERER.read_text(encoding="utf-8")))
        ctx = self.ctx
        self.target.use()
        ctx.viewport = (0, 0, WIDTH, HEIGHT)
        self.colour.write(bytes(WIDTH * HEIGHT * 4))
        ctx.disable(moderngl.DEPTH_TEST)
        program = ctx.program(
            vertex_shader=shaders["kVertexShader"],
            fragment_shader="""#version 330 core
                flat in ivec4 fPalettes;
                flat in int fFlags;
                layout(location=0) out uint oIndex;
                void main() { oIndex = uint(fPalettes.w * 100 + fFlags); }
            """,
        )
        for name, value in (
            ("uEye", (2640.0, 4500.0, 124.0)),
            ("uForward", (0.0, 1.0, 0.0)),
            ("uRight", (1.0, 0.0, 0.0)),
            ("uUp", (0.0, 0.0, 1.0)),
            ("uNearFar", (1.0, 10000.0)), ("uFocal", (1.0, 1.0)),
        ):
            if name in program:
                program[name].value = value
        buf = ctx.buffer(quad(120, 136, 9, False).tobytes())
        vao = ctx.vertex_array(
            program, [(buf, "3f 36x 4i 1i 44x",
                       "aWorld", "aPalettes", "aFlags")])
        vao.render(moderngl.TRIANGLES)
        pixels = np.frombuffer(self.colour.read(), dtype=np.uint32).reshape(HEIGHT, WIDTH)
        print("GL_PRODUCTION_INPUT_DIAGNOSTICS", np.unique(pixels, return_counts=True),
              file=sys.stderr)
        self.assertEqual(int(pixels[HEIGHT//2, WIDTH//2]), 904)
        vao.release()
        buf.release()
        program.release()

    def test_software_framebuffer_rasterizes_reference_quad(self):
        ctx = self.ctx
        self.target.use()
        ctx.viewport = (0, 0, WIDTH, HEIGHT)
        self.colour.write(bytes(WIDTH * HEIGHT * 4))
        ctx.disable(moderngl.DEPTH_TEST)
        program = ctx.program(
            vertex_shader="""#version 330 core
                in vec3 aWorld;
                void main() {
                    gl_Position = vec4(
                        (aWorld.x - 2640.0)/44.0,
                        (aWorld.z - 124.0)/44.0, 0.0, 1.0);
                }
            """,
            fragment_shader="""#version 330 core
                layout(location=0) out uint oIndex;
                void main() { oIndex = 9u; }
            """,
        )
        buf = ctx.buffer(quad(120, 136, 9, False).tobytes())
        vao = ctx.vertex_array(program, [(buf, "3f 100x", "aWorld")])
        vao.render(moderngl.TRIANGLES)
        pixels = np.frombuffer(self.colour.read(), dtype=np.uint32).reshape(HEIGHT, WIDTH)
        print("GL_REFERENCE_DIAGNOSTICS", np.unique(pixels, return_counts=True),
              file=sys.stderr)
        self.assertEqual(int(pixels[HEIGHT//2, WIDTH//2]), 9)
        vao.release()
        buf.release()
        program.release()

    def test_logged_wall_vertices_encode_actual_opposite_native_faces(self):
        # The 2026-10-17 pick recorded these exact first triangles.
        # wallbr32 from (2944,4064), x2976, is outward -X; wallcf16
        # from (2752,4448), x2752, is outward +X.
        wallbr32 = np.array([
            (2976, 4096, 112), (2976, 4064, 112),
            (2976, 4064, 144)], dtype=np.float32)
        wallcf16 = np.array([
            (2752, 4448, 120), (2752, 4480, 120),
            (2752, 4480, 136)], dtype=np.float32)
        for points, expected_sign in (
            (wallbr32, -1), (wallcf16, 1)
        ):
            winding = np.cross(points[1] - points[0],
                               points[2] - points[0])
            self.assertEqual(int(np.sign(winding[0])), expected_sign)
            self.assertEqual(float(winding[1]), 0.0)
            self.assertEqual(float(winding[2]), 0.0)
        renderer = (ROOT / "src/openrct2-ui/drawing/engines/opengl/FirstPersonGLRenderer.cpp").read_text(encoding="utf-8")
        self.assertIn("std::copysign(float(axis + 1), cross[axis])", renderer)
        self.assertIn("uOwnerSidePass", renderer)

    def test_actual_opposite_tile_sides_ignore_wrong_native_ordinal(self):
        # The two clicked pairs lie on x=2752 and x=2976, respectively.
        # Native paint order is the OPPOSITE of viewing side in one case.
        # Neither side may disappear; the front-facing authored material
        # must replace the rear-facing one at identical physical depth.
        cases = (
            # wallcf16 vs wallmm17, selected at x=2752:
            (2752.0, 4448.0, 4480.0, 120.0, 128.0,
             9, 6, 9),
            # wallbr32 vs wallbr8, selected at x=2976:
            (2976.0, 4064.0, 4096.0, 112.0, 120.0,
             9, 6, 6),
        )
        for x, y0, y1, z0, z1, left_colour, right_colour, _ in cases:
            left = quad_x(x, y0, y1, z0, z1, left_colour, -1)
            right = quad_x(x, y0, y1, z0, z1, right_colour, +1)
            for side in (-1, 1):
                eye = (x + 60.0 * side, (y0 + y1) / 2.0, (z0 + z1) / 2.0)
                forward = (-float(side), 0.0, 0.0)
                right_axis = (0.0, -float(side), 0.0)
                expected = right_colour if side == 1 else left_colour
                for draw_order in ((left, right), (right, left)):
                    with self.subTest(x=x, side=side,
                                      first=draw_order[0]["paintOrdinal"][0]):
                        pixels = self.render_overlap(
                            0, eye, surfaces=draw_order, side_passes=(1, 2),
                            forward_override=forward, right_override=right_axis)
                        self.assertEqual(
                            int(pixels[HEIGHT // 2, WIDTH // 2]), expected)

    def test_actual_mismatched_wall_heights_preserve_uncovered_sections(self):
        # Exact clicked wallcf16 (z120..136) vs wallmm17 (z112..128).
        # Rear-facing material must remain visible below the front plane;
        # only the overlapping z120..128 band uses directional ownership.
        left = quad_x(2752, 4448, 4480, 112, 128, 9, -1)
        right = quad_x(2752, 4448, 4480, 120, 136, 6, +1)
        pixels = self.render_overlap(
            0, (2812.0, 4464.0, 124.0),
            surfaces=(right, left), side_passes=(1, 2),
            forward_override=(-1.0, 0.0, 0.0),
            right_override=(0.0, -1.0, 0.0))
        self.assertEqual(int(pixels[HEIGHT//2, WIDTH//2]), 6)
        self.assertEqual(int(pixels[HEIGHT//2 - 7, WIDTH//2]), 9)
        self.assertEqual(int(pixels[HEIGHT//2 + 6, WIDTH//2]), 6)

    def test_cutout_front_material_reveals_back_material_without_holes(self):
        # Opposing wall face on each side of x=2752, using actual native
        # indexed artwork alpha=0 semantics, not synthetic solid colours.
        # Front railing cutouts must reveal the opaque rear wall rather than
        # become holes or invert the entire material's ownership.
        right = quad_x(2752, 4448, 4480, 112, 144, 6, 1)
        left = quad_x(2752, 4448, 4480, 112, 144, 9, -1)
        self.sprites.write(bytes([6, 0, 6, 0] + [9] * 4))
        try:
            pixels = self.render_overlap(
                0, (2812.0, 4464.0, 128.0),
                surfaces=(right, left),
                side_passes=(1, 2),
                forward_override=(-1.0, 0.0, 0.0),
                right_override=(0.0, -1.0, 0.0))
            self.assertGreater(np.count_nonzero(pixels == 6), 20)
            self.assertGreater(np.count_nonzero(pixels == 9), 20)
        finally:
            self.sprites.write(bytes([6] * 4 + [9] * 4))

    def test_true_nearer_wall_beats_later_native_ordinal(self):
        # Native paint order decides ties only. A later ordinal at an
        # objectively farther world plane must never paint over a nearer wall.
        near = quad(112, 128, 6, True, y=4544.0)
        far = quad(120, 136, 9, False, y=4548.0)
        for yaw, eye in (
            (0.0, (2640.0, 4500.0, 124.0)),
            (0.718, (2597.100, 4506.453, 124.764)),
        ):
            with self.subTest(yaw=yaw):
                pixels = self.render_overlap(yaw, eye, surfaces=(near, far))
                self.assertEqual(int(pixels[HEIGHT//2, WIDTH//2]), 6)

    def test_ordinary_opaque_depth_still_blocks_coplanar_artwork(self):
        # This is the guard against fixing wall z-fighting by letting native
        # artwork bleed through nearer terrain, coaster structures or walls.
        mask = np.ones((HEIGHT, WIDTH), dtype=np.float32)
        mask[:, :WIDTH // 2] = 0.01
        self.opaque_depth.write(mask.tobytes())
        try:
            pixels = self.render_overlap(0.0, (2640.0, 4500.0, 124.0))
            self.assertEqual(int(pixels[HEIGHT//2, WIDTH//2 - 7]), 0)
            self.assertEqual(int(pixels[HEIGHT//2, WIDTH//2 + 7]), 9)
        finally:
            self.opaque_depth.write(
                np.ones((HEIGHT, WIDTH), dtype=np.float32).tobytes())

    def test_later_native_wall_wins_overlap_without_erasing_uncovered_rail(self):
        views = [
            (0.0, (2640.0, 4500.0, 124.0)),
            (0.015, (2640.0, 4500.0, 124.0)),
            (-0.015, (2640.0, 4500.0, 124.0)),
            (0.07, (2640.0, 4500.0, 124.0)),
            (-0.07, (2640.0, 4500.0, 124.0)),
            # Close to the exact clicked ray: x/y camera direction
            # ~= 0.655/0.742, with a large world-space x displacement.
            (0.723, (2597.100, 4506.453, 124.764)),
            (0.718, (2597.100, 4506.453, 124.764)),
        ]
        for yaw, eye in views:
            with self.subTest(yaw=yaw, eye=eye):
                pixels = self.render_overlap(yaw, eye)
                print("GL_OWNER_DIAGNOSTICS", yaw,
                      "center", int(pixels[HEIGHT // 2, WIDTH // 2]),
                      "colours", np.unique(pixels, return_counts=True),
                      file=sys.stderr)
                # Camera center ray passes through the actual overlap.
                self.assertEqual(int(pixels[HEIGHT // 2, WIDTH // 2]), 9)
                self.assertGreater(np.count_nonzero(pixels == 9), 0)
                self.assertGreater(np.count_nonzero(pixels == 6), 0)
                # Every interior pixel has one analytically known ray/plane
                # intersection. Compare the WHOLE overlap, not just its
                # centre: z-fighting can produce narrow stripes of wrong
                # native artwork without changing that centre pixel.
                forward = (math.sin(yaw), math.cos(yaw), 0.0)
                right = (math.cos(yaw), -math.sin(yaw), 0.0)
                mismatches = []
                checked = {6: 0, 9: 0}
                for row in range(HEIGHT):
                    v = (2.0 * (row + 0.5) / HEIGHT) - 1.0
                    for column in range(WIDTH):
                        u = (2.0 * (column + 0.5) / WIDTH) - 1.0
                        ry = forward[1] + u * right[1]
                        if ry <= 0:
                            continue
                        t = (4544.0 - eye[1]) / ry
                        x = eye[0] + t * (forward[0] + u * right[0])
                        z = eye[2] + t * v
                        if not (2624.5 < x < 2655.5):
                            continue
                        expected = (
                            6 if 112.5 < z < 119.5
                            else 9 if 120.5 < z < 135.5
                            else 0
                        )
                        if expected == 0:
                            continue
                        checked[expected] += 1
                        actual = int(pixels[row, column])
                        if actual != expected:
                            mismatches.append((row, column, expected, actual))
                print("GL_OWNER_PIXEL_COVERAGE", yaw,
                      checked, "mismatches", len(mismatches),
                      "sample", mismatches[:8], file=sys.stderr)
                self.assertGreater(checked[6], 0)
                self.assertGreater(checked[9], 0)
                self.assertEqual(mismatches, [])


if __name__ == "__main__":
    unittest.main()
