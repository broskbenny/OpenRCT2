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


def quad(z0, z1, colour, opposite_diagonal):
    x0, x1, y = 2624.0, 2656.0, 4544.0
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
    vertices["flags"] = 4 | 16  # coplanar owner, solid indexed palette colour
    vertices["palettes"][:, 3] = colour
    vertices["maskLayer"] = -1
    vertices["paintOrdinal"] = colour
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
            (1, 1, 1), components=1, data=b"\x01", dtype="u1")
        cls.palettes = cls.ctx.texture(
            (256, 256), components=1, data=bytes(256 * 256), dtype="u1")
        cls.ctx.enable(moderngl.DEPTH_TEST)
        cls.ctx.disable(moderngl.CULL_FACE)
        cls.ctx.disable(moderngl.BLEND)
        cls.ctx.depth_func = "<="

    @classmethod
    def tearDownClass(cls):
        cls.ctx.release()

    def render_overlap(self, yaw, eye):
        ctx, program = self.ctx, self.shader
        self.target.use()
        ctx.viewport = (0, 0, WIDTH, HEIGHT)
        self.colour.write(bytes(WIDTH * HEIGHT * 4))
        self.target.clear(depth=1.0)
        # Diagnostic: first prove shader-visible wall pixels are emitted
        # independent of the depth attachment. Re-enable depth after this
        # becomes a nonzero offscreen reference.
        ctx.disable(moderngl.DEPTH_TEST)
        self.sprites.use(location=0)
        self.palettes.use(location=1)
        self.opaque_depth.use(location=2)
        for name, location in (
            ("uSprites", 0), ("uPalettes", 1), ("uOpaqueDepth", 2)):
            if name in program:
                program[name].value = location
        forward = (math.sin(yaw), math.cos(yaw), 0)
        right = (math.cos(yaw), -math.sin(yaw), 0)
        for name, value in (
            ("uEye", eye),
            ("uForward", forward), ("uRight", right),
            ("uUp", (0.0, 0.0, 1.0)),
            ("uNearFar", (1.0, 10000.0)), ("uFocal", (1.0, 1.0)),
            ("uPeeling", False), ("uPeelStage", 0)):
            if name in program:
                program[name].value = value
        # The lower paint ordinal rail material comes first. Its top eight
        # units overlap the primary wall, whose greater ordinal must win.
        for vertices in (quad(112, 128, 6, True), quad(120, 136, 9, False)):
            buf = ctx.buffer(vertices.tobytes())
            vao = ctx.vertex_array(
                program, [(buf, FORMAT, *ATTRIBUTES)])
            vao.render(mode=moderngl.TRIANGLES)
            vao.release()
            buf.release()
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
        self.assertEqual(int(pixels[HEIGHT//2, WIDTH//2]), 920)
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


if __name__ == "__main__":
    unittest.main()
