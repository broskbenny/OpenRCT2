"""Regress lossless GL 3.3 R8UI sprite-atlas growth and GPU residency."""
import ctypes as C
from ctypes.util import find_library
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
CACHE = ROOT / "src/openrct2-ui/drawing/engines/opengl/TextureCache.cpp"
RENDERER = ROOT / "src/openrct2-ui/drawing/engines/opengl/FirstPersonGLRenderer.cpp"


class AtlasGrowthTests(unittest.TestCase):
    def test_production_atlas_never_readbacks_to_reallocate(self):
        src = CACHE.read_text(encoding="utf-8")
        fn = src[src.index("void TextureCache::EnlargeAtlasesTexture("):
                 src.index("AtlasTextureInfo TextureCache::LoadImageTexture(", src.index("void TextureCache::EnlargeAtlasesTexture("))]
        self.assertNotIn("glGetTexImage,", fn)
        self.assertNotIn("glBindBuffer, GL_PIXEL_PACK_BUFFER", fn)
        self.assertIn("glFramebufferTextureLayer", fn)
        self.assertIn("glCopyTexSubImage3D", fn)
        self.assertIn("glCheckFramebufferStatus", fn)
        self.assertIn("_atlasesTextureIndices = newIndices;", fn)
        self.assertIn("GLuint(_atlasesTextureIndicesLimit)", fn)
        api = (ROOT / "src/openrct2-ui/drawing/engines/opengl/OpenGLAPI.h").read_text()
        # SDL_opengl.h declares the GL 1.2 function on Windows. Temporarily
        # rename its prototype, then restore the name before declaring the
        # dynamically loaded function pointer; otherwise MSVC C2365/C2659.
        self.assertLess(api.index("#define glCopyTexSubImage3D __static__glCopyTexSubImage3D"),
                        api.index("#include <SDL_opengl.h>"))
        self.assertLess(api.index("#include <SDL_opengl.h>"),
                        api.index("#undef glCopyTexSubImage3D"))
        self.assertLess(api.index("#undef glCopyTexSubImage3D"),
                        api.index('#include "OpenGLAPIProc.h"'))
        loader = (ROOT / "src/openrct2-ui/drawing/engines/opengl/OpenGLAPIProc.h").read_text()
        self.assertIn("PFNGLFRAMEBUFFERTEXTURELAYERPROC, glFramebufferTextureLayer", loader)
        self.assertIn("PFNGLCOPYTEXSUBIMAGE3DPROC, glCopyTexSubImage3D", loader)
        renderer = RENDERER.read_text(encoding="utf-8")
        self.assertIn("_atlasSource != &textures", renderer)
        self.assertIn("FP_ATLAS_GROW preservedRegions=", renderer)
        self.assertNotIn("if (_atlasHandle != currentAtlas)\n        {\n            DiscardAllRegionBuffers();", renderer)

    @unittest.skipUnless(sys.platform.startswith("linux"), "EGL GPU test runs on headless Linux")
    def test_integer_atlas_layers_round_trip_via_gpu_copy(self):
        # No OpenGL Python package required. Test the actual GL 3.3 operations
        # against a surfaceless Mesa/EGL implementation, preserving pixels in
        # two occupied array layers when the atlas grows to four.
        if not find_library("EGL") or not find_library("GL"):
            self.skipTest("EGL/GL runtime not available")
        egl = C.CDLL(find_library("EGL"))
        gl_lib = C.CDLL(find_library("GL"))
        P, U, I = C.c_void_p, C.c_uint, C.c_int

        def ef(name, restype, args):
            fn = getattr(egl, name)
            fn.restype, fn.argtypes = restype, args
            return fn

        get = ef("eglGetProcAddress", P, [C.c_char_p])
        proc = get(b"eglGetPlatformDisplayEXT")
        if not proc:
            self.skipTest("Surfaceless EGL unavailable")
        get_display = C.CFUNCTYPE(P, U, P, P)(proc)
        display = get_display(0x31DD, None, None)
        major, minor = I(), I()
        if not ef("eglInitialize", U, [P, P, P])(
            display, C.byref(major), C.byref(minor)):
            self.skipTest("Surfaceless EGL could not initialize")
        self.assertTrue(ef("eglBindAPI", U, [U])(0x30A2))
        choose = ef("eglChooseConfig", U, [P, P, P, I, P])
        attrs = (I * 7)(0x3033, 0x0001, 0x3040, 0x0008, 0x3024, 8, 0x3038)
        config, count = P(), I()
        if not choose(display, attrs, C.byref(config), 1, C.byref(count)) or count.value == 0:
            self.skipTest("OpenGL Pbuffer config unavailable")
        context = ef("eglCreateContext", P, [P, P, P, P])(
            display, config, None, (I * 5)(0x3098, 3, 0x30FB, 3, 0x3038))
        if not context:
            self.skipTest("OpenGL 3.3 context unavailable")
        self.assertTrue(ef("eglMakeCurrent", U, [P, P, P, P])(
            display, None, None, context))

        def gl(name, result, args):
            proc = get(name.encode("ascii"))
            if proc:
                return C.CFUNCTYPE(result, *args)(proc)
            fn = getattr(gl_lib, name)
            fn.restype, fn.argtypes = result, args
            return fn

        target, fmt, red, unsigned = 0x8C1A, 0x8232, 0x8D94, 0x1401
        framebuffer, colour = 0x8CA8, 0x8CE0
        gen = gl("glGenTextures", None, [I, P])
        bind = gl("glBindTexture", None, [U, U])
        allocate = gl("glTexImage3D", None, [U, I, I, I, I, I, I, U, U, P])
        upload = gl("glTexSubImage3D", None, [U, I, I, I, I, I, I, I, U, U, P])
        gen_fbo = gl("glGenFramebuffers", None, [I, P])
        bind_fbo = gl("glBindFramebuffer", None, [U, U])
        attach = gl("glFramebufferTextureLayer", None, [U, U, U, I, I])
        check = gl("glCheckFramebufferStatus", U, [U])
        readbuf = gl("glReadBuffer", None, [U])
        copy = gl("glCopyTexSubImage3D", None, [U, I, I, I, I, I, I, I, I])
        read = gl("glGetTexImage", None, [U, I, U, U, P])
        error = gl("glGetError", U, [])
        textures = (U * 2)()
        gen(2, textures)
        bind(target, textures[0])
        allocate(target, 0, fmt, 4, 4, 2, 0, red, unsigned, None)
        for layer in range(2):
            pixels = (C.c_ubyte * 16)(*[17 + 100 * layer + x for x in range(16)])
            upload(target, 0, 0, 0, layer, 4, 4, 1, red, unsigned, pixels)
        bind(target, textures[1])
        allocate(target, 0, fmt, 4, 4, 4, 0, red, unsigned, None)
        fbo = U()
        gen_fbo(1, C.byref(fbo))
        bind_fbo(framebuffer, fbo)
        readbuf(colour)
        for layer in range(2):
            attach(framebuffer, colour, textures[0], 0, layer)
            self.assertEqual(check(framebuffer), 0x8CD5)
            copy(target, 0, 0, 0, layer, 0, 0, 4, 4)
            self.assertEqual(error(), 0)
        values = (C.c_ubyte * 64)()
        read(target, 0, red, unsigned, values)
        self.assertEqual(list(values[:32]),
                         [17 + i for i in range(16)] + [117 + i for i in range(16)])


if __name__ == "__main__":
    unittest.main()
