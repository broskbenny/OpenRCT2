"""Zero-setup RenderDoc inspector capture guard."""
from pathlib import Path
import unittest
ROOT = Path(__file__).resolve().parents[2]
def read(p): return (ROOT/p).read_text(encoding="utf-8")
class RenderDocClickCaptureTests(unittest.TestCase):
    def test_click_arms_capture(self):
        code=read("src/openrct2-ui/FirstPersonController.cpp")
        self.assertIn("Paint::RequestFirstPersonInspectorPick();",code)
        self.assertIn("FirstPersonRenderDocCapture::RequestOnInspectorClick();",code)
    def test_entire_frame_is_captured(self):
        code=read("src/openrct2-ui/drawing/engines/opengl/OpenGLDrawingEngine.cpp")
        self.assertIn("FirstPersonRenderDocCapture::BeginFrame();",code)
        self.assertIn("Display();\n        FirstPersonRenderDocCapture::EndFrame();",code)
    def test_hook_loads_at_process_startup(self):
        code=read("src/openrct2-win/openrct2-win.cpp")
        self.assertIn("FirstPersonRenderDocCapture::InitialiseAtStartup();",code)
        api=read("src/openrct2-ui/FirstPersonRenderDocCapture.h")
        self.assertIn("LOAD_WITH_ALTERED_SEARCH_PATH", api)
        self.assertNotIn("LOAD_LIBRARY_SEARCH_DEFAULT_DIRS", api)
        self.assertIn("renderdoc.dll-load-failed-win32-", api)
        self.assertIn("GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_PIN", api)
        self.assertIn("renderdoc.dll-pin-failed-win32-", api)
        self.assertLess(api.index("GET_MODULE_HANDLE_EX_FLAG_PIN"), api.index("getApi(eRENDERDOC_API_Version_1_1_2"))
        for x in ("LoadLibraryExW(", "StartFrameCapture(nullptr, nullptr)", 
                  "EndFrameCapture(nullptr, nullptr)", "status=saved"):
            self.assertIn(x,api)
    def test_portable_dependency_is_in_build_and_artifact(self):
        self.assertIn("stage-first-person-renderdoc.ps1",
            read("src/openrct2-win/openrct2-win.vcxproj"))
        workflow = read(".github/workflows/first-person-reconstruction.yml")
        self.assertIn("bin/renderdoc.dll", workflow)
        self.assertIn("Probe RenderDoc DLL load and API before compilation", workflow)
        probe = read("test/first-person/probe_renderdoc_windows.ps1")
        self.assertIn("FP_RENDERDOC_PROBE", probe)
        self.assertIn("stage=load-enter", probe)
        self.assertIn("stage=getapi-enter", probe)
        self.assertIn("stage=success-before-shutdown", probe)
        self.assertIn("809da38e3867d9fd09cc5c30dd5310500dee75e999166d7a14ad1ef6e9eca65d",probe)
        self.assertNotIn("FreeLibrary(",probe)
        self.assertIn("vendor-renderdoc:", workflow)
        self.assertIn("needs: vendor-renderdoc", workflow)
        ps = read("scripts/stage-first-person-renderdoc.ps1")
        self.assertIn("Copy-Item -LiteralPath $sourceDll", ps)
        self.assertIn("third_party", ps)
        self.assertNotIn("Tls12", ps)
        self.assertNotIn("Invoke-WebRequest", ps)
        self.assertNotIn("Compression.ZipFile", ps)
        vendor = read("scripts/vendor_first_person_renderdoc.py")
        self.assertIn("0x8664", vendor)
        self.assertIn("RenderDoc_1.46_64.zip", vendor)
if __name__=="__main__": unittest.main()
