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
        for x in ("LoadLibraryExW(", "StartFrameCapture(nullptr, nullptr)", 
                  "EndFrameCapture(nullptr, nullptr)", "status=saved"):
            self.assertIn(x,api)
    def test_portable_dependency_is_in_build_and_artifact(self):
        self.assertIn("stage-first-person-renderdoc.ps1",
            read("src/openrct2-win/openrct2-win.vcxproj"))
        self.assertIn("bin/renderdoc.dll",
            read(".github/workflows/first-person-reconstruction.yml"))
if __name__=="__main__": unittest.main()
