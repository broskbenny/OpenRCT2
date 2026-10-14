"""Production terrain-only native edge material filling: no flat fallback wedges."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class NativeTerrainEdgeMaterialTests(unittest.TestCase):
    def test_native_pixels_extend_without_modifying_observed_palette(self):
        compiler = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++")
        if not compiler:
            self.skipTest("C++ compiler not installed")
        code = r"""
#include "src/openrct2/paint/FirstPersonTerrainEdgeMaterial.h"
#include <cassert>
#include <vector>
using namespace OpenRCT2::Paint;
int main() {
    std::vector<uint8_t> native{
       0, 0, 0, 0, 0,
       0, 0, 41,0, 0,
       0, 41,53,53,0,
       0, 0, 53,0, 0,
       0, 0, 0, 0, 0
    };
    const auto repaired = BuildFirstPersonTerrainEdgeMaterial(native,5,5,3);
    assert(repaired.has_value());
    assert(repaired->width == 11 && repaired->height == 11);
    for(auto pixel : repaired->pixels) assert(pixel != 0);
    for(int y=0;y<5;++y)
      for(int x=0;x<5;++x) {
        auto old = native[size_t(y)*5+size_t(x)];
        auto newPixel = repaired->pixels[size_t(y+3)*11+size_t(x+3)];
        if(old != 0) assert(newPixel == old);
      }
    assert(!BuildFirstPersonTerrainEdgeMaterial({0,0,0,0},2,2,1));
    assert(!BuildFirstPersonTerrainEdgeMaterial({5,5,5},2,2,1));
    assert(!BuildFirstPersonTerrainEdgeMaterial(native,5,5,400));
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            src=Path(tmp)/"test.cpp"
            exe=Path(tmp)/"test"
            src.write_text(code,encoding="utf-8")
            compiled=subprocess.run(
                [compiler,"-std=c++20","-I",str(ROOT),str(src),"-o",str(exe)],
                capture_output=True,text=True)
            self.assertEqual(compiled.returncode,0,compiled.stderr)
            result=subprocess.run([str(exe)],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)

    def test_live_terrain_only_uses_shared_immutable_material(self):
        world=(ROOT/"src/openrct2/paint/FirstPersonRenderer.World.inc").read_text()
        core=(ROOT/"src/openrct2/paint/FirstPersonRenderer.Core.inc").read_text()
        self.assertIn("BuildFirstPersonTerrainEdgeMaterial(",world)
        self.assertIn("cache.ground.persistentPixels = residentMaterial.pixels",world)
        self.assertIn("vertex.u += float(residentMaterial.padding)",world)
        self.assertIn("std::unordered_map<ImageIndex, FirstPersonTerrainResidentMaterial> materials",core)


if __name__ == "__main__":
    unittest.main()
