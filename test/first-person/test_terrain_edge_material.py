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

    def test_flat_native_palette_tiles_have_matching_opposite_edges(self):
        compiler = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++")
        if not compiler:
            self.skipTest("C++ compiler not installed")
        code = r"""
#include "src/openrct2/paint/FirstPersonTerrainEdgeMaterial.h"
#include <array>
#include <cassert>
#include <vector>
using namespace OpenRCT2::Paint;
struct UV { float u, v; };
int main() {
    std::vector<uint8_t> source(64 * 40, 0);
    for (int y = 0; y < 40; ++y)
    for (int x = 0; x < 64; ++x)
        source[size_t(y)*64+size_t(x)] = uint8_t(3 + ((x*7+y*3) % 120));
    auto expanded = BuildFirstPersonTerrainEdgeMaterial(source, 64, 40, 4);
    assert(expanded);
    const auto originalPalette = expanded->pixels;
    const std::array<UV,4> quad{{ {8,8}, {56,8}, {56,32}, {8,32} }};
    assert(PeriodizeFirstPersonFlatTileMaterial(*expanded, quad));
    // Physical edge stitching must never resample or dither interior
    // native texels, even if a periodic edge colour differs substantially.
    for (int y=12;y<28;++y)
    for (int x=16;x<48;++x)
      assert(expanded->pixels[size_t(y+4)*expanded->width+size_t(x+4)]
          == originalPalette[size_t(y+4)*expanded->width+size_t(x+4)]);
    const auto sample = [&](int x,int y) {
        return expanded->pixels[size_t(y+4)*size_t(expanded->width)+size_t(x+4)];
    };
    // Repeated tile geometry samples equal palette colours at both physical
    // sides: no colour jump at the boundary between identical surfaces.
    for (int y=8;y<32;++y)
        assert(sample(8,y) == sample(55,y));
    for (int x=8;x<56;++x)
        assert(sample(x,8) == sample(x,31));
    for (const auto pixel : expanded->pixels) assert(pixel != 0);
    const std::array<UV,4> collapsed{{ {8,8}, {8,8}, {8,8}, {8,8} }};
    assert(!PeriodizeFirstPersonFlatTileMaterial(*expanded, collapsed));
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp)/"test.cpp"
            exe = Path(tmp)/"test"
            source.write_text(code, encoding="utf-8")
            built = subprocess.run(
                [compiler, "-std=c++20", "-I", str(ROOT),
                 str(source), "-o", str(exe)], capture_output=True, text=True)
            self.assertEqual(built.returncode, 0, built.stderr)
            checked = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(checked.returncode, 0, checked.stderr)

    def test_partial_axis_preserves_unconnected_native_border(self):
        compiler = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++")
        if not compiler:
            self.skipTest("C++ compiler not installed")
        code = r"""
#include "src/openrct2/paint/FirstPersonTerrainEdgeMaterial.h"
#include <array>
#include <cassert>
#include <vector>
using namespace OpenRCT2::Paint;
struct UV { float u, v; };
int main() {
    std::vector<uint8_t> native(64*40, 0);
    for(int y=0;y<40;++y)
    for(int x=0;x<64;++x)
        native[size_t(y)*64+x] = uint8_t(8 + y);
    auto tile = BuildFirstPersonTerrainEdgeMaterial(native,64,40,4);
    assert(tile);
    const std::array<UV,4> uv{{ {8,8}, {56,8}, {56,32}, {8,32} }};
    assert(PeriodizeFirstPersonFlatTileMaterial(*tile,uv,1));
    const auto sample = [&](int x,int y) {
        return tile->pixels[size_t(y+4)*tile->width+size_t(x+4)];
    };
    for(int y=8;y<32;++y) assert(sample(8,y) == sample(55,y));
    // Only the connected X faces repeat. Y borders still use the native
    // entrance/edge art and have not been forced into a tiled plaza.
    assert(sample(30,8) != sample(30,31));
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/"test.cpp"
            binary=Path(tmp)/"test"
            source.write_text(code,encoding="utf-8")
            built=subprocess.run(
                [compiler,"-std=c++20","-I",str(ROOT),str(source),
                 "-o",str(binary)], capture_output=True,text=True)
            self.assertEqual(built.returncode,0,built.stderr)
            checked=subprocess.run([str(binary)],capture_output=True,text=True)
            self.assertEqual(checked.returncode,0,checked.stderr)

    def test_flat_terrain_and_full_plazas_are_periodized_once_per_image(self):
        world=(ROOT/"src/openrct2/paint/FirstPersonRenderer.World.inc").read_text()
        path=(ROOT/"src/openrct2/paint/FirstPersonRenderer.MapPath.inc").read_text()
        self.assertIn("if (slope == kTileSlopeFlat)", world)
        self.assertIn("PeriodizeFirstPersonFlatTileMaterial(*expanded, v)", world)
        self.assertIn("if (FirstPersonPathUsesFullTileDeck(path))", path)
        self.assertIn("PeriodizeFirstPersonFlatTileMaterial(", path)
        self.assertIn("*expanded, wholeTile, periodicAxes", path)
        self.assertIn("(connected & 0x05u) == 0x05u", path)
        self.assertIn("(connected & 0x0Au) == 0x0Au", path)
        self.assertIn("path.isSloped() ? 0u", path)
        self.assertIn("bool deckMaterialResolved = false;", path)
        self.assertEqual(path.count("const FirstPersonTerrainResidentMaterial* deckMaterial"), 1)
        self.assertIn("if (!deckMaterialResolved)", path)
        self.assertIn("surface.coplanarOwner = true;", path)

    def test_live_terrain_only_uses_shared_immutable_material(self):
        world=(ROOT/"src/openrct2/paint/FirstPersonRenderer.World.inc").read_text()
        core=(ROOT/"src/openrct2/paint/FirstPersonRenderer.Core.inc").read_text()
        self.assertIn("BuildFirstPersonTerrainEdgeMaterial(",world)
        self.assertIn("cache.ground.persistentPixels = residentMaterial.pixels",world)
        self.assertIn("vertex.u += float(residentMaterial.padding)",world)
        self.assertIn("std::unordered_map<ImageIndex, FirstPersonTerrainResidentMaterial> materials",core)


if __name__ == "__main__":
    unittest.main()
