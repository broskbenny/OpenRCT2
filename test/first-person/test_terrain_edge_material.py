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

    def test_mirrored_tile_uvs_match_both_shared_isometric_edges(self):
        compiler = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++")
        if not compiler:
            self.skipTest("C++ compiler not installed")
        code = r"""
#include "src/openrct2/paint/FirstPersonTerrainEdgeMaterial.h"
#include <array>
#include <cassert>
using namespace OpenRCT2::Paint;
struct UV { float u, v; };
int main() {
    // Parallelogram in isometric image space; adjacent physical edges
    // must sample precisely equal source texels, without any bitmap rewrite.
    const std::array<UV,4> native{{
        {32,4}, {63,20}, {32,36}, {1,20}
    }};
    auto left=native, right=native, above=native, below=native;
    MirrorFirstPersonFlatTileUV(left,false,false);
    MirrorFirstPersonFlatTileUV(right,true,false);
    MirrorFirstPersonFlatTileUV(above,false,false);
    MirrorFirstPersonFlatTileUV(below,false,true);
    for (int i=0;i<2;++i) {
        const auto e0=left[i==0?1:2], e1=right[i==0?0:3];
        assert(e0.u==e1.u && e0.v==e1.v);
        const auto f0=above[i==0?3:2], f1=below[i==0?0:1];
        assert(f0.u==f1.u && f0.v==f1.v);
    }
    auto doubleMirror=native;
    MirrorFirstPersonFlatTileUV(doubleMirror,true,true);
    assert(doubleMirror[0].u==native[2].u);
    assert(doubleMirror[0].v==native[2].v);
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/"mirror.cpp"
            executable=Path(tmp)/"mirror"
            source.write_text(code,encoding="utf-8")
            built=subprocess.run(
                [compiler,"-std=c++20","-I",str(ROOT),str(source),
                 "-o",str(executable)],capture_output=True,text=True)
            self.assertEqual(built.returncode,0,built.stderr)
            checked=subprocess.run([str(executable)],capture_output=True,text=True)
            self.assertEqual(checked.returncode,0,checked.stderr)

    def test_native_terrain_and_path_materials_do_not_rewrite_source_artwork(self):
        terrain=(ROOT / "src/openrct2/paint/FirstPersonRenderer.World.inc").read_text()
        path=(ROOT / "src/openrct2/paint/FirstPersonRenderer.MapPath.inc").read_text()
        material=(ROOT / "src/openrct2/paint/FirstPersonTerrainEdgeMaterial.h").read_text()
        self.assertNotIn("PeriodizeFirstPersonFlatTileMaterial", terrain)
        self.assertNotIn("PeriodizeFirstPersonFlatTileMaterial", path)
        self.assertNotIn("PeriodizeFirstPersonFlatTileMaterial", material)
        self.assertIn("BuildFirstPersonTerrainEdgeMaterial(", terrain)
        self.assertIn("BuildFirstPersonTerrainEdgeMaterial(", path)
        self.assertIn("MirrorFirstPersonFlatTileUV(v, (tx & 1)", terrain)
        self.assertIn("MirrorFirstPersonFlatTileUV(", material)
        self.assertIn("FillFirstPersonOpaqueStructuralMaterial(", material)
        # The footprint, UV origin and sprite crop remain authoritative.
        self.assertIn("GetFirstPersonPathDeckImageOffset(path, sourceRotation)", path)
        self.assertIn("EmitQuad(surface, vertices)", path)

    def test_collision_occupancy_is_never_promoted_as_uncertified_visible_geometry(self):
        appearance=(ROOT/"src/openrct2/paint/FirstPersonSmallSceneryAppearance.h").read_text()
        collision=(ROOT/"src/openrct2/paint/FirstPersonSmallSceneryCollision.h").read_text()
        self.assertIn("if (reconstructedHull->roundTripViewCount < 4)", appearance)
        self.assertNotIn(
            "FirstPersonSmallSceneryCanUseSilhouetteRefinement(entry)\\n"
            "            && reconstructedHull->roundTripViewCount < 4",
            appearance)
        self.assertIn("BuildFirstPersonOccupancyHull(", collision)
        # The low-quality physical occupancy volume remains available to
        # walking collision, never an artwork-bearing public 3D shell.
        self.assertIn("BuildFirstPersonVisualHullBoundaryFaces(", appearance)

    def test_live_terrain_only_uses_shared_immutable_material(self):
        world=(ROOT/"src/openrct2/paint/FirstPersonRenderer.World.inc").read_text()
        core=(ROOT/"src/openrct2/paint/FirstPersonRenderer.Core.inc").read_text()
        self.assertIn("BuildFirstPersonTerrainEdgeMaterial(",world)
        self.assertIn("cache.ground.persistentPixels = residentMaterial.pixels",world)
        self.assertIn("vertex.u += float(residentMaterial.padding)",world)
        self.assertIn("std::unordered_map<ImageIndex, FirstPersonTerrainResidentMaterial> materials",core)


if __name__ == "__main__":
    unittest.main()
