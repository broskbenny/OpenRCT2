"""Compile and execute the real map-driven small-scenery placement function."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "src/openrct2/paint/FirstPersonRenderer.SmallScenery.inc"


class MapSceneryPlacementCppTest(unittest.TestCase):
    def test_compiled_placement_color_and_region_identity(self):
        compiler = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++") or shutil.which("cl")
        if not compiler:
            self.skipTest("C++ compiler unavailable")
        src = SOURCE.read_text(encoding="utf-8")
        first = src.index("        [[nodiscard]] bool AppendFirstPersonMapSmallScenery(")
        last = src.index("        void StartFirstPersonSmallSceneryAssetJob(", first)
        actual = src[first:last]
        fixture = """
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

constexpr int32_t kCoordsXYStep=32;
struct CoordsXY {int32_t x{},y{};};
struct FirstPersonVec3 {float x{},y{},z{};};
struct FirstPersonVertex {FirstPersonVec3 world{};};
struct ImageId {
    uint32_t index{};
    int primary=-1, secondary=-1, tertiary=-1;
    ImageId(uint32_t i=0):index(i){}
    uint32_t GetIndex() const {return index;}
    ImageId WithIndex(uint32_t i) const {auto a=*this;a.index=i;return a;}
    ImageId WithPrimary(int c) const {auto a=*this;a.primary=c;return a;}
    ImageId WithSecondary(int c) const {auto a=*this;a.secondary=c;return a;}
    ImageId WithTertiary(int c) const {auto a=*this;a.tertiary=c;return a;}
};
enum class SmallSceneryFlag {hasPrimaryColour,hasSecondaryColour,hasTertiaryColour};
struct SmallSceneryFlags {
    uint32_t bits{};
    bool has(SmallSceneryFlag f) const {return (bits & (1u<<uint32_t(f)))!=0;}
};
struct SmallSceneryEntry {uint32_t image{};SmallSceneryFlags flags{};bool shareRotation=false;};
struct SmallSceneryElement {
    const SmallSceneryEntry* entry{};
    uint8_t direction{},stage{};
    int32_t baseZ{};
    int primary{},secondary{},tertiary{};
    const SmallSceneryEntry* getEntry() const {return entry;}
    uint8_t getDirection() const {return direction;}
    int32_t getBaseZ() const {return baseZ;}
    int getPrimaryColour() const {return primary;}
    int getSecondaryColour() const {return secondary;}
    int getTertiaryColour() const {return tertiary;}
};
enum class FirstPersonPaintSemanticRole:uint8_t {smallSceneryBody=18};
struct FirstPersonSurface {
    ImageId image{};
    uint8_t diagnosticSemanticRole{};
    uint64_t gpuRegion{};
    FirstPersonVec3 outwardNormal{};
    std::array<FirstPersonVertex,6> triangles{};
};
struct SmallSceneryJob {bool ready=false;std::vector<FirstPersonSurface> surfaces;};
std::unordered_map<uint64_t,std::shared_ptr<SmallSceneryJob>> _smallSceneryJobs;
uint64_t FirstPersonSmallSceneryReconstructionKey(const SmallSceneryEntry& entry,
    const SmallSceneryElement&) {return entry.image;}
bool FirstPersonMapSmallSceneryCanPublish(const SmallSceneryEntry& entry,
    const SmallSceneryElement&,uint32_t viewFlags) {
    auto it=_smallSceneryJobs.find(entry.image);
    return viewFlags==0 && it!=_smallSceneryJobs.end() && it->second->ready
        && !it->second->surfaces.empty();
}
bool FirstPersonSmallSceneryCanShareRotatedAsset(const SmallSceneryEntry& entry,
    const SmallSceneryElement&) {return entry.shareRotation;}
int FirstPersonSmallSceneryWitherStage(const SmallSceneryEntry&,
    const SmallSceneryElement& e) {return e.stage;}
uint64_t FirstPersonGpuRegionKey(int32_t x,int32_t y) {
    return (uint64_t(uint32_t(x/32+1))<<32)|uint32_t(y/32+1);
}
FirstPersonVec3 FirstPersonRotateSceneryPoint(FirstPersonVec3 p,uint8_t rot) {
    for(uint8_t i=0;i<(rot&3);++i){float x=p.x;p.x=-p.y;p.y=x;}
    return p;
}
FirstPersonVec3 FirstPersonRotateSceneryVector(FirstPersonVec3 p,uint8_t rot) {
    return FirstPersonRotateSceneryPoint(p,rot);
}
""" + actual + """
int main() {
    SmallSceneryEntry entry{120000,{},true};
    entry.flags.bits=7;
    auto model=std::make_shared<SmallSceneryJob>();
    model->ready=true;
    FirstPersonSurface local{};
    local.image=ImageId(120002);
    local.triangles[0].world={3,4,5};
    local.outwardNormal={1,0,0};
    model->surfaces.push_back(local);
    _smallSceneryJobs.emplace(entry.image,model);
    SmallSceneryElement object{&entry,1,2,40,5,6,7};
    std::vector<FirstPersonSurface> placed;
    assert(AppendFirstPersonMapSmallScenery(placed,{64,96},object,0));
    assert(placed.size()==1);
    assert(placed[0].image.index==120002);
    assert(placed[0].image.primary==5);
    assert(placed[0].image.secondary==6);
    assert(placed[0].image.tertiary==7);
    assert(placed[0].gpuRegion==FirstPersonGpuRegionKey(2,3));
    assert(placed[0].diagnosticSemanticRole==uint8_t(FirstPersonPaintSemanticRole::smallSceneryBody));
    assert(placed[0].triangles[0].world.x==60);
    assert(placed[0].triangles[0].world.y==99);
    assert(placed[0].triangles[0].world.z==45);
    assert(placed[0].outwardNormal.y==1);
    assert(AppendFirstPersonMapSmallScenery(placed,{128,192},object,0));
    assert(placed.size()==2);
    assert(model->surfaces.size()==1);
    assert(placed[1].triangles[0].world.x==124);
    assert(placed[1].triangles[0].world.y==195);
    assert(placed[1].triangles[0].world.z==45);
    assert(!AppendFirstPersonMapSmallScenery(placed,{64,96},object,2));
    entry.shareRotation=false;
    assert(AppendFirstPersonMapSmallScenery(placed,{0,0},object,0));
    assert(placed[2].triangles[0].world.x==3);
    assert(placed[2].triangles[0].world.y==4);
    assert(placed[2].triangles[0].world.z==45);
}
"""
        with tempfile.TemporaryDirectory() as directory:
            cpp = Path(directory) / "real_map_placement.cpp"
            exe = Path(directory) / ("real_map_placement.exe" if Path(compiler).stem.lower()=="cl" else "real_map_placement")
            cpp.write_text(fixture, encoding="utf-8")
            if Path(compiler).stem.lower()=="cl":
                command=[compiler,"/nologo","/std:c++20","/EHsc",str(cpp),"/Fe:"+str(exe)]
            else:
                command=[compiler,"-std=c++20","-O0",str(cpp),"-o",str(exe)]
            built=subprocess.run(command,cwd=directory,capture_output=True,text=True)
            self.assertEqual(built.returncode,0,built.stdout+built.stderr)
            ran=subprocess.run([str(exe)],cwd=directory,capture_output=True,text=True)
            self.assertEqual(ran.returncode,0,ran.stdout+ran.stderr)


if __name__=="__main__":
    unittest.main()
