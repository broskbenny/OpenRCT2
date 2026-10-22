"""Four-view structural beam reconstruction must simplify lattices without weaker image evidence."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class StructuralBeamReconstructionTest(unittest.TestCase):
    def test_compiled_axial_beam_inference_and_four_view_certificate(self):
        compiler = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++")
        if not compiler:
            self.skipTest("C++ compiler not installed")
        code = r"""
#include "src/openrct2/paint/FirstPersonVisualHull.h"
#include <array>
#include <cassert>
#include <cstddef>
#include <vector>
using namespace OpenRCT2::Paint;

int main() {
    FirstPersonVisualHull hull{};
    hull.valid = true;
    hull.step = 1.0f;
    hull.sizeForward = hull.sizeRight = hull.sizeUp = 16;
    hull.occupied.assign(16*16*16, 0);
    const auto mark = [&](int x,int y,int z) {
        hull.occupied[(size_t(z)*16+size_t(y))*16+size_t(x)] = 1;
    };
    // Four structural posts and two long rails; their exact 3D support is
    // backed by intersected observations rather than guessed fill.
    for (int z=1;z<15;++z) {
        for(int x : {2,12})
        for(int y : {2,12})
            mark(x,y,z);
    }
    for(int x=2;x<=12;++x)
    for(int y : {2,12})
    for(int z : {3,12})
        mark(x,y,z);
    // Disconnected 3D sampling noise must not survive as exposed cuboids.
    for(int i=0;i<22;++i)
        mark(4+(i%8),4+(i/8)*3,(i*7)%16);
    const auto voxel = BuildFirstPersonVisualHullVoxelBoundaryFaces(hull);
    const auto beams = BuildFirstPersonVisualHullStructuralBeamFaces(hull,voxel.size());
    assert(!beams.empty());
    assert(beams.size()<voxel.size()*2/3);
    assert(beams.size()<=512);
    assert(BuildFirstPersonVisualHullStructuralBeamFaces(hull,63).empty());
    const auto project = [](uint8_t direction,FirstPersonVec3 p) {
        switch(direction&3) {
        case 0: return std::array<float,2>{p.x-p.y+50.0f,(p.x+p.y)*0.5f-p.z+50.0f};
        case 1: return std::array<float,2>{-p.x-p.y+50.0f,(p.x-p.y)*0.5f-p.z+50.0f};
        case 2: return std::array<float,2>{p.y-p.x+50.0f,(-p.x-p.y)*0.5f-p.z+50.0f};
        default: return std::array<float,2>{p.x+p.y+50.0f,(p.y-p.x)*0.5f-p.z+50.0f};
        }
    };
    std::vector<FirstPersonVisualHullView> views;
    const auto triangles = TriangulateFirstPersonReconstructionFaces(beams);
    for(uint8_t direction=0;direction<4;++direction) {
        FirstPersonVisualHullView view{};
        view.imageDirection=direction;
        view.observed=RenderFirstPersonReconstructionSilhouette(triangles,direction,project);
        assert(!view.observed.empty());
        views.push_back(std::move(view));
    }
    const auto cert=CertifyFirstPersonReconstruction(triangles,views,project);
    assert(cert.valid && cert.viewCount==4);
    assert(cert.minimumIntersectionOverUnion==1.0f);
    assert(cert.symmetricDifferencePixels==0);
    // Entire production inference route, including baseline rejection,
    // structural selection, face publication and native reprojection.
    FirstPersonVisualHullBounds bounds{};
    bounds.minForward=bounds.minRight=bounds.minUp=0;
    bounds.maxForward=bounds.maxRight=bounds.maxUp=16;
    bounds.step=1;
    FirstPersonVisualHullConfig config{};
    config.maximumAxisCells=32;
    config.minimumOccupiedCells=4;
    config.maximumOccupiedCells=4096;
    config.maximumGridCells=4096;
    config.minimumCandidateCoverage=0.85f;
    config.minimumObservedCoverage=0.85f;
    config.maximumEdgeError=2;
    config.allowStructuralBeamReconstruction=true;
    config.allowContinuousSurfaceRefinement=false;
    FirstPersonVisualHullAttempt attempt{};
    auto result=BuildFirstPersonVisualHull(
        views,bounds,config,project,
        [](FirstPersonVec3) { return true; },
        [&](const FirstPersonVisualHullView&,FirstPersonVec3 point) {
            const int x=int(point.x),y=int(point.y),z=int(point.z);
            return hull.contains(x,y,z);
        },&attempt);
    assert(result.valid);
    assert(result.roundTripViewCount==4);
    assert(result.structuralBeamSelected);
    assert(!result.continuousSurfaceRefined);
    assert(result.refinedFaces.size()==beams.size());
    assert(result.roundTripSymmetricDifference<result.voxelRoundTripSymmetricDifference);
    assert(attempt.stage==6);
    // Do the real inverse problem as well: obtain occupied 3D samples from
    // the four 2D silhouettes alone. No 3D ground-truth oracle is available
    // to this candidate. The independently published surface must still
    // reproduce every native view.
    FirstPersonVisualHullAttempt imageAttempt{};
    auto fromImages=BuildFirstPersonVisualHull(
        views,bounds,config,project,
        [](FirstPersonVec3) { return true; },
        [&](const FirstPersonVisualHullView& view,FirstPersonVec3 point) {
            const auto uv=project(view.imageDirection,point);
            const int px=int(std::lround(uv[0])),py=int(std::lround(uv[1]));
            for(int dy=-1;dy<=1;++dy)
            for(int dx=-1;dx<=1;++dx)
                if(view.observed.contains(px+dx,py+dy))
                    return true;
            return false;
        },&imageAttempt);
    assert(fromImages.valid);
    assert(fromImages.roundTripViewCount==4);
    assert(fromImages.minimumObservedCoverage>=0.90f);
    assert(fromImages.minimumCandidateCoverage>=0.90f);
    // Compact solids and isolated fragments do not get falsely identified
    // as beams merely because a voxel mesh has individual planar faces.
    FirstPersonVisualHull compact{};
    compact.valid=true; compact.step=1;
    compact.sizeForward=compact.sizeRight=compact.sizeUp=5;
    compact.occupied.assign(125,1);
    auto compactFaces=BuildFirstPersonVisualHullVoxelBoundaryFaces(compact);
    assert(BuildFirstPersonVisualHullStructuralBeamFaces(compact,compactFaces.size()).empty());
}
"""
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "beams.cpp"
            binary = Path(folder) / "beams"
            source.write_text(code, encoding="utf-8")
            build = subprocess.run(
                [compiler, "-std=c++20", "-O1", "-I", str(ROOT),
                 str(source), "-o", str(binary)],
                capture_output=True, text=True)
            self.assertEqual(build.returncode, 0, build.stderr)
            run = subprocess.run([str(binary)], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)

    def test_beam_candidate_is_opt_in_and_diagnostics_are_grounded(self):
        hull = (ROOT/"src/openrct2/paint/FirstPersonVisualHull.h").read_text()
        collision = (ROOT/"src/openrct2/paint/FirstPersonSmallSceneryCollision.h").read_text()
        renderer = (ROOT/"src/openrct2/paint/FirstPersonRenderer.SmallScenery.inc").read_text()
        self.assertIn("bool allowStructuralBeamReconstruction = false;", hull)
        self.assertIn("config.allowStructuralBeamReconstruction = true;", collision)
        self.assertIn("CertifyFirstPersonReconstruction(", hull)
        self.assertIn("FirstPersonRoundTripNonWorse(", hull)
        self.assertIn("result.structuralBeamSelected = true;", hull)
        self.assertIn("beamCertificate.minimumIntersectionOverUnion >= 0.85f", hull)
        self.assertIn("beamCertificate.minimumCandidateCoverage >= 0.90f", hull)
        self.assertIn("beamCertificate.minimumObservedCoverage >= 0.90f", hull)
        self.assertIn("beamCertificate.maximumEdgeError <= 2", hull)
        self.assertIn("result.structuralBeamSelected = false;", hull)
        self.assertIn("strict.structuralBeamSelected", collision)
        self.assertIn("fine.structuralBeamSelected", collision)
        self.assertIn('"structural-beam"', renderer)
        self.assertIn("|| buildEvidence.structuralBeamSelected", renderer)


if __name__ == "__main__":
    unittest.main()
