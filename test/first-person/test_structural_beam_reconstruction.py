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
#include <cstdio>
#include <cstddef>
#include <unordered_map>
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
    // A single missing source-raster cell may not split a continuous post
    // into two staircase caps; the 3D collision occupancy is still untouched.
    auto gapped=hull;
    gapped.occupied[(size_t(7)*16+2)*16+2]=0;
    const auto gappedVoxel=BuildFirstPersonVisualHullVoxelBoundaryFaces(gapped);
    const auto repaired=BuildFirstPersonVisualHullStructuralBeamFaces(
        gapped,gappedVoxel.size());
    assert(!repaired.empty());
    assert(repaired.size()==beams.size());
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
    auto fromObservedMasks = [&](int radius,FirstPersonVisualHullAttempt& certificate) {
        return BuildFirstPersonVisualHull(
            views,bounds,config,project,
            [](FirstPersonVec3) { return true; },
            [&](const FirstPersonVisualHullView& view,FirstPersonVec3 point) {
                const auto uv=project(view.imageDirection,point);
                const int px=int(std::lround(uv[0])),py=int(std::lround(uv[1]));
                for(int dy=-radius;dy<=radius;++dy)
                for(int dx=-radius;dx<=radius;++dx)
                    if(view.observed.contains(px+dx,py+dy))
                        return true;
                return false;
            },&certificate);
    };
    FirstPersonVisualHullAttempt looseAttempt{},strictAttempt{};
    auto loose=fromObservedMasks(1,looseAttempt);
    auto strict=fromObservedMasks(0,strictAttempt);
    if (!loose.valid && !strict.valid) {
        for (int radius : {0,1}) {
            auto candidate = hull;
            candidate.occupied.assign(16*16*16,0);
            for(int z=0;z<16;++z)
            for(int y=0;y<16;++y)
            for(int x=0;x<16;++x) {
                const FirstPersonVec3 pt{float(x)+0.5f,float(y)+0.5f,float(z)+0.5f};
                bool supported=true;
                for(const auto& view:views) {
                    const auto uv=project(view.imageDirection,pt);
                    const int px=int(std::lround(uv[0])),py=int(std::lround(uv[1]));
                    bool found=false;
                    for(int dy=-radius;dy<=radius;++dy)
                    for(int dx=-radius;dx<=radius;++dx)
                        if(view.observed.contains(px+dx,py+dy))
                            found=true;
                    if(!found) {supported=false;break;}
                }
                if(supported)
                    candidate.occupied[(size_t(z)*16+size_t(y))*16+size_t(x)]=1;
            }
            const auto boundary=BuildFirstPersonVisualHullVoxelBoundaryFaces(candidate);
            auto extracted=BuildFirstPersonVisualHullStructuralBeamFaces(
                candidate,boundary.size());
            std::fprintf(stderr,"RADIUS=%d original=%zu extracted=%zu\\n",
                radius,boundary.size(),extracted.size());
            if(!extracted.empty()) {
                const auto quality=CertifyFirstPersonReconstruction(
                    TriangulateFirstPersonReconstructionFaces(extracted),
                    views,project);
                std::fprintf(stderr,
                    "BEAM valid=%d iou=%.3f candidate=%.3f observed=%.3f edge=%d disagreement=%zu\\n",
                    quality.valid ? 1 : 0,
                    double(quality.minimumIntersectionOverUnion),
                    double(quality.minimumCandidateCoverage),
                    double(quality.minimumObservedCoverage),
                    quality.maximumEdgeError,quality.symmetricDifferencePixels);
            }
        }
        const auto report=[](const char* label,const FirstPersonVisualHullAttempt& attempt) {
            std::fprintf(stderr,
                "%s: stage=%u cells=%u faces=%u iou=%.3f cand=%.3f obs=%.3f edge=%d diff=%zu\\n",
                label,unsigned(attempt.stage),attempt.occupiedCells,attempt.voxelFaces,
                double(attempt.minimumIoU),double(attempt.candidateCoverage),
                double(attempt.observedCoverage),attempt.maximumEdgeError,
                attempt.disagreementPixels);
        };
        report("loose",looseAttempt);
        report("strict",strictAttempt);
    }
    const auto& fromImages=loose.valid?loose:strict;
    assert(fromImages.valid);
    assert(fromImages.roundTripViewCount==4);
    assert(fromImages.structuralBeamSelected);
    assert(fromImages.refinedFaces.size()*2<strictAttempt.voxelFaces);
    // A central phantom post overlaps the four projected bars but is not
    // present in the original structure. Removing this wholly redundant
    // volume is possible without changing ANY source-view silhouette.
    for(const auto& face:fromImages.refinedFaces) {
        bool central=true;
        for(const auto& point:face.corners)
            if(point.x<7.0f || point.x>8.0f
                || point.y<7.0f || point.y>8.0f)
                central=false;
        assert(!central);
    }
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
    // Real ornamental DATs have hundreds of source-supported axial runs.
    // A candidate search that cannot keep within its primitive budget must
    // reject before costly projection, not render 128 rods and then fail.
    auto dense = hull;
    dense.occupied.assign(16*16*16,1);
    int calls=0;
    auto noCandidate=BuildFirstPersonVisualHullSourceMatchedBeamFaces(
        dense,views,[&](uint8_t d,FirstPersonVec3 p){
            ++calls;
            return project(d,p);
        },2048);
    assert(noCandidate.empty());
    assert(calls==0);
    // Independent diagonal truss support: no axial line can explain either
    // sloped bar, but the image-constrained inferred surfaces must have
    // continuous oblique normals and use genuine world-space geometry.
    FirstPersonVisualHull diagonal{};
    diagonal.valid=true;
    diagonal.step=1;
    diagonal.sizeForward=diagonal.sizeRight=diagonal.sizeUp=16;
    diagonal.occupied.assign(16*16*16,0);
    auto addDiagonal=[&](int x,int y,int z) {
        diagonal.occupied[(size_t(z)*16+size_t(y))*16+size_t(x)]=1;
    };
    for (int k=1;k<15;++k) {
        addDiagonal(k,4,k);
        addDiagonal(k,11,15-k);
    }
    const auto voxelDiagonal=BuildFirstPersonVisualHullVoxelBoundaryFaces(diagonal);
    assert(voxelDiagonal.size()>64);
    const auto nativeTriangles=TriangulateFirstPersonReconstructionFaces(voxelDiagonal);
    std::vector<FirstPersonVisualHullView> diagonalViews;
    for (uint8_t direction=0;direction<4;++direction) {
        FirstPersonVisualHullView view{};
        view.imageDirection=direction;
        view.observed=RenderFirstPersonReconstructionSilhouette(
            nativeTriangles,direction,project);
        assert(!view.observed.empty());
        diagonalViews.push_back(std::move(view));
    }
    const auto diagonalFaces=BuildFirstPersonVisualHullSourceMatchedOrientedBeamFaces(
        diagonal,diagonalViews,project,voxelDiagonal.size());
    assert(!diagonalFaces.empty());
    assert(diagonalFaces.size()<voxelDiagonal.size()*2/3);
    bool hasObliqueNormal=false;
    for (const auto& face:diagonalFaces) {
        const auto n=face.normal;
        if (std::abs(n.x)>.2f && std::abs(n.z)>.2f)
            hasObliqueNormal=true;
        for (const auto& v:face.corners)
            assert(v.x>=0 && v.x<=16 && v.y>=0 && v.y<=16 && v.z>=0 && v.z<=16);
    }
    assert(hasObliqueNormal);
    // Native 2D strokes can have arbitrary world directions and still
    // triangulate back to real 3D, even if the raw voxel hull is incoherent.
    // Generate FOUR independent source silhouettes from a connected,
    // asymmetric structural truss; no DAT artwork or object IDs are used.
    FirstPersonVisualHull wire{};
    wire.valid=true;wire.step=1;
    wire.sizeForward=wire.sizeRight=32;wire.sizeUp=48;
    wire.occupied.assign(32*32*48,0);
    const auto markWire=[&](int x,int y,int z) {
        wire.occupied[(size_t(z)*32+size_t(y))*32+size_t(x)]=1;
    };
    for (int z=2;z<46;++z)
    for (int x : {2,8,14,20,26})
    for (int y : {3,28})
        markWire(x,y,z);
    for (int z : {5,15,25,35,44})
    for (int x=2;x<=26;++x)
    for (int y : {3,28})
        markWire(x,y,z);
    for (int k=0;k<25;++k)
    for (int y : {3,28}) {
        markWire(k+2,y,k+8);
        markWire(k+2,y,40-k);
    }
    auto originalWire=BuildFirstPersonVisualHullVoxelBoundaryFaces(wire);
    assert(originalWire.size()>200);
    const auto originalTriangles=TriangulateFirstPersonReconstructionFaces(originalWire);
    std::vector<FirstPersonVisualHullView> nativeWireViews;
    const auto wireProject=[](uint8_t d,FirstPersonVec3 p) {
        const int x=int(std::lround(p.x)),y=int(std::lround(p.y)),
            z=int(std::lround(p.z));
        switch(d&3) {
        case 0: return std::array<float,2>{float(y-x),float((x+y)/2-z)};
        case 1: return std::array<float,2>{float(-x-y),float((y-x)/2-z)};
        case 2: return std::array<float,2>{float(x-y),float((-x-y)/2-z)};
        default:return std::array<float,2>{float(x+y),float((x-y)/2-z)};
        }
    };
    for (uint8_t d=0;d<4;++d) {
        FirstPersonVisualHullView v{};
        v.imageDirection=d;
        v.observed=RenderFirstPersonReconstructionSilhouette(
            originalTriangles,d,wireProject);
        assert(v.observed.size()>250);
        nativeWireViews.push_back(std::move(v));
    }
    const auto inferredWire=BuildFirstPersonSourceTriangulatedWireFaces(
        wire,nativeWireViews,wireProject,originalWire.size(),768);
    if(inferredWire.empty()) {
        std::fprintf(stderr,
            "Native line triangulation failed on 4-view connected synthetic truss\n");
    }
    assert(!inferredWire.empty());
    assert(inferredWire.size()<=768);
    const auto wireCert=CertifyFirstPersonReconstruction(
        TriangulateFirstPersonReconstructionFaces(inferredWire),
        nativeWireViews,wireProject);
    assert(wireCert.valid && wireCert.viewCount==4);
    assert(wireCert.minimumObservedCoverage>.3f);
    // Coverage ranking is a separate optional geometric hypothesis; the
    // original longest-stroke reconstruction must remain available even
    // when a coverage-biased set has no consistent 3D interpretation.
    const auto coverageWire=BuildFirstPersonSourceTriangulatedWireFaces(
        wire,nativeWireViews,wireProject,originalWire.size(),768,true);
    if (!coverageWire.empty()) {
        const auto coverageCert=CertifyFirstPersonReconstruction(
            TriangulateFirstPersonReconstructionFaces(coverageWire),
            nativeWireViews,wireProject);
        assert(coverageCert.valid && coverageCert.viewCount==4);
    }
    // This reconstruction uses only native strokes: it must also be
    // constructible when the voxel-boundary candidate is entirely missing.
    const auto sourceOnlyWire=BuildFirstPersonSourceTriangulatedWireFaces(
        wire,nativeWireViews,wireProject,0,768);
    assert(!sourceOnlyWire.empty());
    // Source-only mode has a larger relative face budget for optional
    // silhouette-neutral structural junctions.
    assert(sourceOnlyWire.size()>=inferredWire.size());
    // Independently verify the extra two orthographic pair constraints.
    // The expanded alternative is admissible only through the same
    // complete four-view certificate and geometric face budget.
    const auto allPairsWire=BuildFirstPersonSourceTriangulatedWireFaces(
        wire,nativeWireViews,wireProject,originalWire.size(),768,false,true);
    if (!allPairsWire.empty()) {
        const auto extraCert=CertifyFirstPersonReconstruction(
            TriangulateFirstPersonReconstructionFaces(allPairsWire),
            nativeWireViews,wireProject);
        assert(extraCert.valid && extraCert.viewCount==4);
        assert(allPairsWire.size()<=768);
    }
    // Decorative cutouts can be actual thin 3D panels rather than
    // straight structural bars. The independent source silhouettes for this
    // synthetic fixture are produced from 3D strips, then passed to both
    // native reconstruction methods: no privileged original geometry is
    // supplied to either inference method.
    {
        auto ornamentViews=nativeWireViews;
        std::vector<FirstPersonVisualHullFace> ornamentSheets;
        for (int x=4;x<28;x+=4)
        for (int z=7;z<38;z+=5)
        {
            FirstPersonVisualHullFace panel{};
            panel.corners={{
                {float(x),14.0f,float(z)},
                {float(x+1),14.0f,float(z)},
                {float(x+1),14.0f,float(z+3)},
                {float(x),14.0f,float(z+3)}
            }};
            panel.normal={0,1,0};
            panel.kind=FirstPersonVisualHullFaceKind::maxRight;
            ornamentSheets.push_back(panel);
        }
        auto sheetTriangles=TriangulateFirstPersonReconstructionFaces(
            ornamentSheets);
        for (uint8_t d=0;d<4;++d)
        {
            const auto paint=RenderFirstPersonReconstructionSilhouette(
                sheetTriangles,d,wireProject);
            for (uint64_t pixel:paint.pixels)
                ornamentViews[d].observed.add(
                    FirstPersonSilhouettePixelX(pixel),
                    FirstPersonSilhouettePixelY(pixel));
        }
        const auto sourceOnlyRods=BuildFirstPersonSourceTriangulatedWireFaces(
            wire,ornamentViews,wireProject,0,768,false,false,false);
        const auto sourceWithPanels=BuildFirstPersonSourceTriangulatedWireFaces(
            wire,ornamentViews,wireProject,0,768,false,false,true);
        assert(!sourceOnlyRods.empty() && !sourceWithPanels.empty());
        assert(sourceWithPanels.size()<=768);
        assert(sourceWithPanels.size()>sourceOnlyRods.size());
        const auto rodCert=CertifyFirstPersonReconstruction(
            TriangulateFirstPersonReconstructionFaces(sourceOnlyRods),
            ornamentViews,wireProject);
        const auto patchCert=CertifyFirstPersonReconstruction(
            TriangulateFirstPersonReconstructionFaces(sourceWithPanels),
            ornamentViews,wireProject);
        assert(rodCert.valid && patchCert.valid);
        assert(rodCert.viewCount==4 && patchCert.viewCount==4);
        assert(patchCert.minimumIntersectionOverUnion
            > rodCert.minimumIntersectionOverUnion + .005f);
        // A quad is kept only after its exact source-view raster adds more
        // observed than invented pixels in EACH direction. Thus the 4-view
        // silhouette IoU must not worsen.
        for(size_t d=0;d<4;++d)
            assert(patchCert.views[d].fit.intersectionOverUnion + 1e-5f
                >= rodCert.views[d].fit.intersectionOverUnion);
        std::fprintf(stderr,
            "Source-surface fixture rods=%zu augmented=%zu"
            " IoU=%.4f->%.4f\\n",
            sourceOnlyRods.size(),sourceWithPanels.size(),
            double(rodCert.minimumIntersectionOverUnion),
            double(patchCert.minimumIntersectionOverUnion));
    }
    // Every independently inferred member contributes native pixels.
    // Extra prisms are allowed only as evidence-backed structural joints:
    // their pixels must already be covered by native art AND other members.
    assert(inferredWire.size()%6==0);
    std::array<std::unordered_map<uint64_t, unsigned>,4> wirePixelUses{};
    std::vector<std::array<FirstPersonSilhouette,4>> prismImages;
    for(size_t face=0;face<inferredWire.size();face+=6) {
        const std::vector<FirstPersonVisualHullFace> prism(
            inferredWire.begin()+face,inferredWire.begin()+face+6);
        const auto prismTriangles=TriangulateFirstPersonReconstructionFaces(prism);
        std::array<FirstPersonSilhouette,4> projected{};
        for(uint8_t d=0;d<4;++d) {
            projected[d]=RenderFirstPersonReconstructionSilhouette(
                prismTriangles,d,wireProject);
            assert(!projected[d].empty());
            for(uint64_t pixel:projected[d].pixels)
                ++wirePixelUses[d][pixel];
        }
        prismImages.push_back(std::move(projected));
    }
    for(const auto& projected:prismImages) {
        bool unique=false;
        for(uint8_t d=0;d<4 && !unique;++d)
            for(uint64_t pixel:projected[d].pixels)
                if(wirePixelUses[d].at(pixel)==1) {
                    unique=true;
                    break;
                }
        if (!unique)
        {
            for (uint8_t d=0;d<4;++d)
                for (uint64_t px:projected[d].pixels)
                {
                    assert(nativeWireViews[d].observed.contains(
                        FirstPersonSilhouettePixelX(px),
                        FirstPersonSilhouettePixelY(px)));
                    assert(wirePixelUses[d].at(px)>=2);
                }
        }
    }
    // Translate every native source pixel together with the independent
    // projector. This must NOT destroy the physical 3D inference: imported
    // artwork can have arbitrary sprite x/y drawing offsets.
    auto shiftedViews=nativeWireViews;
    constexpr int32_t shiftX=500,shiftY=-300;
    for(auto& view:shiftedViews) {
        FirstPersonSilhouette moved{};
        for(uint64_t id:view.observed.pixels)
            moved.add(FirstPersonSilhouettePixelX(id)+shiftX,
                FirstPersonSilhouettePixelY(id)+shiftY);
        view.observed=std::move(moved);
    }
    const auto shiftedProjection=[&](uint8_t d,FirstPersonVec3 p) {
        auto uv=wireProject(d,p);
        uv[0]+=shiftX;
        uv[1]+=shiftY;
        return uv;
    };
    const auto shiftedWire=BuildFirstPersonSourceTriangulatedWireFaces(
        wire,shiftedViews,shiftedProjection,originalWire.size(),768);
    assert(!shiftedWire.empty());
    const auto shiftedCert=CertifyFirstPersonReconstruction(
        TriangulateFirstPersonReconstructionFaces(shiftedWire),
        shiftedViews,shiftedProjection);
    assert(shiftedCert.valid && shiftedCert.viewCount==4);
    assert(shiftedCert.minimumObservedCoverage>.3f);
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

    def test_native_stroke_pairs_cover_all_unordered_source_views(self):
        source = (ROOT/"src/openrct2/paint/FirstPersonSourceTriangulation.inc").read_text()
        import re
        pairs = re.search(r"constexpr int pairs\[6\]\[2\]\s*=\s*\{(.*?)\};", source, re.S)
        self.assertIsNotNone(pairs)
        extracted = {tuple(map(int, pair)) for pair in
                     re.findall(r"\{([0-3]),([0-3])\}", pairs.group(1))}
        self.assertEqual(extracted, {(a, b) for a in range(4) for b in range(a+1, 4)})
        self.assertIn("constexpr size_t kMaximumProjections=400", source)
        self.assertIn("includeAllSourceViewPairs ? 6 : 4", source)
        collision = (ROOT/"src/openrct2/paint/FirstPersonSmallSceneryCollision.h").read_text()
        hull = (ROOT/"src/openrct2/paint/FirstPersonVisualHull.h").read_text()
        self.assertIn("if (expandedPairs && !bestFaces.empty())", collision)
        self.assertIn("result.sourceTriangulatedWireSelected", hull)
        self.assertIn("minimumNear<0.80f", source)

    def test_source_supported_surface_patches_require_fully_certified_source(self):
        patch = (ROOT/"src/openrct2/paint/FirstPersonSourcePatchReconstruction.inc").read_text()
        wire = (ROOT/"src/openrct2/paint/FirstPersonSourceTriangulation.inc").read_text()
        collision = (ROOT/"src/openrct2/paint/FirstPersonSmallSceneryCollision.h").read_text()
        hull = (ROOT/"src/openrct2/paint/FirstPersonVisualHull.h").read_text()
        self.assertIn("if (includeSourceSupportedPatches", patch)
        self.assertIn("for (uint8_t d = 0; d < 4; ++d)", patch)
        self.assertIn("kMaximumSourcePatchCandidates = 1200", patch)
        self.assertIn("for (int32_t height = minZ + 1;", patch)
        self.assertIn("if (along - runStart >= 2)", patch)
        self.assertIn("face.normal.z > .7f", patch)
        self.assertIn("const auto diagonalPoint = [&](", patch)
        self.assertIn("two simple x+y and x-y planes", patch)
        self.assertIn("8.0f * (strongestIoU - sourceIoU[d])", patch)
        self.assertIn("gain += float(viewGain) * viewWeights[d]", patch)
        self.assertIn("++observedMatches[d]", patch)
        self.assertIn("remainingGain(patches.size())", patch)
        self.assertIn("dependentCandidates[d][pixel].push_back", patch)
        self.assertIn("remainingGain[index][d] -= contribution", patch)
        self.assertIn("directMemberContact(patches.size(),0)", patch)
        self.assertIn("std::vector<std::vector<uint16_t>> adjacent", patch)
        self.assertIn("if (attached)", patch)
        self.assertIn("gain*=2.0f", patch)
        self.assertIn("connectedToMembers[connected]=1", patch)
        self.assertIn("const int32_t viewGain = remainingGain[i][d]", patch)
        self.assertIn("240, faceLimit - result.size()", patch)
        self.assertIn("if (viewGain < 0)", patch)
        self.assertIn("AddFirstPersonSilhouetteQuad(silhouette, screen)", patch)
        self.assertIn("FirstPersonSourcePatchReconstruction.inc", wire)
        self.assertIn("prioritizeCoverage, expandedPairs, true", collision)
        self.assertIn("prioritizeCoverage, expandedPairs, true", hull)

    def test_beam_candidate_is_opt_in_and_diagnostics_are_grounded(self):
        hull = (ROOT/"src/openrct2/paint/FirstPersonVisualHull.h").read_text()
        collision = (ROOT/"src/openrct2/paint/FirstPersonSmallSceneryCollision.h").read_text()
        renderer = (ROOT/"src/openrct2/paint/FirstPersonRenderer.SmallScenery.inc").read_text()
        self.assertIn("bool allowStructuralBeamReconstruction = false;", hull)
        self.assertIn("config.allowStructuralBeamReconstruction = true;", collision)
        self.assertIn("CertifyFirstPersonReconstruction(", hull)
        self.assertIn("FirstPersonRoundTripNonWorse(", hull)
        self.assertIn("if (redundant && chosenCount > 2)", hull)
        self.assertIn("BuildFirstPersonVisualHullSourceMatchedOrientedBeamFaces(", hull)
        self.assertIn('#include "FirstPersonSourceTriangulation.inc"', hull)
        self.assertIn("allowSourceTriangulatedWireFallback = false", hull)
        self.assertIn("result.sourceTriangulatedWireSelected = true;", hull)
        self.assertIn("constexpr int8_t directions[10][3]", hull)
        self.assertIn("kMaximumProjectedRuns = 96", hull)
        self.assertIn("tryStructuralCandidate(BuildFirstPersonVisualHullSourceMatchedOrientedBeamFaces(", hull)
        self.assertIn("result.structuralBeamSelected = true;", hull)
        self.assertIn("certificate.minimumIntersectionOverUnion >= 0.85f", hull)
        self.assertIn("certificate.minimumCandidateCoverage >= 0.90f", hull)
        self.assertIn("certificate.minimumObservedCoverage >= 0.90f", hull)
        self.assertIn("certificate.maximumEdgeError <= 2", hull)
        self.assertIn("result.structuralBeamSelected = false;", hull)
        self.assertIn("strict.structuralBeamSelected", collision)
        self.assertIn("fine.structuralBeamSelected", collision)
        self.assertIn('"structural-beam"', renderer)
        self.assertIn("|| buildEvidence.structuralBeamSelected", renderer)
        # A source-only candidate may rescue a failed voxel intersection,
        # but cannot bypass the independent projection quality certificate.
        # Failure at stage 5 must not block an independently certified
        # native-stroke mesh; the original absolute source-only gates remain.
        self.assertIn("carveAttempt.stage <= 5", collision)
        self.assertIn("carveAttempt.stage >= 3", collision)
        # Moderately overlapping but misregistered four-view lattices must
        # reach the bounded origin estimator. The expanded hypothesis
        # requires a measured differential shift before another voxel pass.
        self.assertIn("suspectDifferentialRegistration", collision)
        self.assertIn("hasDifferentialRegistration()", collision)
        self.assertIn("kMinimumDifferentialOriginPixels = 8", collision)
        self.assertIn("carveAttempt.minimumIoU < 0.50f", collision)
        self.assertIn("certificate.minimumIntersectionOverUnion >= 0.60f", collision)
        self.assertIn("certificate.minimumCandidateCoverage >= 0.74f", collision)
        self.assertIn("certificate.minimumObservedCoverage >= 0.66f", collision)
        self.assertIn("sourceOnly, views, projectPoint, 0, 768", collision)
        self.assertIn("sourceOnly.sourceTriangulatedWireSelected = true;", collision)
        # A registered source-only mesh must pass its native-origin offset
        # to production material baking, exactly like convex and carved
        # hulls. Without this, four-view mesh certification can pass while
        # the actual G1 material lookup reads unrelated transparent pixels.
        source_return = collision.split(
            "sourceOnly.refinedFaces = std::move(bestFaces);", 1)[1].split(
            "return sourceOnly;", 1)[0]
        self.assertIn("sourceOnly.viewPixelOffsets = registrationPixels;", source_return)
        self.assertIn("compact.viewPixelOffsets = registrationPixels;", collision)
        self.assertIn("aligned.viewPixelOffsets = registrationPixels;", collision)
        material = (ROOT/"src/openrct2/paint/FirstPersonRenderer.SmallScenery.inc").read_text()
        self.assertIn("hull->viewPixelOffsets[view.imageDirection & 3u].x", material)
        self.assertIn("hull->viewPixelOffsets[view.imageDirection & 3u].y", material)
        self.assertIn("certificate.minimumIntersectionOverUnion >= 0.60f", collision)
        self.assertIn("certificate.minimumObservedCoverage >= 0.66f", collision)
        self.assertIn("for (bool prioritizeCoverage : { false, true })", hull)
        self.assertIn("FirstPersonRoundTripStrictlyImproves(", collision)


if __name__ == "__main__":
    unittest.main()
