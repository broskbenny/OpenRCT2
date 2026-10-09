/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonAssetReconstruction.h"
#include "FirstPersonMath.h"
#include "FirstPersonRoundTripReconstruction.h"

#include "../drawing/Drawing.Sprite.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace OpenRCT2::Paint
{
    struct FirstPersonVisualHullTextureView
    {
        uint8_t imageDirection{};
        ImageIndex image{};
    };

    enum class FirstPersonVisualHullFaceKind : uint8_t
    {
        minForward,
        maxForward,
        minRight,
        maxRight,
        bottom,
        top,
    };

    struct FirstPersonVisualHullFace
    {
        std::array<FirstPersonVec3, 4> corners{};
        FirstPersonVec3 normal{};
        FirstPersonVisualHullFaceKind kind{};
    };

    // The native material bake must sample the exact surface rasterised by
    // EmitQuad: triangles (0,1,2) and (0,2,3), split on the s == t UV
    // diagonal. Bilinear interpolation is a DIFFERENT surface when a quad is
    // warped or not a parallelogram, and gives visibly folded textures.
    //
    // A triangle is represented by a degenerate quad {a,b,c,c}; its second
    // GPU triangle is empty. Never paint outside its actual UV triangle.
    [[nodiscard]] inline bool FirstPersonVisualHullSampleFace(
        const FirstPersonVisualHullFace& face,
        float s, float t, FirstPersonVec3& point)
    {
        const auto& a = face.corners[0];
        const auto& b = face.corners[1];
        const auto& c = face.corners[2];
        const auto& d = face.corners[3];
        const auto delta = FirstPersonVec3{
            d.x - c.x, d.y - c.y, d.z - c.z
        };
        const bool triangle =
            FpDot(delta, delta) <= 1.0e-8f;
        if (s >= t)
        {
            point = {
                a.x + (b.x - a.x) * s + (c.x - b.x) * t,
                a.y + (b.y - a.y) * s + (c.y - b.y) * t,
                a.z + (b.z - a.z) * s + (c.z - b.z) * t,
            };
            return true;
        }
        if (triangle)
            return false;

        point = {
            a.x + (c.x - d.x) * s + (d.x - a.x) * t,
            a.y + (c.y - d.y) * s + (d.y - a.y) * t,
            a.z + (c.z - d.z) * s + (d.z - a.z) * t,
        };
        return true;
    }

    struct FirstPersonVisualHull
    {
        bool valid = false;
        float step = 4.0f;
        float minForward{};
        float minRight{};
        float minUp{};
        uint8_t sizeForward{};
        uint8_t sizeRight{};
        uint8_t sizeUp{};
        std::vector<uint8_t> occupied;
        std::vector<FirstPersonVisualHullTextureView> textureViews;
        // When native evidence proves that a continuous relaxation reproduces
        // the observations better than the raw voxel boundary, these are the
        // renderable faces. Occupancy/collision remains the original grid.
        std::vector<FirstPersonVisualHullFace> refinedFaces;
        bool continuousSurfaceRefined = false;
        // Exact surface-level round-trip evidence. The legacy coverage fields
        // remain directly readable because reconstruction policies use them as
        // conservative admission gates.
        float averageIntersectionOverUnion{};
        float minimumIntersectionOverUnion{};
        float minimumCandidateCoverage{};
        float minimumObservedCoverage{};
        int32_t maximumEdgeError{};
        size_t roundTripViewCount{};
        size_t voxelRoundTripSymmetricDifference{};
        size_t roundTripSymmetricDifference{};

        [[nodiscard]] bool contains(
            int32_t forward, int32_t right, int32_t up) const
        {
            if (forward < 0 || right < 0 || up < 0
                || forward >= sizeForward
                || right >= sizeRight || up >= sizeUp)
                return false;
            const size_t index =
                (size_t(up) * sizeRight + size_t(right))
                    * sizeForward + size_t(forward);
            return index < occupied.size()
                && occupied[index] != 0;
        }

        [[nodiscard]] FirstPersonVec3 centre(
            int32_t forward, int32_t right, int32_t up) const
        {
            return {
                minForward
                    + (float(forward) + 0.5f) * step,
                minRight
                    + (float(right) + 0.5f) * step,
                minUp
                    + (float(up) + 0.5f) * step,
            };
        }

        [[nodiscard]] bool containsPoint(
            FirstPersonVec3 point) const
        {
            const int32_t forward = int32_t(std::floor(
                (point.x - minForward) / step));
            const int32_t right = int32_t(std::floor(
                (point.y - minRight) / step));
            const int32_t up = int32_t(std::floor(
                (point.z - minUp) / step));
            return contains(forward, right, up);
        }
    };

    // A closed occupancy hull does not imply a closed rendered surface. Baking
    // can omit unobserved faces or leave transparent texels on a boundary.
    // Missing material is directional evidence: an opening on one side can
    // expose the reverse of the opposing boundary, but it must not make every
    // unrelated face two-sided. This preserves open reconstructed artwork
    // without reintroducing fighting between adjacent exterior wall planes.
    // This contract is for reconstructed hulls, not authored one-sided planes.
    class FirstPersonHullMaterialCoverage
    {
    public:
        template<typename FaceContainer>
        explicit FirstPersonHullMaterialCoverage(
            const FaceContainer& boundaryFaces)
            : _faces(
                  boundaryFaces.size(),
                  FaceCoverage::absent)
            , _normals(boundaryFaces.size())
        {
            for (size_t i = 0;
                 i < boundaryFaces.size(); ++i)
            {
                _normals[i] =
                    boundaryFaces[i].normal;
            }
        }

        void recordFace(size_t face, size_t texels, size_t opaqueTexels)
        {
            if (face >= _faces.size())
                return;
            _faces[face] = texels == 0 || opaqueTexels == 0 || opaqueTexels > texels
                ? FaceCoverage::absent
                : opaqueTexels == texels ? FaceCoverage::opaque : FaceCoverage::partial;
        }

        [[nodiscard]] size_t materialFaces() const
        {
            return size_t(std::count_if(_faces.begin(), _faces.end(), [](auto coverage) {
                return coverage != FaceCoverage::absent;
            }));
        }

        [[nodiscard]] size_t opaqueFaces() const
        {
            return size_t(std::count(_faces.begin(), _faces.end(), FaceCoverage::opaque));
        }

        [[nodiscard]] bool isClosed() const
        {
            return !_faces.empty() && opaqueFaces() == _faces.size();
        }

        [[nodiscard]] bool reverseSideExposed(
            FirstPersonVec3 outwardNormal) const
        {
            const float lengthSquared =
                outwardNormal.x * outwardNormal.x
                + outwardNormal.y * outwardNormal.y
                + outwardNormal.z * outwardNormal.z;
            if (lengthSquared < 0.5f)
                return true;

            for (size_t i = 0; i < _faces.size(); ++i)
            {
                if (_faces[i] == FaceCoverage::opaque)
                    continue;
                const auto& openingNormal = _normals[i];
                const float alignment =
                    outwardNormal.x * openingNormal.x
                    + outwardNormal.y * openingNormal.y
                    + outwardNormal.z * openingNormal.z;
                if (alignment < -0.5f)
                    return true;
            }
            return false;
        }

        // Finalize the whole object after baking, before any surfaces are cached
        // or published. A missing/transparent boundary relaxes only surfaces
        // facing the opposite direction that can actually be exposed through it.
        template<typename SurfaceIterator>
        void applyTo(SurfaceIterator begin, SurfaceIterator end) const
        {
            const auto materialCount = uint32_t(materialFaces());
            const auto opaqueCount = uint32_t(opaqueFaces());
            for (auto it = begin; it != end; ++it)
            {
                it->reconstructedOccupancyBoundary =
                    !_faces.empty();
                it->exteriorOnly =
                    !_faces.empty()
                    && !reverseSideExposed(
                        it->outwardNormal);
                it->diagnosticHullBoundaryFaces = uint32_t(_faces.size());
                it->diagnosticHullMaterialFaces = materialCount;
                it->diagnosticHullOpaqueFaces = opaqueCount;
            }
        }

    private:
        enum class FaceCoverage : uint8_t
        {
            absent,
            partial,
            opaque,
        };
        std::vector<FaceCoverage> _faces;
        std::vector<FirstPersonVec3> _normals;
    };

    template<typename SampleFace, typename BuildFace>
    inline void AppendFirstPersonGreedyHullFaceSlices(
        std::vector<FirstPersonVisualHullFace>& result,
        int32_t sliceCount, int32_t axisACount,
        int32_t axisBCount, SampleFace&& sampleFace,
        BuildFace&& buildFace)
    {
        std::vector<uint8_t> mask(
            size_t(axisACount) * size_t(axisBCount), 0);
        for (int32_t slice = 0; slice < sliceCount; ++slice)
        {
            std::fill(mask.begin(), mask.end(), 0);
            for (int32_t b = 0; b < axisBCount; ++b)
            for (int32_t a = 0; a < axisACount; ++a)
                mask[size_t(b) * size_t(axisACount) + size_t(a)] =
                    sampleFace(slice, a, b) ? 1 : 0;

            for (int32_t b = 0; b < axisBCount; ++b)
            for (int32_t a = 0; a < axisACount; ++a)
            {
                if (mask[size_t(b) * size_t(axisACount) + size_t(a)] == 0)
                    continue;
                int32_t width = 1;
                while (a + width < axisACount
                    && mask[size_t(b) * size_t(axisACount)
                        + size_t(a + width)] != 0)
                    ++width;
                int32_t height = 1;
                for (; b + height < axisBCount; ++height)
                {
                    bool full = true;
                    for (int32_t x = 0; x < width; ++x)
                    {
                        if (mask[size_t(b + height)
                                * size_t(axisACount)
                            + size_t(a + x)] == 0)
                        {
                            full = false;
                            break;
                        }
                    }
                    if (!full)
                        break;
                }
                for (int32_t y = 0; y < height; ++y)
                for (int32_t x = 0; x < width; ++x)
                    mask[size_t(b + y) * size_t(axisACount)
                        + size_t(a + x)] = 0;
                result.push_back(
                    buildFace(slice, a, b, width, height));
            }
        }
    }

    [[nodiscard]] inline std::vector<FirstPersonVisualHullFace>
        BuildFirstPersonVisualHullVoxelBoundaryFaces(
            const FirstPersonVisualHull& hull)
    {
        std::vector<FirstPersonVisualHullFace> result;
        if (!hull.valid || hull.occupied.empty())
            return result;
        const auto coord =
            [step = hull.step](float base, int32_t cell) {
                return base + float(cell) * step;
            };

        AppendFirstPersonGreedyHullFaceSlices(
            result, hull.sizeForward, hull.sizeRight, hull.sizeUp,
            [&](int32_t f, int32_t r, int32_t u) {
                return hull.contains(f, r, u)
                    && !hull.contains(f - 1, r, u);
            },
            [&](int32_t f, int32_t r, int32_t u, int32_t w, int32_t h) {
                const float x = coord(hull.minForward, f);
                const float y0 = coord(hull.minRight, r);
                const float y1 = coord(hull.minRight, r + w);
                const float z0 = coord(hull.minUp, u);
                const float z1 = coord(hull.minUp, u + h);
                return FirstPersonVisualHullFace{ { {
                    { x, y1, z0 }, { x, y0, z0 },
                    { x, y0, z1 }, { x, y1, z1 },
                } }, { -1, 0, 0 }, FirstPersonVisualHullFaceKind::minForward };
            });
        AppendFirstPersonGreedyHullFaceSlices(
            result, hull.sizeForward, hull.sizeRight, hull.sizeUp,
            [&](int32_t f, int32_t r, int32_t u) {
                return hull.contains(f, r, u)
                    && !hull.contains(f + 1, r, u);
            },
            [&](int32_t f, int32_t r, int32_t u, int32_t w, int32_t h) {
                const float x = coord(hull.minForward, f + 1);
                const float y0 = coord(hull.minRight, r);
                const float y1 = coord(hull.minRight, r + w);
                const float z0 = coord(hull.minUp, u);
                const float z1 = coord(hull.minUp, u + h);
                return FirstPersonVisualHullFace{ { {
                    { x, y0, z0 }, { x, y1, z0 },
                    { x, y1, z1 }, { x, y0, z1 },
                } }, { 1, 0, 0 }, FirstPersonVisualHullFaceKind::maxForward };
            });
        AppendFirstPersonGreedyHullFaceSlices(
            result, hull.sizeRight, hull.sizeForward, hull.sizeUp,
            [&](int32_t r, int32_t f, int32_t u) {
                return hull.contains(f, r, u)
                    && !hull.contains(f, r - 1, u);
            },
            [&](int32_t r, int32_t f, int32_t u, int32_t w, int32_t h) {
                const float y = coord(hull.minRight, r);
                const float x0 = coord(hull.minForward, f);
                const float x1 = coord(hull.minForward, f + w);
                const float z0 = coord(hull.minUp, u);
                const float z1 = coord(hull.minUp, u + h);
                return FirstPersonVisualHullFace{ { {
                    { x0, y, z0 }, { x1, y, z0 },
                    { x1, y, z1 }, { x0, y, z1 },
                } }, { 0, -1, 0 }, FirstPersonVisualHullFaceKind::minRight };
            });
        AppendFirstPersonGreedyHullFaceSlices(
            result, hull.sizeRight, hull.sizeForward, hull.sizeUp,
            [&](int32_t r, int32_t f, int32_t u) {
                return hull.contains(f, r, u)
                    && !hull.contains(f, r + 1, u);
            },
            [&](int32_t r, int32_t f, int32_t u, int32_t w, int32_t h) {
                const float y = coord(hull.minRight, r + 1);
                const float x0 = coord(hull.minForward, f);
                const float x1 = coord(hull.minForward, f + w);
                const float z0 = coord(hull.minUp, u);
                const float z1 = coord(hull.minUp, u + h);
                return FirstPersonVisualHullFace{ { {
                    { x1, y, z0 }, { x0, y, z0 },
                    { x0, y, z1 }, { x1, y, z1 },
                } }, { 0, 1, 0 }, FirstPersonVisualHullFaceKind::maxRight };
            });
        AppendFirstPersonGreedyHullFaceSlices(
            result, hull.sizeUp, hull.sizeForward, hull.sizeRight,
            [&](int32_t u, int32_t f, int32_t r) {
                return hull.contains(f, r, u)
                    && !hull.contains(f, r, u - 1);
            },
            [&](int32_t u, int32_t f, int32_t r, int32_t w, int32_t h) {
                const float z = coord(hull.minUp, u);
                const float x0 = coord(hull.minForward, f);
                const float x1 = coord(hull.minForward, f + w);
                const float y0 = coord(hull.minRight, r);
                const float y1 = coord(hull.minRight, r + h);
                return FirstPersonVisualHullFace{ { {
                    { x0, y1, z }, { x1, y1, z },
                    { x1, y0, z }, { x0, y0, z },
                } }, { 0, 0, -1 }, FirstPersonVisualHullFaceKind::bottom };
            });
        AppendFirstPersonGreedyHullFaceSlices(
            result, hull.sizeUp, hull.sizeForward, hull.sizeRight,
            [&](int32_t u, int32_t f, int32_t r) {
                return hull.contains(f, r, u)
                    && !hull.contains(f, r, u + 1);
            },
            [&](int32_t u, int32_t f, int32_t r, int32_t w, int32_t h) {
                const float z = coord(hull.minUp, u + 1);
                const float x0 = coord(hull.minForward, f);
                const float x1 = coord(hull.minForward, f + w);
                const float y0 = coord(hull.minRight, r);
                const float y1 = coord(hull.minRight, r + h);
                return FirstPersonVisualHullFace{ { {
                    { x0, y0, z }, { x1, y0, z },
                    { x1, y1, z }, { x0, y1, z },
                } }, { 0, 0, 1 }, FirstPersonVisualHullFaceKind::top };
            });
        return result;
    }

    [[nodiscard]] inline FirstPersonVec3
        FirstPersonVisualHullLatticePoint(
            const FirstPersonVisualHull& hull,
            int32_t forward, int32_t right, int32_t up)
    {
        return {
            hull.minForward
                + float(forward) * hull.step,
            hull.minRight
                + float(right) * hull.step,
            hull.minUp
                + float(up) * hull.step,
        };
    }

    [[nodiscard]] inline FirstPersonVec3
        FirstPersonVisualHullRelaxedVertexTarget(
            const FirstPersonVisualHull& hull,
            int32_t vertexForward,
            int32_t vertexRight,
            int32_t vertexUp)
    {
        const auto original =
            FirstPersonVisualHullLatticePoint(
                hull, vertexForward,
                vertexRight, vertexUp);

        // The lower envelope is hard placement evidence. A continuous visual
        // candidate may taper or slope above it, but must not make the object
        // float or slide its contact footprint away from the authoritative
        // base plane.
        if (vertexUp == 0)
            return original;

        FirstPersonVec3 sum{};
        size_t count = 0;
        const auto addFaceCentre =
            [&](float f, float r, float u) {
                sum.x += hull.minForward
                    + f * hull.step;
                sum.y += hull.minRight
                    + r * hull.step;
                sum.z += hull.minUp
                    + u * hull.step;
                ++count;
            };

        // A lattice vertex can touch at most eight occupied cells. The target
        // is the centroid of the exposed unit-face centres incident on that
        // vertex. On a flat surface these contributions cancel back to the
        // original lattice point; at voxel stair-steps they define a natural
        // continuous relaxation direction from the support field itself.
        for (int32_t up = vertexUp - 1;
             up <= vertexUp; ++up)
        for (int32_t right = vertexRight - 1;
             right <= vertexRight; ++right)
        for (int32_t forward = vertexForward - 1;
             forward <= vertexForward; ++forward)
        {
            if (!hull.contains(forward, right, up))
                continue;

            if (vertexForward == forward
                && !hull.contains(
                    forward - 1, right, up))
            {
                addFaceCentre(
                    float(forward),
                    float(right) + 0.5f,
                    float(up) + 0.5f);
            }
            if (vertexForward == forward + 1
                && !hull.contains(
                    forward + 1, right, up))
            {
                addFaceCentre(
                    float(forward) + 1.0f,
                    float(right) + 0.5f,
                    float(up) + 0.5f);
            }
            if (vertexRight == right
                && !hull.contains(
                    forward, right - 1, up))
            {
                addFaceCentre(
                    float(forward) + 0.5f,
                    float(right),
                    float(up) + 0.5f);
            }
            if (vertexRight == right + 1
                && !hull.contains(
                    forward, right + 1, up))
            {
                addFaceCentre(
                    float(forward) + 0.5f,
                    float(right) + 1.0f,
                    float(up) + 0.5f);
            }
            if (vertexUp == up
                && !hull.contains(
                    forward, right, up - 1))
            {
                addFaceCentre(
                    float(forward) + 0.5f,
                    float(right) + 0.5f,
                    float(up));
            }
            if (vertexUp == up + 1
                && !hull.contains(
                    forward, right, up + 1))
            {
                addFaceCentre(
                    float(forward) + 0.5f,
                    float(right) + 0.5f,
                    float(up) + 1.0f);
            }
        }

        if (count == 0)
            return original;
        const float inverse =
            1.0f / float(count);
        return {
            sum.x * inverse,
            sum.y * inverse,
            sum.z * inverse,
        };
    }

    // A silhouette round trip alone cannot certify physical topology: two
    // disconnected sheets can project exactly like a connected shell. Greedy
    // voxel meshing creates long edges, with neighbouring faces meeting those
    // edges at intermediate lattice vertices. Moving only the four corners
    // of each greedy face makes these junctions tear and produces warped,
    // folded quads even when the 2D projection score improves.
    //
    // Refine a shared lattice displacement field instead. Preserve a greedy
    // face when every intermediate lattice vertex agrees with its planar
    // interpolation; otherwise tessellate that face at the native occupancy
    // resolution. All adjoining patches then interpolate precisely the same
    // displaced vertex positions. Nonplanar unit quads are triangulated with
    // their real normals rather than pretending they are a single plane.
    [[nodiscard]] inline std::vector<FirstPersonVisualHullFace>
        BuildFirstPersonVisualHullContinuousCandidateFaces(
            const FirstPersonVisualHull& hull,
            const std::vector<FirstPersonVisualHullFace>& sourceFaces,
            float blend, size_t maximumFaces = 8192)
    {
        std::vector<FirstPersonVisualHullFace> result;
        if (!hull.valid || sourceFaces.empty()
            || !(hull.step > 0.0f)
            || blend <= 0.0f || blend > 1.0f
            || maximumFaces == 0)
            return result;

        const auto subtract =
            [](FirstPersonVec3 a, FirstPersonVec3 b) {
                return FirstPersonVec3{
                    a.x - b.x, a.y - b.y, a.z - b.z
                };
            };
        const auto crossNormal =
            [&](FirstPersonVec3 a, FirstPersonVec3 b,
                FirstPersonVec3 c)
                -> std::optional<FirstPersonVec3> {
                const auto n = FpCross(
                    subtract(b, a), subtract(c, a));
                const float lengthSquared = FpDot(n, n);
                if (lengthSquared <= 1.0e-8f)
                    return std::nullopt;
                const float scale = 1.0f / std::sqrt(lengthSquared);
                return FirstPersonVec3{
                    n.x * scale, n.y * scale, n.z * scale
                };
            };
        const auto lerp =
            [](FirstPersonVec3 a, FirstPersonVec3 b, float t) {
                return FirstPersonVec3{
                    a.x + (b.x - a.x) * t,
                    a.y + (b.y - a.y) * t,
                    a.z + (b.z - a.z) * t,
                };
            };
        const auto distanceSquared =
            [&](FirstPersonVec3 a, FirstPersonVec3 b) {
                const auto d = subtract(a, b);
                return FpDot(d, d);
            };
        const auto latticeCoordinate =
            [step = hull.step](float coordinate, float base) {
                return int32_t(std::lround(
                    (coordinate - base) / step));
            };
        const auto makeVertex =
            [&](FirstPersonVec3 original)
                -> std::optional<FirstPersonVec3> {
                const int32_t forward = latticeCoordinate(
                    original.x, hull.minForward);
                const int32_t right = latticeCoordinate(
                    original.y, hull.minRight);
                const int32_t up = latticeCoordinate(
                    original.z, hull.minUp);
                if (forward < 0 || forward > hull.sizeForward
                    || right < 0 || right > hull.sizeRight
                    || up < 0 || up > hull.sizeUp)
                    return std::nullopt;
                const auto target =
                    FirstPersonVisualHullRelaxedVertexTarget(
                        hull, forward, right, up);
                return lerp(original, target, blend);
            };
        const auto addTriangle =
            [&](const FirstPersonVisualHullFace& source,
                FirstPersonVec3 a, FirstPersonVec3 b,
                FirstPersonVec3 c) {
                const auto n = crossNormal(a, b, c);
                if (!n.has_value()
                    || FpDot(*n, source.normal) <= 0.20f
                    || result.size() >= maximumFaces)
                    return false;
                FirstPersonVisualHullFace triangle{};
                triangle.kind = source.kind;
                triangle.normal = *n;
                triangle.corners = { a, b, c, c };
                result.push_back(triangle);
                return true;
            };
        const auto addQuad =
            [&](const FirstPersonVisualHullFace& source,
                FirstPersonVec3 a, FirstPersonVec3 b,
                FirstPersonVec3 c, FirstPersonVec3 d,
                bool canSplit) {
                const auto n0 = crossNormal(a, b, c);
                const auto n1 = crossNormal(a, c, d);
                if (!n0.has_value() || !n1.has_value()
                    || FpDot(*n0, source.normal) <= 0.20f
                    || FpDot(*n1, source.normal) <= 0.20f)
                    return false;
                // The real GPU geometry contains two triangles. A warped
                // quad cannot be described by one normal and one material
                // plane; give each triangle its own true normal instead.
                if (FpDot(*n0, *n1) < 0.995f)
                {
                    return canSplit
                        && addTriangle(source, a, b, c)
                        && addTriangle(source, a, c, d);
                }
                if (result.size() >= maximumFaces)
                    return false;
                FirstPersonVisualHullFace face{};
                face.kind = source.kind;
                face.corners = { a, b, c, d };
                const FirstPersonVec3 sum{
                    n0->x + n1->x,
                    n0->y + n1->y,
                    n0->z + n1->z,
                };
                const float scale =
                    1.0f / std::sqrt(FpDot(sum, sum));
                face.normal = {
                    sum.x * scale,
                    sum.y * scale,
                    sum.z * scale
                };
                result.push_back(face);
                return true;
            };

        result.reserve(std::min(sourceFaces.size(), maximumFaces));
        for (const auto& source : sourceFaces)
        {
            const auto alongU = subtract(
                source.corners[1], source.corners[0]);
            const auto alongV = subtract(
                source.corners[3], source.corners[0]);
            const int32_t uCount = int32_t(std::lround(
                std::sqrt(FpDot(alongU, alongU)) / hull.step));
            const int32_t vCount = int32_t(std::lround(
                std::sqrt(FpDot(alongV, alongV)) / hull.step));
            if (uCount < 1 || vCount < 1
                || uCount > hull.sizeForward + hull.sizeRight
                    + hull.sizeUp
                || vCount > hull.sizeForward + hull.sizeRight
                    + hull.sizeUp)
                return {};

            const size_t stride = size_t(uCount) + 1;
            std::vector<FirstPersonVec3> grid(
                stride * (size_t(vCount) + 1));
            const auto at = [&](int32_t u, int32_t v)
                -> FirstPersonVec3& {
                return grid[size_t(v) * stride + size_t(u)];
            };
            for (int32_t v = 0; v <= vCount; ++v)
            for (int32_t u = 0; u <= uCount; ++u)
            {
                const float x = float(u) / float(uCount);
                const float y = float(v) / float(vCount);
                const auto original = lerp(
                    lerp(source.corners[0], source.corners[1], x),
                    lerp(source.corners[3], source.corners[2], x),
                    y);
                const auto refined = makeVertex(original);
                if (!refined.has_value())
                    return {};
                at(u, v) = *refined;
            }

            const auto p0 = at(0, 0);
            const auto p1 = at(uCount, 0);
            const auto p2 = at(uCount, vCount);
            const auto p3 = at(0, vCount);
            // A retained long edge may have other faces meeting it at
            // intermediate grid vertices. Only keep it if those vertices
            // still lie on the same straight boundary after relaxation.
            const float toleranceSquared =
                (hull.step * 0.002f) * (hull.step * 0.002f);
            bool affine = true;
            for (int32_t v = 0; v <= vCount && affine; ++v)
            for (int32_t u = 0; u <= uCount; ++u)
            {
                const float x = float(u) / float(uCount);
                const float y = float(v) / float(vCount);
                const auto expected = lerp(
                    lerp(p0, p1, x), lerp(p3, p2, x), y);
                if (distanceSquared(at(u, v), expected)
                    > toleranceSquared)
                {
                    affine = false;
                    break;
                }
            }

            if (affine)
            {
                // If the whole surface is planar and its shared lattice
                // vertices interpolate exactly, preserve the greedy face.
                const size_t before = result.size();
                if (addQuad(
                        source, p0, p1, p2, p3, false))
                    continue;
                if (result.size() != before)
                    return {};
            }

            // Large, unchanged planar patches may remain one greedy face,
            // even if their grid footprint exceeds the tessellation budget.
            // Apply the bound only once subdivision is actually necessary.
            if (size_t(uCount) * size_t(vCount)
                    > maximumFaces - result.size())
                return {};

            // Conforming refinement: all unit faces use the same displaced
            // occupancy-lattice vertices, including along former T-junctions.
            for (int32_t v = 0; v < vCount; ++v)
            for (int32_t u = 0; u < uCount; ++u)
            {
                if (!addQuad(
                        source, at(u, v), at(u + 1, v),
                        at(u + 1, v + 1), at(u, v + 1),
                        true))
                    return {};
            }
        }
        return result;
    }

    [[nodiscard]] inline std::vector<FirstPersonVisualHullFace>
        BuildFirstPersonVisualHullBoundaryFaces(
            const FirstPersonVisualHull& hull)
    {
        if (!hull.refinedFaces.empty())
            return hull.refinedFaces;
        return BuildFirstPersonVisualHullVoxelBoundaryFaces(
            hull);
    }

    struct FirstPersonVisualHullView
    {
        uint8_t imageDirection{};
        ImageIndex image{};
        const G1Element* g1{};
        FirstPersonSilhouette observed{};
    };

    struct FirstPersonVisualHullBounds
    {
        float minForward{};
        float maxForward{};
        float minRight{};
        float maxRight{};
        float minUp{};
        float maxUp{};
        float step = 4.0f;
    };

    struct FirstPersonVisualHullConfig
    {
        size_t minimumViews = 4;
        size_t minimumOccupiedCells = 4;
        size_t maximumOccupiedCells = 4096;
        size_t maximumGridCells = 65536;
        int32_t maximumAxisCells = 24;
        // Continuous refinement is opt-in per reconstruction family until its
        // material path has been verified. Search is bounded by face count and
        // a small deterministic line search; promotion is decided only by the
        // native round-trip evidence.
        bool allowContinuousSurfaceRefinement = false;
        size_t maximumContinuousSurfaceFaces = 8192;
        uint8_t continuousSurfaceSearchSteps = 3;
        // Kept at zero by default until asset-wide calibration establishes a
        // justified threshold. The score is still recorded for every accepted
        // silhouette-derived hull.
        float minimumIntersectionOverUnion = 0.0f;
        float minimumCandidateCoverage = 0.60f;
        float minimumObservedCoverage = 0.32f;
        int32_t maximumEdgeError = 6;
    };

    template<typename OccupancyPredicate>
    [[nodiscard]] FirstPersonVisualHull BuildFirstPersonOccupancyHull(
        const FirstPersonVisualHullBounds& bounds,
        const FirstPersonVisualHullConfig& config,
        const std::vector<FirstPersonVisualHullTextureView>& textureViews,
        OccupancyPredicate&& occupancyPredicate)
    {
        FirstPersonVisualHull result{};
        if (!(bounds.step > 0.0f)
            || bounds.maxForward <= bounds.minForward
            || bounds.maxRight <= bounds.minRight
            || bounds.maxUp <= bounds.minUp)
            return result;

        const int32_t nForward = int32_t(std::ceil(
            (bounds.maxForward - bounds.minForward) / bounds.step));
        const int32_t nRight = int32_t(std::ceil(
            (bounds.maxRight - bounds.minRight) / bounds.step));
        const int32_t nUp = int32_t(std::ceil(
            (bounds.maxUp - bounds.minUp) / bounds.step));
        if (nForward <= 0 || nRight <= 0 || nUp <= 0
            || nForward > config.maximumAxisCells
            || nRight > config.maximumAxisCells
            || nUp > config.maximumAxisCells
            || nForward > 255 || nRight > 255 || nUp > 255)
            return result;

        const size_t gridCells =
            size_t(nForward) * size_t(nRight) * size_t(nUp);
        if (gridCells > config.maximumGridCells)
            return result;

        result.step = bounds.step;
        result.minForward = bounds.minForward;
        result.minRight = bounds.minRight;
        result.minUp = bounds.minUp;
        result.sizeForward = uint8_t(nForward);
        result.sizeRight = uint8_t(nRight);
        result.sizeUp = uint8_t(nUp);
        result.occupied.assign(gridCells, 0);
        result.textureViews = textureViews;

        size_t occupiedCount = 0;
        for (int32_t up = 0; up < nUp; ++up)
        for (int32_t right = 0; right < nRight; ++right)
        for (int32_t forward = 0; forward < nForward; ++forward)
        {
            const auto point = result.centre(forward, right, up);
            if (!occupancyPredicate(point))
                continue;
            const size_t index =
                (size_t(up) * size_t(nRight) + size_t(right))
                    * size_t(nForward) + size_t(forward);
            result.occupied[index] = 1;
            ++occupiedCount;
        }
        if (occupiedCount == 0
            || occupiedCount > config.maximumOccupiedCells)
            return {};

        // No silhouette evidence was used: valid geometry does not imply
        // confidence in a sprite-derived fit.
        result.minimumCandidateCoverage = 0.0f;
        result.minimumObservedCoverage = 0.0f;
        result.maximumEdgeError = 0;
        result.valid = true;
        return result;
    }

    [[nodiscard]] inline bool
        FirstPersonVisualHullPixelOpaque(
            const G1Element& g1, int32_t x, int32_t y)
    {
        if (g1.offset == nullptr || x < 0 || y < 0
            || x >= g1.width || y >= g1.height
            || g1.flags.has(G1Flag::isPalette))
            return false;
        if (g1.flags.has(G1Flag::hasRLECompression))
        {
            const uint16_t lineOffset =
                uint16_t(g1.offset[y * 2])
                | (uint16_t(g1.offset[y * 2 + 1]) << 8);
            const uint8_t* run = g1.offset + lineOffset;
            bool endOfLine = false;
            size_t guard = 0;
            while (!endOfLine && guard++ < 256)
            {
                uint8_t length = *run++;
                const int32_t start = *run++;
                endOfLine = (length & 0x80u) != 0;
                length &= 0x7Fu;
                if (x >= start && x < start + length)
                    return run[x - start] != 0;
                run += length;
            }
            return false;
        }
        const uint8_t pixel =
            g1.offset[size_t(y) * size_t(g1.width)
                + size_t(x)];
        return !g1.flags.has(G1Flag::hasTransparency)
            || pixel != 0;
    }

    template<
        typename ProjectPoint, typename OccupancyPredicate,
        typename PointSupported>
    [[nodiscard]] FirstPersonVisualHull BuildFirstPersonVisualHull(
        const std::vector<FirstPersonVisualHullView>& views,
        const FirstPersonVisualHullBounds& bounds,
        const FirstPersonVisualHullConfig& config,
        ProjectPoint&& projectPoint,
        OccupancyPredicate&& occupancyPredicate,
        PointSupported&& pointSupported)
    {
        FirstPersonVisualHull result{};
        if (views.size() < config.minimumViews
            || !(bounds.step > 0.0f)
            || bounds.maxForward <= bounds.minForward
            || bounds.maxRight <= bounds.minRight
            || bounds.maxUp <= bounds.minUp)
            return result;

        const int32_t nForward = int32_t(std::ceil(
            (bounds.maxForward - bounds.minForward) / bounds.step));
        const int32_t nRight = int32_t(std::ceil(
            (bounds.maxRight - bounds.minRight) / bounds.step));
        const int32_t nUp = int32_t(std::ceil(
            (bounds.maxUp - bounds.minUp) / bounds.step));
        if (nForward <= 0 || nRight <= 0 || nUp <= 0
            || nForward > config.maximumAxisCells
            || nRight > config.maximumAxisCells
            || nUp > config.maximumAxisCells
            || nForward > 255 || nRight > 255 || nUp > 255)
            return result;

        const size_t gridCells =
            size_t(nForward) * size_t(nRight) * size_t(nUp);
        if (gridCells > config.maximumGridCells)
            return result;

        result.step = bounds.step;
        result.minForward = bounds.minForward;
        result.minRight = bounds.minRight;
        result.minUp = bounds.minUp;
        result.sizeForward = uint8_t(nForward);
        result.sizeRight = uint8_t(nRight);
        result.sizeUp = uint8_t(nUp);
        result.occupied.assign(gridCells, 0);
        result.textureViews.reserve(views.size());
        for (const auto& view : views)
            result.textureViews.push_back(
                { view.imageDirection, view.image });

        size_t occupiedCount = 0;
        for (int32_t up = 0; up < nUp; ++up)
        for (int32_t right = 0; right < nRight; ++right)
        for (int32_t forward = 0; forward < nForward; ++forward)
        {
            const auto point = result.centre(forward, right, up);
            if (!occupancyPredicate(point))
                continue;

            bool supported = true;
            // A transparent pixel in any admitted view disproves occupancy.
            for (const auto& view : views)
            {
                if (!pointSupported(view, point))
                {
                    supported = false;
                    break;
                }
            }
            if (!supported)
                continue;

            const size_t index =
                (size_t(up) * size_t(nRight) + size_t(right))
                    * size_t(nForward) + size_t(forward);
            result.occupied[index] = 1;
            ++occupiedCount;
        }
        if (occupiedCount < config.minimumOccupiedCells
            || occupiedCount > config.maximumOccupiedCells)
            return {};

        // The boundary extractor treats valid=true as permission to expose
        // geometry. Mark this local candidate provisionally so both the raw
        // boundary and evidence-driven alternatives can be forward-rendered.
        // Any failed admission below returns a fresh invalid hull.
        result.valid = true;

        const auto voxelFaces =
            BuildFirstPersonVisualHullVoxelBoundaryFaces(
                result);
        const auto voxelTriangles =
            TriangulateFirstPersonReconstructionFaces(
                voxelFaces);
        const auto voxelCertificate =
            CertifyFirstPersonReconstruction(
                voxelTriangles, views, projectPoint);
        const auto passesAdmission =
            [&](const FirstPersonRoundTripCertificate& certificate) {
                return certificate.valid
                    && certificate.viewCount
                        >= config.minimumViews
                    && certificate.minimumIntersectionOverUnion
                        >= config.minimumIntersectionOverUnion
                    && certificate.minimumCandidateCoverage
                        >= config.minimumCandidateCoverage
                    && certificate.minimumObservedCoverage
                        >= config.minimumObservedCoverage
                    && certificate.maximumEdgeError
                        <= config.maximumEdgeError;
            };
        // Admission belongs to the FINAL reconstructed surface, not the
        // unrefined voxel baseline. Rejecting the baseline here prevented a
        // smoother candidate from ever rescuing a visually correct object.
        // A valid baseline certificate is still required for comparison.
        if (!voxelCertificate.valid)
            return {};

        auto selectedCertificate =
            voxelCertificate;
        if (config.allowContinuousSurfaceRefinement
            && !voxelFaces.empty()
            && voxelFaces.size()
                <= config.maximumContinuousSurfaceFaces
            && config.continuousSurfaceSearchSteps != 0)
        {
            for (uint8_t stepIndex = 1;
                 stepIndex
                    <= config.continuousSurfaceSearchSteps;
                 ++stepIndex)
            {
                const float blend =
                    float(stepIndex)
                    / float(
                        config.continuousSurfaceSearchSteps);
                const auto candidateFaces =
                    BuildFirstPersonVisualHullContinuousCandidateFaces(
                        result, voxelFaces, blend,
                    config.maximumContinuousSurfaceFaces);
                if (candidateFaces.empty())
                    continue;
                const auto candidateCertificate =
                    CertifyFirstPersonReconstruction(
                        TriangulateFirstPersonReconstructionFaces(
                            candidateFaces),
                        views, projectPoint);
                if (!passesAdmission(candidateCertificate))
                    continue;
                // When the baseline does not meet admission, a candidate
                // that independently passes and is no worse than the
                // baseline is meaningful progress even when total mismatch
                // happens to tie. Once admitted, require strict improvement.
                if (passesAdmission(selectedCertificate)
                    ? !FirstPersonRoundTripStrictlyImproves(
                        candidateCertificate, selectedCertificate)
                    : !FirstPersonRoundTripNonWorse(
                        candidateCertificate, selectedCertificate))
                    continue;

                result.refinedFaces =
                    candidateFaces;
                selectedCertificate =
                    candidateCertificate;
            }
        }

        if (!passesAdmission(selectedCertificate))
            return {};

        result.continuousSurfaceRefined =
            !result.refinedFaces.empty();
        result.voxelRoundTripSymmetricDifference =
            voxelCertificate.symmetricDifferencePixels;
        result.roundTripSymmetricDifference =
            selectedCertificate.symmetricDifferencePixels;
        result.averageIntersectionOverUnion =
            selectedCertificate.averageIntersectionOverUnion;
        result.minimumIntersectionOverUnion =
            selectedCertificate.minimumIntersectionOverUnion;
        result.minimumCandidateCoverage =
            selectedCertificate.minimumCandidateCoverage;
        result.minimumObservedCoverage =
            selectedCertificate.minimumObservedCoverage;
        result.maximumEdgeError =
            selectedCertificate.maximumEdgeError;
        result.roundTripViewCount =
            selectedCertificate.viewCount;
        result.valid = true;
        return result;
    }

    template<typename ProjectPoint, typename OccupancyPredicate>
    [[nodiscard]] FirstPersonVisualHull BuildFirstPersonVisualHull(
        const std::vector<FirstPersonVisualHullView>& views,
        const FirstPersonVisualHullBounds& bounds,
        const FirstPersonVisualHullConfig& config,
        ProjectPoint&& projectPoint,
        OccupancyPredicate&& occupancyPredicate)
    {
        const auto pointSupported =
            [&](const FirstPersonVisualHullView& view,
                FirstPersonVec3 point) {
                if (view.g1 == nullptr)
                    return false;
                const auto projected =
                    projectPoint(view.imageDirection, point);
                const int32_t px =
                    int32_t(std::lround(projected[0]))
                    - view.g1->xOffset;
                const int32_t py =
                    int32_t(std::lround(projected[1]))
                    - view.g1->yOffset;
                for (int32_t dy = -1; dy <= 1; ++dy)
                for (int32_t dx = -1; dx <= 1; ++dx)
                {
                    if (FirstPersonVisualHullPixelOpaque(
                            *view.g1, px + dx, py + dy))
                        return true;
                }
                return false;
            };
        return BuildFirstPersonVisualHull(
            views, bounds, config,
            projectPoint, occupancyPredicate,
            pointSupported);
    }
} // namespace OpenRCT2::Paint
