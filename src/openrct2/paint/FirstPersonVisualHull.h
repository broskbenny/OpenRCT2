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
#include <unordered_map>
#include <unordered_set>
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

    // Nearest texture sampling can select a texel whose centre is outside a
    // triangular UV domain even though part of that texel lies inside it.
    // Sample that covered part, rather than leaving a transparent diagonal.
    // This pads only the texture footprint; it never fills holes in source art.
    [[nodiscard]] inline std::optional<std::array<float, 2>> FirstPersonVisualHullTexelUV(
        const FirstPersonVisualHullFace& face, int32_t x, int32_t y, int32_t width, int32_t height)
    {
        if (width <= 0 || height <= 0 || x < 0 || y < 0 || x >= width || y >= height)
            return std::nullopt;
        float s = (float(x) + 0.5f) / float(width);
        float t = (float(y) + 0.5f) / float(height);
        const auto& c = face.corners[2];
        const auto& d = face.corners[3];
        const FirstPersonVec3 delta{ d.x - c.x, d.y - c.y, d.z - c.z };
        if (s < t && FpDot(delta, delta) <= 1.0e-8f)
        {
            const float left = float(x) / float(width), right = float(x + 1) / float(width);
            const float top = float(y) / float(height), bottom = float(y + 1) / float(height);
            if (right <= top) return std::nullopt;
            s = right - 0.25f * (right - std::max(left, top));
            t = top + 0.25f * (std::min(bottom, right) - top);
        }
        return std::array<float, 2>{ s, t };
    }

    struct FirstPersonVisualHullAttempt
    {
        // 1=views/bounds, 2=grid, 3=carving, 4=round-trip,
        // 5=admission, 6=accepted. A value below 6 identifies the
        // stage whose output was rejected (rather than an object type).
        uint8_t stage = 0;
        uint8_t nativeViews = 0;
        uint32_t occupiedCells = 0;
        uint32_t voxelFaces = 0;
        uint32_t certificateViews = 0;
        float minimumIoU = 0.0f;
        float candidateCoverage = 0.0f;
        float observedCoverage = 0.0f;
        int32_t maximumEdgeError = 0;
        size_t disagreementPixels = 0;
        // Diagnostic image-registration trial. An accepted reconstruction
        // also retains the correction for material sampling, not geometry.
        bool registrationAttempted = false;
        std::array<CoordsXY, 4> registrationPixels{};
    };

    struct FirstPersonVisualHull
    {
        bool valid = false;
        FirstPersonVisualHullAttempt attempt{};
        // Native sprite frame corrections are independent of the physical
        // occupancy and must be reused by the material-projection stage.
        std::array<CoordsXY, 4> viewPixelOffsets{};
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
        // Independent visual-only candidate with shared planar roof/wall
        // edges, accepted only after four-view raster certification.
        bool planarSurfaceSelected = false;
        bool directConvexSurface = false;
        bool structuralBeamSelected = false;
        bool sourceTriangulatedWireSelected = false;
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

    // Material admission is independent of the already certified geometry.
    // When a heavily perforated hull loses many textured faces, keep only
    // source-backed patches connected by PHYSICAL EDGES. A shared vertex is
    // insufficient: otherwise hundreds of isolated voxel glints can form a
    // fake coherent surface. Unsupported faces cannot bridge components.
    struct FirstPersonCoherentMaterialPatches
    {
        std::vector<uint8_t> keep;
        size_t retainedFaces = 0;
    };

    [[nodiscard]] inline FirstPersonCoherentMaterialPatches
        SelectFirstPersonCoherentMaterialPatches(
            const std::vector<FirstPersonVisualHullFace>& faces,
            const std::vector<size_t>& texturedFaceIndices)
    {
        FirstPersonCoherentMaterialPatches result{};
        result.keep.assign(faces.size(), 0);
        if (texturedFaceIndices.empty())
            return result;

        using Vertex = std::array<int32_t, 3>;
        using Edge = std::array<Vertex, 2>;
        struct EdgeHash
        {
            size_t operator()(const Edge& edge) const
            {
                uint64_t hash = 14695981039346656037ull;
                for (const auto& point : edge)
                    for (int32_t value : point)
                    {
                        hash ^= uint32_t(value);
                        hash *= 1099511628211ull;
                    }
                return size_t(hash);
            }
        };
        const auto vertex = [](FirstPersonVec3 p) -> Vertex {
            return { int32_t(std::lround(double(p.x) * 64.0)),
                     int32_t(std::lround(double(p.y) * 64.0)),
                     int32_t(std::lround(double(p.z) * 64.0)) };
        };
        const auto area = [](const FirstPersonVisualHullFace& face) {
            const auto triangle = [](FirstPersonVec3 a, FirstPersonVec3 b,
                                      FirstPersonVec3 c) {
                const double ux = double(b.x) - a.x;
                const double uy = double(b.y) - a.y;
                const double uz = double(b.z) - a.z;
                const double vx = double(c.x) - a.x;
                const double vy = double(c.y) - a.y;
                const double vz = double(c.z) - a.z;
                const double cx = uy * vz - uz * vy;
                const double cy = uz * vx - ux * vz;
                const double cz = ux * vy - uy * vx;
                return 0.5 * std::sqrt(cx * cx + cy * cy + cz * cz);
            };
            return triangle(face.corners[0], face.corners[1], face.corners[2])
                + triangle(face.corners[0], face.corners[2], face.corners[3]);
        };

        const size_t count = texturedFaceIndices.size();
        std::vector<size_t> parent(count);
        for (size_t i = 0; i < count; ++i)
            parent[i] = i;
        const auto rootOf = [&](size_t index) {
            while (parent[index] != index)
            {
                parent[index] = parent[parent[index]];
                index = parent[index];
            }
            return index;
        };
        std::unordered_map<Edge, size_t, EdgeHash> edgeOwners;
        edgeOwners.reserve(count * 4);
        for (size_t i = 0; i < count; ++i)
        {
            const size_t index = texturedFaceIndices[i];
            if (index >= faces.size())
                return result;
            const auto& face = faces[index];
            for (size_t e = 0; e < 4; ++e)
            {
                auto a = vertex(face.corners[e]);
                auto b = vertex(face.corners[(e + 1) & 3u]);
                if (a == b)
                    continue;
                if (b < a)
                    std::swap(a, b);
                const Edge key{ a, b };
                const auto [found, inserted] = edgeOwners.emplace(key, i);
                if (!inserted)
                    parent[rootOf(i)] = rootOf(found->second);
            }
        }

        std::vector<size_t> sizes(count, 0);
        std::vector<double> areas(count, 0.0);
        double totalArea = 0.0;
        for (size_t i = 0; i < count; ++i)
        {
            const size_t root = rootOf(i);
            const double surfaceArea = area(faces[texturedFaceIndices[i]]);
            ++sizes[root];
            areas[root] += surfaceArea;
            totalArea += surfaceArea;
        }

        // Reject small isolated voxel flakes. A large source-backed panel
        // may be one quad; a thin lattice needs a substantial connected run.
        // Require aggregate retention as well, so a single good-looking
        // island does not turn a mostly unsupported hull into visual noise.
        double retainedArea = 0.0;
        for (size_t i = 0; i < count; ++i)
        {
            const size_t root = rootOf(i);
            if (areas[root] < 128.0
                && !(sizes[root] >= 8 && areas[root] >= 32.0))
                continue;
            result.keep[texturedFaceIndices[i]] = 1;
            ++result.retainedFaces;
            retainedArea += area(faces[texturedFaceIndices[i]]);
        }
        if (result.retainedFaces * 4 < count
            || retainedArea * 4.0 < totalArea)
        {
            std::fill(result.keep.begin(), result.keep.end(), 0);
            result.retainedFaces = 0;
        }
        return result;
    }

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

        void clearFace(size_t face)
        {
            if (face < _faces.size())
                _faces[face] = FaceCoverage::absent;
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

    // Infer *continuous axial structure* from the four-view intersection
    // field, rather than accepting every isolated occupied voxel as artwork.
    // A candidate keeps runs extending at least ten world units along an
    // orthogonal axis. Their union has long planar beam surfaces and retains
    // actual openings. This is a visual hypothesis ONLY: its physical
    // occupancy is never substituted for collision geometry.
    //
    // Runs are sourced from cells independently supported by all four
    // silhouettes; reprojection of the resulting boundary is mandatory before
    // publication. Diagonal, curved and compact shapes remain on the normal
    // reconstruction route when no certified axial hypothesis exists.
    [[nodiscard]] inline std::vector<FirstPersonVisualHullFace>
        BuildFirstPersonVisualHullStructuralBeamFaces(
            const FirstPersonVisualHull& hull,
            size_t originalFaceCount, size_t maximumFaces = 512)
    {
        if (!hull.valid || hull.occupied.empty()
            || !(hull.step > 0.0f) || originalFaceCount < 64
            || maximumFaces < 6)
            return {};

        const int32_t size[3] = {
            int32_t(hull.sizeForward), int32_t(hull.sizeRight),
            int32_t(hull.sizeUp)
        };
        const size_t total = size_t(size[0]) * size_t(size[1]) * size_t(size[2]);
        if (total != hull.occupied.size() || total > 131072)
            return {};

        const int32_t minimumRun = std::max(
            4, int32_t(std::ceil(8.0f / hull.step)));
        // A straight physical beam can disappear for one or two voxel
        // centres because integer isometric sprite rasterisation samples
        // its thin silhouette between pixels. A short enclosed gap is not
        // a true structural endpoint. Never extrapolate past measured runs;
        // native reprojection must independently certify every repaired gap.
        const int32_t maximumInternalGap = std::max(
            1, int32_t(std::ceil(2.0f / hull.step)));
        std::vector<uint8_t> beams(total, 0);
        const auto index = [&](int32_t f, int32_t r, int32_t u) -> size_t {
            return (size_t(u) * size_t(size[1]) + size_t(r))
                * size_t(size[0]) + size_t(f);
        };
        for (int32_t axis = 0; axis < 3; ++axis)
        {
            const int32_t alongSize = size[axis];
            const int32_t aSize = axis == 0 ? size[1] : size[0];
            const int32_t bSize = axis == 2 ? size[1] : size[2];
            if (alongSize < minimumRun)
                continue;
            const auto lineIndex = [&](int32_t along, int32_t a, int32_t b) {
                const int32_t f = axis == 0 ? along : a;
                const int32_t r = axis == 1 ? along : (axis == 0 ? a : b);
                const int32_t u = axis == 2 ? along : b;
                return index(f, r, u);
            };
            for (int32_t b = 0; b < bSize; ++b)
            for (int32_t a = 0; a < aSize; ++a)
            {
                int32_t along = 0;
                while (along < alongSize)
                {
                    if (hull.occupied[lineIndex(along, a, b)] == 0)
                    {
                        ++along;
                        continue;
                    }
                    const int32_t start = along;
                    while (along < alongSize)
                    {
                        if (hull.occupied[lineIndex(along, a, b)] != 0)
                        {
                            ++along;
                            continue;
                        }
                        const int32_t gapStart = along;
                        while (along < alongSize
                            && along - gapStart <= maximumInternalGap
                            && hull.occupied[lineIndex(along, a, b)] == 0)
                            ++along;
                        // Do not connect genuine openings or extend a
                        // beam beyond its last supported endpoint.
                        if (along >= alongSize
                            || along - gapStart > maximumInternalGap
                            || hull.occupied[lineIndex(along, a, b)] == 0)
                        {
                            along = gapStart;
                            break;
                        }
                        ++along;
                    }
                    if (along - start < minimumRun)
                    {
                        // A short run is not structure. Advance across the
                        // unsupported separator before scanning a new run.
                        while (along < alongSize
                            && hull.occupied[lineIndex(along, a, b)] == 0)
                            ++along;
                        continue;
                    }
                    for (int32_t k = start; k < along; ++k)
                        beams[lineIndex(k, a, b)] = 1;
                }
            }
        }

        const size_t kept = size_t(std::count(beams.begin(), beams.end(), uint8_t{ 1 }));
        const size_t occupied = size_t(std::count_if(
            hull.occupied.begin(), hull.occupied.end(),
            [](uint8_t cell) { return cell != 0; }));
        // Require a meaningful set of beams, rather than isolated rods
        // surviving from an otherwise unrelated 3D shape.
        if (kept < size_t(minimumRun) * 2 || kept * 3 < occupied)
            return {};
        FirstPersonVisualHull candidate = hull;
        candidate.occupied = std::move(beams);
        candidate.refinedFaces.clear();
        auto faces = BuildFirstPersonVisualHullVoxelBoundaryFaces(candidate);
        // The structural hypothesis is valuable only if it materially
        // removes the visual fragmentation, not if it merely renames voxels.
        if (faces.empty() || faces.size() > maximumFaces
            || faces.size() * 3 >= originalFaceCount * 2)
            return {};
        return faces;
    }

    struct FirstPersonVisualHullView
    {
        uint8_t imageDirection{};
        ImageIndex image{};
        const G1Element* g1{};
        FirstPersonSilhouette observed{};
    };

    // A four-view visual hull contains *possible* matter, not just the
    // true scaffold. Multiple candidate rods can project onto the same
    // painted bar, so preserving every long run overpaints the source.
    // Choose a compact set of straight beams that contributes NEW observed
    // pixels in all four views, penalising predictions outside the artwork.
    // This does not use object IDs, colour heuristics or any scene-specific
    // topology. The final complete union is separately round-trip certified.
    template<typename ProjectPoint>
    [[nodiscard]] inline std::vector<FirstPersonVisualHullFace>
        BuildFirstPersonVisualHullSourceMatchedBeamFaces(
            const FirstPersonVisualHull& hull,
            const std::vector<FirstPersonVisualHullView>& views,
            ProjectPoint&& projectPoint,
            size_t originalFaceCount, size_t maximumFaces = 512)
    {
        if (!hull.valid || hull.occupied.empty() || views.size() != 4
            || originalFaceCount < 64 || maximumFaces < 6)
            return {};
        const int32_t dims[3]{
            int32_t(hull.sizeForward), int32_t(hull.sizeRight),
            int32_t(hull.sizeUp)
        };
        const size_t cellCount = size_t(dims[0]) * size_t(dims[1]) * size_t(dims[2]);
        if (cellCount != hull.occupied.size() || cellCount > 65536)
            return {};
        std::array<bool,4> seen{};
        for (const auto& view : views)
        {
            if (view.imageDirection >= 4 || view.observed.empty()
                || seen[view.imageDirection])
                return {};
            seen[view.imageDirection] = true;
        }
        constexpr size_t kMaximumLines = 128;
        constexpr size_t kMaximumSelected = 48;
        const int32_t minLength = std::max(4, int32_t(std::ceil(8.0f / hull.step)));
        const int32_t maxGap = std::max(1, int32_t(std::ceil(2.0f / hull.step)));
        const auto index = [&](int32_t f, int32_t r, int32_t u) -> size_t {
            return (size_t(u) * size_t(dims[1]) + size_t(r))
                * size_t(dims[0]) + size_t(f);
        };
        struct Candidate
        {
            std::vector<size_t> cells;
            std::array<FirstPersonSilhouette,4> projected{};
        };
        std::vector<Candidate> candidates;
        struct AxialRun
        {
            int32_t axis, start, end, a, b;
        };
        std::vector<AxialRun> runs;
        for (int32_t axis = 0; axis < 3; ++axis)
        {
            const int32_t alongSize = dims[axis];
            if (alongSize < minLength)
                continue;
            const int32_t aSize = axis == 0 ? dims[1] : dims[0];
            const int32_t bSize = axis == 2 ? dims[1] : dims[2];
            const auto lineIndex = [&](int32_t along, int32_t a, int32_t b) {
                const int32_t f = axis == 0 ? along : a;
                const int32_t r = axis == 1 ? along : (axis == 0 ? a : b);
                const int32_t u = axis == 2 ? along : b;
                return index(f, r, u);
            };
            for (int32_t b = 0; b < bSize; ++b)
            for (int32_t a = 0; a < aSize; ++a)
            {
                int32_t along = 0;
                while (along < alongSize)
                {
                    if (!hull.occupied[lineIndex(along, a, b)])
                    {
                        ++along;
                        continue;
                    }
                    const int32_t start = along;
                    while (along < alongSize)
                    {
                        if (hull.occupied[lineIndex(along, a, b)])
                        {
                            ++along;
                            continue;
                        }
                        const int32_t gapStart = along;
                        while (along < alongSize
                            && along - gapStart <= maxGap
                            && !hull.occupied[lineIndex(along, a, b)])
                            ++along;
                        if (along >= alongSize || along - gapStart > maxGap
                            || !hull.occupied[lineIndex(along, a, b)])
                        {
                            along = gapStart;
                            break;
                        }
                        ++along;
                    }
                    if (along - start >= minLength)
                    {
                        // Most candidate runs in dense native lattice art
                        // are ambiguous. Reject unbounded source hypotheses
                        // before rasterising 128 individual voxel primitives.
                        if (runs.size() >= kMaximumLines * 2)
                            return {};
                        runs.push_back({ axis, start, along, a, b });
                    }
                    // An enclosed gap that was too long to bridge is the
                    // start of another run; do not loop on the same cell.
                    while (along < alongSize
                        && !hull.occupied[lineIndex(along, a, b)])
                        ++along;
                }
            }
        }
        // Scan the inexpensive occupancy runs before any primitive
        // allocations or four-view raster work. Preserve the normal 128
        // projected-candidate limit for less ambiguous native artwork.
        for (const auto& run : runs)
        {
            const auto lineIndex = [&](int32_t along) {
                const int32_t f = run.axis == 0 ? along : run.a;
                const int32_t r = run.axis == 1 ? along : (run.axis == 0 ? run.a : run.b);
                const int32_t u = run.axis == 2 ? along : run.b;
                return index(f, r, u);
            };
            if (candidates.size() >= kMaximumLines)
                return {};
            Candidate line{};
            line.cells.reserve(size_t(run.end - run.start));
            FirstPersonVisualHull primitive = hull;
            primitive.occupied.assign(cellCount, 0);
            primitive.refinedFaces.clear();
            for (int32_t k = run.start; k < run.end; ++k)
            {
                const auto cell = lineIndex(k);
                primitive.occupied[cell] = 1;
                line.cells.push_back(cell);
            }
            const auto primitiveFaces = BuildFirstPersonVisualHullVoxelBoundaryFaces(primitive);
            const auto triangles = TriangulateFirstPersonReconstructionFaces(primitiveFaces);
            if (triangles.empty())
                continue;
            bool valid = true;
            for (const auto& view : views)
            {
                auto predicted = RenderFirstPersonReconstructionSilhouette(
                    triangles, view.imageDirection, projectPoint);
                if (predicted.empty() || predicted.overflowed)
                {
                    valid = false;
                    break;
                }
                line.projected[view.imageDirection] = std::move(predicted);
            }
            if (valid)
                candidates.push_back(std::move(line));
        }
        if (candidates.size() < 2)
            return {};
        std::array<std::unordered_set<uint64_t>,4> covered{};
        std::vector<uint8_t> chosen(candidates.size(),0);
        std::vector<uint8_t> selectedCells(cellCount,0);
        size_t chosenCount = 0;
        for (; chosenCount < kMaximumSelected; ++chosenCount)
        {
            int64_t bestScore = 0;
            size_t best = candidates.size();
            for (size_t i = 0; i < candidates.size(); ++i)
            {
                if (chosen[i])
                    continue;
                int64_t newObserved = 0;
                int64_t newUnsupported = 0;
                for (const auto& view : views)
                {
                    const auto direction = view.imageDirection;
                    const auto& seenPixels = covered[direction];
                    for (const auto pixel : candidates[i].projected[direction].pixels)
                    {
                        if (seenPixels.contains(pixel))
                            continue;
                        if (view.observed.pixels.contains(pixel))
                            ++newObserved;
                        else
                            ++newUnsupported;
                    }
                }
                // A false intersection filament cannot win simply by
                // explaining a few source pixels if it creates many novel
                // pixels outside the four authored silhouettes.
                const int64_t score = newObserved * 2 - newUnsupported * 5;
                if (score > bestScore)
                {
                    bestScore = score;
                    best = i;
                }
            }
            if (best == candidates.size())
                break;
            chosen[best] = 1;
            for (const auto cell : candidates[best].cells)
                selectedCells[cell] = 1;
            for (const auto& view : views)
            {
                const auto direction = view.imageDirection;
                const auto& pixels = candidates[best].projected[direction].pixels;
                covered[direction].insert(pixels.begin(), pixels.end());
            }
        }
        if (chosenCount < 2)
            return {};
        // Final reverse coverage proof: remove any candidate beam whose
        // entire projection is already explained by the other selected
        // beams in ALL four views. Such a phantom rod has no independent
        // image evidence; retaining it harms geometry but cannot improve
        // the source silhouette. Unlike heuristic pruning this changes
        // exactly zero predicted pixels (including negative/open regions).
        for (size_t i = 0; i < candidates.size(); ++i)
        {
            if (!chosen[i])
                continue;
            bool redundant = true;
            for (uint8_t direction = 0; direction < 4 && redundant; ++direction)
            {
                std::unordered_set<uint64_t> others;
                for (size_t j = 0; j < candidates.size(); ++j)
                    if (chosen[j] && j != i)
                    {
                        const auto& pixels = candidates[j].projected[direction].pixels;
                        others.insert(pixels.begin(), pixels.end());
                    }
                for (const auto pixel : candidates[i].projected[direction].pixels)
                    if (!others.contains(pixel))
                    {
                        redundant = false;
                        break;
                    }
            }
            if (redundant && chosenCount > 2)
            {
                chosen[i] = 0;
                --chosenCount;
            }
        }
        std::fill(selectedCells.begin(), selectedCells.end(), 0);
        for (size_t i = 0; i < candidates.size(); ++i)
            if (chosen[i])
                for (const auto cell : candidates[i].cells)
                    selectedCells[cell] = 1;
        FirstPersonVisualHull selected = hull;
        selected.occupied = std::move(selectedCells);
        selected.refinedFaces.clear();
        auto output = BuildFirstPersonVisualHullVoxelBoundaryFaces(selected);
        if (output.empty() || output.size() > maximumFaces
            || output.size() * 3 >= originalFaceCount * 2)
            return {};
        return output;
    }


    // Extract connected diagonal members from the four-view intersection
    // field. A visual hull is a set of POSSIBLE solid cells; a long line
    // through it is not sufficient evidence of a physical beam. Rank every
    // lattice diagonal against the *source images* before projecting bounded
    // continuous prism candidates and publishing only a certified union.
    //
    // Unlike a chain of voxel cubes, these faces are six genuinely planar
    // quads with a continuous world-space axis. Material is still painted
    // from the native source projections, and the occupancy/collision body
    // is never changed.
    template<typename ProjectPoint>
    [[nodiscard]] inline std::vector<FirstPersonVisualHullFace>
        BuildFirstPersonVisualHullSourceMatchedOrientedBeamFaces(
            const FirstPersonVisualHull& hull,
            const std::vector<FirstPersonVisualHullView>& views,
            ProjectPoint&& projectPoint,
            size_t originalFaceCount, size_t maximumFaces = 512)
    {
        if (!hull.valid || hull.occupied.empty()
            || !(hull.step > 0.0f) || views.size() != 4
            || originalFaceCount < 64 || maximumFaces < 12)
            return {};
        const int32_t dims[3] = {
            int32_t(hull.sizeForward), int32_t(hull.sizeRight),
            int32_t(hull.sizeUp)
        };
        const size_t cells = size_t(dims[0]) * size_t(dims[1]) * size_t(dims[2]);
        if (cells != hull.occupied.size() || cells > 65536)
            return {};
        std::array<bool, 4> seen{};
        for (const auto& view : views)
        {
            if (view.imageDirection > 3 || seen[view.imageDirection]
                || view.observed.empty())
                return {};
            seen[view.imageDirection] = true;
        }
        constexpr int8_t directions[10][3] = {
            { 0, 1, 1 }, { 0, 1, -1 },
            { 1, 0, 1 }, { 1, 0, -1 },
            { 1, 1, 0 }, { 1, -1, 0 },
            { 1, 1, 1 }, { 1, 1, -1 },
            { 1, -1, 1 }, { 1, -1, -1 }
        };
        struct Run
        {
            int32_t start[3]{};
            int32_t delta[3]{};
            int32_t length{};
            float score{};
        };
        const auto contains = [&](int32_t f, int32_t r, int32_t u) {
            return hull.contains(f, r, u);
        };
        std::vector<Run> ranked;
        // This pass makes no individual triangle meshes or pixel rasters:
        // large ornamental objects have thousands of false intersection
        // lines. A cheap centre-line check cuts the expensive hypotheses.
        for (const auto& d : directions)
        for (int32_t u = 0; u < dims[2]; ++u)
        for (int32_t r = 0; r < dims[1]; ++r)
        for (int32_t f = 0; f < dims[0]; ++f)
        {
            if (!contains(f, r, u)
                || contains(f - d[0], r - d[1], u - d[2]))
                continue;
            int32_t length = 0;
            while (contains(f + length * d[0], r + length * d[1],
                       u + length * d[2]))
                ++length;
            if (length < 4)
                continue;
            int32_t minimumExact = length;
            int32_t exactTotal = 0;
            for (const auto& view : views)
            {
                int32_t exact = 0;
                for (int32_t i = 0; i < length; ++i)
                {
                    const auto point = hull.centre(
                        f + i * d[0], r + i * d[1], u + i * d[2]);
                    const auto uv = projectPoint(view.imageDirection, point);
                    if (view.observed.contains(
                            int32_t(std::lround(uv[0])),
                            int32_t(std::lround(uv[1]))))
                        ++exact;
                }
                minimumExact = std::min(minimumExact, exact);
                exactTotal += exact;
            }
            if (minimumExact * 2 < length
                || exactTotal * 5 < length * 12)
                continue;
            Run run{};
            run.start[0] = f;
            run.start[1] = r;
            run.start[2] = u;
            for (int32_t k = 0; k < 3; ++k)
                run.delta[k] = d[k];
            run.length = length;
            run.score = float(minimumExact) / float(length) * 3.0f
                + float(exactTotal) / float(length * 4)
                + float(length) * 0.02f;
            ranked.push_back(run);
        }
        if (ranked.size() < 2)
            return {};
        std::sort(ranked.begin(), ranked.end(), [](const Run& a, const Run& b) {
            // Stable lattice-scan order breaks equal-score ties.
            return a.score > b.score;
        });
        constexpr size_t kMaximumProjectedRuns = 96;
        if (ranked.size() > kMaximumProjectedRuns)
            ranked.resize(kMaximumProjectedRuns);

        const auto add = [](FirstPersonVec3 a, FirstPersonVec3 b) {
            return FirstPersonVec3{ a.x + b.x, a.y + b.y, a.z + b.z };
        };
        const auto sub = [](FirstPersonVec3 a, FirstPersonVec3 b) {
            return FirstPersonVec3{ a.x - b.x, a.y - b.y, a.z - b.z };
        };
        const auto scale = [](FirstPersonVec3 a, float k) {
            return FirstPersonVec3{ a.x * k, a.y * k, a.z * k };
        };
        const auto unit = [&](FirstPersonVec3 v) {
            const float n = std::sqrt(FpDot(v, v));
            return n > 1.0e-6f ? scale(v, 1.0f / n) : FirstPersonVec3{};
        };
        const auto makeFaces = [&](const Run& run) {
            std::vector<FirstPersonVisualHullFace> result;
            const auto direction = unit({
                float(run.delta[0]), float(run.delta[1]), float(run.delta[2])
            });
            auto start = hull.centre(
                run.start[0], run.start[1], run.start[2]);
            auto end = hull.centre(
                run.start[0] + (run.length - 1) * run.delta[0],
                run.start[1] + (run.length - 1) * run.delta[1],
                run.start[2] + (run.length - 1) * run.delta[2]);
            start = sub(start, scale(direction, 0.5f * hull.step));
            end = add(end, scale(direction, 0.5f * hull.step));
            const auto reference = std::abs(direction.z) < 0.8f
                ? FirstPersonVec3{ 0, 0, 1 }
                : FirstPersonVec3{ 0, 1, 0 };
            const auto side = unit(FpCross(direction, reference));
            const auto across = unit(FpCross(direction, side));
            // One native 2-unit cell wide. These are geometric side faces,
            // not semi-transparent crossed sprite planes.
            const float radius = 0.5f * hull.step;
            std::array<FirstPersonVec3, 4> startRing{}, endRing{};
            for (int i = 0; i < 4; ++i)
            {
                const float sx = i == 0 || i == 3 ? 1.0f : -1.0f;
                const float sy = i < 2 ? 1.0f : -1.0f;
                const auto offset = add(
                    scale(side, sx * radius), scale(across, sy * radius));
                startRing[i] = add(start, offset);
                endRing[i] = add(end, offset);
            }
            const auto appendQuad =
                [&](std::array<FirstPersonVec3, 4> corners,
                    FirstPersonVec3 outward) {
                    auto normal = unit(FpCross(
                        sub(corners[1], corners[0]),
                        sub(corners[2], corners[0])));
                    if (FpDot(normal, normal) < 0.5f)
                        return;
                    if (FpDot(normal, outward) < 0.0f)
                    {
                        std::swap(corners[1], corners[3]);
                        normal = scale(normal, -1.0f);
                    }
                    FirstPersonVisualHullFace face{};
                    face.corners = corners;
                    face.normal = normal;
                    face.kind = normal.z < -0.70f
                        ? FirstPersonVisualHullFaceKind::bottom
                        : normal.z > 0.70f
                            ? FirstPersonVisualHullFaceKind::top
                            : std::abs(normal.x) > std::abs(normal.y)
                                ? (normal.x < 0.0f
                                    ? FirstPersonVisualHullFaceKind::minForward
                                    : FirstPersonVisualHullFaceKind::maxForward)
                                : (normal.y < 0.0f
                                    ? FirstPersonVisualHullFaceKind::minRight
                                    : FirstPersonVisualHullFaceKind::maxRight);
                    result.push_back(face);
                };
            appendQuad({ startRing[3], startRing[2], startRing[1], startRing[0] },
                scale(direction, -1.0f));
            appendQuad({ endRing[0], endRing[1], endRing[2], endRing[3] }, direction);
            for (int i = 0; i < 4; ++i)
            {
                const int j = (i + 1) % 4;
                const auto outward = add(
                    sub(startRing[i], start), sub(startRing[j], start));
                appendQuad({ startRing[i], startRing[j], endRing[j], endRing[i] }, outward);
            }
            return result;
        };

        struct Candidate
        {
            std::vector<FirstPersonVisualHullFace> faces;
            std::array<FirstPersonSilhouette, 4> projected{};
        };
        std::vector<Candidate> candidates;
        for (const auto& run : ranked)
        {
            auto faces = makeFaces(run);
            if (faces.size() != 6)
                continue;
            auto triangles = TriangulateFirstPersonReconstructionFaces(faces);
            if (triangles.empty())
                continue;
            Candidate c{};
            c.faces = std::move(faces);
            bool valid = true;
            for (const auto& view : views)
            {
                auto pixels = RenderFirstPersonReconstructionSilhouette(
                    triangles, view.imageDirection, projectPoint);
                if (pixels.empty() || pixels.overflowed)
                {
                    valid = false;
                    break;
                }
                c.projected[view.imageDirection] = std::move(pixels);
            }
            if (valid)
                candidates.push_back(std::move(c));
        }
        if (candidates.size() < 2)
            return {};
        std::array<std::unordered_set<uint64_t>, 4> covered{};
        std::vector<uint8_t> chosen(candidates.size(), 0);
        size_t chosenCount = 0;
        constexpr size_t kMaximumBeams = 64;
        for (; chosenCount < kMaximumBeams
                 && (chosenCount + 1) * 6 <= maximumFaces; ++chosenCount)
        {
            size_t best = candidates.size();
            int64_t bestScore = 0;
            for (size_t i = 0; i < candidates.size(); ++i)
            {
                if (chosen[i])
                    continue;
                int64_t observed = 0, unsupported = 0;
                for (const auto& view : views)
                {
                    const auto d = view.imageDirection;
                    for (uint64_t px : candidates[i].projected[d].pixels)
                    {
                        if (covered[d].contains(px))
                            continue;
                        if (view.observed.pixels.contains(px))
                            ++observed;
                        else
                            ++unsupported;
                    }
                }
                const int64_t value = 2 * observed - 5 * unsupported;
                if (value > bestScore)
                {
                    bestScore = value;
                    best = i;
                }
            }
            if (best == candidates.size())
                break;
            chosen[best] = 1;
            for (const auto& view : views)
            {
                const auto d = view.imageDirection;
                const auto& pixels = candidates[best].projected[d].pixels;
                covered[d].insert(pixels.begin(), pixels.end());
            }
        }
        if (chosenCount < 2)
            return {};
        // Remove rods that contribute zero unique source-view pixels.
        // Otherwise a long intersection filament may survive despite
        // never being visible in any of the four authored directions.
        for (size_t i = 0; i < candidates.size(); ++i)
        {
            if (!chosen[i] || chosenCount <= 2)
                continue;
            bool redundant = true;
            for (const auto& view : views)
            {
                if (!redundant)
                    break;
                std::unordered_set<uint64_t> others;
                const auto d = view.imageDirection;
                for (size_t j = 0; j < candidates.size(); ++j)
                {
                    if (chosen[j] && j != i)
                    {
                        const auto& px = candidates[j].projected[d].pixels;
                        others.insert(px.begin(), px.end());
                    }
                }
                for (uint64_t px : candidates[i].projected[d].pixels)
                {
                    if (!others.contains(px))
                    {
                        redundant = false;
                        break;
                    }
                }
            }
            if (redundant)
            {
                chosen[i] = 0;
                --chosenCount;
            }
        }
        std::vector<FirstPersonVisualHullFace> output;
        for (size_t i = 0; i < candidates.size(); ++i)
            if (chosen[i])
                output.insert(output.end(),
                    candidates[i].faces.begin(), candidates[i].faces.end());
        if (output.empty() || output.size() > maximumFaces
            || output.size() * 3 >= originalFaceCount * 2)
            return {};
        return output;
    }

    // Native stroke triangulation is implemented after four-view hull types.
#include "FirstPersonSourceTriangulation.inc"

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
        // A thin reconstructed lattice may have an equally accurate but
        // visibly smoother conforming surface. This preference is opt-in;
        // other families continue to demand a strictly better certificate.
        bool preferCertifiedSmoothSurface = false;
        bool allowStructuralBeamReconstruction = false;
        // Strictly isolated last resort for sparse source-backed truss artwork.
        bool allowSourceTriangulatedWireFallback = false;
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
        PointSupported&& pointSupported,
        FirstPersonVisualHullAttempt* attempt = nullptr)
    {
        FirstPersonVisualHull result{};
        if (attempt != nullptr)
        {
            *attempt = {};
            attempt->stage = 1;
            attempt->nativeViews = uint8_t(std::min<size_t>(views.size(), 255));
        }
        if (views.size() < config.minimumViews
            || !(bounds.step > 0.0f)
            || bounds.maxForward <= bounds.minForward
            || bounds.maxRight <= bounds.minRight
            || bounds.maxUp <= bounds.minUp)
            return result;
        if (attempt != nullptr)
            attempt->stage = 2;

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
        if (attempt != nullptr)
            attempt->stage = 3;

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
        if (attempt != nullptr)
            attempt->occupiedCells = uint32_t(occupiedCount);
        if (occupiedCount < config.minimumOccupiedCells
            || occupiedCount > config.maximumOccupiedCells)
            return {};
        if (attempt != nullptr)
            attempt->stage = 4;

        // The boundary extractor treats valid=true as permission to expose
        // geometry. Mark this local candidate provisionally so both the raw
        // boundary and evidence-driven alternatives can be forward-rendered.
        // Any failed admission below returns a fresh invalid hull.
        result.valid = true;

        const auto voxelFaces =
            BuildFirstPersonVisualHullVoxelBoundaryFaces(
                result);
        if (attempt != nullptr)
            attempt->voxelFaces = uint32_t(voxelFaces.size());
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
        if (attempt != nullptr)
            attempt->stage = 5;

        auto selectedCertificate =
            voxelCertificate;
        if (config.allowStructuralBeamReconstruction)
        {
            // Independently render every structural hypothesis through the
            // four native observations. A single long-run extraction is not
            // sufficient for cluttered visual hulls: phantom beams can share
            // the same source pixels, so also evaluate a sparse image-matched
            // subset before giving up or publishing voxel fragments.
            const auto tryStructuralCandidate =
                [&](std::vector<FirstPersonVisualHullFace> faces) {
                    if (faces.empty())
                        return;
                    const auto certificate = CertifyFirstPersonReconstruction(
                        TriangulateFirstPersonReconstructionFaces(faces),
                        views, projectPoint);
                    const bool highQuality =
                        passesAdmission(certificate)
                        && certificate.minimumIntersectionOverUnion >= 0.85f
                        && certificate.minimumCandidateCoverage >= 0.90f
                        && certificate.minimumObservedCoverage >= 0.90f
                        && certificate.maximumEdgeError <= 2;
                    if (!highQuality)
                        return;
                    if (passesAdmission(selectedCertificate)
                        && !FirstPersonRoundTripNonWorse(
                            certificate, selectedCertificate))
                        return;
                    result.refinedFaces = std::move(faces);
                    result.structuralBeamSelected = true;
                    selectedCertificate = certificate;
                };
            tryStructuralCandidate(BuildFirstPersonVisualHullStructuralBeamFaces(
                result, voxelFaces.size(),
                std::min<size_t>(512, config.maximumContinuousSurfaceFaces)));
            if (!result.structuralBeamSelected)
                tryStructuralCandidate(BuildFirstPersonVisualHullSourceMatchedBeamFaces(
                    result, views, projectPoint, voxelFaces.size(),
                    std::min<size_t>(512, config.maximumContinuousSurfaceFaces)));
            if (!result.structuralBeamSelected)
                tryStructuralCandidate(BuildFirstPersonVisualHullSourceMatchedOrientedBeamFaces(
                    result, views, projectPoint, voxelFaces.size(),
                    std::min<size_t>(512, config.maximumContinuousSurfaceFaces)));
        }
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
                    ? !(config.preferCertifiedSmoothSurface
                        ? FirstPersonRoundTripNonWorse(
                            candidateCertificate, selectedCertificate)
                        : FirstPersonRoundTripStrictlyImproves(
                            candidateCertificate, selectedCertificate))
                    : !FirstPersonRoundTripNonWorse(
                        candidateCertificate, selectedCertificate))
                    continue;

                result.refinedFaces =
                    candidateFaces;
                result.structuralBeamSelected = false;
                selectedCertificate =
                    candidateCertificate;
            }
        }

        // Source-backed trusses may remain broken or severely overpainted
        // after ordinary visual-hull admission. Independently triangulate
        // their authored strokes only when the selected conventional model
        // is rejected or demonstrably overpainted. Keep previously certified
        // structures unless the candidate improves total and worst-view
        // source disagreement. This runs AFTER smooth-surface selection.
        if (config.allowSourceTriangulatedWireFallback
            && !result.structuralBeamSelected
            && voxelFaces.size() >= 200
            // A fully source-supported solid never pays for expensive
            // artistic-stroke inference. Perforated, overpainted hulls may
            // still require replacement even after passing loose admission.
            && (!passesAdmission(selectedCertificate)
                || (voxelFaces.size() >= 900
                    && selectedCertificate.minimumCandidateCoverage < 0.80f
                    && selectedCertificate.minimumObservedCoverage >= 0.85f)))
        {
            // The long-stroke and coverage-ranked candidates explain
            // different native features. Admit them independently; selecting
            // a better 2D line set cannot by itself justify 3D promotion.
            for (bool expandedPairs : { false, true })
            {
                // Never replace a previously selected or already admitted
                // four-pair hypothesis just to search more stereo pairs.
                if (expandedPairs
                    && (result.sourceTriangulatedWireSelected
                        || passesAdmission(selectedCertificate)))
                    break;
                for (bool prioritizeCoverage : { false, true })
                {
                auto wireFaces = BuildFirstPersonSourceTriangulatedWireFaces(
                    result, views, projectPoint, voxelFaces.size(),
                    std::min<size_t>(768, config.maximumContinuousSurfaceFaces),
                    prioritizeCoverage, expandedPairs, true);
                if (wireFaces.empty())
                    continue;
                const auto wireCertificate = CertifyFirstPersonReconstruction(
                    TriangulateFirstPersonReconstructionFaces(wireFaces),
                    views, projectPoint);
                // A thin artistic lattice need not project like a solid
                // occupancy volume. It must improve on the BEST selected
                // native certificate while retaining observed coverage and
                // source-supported pixels across all four directions.
                if (wireCertificate.valid
                    && wireCertificate.viewCount == 4
                    && wireCertificate.minimumIntersectionOverUnion >= 0.43f
                    && wireCertificate.minimumCandidateCoverage >= 0.64f
                    && wireCertificate.minimumObservedCoverage >= 0.54f
                    && wireCertificate.maximumEdgeError <= 8
                    && wireCertificate.symmetricDifferencePixels
                        < selectedCertificate.symmetricDifferencePixels
                    && wireCertificate.maximumViewSymmetricDifferencePixels
                        <= selectedCertificate.maximumViewSymmetricDifferencePixels)
                {
                    result.refinedFaces = std::move(wireFaces);
                    result.structuralBeamSelected = true;
                    result.sourceTriangulatedWireSelected = true;
                    selectedCertificate = wireCertificate;
                }
                }
            }
        }

        if (attempt != nullptr)
        {
            attempt->certificateViews = uint32_t(selectedCertificate.viewCount);
            attempt->minimumIoU = selectedCertificate.minimumIntersectionOverUnion;
            attempt->candidateCoverage = selectedCertificate.minimumCandidateCoverage;
            attempt->observedCoverage = selectedCertificate.minimumObservedCoverage;
            attempt->maximumEdgeError = selectedCertificate.maximumEdgeError;
            attempt->disagreementPixels = selectedCertificate.symmetricDifferencePixels;
        }
        if (!passesAdmission(selectedCertificate)
            && !result.sourceTriangulatedWireSelected)
            return {};
        if (attempt != nullptr)
            attempt->stage = 6;

        result.continuousSurfaceRefined =
            !result.refinedFaces.empty() && !result.structuralBeamSelected;
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
        if (attempt != nullptr)
            result.attempt = *attempt;
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
