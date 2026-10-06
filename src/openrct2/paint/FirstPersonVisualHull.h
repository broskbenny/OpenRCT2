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
#include <vector>

namespace OpenRCT2::Paint
{
    struct FirstPersonVisualHullTextureView
    {
        uint8_t imageDirection{};
        ImageIndex image{};
    };

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
        // Exact surface-level round-trip evidence. The legacy
        // coverage fields remain directly readable because several
        // reconstruction policies use them as conservative admission gates.
        float averageIntersectionOverUnion{};
        float minimumIntersectionOverUnion{};
        float minimumCandidateCoverage{};
        float minimumObservedCoverage{};
        int32_t maximumEdgeError{};
        size_t roundTripViewCount{};

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
        BuildFirstPersonVisualHullBoundaryFaces(
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
        // geometry. Mark this local candidate provisionally so it can be
        // forward-rendered; any failed certificate below returns a fresh
        // invalid hull, so provisional validity never escapes this function.
        result.valid = true;

        // Certify the geometry we will actually expose, not a proxy made from
        // inflated voxel-centre samples. This makes the admission test
        // independent of the reconstruction method and suitable for future
        // non-axis-aligned fitted meshes.
        const auto boundaryFaces =
            BuildFirstPersonVisualHullBoundaryFaces(result);
        const auto triangles =
            TriangulateFirstPersonReconstructionFaces(
                boundaryFaces);
        const auto certificate =
            CertifyFirstPersonReconstruction(
                triangles, views, projectPoint);
        if (!certificate.valid
            || certificate.viewCount < config.minimumViews
            || certificate.minimumIntersectionOverUnion
                < config.minimumIntersectionOverUnion
            || certificate.minimumCandidateCoverage
                < config.minimumCandidateCoverage
            || certificate.minimumObservedCoverage
                < config.minimumObservedCoverage
            || certificate.maximumEdgeError
                > config.maximumEdgeError)
            return {};

        result.averageIntersectionOverUnion =
            certificate.averageIntersectionOverUnion;
        result.minimumIntersectionOverUnion =
            certificate.minimumIntersectionOverUnion;
        result.minimumCandidateCoverage =
            certificate.minimumCandidateCoverage;
        result.minimumObservedCoverage =
            certificate.minimumObservedCoverage;
        result.maximumEdgeError =
            certificate.maximumEdgeError;
        result.roundTripViewCount =
            certificate.viewCount;
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
