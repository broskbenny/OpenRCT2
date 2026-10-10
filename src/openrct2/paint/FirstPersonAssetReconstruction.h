/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "../interface/ScreenCoords.hpp"
#include "../world/Location.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace OpenRCT2::Paint
{
    struct FirstPersonVerticalInterval
    {
        int32_t low{};
        int32_t high{};
    };

    [[nodiscard]] inline std::vector<FirstPersonVerticalInterval>
        SubtractFirstPersonVerticalCoverage(
            FirstPersonVerticalInterval source,
            std::vector<FirstPersonVerticalInterval> coverage)
    {
        std::vector<FirstPersonVerticalInterval> result;
        if (source.high <= source.low)
            return result;

        for (auto& interval : coverage)
        {
            interval.low = std::max(interval.low, source.low);
            interval.high = std::min(interval.high, source.high);
        }
        coverage.erase(
            std::remove_if(
                coverage.begin(), coverage.end(),
                [](const auto& interval) {
                    return interval.high <= interval.low;
                }),
            coverage.end());
        std::sort(
            coverage.begin(), coverage.end(),
            [](const auto& a, const auto& b) {
                return a.low < b.low
                    || (a.low == b.low && a.high < b.high);
            });

        int32_t cursor = source.low;
        for (const auto& interval : coverage)
        {
            if (interval.low > cursor)
                result.push_back({ cursor, interval.low });
            cursor = std::max(cursor, interval.high);
            if (cursor >= source.high)
                break;
        }
        if (cursor < source.high)
            result.push_back({ cursor, source.high });
        return result;
    }

    [[nodiscard]] inline bool FirstPersonVerticalPointCoveredAbove(
        int32_t z, const std::vector<FirstPersonVerticalInterval>& coverage)
    {
        return std::any_of(
            coverage.begin(), coverage.end(),
            [z](const auto& interval) {
                return interval.low <= z && interval.high > z;
            });
    }

    enum class FirstPersonOccupancyFaceKind : uint8_t
    {
        minX,
        maxX,
        minY,
        maxY,
        top,
        bottom,
    };

    struct FirstPersonOccupancyFace
    {
        std::array<CoordsXYZ, 4> corners{};
        uint16_t sequence{};
        FirstPersonOccupancyFaceKind kind{};
        uint8_t sourceDirection{};
        bool textureFallbackOnly = false;
    };

    template<typename TCell>
    [[nodiscard]] std::vector<FirstPersonOccupancyFace>
        BuildFirstPersonQuarterCellOccupancyFaces(
            const std::vector<TCell>& cells)
    {
        std::vector<FirstPersonOccupancyFace> result;
        result.reserve(cells.size() * 6);

        const auto coverageAt =
            [&](int32_t qx, int32_t qy) {
                std::vector<FirstPersonVerticalInterval> intervals;
                for (const auto& other : cells)
                {
                    if (other.qx == qx && other.qy == qy
                        && other.highZ > other.lowZ)
                        intervals.push_back({ other.lowZ, other.highZ });
                }
                return intervals;
            };

        for (const auto& cell : cells)
        {
            if (cell.highZ <= cell.lowZ)
                continue;
            const int32_t x0 = cell.qx * 16;
            const int32_t y0 = cell.qy * 16;
            const int32_t x1 = x0 + 16;
            const int32_t y1 = y0 + 16;
            const int32_t z0 = cell.lowZ;
            const int32_t z1 = cell.highZ;
            const auto append =
                [&](FirstPersonOccupancyFaceKind kind,
                    std::array<CoordsXYZ, 4> corners) {
                    result.push_back({
                        corners, uint16_t(cell.sequence),
                        kind, 0, false
                    });
                };
            const auto appendSide =
                [&](FirstPersonOccupancyFaceKind kind,
                    int32_t neighbourQx, int32_t neighbourQy) {
                    const auto exposed =
                        SubtractFirstPersonVerticalCoverage(
                            { z0, z1 },
                            coverageAt(neighbourQx, neighbourQy));
                    for (const auto& interval : exposed)
                    {
                        switch (kind)
                        {
                            case FirstPersonOccupancyFaceKind::minX:
                                append(kind, { {
                                    { x0, y0, interval.low },
                                    { x0, y1, interval.low },
                                    { x0, y1, interval.high },
                                    { x0, y0, interval.high },
                                } });
                                break;
                            case FirstPersonOccupancyFaceKind::maxX:
                                append(kind, { {
                                    { x1, y1, interval.low },
                                    { x1, y0, interval.low },
                                    { x1, y0, interval.high },
                                    { x1, y1, interval.high },
                                } });
                                break;
                            case FirstPersonOccupancyFaceKind::minY:
                                append(kind, { {
                                    { x1, y0, interval.low },
                                    { x0, y0, interval.low },
                                    { x0, y0, interval.high },
                                    { x1, y0, interval.high },
                                } });
                                break;
                            case FirstPersonOccupancyFaceKind::maxY:
                                append(kind, { {
                                    { x0, y1, interval.low },
                                    { x1, y1, interval.low },
                                    { x1, y1, interval.high },
                                    { x0, y1, interval.high },
                                } });
                                break;
                            case FirstPersonOccupancyFaceKind::top:
                            case FirstPersonOccupancyFaceKind::bottom:
                                break;
                        }
                    }
                };

            appendSide(
                FirstPersonOccupancyFaceKind::minX,
                cell.qx - 1, cell.qy);
            appendSide(
                FirstPersonOccupancyFaceKind::maxX,
                cell.qx + 1, cell.qy);
            appendSide(
                FirstPersonOccupancyFaceKind::minY,
                cell.qx, cell.qy - 1);
            appendSide(
                FirstPersonOccupancyFaceKind::maxY,
                cell.qx, cell.qy + 1);

            const auto sameColumn = coverageAt(
                cell.qx, cell.qy);
            if (!FirstPersonVerticalPointCoveredAbove(
                    z1, sameColumn))
            {
                append(FirstPersonOccupancyFaceKind::top, { {
                    { x0, y0, z1 }, { x1, y0, z1 },
                    { x1, y1, z1 }, { x0, y1, z1 },
                } });
            }
            const bool coveredBelow =
                std::any_of(
                    sameColumn.begin(), sameColumn.end(),
                    [z0](const auto& interval) {
                        return interval.low < z0
                            && interval.high >= z0;
                    });
            if (!coveredBelow)
            {
                append(FirstPersonOccupancyFaceKind::bottom, { {
                    { x0, y1, z0 }, { x1, y1, z0 },
                    { x1, y0, z0 }, { x0, y0, z0 },
                } });
            }
        }
        return result;
    }

    [[nodiscard]] constexpr CoordsXY FirstPersonLargeSceneryPlacedPoint(
        CoordsXY tileOffset, CoordsXY assetPoint, uint8_t objectDirection)
    {
        const auto placedTile = tileOffset.rotate(objectDirection);
        const auto withinTile = assetPoint - tileOffset;
        const auto centred = withinTile - CoordsXY{ kCoordsXYHalfTile, kCoordsXYHalfTile };
        const auto rotatedWithin = centred.rotate(objectDirection)
            + CoordsXY{ kCoordsXYHalfTile, kCoordsXYHalfTile };
        return placedTile + rotatedWithin;
    }

    [[nodiscard]] constexpr CoordsXY FirstPersonNativeViewDirection(
        uint8_t rotation)
    {
        // Translate3DTo2DWithZ() depth increases toward the native camera.
        // At rotation zero, +X/+Y is therefore the viewer-facing quadrant.
        constexpr std::array<CoordsXY, 4> kDirections{ {
            { 1, 1 }, { -1, 1 }, { -1, -1 }, { 1, -1 },
        } };
        return kDirections[rotation & 3u];
    }

    [[nodiscard]] constexpr bool FirstPersonFaceVisibleFromNativeView(
        CoordsXY outwardNormal, uint8_t rotation)
    {
        const auto view = FirstPersonNativeViewDirection(rotation);
        return outwardNormal.x * view.x + outwardNormal.y * view.y > 0;
    }

    template<typename TAvailable>
    [[nodiscard]] std::optional<uint8_t>
        ChooseFirstPersonOccupancyFaceSourceDirection(
            FirstPersonOccupancyFaceKind kind, TAvailable&& available)
    {
        CoordsXY normal{};
        switch (kind)
        {
            case FirstPersonOccupancyFaceKind::minX: normal = { -1, 0 }; break;
            case FirstPersonOccupancyFaceKind::maxX: normal = { 1, 0 }; break;
            case FirstPersonOccupancyFaceKind::minY: normal = { 0, -1 }; break;
            case FirstPersonOccupancyFaceKind::maxY: normal = { 0, 1 }; break;
            case FirstPersonOccupancyFaceKind::top:
            case FirstPersonOccupancyFaceKind::bottom:
                break;
        }
        for (uint8_t direction = 0; direction < 4; ++direction)
        {
            const bool visible =
                kind == FirstPersonOccupancyFaceKind::top
                || (kind != FirstPersonOccupancyFaceKind::bottom
                    && FirstPersonFaceVisibleFromNativeView(
                        normal, direction));
            if (visible && available(direction))
                return direction;
        }
        for (uint8_t direction = 0; direction < 4; ++direction)
            if (available(direction))
                return direction;
        return std::nullopt;
    }

    // Large-scenery body images are indexed by
    // (objectDirection + viewportRotation) & 3. Recover the viewport rotation
    // that makes a chosen native image the correct projective source.
    [[nodiscard]] constexpr uint8_t FirstPersonViewportRotationForNativeView(
        uint8_t objectDirection, uint8_t nativeImageDirection)
    {
        return uint8_t((nativeImageDirection + 4u - (objectDirection & 3u)) & 3u);
    }

    [[nodiscard]] constexpr uint64_t FirstPersonSilhouettePixelKey(int32_t x, int32_t y)
    {
        return (uint64_t(uint32_t(x)) << 32) | uint32_t(y);
    }

    struct FirstPersonSilhouette
    {
        std::unordered_set<uint64_t> pixels;
        int32_t minX = std::numeric_limits<int32_t>::max();
        int32_t minY = std::numeric_limits<int32_t>::max();
        int32_t maxX = std::numeric_limits<int32_t>::min(); // exclusive
        int32_t maxY = std::numeric_limits<int32_t>::min(); // exclusive
        bool overflowed = false;

        void add(int32_t x, int32_t y)
        {
            if (!pixels.insert(FirstPersonSilhouettePixelKey(x, y)).second)
                return;
            minX = std::min(minX, x);
            minY = std::min(minY, y);
            maxX = std::max(maxX, x + 1);
            maxY = std::max(maxY, y + 1);
        }

        [[nodiscard]] bool contains(int32_t x, int32_t y) const
        {
            return pixels.contains(FirstPersonSilhouettePixelKey(x, y));
        }

        [[nodiscard]] bool empty() const
        {
            return pixels.empty();
        }

        [[nodiscard]] size_t size() const
        {
            return pixels.size();
        }
    };

    [[nodiscard]] constexpr int32_t
        FirstPersonSilhouettePixelX(uint64_t pixel)
    {
        return std::bit_cast<int32_t>(
            uint32_t(pixel >> 32));
    }

    [[nodiscard]] constexpr int32_t
        FirstPersonSilhouettePixelY(uint64_t pixel)
    {
        return std::bit_cast<int32_t>(
            uint32_t(pixel));
    }

    template<typename TPredicate>
    [[nodiscard]] float
        FirstPersonSilhouettePredicateCoverage(
            const FirstPersonSilhouette& silhouette,
            TPredicate&& predicate,
            size_t maximumPixels = 16384)
    {
        if (silhouette.empty()
            || silhouette.size() > maximumPixels)
            return 0.0f;
        size_t covered = 0;
        for (const auto pixel : silhouette.pixels)
        {
            if (predicate(
                    FirstPersonSilhouettePixelX(pixel),
                    FirstPersonSilhouettePixelY(pixel)))
                ++covered;
        }
        return float(covered)
            / float(silhouette.size());
    }

    [[nodiscard]] constexpr bool
        FirstPersonTextureReprojectionIsReliable(
            float ownership, float sourceCoverage)
    {
        return ownership >= 0.98f
            && sourceCoverage >= 0.90f;
    }

    enum class FirstPersonArtworkSampleCoverage : uint8_t
    {
        unknown,
        transparent,
        raster,
    };

    [[nodiscard]] constexpr FirstPersonArtworkSampleCoverage
        FirstPersonArtworkSampleCoverageForPoint(
            bool sourceReady, int32_t u, int32_t v,
            int32_t width, int32_t height)
    {
        if (!sourceReady || width <= 0 || height <= 0)
            return FirstPersonArtworkSampleCoverage::unknown;
        if (u < 0 || v < 0 || u >= width || v >= height)
            return FirstPersonArtworkSampleCoverage::transparent;
        return FirstPersonArtworkSampleCoverage::raster;
    }

    [[nodiscard]] constexpr float FirstPersonIsoDepth(
        uint8_t rotation, const CoordsXYZ& point)
    {
        const auto rotated = CoordsXY{ point.x, point.y }.rotate(rotation);
        return float(rotated.x + rotated.y + point.z);
    }

    struct FirstPersonDepthOwnerPixel
    {
        float depth = -std::numeric_limits<float>::infinity();
        uint32_t owner = std::numeric_limits<uint32_t>::max();
    };

    using FirstPersonDepthOwnerMap =
        std::unordered_map<uint64_t, FirstPersonDepthOwnerPixel>;

    template<typename VisitPixel>
    inline void RasteriseFirstPersonDepthTriangle(
        const std::array<ScreenCoordsXY, 3>& screen,
        const std::array<float, 3>& depth, VisitPixel&& visitPixel)
    {
        const int32_t minX = std::min({ screen[0].x, screen[1].x, screen[2].x });
        const int32_t minY = std::min({ screen[0].y, screen[1].y, screen[2].y });
        const int32_t maxX = std::max({ screen[0].x, screen[1].x, screen[2].x });
        const int32_t maxY = std::max({ screen[0].y, screen[1].y, screen[2].y });
        const int64_t spanX = int64_t(maxX) - int64_t(minX);
        const int64_t spanY = int64_t(maxY) - int64_t(minY);
        constexpr int64_t kMaxRasterPixels = 262144;
        if (spanX <= 0 || spanY <= 0
            || spanX > kMaxRasterPixels || spanY > kMaxRasterPixels
            || spanX * spanY > kMaxRasterPixels)
            return;

        const auto edge = [](const ScreenCoordsXY& a, const ScreenCoordsXY& b, float x, float y) {
            return (x - float(a.x)) * float(b.y - a.y)
                - (y - float(a.y)) * float(b.x - a.x);
        };
        const float area = edge(screen[0], screen[1], float(screen[2].x), float(screen[2].y));
        if (std::abs(area) < 1e-6f)
            return;

        for (int32_t y = minY; y < maxY; ++y)
        for (int32_t x = minX; x < maxX; ++x)
        {
            const float px = float(x) + 0.5f;
            const float py = float(y) + 0.5f;
            const float w0 = edge(screen[1], screen[2], px, py) / area;
            const float w1 = edge(screen[2], screen[0], px, py) / area;
            const float w2 = 1.0f - w0 - w1;
            constexpr float kInsideEpsilon = -1e-5f;
            if (w0 < kInsideEpsilon || w1 < kInsideEpsilon || w2 < kInsideEpsilon)
                continue;

            const float z = w0 * depth[0] + w1 * depth[1] + w2 * depth[2];
            visitPixel(x, y, z);
        }
    }

    inline void AddFirstPersonDepthTriangle(
        FirstPersonDepthOwnerMap& map, uint32_t owner,
        const std::array<ScreenCoordsXY, 3>& screen,
        const std::array<float, 3>& depth)
    {
        RasteriseFirstPersonDepthTriangle(screen, depth, [&](int32_t x, int32_t y, float z) {
            auto& pixel = map[FirstPersonSilhouettePixelKey(x, y)];
            if (z > pixel.depth + 1e-4f)
            {
                pixel.depth = z;
                pixel.owner = owner;
            }
            else if (std::abs(z - pixel.depth) <= 1e-4f && pixel.owner != owner)
            {
                pixel.owner = std::numeric_limits<uint32_t>::max();
            }
        });
    }

    struct FirstPersonDepthVisibility
    {
        FirstPersonSilhouette projected;
        FirstPersonSilhouette visible;

        [[nodiscard]] float coverage() const
        {
            return projected.empty() ? 0.0f : float(visible.size()) / float(projected.size());
        }
    };

    // Unique ownership is useful for structural correspondence, but it is not
    // visibility: adjoining triangles can share a frontmost pixel. Test depth
    // at the SAME raster sample as the complete mesh, including ambiguous
    // ties, without granting material to an occluded face or inventing texels.
    [[nodiscard]] inline FirstPersonDepthVisibility BuildFirstPersonDepthVisibility(
        const FirstPersonDepthOwnerMap& map,
        const std::array<ScreenCoordsXY, 4>& screen,
        const std::array<float, 4>& depth)
    {
        FirstPersonDepthVisibility result;
        const auto sample = [&](int32_t x, int32_t y, float z) {
            result.projected.add(x, y);
            const auto found = map.find(FirstPersonSilhouettePixelKey(x, y));
            if (found != map.end() && std::abs(z - found->second.depth) <= 1e-4f)
                result.visible.add(x, y);
        };
        RasteriseFirstPersonDepthTriangle(
            { screen[0], screen[1], screen[2] }, { depth[0], depth[1], depth[2] }, sample);
        RasteriseFirstPersonDepthTriangle(
            { screen[0], screen[2], screen[3] }, { depth[0], depth[2], depth[3] }, sample);
        return result;
    }

    [[nodiscard]] inline float FirstPersonDepthOwnerCoverage(
        const FirstPersonDepthOwnerMap& map, uint32_t owner,
        const FirstPersonSilhouette& silhouette)
    {
        if (silhouette.empty())
            return 0.0f;
        size_t owned = 0;
        for (const auto pixel : silhouette.pixels)
        {
            const auto found = map.find(pixel);
            if (found != map.end() && found->second.owner == owner)
                ++owned;
        }
        return float(owned) / float(silhouette.size());
    }


    inline void AddFirstPersonSilhouetteTriangle(
        FirstPersonSilhouette& silhouette,
        const ScreenCoordsXY& a, const ScreenCoordsXY& b, const ScreenCoordsXY& c)
    {
        const int32_t minX = std::min({ a.x, b.x, c.x });
        const int32_t minY = std::min({ a.y, b.y, c.y });
        const int32_t maxX = std::max({ a.x, b.x, c.x });
        const int32_t maxY = std::max({ a.y, b.y, c.y });
        const int64_t spanX = int64_t(maxX) - int64_t(minX);
        const int64_t spanY = int64_t(maxY) - int64_t(minY);
        if (spanX <= 0 || spanY <= 0)
            return;

        // Reconstruction is optional evidence. Malformed custom-object metadata
        // must never turn one projected face into an unbounded CPU raster job.
        constexpr int64_t kMaxRasterPixels = 262144;
        if (spanX > kMaxRasterPixels || spanY > kMaxRasterPixels
            || spanX * spanY > kMaxRasterPixels)
        {
            silhouette.overflowed = true;
            return;
        }

        const auto edge = [](const ScreenCoordsXY& p0, const ScreenCoordsXY& p1, double x, double y) {
            return (x - double(p0.x)) * double(p1.y - p0.y)
                - (y - double(p0.y)) * double(p1.x - p0.x);
        };

        for (int32_t y = minY; y < maxY; ++y)
        for (int32_t x = minX; x < maxX; ++x)
        {
            const double px = double(x) + 0.5;
            const double py = double(y) + 0.5;
            const double e0 = edge(a, b, px, py);
            const double e1 = edge(b, c, px, py);
            const double e2 = edge(c, a, px, py);
            const bool hasNegative = e0 < 0.0 || e1 < 0.0 || e2 < 0.0;
            const bool hasPositive = e0 > 0.0 || e1 > 0.0 || e2 > 0.0;
            if (!(hasNegative && hasPositive))
                silhouette.add(x, y);
        }
    }

    inline void AddFirstPersonSilhouetteQuad(
        FirstPersonSilhouette& silhouette,
        const std::array<ScreenCoordsXY, 4>& quad)
    {
        AddFirstPersonSilhouetteTriangle(silhouette, quad[0], quad[1], quad[2]);
        AddFirstPersonSilhouetteTriangle(silhouette, quad[0], quad[2], quad[3]);
    }

    struct FirstPersonSilhouetteFit
    {
        bool valid = false;
        float intersectionOverUnion = 0.0f;
        float candidateCoverage = 0.0f;
        float observedCoverage = 0.0f;
        int32_t maxEdgeError = std::numeric_limits<int32_t>::max();
    };

    [[nodiscard]] inline FirstPersonSilhouetteFit CompareFirstPersonSilhouettes(
        const FirstPersonSilhouette& observed, const FirstPersonSilhouette& candidate)
    {
        FirstPersonSilhouetteFit fit{};
        if (observed.overflowed || candidate.overflowed
            || observed.empty() || candidate.empty())
            return fit;

        size_t intersection = 0;
        const auto& smaller = observed.size() < candidate.size() ? observed : candidate;
        const auto& larger = observed.size() < candidate.size() ? candidate : observed;
        for (const auto pixel : smaller.pixels)
        {
            if (larger.pixels.contains(pixel))
                ++intersection;
        }

        const size_t unionCount = observed.size() + candidate.size() - intersection;
        if (unionCount == 0)
            return fit;

        fit.valid = true;
        fit.intersectionOverUnion = float(intersection) / float(unionCount);
        fit.candidateCoverage = float(intersection) / float(candidate.size());
        fit.observedCoverage = float(intersection) / float(observed.size());
        fit.maxEdgeError = std::max({
            std::abs(observed.minX - candidate.minX),
            std::abs(observed.minY - candidate.minY),
            std::abs(observed.maxX - candidate.maxX),
            std::abs(observed.maxY - candidate.maxY),
        });
        return fit;
    }

    struct FirstPersonMultiViewFit
    {
        bool valid = false;
        float averageIntersectionOverUnion = 0.0f;
        float minimumIntersectionOverUnion = 0.0f;
        float minimumCandidateCoverage = 0.0f;
        float minimumObservedCoverage = 0.0f;
        int32_t maximumEdgeError = 0;
        std::array<FirstPersonSilhouetteFit, 4> views{};
    };

    [[nodiscard]] inline FirstPersonMultiViewFit CompareFirstPersonMultiViewSilhouettes(
        const std::array<FirstPersonSilhouette, 4>& observed,
        const std::array<FirstPersonSilhouette, 4>& candidate)
    {
        FirstPersonMultiViewFit result{};
        result.valid = true;
        result.minimumIntersectionOverUnion = 1.0f;
        result.minimumCandidateCoverage = 1.0f;
        result.minimumObservedCoverage = 1.0f;

        for (size_t i = 0; i < result.views.size(); ++i)
        {
            auto& fit = result.views[i];
            fit = CompareFirstPersonSilhouettes(observed[i], candidate[i]);
            if (!fit.valid)
            {
                result.valid = false;
                return result;
            }
            result.averageIntersectionOverUnion += fit.intersectionOverUnion;
            result.minimumIntersectionOverUnion =
                std::min(result.minimumIntersectionOverUnion, fit.intersectionOverUnion);
            result.minimumCandidateCoverage =
                std::min(result.minimumCandidateCoverage, fit.candidateCoverage);
            result.minimumObservedCoverage =
                std::min(result.minimumObservedCoverage, fit.observedCoverage);
            result.maximumEdgeError = std::max(result.maximumEdgeError, fit.maxEdgeError);
        }
        result.averageIntersectionOverUnion /= float(result.views.size());
        return result;
    }

    [[nodiscard]] inline bool IsFirstPersonMultiViewFitReliable(
        const FirstPersonMultiViewFit& fit)
    {
        // Deliberately conservative. This is a visual reconstruction gate, not
        // permission to turn arbitrary sprites into collision geometry.
        return fit.valid
            && fit.minimumIntersectionOverUnion >= 0.52f
            && fit.minimumCandidateCoverage >= 0.68f
            && fit.minimumObservedCoverage >= 0.65f
            && fit.maximumEdgeError <= 8;
    }
} // namespace OpenRCT2::Paint
