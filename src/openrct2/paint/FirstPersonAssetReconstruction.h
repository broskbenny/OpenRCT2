/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "../interface/ScreenCoords.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_set>

namespace OpenRCT2::Paint
{
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

    inline void AddFirstPersonSilhouetteTriangle(
        FirstPersonSilhouette& silhouette,
        const ScreenCoordsXY& a, const ScreenCoordsXY& b, const ScreenCoordsXY& c)
    {
        const int32_t minX = std::min({ a.x, b.x, c.x });
        const int32_t minY = std::min({ a.y, b.y, c.y });
        const int32_t maxX = std::max({ a.x, b.x, c.x });
        const int32_t maxY = std::max({ a.y, b.y, c.y });
        if (minX == maxX || minY == maxY)
            return;

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
        if (observed.empty() || candidate.empty())
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
