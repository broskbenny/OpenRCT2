/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace OpenRCT2::Ui
{
    struct FirstPersonProjectedScreenPolygon
    {
        std::array<std::array<float, 2>, 4> points{};
        uint8_t count = 0;
    };

    // Conservative convex-polygon versus screen-cell test. The 2-pixel
    // inflation includes raster pixel centres and projection roundoff; a
    // false positive only retains additional exact GPU peel passes.
    [[nodiscard]] inline bool FirstPersonProjectedPolygonMayTouchCell(
        const FirstPersonProjectedScreenPolygon& polygon,
        int32_t left, int32_t top, int32_t right, int32_t bottom)
    {
        if (polygon.count == 0)
            return false; // Triangle was fully clipped behind the near plane.
        if (polygon.count < 3 || polygon.count > polygon.points.size())
            return true; // Unknown coverage must never lower a peel bound.
        constexpr float kRasterMargin = 2.0f;
        const float x0 = float(left) - kRasterMargin;
        const float y0 = float(top) - kRasterMargin;
        const float x1 = float(right) + kRasterMargin;
        const float y1 = float(bottom) + kRasterMargin;
        for (size_t i = 0; i < polygon.count; ++i)
        {
            if (!std::isfinite(polygon.points[i][0])
                || !std::isfinite(polygon.points[i][1]))
                return true;
        }
        const auto separated = [&](float normalX, float normalY) {
            float polyMin = normalX * polygon.points[0][0]
                + normalY * polygon.points[0][1];
            float polyMax = polyMin;
            for (size_t i = 1; i < polygon.count; ++i)
            {
                const float value = normalX * polygon.points[i][0]
                    + normalY * polygon.points[i][1];
                polyMin = std::min(polyMin, value);
                polyMax = std::max(polyMax, value);
            }
            const float rectMin = normalX * (normalX >= 0 ? x0 : x1)
                + normalY * (normalY >= 0 ? y0 : y1);
            const float rectMax = normalX * (normalX >= 0 ? x1 : x0)
                + normalY * (normalY >= 0 ? y1 : y0);
            return rectMax < polyMin || rectMin > polyMax;
        };
        if (separated(1, 0) || separated(0, 1))
            return false;
        for (size_t i = 0; i < polygon.count; ++i)
        {
            const auto& a = polygon.points[i];
            const auto& b = polygon.points[(i + 1) % polygon.count];
            if (separated(b[1] - a[1], a[0] - b[0]))
                return false;
        }
        return true;
    }

    // Upper-bound the simultaneous number of transparent source surfaces per
    // pixel using small screen cells and the actual near-clipped triangles.
    // A source is counted once per cell, even if both triangles intersect it.
    // Every real raster fragment lies in at least one counted cell. This
    // count may be larger than the true overlap but NEVER smaller. Callers
    // can safely min() it with the existing rectangle-only upper bound.
    template<typename CandidateRange>
    [[nodiscard]] inline size_t FirstPersonMaximumPolygonCellOverlap(
        const CandidateRange& candidates,
        int32_t left, int32_t top, int32_t right, int32_t bottom,
        int32_t cellSize = 16)
    {
        if (cellSize <= 0 || right <= left || bottom <= top)
            return 0;
        size_t maximum = 0;
        for (int32_t y = top; y < bottom; y += cellSize)
        for (int32_t x = left; x < right; x += cellSize)
        {
            const int32_t cellRight = std::min(x + cellSize, right);
            const int32_t cellBottom = std::min(y + cellSize, bottom);
            size_t count = 0;
            for (const auto& candidate : candidates)
            {
                if (candidate.x1 <= x || candidate.x0 >= cellRight
                    || candidate.y1 <= y || candidate.y0 >= cellBottom)
                    continue;
                if (FirstPersonProjectedPolygonMayTouchCell(
                        candidate.projectedPolygons[0], x, y, cellRight, cellBottom)
                    || FirstPersonProjectedPolygonMayTouchCell(
                        candidate.projectedPolygons[1], x, y, cellRight, cellBottom))
                    ++count;
            }
            maximum = std::max(maximum, count);
        }
        return maximum;
    }
} // namespace OpenRCT2::Ui
