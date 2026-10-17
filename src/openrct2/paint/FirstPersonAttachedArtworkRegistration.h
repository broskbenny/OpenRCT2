/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>

namespace OpenRCT2::Paint
{
    // Orthographic inverse projection of the EXACT original glyph raster
    // rectangle onto one already-reconstructed planar source face. A failed
    // certificate leaves the native sprite alone: screen offsets are not
    // evidence for synthesising an arbitrary perpendicular plane.
    struct FirstPersonAttachedArtworkPoint2
    {
        float x{};
        float y{};
    };

    struct FirstPersonAttachedArtworkPoint3
    {
        float x{};
        float y{};
        float z{};
    };

    struct FirstPersonAttachedArtworkPlacement
    {
        std::array<FirstPersonAttachedArtworkPoint3, 4> corners{};
        float centerS{};
        float centerT{};
    };

    [[nodiscard]] inline std::optional<FirstPersonAttachedArtworkPlacement>
    ReprojectFirstPersonAttachedArtwork(
        const std::array<FirstPersonAttachedArtworkPoint3, 4>& worldFace,
        const std::array<FirstPersonAttachedArtworkPoint2, 3>& sourceFace,
        float left, float top, float width, float height)
    {
        if (!(width > 0.0f) || !(height > 0.0f))
            return std::nullopt;
        const auto& p0 = worldFace[0];
        const auto& p1 = worldFace[1];
        const auto& p2 = worldFace[2];
        const auto& p3 = worldFace[3];

        // The GPU emits (0,1,2) and (0,2,3). Non-parallelogram faces
        // require piecewise inverse projection, not this affine mapping.
        const float residualX = p1.x + p3.x - p0.x - p2.x;
        const float residualY = p1.y + p3.y - p0.y - p2.y;
        const float residualZ = p1.z + p3.z - p0.z - p2.z;
        if (residualX * residualX + residualY * residualY
                + residualZ * residualZ > 0.25f)
            return std::nullopt;

        const float ux = sourceFace[1].x - sourceFace[0].x;
        const float uy = sourceFace[1].y - sourceFace[0].y;
        const float vx = sourceFace[2].x - sourceFace[0].x;
        const float vy = sourceFace[2].y - sourceFace[0].y;
        const float determinant = ux * vy - uy * vx;
        if (std::abs(determinant) < 0.5f)
            return std::nullopt;

        const auto inverse = [&](float x, float y) {
            const float dx = x - sourceFace[0].x;
            const float dy = y - sourceFace[0].y;
            return std::array<float, 2>{
                (dx * vy - dy * vx) / determinant,
                (ux * dy - uy * dx) / determinant
            };
        };
        const auto middle = inverse(
            left + 0.5f * width, top + 0.5f * height);
        if (middle[0] < -0.05f || middle[0] > 1.05f
            || middle[1] < -0.05f || middle[1] > 1.05f)
            return std::nullopt;

        const std::array<FirstPersonAttachedArtworkPoint2, 4> corners{ {
            { left, top },
            { left + width, top },
            { left + width, top + height },
            { left, top + height }
        } };
        FirstPersonAttachedArtworkPlacement result{};
        result.centerS = middle[0];
        result.centerT = middle[1];
        for (std::size_t i = 0; i < corners.size(); ++i)
        {
            const auto uv = inverse(corners[i].x, corners[i].y);
            result.corners[i] = {
                p0.x + uv[0] * (p1.x - p0.x) + uv[1] * (p3.x - p0.x),
                p0.y + uv[0] * (p1.y - p0.y) + uv[1] * (p3.y - p0.y),
                p0.z + uv[0] * (p1.z - p0.z) + uv[1] * (p3.z - p0.z),
            };
        }
        return result;
    }
}
