/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonMath.h"
#include "tile_element/Paint.Tunnel.h"

#include "../world/Location.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

namespace OpenRCT2::Paint
{
    enum class FirstPersonTunnelEdge : uint8_t
    {
        xMin = 0,
        yMax = 1,
        xMax = 2,
        yMin = 3,
    };

    struct FirstPersonTunnelPortal
    {
        CoordsXY tile{};
        FirstPersonTunnelEdge edge = FirstPersonTunnelEdge::xMin;
        TunnelType type = TunnelType::standardFlat;
        int32_t lowZ = 0;
        int32_t highZ = 0;
        int16_t boundBoxZOffset = 0;
        uint8_t boundBoxLength = 0;
    };

    [[nodiscard]] constexpr FirstPersonTunnelEdge
        FirstPersonLeftTunnelWorldEdge(uint8_t paintRotation)
    {
        return static_cast<FirstPersonTunnelEdge>(
            (2u + 4u - (paintRotation & 3u)) & 3u);
    }

    [[nodiscard]] constexpr FirstPersonTunnelEdge
        FirstPersonRightTunnelWorldEdge(uint8_t paintRotation)
    {
        return static_cast<FirstPersonTunnelEdge>(
            (1u + 4u - (paintRotation & 3u)) & 3u);
    }

    [[nodiscard]] inline FirstPersonTunnelPortal
        BuildFirstPersonTunnelPortal(
            CoordsXY tile, FirstPersonTunnelEdge edge,
            const TunnelEntry& entry)
    {
        const auto& descriptor =
            GetTunnelDescriptor(entry.type);
        const int32_t lowZ =
            int32_t(entry.height) * kCoordsZPerTinyZ;
        return {
            tile.toTileStart(),
            edge,
            entry.type,
            lowZ,
            lowZ
                + int32_t(descriptor.height)
                    * kCoordsZPerTinyZ,
            descriptor.boundBoxZOffset,
            descriptor.boundBoxLength,
        };
    }

    struct FirstPersonTunnelQuad
    {
        std::array<FirstPersonVec3, 4> corners{};
    };

    [[nodiscard]] inline std::vector<FirstPersonTunnelQuad>
        BuildFirstPersonTunnelPortalWall(
            const FirstPersonTunnelPortal& portal,
            float lowerA, float lowerB,
            float upperA, float upperB)
    {
        std::vector<FirstPersonTunnelQuad> result;
        const float x = float(portal.tile.x);
        const float y = float(portal.tile.y);
        FirstPersonVec3 a{}, b{};
        switch (portal.edge)
        {
            case FirstPersonTunnelEdge::xMin:
                a = { x, y, 0.0f };
                b = { x, y + kCoordsXYStep, 0.0f };
                break;
            case FirstPersonTunnelEdge::yMax:
                a = { x, y + kCoordsXYStep, 0.0f };
                b = { x + kCoordsXYStep,
                      y + kCoordsXYStep, 0.0f };
                break;
            case FirstPersonTunnelEdge::xMax:
                a = { x + kCoordsXYStep,
                      y + kCoordsXYStep, 0.0f };
                b = { x + kCoordsXYStep, y, 0.0f };
                std::swap(lowerA, lowerB);
                std::swap(upperA, upperB);
                break;
            case FirstPersonTunnelEdge::yMin:
                a = { x + kCoordsXYStep, y, 0.0f };
                b = { x, y, 0.0f };
                std::swap(lowerA, lowerB);
                std::swap(upperA, upperB);
                break;
        }

        const float low = float(portal.lowZ);
        const float high = float(portal.highZ);
        if (low > lowerA + 0.01f
            || low > lowerB + 0.01f)
        {
            result.push_back({ {
                { a.x, a.y, lowerA },
                { b.x, b.y, lowerB },
                { b.x, b.y,
                  std::min(low, upperB) },
                { a.x, a.y,
                  std::min(low, upperA) },
            } });
        }
        if (upperA > high + 0.01f
            || upperB > high + 0.01f)
        {
            result.push_back({ {
                { a.x, a.y,
                  std::max(high, lowerA) },
                { b.x, b.y,
                  std::max(high, lowerB) },
                { b.x, b.y, upperB },
                { a.x, a.y, upperA },
            } });
        }
        return result;
    }

    [[nodiscard]] inline std::array<FirstPersonTunnelQuad, 3>
        BuildFirstPersonTunnelSweepSegment(
            FirstPersonVec3 a, FirstPersonVec3 b,
            FirstPersonVec3 rightA, FirstPersonVec3 rightB,
            float halfWidth, float floorOffset,
            float ceilingOffset)
    {
        const auto normaliseXY =
            [](FirstPersonVec3 v) {
                const float length =
                    std::sqrt(v.x * v.x + v.y * v.y);
                if (length <= 1e-4f)
                    return FirstPersonVec3{
                        0.0f, 1.0f, 0.0f
                    };
                return FirstPersonVec3{
                    v.x / length,
                    v.y / length,
                    0.0f
                };
            };
        rightA = normaliseXY(rightA);
        rightB = normaliseXY(rightB);

        const FirstPersonVec3 al{
            a.x - rightA.x * halfWidth,
            a.y - rightA.y * halfWidth,
            a.z + floorOffset,
        };
        const FirstPersonVec3 ar{
            a.x + rightA.x * halfWidth,
            a.y + rightA.y * halfWidth,
            a.z + floorOffset,
        };
        const FirstPersonVec3 bl{
            b.x - rightB.x * halfWidth,
            b.y - rightB.y * halfWidth,
            b.z + floorOffset,
        };
        const FirstPersonVec3 br{
            b.x + rightB.x * halfWidth,
            b.y + rightB.y * halfWidth,
            b.z + floorOffset,
        };
        const FirstPersonVec3 alt{
            al.x, al.y, a.z + ceilingOffset
        };
        const FirstPersonVec3 art{
            ar.x, ar.y, a.z + ceilingOffset
        };
        const FirstPersonVec3 blt{
            bl.x, bl.y, b.z + ceilingOffset
        };
        const FirstPersonVec3 brt{
            br.x, br.y, b.z + ceilingOffset
        };
        return { {
            { { al, bl, blt, alt } },
            { { br, ar, art, brt } },
            { { alt, blt, brt, art } },
        } };
    }

    template<typename TunnelContainer>
    inline void AppendFirstPersonTunnelPortals(
        std::vector<FirstPersonTunnelPortal>& output,
        CoordsXY tile, FirstPersonTunnelEdge edge,
        const TunnelContainer& tunnels)
    {
        for (const auto& tunnel : tunnels)
        {
            const auto portal =
                BuildFirstPersonTunnelPortal(
                    tile, edge, tunnel);
            const bool duplicate = std::any_of(
                output.begin(), output.end(),
                [&](const auto& existing) {
                    return existing.tile == portal.tile
                        && existing.edge == portal.edge
                        && existing.lowZ == portal.lowZ
                        && existing.highZ == portal.highZ
                        && existing.type == portal.type;
                });
            if (!duplicate)
                output.push_back(portal);
        }
    }
} // namespace OpenRCT2::Paint
