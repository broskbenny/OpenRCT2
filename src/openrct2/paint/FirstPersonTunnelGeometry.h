/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonMath.h"
#include "FirstPersonTrackTrajectory.h"
#include "tile_element/Paint.Tunnel.h"

#include "../world/Location.hpp"
#include "../world/Map.h"
#include "../world/tile_element/Slope.h"
#include "../world/tile_element/SurfaceElement.h"

#include <algorithm>
#include <array>
#include <cmath>
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
        uint8_t paintRotation = 0;
        bool nativeLeft = false;
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
            const TunnelEntry& entry,
            uint8_t paintRotation = 0,
            bool nativeLeft = false)
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
            uint8_t(paintRotation & 3u),
            nativeLeft,
        };
    }

    struct FirstPersonTunnelQuad
    {
        std::array<FirstPersonVec3, 4> corners{};
    };

    [[nodiscard]] constexpr FirstPersonTunnelQuad
        MakeFirstPersonTunnelQuad(
            FirstPersonVec3 a, FirstPersonVec3 b,
            FirstPersonVec3 c, FirstPersonVec3 d)
    {
        FirstPersonTunnelQuad result{};
        result.corners = { a, b, c, d };
        return result;
    }

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
            result.push_back(MakeFirstPersonTunnelQuad(
                { a.x, a.y, lowerA },
                { b.x, b.y, lowerB },
                { b.x, b.y, std::min(low, upperB) },
                { a.x, a.y, std::min(low, upperA) }));
        }
        if (upperA > high + 0.01f
            || upperB > high + 0.01f)
        {
            result.push_back(MakeFirstPersonTunnelQuad(
                { a.x, a.y, std::max(high, lowerA) },
                { b.x, b.y, std::max(high, lowerB) },
                { b.x, b.y, upperB },
                { a.x, a.y, upperA }));
        }
        return result;
    }

    [[nodiscard]] inline std::vector<FirstPersonTunnelQuad>
        BuildFirstPersonTunnelPortalTerrainWall(
            FirstPersonTunnelPortal portal)
    {
        const auto* surface =
            MapGetSurfaceElementAt(portal.tile);
        if (surface == nullptr)
            return {};

        const auto self =
            GetSlopeCornerHeights(
                surface->getBaseZ(),
                surface->getSlope());
        CoordsXY neighbourTile = portal.tile;
        switch (portal.edge)
        {
            case FirstPersonTunnelEdge::xMin:
                neighbourTile.x -= kCoordsXYStep;
                break;
            case FirstPersonTunnelEdge::yMax:
                neighbourTile.y += kCoordsXYStep;
                break;
            case FirstPersonTunnelEdge::xMax:
                neighbourTile.x += kCoordsXYStep;
                break;
            case FirstPersonTunnelEdge::yMin:
                neighbourTile.y -= kCoordsXYStep;
                break;
        }

        float lowerA = 0.0f;
        float lowerB = 0.0f;
        float upperA = 0.0f;
        float upperB = 0.0f;
        const auto* neighbour =
            MapIsLocationValid(neighbourTile)
            ? MapGetSurfaceElementAt(neighbourTile)
            : nullptr;
        const auto neighbourHeights =
            neighbour != nullptr
            ? GetSlopeCornerHeights(
                neighbour->getBaseZ(),
                neighbour->getSlope())
            : decltype(self){};

        switch (portal.edge)
        {
            case FirstPersonTunnelEdge::xMin:
                upperA = float(self.south);
                upperB = float(self.west);
                if (neighbour != nullptr)
                {
                    lowerA = float(neighbourHeights.east);
                    lowerB = float(neighbourHeights.north);
                }
                break;
            case FirstPersonTunnelEdge::yMax:
                upperA = float(self.west);
                upperB = float(self.north);
                if (neighbour != nullptr)
                {
                    lowerA = float(neighbourHeights.south);
                    lowerB = float(neighbourHeights.east);
                }
                break;
            case FirstPersonTunnelEdge::xMax:
                upperA = float(self.north);
                upperB = float(self.east);
                if (neighbour != nullptr)
                {
                    lowerA = float(neighbourHeights.west);
                    lowerB = float(neighbourHeights.south);
                }
                break;
            case FirstPersonTunnelEdge::yMin:
                upperA = float(self.east);
                upperB = float(self.south);
                if (neighbour != nullptr)
                {
                    lowerA = float(neighbourHeights.north);
                    lowerB = float(neighbourHeights.west);
                }
                break;
        }

        const auto cornerHeight =
            [](const auto& heights, uint8_t corner) {
                switch (corner & 3u)
                {
                    case 0: return float(heights.south);
                    case 1: return float(heights.east);
                    case 2: return float(heights.north);
                    default: return float(heights.west);
                }
            };
        const uint8_t selfCorner =
            uint8_t(
                (portal.nativeLeft ? 1u : 3u)
                + portal.paintRotation)
            & 3u;
        const float selfClearance =
            cornerHeight(self, selfCorner);
        const float neighbourClearance =
            neighbour != nullptr
            ? cornerHeight(
                neighbourHeights,
                portal.paintRotation)
            : selfClearance;

        const auto& requested =
            GetTunnelDescriptor(portal.type);
        const float clearance =
            std::min(selfClearance, neighbourClearance);
        if (float(portal.lowZ)
                + float(requested.height
                    * kCoordsZPerTinyZ)
            > clearance)
        {
            const auto& fallback =
                GetTunnelDescriptor(
                    requested.lowClearanceAlternative);
            portal.type =
                requested.lowClearanceAlternative;
            portal.highZ = portal.lowZ
                + int32_t(fallback.height)
                    * kCoordsZPerTinyZ;
            portal.boundBoxZOffset =
                fallback.boundBoxZOffset;
            portal.boundBoxLength =
                fallback.boundBoxLength;
        }

        lowerA = std::min(lowerA, upperA);
        lowerB = std::min(lowerB, upperB);
        return BuildFirstPersonTunnelPortalWall(
            portal, lowerA, lowerB,
            upperA, upperB);
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
        return {
            MakeFirstPersonTunnelQuad(al, bl, blt, alt),
            MakeFirstPersonTunnelQuad(br, ar, art, brt),
            MakeFirstPersonTunnelQuad(alt, blt, brt, art),
        };
    }

    struct FirstPersonTunnelRouteQuad
    {
        FirstPersonTunnelQuad quad{};
        FirstPersonVec3 midpoint{};
    };

    [[nodiscard]] inline bool
        FirstPersonTunnelClearanceUnderTerrain(
            FirstPersonVec3 point, float ceilingOffset)
    {
        const CoordsXY position{
            int32_t(std::lround(point.x)),
            int32_t(std::lround(point.y)),
        };
        if (!MapIsLocationValid(position))
            return false;
        return float(TileElementHeight(position))
            > point.z + ceilingOffset + 0.5f;
    }

    [[nodiscard]] inline std::vector<FirstPersonTunnelRouteQuad>
        BuildFirstPersonTrackTunnelRoute(
            const FirstPersonTrackTrajectory& trajectory,
            float halfWidth, float floorOffset,
            float ceilingOffset)
    {
        std::vector<FirstPersonTunnelRouteQuad> result;
        if (trajectory.points.size() < 2
            || !(halfWidth > 0.0f)
            || !(ceilingOffset > floorOffset))
            return result;

        result.reserve(
            (trajectory.points.size() - 1) * 3);
        for (size_t i = 1;
             i < trajectory.points.size(); ++i)
        {
            const auto& a = trajectory.points[i - 1];
            const auto& b = trajectory.points[i];
            const FirstPersonVec3 midpoint{
                (a.position.x + b.position.x) * 0.5f,
                (a.position.y + b.position.y) * 0.5f,
                (a.position.z + b.position.z) * 0.5f,
            };
            if (!FirstPersonTunnelClearanceUnderTerrain(
                    midpoint, ceilingOffset))
                continue;

            const auto quads =
                BuildFirstPersonTunnelSweepSegment(
                    a.position, b.position,
                    a.basis.right, b.basis.right,
                    halfWidth, floorOffset,
                    ceilingOffset);
            for (const auto& quad : quads)
                result.push_back({ quad, midpoint });
        }
        return result;
    }

    template<typename TunnelContainer>
    inline void AppendFirstPersonTunnelPortals(
        std::vector<FirstPersonTunnelPortal>& output,
        CoordsXY tile, FirstPersonTunnelEdge edge,
        const TunnelContainer& tunnels,
        uint8_t paintRotation = 0,
        bool nativeLeft = false)
    {
        for (const auto& tunnel : tunnels)
        {
            const auto portal =
                BuildFirstPersonTunnelPortal(
                    tile, edge, tunnel,
                    paintRotation, nativeLeft);
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
