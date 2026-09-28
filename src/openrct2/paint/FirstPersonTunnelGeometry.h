/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "tile_element/Paint.Tunnel.h"

#include "../world/Location.hpp"

#include <algorithm>
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
