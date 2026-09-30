/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include <cstdint>
#include <vector>

namespace OpenRCT2::Paint
{
    struct FirstPersonPathFootprintCell
    {
        int32_t x0{};
        int32_t y0{};
        int32_t x1{};
        int32_t y1{};
    };

    [[nodiscard]] inline std::vector<FirstPersonPathFootprintCell>
        BuildFirstPersonPathFootprint(
            uint8_t edges, uint8_t corners, bool queue)
    {
        edges &= 0x0F;
        corners &= 0x0F;
        if (queue)
            corners = 0;

        std::vector<FirstPersonPathFootprintCell> result;
        result.reserve(9);
        // Native path bounds use a 3-unit margin. Connections extend the
        // 26x26 centre to one tile edge; corner bits fill the corresponding
        // 3x3 square where four ordinary paths meet.
        result.push_back({ 3, 3, 29, 29 });
        if ((edges & (1u << 0)) != 0)
            result.push_back({ 0, 3, 3, 29 });
        if ((edges & (1u << 1)) != 0)
            result.push_back({ 3, 29, 29, 32 });
        if ((edges & (1u << 2)) != 0)
            result.push_back({ 29, 3, 32, 29 });
        if ((edges & (1u << 3)) != 0)
            result.push_back({ 3, 0, 29, 3 });

        if ((corners & (1u << 0)) != 0)
            result.push_back({ 0, 29, 3, 32 });
        if ((corners & (1u << 1)) != 0)
            result.push_back({ 29, 29, 32, 32 });
        if ((corners & (1u << 2)) != 0)
            result.push_back({ 29, 0, 32, 3 });
        if ((corners & (1u << 3)) != 0)
            result.push_back({ 0, 0, 3, 3 });
        return result;
    }
}
