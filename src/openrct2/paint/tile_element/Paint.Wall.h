/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include <cstdint>

struct PaintSession;

namespace OpenRCT2
{
    struct WallElement;
}


 // Native wall artwork offsets shared with the map-derived simple-wall
 // renderer. Keep the full direction/slope/double-sided table here rather
 // than allowing a second mapping to drift from PaintWallWall.
[[nodiscard]] constexpr uint32_t GetFirstPersonWallBodyImageOffset(
    uint8_t direction, uint8_t slope, bool doubleSided, bool glass)
{
    uint32_t offset = 0;
    switch (direction & 3u)
    {
        case 0:
            offset = slope == 2 ? 3 : slope == 1 ? 5 : 1;
            break;
        case 1:
            offset = slope == 2 ? 2 : slope == 1 ? 4 : 0;
            if (doubleSided)
                offset += glass ? 12 : 6;
            break;
        case 2:
            offset = slope == 2 ? 5 : slope == 1 ? 3 : 1;
            if (doubleSided)
                offset += 6;
            break;
        case 3:
            offset = slope == 2 ? 4 : slope == 1 ? 2 : 0;
            break;
    }
    return offset;
}

void PaintWall(PaintSession& session, uint8_t direction, int32_t height, const OpenRCT2::WallElement& tileElement);
