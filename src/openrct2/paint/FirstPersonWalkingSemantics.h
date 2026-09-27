/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include <cstdint>

namespace OpenRCT2::Paint
{
    [[nodiscard]] constexpr bool FirstPersonDoorBlocksWalking(
        uint8_t animationFrame)
    {
        // Native MapAnimation keeps frame 5 as the stable open state and
        // returns closing frame 15 to stable closed frame 0. Transitional
        // frames remain conservatively blocking until their panel aperture is
        // calibrated from the door artwork.
        return animationFrame != 5;
    }
} // namespace OpenRCT2::Paint
