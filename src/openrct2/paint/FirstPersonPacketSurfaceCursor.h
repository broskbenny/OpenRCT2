/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include <cstddef>
#include <cstdint>

namespace OpenRCT2::Paint
{
    // A tile may own thousands of native/semantic faces. A region is not
    // budgeted if a single tile copies every face without yielding.
    // The cursor is stable across frames and advances only after admission.
    struct FirstPersonPacketTileSurfaceCursor
    {
        uint8_t phase = 0;
        size_t surface = 0;

        void nextPhase()
        {
            surface = 0;
            ++phase;
        }

        void nextTile()
        {
            phase = 0;
            surface = 0;
        }
    };

    template<typename Surfaces, typename Budget, typename Consume>
    [[nodiscard]] bool ResumeFirstPersonPacketSurfaces(
        const Surfaces& surfaces, FirstPersonPacketTileSurfaceCursor& cursor,
        Budget& budget, Consume&& consume)
    {
        while (cursor.surface < surfaces.size())
        {
            if (!budget.take())
                return false;
            consume(surfaces[cursor.surface]);
            ++cursor.surface;
        }
        cursor.nextPhase();
        return true;
    }
} // namespace OpenRCT2::Paint
