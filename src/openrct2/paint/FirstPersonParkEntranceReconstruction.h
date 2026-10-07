/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "../world/tile_element/EntranceElement.h"

#include <cstdint>

namespace OpenRCT2::Paint
{
    // Park-entrance artwork is painted in a local frame whose X axis follows
    // the passage and whose Y axis runs across the three-tile facade.
    // These profiles are structural support envelopes, not collision boxes.
    struct FirstPersonParkEntranceBodyProfile
    {
        float halfPassageDepth = 0.0f;
        float halfFacadeWidth = 0.0f;
        float lowZ = 0.0f;
        float highZ = 0.0f;
    };

    [[nodiscard]] constexpr FirstPersonParkEntranceBodyProfile
        FirstPersonParkEntrancePublishedBodyProfile(
            ParkEntranceSequence sequence)
    {
        if (sequence == ParkEntranceSequence::centre)
        {
            // Native centre PaintStruct:
            // origin=(2,2,height+32), size=(28,28,47).
            return { 14.0f, 14.0f, 32.0f, 79.0f };
        }

        // Native side PaintStruct:
        // origin=(3,3,height), size=(26,26,79).
        return { 13.0f, 13.0f, 0.0f, 79.0f };
    }

    [[nodiscard]] constexpr FirstPersonParkEntranceBodyProfile
        FirstPersonParkEntranceReconstructionSupportProfile(
            ParkEntranceSequence sequence)
    {
        auto result =
            FirstPersonParkEntrancePublishedBodyProfile(sequence);

        // PaintStruct bounds are inset sorting bounds, not proof of a physical
        // gap between adjacent pieces. The three entrance tiles are one
        // structure, so let support reach the shared tile boundary along the
        // facade axis while retaining the much tighter native depth and height
        // envelopes. Multi-view artwork still decides what survives inside.
        result.halfFacadeWidth = 16.0f;
        return result;
    }

    // Versioned into the asset-wide reconstruction cache. Bump whenever the
    // entrance support or material policy changes.
    inline constexpr uint64_t
        kFirstPersonParkEntranceReconstructionPolicyKey =
            0x50454e5452414e02ull; // "PENTRAN" + v2
} // namespace OpenRCT2::Paint
