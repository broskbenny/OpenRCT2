/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>

namespace OpenRCT2::Ui
{
    // The next region must coexist with the visible last-known-good VBOs.
    // Desired geometry can be SMALLER than existing residents after an edit.
    // A budget based solely on desired sizes can then permanently reject its
    // own replacement, because the existing visible VBO cannot be evicted.
    [[nodiscard]] inline size_t FirstPersonVisibleRegionGpuBudget(
        size_t baselineBytes, size_t desiredVisibleBytes,
        size_t currentlyResidentVisibleBytes, size_t largestReplacementBytes)
    {
        const size_t overlap = std::max(
            desiredVisibleBytes, currentlyResidentVisibleBytes);
        if (largestReplacementBytes
            > std::numeric_limits<size_t>::max() - overlap)
            return std::numeric_limits<size_t>::max();
        return std::max(
            baselineBytes, overlap + largestReplacementBytes);
    }
}
