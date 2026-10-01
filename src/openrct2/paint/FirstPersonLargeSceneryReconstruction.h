/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonAssetReconstruction.h"

#include <cstdint>
#include <vector>

namespace OpenRCT2
{
    struct LargeSceneryEntry;
}

namespace OpenRCT2::Paint
{
    using LargeSceneryAssetFaceKind =
        FirstPersonOccupancyFaceKind;
    using LargeSceneryAssetFace =
        FirstPersonOccupancyFace;

    struct LargeSceneryAssetModel
    {
        bool attempted = false;
        bool usable = false;
        uint32_t bodyImageFirst = 0;
        uint32_t bodyImageLast = 0;
        std::vector<LargeSceneryAssetFace> faces;
    };

    [[nodiscard]] bool LargeSceneryAssetEligible(
        const LargeSceneryEntry& entry);
    [[nodiscard]] const LargeSceneryAssetModel*
        GetLargeSceneryAssetModel(
            const LargeSceneryEntry& entry);
    void ClearLargeSceneryAssetModelCache();
}
