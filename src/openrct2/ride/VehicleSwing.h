/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include <array>
#include <cstdint>

namespace OpenRCT2
{
    inline constexpr std::array<int16_t, 6> kVehicleSwingSpriteThresholds{
        910, 2730, 4550, 6370, 8190, 10010
    };

    [[nodiscard]] constexpr uint8_t VehicleSwingLevelForPosition(
        int32_t swingPosition)
    {
        const int32_t magnitude =
            swingPosition < 0 ? -swingPosition : swingPosition;
        uint8_t level = 0;
        for (const auto threshold : kVehicleSwingSpriteThresholds)
        {
            if (magnitude <= threshold)
                break;
            ++level;
        }
        return level;
    }

    [[nodiscard]] constexpr uint8_t VehicleSwingSpriteForPosition(
        int32_t swingPosition)
    {
        const uint8_t level =
            VehicleSwingLevelForPosition(swingPosition);
        if (level == 0)
            return 0;
        return uint8_t(level * 2u - (swingPosition < 0 ? 1u : 0u));
    }

    [[nodiscard]] constexpr float VehicleSwingPositiveBandCentre(
        uint8_t level)
    {
        if (level == 0)
            return 0.0f;
        if (level < kVehicleSwingSpriteThresholds.size())
        {
            return 0.5f * float(
                kVehicleSwingSpriteThresholds[level - 1]
                + kVehicleSwingSpriteThresholds[level]);
        }

        const auto last = kVehicleSwingSpriteThresholds.back();
        const auto previous =
            kVehicleSwingSpriteThresholds[
                kVehicleSwingSpriteThresholds.size() - 2];
        return float(last) + 0.5f * float(last - previous);
    }
} // namespace OpenRCT2
