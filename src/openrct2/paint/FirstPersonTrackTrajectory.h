/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonVehiclePose.h"

#include "../ride/Vehicle.h"
#include "../ride/VehicleSubpositionData.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace OpenRCT2::Paint
{
    // The simulation's standard vehicle subposition table is the strongest
    // available geometric evidence for where a conventional tracked vehicle
    // travels through one track piece. Keep the raw reference trajectory
    // separate from any ride-specific car/body offset.
    struct FirstPersonTrackTrajectoryPoint
    {
        FirstPersonVec3 position{};
        FirstPersonBasis basis{};
        uint16_t progress{};
    };

    struct FirstPersonTrackTrajectory
    {
        std::vector<FirstPersonTrackTrajectoryPoint> points;
    };

    // Conservative generic rail cross-section. Ride families that deliberately
    // offset the vehicle reference path can override this in future rather than
    // corrupting the authoritative trajectory itself.
    struct FirstPersonTrackRailProfile
    {
        float halfGauge = 4.0f;
        float halfWidth = 0.75f;
        float halfHeight = 0.75f;
    };

    [[nodiscard]] inline const VehicleInfoList* GetFirstPersonStandardTrackVehicleInfo(
        TrackElemType type, uint8_t direction)
    {
        const size_t index = size_t(EnumValue(type)) * kNumOrthogonalDirections + (direction & 3);
        if (index >= VehicleTrackSubpositionSizeDefault)
            return nullptr;

        const auto* table = gTrackVehicleInfo[EnumValue(VehicleTrackSubposition::standard)];
        if (table == nullptr)
            return nullptr;
        const auto* list = table[index];
        if (list == nullptr || list->info == nullptr || list->size < 2)
            return nullptr;
        return list;
    }

    [[nodiscard]] inline std::optional<FirstPersonTrackTrajectory> BuildFirstPersonTrackTrajectory(
        TrackElemType type, uint8_t direction, FirstPersonVec3 trackLocation)
    {
        const auto* list = GetFirstPersonStandardTrackVehicleInfo(type, direction);
        if (list == nullptr)
            return std::nullopt;

        FirstPersonTrackTrajectory result{};
        result.points.reserve(list->size);
        for (uint16_t progress = 0; progress < list->size; ++progress)
        {
            const auto& sample = list->info[progress];
            FirstPersonCamera orientation{};
            orientation.yaw = FirstPersonVehicleYawRadians(sample.yaw);
            orientation.pitch = FirstPersonVehiclePitchRadians(sample.pitch);
            orientation.roll = FirstPersonVehicleRollRadians(sample.roll);
            result.points.push_back({
                {
                    trackLocation.x + float(sample.x),
                    trackLocation.y + float(sample.y),
                    trackLocation.z + float(sample.z),
                },
                GetFirstPersonBasis(orientation),
                progress,
            });
        }
        return result;
    }

    [[nodiscard]] inline float FirstPersonTrackTrajectoryPointDistance(
        const FirstPersonTrackTrajectoryPoint& a, const FirstPersonTrackTrajectoryPoint& b)
    {
        const float x = b.position.x - a.position.x;
        const float y = b.position.y - a.position.y;
        const float z = b.position.z - a.position.z;
        return std::sqrt(x * x + y * y + z * z);
    }

    [[nodiscard]] inline bool FirstPersonTrackTrajectorySamplesContinuous(
        const FirstPersonTrackTrajectory& trajectory, float maximumGap = 4.0f)
    {
        if (trajectory.points.size() < 2)
            return false;
        for (size_t i = 1; i < trajectory.points.size(); ++i)
        {
            if (FirstPersonTrackTrajectoryPointDistance(
                    trajectory.points[i - 1], trajectory.points[i]) > maximumGap)
                return false;
        }
        return true;
    }

    [[nodiscard]] inline float FirstPersonTrackTrajectoryEndpointGap(
        const FirstPersonTrackTrajectory& a, const FirstPersonTrackTrajectory& b)
    {
        if (a.points.empty() || b.points.empty())
            return std::numeric_limits<float>::infinity();
        return FirstPersonTrackTrajectoryPointDistance(a.points.back(), b.points.front());
    }
} // namespace OpenRCT2::Paint
