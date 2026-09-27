/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "RideData.h"
#include "RideEntry.h"
#include "Vehicle.h"

#include <cstddef>
#include <cstdint>

namespace OpenRCT2
{
    enum class VehicleVisualVariant : uint8_t
    {
        regular,
        inverted,
        cableLift,
    };

    struct VehicleVisualState
    {
        const CarEntry* carEntry = nullptr;
        int32_t zOffset = 0;
        uint8_t carEntryIndex = 0;
        VehicleVisualVariant variant =
            VehicleVisualVariant::regular;

        [[nodiscard]] explicit operator bool() const
        {
            return carEntry != nullptr;
        }
    };

    // Native vehicle painting can deliberately use a different CarEntry and
    // origin than simulation code. Keep that distinction explicit: callers
    // asking artwork/seat/body questions must resolve the visual state rather
    // than using Vehicle::Entry(), which intentionally returns the simulation
    // entry.
    [[nodiscard]] inline VehicleVisualState ResolveVehicleVisualState(
        const Vehicle& vehicle)
    {
        if (vehicle.IsCableLift())
        {
            return {
                &kCableLiftVehicle,
                0,
                0,
                VehicleVisualVariant::cableLift,
            };
        }

        const auto* rideEntry = vehicle.GetRideEntry();
        if (rideEntry == nullptr)
            return {};

        size_t index = vehicle.vehicle_type;
        VehicleVisualVariant variant =
            VehicleVisualVariant::regular;
        int32_t zOffset = 0;
        if (vehicle.flags.has(VehicleFlag::carIsInverted))
        {
            ++index;
            zOffset = 16;
            variant = VehicleVisualVariant::inverted;
        }

        const auto* carEntry =
            rideEntry->GetCar(index);
        if (carEntry == nullptr)
            return {};

        return {
            carEntry,
            zOffset,
            static_cast<uint8_t>(index),
            variant,
        };
    }
} // namespace OpenRCT2
