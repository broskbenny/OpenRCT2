/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonMath.h"
#include "../entity/Yaw.hpp"
#include "../ride/Angles.h"
#include "../ride/CarEntry.h"
#include "../ride/Ride.h"
#include "../ride/RideData.h"
#include "../ride/Vehicle.h"
#include "../ride/VehicleGeometry.h"

#include <cmath>
#include <cstdint>

namespace OpenRCT2::Paint
{
    [[nodiscard]] inline float FirstPersonVehicleYawRadians(uint8_t orientation)
    {
        // Vehicle orientation is not a conventional +X-zero angle. Derive the
        // camera heading from the same native XY movement table used by
        // free-roaming vehicles, which encodes orientation 0 as -X, 8 as +Y,
        // 16 as +X and 24 as -Y.
        const auto movement = RideVehicle::Geometry::getFreeroamVehicleMovementData(orientation & 0x1F);
        return std::atan2(static_cast<float>(movement.y), static_cast<float>(movement.x));
    }

    [[nodiscard]] inline float FirstPersonVehiclePitchRadians(VehiclePitch pitch)
    {
        const auto value = static_cast<uint8_t>(pitch);
        if (value >= static_cast<uint8_t>(VehiclePitch::pitchCount))
            return 0.0f;

        // OpenRCT2 already owns the authoritative tangent for every physical
        // vehicle pitch, including corkscrews, helices, flyer uninversion and
        // curved lift hills. Derive the camera angle from that table instead of
        // maintaining a second, incomplete enum-to-angle switch.
        const auto direction = RideVehicle::Geometry::getPitchVector32(pitch);
        return std::atan2(static_cast<float>(direction.y), static_cast<float>(direction.x));
    }

    [[nodiscard]] inline float FirstPersonVehicleRollRadians(VehicleRoll roll)
    {
        constexpr float kPi = 3.14159265358979323846f;
        constexpr float kStep = kPi / 8.0f; // 22.5 degrees
        switch (roll)
        {
            case VehicleRoll::left22:
            case VehicleRoll::uninvertingLeft22:
                return -kStep;
            case VehicleRoll::left45:
            case VehicleRoll::uninvertingLeft45:
                return -2.0f * kStep;
            case VehicleRoll::left67:
                return -3.0f * kStep;
            case VehicleRoll::left90:
                return -4.0f * kStep;
            case VehicleRoll::left112:
                return -5.0f * kStep;
            case VehicleRoll::left135:
                return -6.0f * kStep;
            case VehicleRoll::left157:
                return -7.0f * kStep;
            case VehicleRoll::right22:
            case VehicleRoll::uninvertingRight22:
                return kStep;
            case VehicleRoll::right45:
            case VehicleRoll::uninvertingRight45:
                return 2.0f * kStep;
            case VehicleRoll::right67:
                return 3.0f * kStep;
            case VehicleRoll::right90:
                return 4.0f * kStep;
            case VehicleRoll::right112:
                return 5.0f * kStep;
            case VehicleRoll::right135:
                return 6.0f * kStep;
            case VehicleRoll::right157:
                return 7.0f * kStep;
            case VehicleRoll::unbanked:
            case VehicleRoll::uninvertingUnbanked:
            default:
                return 0.0f;
        }
    }

    [[nodiscard]] inline FirstPersonCamera FirstPersonVehicleSimulationOrientation(const Vehicle& car)
    {
        FirstPersonCamera orientation{};
        orientation.yaw = FirstPersonVehicleYawRadians(car.orientation);

        // Vehicle::pitch/roll share storage with flat-ride animation fields.
        // They are physical track orientation only for non-flat rides.
        const auto* ride = car.GetRide();
        const bool flatRide = ride != nullptr && ride->getRideTypeDescriptor().flags.has(RtdFlag::isFlatRide);
        if (!flatRide)
        {
            orientation.pitch = FirstPersonVehiclePitchRadians(car.pitch);
            orientation.roll = FirstPersonVehicleRollRadians(car.roll);
        }

        const auto* entry = car.Entry();
        if (entry != nullptr && entry->flags.has(CarEntryFlag::hasSpinning))
            orientation.yaw += SpinSpriteYawRadians(car.spin_sprite);
        return orientation;
    }
} // namespace OpenRCT2::Paint

