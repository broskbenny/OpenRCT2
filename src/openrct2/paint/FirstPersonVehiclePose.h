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
        // Native vehicle orientation is a 32-step turn whose X component is
        // reflected relative to the conventional camera angle used here:
        // native forward = { -cos(theta), +sin(theta) }. Preserve all 32
        // headings (rather than quantising through the 8-way free-roam table)
        // while matching native movement: 0=-X, 8=+Y, 16=+X, 24=-Y.
        constexpr float kPi = 3.14159265358979323846f;
        constexpr float kTwoPi = 2.0f * kPi;
        const float theta = float(orientation & 0x1F) * (kTwoPi / 32.0f);
        return std::remainder(kPi - theta, kTwoPi);
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

    struct FirstPersonPassengerPose
    {
        FirstPersonVec3 position{};
        FirstPersonBasis basis{};
        FirstPersonVec3 localEyeOffset{};
        uint8_t seatIndex = 0;
        uint8_t seatingRow = 0;
        bool rideSpecificTransform = false;
    };

    [[nodiscard]] inline uint8_t FirstPersonPassengerSeatIndex(const Vehicle& car)
    {
        const uint8_t seatCount = std::min<uint8_t>(car.num_seats, 32);
        for (uint8_t i = 0; i < seatCount; ++i)
        {
            if (!car.peep[i].IsNull())
                return i;
        }
        return 0;
    }

    [[nodiscard]] inline FirstPersonVec3 FirstPersonPassengerFallbackEyeOffset(
        const Vehicle& car, uint8_t seatIndex)
    {
        const auto* entry = car.Entry();
        if (entry == nullptr)
            return { 0.0f, 0.0f, 8.0f };

        const uint8_t rows = std::max<uint8_t>(entry->numSeatingRows, 1);
        const uint8_t row = std::min<uint8_t>(seatIndex / 2, rows - 1);
        const float visualRadius = std::max(8.0f, float(entry->spriteWidth));
        const float rowSpan = std::min(16.0f, visualRadius * 0.5f);
        const float forward = rows > 1
            ? (0.5f - float(row) / float(rows - 1)) * rowSpan
            : 0.0f;
        const float lateralMagnitude = car.num_seats > 1
            ? std::clamp(visualRadius * 0.125f, 2.0f, 4.0f)
            : 0.0f;
        const float lateral = (seatIndex & 1) != 0
            ? lateralMagnitude : -lateralMagnitude;
        const float eyeHeight = std::clamp(
            0.35f * float(std::max<uint8_t>(entry->spriteHeightPositive, 1)),
            8.0f, 20.0f);
        return { forward, lateral, eyeHeight };
    }

    [[nodiscard]] inline FirstPersonBasis FirstPersonRotatePassengerPitch(
        const FirstPersonBasis& basis, float angle)
    {
        const float c = std::cos(angle);
        const float s = std::sin(angle);
        return {
            {
                basis.forward.x * c + basis.up.x * s,
                basis.forward.y * c + basis.up.y * s,
                basis.forward.z * c + basis.up.z * s,
            },
            basis.right,
            {
                basis.up.x * c - basis.forward.x * s,
                basis.up.y * c - basis.forward.y * s,
                basis.up.z * c - basis.forward.z * s,
            },
        };
    }

    [[nodiscard]] inline FirstPersonPassengerPose BuildFirstPersonPassengerPose(
        const Vehicle& car, FirstPersonVec3 vehiclePosition,
        const FirstPersonBasis& vehicleBasis,
        float flatPrimaryFrame = -1.0f, float flatSecondaryFrame = -1.0f)
    {
        const uint8_t seatIndex = FirstPersonPassengerSeatIndex(car);
        const auto* entry = car.Entry();
        const uint8_t rows = entry != nullptr
            ? std::max<uint8_t>(entry->numSeatingRows, 1) : 1;
        const uint8_t row = std::min<uint8_t>(seatIndex / 2, rows - 1);
        const auto localEye = FirstPersonPassengerFallbackEyeOffset(car, seatIndex);

        FirstPersonPassengerPose pose{};
        pose.basis = vehicleBasis;
        pose.localEyeOffset = localEye;
        pose.seatIndex = seatIndex;
        pose.seatingRow = row;

        const auto* ride = car.GetRide();
        if (ride != nullptr && ride->getRideTypeDescriptor().Name == "top_spin")
        {
            // These are the same physical seat offsets used by the native Top
            // Spin painter. Unlike most flat rides, its cabin translation and
            // independent seat-bank frame are recoverable from simulation state.
            static constexpr int16_t kSeatHeight[48] = {
                -10,-10,-9,-7,-4,-1,2,6,11,16,21,26,31,37,42,47,52,57,61,64,67,70,72,73,
                73,73,72,70,67,64,61,57,52,47,42,37,31,26,21,16,11,6,2,-1,-4,-7,-9,-10
            };
            static constexpr int8_t kSeatPosition[48] = {
                0,4,9,13,17,21,24,27,29,31,33,34,34,34,33,31,29,27,24,21,17,13,9,4,
                0,-3,-8,-12,-16,-20,-23,-26,-28,-30,-32,-33,-33,-33,-32,-30,-28,-26,-23,-20,-16,-12,-8,-3
            };
            const float armFrame = std::clamp(
                flatPrimaryFrame >= 0.0f ? flatPrimaryFrame
                                         : float(car.flatRideAnimationFrame),
                0.0f, 47.0f);
            const int32_t arm0 = int32_t(std::floor(armFrame));
            const int32_t arm1 = std::min(arm0 + 1, 47);
            const float armAlpha = armFrame - float(arm0);
            const float seatPosition =
                float(kSeatPosition[arm0])
                + (float(kSeatPosition[arm1]) - float(kSeatPosition[arm0])) * armAlpha;
            const float seatHeight =
                float(kSeatHeight[arm0])
                + (float(kSeatHeight[arm1]) - float(kSeatHeight[arm0])) * armAlpha;

            const float seatFrame = flatSecondaryFrame >= 0.0f
                ? flatSecondaryFrame
                : float(car.flatRideSecondaryAnimationFrame & 0x0F);
            const float seatAngle = seatFrame
                * (6.28318530717958647692f / 16.0f);
            pose.basis = FirstPersonRotatePassengerPitch(vehicleBasis, seatAngle);
            vehiclePosition = {
                vehiclePosition.x + vehicleBasis.forward.x * seatPosition,
                vehiclePosition.y + vehicleBasis.forward.y * seatPosition,
                vehiclePosition.z + 3.0f + seatHeight,
            };
            pose.rideSpecificTransform = true;
        }

        pose.position = FirstPersonPassengerEye(
            vehiclePosition, pose.basis, pose.localEyeOffset);
        return pose;
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

    [[nodiscard]] inline FirstPersonPassengerPose FirstPersonVehicleSimulationPassengerPose(
        const Vehicle& car)
    {
        const auto orientation = FirstPersonVehicleSimulationOrientation(car);
        const auto basis = GetFirstPersonBasis(orientation);
        const auto loc = car.getLocation();
        return BuildFirstPersonPassengerPose(
            car, { float(loc.x), float(loc.y), float(loc.z) }, basis);
    }
} // namespace OpenRCT2::Paint

