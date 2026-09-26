/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonMath.h"
#include "FirstPersonPeriodicPassengerMotion.h"
#include "../entity/Yaw.hpp"
#include "../ride/Angles.h"
#include "../ride/CarEntry.h"
#include "../ride/Ride.h"
#include "../ride/RideData.h"
#include "../ride/Vehicle.h"
#include "../ride/VehicleGeometry.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <unordered_map>

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

    [[nodiscard]] inline float FirstPersonLerpCyclicFrame(
        float before, float after, float alpha, float frameCount)
    {
        if (!(frameCount > 0.0f))
            return 0.0f;
        constexpr float kTwoPi = 6.28318530717958647692f;
        const float step = kTwoPi / frameCount;
        float frame = FirstPersonLerpAngle(before * step, after * step, alpha) / step;
        frame = std::fmod(frame, frameCount);
        if (frame < 0.0f)
            frame += frameCount;
        return frame;
    }

    [[nodiscard]] inline FirstPersonBasis FirstPersonRotateLocalYaw(
        const FirstPersonBasis& basis, float angle)
    {
        const float c = std::cos(angle);
        const float s = std::sin(angle);
        return {
            {
                basis.forward.x * c + basis.right.x * s,
                basis.forward.y * c + basis.right.y * s,
                basis.forward.z * c + basis.right.z * s,
            },
            {
                basis.right.x * c - basis.forward.x * s,
                basis.right.y * c - basis.forward.y * s,
                basis.right.z * c - basis.forward.z * s,
            },
            basis.up,
        };
    }

    [[nodiscard]] inline FirstPersonBasis FirstPersonRotateLocalRoll(
        const FirstPersonBasis& basis, float angle)
    {
        const float c = std::cos(angle);
        const float s = std::sin(angle);
        return {
            basis.forward,
            {
                basis.right.x * c + basis.up.x * s,
                basis.right.y * c + basis.up.y * s,
                basis.right.z * c + basis.up.z * s,
            },
            {
                basis.up.x * c - basis.right.x * s,
                basis.up.y * c - basis.right.y * s,
                basis.up.z * c - basis.right.z * s,
            },
        };
    }

    struct FirstPersonSwingCalibration
    {
        bool valid = false;
        std::array<float, 4> positiveAngles{};
        float pivotLength = 0.0f;
        float angularUncertainty = 0.0f;
    };

    struct FirstPersonSpriteMarker
    {
        bool valid = false;
        float x{};
        float y{};
    };

    [[nodiscard]] inline FirstPersonSpriteMarker FirstPersonSpriteMarkerFromG1(
        const G1Element* g1)
    {
        if (g1 == nullptr || g1->width <= 0 || g1->height <= 0
            || g1->width > 512 || g1->height > 512)
            return {};
        // G1 bounds are already cropped to the stored sprite. A lower-body
        // marker is more stable for a hanging cabin than the silhouette centre.
        return {
            true,
            float(g1->xOffset) + 0.5f * float(g1->width),
            float(g1->yOffset) + 0.72f * float(g1->height),
        };
    }

    [[nodiscard]] inline FirstPersonSwingCalibration
        BuildFirstPersonSwingCalibration(const CarEntry& entry)
    {
        FirstPersonSwingCalibration result{};
        if (!entry.flags.has(CarEntryFlag::hasSwinging)
            || !entry.groupEnabled(SpriteGroupType::slopeFlat)
            || entry.flags.has(CarEntryFlag::hasVehicleAnimation))
            return result;

        std::array<std::array<FirstPersonSpriteMarker, 7>, 2> views{};
        for (size_t view = 0; view < views.size(); ++view)
        {
            const int32_t imageDirection = view == 0 ? 0 : 8;
            const int32_t base =
                entry.getSpriteOffset(SpriteGroupType::slopeFlat, imageDirection, 0);
            for (uint8_t frame = 0; frame < 7; ++frame)
            {
                views[view][frame] =
                    FirstPersonSpriteMarkerFromG1(GfxGetG1Element(base + frame));
                if (!views[view][frame].valid)
                    return result;
            }
        }

        static constexpr std::array<uint8_t, 3> kNegativeFrames{ 1, 3, 5 };
        static constexpr std::array<uint8_t, 3> kPositiveFrames{ 2, 4, 6 };
        std::array<float, 3> pairAngles{};
        std::array<float, 3> pairPivots{};
        float uncertainty = 0.0f;
        for (size_t level = 0; level < pairAngles.size(); ++level)
        {
            std::array<float, 2> viewAngles{};
            std::array<float, 2> viewPivots{};
            for (size_t view = 0; view < views.size(); ++view)
            {
                const auto& centre = views[view][0];
                const auto& negative = views[view][kNegativeFrames[level]];
                const auto& positive = views[view][kPositiveFrames[level]];
                const float midpointX = 0.5f * (negative.x + positive.x);
                const float lateral =
                    0.5f * std::abs(positive.x - negative.x);
                const float rise =
                    centre.y - 0.5f * (negative.y + positive.y);
                if (std::abs(midpointX - centre.x) > 4.0f
                    || lateral < 0.5f || rise < -1.0f)
                    return result;

                const float angle =
                    2.0f * std::atan2(std::max(rise, 0.25f), lateral);
                if (!(angle > 0.0f) || angle > 1.35f)
                    return result;
                const float pivot =
                    lateral / std::max(std::sin(angle), 0.05f);
                viewAngles[view] = angle;
                viewPivots[view] = pivot;
            }
            pairAngles[level] = 0.5f * (viewAngles[0] + viewAngles[1]);
            pairPivots[level] = 0.5f * (viewPivots[0] + viewPivots[1]);
            uncertainty = std::max(
                uncertainty, std::abs(viewAngles[0] - viewAngles[1]));
        }

        // Sprite categories are ordered by increasing physical swing.
        if (pairAngles[1] + 0.05f < pairAngles[0]
            || pairAngles[2] + 0.05f < pairAngles[1])
            return result;

        result.positiveAngles = {
            0.0f, pairAngles[0], pairAngles[1], pairAngles[2]
        };
        result.pivotLength =
            (pairPivots[0] + pairPivots[1] + pairPivots[2]) / 3.0f;
        const float pivotSpread = std::max({
            std::abs(pairPivots[0] - result.pivotLength),
            std::abs(pairPivots[1] - result.pivotLength),
            std::abs(pairPivots[2] - result.pivotLength),
        });
        result.angularUncertainty =
            uncertainty + pivotSpread / std::max(result.pivotLength, 1.0f);
        result.valid = std::isfinite(result.pivotLength)
            && result.pivotLength >= 2.0f
            && result.pivotLength <= 128.0f
            && result.angularUncertainty <= 0.45f;
        return result;
    }

    [[nodiscard]] inline const FirstPersonSwingCalibration*
        GetFirstPersonSwingCalibration(const CarEntry& entry)
    {
        struct CacheEntry
        {
            const uint8_t* sourceIdentity = nullptr;
            FirstPersonSwingCalibration calibration{};
        };
        static std::unordered_map<const CarEntry*, CacheEntry> cache;

        if (!entry.groupEnabled(SpriteGroupType::slopeFlat))
            return nullptr;
        const int32_t base =
            entry.getSpriteOffset(SpriteGroupType::slopeFlat, 0, 0);
        const auto* first = GfxGetG1Element(base);
        if (first == nullptr || first->offset == nullptr)
            return nullptr;

        auto& cached = cache[&entry];
        if (cached.sourceIdentity != first->offset)
        {
            cached.sourceIdentity = first->offset;
            cached.calibration = BuildFirstPersonSwingCalibration(entry);
        }
        return cached.calibration.valid ? &cached.calibration : nullptr;
    }

    [[nodiscard]] inline float FirstPersonSwingAngleForPosition(
        const FirstPersonSwingCalibration& calibration, float swingPosition)
    {
        const float sign = swingPosition < 0.0f ? -1.0f : 1.0f;
        const float position = std::abs(swingPosition);
        static constexpr std::array<float, 4> kRepresentativePosition{
            0.0f, 1820.0f, 5460.0f, 10000.0f
        };
        size_t upper = 1;
        while (upper + 1 < kRepresentativePosition.size()
            && position > kRepresentativePosition[upper])
            ++upper;
        const size_t lower = upper - 1;
        const float span =
            kRepresentativePosition[upper] - kRepresentativePosition[lower];
        const float alpha = span > 0.0f
            ? std::clamp(
                (position - kRepresentativePosition[lower]) / span,
                0.0f, 1.0f)
            : 0.0f;
        const float angle =
            calibration.positiveAngles[lower]
            + (calibration.positiveAngles[upper]
                - calibration.positiveAngles[lower]) * alpha;
        return sign * angle;
    }

    struct FirstPersonCarriageTransform
    {
        FirstPersonBasis basis{};
        FirstPersonVec3 originOffset{};
        float swingAngle = 0.0f;
    };

    [[nodiscard]] inline FirstPersonCarriageTransform
        BuildFirstPersonCarriageTransform(
            const Vehicle& car, FirstPersonBasis trackBasis,
            float spinAngle, float swingPosition)
    {
        FirstPersonCarriageTransform result{};
        if (car.flags.has(VehicleFlag::carIsReversed))
        {
            constexpr float kPi = 3.14159265358979323846f;
            trackBasis = FirstPersonRotateLocalYaw(trackBasis, kPi);
        }

        const auto* entry = car.Entry();
        if (entry != nullptr && entry->flags.has(CarEntryFlag::hasSpinning))
            trackBasis = FirstPersonRotateLocalYaw(trackBasis, spinAngle);

        if (entry != nullptr && entry->flags.has(CarEntryFlag::hasSwinging))
        {
            if (const auto* calibration =
                    GetFirstPersonSwingCalibration(*entry);
                calibration != nullptr)
            {
                result.swingAngle =
                    FirstPersonSwingAngleForPosition(
                        *calibration, swingPosition);
                const float lateral =
                    calibration->pivotLength * std::sin(result.swingAngle);
                const float rise =
                    calibration->pivotLength
                    * (1.0f - std::cos(result.swingAngle));
                result.originOffset = {
                    trackBasis.right.x * lateral + trackBasis.up.x * rise,
                    trackBasis.right.y * lateral + trackBasis.up.y * rise,
                    trackBasis.right.z * lateral + trackBasis.up.z * rise,
                };
                trackBasis =
                    FirstPersonRotateLocalRoll(trackBasis, result.swingAngle);
            }
        }
        result.basis = trackBasis;
        return result;
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

    [[nodiscard]] inline float FirstPersonFlatRidePrimaryFrameCount(
        const Vehicle& car)
    {
        const auto* ride = car.GetRide();
        if (ride != nullptr && ride->getRideTypeDescriptor().Name == "ferris_wheel")
            return float(kFirstPersonFerrisWheelFrameCount);
        // Existing Top Spin arm geometry is a 48-frame source. Other flat
        // rides keep the legacy interpolation period until their own native
        // animation semantics are reconstructed explicitly.
        return 48.0f;
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
        float flatPrimaryFrame = -1.0f, float flatSecondaryFrame = -1.0f,
        uint8_t pinnedSeatIndex = 0xFF)
    {
        const uint8_t seatCount = std::max<uint8_t>(car.num_seats, 1);
        const uint8_t seatIndex = pinnedSeatIndex == 0xFF
            ? FirstPersonPassengerSeatIndex(car)
            : std::min<uint8_t>(pinnedSeatIndex, uint8_t(seatCount - 1));
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
        if (ride != nullptr && ride->getRideTypeDescriptor().Name == "ferris_wheel")
        {
            const auto* rideEntry = car.GetRideEntry();
            const auto* calibration = rideEntry != nullptr
                ? GetFirstPersonFerrisWheelCalibration(*rideEntry)
                : nullptr;
            if (calibration != nullptr)
            {
                const float primaryFrame = flatPrimaryFrame >= 0.0f
                    ? flatPrimaryFrame : float(car.flatRideAnimationFrame);
                const float phase =
                    FirstPersonFerrisWheelRiderPhase(primaryFrame, seatIndex);
                const auto orbit =
                    SampleFirstPersonPeriodicOrbit(*calibration, phase);

                // For Ferris wheel's integral vehicle, native vehicle creation
                // anchors the stationary entity at sequence-0 tile centre
                // (+16,+16,+VehicleZOffset). The painter's sequence-0 wheel
                // origin is (-16,0,+7) from that tile. Express the calibrated
                // asset-space orbit relative to the vehicle so rotated ride
                // placements reuse exactly the same recovered mechanism.
                const auto seatBaseOffset =
                    FirstPersonFerrisWheelSeatBaseOffsetFromVehicle(
                        orbit, uint8_t((car.orientation >> 3) & 3u),
                        float(ride->getRideTypeDescriptor().Heights.VehicleZOffset));
                vehiclePosition.x += seatBaseOffset.x;
                vehiclePosition.y += seatBaseOffset.y;
                vehiclePosition.z += seatBaseOffset.z;

                // The fitted orbit tracks a lower-body seat marker, not the
                // changing rider-sprite centroid. Apply a separately measured
                // eye height and pair separation inside the upright cabin.
                const float lateral = (seatIndex & 1u) != 0
                    ? calibration->seatHalfSeparation
                    : -calibration->seatHalfSeparation;
                pose.localEyeOffset = {
                    0.0f, lateral, calibration->eyeHeight
                };
                pose.rideSpecificTransform = true;
            }
        }
        else if (ride != nullptr && ride->getRideTypeDescriptor().Name == "top_spin")
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
            float armFrame = flatPrimaryFrame >= 0.0f
                ? flatPrimaryFrame : float(car.flatRideAnimationFrame);
            armFrame = std::fmod(armFrame, 48.0f);
            if (armFrame < 0.0f)
                armFrame += 48.0f;
            const int32_t arm0 = int32_t(std::floor(armFrame));
            const int32_t arm1 = (arm0 + 1) % 48;
            const float armAlpha = armFrame - float(arm0);
            const float seatPosition =
                float(kSeatPosition[arm0])
                + (float(kSeatPosition[arm1]) - float(kSeatPosition[arm0])) * armAlpha;
            const float seatHeight =
                float(kSeatHeight[arm0])
                + (float(kSeatHeight[arm1]) - float(kSeatHeight[arm0])) * armAlpha;

            float seatFrame = flatSecondaryFrame >= 0.0f
                ? flatSecondaryFrame
                : float(car.flatRideSecondaryAnimationFrame & 0x0F);
            seatFrame = std::fmod(seatFrame, 16.0f);
            if (seatFrame < 0.0f)
                seatFrame += 16.0f;
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

    [[nodiscard]] inline FirstPersonBasis FirstPersonVehicleTrackBasis(
        const Vehicle& car, float yaw, float pitch, float roll)
    {
        FirstPersonCamera orientation{};
        orientation.yaw = yaw;
        const auto* ride = car.GetRide();
        const bool flatRide = ride != nullptr
            && ride->getRideTypeDescriptor().flags.has(RtdFlag::isFlatRide);
        if (!flatRide)
        {
            orientation.pitch = pitch;
            orientation.roll = roll;
        }
        return GetFirstPersonBasis(orientation);
    }

    [[nodiscard]] inline FirstPersonCarriageTransform
        FirstPersonVehicleSimulationCarriageTransform(const Vehicle& car)
    {
        const auto trackBasis = FirstPersonVehicleTrackBasis(
            car,
            FirstPersonVehicleYawRadians(car.orientation),
            FirstPersonVehiclePitchRadians(car.pitch),
            FirstPersonVehicleRollRadians(car.roll));
        return BuildFirstPersonCarriageTransform(
            car, trackBasis, SpinSpriteYawRadians(car.spin_sprite),
            float(car.SwingPosition));
    }

    [[nodiscard]] inline FirstPersonCamera
        FirstPersonVehicleSimulationOrientation(const Vehicle& car)
    {
        FirstPersonCamera result{};
        result.hasExplicitBasis = true;
        result.explicitBasis =
            FirstPersonVehicleSimulationCarriageTransform(car).basis;
        return result;
    }

    [[nodiscard]] inline FirstPersonPassengerPose FirstPersonVehicleSimulationPassengerPose(
        const Vehicle& car, uint8_t pinnedSeatIndex = 0xFF)
    {
        const auto carriage =
            FirstPersonVehicleSimulationCarriageTransform(car);
        const auto loc = car.getLocation();
        FirstPersonVec3 position{
            float(loc.x) + carriage.originOffset.x,
            float(loc.y) + carriage.originOffset.y,
            float(loc.z) + carriage.originOffset.z,
        };
        return BuildFirstPersonPassengerPose(
            car, position, carriage.basis,
            -1.0f, -1.0f, pinnedSeatIndex);
    }
} // namespace OpenRCT2::Paint

