/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonMath.h"
#include "FirstPersonPassengerAssetCalibration.h"
#include "FirstPersonSwingAssetCalibration.h"
#include "Paint.h"
#include "../entity/EntityTweener.h"
#include "../entity/Yaw.hpp"
#include "../ride/Angles.h"
#include "../ride/CarEntry.h"
#include "../ride/Ride.h"
#include "../ride/RideData.h"
#include "../ride/Vehicle.h"
#include "../ride/VehicleVisualState.h"
#include "../ride/VehicleGeometry.h"
#include "../ride/VehicleSwing.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
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

    [[nodiscard]] inline FirstPersonBasis FirstPersonRotateLocalPitch(
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

    [[nodiscard]] constexpr int16_t FirstPersonMultiDimensionAnimationFrame(
        int16_t seatRotation, int16_t animationFrames)
    {
        if (animationFrames <= 0)
            return 0;
        return ((seatRotation - 4) % animationFrames + animationFrames)
            % animationFrames;
    }

    [[nodiscard]] inline float FirstPersonMultiDimensionFrameAngle(
        int16_t animationFrame, int16_t animationFrames)
    {
        if (animationFrames <= 0)
            return 0.0f;
        constexpr float kTwoPi = 6.28318530717958647692f;
        const int16_t wrapped =
            ((animationFrame % animationFrames) + animationFrames)
            % animationFrames;
        return std::remainder(
            float(wrapped) * (kTwoPi / float(animationFrames)), kTwoPi);
    }

    [[nodiscard]] inline float FirstPersonMultiDimensionSeatAngle(
        int16_t seatRotation, int16_t animationFrame,
        int16_t animationFrames)
    {
        if (animationFrames <= 0)
            return 0.0f;
        const int16_t semanticFrame =
            FirstPersonMultiDimensionAnimationFrame(
                seatRotation, animationFrames);
        const int16_t renderedFrame =
            animationFrame >= 0 && animationFrame < animationFrames
            ? animationFrame : semanticFrame;
        return FirstPersonMultiDimensionFrameAngle(
            renderedFrame, animationFrames);
    }

    struct FirstPersonCarriageTransform
    {
        FirstPersonBasis basis{};
        // When an independently rotating seat has a known orientation but no
        // calibrated pivot, keep the passenger at the pre-seat-rotation
        // position while rotating only their orientation.
        FirstPersonBasis positionBasis{};
        FirstPersonVec3 originOffset{};
        float swingAngle = 0.0f;
        float seatAngle = 0.0f;
        bool orientationOnlySeatRotation = false;
    };

    [[nodiscard]] inline FirstPersonCarriageTransform
        BuildFirstPersonCarriageTransform(
            const Vehicle& car, FirstPersonBasis trackBasis,
            float spinAngle, float swingPosition, float seatAngle)
    {
        FirstPersonCarriageTransform result{};
        if (car.flags.has(VehicleFlag::carIsReversed))
        {
            constexpr float kPi = 3.14159265358979323846f;
            trackBasis = FirstPersonRotateLocalYaw(trackBasis, kPi);
        }

        const auto visual = ResolveVehicleVisualState(car);
        const auto* entry = visual.carEntry;
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
                const auto chassisBasis = trackBasis;
                const auto swingBasis =
                    FirstPersonRotateLocalRoll(
                        chassisBasis, result.swingAngle);
                const auto& pivot = calibration->pivotLocal;
                const FirstPersonVec3 before{
                    chassisBasis.forward.x * pivot.x
                        + chassisBasis.right.x * pivot.y
                        + chassisBasis.up.x * pivot.z,
                    chassisBasis.forward.y * pivot.x
                        + chassisBasis.right.y * pivot.y
                        + chassisBasis.up.y * pivot.z,
                    chassisBasis.forward.z * pivot.x
                        + chassisBasis.right.z * pivot.y
                        + chassisBasis.up.z * pivot.z,
                };
                const FirstPersonVec3 after{
                    swingBasis.forward.x * pivot.x
                        + swingBasis.right.x * pivot.y
                        + swingBasis.up.x * pivot.z,
                    swingBasis.forward.y * pivot.x
                        + swingBasis.right.y * pivot.y
                        + swingBasis.up.y * pivot.z,
                    swingBasis.forward.z * pivot.x
                        + swingBasis.right.z * pivot.y
                        + swingBasis.up.z * pivot.z,
                };
                result.originOffset.x += before.x - after.x;
                result.originOffset.y += before.y - after.y;
                result.originOffset.z += before.z - after.z;
                trackBasis = swingBasis;
            }
        }

        result.positionBasis = trackBasis;
        if (entry != nullptr
            && entry->animation == CarEntryAnimation::multiDimension
            && entry->animationFrames > 0)
        {
            const auto* artwork =
                GetFirstPersonMultiDimensionArtworkCalibration(*entry);
            const float physicalAngle = artwork != nullptr
                ? seatAngle * float(artwork->angleSign)
                : seatAngle;
            result.seatAngle = physicalAngle;

            const auto chassisBasis = trackBasis;
            const auto seatBasis =
                FirstPersonRotateLocalPitch(chassisBasis, physicalAngle);
            if (artwork != nullptr)
            {
                const auto& pivot = artwork->pivotLocal;
                const FirstPersonVec3 before{
                    chassisBasis.forward.x * pivot.x
                        + chassisBasis.right.x * pivot.y
                        + chassisBasis.up.x * pivot.z,
                    chassisBasis.forward.y * pivot.x
                        + chassisBasis.right.y * pivot.y
                        + chassisBasis.up.y * pivot.z,
                    chassisBasis.forward.z * pivot.x
                        + chassisBasis.right.z * pivot.y
                        + chassisBasis.up.z * pivot.z,
                };
                const FirstPersonVec3 after{
                    seatBasis.forward.x * pivot.x
                        + seatBasis.right.x * pivot.y
                        + seatBasis.up.x * pivot.z,
                    seatBasis.forward.y * pivot.x
                        + seatBasis.right.y * pivot.y
                        + seatBasis.up.y * pivot.z,
                    seatBasis.forward.z * pivot.x
                        + seatBasis.right.z * pivot.y
                        + seatBasis.up.z * pivot.z,
                };
                result.originOffset.x += before.x - after.x;
                result.originOffset.y += before.y - after.y;
                result.originOffset.z += before.z - after.z;
            }
            else
            {
                // No pivot evidence is not evidence that the pivot is the car
                // origin. Rotate orientation only; passenger translation uses
                // positionBasis below and therefore stays at the neutral point.
                result.orientationOnlySeatRotation = true;
            }
            trackBasis = seatBasis;
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
        bool supported = true;
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

    [[nodiscard]] inline FirstPersonBasis FirstPersonRotatePassengerPitch(
        const FirstPersonBasis& basis, float angle)
    {
        return FirstPersonRotateLocalPitch(basis, angle);
    }

    [[nodiscard]] inline FirstPersonPassengerPose BuildFirstPersonPassengerPose(
        const Vehicle& car, FirstPersonVec3 vehiclePosition,
        const FirstPersonBasis& vehicleBasis,
        const PassengerPaintAnchor* nativeAnchor = nullptr,
        uint8_t pinnedSeatIndex = 0xFF)
    {
        const uint8_t seatCount = std::max<uint8_t>(car.num_seats, 1);
        const uint8_t seatIndex = pinnedSeatIndex == 0xFF
            ? FirstPersonPassengerSeatIndex(car)
            : std::min<uint8_t>(pinnedSeatIndex, uint8_t(seatCount - 1));
        const auto visual = ResolveVehicleVisualState(car);
        const auto* entry = visual.carEntry;
        const uint8_t rows = entry != nullptr
            ? std::max<uint8_t>(entry->numSeatingRows, 1) : 1;
        const uint8_t row = std::min<uint8_t>(seatIndex / 2, rows - 1);
        const auto* calibrated =
            entry != nullptr
            ? GetFirstPersonPassengerAssetSeat(*entry, seatIndex)
            : nullptr;

        FirstPersonPassengerPose pose{};
        pose.basis = vehicleBasis;
        pose.seatIndex = seatIndex;
        pose.seatingRow = row;

        const bool anchorMatches =
            nativeAnchor != nullptr
            && nativeAnchor->Entity == &car
            && seatIndex < 32
            && (nativeAnchor->seatMask
                & (uint32_t{ 1 } << seatIndex)) != 0;
        if (anchorMatches)
        {
            vehiclePosition = {
                nativeAnchor->x,
                nativeAnchor->y,
                nativeAnchor->z,
            };
            if (nativeAnchor->hasLocalPitch)
            {
                pose.basis = FirstPersonRotatePassengerPitch(
                    pose.basis, nativeAnchor->localPitch);
            }
            if (nativeAnchor->hasEyeOffset)
            {
                pose.localEyeOffset = {
                    nativeAnchor->eyeForward,
                    nativeAnchor->eyeRight,
                    nativeAnchor->eyeUp,
                };
            }
            else if (calibrated != nullptr)
            {
                pose.localEyeOffset = calibrated->localEye;
            }
            else
            {
                pose.supported = false;
                return pose;
            }
            pose.rideSpecificTransform = true;
        }
        else
        {
            const auto* ride = car.GetRide();
            const bool flatRide = ride != nullptr
                && ride->getRideTypeDescriptor().flags.has(
                    RtdFlag::isFlatRide);
            // Sprite-baked moving cabins must publish their authoritative
            // passenger anchor. For ordinary vehicles, the calibrated seat
            // remains the explicit carriage-local attachment contract.
            if (flatRide || calibrated == nullptr)
            {
                pose.supported = false;
                return pose;
            }
            pose.localEyeOffset = calibrated->localEye;
        }

        pose.position = FirstPersonPassengerEye(
            vehiclePosition, pose.basis, pose.localEyeOffset);
        return pose;
    }

    [[nodiscard]] inline FirstPersonVec3
        FirstPersonPassengerEyeForCarriage(
            FirstPersonVec3 transformedOrigin,
            const FirstPersonCarriageTransform& carriage,
            FirstPersonVec3 localEyeOffset,
            bool rideSpecificTransform = false)
    {
        const auto& positionBasis =
            carriage.orientationOnlySeatRotation
                && !rideSpecificTransform
            ? carriage.positionBasis : carriage.basis;
        return FirstPersonPassengerEye(
            transformedOrigin, positionBasis, localEyeOffset);
    }

    inline void ApplyFirstPersonPassengerCarriagePositionFallback(
        FirstPersonPassengerPose& pose,
        FirstPersonVec3 transformedOrigin,
        const FirstPersonCarriageTransform& carriage)
    {
        // Only the uncalibrated orientation-only seat rotation needs a
        // position fallback. Ride-specific poses have already completed their
        // cabin/orbit translation and must not be re-anchored to the carriage.
        if (!carriage.orientationOnlySeatRotation
            || pose.rideSpecificTransform)
            return;

        pose.position = FirstPersonPassengerEyeForCarriage(
            transformedOrigin, carriage, pose.localEyeOffset, false);
    }

    [[nodiscard]] inline FirstPersonPassengerPose
        BuildFirstPersonPassengerPoseWithCarriage(
            const Vehicle& car, FirstPersonVec3 vehicleOrigin,
            const FirstPersonCarriageTransform& carriage,
            const PassengerPaintAnchor* nativeAnchor = nullptr,
            uint8_t pinnedSeatIndex = 0xFF)
    {
        FirstPersonVec3 transformedOrigin{
            vehicleOrigin.x + carriage.originOffset.x,
            vehicleOrigin.y + carriage.originOffset.y,
            vehicleOrigin.z + carriage.originOffset.z,
        };
        auto pose = BuildFirstPersonPassengerPose(
            car, transformedOrigin, carriage.basis,
            nativeAnchor, pinnedSeatIndex);
        if (!pose.supported)
            return pose;
        ApplyFirstPersonPassengerCarriagePositionFallback(
            pose, transformedOrigin, carriage);
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
        const auto visual = ResolveVehicleVisualState(car);
        const auto* entry = visual.carEntry;
        const float seatAngle =
            entry != nullptr
                && entry->animation == CarEntryAnimation::multiDimension
            ? FirstPersonMultiDimensionSeatAngle(
                  car.seat_rotation, car.animation_frame,
                  entry->animationFrames)
            : 0.0f;
        return BuildFirstPersonCarriageTransform(
            car, trackBasis, SpinSpriteYawRadians(car.spin_sprite),
            float(car.SwingPosition), seatAngle);
    }

    struct FirstPersonVehiclePresentationState
    {
        VehicleVisualState visual{};
        FirstPersonVec3 vehicleOrigin{};
        FirstPersonCarriageTransform carriage{};
    };

    [[nodiscard]] inline FirstPersonVehiclePresentationState
        BuildFirstPersonVehiclePresentationState(
            const Vehicle& car,
            const std::optional<FirstPersonTrackedVehicleVisuals>& tracked)
    {
        FirstPersonVehiclePresentationState result{};
        result.visual = ResolveVehicleVisualState(car);

        float yaw =
            FirstPersonVehicleYawRadians(car.orientation);
        const auto* ride = car.GetRide();
        const bool flatRide = ride != nullptr
            && ride->getRideTypeDescriptor().flags.has(
                RtdFlag::isFlatRide);
        float pitch = flatRide
            ? 0.0f
            : FirstPersonVehiclePitchRadians(car.pitch);
        float roll = flatRide
            ? 0.0f
            : FirstPersonVehicleRollRadians(car.roll);

        if (tracked.has_value())
        {
            yaw = FirstPersonLerpAngle(
                FirstPersonVehicleYawRadians(tracked->yawBefore),
                FirstPersonVehicleYawRadians(tracked->yawAfter),
                tracked->alpha);
            if (!flatRide)
            {
                pitch = FirstPersonLerpAngle(
                    FirstPersonVehiclePitchRadians(
                        static_cast<VehiclePitch>(
                            tracked->pitchBefore)),
                    FirstPersonVehiclePitchRadians(
                        static_cast<VehiclePitch>(
                            tracked->pitchAfter)),
                    tracked->alpha);
                roll = FirstPersonLerpAngle(
                    FirstPersonVehicleRollRadians(
                        static_cast<VehicleRoll>(
                            tracked->rollBefore)),
                    FirstPersonVehicleRollRadians(
                        static_cast<VehicleRoll>(
                            tracked->rollAfter)),
                    tracked->alpha);
            }
        }

        const auto* entry = result.visual.carEntry;
        float spinAngle = 0.0f;
        if (entry != nullptr
            && entry->flags.has(CarEntryFlag::hasSpinning))
        {
            spinAngle = tracked.has_value()
                ? FirstPersonLerpAngle(
                    SpinSpriteYawRadians(tracked->spinBefore),
                    SpinSpriteYawRadians(tracked->spinAfter),
                    tracked->alpha)
                : SpinSpriteYawRadians(car.spin_sprite);
        }

        float swingPosition = float(car.SwingPosition);
        if (tracked.has_value())
        {
            swingPosition =
                float(tracked->swingPositionBefore)
                + (float(tracked->swingPositionAfter)
                    - float(tracked->swingPositionBefore))
                    * tracked->alpha;
        }

        float seatAngle = 0.0f;
        if (entry != nullptr
            && entry->animation
                == CarEntryAnimation::multiDimension
            && entry->animationFrames > 0)
        {
            seatAngle = tracked.has_value()
                ? FirstPersonLerpAngle(
                    FirstPersonMultiDimensionSeatAngle(
                        tracked->seatRotationBefore,
                        tracked->animationFrameBefore,
                        entry->animationFrames),
                    FirstPersonMultiDimensionSeatAngle(
                        tracked->seatRotationAfter,
                        tracked->animationFrameAfter,
                        entry->animationFrames),
                    tracked->alpha)
                : FirstPersonMultiDimensionSeatAngle(
                    car.seat_rotation, car.animation_frame,
                    entry->animationFrames);
        }

        const auto trackBasis =
            FirstPersonVehicleTrackBasis(
                car, yaw, pitch, roll);
        result.carriage =
            BuildFirstPersonCarriageTransform(
                car, trackBasis, spinAngle,
                swingPosition, seatAngle);

        const auto loc = car.getLocation();
        result.vehicleOrigin = {
            float(loc.x),
            float(loc.y),
            float(loc.z + result.visual.zOffset),
        };
        return result;
    }

    [[nodiscard]] inline FirstPersonPassengerPose
        BuildFirstPersonVehiclePresentationPassengerPose(
            const Vehicle& car,
            const std::optional<FirstPersonTrackedVehicleVisuals>& tracked,
            const PassengerPaintAnchor* nativeAnchor = nullptr,
            uint8_t pinnedSeatIndex = 0xFF)
    {
        const auto state =
            BuildFirstPersonVehiclePresentationState(
                car, tracked);
        return BuildFirstPersonPassengerPoseWithCarriage(
            car, state.vehicleOrigin, state.carriage,
            nativeAnchor, pinnedSeatIndex);
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
        const Vehicle& car,
        const PassengerPaintAnchor* nativeAnchor = nullptr,
        uint8_t pinnedSeatIndex = 0xFF)
    {
        const auto carriage =
            FirstPersonVehicleSimulationCarriageTransform(car);
        const auto visual = ResolveVehicleVisualState(car);
        const auto loc = car.getLocation();
        return BuildFirstPersonPassengerPoseWithCarriage(
            car,
            {
                float(loc.x), float(loc.y),
                float(loc.z + visual.zOffset)
            },
            carriage, nativeAnchor, pinnedSeatIndex);
    }
} // namespace OpenRCT2::Paint

