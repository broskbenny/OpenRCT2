/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonPassengerAssetCalibration.h"

#include "../ride/VehicleSwing.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>

namespace OpenRCT2::Paint
{
    struct FirstPersonSwingCalibration
    {
        bool valid = false;
        uint8_t pairCount = 0;
        std::array<float, 7> positiveAngles{};
        FirstPersonVec3 pivotLocal{};
        float radius = 0.0f;
        float angularUncertainty = 0.0f;
        FirstPersonAssetMarkerEvidence evidence =
            FirstPersonAssetMarkerEvidence::bodySilhouette;
    };

    [[nodiscard]] inline uint8_t FirstPersonSwingPairCount(
        const CarEntry& entry)
    {
        if (!entry.flags.has(CarEntryFlag::hasSwinging))
            return 0;
        if (entry.flags.hasAll(
                CarEntryFlag::useSuspendedSwing,
                CarEntryFlag::useSlideSwing))
            return 6;
        if (entry.flags.hasAny(
                CarEntryFlag::useSuspendedSwing,
                CarEntryFlag::useSlideSwing))
            return 3;
        return entry.flags.has(CarEntryFlag::useWoodenWildMouseSwing)
            ? 1 : 2;
    }

    [[nodiscard]] inline bool SolveFirstPerson3x3(
        std::array<std::array<double, 4>, 3> m,
        std::array<double, 3>& solution)
    {
        for (size_t column = 0; column < 3; ++column)
        {
            size_t pivot = column;
            for (size_t row = column + 1; row < 3; ++row)
            {
                if (std::abs(m[row][column]) > std::abs(m[pivot][column]))
                    pivot = row;
            }
            if (std::abs(m[pivot][column]) < 1e-8)
                return false;
            if (pivot != column)
                std::swap(m[pivot], m[column]);

            const double divisor = m[column][column];
            for (size_t j = column; j < 4; ++j)
                m[column][j] /= divisor;

            for (size_t row = 0; row < 3; ++row)
            {
                if (row == column)
                    continue;
                const double factor = m[row][column];
                for (size_t j = column; j < 4; ++j)
                    m[row][j] -= factor * m[column][j];
            }
        }
        for (size_t i = 0; i < 3; ++i)
            solution[i] = m[i][3];
        return true;
    }

    [[nodiscard]] inline FirstPersonSwingCalibration
        FitFirstPersonSwingCalibration(
            const std::array<FirstPersonVec3, 13>& points,
            uint8_t pairCount,
            FirstPersonAssetMarkerEvidence evidence,
            float sourceUncertainty = 0.0f)
    {
        FirstPersonSwingCalibration result{};
        if (pairCount == 0 || pairCount > 6)
            return result;
        const uint8_t frameCount = uint8_t(1 + pairCount * 2);

        double sumR = 0.0;
        double sumU = 0.0;
        double sumRR = 0.0;
        double sumUU = 0.0;
        double sumRU = 0.0;
        double sumQ = 0.0;
        double sumRQ = 0.0;
        double sumUQ = 0.0;
        float meanForward = 0.0f;
        for (uint8_t frame = 0; frame < frameCount; ++frame)
        {
            const double r = points[frame].y;
            const double u = points[frame].z;
            const double q = r * r + u * u;
            sumR += r;
            sumU += u;
            sumRR += r * r;
            sumUU += u * u;
            sumRU += r * u;
            sumQ += q;
            sumRQ += r * q;
            sumUQ += u * q;
            meanForward += points[frame].x / float(frameCount);
        }

        std::array<std::array<double, 4>, 3> normal{ {
            { sumRR, sumRU, sumR, -sumRQ },
            { sumRU, sumUU, sumU, -sumUQ },
            { sumR, sumU, double(frameCount), -sumQ },
        } };
        std::array<double, 3> circle{};
        if (!SolveFirstPerson3x3(normal, circle))
            return result;

        const float centreR = float(-0.5 * circle[0]);
        const float centreU = float(-0.5 * circle[1]);
        const float radiusSquared =
            centreR * centreR + centreU * centreU - float(circle[2]);
        if (!(radiusSquared > 0.0f))
            return result;
        const float radius = std::sqrt(radiusSquared);
        if (!(radius >= 2.0f && radius <= 128.0f))
            return result;

        float maxForwardError = 0.0f;
        float maxRadiusError = 0.0f;
        float radialError2 = 0.0f;
        for (uint8_t frame = 0; frame < frameCount; ++frame)
        {
            maxForwardError = std::max(
                maxForwardError,
                std::abs(points[frame].x - meanForward));
            const float frameRadius = std::hypot(
                points[frame].y - centreR,
                points[frame].z - centreU);
            const float error = frameRadius - radius;
            maxRadiusError = std::max(maxRadiusError, std::abs(error));
            radialError2 += error * error;
        }
        const float radialRmse =
            std::sqrt(radialError2 / float(frameCount));
        if (maxForwardError > 3.0f
            || radialRmse > std::max(1.5f, radius * 0.10f)
            || maxRadiusError > std::max(2.5f, radius * 0.18f))
            return result;

        const float neutralR = points[0].y - centreR;
        const float neutralU = points[0].z - centreU;
        if (std::hypot(neutralR, neutralU) < 1.0f)
            return result;

        result.positiveAngles[0] = 0.0f;
        float previousMagnitude = 0.0f;
        float symmetryError = 0.0f;
        for (uint8_t level = 0; level < pairCount; ++level)
        {
            const auto angleFor = [&](uint8_t frame) {
                const float r = points[frame].y - centreR;
                const float u = points[frame].z - centreU;
                const float cross = neutralR * u - neutralU * r;
                const float dot = neutralR * r + neutralU * u;
                return std::atan2(cross, dot);
            };

            const float negative =
                angleFor(uint8_t(1 + level * 2));
            const float positive =
                angleFor(uint8_t(2 + level * 2));
            if (negative * positive >= 0.0f)
                return {};
            const float magnitude =
                0.5f * (std::abs(negative) + std::abs(positive));
            if (!(magnitude > 0.01f) || magnitude > 1.55f
                || magnitude + 0.04f < previousMagnitude)
                return {};

            symmetryError = std::max(
                symmetryError,
                std::abs(std::abs(negative) - std::abs(positive)));
            result.positiveAngles[level + 1] =
                std::copysign(magnitude, positive);
            previousMagnitude = magnitude;
        }
        if (symmetryError > 0.20f)
            return {};

        result.valid = true;
        result.pairCount = pairCount;
        result.pivotLocal = { meanForward, centreR, centreU };
        result.radius = radius;
        result.angularUncertainty =
            symmetryError
            + radialRmse / std::max(radius, 1.0f)
            + maxForwardError / std::max(radius, 1.0f)
            + sourceUncertainty / std::max(radius, 1.0f);
        result.evidence = evidence;
        if (result.angularUncertainty > 0.55f)
            return {};
        return result;
    }

    [[nodiscard]] inline FirstPersonSwingCalibration
        BuildFirstPersonSwingCalibration(const CarEntry& entry)
    {
        if (!entry.flags.has(CarEntryFlag::hasSwinging)
            || !entry.groupEnabled(SpriteGroupType::slopeFlat)
            || entry.flags.has(CarEntryFlag::hasVehicleAnimation))
            return {};

        const uint8_t pairCount = FirstPersonSwingPairCount(entry);
        if (pairCount == 0 || pairCount > 6)
            return {};
        const uint8_t frameCount = uint8_t(1 + pairCount * 2);

        static constexpr std::array<FirstPersonAssetMarkerEvidence, 4>
            kEvidenceOrder{ {
                FirstPersonAssetMarkerEvidence::riderPrimary,
                FirstPersonAssetMarkerEvidence::riderSecondary,
                FirstPersonAssetMarkerEvidence::riderSilhouette,
                FirstPersonAssetMarkerEvidence::bodySilhouette,
            } };

        for (const auto evidence : kEvidenceOrder)
        {
            std::array<FirstPersonVec3, 13> points{};
            float sourceUncertainty = 0.0f;
            bool complete = true;
            for (uint8_t frame = 0; frame < frameCount; ++frame)
            {
                const auto marker =
                    RecoverFirstPersonVehicleFrameMarker(
                        entry, frame, 0, evidence);
                if (!marker.has_value())
                {
                    complete = false;
                    break;
                }
                points[frame] = marker->local;
                sourceUncertainty = std::max(
                    sourceUncertainty,
                    marker->reprojectionRmse + marker->holdoutError);
            }
            if (!complete)
                continue;

            auto calibration = FitFirstPersonSwingCalibration(
                points, pairCount, evidence, sourceUncertainty);
            if (calibration.valid)
                return calibration;
        }
        return {};
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
        const auto firstImage =
            entry.getSpriteOffset(SpriteGroupType::slopeFlat, 0, 0);
        const auto* first = GfxGetG1Element(firstImage);
        if (first == nullptr || first->offset == nullptr)
            return nullptr;

        auto& cached = cache[&entry];
        if (cached.sourceIdentity != first->offset)
        {
            cached.sourceIdentity = first->offset;
            cached.calibration =
                BuildFirstPersonSwingCalibration(entry);
        }
        return cached.calibration.valid
            ? &cached.calibration : nullptr;
    }

    [[nodiscard]] inline float FirstPersonSwingAngleForPosition(
        const FirstPersonSwingCalibration& calibration,
        float swingPosition)
    {
        const float sign = swingPosition < 0.0f ? -1.0f : 1.0f;
        const float position = std::abs(swingPosition);
        if (calibration.pairCount == 0)
            return 0.0f;

        const uint8_t last =
            std::min<uint8_t>(calibration.pairCount, 6);
        const float lastCentre =
            VehicleSwingPositiveBandCentre(last);
        if (position >= lastCentre)
            return sign * calibration.positiveAngles[last];

        uint8_t upper = 1;
        while (upper < last
            && position > VehicleSwingPositiveBandCentre(upper))
            ++upper;
        const uint8_t lower = upper - 1;
        const float lowerCentre =
            VehicleSwingPositiveBandCentre(lower);
        const float upperCentre =
            VehicleSwingPositiveBandCentre(upper);
        const float span = upperCentre - lowerCentre;
        const float alpha = span > 0.0f
            ? std::clamp(
                (position - lowerCentre) / span,
                0.0f, 1.0f)
            : 0.0f;
        const float angle =
            calibration.positiveAngles[lower]
            + (calibration.positiveAngles[upper]
                - calibration.positiveAngles[lower]) * alpha;
        return sign * angle;
    }
} // namespace OpenRCT2::Paint
