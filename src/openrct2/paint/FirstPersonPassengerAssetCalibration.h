/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonMath.h"

#include "../drawing/Drawing.Sprite.h"
#include "../drawing/PaletteIndex.h"
#include "../ride/CarEntry.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <unordered_map>

namespace OpenRCT2::Paint
{
    struct FirstPersonRiderChannelObservation
    {
        bool valid = false;
        float x{};
        float y{};
        float width{};
        float height{};
    };

    struct FirstPersonPassengerAssetSeat
    {
        bool valid = false;
        FirstPersonVec3 localEye{};
        float reprojectionRmse{};
        float uncertainty{};
    };

    struct FirstPersonPassengerAssetCalibration
    {
        std::array<bool, 16> attempted{};
        std::array<FirstPersonPassengerAssetSeat, 16> seats{};
    };

    [[nodiscard]] constexpr bool FirstPersonRiderPixelUsesPrimaryRemap(uint8_t pixel)
    {
        return pixel >= static_cast<uint8_t>(Drawing::PaletteIndex::primaryRemap0)
            && pixel <= static_cast<uint8_t>(Drawing::PaletteIndex::primaryRemap11);
    }

    [[nodiscard]] constexpr bool FirstPersonRiderPixelUsesSecondaryRemap(uint8_t pixel)
    {
        return pixel >= static_cast<uint8_t>(Drawing::PaletteIndex::secondaryRemap0)
            && pixel <= static_cast<uint8_t>(Drawing::PaletteIndex::secondaryRemap11);
    }

    [[nodiscard]] inline bool ExtractFirstPersonRiderChannelObservation(
        const G1Element& g1, bool secondary, FirstPersonRiderChannelObservation& observation)
    {
        observation = {};
        if (g1.offset == nullptr || g1.width <= 0 || g1.height <= 0
            || g1.width > 512 || g1.height > 512
            || g1.flags.has(G1Flag::isPalette))
            return false;

        int32_t minX = g1.width;
        int32_t minY = g1.height;
        int32_t maxX = -1;
        int32_t maxY = -1;
        size_t count = 0;
        const auto acceptPixel = [&](uint8_t pixel, int32_t x, int32_t y) {
            const bool accepted = secondary
                ? FirstPersonRiderPixelUsesSecondaryRemap(pixel)
                : FirstPersonRiderPixelUsesPrimaryRemap(pixel);
            if (!accepted)
                return;
            minX = std::min(minX, x);
            minY = std::min(minY, y);
            maxX = std::max(maxX, x);
            maxY = std::max(maxY, y);
            ++count;
        };

        if (g1.flags.has(G1Flag::hasRLECompression))
        {
            for (int32_t y = 0; y < g1.height; ++y)
            {
                const uint16_t lineOffset =
                    uint16_t(g1.offset[y * 2])
                    | (uint16_t(g1.offset[y * 2 + 1]) << 8);
                const uint8_t* run = g1.offset + lineOffset;
                bool endOfLine = false;
                size_t guard = 0;
                while (!endOfLine && guard++ < 256)
                {
                    uint8_t length = *run++;
                    const int32_t x = *run++;
                    endOfLine = (length & 0x80u) != 0;
                    length &= 0x7Fu;
                    for (uint8_t n = 0; n < length; ++n)
                        acceptPixel(run[n], x + n, y);
                    run += length;
                }
                if (!endOfLine)
                    return false;
            }
        }
        else
        {
            const bool transparent = g1.flags.has(G1Flag::hasTransparency);
            for (int32_t y = 0; y < g1.height; ++y)
            for (int32_t x = 0; x < g1.width; ++x)
            {
                const uint8_t pixel =
                    g1.offset[size_t(y) * size_t(g1.width) + size_t(x)];
                if (transparent && pixel == 0)
                    continue;
                acceptPixel(pixel, x, y);
            }
        }

        if (count < 3 || maxX < minX || maxY < minY)
            return false;

        const float width = float(maxX - minX + 1);
        const float height = float(maxY - minY + 1);
        // The remap region is the shirt/upper body. Infer the eye above its top
        // from that rider-specific extent, not from the complete vehicle sprite.
        constexpr float kEyeAboveShirtFraction = 0.35f;
        observation.valid = true;
        observation.x =
            float(g1.xOffset) + 0.5f * float(minX + maxX + 1);
        observation.y =
            float(g1.yOffset + minY) - kEyeAboveShirtFraction * height;
        observation.width = width;
        observation.height = height;
        return true;
    }

    [[nodiscard]] inline std::array<float, 2> ProjectFirstPersonLocalIso(
        uint8_t direction, FirstPersonVec3 point)
    {
        switch (direction & 3u)
        {
            case 0:
                return { point.y - point.x, 0.5f * (point.x + point.y) - point.z };
            case 1:
                return { -point.x - point.y, 0.5f * (point.y - point.x) - point.z };
            case 2:
                return { point.x - point.y, -0.5f * (point.x + point.y) - point.z };
            default:
                return { point.x + point.y, 0.5f * (point.x - point.y) - point.z };
        }
    }

    [[nodiscard]] inline std::optional<FirstPersonVec3>
        RecoverFirstPersonLocalPointFromFourViews(
            const std::array<FirstPersonRiderChannelObservation, 4>& views,
            float* outRmse = nullptr)
    {
        for (const auto& view : views)
        {
            if (!view.valid || !std::isfinite(view.x) || !std::isfinite(view.y))
                return std::nullopt;
        }

        FirstPersonVec3 point{};
        point.x = (-views[0].x - views[1].x + views[2].x + views[3].x) * 0.25f;
        point.y = ( views[0].x - views[1].x - views[2].x + views[3].x) * 0.25f;
        point.z = -(views[0].y + views[1].y + views[2].y + views[3].y) * 0.25f;

        float error2 = 0.0f;
        for (uint8_t direction = 0; direction < 4; ++direction)
        {
            const auto projected = ProjectFirstPersonLocalIso(direction, point);
            const float dx = projected[0] - views[direction].x;
            const float dy = projected[1] - views[direction].y;
            error2 += dx * dx + dy * dy;
        }
        const float rmse = std::sqrt(error2 / 8.0f);
        if (outRmse != nullptr)
            *outRmse = rmse;
        return point;
    }

    [[nodiscard]] inline std::optional<FirstPersonPassengerAssetSeat>
        RecoverFirstPersonPassengerAssetSeat(
            const CarEntry& entry, uint8_t seatIndex,
            uint8_t animationFrame = 0)
    {
        if (seatIndex >= 16 || entry.numCarImages == 0
            || !entry.groupEnabled(SpriteGroupType::slopeFlat))
            return std::nullopt;
        const uint8_t row = seatIndex / 2;
        if (row >= std::min<uint8_t>(entry.numSeatingRows, 8))
            return std::nullopt;
        const bool secondary = (seatIndex & 1u) != 0;
        constexpr std::array<int32_t, 4> kImageDirections{ 0, 8, 16, 24 };

        std::array<FirstPersonRiderChannelObservation, 4> views{};
        float meanHeight = 0.0f;
        for (uint8_t direction = 0; direction < 4; ++direction)
        {
            uint32_t bodyImage = entry.getSpriteOffset(
                SpriteGroupType::slopeFlat, kImageDirections[direction], 0);
            if (entry.flags.has(CarEntryFlag::hasVehicleAnimation))
                bodyImage += animationFrame;
            uint32_t riderImage =
                bodyImage + entry.numCarImages * uint32_t(row + 1);
            if (row == 0
                && entry.flags.has(CarEntryFlag::hasRiderAnimation))
            {
                riderImage +=
                    entry.numCarImages * uint32_t(animationFrame);
            }
            const auto* g1 = GfxGetG1Element(riderImage);
            constexpr size_t kMaxRiderSpritePixels = 65536;
            if (g1 == nullptr || g1->width <= 0 || g1->height <= 0
                || size_t(g1->width) * size_t(g1->height)
                    > kMaxRiderSpritePixels
                || !ExtractFirstPersonRiderChannelObservation(
                    *g1, secondary, views[direction]))
                return std::nullopt;
            meanHeight += views[direction].height * 0.25f;
        }

        float rmse = std::numeric_limits<float>::infinity();
        const auto recovered =
            RecoverFirstPersonLocalPointFromFourViews(views, &rmse);
        if (!recovered.has_value() || rmse > 2.5f
            || std::abs(recovered->x) > 64.0f
            || std::abs(recovered->y) > 64.0f
            || recovered->z < -32.0f || recovered->z > 64.0f)
            return std::nullopt;

        FirstPersonPassengerAssetSeat result{};
        result.valid = true;
        // Native yaw 0 has forward=-X and right=-Y.
        result.localEye = {
            -recovered->x, -recovered->y, recovered->z
        };
        result.reprojectionRmse = rmse;
        result.uncertainty =
            rmse + std::max(1.0f, 0.15f * meanHeight);
        return result;
    }

    struct FirstPersonMultiDimensionArtworkCalibration
    {
        bool valid = false;
        int8_t angleSign = 1;
        FirstPersonVec3 pivotLocal{};
        float radius{};
        float uncertainty{};
    };

    [[nodiscard]] inline FirstPersonMultiDimensionArtworkCalibration
        FitFirstPersonMultiDimensionArtworkCalibration(
            const std::array<FirstPersonVec3, 16>& points,
            uint8_t frames, float sourceUncertainty = 0.0f)
    {
        FirstPersonMultiDimensionArtworkCalibration result{};
        if (frames < 4 || frames > points.size())
            return result;

        FirstPersonVec3 centre{};
        for (uint8_t frame = 0; frame < frames; ++frame)
        {
            centre.x += points[frame].x / float(frames);
            centre.y += points[frame].y / float(frames);
            centre.z += points[frame].z / float(frames);
        }

        float meanRadius = 0.0f;
        float maxLateralError = 0.0f;
        for (uint8_t frame = 0; frame < frames; ++frame)
        {
            const float df = points[frame].x - centre.x;
            const float du = points[frame].z - centre.z;
            meanRadius += std::hypot(df, du) / float(frames);
            maxLateralError = std::max(
                maxLateralError,
                std::abs(points[frame].y - centre.y));
        }
        if (!(meanRadius >= 2.0f && meanRadius <= 64.0f)
            || maxLateralError > 3.0f)
            return result;

        const float f0 = points[0].x - centre.x;
        const float u0 = points[0].z - centre.z;
        const float f1 = points[1].x - centre.x;
        const float u1 = points[1].z - centre.z;
        const float cross = f0 * u1 - u0 * f1;
        const float dot = f0 * f1 + u0 * u1;
        const float observedStep = std::atan2(cross, dot);
        constexpr float kTwoPi = 6.28318530717958647692f;
        const float expectedStep = kTwoPi / float(frames);
        if (std::abs(std::abs(observedStep) - expectedStep) > 0.4f)
            return result;
        result.angleSign = observedStep >= 0.0f ? 1 : -1;

        float error2 = 0.0f;
        float maxRadiusError = 0.0f;
        const float startAngle = std::atan2(u0, f0);
        for (uint8_t frame = 0; frame < frames; ++frame)
        {
            const float radius = std::hypot(
                points[frame].x - centre.x,
                points[frame].z - centre.z);
            maxRadiusError = std::max(
                maxRadiusError, std::abs(radius - meanRadius));
            const float phase = startAngle
                + float(result.angleSign) * expectedStep * float(frame);
            const float predictedF =
                centre.x + meanRadius * std::cos(phase);
            const float predictedU =
                centre.z + meanRadius * std::sin(phase);
            const float df = predictedF - points[frame].x;
            const float du = predictedU - points[frame].z;
            error2 += df * df + du * du;
        }
        const float orbitRmse =
            std::sqrt(error2 / float(frames * 2));
        if (orbitRmse > 3.0f
            || maxRadiusError > std::max(3.0f, meanRadius * 0.2f))
            return result;

        result.valid = true;
        result.pivotLocal = centre;
        result.radius = meanRadius;
        result.uncertainty =
            sourceUncertainty + orbitRmse + maxRadiusError;
        return result;
    }

    [[nodiscard]] inline FirstPersonMultiDimensionArtworkCalibration
        BuildFirstPersonMultiDimensionArtworkCalibration(
            const CarEntry& entry)
    {
        if (entry.animation != CarEntryAnimation::multiDimension
            || entry.animationFrames < 4 || entry.animationFrames > 16
            || entry.numSeatingRows == 0)
            return {};

        const uint8_t frames = entry.animationFrames;
        std::array<FirstPersonVec3, 16> points{};
        float sourceUncertainty = 0.0f;
        for (uint8_t frame = 0; frame < frames; ++frame)
        {
            const auto seat =
                RecoverFirstPersonPassengerAssetSeat(entry, 0, frame);
            if (!seat.has_value())
                return {};
            points[frame] = seat->localEye;
            sourceUncertainty =
                std::max(sourceUncertainty, seat->uncertainty);
        }
        return FitFirstPersonMultiDimensionArtworkCalibration(
            points, frames, sourceUncertainty);
    }

    [[nodiscard]] inline const FirstPersonMultiDimensionArtworkCalibration*
        GetFirstPersonMultiDimensionArtworkCalibration(
            const CarEntry& entry)
    {
        if (entry.animation != CarEntryAnimation::multiDimension
            || !entry.groupEnabled(SpriteGroupType::slopeFlat))
            return nullptr;

        struct CacheEntry
        {
            const uint8_t* sourceIdentity = nullptr;
            FirstPersonMultiDimensionArtworkCalibration calibration{};
        };
        static std::unordered_map<const CarEntry*, CacheEntry> cache;

        const uint32_t bodyImage =
            entry.getSpriteOffset(SpriteGroupType::slopeFlat, 0, 0);
        const uint32_t firstRiderImage = bodyImage + entry.numCarImages;
        const auto* first = GfxGetG1Element(firstRiderImage);
        if (first == nullptr || first->offset == nullptr)
            return nullptr;

        auto& cached = cache[&entry];
        if (cached.sourceIdentity != first->offset)
        {
            cached.sourceIdentity = first->offset;
            cached.calibration =
                BuildFirstPersonMultiDimensionArtworkCalibration(entry);
        }
        return cached.calibration.valid
            ? &cached.calibration : nullptr;
    }

    [[nodiscard]] inline const FirstPersonPassengerAssetSeat*
        GetFirstPersonPassengerAssetSeat(
            const CarEntry& entry, uint8_t seatIndex)
    {
        if (seatIndex >= 16 || entry.numSeatingRows == 0
            || !entry.groupEnabled(SpriteGroupType::slopeFlat))
            return nullptr;

        struct CacheEntry
        {
            const uint8_t* sourceIdentity = nullptr;
            FirstPersonPassengerAssetCalibration calibration{};
        };
        static std::unordered_map<const CarEntry*, CacheEntry> cache;

        const uint32_t bodyImage =
            entry.getSpriteOffset(SpriteGroupType::slopeFlat, 0, 0);
        const uint32_t firstRiderImage = bodyImage + entry.numCarImages;
        const auto* first = GfxGetG1Element(firstRiderImage);
        if (first == nullptr || first->offset == nullptr)
            return nullptr;

        auto& cached = cache[&entry];
        if (cached.sourceIdentity != first->offset)
        {
            cached.sourceIdentity = first->offset;
            cached.calibration = {};
        }

        if (!cached.calibration.attempted[seatIndex])
        {
            cached.calibration.attempted[seatIndex] = true;
            if (const auto calibrated =
                    RecoverFirstPersonPassengerAssetSeat(
                        entry, seatIndex, 0);
                calibrated.has_value())
            {
                cached.calibration.seats[seatIndex] = *calibrated;
            }
        }

        const auto& seat = cached.calibration.seats[seatIndex];
        return seat.valid ? &seat : nullptr;
    }
} // namespace OpenRCT2::Paint
