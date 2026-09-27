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
#include <vector>

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
        // This is deliberately only an upper-body landmark. Do not turn the
        // shirt extent into an eye with a shared anatomical magic constant:
        // passenger calibration below independently extracts the opaque head.
        observation.valid = true;
        observation.x =
            float(g1.xOffset) + 0.5f * float(minX + maxX + 1);
        observation.y =
            float(g1.yOffset) + 0.5f * float(minY + maxY + 1);
        observation.width = width;
        observation.height = height;
        return true;
    }

    enum class FirstPersonAssetMarkerEvidence : uint8_t
    {
        riderPrimary,
        riderSecondary,
        riderSilhouette,
        bodySilhouette,
    };

    struct FirstPersonRecoveredAssetMarker
    {
        bool valid = false;
        FirstPersonVec3 local{};
        float reprojectionRmse{};
        float holdoutError{};
        FirstPersonAssetMarkerEvidence evidence =
            FirstPersonAssetMarkerEvidence::bodySilhouette;
    };

    template<typename TAccept>
    [[nodiscard]] inline bool ExtractFirstPersonPixelCentroidObservation(
        const G1Element& g1, TAccept&& accept,
        FirstPersonRiderChannelObservation& observation)
    {
        observation = {};
        if (g1.offset == nullptr || g1.width <= 0 || g1.height <= 0
            || g1.width > 512 || g1.height > 512
            || g1.flags.has(G1Flag::isPalette))
            return false;

        double sumX = 0.0;
        double sumY = 0.0;
        size_t count = 0;
        int32_t minX = g1.width;
        int32_t minY = g1.height;
        int32_t maxX = -1;
        int32_t maxY = -1;
        const auto visit = [&](uint8_t pixel, int32_t x, int32_t y) {
            if (!accept(pixel))
                return;
            sumX += double(x);
            sumY += double(y);
            ++count;
            minX = std::min(minX, x);
            minY = std::min(minY, y);
            maxX = std::max(maxX, x);
            maxY = std::max(maxY, y);
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
                    {
                        if (run[n] != 0)
                            visit(run[n], x + n, y);
                    }
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
                visit(pixel, x, y);
            }
        }

        if (count < 3 || maxX < minX || maxY < minY)
            return false;
        observation.valid = true;
        observation.x = float(g1.xOffset) + float(sumX / double(count));
        observation.y = float(g1.yOffset) + float(sumY / double(count));
        observation.width = float(maxX - minX + 1);
        observation.height = float(maxY - minY + 1);
        return true;
    }

    [[nodiscard]] inline bool ExtractFirstPersonRiderHeadObservation(
        const G1Element& g1, bool secondary,
        FirstPersonRiderChannelObservation& observation)
    {
        observation = {};
        FirstPersonRiderChannelObservation shirt{};
        if (!ExtractFirstPersonRiderChannelObservation(
                g1, secondary, shirt))
            return false;

        const int32_t shirtMinX = int32_t(std::floor(
            shirt.x - float(g1.xOffset) - shirt.width * 0.5f));
        const int32_t shirtMaxX = int32_t(std::ceil(
            shirt.x - float(g1.xOffset) + shirt.width * 0.5f)) - 1;
        const int32_t shirtMinY = int32_t(std::floor(
            shirt.y - float(g1.yOffset) - shirt.height * 0.5f));
        const int32_t searchPadX =
            std::max(3, int32_t(std::ceil(shirt.width * 0.75f)));
        const int32_t searchHeight =
            std::max(5, int32_t(std::ceil(shirt.height * 1.75f)));
        const int32_t minX =
            std::max(0, shirtMinX - searchPadX);
        const int32_t maxX =
            std::min(g1.width - 1, shirtMaxX + searchPadX);
        const int32_t minY =
            std::max(0, shirtMinY - searchHeight);
        const int32_t maxY =
            std::min(g1.height - 1, shirtMinY + 1);
        if (minX > maxX || minY > maxY)
            return false;

        const size_t pixelCount =
            size_t(g1.width) * size_t(g1.height);
        std::vector<uint8_t> opaque(pixelCount, 0);
        std::vector<uint8_t> sourcePixels(pixelCount, 0);
        std::vector<uint8_t> selectedShirt(pixelCount, 0);
        const auto acceptPixel = [&](uint8_t pixel, int32_t x, int32_t y) {
            if (x < 0 || y < 0 || x >= g1.width || y >= g1.height
                || pixel == 0)
                return;
            const size_t index =
                size_t(y) * size_t(g1.width) + size_t(x);
            opaque[index] = 1;
            sourcePixels[index] = pixel;
            const bool selected = secondary
                ? FirstPersonRiderPixelUsesSecondaryRemap(pixel)
                : FirstPersonRiderPixelUsesPrimaryRemap(pixel);
            if (selected)
                selectedShirt[index] = 1;
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
            const bool transparent =
                g1.flags.has(G1Flag::hasTransparency);
            for (int32_t y = 0; y < g1.height; ++y)
            for (int32_t x = 0; x < g1.width; ++x)
            {
                const uint8_t pixel = g1.offset[
                    size_t(y) * size_t(g1.width) + size_t(x)];
                if (!transparent || pixel != 0)
                    acceptPixel(pixel, x, y);
            }
        }

        // Remove both riders' shirt/remap pixels from head candidates. The
        // selected shirt is used only to define where its head must attach.
        std::vector<uint8_t> candidate(pixelCount, 0);
        for (int32_t y = minY; y <= maxY; ++y)
        for (int32_t x = minX; x <= maxX; ++x)
        {
            const size_t index =
                size_t(y) * size_t(g1.width) + size_t(x);
            if (!opaque[index])
                continue;
            const uint8_t pixel = sourcePixels[index];
            if (FirstPersonRiderPixelUsesPrimaryRemap(pixel)
                || FirstPersonRiderPixelUsesSecondaryRemap(pixel))
                continue;
            candidate[index] = selectedShirt[index] ? 0 : 1;
        }

        // For RLE we do not retain source palette values above. Remove selected
        // shirt cells directly and reject components whose vertical extent lies
        // predominantly below the shirt top; the search window is otherwise
        // intentionally tight around this rider.
        std::vector<uint8_t> visited(pixelCount, 0);
        struct Component
        {
            size_t count = 0;
            double sumX = 0.0;
            double sumY = 0.0;
            int32_t minX = 0;
            int32_t maxX = -1;
            int32_t minY = 0;
            int32_t maxY = -1;
        };
        std::optional<Component> best;
        float bestScore = -std::numeric_limits<float>::infinity();
        const float shirtCentreX =
            shirt.x - float(g1.xOffset);

        for (int32_t seedY = minY; seedY <= maxY; ++seedY)
        for (int32_t seedX = minX; seedX <= maxX; ++seedX)
        {
            const size_t seedIndex =
                size_t(seedY) * size_t(g1.width) + size_t(seedX);
            if (!candidate[seedIndex] || visited[seedIndex])
                continue;

            Component component{};
            component.minX = component.maxX = seedX;
            component.minY = component.maxY = seedY;
            std::vector<CoordsXY> stack;
            stack.push_back({ seedX, seedY });
            visited[seedIndex] = 1;
            while (!stack.empty())
            {
                const auto point = stack.back();
                stack.pop_back();
                ++component.count;
                component.sumX += point.x;
                component.sumY += point.y;
                component.minX =
                    std::min(component.minX, point.x);
                component.maxX =
                    std::max(component.maxX, point.x);
                component.minY =
                    std::min(component.minY, point.y);
                component.maxY =
                    std::max(component.maxY, point.y);

                for (int32_t dy = -1; dy <= 1; ++dy)
                for (int32_t dx = -1; dx <= 1; ++dx)
                {
                    if (dx == 0 && dy == 0)
                        continue;
                    const int32_t nx = point.x + dx;
                    const int32_t ny = point.y + dy;
                    if (nx < minX || nx > maxX
                        || ny < minY || ny > maxY)
                        continue;
                    const size_t ni =
                        size_t(ny) * size_t(g1.width)
                        + size_t(nx);
                    if (!candidate[ni] || visited[ni])
                        continue;
                    visited[ni] = 1;
                    stack.push_back({ nx, ny });
                }
            }

            if (component.count < 2
                || component.maxY > shirtMinY + 1)
                continue;
            const float centreX =
                float(component.sumX / double(component.count));
            const float horizontalDistance =
                std::abs(centreX - shirtCentreX);
            const float verticalGap =
                float(std::max(0, shirtMinY - component.maxY - 1));
            const float overlap =
                float(std::max(
                    0,
                    std::min(component.maxX, shirtMaxX)
                        - std::max(component.minX, shirtMinX) + 1));
            const float score =
                3.0f * overlap
                - 1.5f * horizontalDistance
                - 2.0f * verticalGap
                + 0.2f * float(component.count);
            if (!best.has_value() || score > bestScore)
            {
                best = component;
                bestScore = score;
            }
        }

        if (!best.has_value())
            return false;
        const float headWidth =
            float(best->maxX - best->minX + 1);
        const float headHeight =
            float(best->maxY - best->minY + 1);
        if (headWidth < 1.0f || headHeight < 1.0f
            || headWidth > shirt.width * 2.0f
            || headHeight > shirt.height * 2.5f)
            return false;

        observation.valid = true;
        observation.x =
            float(g1.xOffset)
            + float(best->sumX / double(best->count));
        observation.y =
            float(g1.yOffset)
            + float(best->sumY / double(best->count));
        observation.width = headWidth;
        observation.height = headHeight;
        return true;
    }

    [[nodiscard]] inline bool ExtractFirstPersonAssetMarkerObservation(
        const G1Element& g1, FirstPersonAssetMarkerEvidence evidence,
        FirstPersonRiderChannelObservation& observation)
    {
        switch (evidence)
        {
            case FirstPersonAssetMarkerEvidence::riderPrimary:
                return ExtractFirstPersonPixelCentroidObservation(
                    g1,
                    [](uint8_t pixel) {
                        return FirstPersonRiderPixelUsesPrimaryRemap(pixel);
                    },
                    observation);
            case FirstPersonAssetMarkerEvidence::riderSecondary:
                return ExtractFirstPersonPixelCentroidObservation(
                    g1,
                    [](uint8_t pixel) {
                        return FirstPersonRiderPixelUsesSecondaryRemap(pixel);
                    },
                    observation);
            case FirstPersonAssetMarkerEvidence::riderSilhouette:
            case FirstPersonAssetMarkerEvidence::bodySilhouette:
                return ExtractFirstPersonPixelCentroidObservation(
                    g1, [](uint8_t) { return true; }, observation);
        }
        return false;
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

    [[nodiscard]] inline std::optional<FirstPersonVec3>
        RecoverFirstPersonLocalPointFromOppositeViews(
            const std::array<FirstPersonRiderChannelObservation, 4>& views,
            uint8_t pair)
    {
        const uint8_t a = pair == 0 ? 0 : 1;
        const uint8_t b = pair == 0 ? 2 : 3;
        if (!views[a].valid || !views[b].valid)
            return std::nullopt;

        FirstPersonVec3 point{};
        if (pair == 0)
        {
            const float difference = views[0].x; // y - x
            const float sum = views[0].y - views[2].y; // x + y
            point.x = 0.5f * (sum - difference);
            point.y = 0.5f * (sum + difference);
            point.z = -0.5f * (views[0].y + views[2].y);
        }
        else
        {
            const float sum = views[3].x; // x + y
            const float difference = views[1].y - views[3].y; // y - x
            point.x = 0.5f * (sum - difference);
            point.y = 0.5f * (sum + difference);
            point.z = -0.5f * (views[1].y + views[3].y);
        }
        return point;
    }

    [[nodiscard]] inline std::optional<FirstPersonRecoveredAssetMarker>
        RecoverFirstPersonVehicleFrameMarker(
            const CarEntry& entry, uint8_t bodyFrameOffset,
            uint8_t riderAnimationFrame,
            FirstPersonAssetMarkerEvidence evidence)
    {
        if (entry.numCarImages == 0
            || !entry.groupEnabled(SpriteGroupType::slopeFlat))
            return std::nullopt;
        if (evidence != FirstPersonAssetMarkerEvidence::bodySilhouette
            && entry.numSeatingRows == 0)
            return std::nullopt;

        constexpr std::array<int32_t, 4> kImageDirections{ 0, 8, 16, 24 };
        std::array<FirstPersonRiderChannelObservation, 4> views{};
        for (uint8_t direction = 0; direction < 4; ++direction)
        {
            uint32_t image = entry.getSpriteOffset(
                SpriteGroupType::slopeFlat, kImageDirections[direction], 0)
                + bodyFrameOffset;
            if (evidence != FirstPersonAssetMarkerEvidence::bodySilhouette)
            {
                image += entry.numCarImages;
                if (entry.flags.has(CarEntryFlag::hasRiderAnimation))
                {
                    image += entry.numCarImages
                        * uint32_t(riderAnimationFrame);
                }
            }

            const auto* g1 = GfxGetG1Element(image);
            constexpr size_t kMaxMarkerSpritePixels = 65536;
            if (g1 == nullptr || g1->width <= 0 || g1->height <= 0
                || size_t(g1->width) * size_t(g1->height)
                    > kMaxMarkerSpritePixels
                || !ExtractFirstPersonAssetMarkerObservation(
                    *g1, evidence, views[direction]))
                return std::nullopt;
        }

        float reprojectionRmse = std::numeric_limits<float>::infinity();
        const auto all =
            RecoverFirstPersonLocalPointFromFourViews(
                views, &reprojectionRmse);
        const auto pairA =
            RecoverFirstPersonLocalPointFromOppositeViews(views, 0);
        const auto pairB =
            RecoverFirstPersonLocalPointFromOppositeViews(views, 1);
        if (!all.has_value() || !pairA.has_value() || !pairB.has_value()
            || reprojectionRmse > 2.5f)
            return std::nullopt;

        const float holdoutError = std::sqrt(
            (pairA->x - pairB->x) * (pairA->x - pairB->x)
            + (pairA->y - pairB->y) * (pairA->y - pairB->y)
            + (pairA->z - pairB->z) * (pairA->z - pairB->z));
        if (holdoutError > 4.0f)
            return std::nullopt;

        FirstPersonRecoveredAssetMarker result{};
        result.valid = true;
        // Native yaw zero has forward=-X and right=-Y.
        result.local = { -all->x, -all->y, all->z };
        result.reprojectionRmse = reprojectionRmse;
        result.holdoutError = holdoutError;
        result.evidence = evidence;
        return result;
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
        constexpr std::array<int32_t, 4>
            kImageDirections{ 0, 8, 16, 24 };

        std::array<FirstPersonRiderChannelObservation, 4>
            headViews{};
        std::array<FirstPersonRiderChannelObservation, 4>
            shirtViews{};
        float meanHeadExtent = 0.0f;
        for (uint8_t direction = 0; direction < 4; ++direction)
        {
            uint32_t bodyImage = entry.getSpriteOffset(
                SpriteGroupType::slopeFlat,
                kImageDirections[direction], 0);
            if (entry.flags.has(
                    CarEntryFlag::hasVehicleAnimation))
                bodyImage += animationFrame;
            uint32_t riderImage =
                bodyImage
                + entry.numCarImages * uint32_t(row + 1);
            if (row == 0
                && entry.flags.has(
                    CarEntryFlag::hasRiderAnimation))
            {
                riderImage +=
                    entry.numCarImages
                    * uint32_t(animationFrame);
            }

            const auto* g1 = GfxGetG1Element(riderImage);
            constexpr size_t kMaxRiderSpritePixels = 65536;
            if (g1 == nullptr || g1->width <= 0
                || g1->height <= 0
                || size_t(g1->width) * size_t(g1->height)
                    > kMaxRiderSpritePixels
                || !ExtractFirstPersonRiderChannelObservation(
                    *g1, secondary, shirtViews[direction])
                || !ExtractFirstPersonRiderHeadObservation(
                    *g1, secondary, headViews[direction]))
                return std::nullopt;
            meanHeadExtent +=
                0.125f
                * (headViews[direction].width
                    + headViews[direction].height);
        }

        float headRmse = std::numeric_limits<float>::infinity();
        float shirtRmse =
            std::numeric_limits<float>::infinity();
        const auto recoveredHead =
            RecoverFirstPersonLocalPointFromFourViews(
                headViews, &headRmse);
        const auto recoveredShirt =
            RecoverFirstPersonLocalPointFromFourViews(
                shirtViews, &shirtRmse);
        if (!recoveredHead.has_value()
            || !recoveredShirt.has_value()
            || headRmse > 2.5f || shirtRmse > 2.5f)
            return std::nullopt;

        const float dx =
            recoveredHead->x - recoveredShirt->x;
        const float dy =
            recoveredHead->y - recoveredShirt->y;
        const float dz =
            recoveredHead->z - recoveredShirt->z;
        // Independent anatomical check: the connected opaque head recovered
        // from all four views must sit above and close to the independently
        // recovered shirt marker. Projection consistency alone cannot satisfy
        // this if the selected opaque component belongs to the other rider or
        // to vehicle artwork.
        if (dz < 0.5f || dz > 20.0f
            || std::hypot(dx, dy) > 10.0f
            || std::abs(recoveredHead->x) > 64.0f
            || std::abs(recoveredHead->y) > 64.0f
            || recoveredHead->z < -32.0f
            || recoveredHead->z > 64.0f)
            return std::nullopt;

        FirstPersonPassengerAssetSeat result{};
        result.valid = true;
        // Native yaw 0 has forward=-X and right=-Y. The camera now follows
        // the recovered head centroid; no shirt->eye offset is assumed.
        result.localEye = {
            -recoveredHead->x,
            -recoveredHead->y,
            recoveredHead->z,
        };
        result.reprojectionRmse = headRmse;
        result.uncertainty =
            headRmse + shirtRmse
            + std::max(0.75f, 0.10f * meanHeadExtent);
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
            || entry.animationFrames < 4 || entry.animationFrames > 16)
            return {};

        static constexpr std::array<FirstPersonAssetMarkerEvidence, 4>
            kEvidenceOrder{ {
                FirstPersonAssetMarkerEvidence::riderPrimary,
                FirstPersonAssetMarkerEvidence::riderSecondary,
                FirstPersonAssetMarkerEvidence::riderSilhouette,
                FirstPersonAssetMarkerEvidence::bodySilhouette,
            } };

        const uint8_t frames = entry.animationFrames;
        for (const auto evidence : kEvidenceOrder)
        {
            std::array<FirstPersonVec3, 16> points{};
            float sourceUncertainty = 0.0f;
            bool complete = true;
            for (uint8_t frame = 0; frame < frames; ++frame)
            {
                const auto marker =
                    RecoverFirstPersonVehicleFrameMarker(
                        entry, frame, frame, evidence);
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

            auto calibration =
                FitFirstPersonMultiDimensionArtworkCalibration(
                    points, frames, sourceUncertainty);
            if (calibration.valid)
                return calibration;
        }
        return {};
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
        const auto* first = GfxGetG1Element(bodyImage);
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
