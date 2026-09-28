/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonVisualHull.h"

#include "../drawing/Drawing.Sprite.h"
#include "../ride/CarEntry.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace OpenRCT2::Paint
{
    using FirstPersonVehicleBodyTextureView =
        FirstPersonVisualHullTextureView;
    using FirstPersonVehicleBodyHull =
        FirstPersonVisualHull;

    [[nodiscard]] inline std::array<float, 2>
        ProjectFirstPersonVehicleLocalPoint(
            uint8_t imageDirection, FirstPersonVec3 point)
    {
        constexpr float kTwoPi = 6.28318530717958647692f;
        const float theta =
            float(imageDirection & 31u) * (kTwoPi / 32.0f);
        const float c = std::cos(theta);
        const float s = std::sin(theta);
        // Native yaw zero points along -X. Local right at yaw zero is -Y.
        const float worldX = -c * point.x - s * point.y;
        const float worldY = s * point.x - c * point.y;
        return {
            worldY - worldX,
            0.5f * (worldX + worldY) - point.z,
        };
    }

    [[nodiscard]] inline bool FirstPersonVehicleBodyPixelOpaque(
        const G1Element& g1, int32_t x, int32_t y)
    {
        return FirstPersonVisualHullPixelOpaque(
            g1, x, y);
    }

    using FirstPersonVehicleBodyView =
        FirstPersonVisualHullView;

    [[nodiscard]] inline std::vector<FirstPersonVehicleBodyView>
        CollectFirstPersonVehicleBodyViews(const CarEntry& entry)
    {
        std::vector<FirstPersonVehicleBodyView> result;
        if (!entry.groupEnabled(SpriteGroupType::slopeFlat)
            || entry.flags.hasAny(
                CarEntryFlag::hasVehicleAnimation,
                CarEntryFlag::hasRiderAnimation,
                CarEntryFlag::hasSpinning,
                CarEntryFlag::hasSpinningCombinedWithNonSpinning,
                CarEntryFlag::hasSwinging,
                CarEntryFlag::isChairlift,
                CarEntryFlag::isGoKart,
                CarEntryFlag::isMiniGolf,
                CarEntryFlag::isReverserCoasterBogie,
                CarEntryFlag::isReverserCoasterPassengerCar))
            return result;

        std::unordered_set<uint32_t> seenImages;
        for (uint8_t direction = 0; direction < 32; ++direction)
        {
            const uint32_t image = entry.getSpriteOffset(
                SpriteGroupType::slopeFlat, direction, 0);
            if (!seenImages.insert(image).second)
                continue;
            const auto* g1 = GfxGetG1Element(image);
            constexpr size_t kMaxBodyPixels = 65536;
            if (g1 == nullptr || g1->offset == nullptr
                || g1->width <= 0 || g1->height <= 0
                || size_t(g1->width) * size_t(g1->height)
                    > kMaxBodyPixels)
                return {};

            FirstPersonVehicleBodyView view{};
            view.imageDirection = direction;
            view.image = image;
            view.g1 = g1;
            if (g1->flags.has(G1Flag::hasRLECompression))
            {
                for (int32_t y = 0; y < g1->height; ++y)
                for (int32_t x = 0; x < g1->width; ++x)
                {
                    if (FirstPersonVehicleBodyPixelOpaque(*g1, x, y))
                        view.observed.add(
                            g1->xOffset + x, g1->yOffset + y);
                }
            }
            else
            {
                const bool transparent =
                    g1->flags.has(G1Flag::hasTransparency);
                for (int32_t y = 0; y < g1->height; ++y)
                for (int32_t x = 0; x < g1->width; ++x)
                {
                    const uint8_t pixel = g1->offset[
                        size_t(y) * size_t(g1->width) + size_t(x)];
                    if (!transparent || pixel != 0)
                        view.observed.add(
                            g1->xOffset + x, g1->yOffset + y);
                }
            }
            if (view.observed.empty())
                return {};
            result.push_back(std::move(view));
        }
        return result;
    }

    [[nodiscard]] inline FirstPersonVehicleBodyHull
        BuildFirstPersonVehicleBodyHull(const CarEntry& entry)
    {
        const auto views =
            CollectFirstPersonVehicleBodyViews(entry);
        if (views.size() < 16)
            return {};

        constexpr float step = 4.0f;
        const int32_t halfXY = std::clamp(
            int32_t(entry.spriteWidth) + 4, 8, 40);
        const int32_t lowUp = -std::clamp(
            int32_t(entry.spriteHeightNegative) + 4, 4, 32);
        const int32_t highUp = std::clamp(
            int32_t(entry.spriteHeightPositive) + 4, 8, 48);

        FirstPersonVisualHullBounds bounds{};
        bounds.minForward = float(-halfXY);
        bounds.maxForward = float(halfXY);
        bounds.minRight = float(-halfXY);
        bounds.maxRight = float(halfXY);
        bounds.minUp = float(lowUp);
        bounds.maxUp = float(highUp);
        bounds.step = step;

        FirstPersonVisualHullConfig config{};
        config.minimumViews = 16;
        config.minimumConstructionViews = 8;
        config.minimumValidationViews = 8;
        config.minimumOccupiedCells = 8;
        config.maximumOccupiedCells = 2048;
        config.maximumAxisCells = 20;
        config.minimumCandidateCoverage = 0.60f;
        config.minimumObservedCoverage = 0.32f;
        config.maximumEdgeError = 6;

        return BuildFirstPersonVisualHull(
            views, bounds, config,
            [](uint8_t imageDirection,
                FirstPersonVec3 point) {
                return ProjectFirstPersonVehicleLocalPoint(
                    imageDirection, point);
            },
            [](FirstPersonVec3) {
                return true;
            });
    }

    [[nodiscard]] inline const FirstPersonVehicleBodyHull*
        GetFirstPersonVehicleBodyHull(const CarEntry& entry)
    {
        struct CacheEntry
        {
            const uint8_t* sourceIdentity{};
            FirstPersonVehicleBodyHull hull{};
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
            cached.hull = BuildFirstPersonVehicleBodyHull(entry);
        }
        return cached.hull.valid ? &cached.hull : nullptr;
    }
} // namespace OpenRCT2::Paint
