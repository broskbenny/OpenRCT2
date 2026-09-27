/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonAssetReconstruction.h"
#include "FirstPersonMath.h"

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
    struct FirstPersonVehicleBodyHull
    {
        bool valid = false;
        float step = 4.0f;
        int16_t minForward{};
        int16_t minRight{};
        int16_t minUp{};
        uint8_t sizeForward{};
        uint8_t sizeRight{};
        uint8_t sizeUp{};
        std::vector<uint8_t> occupied;
        float minimumCandidateCoverage{};
        float minimumObservedCoverage{};
        int32_t maximumEdgeError{};

        [[nodiscard]] bool contains(
            int32_t forward, int32_t right, int32_t up) const
        {
            if (forward < 0 || right < 0 || up < 0
                || forward >= sizeForward
                || right >= sizeRight || up >= sizeUp)
                return false;
            const size_t index =
                (size_t(up) * sizeRight + size_t(right))
                    * sizeForward + size_t(forward);
            return index < occupied.size() && occupied[index] != 0;
        }

        [[nodiscard]] FirstPersonVec3 centre(
            int32_t forward, int32_t right, int32_t up) const
        {
            return {
                float(minForward) + (float(forward) + 0.5f) * step,
                float(minRight) + (float(right) + 0.5f) * step,
                float(minUp) + (float(up) + 0.5f) * step,
            };
        }

        [[nodiscard]] bool containsPoint(
            FirstPersonVec3 point) const
        {
            const int32_t forward = int32_t(std::floor(
                (point.x - float(minForward)) / step));
            const int32_t right = int32_t(std::floor(
                (point.y - float(minRight)) / step));
            const int32_t up = int32_t(std::floor(
                (point.z - float(minUp)) / step));
            return contains(forward, right, up);
        }
    };

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
        if (g1.offset == nullptr || x < 0 || y < 0
            || x >= g1.width || y >= g1.height
            || g1.flags.has(G1Flag::isPalette))
            return false;
        if (g1.flags.has(G1Flag::hasRLECompression))
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
                const int32_t start = *run++;
                endOfLine = (length & 0x80u) != 0;
                length &= 0x7Fu;
                if (x >= start && x < start + length)
                    return run[x - start] != 0;
                run += length;
            }
            return false;
        }
        const uint8_t pixel =
            g1.offset[size_t(y) * size_t(g1.width) + size_t(x)];
        return !g1.flags.has(G1Flag::hasTransparency) || pixel != 0;
    }

    struct FirstPersonVehicleBodyView
    {
        uint8_t imageDirection{};
        uint32_t image{};
        const G1Element* g1{};
        FirstPersonSilhouette observed{};
    };

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

    [[nodiscard]] inline bool FirstPersonVehiclePointSupportedByView(
        const FirstPersonVehicleBodyView& view,
        FirstPersonVec3 point)
    {
        const auto projected =
            ProjectFirstPersonVehicleLocalPoint(
                view.imageDirection, point);
        const int32_t px =
            int32_t(std::lround(projected[0]))
            - view.g1->xOffset;
        const int32_t py =
            int32_t(std::lround(projected[1]))
            - view.g1->yOffset;
        for (int32_t dy = -1; dy <= 1; ++dy)
        for (int32_t dx = -1; dx <= 1; ++dx)
        {
            if (FirstPersonVehicleBodyPixelOpaque(
                    *view.g1, px + dx, py + dy))
                return true;
        }
        return false;
    }

    inline void AddFirstPersonVehicleHullProjection(
        FirstPersonSilhouette& silhouette,
        uint8_t imageDirection, FirstPersonVec3 centre,
        float step)
    {
        const auto projected =
            ProjectFirstPersonVehicleLocalPoint(
                imageDirection, centre);
        const int32_t x = int32_t(std::lround(projected[0]));
        const int32_t y = int32_t(std::lround(projected[1]));
        const int32_t radius = std::max(
            1, int32_t(std::ceil(step * 0.75f)));
        for (int32_t py = y - radius; py <= y + radius; ++py)
        for (int32_t px = x - radius; px <= x + radius; ++px)
            silhouette.add(px, py);
    }

    [[nodiscard]] inline FirstPersonVehicleBodyHull
        BuildFirstPersonVehicleBodyHull(const CarEntry& entry)
    {
        FirstPersonVehicleBodyHull result{};
        const auto views = CollectFirstPersonVehicleBodyViews(entry);
        // A body shell is only worthwhile when the asset gives substantially
        // more angular evidence than a four-view scenery reconstruction.
        if (views.size() < 16)
            return result;

        std::vector<const FirstPersonVehicleBodyView*> construction;
        std::vector<const FirstPersonVehicleBodyView*> validation;
        construction.reserve((views.size() + 1) / 2);
        validation.reserve(views.size() / 2);
        for (size_t i = 0; i < views.size(); ++i)
        {
            (i & 1u ? validation : construction).push_back(&views[i]);
        }
        if (construction.size() < 8 || validation.size() < 8)
            return result;

        constexpr float step = 4.0f;
        const int32_t halfXY = std::clamp(
            int32_t(entry.spriteWidth) + 4, 8, 40);
        const int32_t lowUp = -std::clamp(
            int32_t(entry.spriteHeightNegative) + 4, 4, 32);
        const int32_t highUp = std::clamp(
            int32_t(entry.spriteHeightPositive) + 4, 8, 48);
        const int32_t spanXY = halfXY * 2;
        const int32_t spanUp = highUp - lowUp;
        const int32_t nXY =
            std::max(1, int32_t(std::ceil(float(spanXY) / step)));
        const int32_t nUp =
            std::max(1, int32_t(std::ceil(float(spanUp) / step)));
        if (nXY > 20 || nUp > 20)
            return result;

        result.step = step;
        result.minForward = int16_t(-halfXY);
        result.minRight = int16_t(-halfXY);
        result.minUp = int16_t(lowUp);
        result.sizeForward = uint8_t(nXY);
        result.sizeRight = uint8_t(nXY);
        result.sizeUp = uint8_t(nUp);
        result.occupied.assign(
            size_t(nXY) * size_t(nXY) * size_t(nUp), 0);

        size_t occupiedCount = 0;
        for (int32_t up = 0; up < nUp; ++up)
        for (int32_t right = 0; right < nXY; ++right)
        for (int32_t forward = 0; forward < nXY; ++forward)
        {
            const auto point = result.centre(
                forward, right, up);
            bool supported = true;
            for (const auto* view : construction)
            {
                if (!FirstPersonVehiclePointSupportedByView(
                        *view, point))
                {
                    supported = false;
                    break;
                }
            }
            if (!supported)
                continue;
            const size_t index =
                (size_t(up) * size_t(nXY) + size_t(right))
                    * size_t(nXY) + size_t(forward);
            result.occupied[index] = 1;
            ++occupiedCount;
        }
        if (occupiedCount < 8 || occupiedCount > 2048)
            return {};

        result.minimumCandidateCoverage = 1.0f;
        result.minimumObservedCoverage = 1.0f;
        result.maximumEdgeError = 0;
        for (const auto* view : validation)
        {
            FirstPersonSilhouette predicted{};
            for (int32_t up = 0; up < nUp; ++up)
            for (int32_t right = 0; right < nXY; ++right)
            for (int32_t forward = 0; forward < nXY; ++forward)
            {
                if (!result.contains(forward, right, up))
                    continue;
                AddFirstPersonVehicleHullProjection(
                    predicted, view->imageDirection,
                    result.centre(forward, right, up), step);
            }
            const auto fit =
                CompareFirstPersonSilhouettes(
                    view->observed, predicted);
            if (!fit.valid
                || fit.candidateCoverage < 0.60f
                || fit.observedCoverage < 0.32f
                || fit.maxEdgeError > 6)
                return {};
            result.minimumCandidateCoverage = std::min(
                result.minimumCandidateCoverage,
                fit.candidateCoverage);
            result.minimumObservedCoverage = std::min(
                result.minimumObservedCoverage,
                fit.observedCoverage);
            result.maximumEdgeError = std::max(
                result.maximumEdgeError, fit.maxEdgeError);
        }

        result.valid = true;
        return result;
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
