/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonVisualHull.h"

#include "../drawing/Drawing.Sprite.h"
#include "../interface/Viewport.h"
#include "../object/SmallSceneryEntry.h"
#include "../world/Location.hpp"
#include "../world/Scenery.h"
#include "../world/tile_element/SmallSceneryElement.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace OpenRCT2::Paint
{
    inline constexpr int32_t kFirstPersonSmallSceneryCollisionMaxHeight = 20;

    [[nodiscard]] constexpr bool
        FirstPersonSmallSceneryVisualReconstructionCoversHeight(
            int32_t visualHeight, int32_t reconstructedHeight)
    {
        return visualHeight > 0
            && visualHeight <= kFirstPersonSmallSceneryCollisionMaxHeight
            && reconstructedHeight >= visualHeight;
    }

    struct FirstPersonSmallSceneryWalkingMask
    {
        bool valid = false;
        static constexpr int32_t kCellSize = 2;
        static constexpr int32_t kCellsPerAxis =
            kCoordsXYStep / kCellSize;
        static constexpr size_t kMaxZLayers = 5;
        static constexpr size_t kWordsPerLayer =
            size_t(kCellsPerAxis * kCellsPerAxis) / 64;
        std::array<std::array<uint64_t, kWordsPerLayer>, kMaxZLayers>
            layers{};
        std::array<int16_t, kMaxZLayers> layerLowZ{};
        std::array<int16_t, kMaxZLayers> layerHighZ{};
        uint8_t layerCount = 0;

        [[nodiscard]] bool contains(
            size_t zLayer, int32_t xCell, int32_t yCell) const
        {
            if (zLayer >= layerCount || xCell < 0 || yCell < 0
                || xCell >= kCellsPerAxis
                || yCell >= kCellsPerAxis)
                return false;
            const size_t index =
                size_t(yCell * kCellsPerAxis + xCell);
            return (layers[zLayer][index / 64]
                    & (uint64_t{ 1 } << (index % 64)))
                != 0;
        }

        void add(size_t zLayer, int32_t xCell, int32_t yCell)
        {
            if (zLayer >= kMaxZLayers)
                return;
            const size_t index =
                size_t(yCell * kCellsPerAxis + xCell);
            layers[zLayer][index / 64]
                |= uint64_t{ 1 } << (index % 64);
            valid = true;
        }
    };

    [[nodiscard]] constexpr uint8_t FirstPersonSmallSceneryQuarterForPoint(
        int32_t x, int32_t y)
    {
        if (x >= kCoordsXYHalfTile)
            return y >= kCoordsXYHalfTile ? 0 : 1;
        return y < kCoordsXYHalfTile ? 2 : 3;
    }

    [[nodiscard]] inline bool FirstPersonG1PixelOpaque(
        const G1Element& g1, int32_t x, int32_t y)
    {
        return FirstPersonVisualHullPixelOpaque(
            g1, x, y);
    }

    [[nodiscard]] inline CoordsXY FirstPersonSmallSceneryPaintOffset(
        const SmallSceneryEntry& entry,
        const SmallSceneryElement& element,
        uint8_t viewportRotation)
    {
        const uint8_t direction =
            element.getDirectionWithOffset(viewportRotation) & 3u;
        CoordsXY offset{};
        if (entry.flags.has(SmallSceneryFlag::occupiesFullTile))
        {
            if (entry.flags.has(SmallSceneryFlag::occupiesHalfTile))
            {
                // PaintSmallSceneryBody changes only the sorting bounds with
                // direction; the sprite itself is always anchored at {3,3}.
                offset = { 3, 3 };
            }
            else
            {
                offset = { 15, 15 };
                if (entry.flags.has(SmallSceneryFlag::vOffsetCentre))
                {
                    offset = entry.flags.has(SmallSceneryFlag::prohibitWalls)
                        ? CoordsXY{ 1, 1 } : CoordsXY{ 3, 3 };
                }
            }
        }
        else
        {
            const uint8_t quadrant =
                (element.getSceneryQuadrant() + viewportRotation) & 3u;
            offset = {
                SceneryQuadrantOffsets[quadrant].x - 1,
                SceneryQuadrantOffsets[quadrant].y - 1,
            };
        }
        return offset.rotate(DirectionFlipXAxis(viewportRotation));
    }

    [[nodiscard]] inline ScreenCoordsXY
        FirstPersonSmallScenerySpritePixelForPoint(
            const SmallSceneryEntry& entry,
            const SmallSceneryElement& element,
            uint8_t viewportRotation,
            const CoordsXYZ& localPoint,
            const G1Element& g1)
    {
        const auto offset =
            FirstPersonSmallSceneryPaintOffset(
                entry, element, viewportRotation);
        const auto spriteOrigin = Translate3DTo2DWithZ(
            viewportRotation, { offset, 0 });
        const auto point =
            Translate3DTo2DWithZ(viewportRotation, localPoint);
        return {
            point.x - spriteOrigin.x - g1.xOffset,
            point.y - spriteOrigin.y - g1.yOffset,
        };
    }

    [[nodiscard]] inline uint8_t FirstPersonSmallSceneryWitherStage(
        const SmallSceneryEntry& entry,
        const SmallSceneryElement& element)
    {
        if (!entry.flags.has(SmallSceneryFlag::canWither))
            return 0;
        if (element.getAge() >= kSceneryWitherAgeThreshold2)
            return 2;
        if (element.getAge() >= kSceneryWitherAgeThreshold1)
            return 1;
        return 0;
    }

    [[nodiscard]] inline FirstPersonSmallSceneryWalkingMask
        BuildFirstPersonSmallSceneryWalkingMask(
            const SmallSceneryEntry& entry,
            const SmallSceneryElement& element)
    {
        FirstPersonSmallSceneryWalkingMask result{};
        if (entry.height < 4
            || entry.flags.hasAny(
                SmallSceneryFlag::isAnimated,
                SmallSceneryFlag::hasGlass,
                SmallSceneryFlag::isFountain,
                SmallSceneryFlag::isCupidFountain))
            return result;

        const uint8_t occupied =
            element.getOccupiedQuadrants() & 0x0Fu;
        if (occupied == 0)
            return result;

        const int32_t sampleTop =
            std::min<int32_t>(
                entry.height - 1,
                kFirstPersonSmallSceneryCollisionMaxHeight - 2);
        std::vector<int32_t> heights;
        for (const int32_t z : { 2, 6, 10, 14, 18 })
        {
            if (z <= sampleTop)
                heights.push_back(z);
        }
        if (heights.empty())
            heights.push_back(std::max(1, sampleTop));

        const uint8_t witherStage =
            FirstPersonSmallSceneryWitherStage(entry, element);
        std::array<const G1Element*, 4> sprites{};
        for (uint8_t rotation = 0; rotation < 4; ++rotation)
        {
            const uint8_t direction =
                element.getDirectionWithOffset(rotation) & 3u;
            const uint32_t image =
                entry.image + direction + uint32_t(witherStage) * 4u;
            sprites[rotation] = GfxGetG1Element(image);
            const auto* g1 = sprites[rotation];
            constexpr size_t kMaxSourcePixels = 65536;
            if (g1 == nullptr || g1->width <= 0 || g1->height <= 0
                || size_t(g1->width) * size_t(g1->height)
                    > kMaxSourcePixels)
                return {};
        }

        result.layerCount = uint8_t(std::min<size_t>(
            heights.size(),
            FirstPersonSmallSceneryWalkingMask::kMaxZLayers));
        const int32_t collisionTop =
            std::min<int32_t>(
                entry.height,
                kFirstPersonSmallSceneryCollisionMaxHeight);
        for (size_t layer = 0; layer < result.layerCount; ++layer)
        {
            const int32_t low = layer == 0
                ? 0
                : (heights[layer - 1] + heights[layer]) / 2;
            const int32_t high = layer + 1 == result.layerCount
                ? collisionTop
                : (heights[layer] + heights[layer + 1] + 1) / 2;
            result.layerLowZ[layer] =
                int16_t(std::clamp(low, 0, collisionTop));
            result.layerHighZ[layer] =
                int16_t(std::clamp(
                    std::max(low + 1, high), 0, collisionTop));
        }

        constexpr int32_t cellSize =
            FirstPersonSmallSceneryWalkingMask::kCellSize;
        constexpr int32_t cells =
            FirstPersonSmallSceneryWalkingMask::kCellsPerAxis;
        for (int32_t yCell = 0; yCell < cells; ++yCell)
        for (int32_t xCell = 0; xCell < cells; ++xCell)
        {
            const int32_t x = xCell * cellSize + cellSize / 2;
            const int32_t y = yCell * cellSize + cellSize / 2;
            const uint8_t quarter =
                FirstPersonSmallSceneryQuarterForPoint(x, y);
            if ((occupied & (1u << quarter)) == 0)
                continue;

            for (size_t layer = 0; layer < result.layerCount; ++layer)
            {
                const int32_t z = heights[layer];
                bool supported = true;
                for (uint8_t rotation = 0; rotation < 4; ++rotation)
                {
                    const auto pixel =
                        FirstPersonSmallScenerySpritePixelForPoint(
                            entry, element, rotation,
                            { x, y, z }, *sprites[rotation]);
                    if (!FirstPersonG1PixelOpaque(
                            *sprites[rotation],
                            pixel.x, pixel.y))
                    {
                        supported = false;
                        break;
                    }
                }
                if (supported)
                    result.add(layer, xCell, yCell);
            }
        }

        return result;
    }

    [[nodiscard]] inline FirstPersonVisualHull
        BuildFirstPersonSmallSceneryVisualHull(
            const SmallSceneryEntry& entry,
            const SmallSceneryElement& element)
    {
        // Preserve the established visual eligibility: the old collision-derived
        // renderer only replaced objects whose complete declared height fit
        // inside the 20-unit reconstruction envelope. The new visual hull is
        // independent of collision, but does not broaden that policy implicitly.
        if (!FirstPersonSmallSceneryVisualReconstructionCoversHeight(
                entry.height, entry.height)
            || entry.flags.hasAny(
                SmallSceneryFlag::isAnimated,
                SmallSceneryFlag::hasGlass,
                SmallSceneryFlag::isFountain,
                SmallSceneryFlag::isCupidFountain))
            return {};

        const uint8_t occupied =
            element.getOccupiedQuadrants() & 0x0Fu;
        if (occupied == 0)
            return {};

        const uint8_t witherStage =
            FirstPersonSmallSceneryWitherStage(
                entry, element);
        std::vector<FirstPersonVisualHullView> views;
        views.reserve(4);
        for (uint8_t rotation = 0; rotation < 4; ++rotation)
        {
            const uint8_t direction =
                element.getDirectionWithOffset(rotation)
                & 3u;
            const ImageIndex image =
                entry.image + direction
                + uint32_t(witherStage) * 4u;
            const auto* g1 = GfxGetG1Element(image);
            constexpr size_t kMaxSourcePixels = 65536;
            if (g1 == nullptr || g1->offset == nullptr
                || g1->width <= 0 || g1->height <= 0
                || size_t(g1->width)
                        * size_t(g1->height)
                    > kMaxSourcePixels)
                return {};

            FirstPersonVisualHullView view{};
            view.imageDirection = rotation;
            view.image = image;
            view.g1 = g1;
            for (int32_t y = 0; y < g1->height; ++y)
            for (int32_t x = 0; x < g1->width; ++x)
            {
                if (FirstPersonVisualHullPixelOpaque(
                        *g1, x, y))
                {
                    view.observed.add(
                        g1->xOffset + x,
                        g1->yOffset + y);
                }
            }
            if (view.observed.empty())
                return {};
            views.push_back(std::move(view));
        }

        FirstPersonVisualHullBounds bounds{};
        bounds.minForward = 0.0f;
        bounds.maxForward = float(kCoordsXYStep);
        bounds.minRight = 0.0f;
        bounds.maxRight = float(kCoordsXYStep);
        bounds.minUp = 0.0f;
        bounds.maxUp = float(entry.height);
        bounds.step = 2.0f;

        FirstPersonVisualHullConfig config{};
        config.minimumViews = 4;
        config.minimumOccupiedCells = 4;
        config.maximumOccupiedCells = 4096;
        config.maximumAxisCells = 20;
        config.minimumCandidateCoverage = 0.55f;
        config.minimumObservedCoverage = 0.30f;
        config.maximumEdgeError = 8;

        return BuildFirstPersonVisualHull(
            views, bounds, config,
            [&](uint8_t rotation,
                FirstPersonVec3 point) {
                const auto offset =
                    FirstPersonSmallSceneryPaintOffset(
                        entry, element, rotation);
                const auto spriteOrigin =
                    Translate3DTo2DWithZ(
                        rotation, { offset, 0 });
                const auto projected =
                    Translate3DTo2DWithZ(
                        rotation,
                        {
                            int32_t(std::lround(point.x)),
                            int32_t(std::lround(point.y)),
                            int32_t(std::lround(point.z)),
                        });
                return std::array<float, 2>{
                    float(projected.x - spriteOrigin.x),
                    float(projected.y - spriteOrigin.y),
                };
            },
            [occupied](FirstPersonVec3 point) {
                const int32_t x =
                    std::clamp(
                        int32_t(std::floor(point.x)),
                        0, kCoordsXYStep - 1);
                const int32_t y =
                    std::clamp(
                        int32_t(std::floor(point.y)),
                        0, kCoordsXYStep - 1);
                const uint8_t quarter =
                    FirstPersonSmallSceneryQuarterForPoint(
                        x, y);
                return (occupied
                        & (1u << quarter))
                    != 0;
            });
    }

    [[nodiscard]] inline const FirstPersonVisualHull*
        GetFirstPersonSmallSceneryVisualHull(
            const SmallSceneryEntry& entry,
            const SmallSceneryElement& element)
    {
        struct CacheKey
        {
            const SmallSceneryEntry* entry{};
            uint8_t direction{};
            uint8_t quadrant{};
            uint8_t occupied{};
            uint8_t witherStage{};

            bool operator==(const CacheKey&) const = default;
        };
        struct CacheKeyHash
        {
            size_t operator()(const CacheKey& key) const
            {
                size_t value =
                    reinterpret_cast<uintptr_t>(
                        key.entry) >> 4;
                value ^= size_t(key.direction) << 1;
                value ^= size_t(key.quadrant) << 4;
                value ^= size_t(key.occupied) << 7;
                value ^= size_t(key.witherStage) << 12;
                return value;
            }
        };
        struct CacheEntry
        {
            const uint8_t* sourceIdentity{};
            FirstPersonVisualHull hull{};
        };
        static std::unordered_map<
            CacheKey, CacheEntry, CacheKeyHash> cache;

        const CacheKey key{
            &entry,
            uint8_t(element.getDirection() & 3u),
            element.getSceneryQuadrant(),
            uint8_t(element.getOccupiedQuadrants()
                & 0x0Fu),
            FirstPersonSmallSceneryWitherStage(
                entry, element),
        };
        const auto* first =
            GfxGetG1Element(
                entry.image + key.direction
                + uint32_t(key.witherStage) * 4u);
        if (first == nullptr
            || first->offset == nullptr)
            return nullptr;

        auto& cached = cache[key];
        if (cached.sourceIdentity != first->offset)
        {
            cached.sourceIdentity = first->offset;
            cached.hull =
                BuildFirstPersonSmallSceneryVisualHull(
                    entry, element);
        }
        return cached.hull.valid
            ? &cached.hull : nullptr;
    }

    [[nodiscard]] inline const FirstPersonSmallSceneryWalkingMask*
        GetFirstPersonSmallSceneryWalkingMask(
            const SmallSceneryEntry& entry,
            const SmallSceneryElement& element)
    {
        struct CacheKey
        {
            const SmallSceneryEntry* entry{};
            uint8_t direction{};
            uint8_t quadrant{};
            uint8_t occupied{};
            uint8_t witherStage{};

            bool operator==(const CacheKey&) const = default;
        };
        struct CacheKeyHash
        {
            size_t operator()(const CacheKey& key) const
            {
                size_t value =
                    reinterpret_cast<uintptr_t>(key.entry) >> 4;
                value ^= size_t(key.direction) << 1;
                value ^= size_t(key.quadrant) << 4;
                value ^= size_t(key.occupied) << 7;
                value ^= size_t(key.witherStage) << 12;
                return value;
            }
        };
        struct CacheEntry
        {
            const uint8_t* sourceIdentity{};
            FirstPersonSmallSceneryWalkingMask mask{};
        };
        static std::unordered_map<CacheKey, CacheEntry, CacheKeyHash> cache;

        const CacheKey key{
            &entry,
            uint8_t(element.getDirection() & 3u),
            element.getSceneryQuadrant(),
            uint8_t(element.getOccupiedQuadrants() & 0x0Fu),
            FirstPersonSmallSceneryWitherStage(entry, element),
        };
        const auto* first =
            GfxGetG1Element(entry.image + key.direction
                + uint32_t(key.witherStage) * 4u);
        if (first == nullptr || first->offset == nullptr)
            return nullptr;

        auto& cached = cache[key];
        if (cached.sourceIdentity != first->offset)
        {
            cached.sourceIdentity = first->offset;
            cached.mask =
                BuildFirstPersonSmallSceneryWalkingMask(
                    entry, element);
        }
        return cached.mask.valid ? &cached.mask : nullptr;
    }
} // namespace OpenRCT2::Paint
