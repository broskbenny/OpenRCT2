/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

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
    struct FirstPersonSmallSceneryWalkingMask
    {
        bool valid = false;
        static constexpr int32_t kCellSize = 2;
        static constexpr int32_t kCellsPerAxis =
            kCoordsXYStep / kCellSize;
        std::array<uint64_t, 4> cells{};
        int32_t collisionHeight = 0;

        [[nodiscard]] bool contains(int32_t xCell, int32_t yCell) const
        {
            if (xCell < 0 || yCell < 0
                || xCell >= kCellsPerAxis
                || yCell >= kCellsPerAxis)
                return false;
            const size_t index =
                size_t(yCell * kCellsPerAxis + xCell);
            return (cells[index / 64] & (uint64_t{ 1 } << (index % 64)))
                != 0;
        }

        void add(int32_t xCell, int32_t yCell)
        {
            const size_t index =
                size_t(yCell * kCellsPerAxis + xCell);
            cells[index / 64] |= uint64_t{ 1 } << (index % 64);
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
                const int32_t startX = *run++;
                endOfLine = (length & 0x80u) != 0;
                length &= 0x7Fu;
                if (x >= startX && x < startX + length)
                    return run[x - startX] != 0;
                run += length;
            }
            return false;
        }

        const uint8_t pixel =
            g1.offset[size_t(y) * size_t(g1.width) + size_t(x)];
        return !g1.flags.has(G1Flag::hasTransparency) || pixel != 0;
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
            std::min<int32_t>(entry.height - 1, 18);
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

            size_t supportedHeights = 0;
            for (const int32_t z : heights)
            {
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
                    ++supportedHeights;
            }

            const size_t required =
                std::max<size_t>(1, (heights.size() + 1) / 2);
            if (supportedHeights >= required)
                result.add(xCell, yCell);
        }

        result.collisionHeight =
            std::min<int32_t>(entry.height, 20);
        return result;
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
