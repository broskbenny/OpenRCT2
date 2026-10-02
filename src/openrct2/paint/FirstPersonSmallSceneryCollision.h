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

    [[nodiscard]] inline bool
        FirstPersonSmallSceneryUsesAuthoritativeOccupancy(
            const SmallSceneryEntry& entry)
    {
        return entry.height > 0;
    }

    [[nodiscard]] inline bool
        FirstPersonSmallSceneryCanUseSilhouetteRefinement(
            const SmallSceneryEntry& entry)
    {
        return !entry.flags.hasAny(
            SmallSceneryFlag::isAnimated,
            SmallSceneryFlag::hasGlass,
            SmallSceneryFlag::isFountain,
            SmallSceneryFlag::isCupidFountain);
    }

    inline uint64_t&
        FirstPersonSmallSceneryReconstructionGeneration()
    {
        static uint64_t generation = 1;
        return generation;
    }

    inline void ClearFirstPersonSmallSceneryReconstructionCache()
    {
        auto& generation =
            FirstPersonSmallSceneryReconstructionGeneration();
        ++generation;
        if (generation == 0)
            generation = 1;
    }

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

    inline void AppendFirstPersonG1OpaqueSilhouette(
        const G1Element& g1,
        FirstPersonSilhouette& silhouette)
    {
        if (g1.offset == nullptr
            || g1.width <= 0 || g1.height <= 0
            || g1.flags.has(G1Flag::isPalette))
            return;

        if (g1.flags.has(G1Flag::hasRLECompression))
        {
            for (int32_t y = 0; y < g1.height; ++y)
            {
                const uint16_t lineOffset =
                    uint16_t(g1.offset[y * 2])
                    | (uint16_t(g1.offset[y * 2 + 1])
                        << 8);
                const uint8_t* run =
                    g1.offset + lineOffset;
                bool endOfLine = false;
                size_t guard = 0;
                while (!endOfLine && guard++ < 256)
                {
                    uint8_t length = *run++;
                    const int32_t start = *run++;
                    endOfLine =
                        (length & 0x80u) != 0;
                    length &= 0x7fu;
                    for (uint8_t i = 0;
                         i < length; ++i)
                    {
                        if (run[i] != 0)
                        {
                            silhouette.add(
                                g1.xOffset + start + i,
                                g1.yOffset + y);
                        }
                    }
                    run += length;
                }
            }
            return;
        }

        const bool transparent =
            g1.flags.has(G1Flag::hasTransparency);
        for (int32_t y = 0; y < g1.height; ++y)
        for (int32_t x = 0; x < g1.width; ++x)
        {
            const uint8_t pixel =
                g1.offset[
                    size_t(y) * size_t(g1.width)
                    + size_t(x)];
            if (!transparent || pixel != 0)
            {
                silhouette.add(
                    g1.xOffset + x,
                    g1.yOffset + y);
            }
        }
    }

    [[nodiscard]] inline CoordsXY FirstPersonSmallSceneryPaintOffset(
        const SmallSceneryEntry& entry,
        const SmallSceneryElement& element,
        uint8_t viewportRotation)
    {
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

    template<typename SpriteLookup>
    [[nodiscard]] inline FirstPersonVisualHull
        BuildFirstPersonSmallSceneryVisualHullFromSnapshot(
            const SmallSceneryEntry& entry,
            const SmallSceneryElement& element, SpriteLookup&& lookup)
    {
        // Declared occupied quadrants and object height are the physical
        // contract for every scenery item that has one. Stateful/glass/effect
        // artwork may skip silhouette carving, but it must not erase that
        // authoritative body.
        if (!FirstPersonSmallSceneryUsesAuthoritativeOccupancy(entry))
            return {};
        const bool allowSilhouetteRefinement =
            FirstPersonSmallSceneryCanUseSilhouetteRefinement(
                entry);

        const uint8_t occupied =
            element.getOccupiedQuadrants() & 0x0Fu;
        if (occupied == 0)
            return {};

        const uint8_t witherStage =
            FirstPersonSmallSceneryWitherStage(
                entry, element);
        std::vector<FirstPersonVisualHullView> views;
        views.reserve(4);
        std::vector<FirstPersonVisualHullTextureView> textureViews;
        textureViews.reserve(4);
        for (uint8_t rotation = 0; rotation < 4; ++rotation)
        {
            const uint8_t direction =
                element.getDirectionWithOffset(rotation)
                & 3u;
            const ImageIndex image =
                entry.image + direction
                + uint32_t(witherStage) * 4u;
            const auto* g1 = lookup(image);
            if (g1 == nullptr || g1->offset == nullptr
                || g1->width <= 0 || g1->height <= 0)
                continue;

            // Texture availability and silhouette-analysis eligibility are
            // separate. Special/stateful artwork and oversized sprites skip
            // carving while the occupancy body remains valid.
            textureViews.push_back({ rotation, image });
            if (!allowSilhouetteRefinement)
                continue;
            constexpr size_t kMaxSourcePixels = 65536;
            if (size_t(g1->width) * size_t(g1->height)
                > kMaxSourcePixels)
                continue;

            FirstPersonVisualHullView view{};
            view.imageDirection = rotation;
            view.image = image;
            view.g1 = g1;
            AppendFirstPersonG1OpaqueSilhouette(
                *g1, view.observed);
            if (!view.observed.empty())
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
        config.maximumOccupiedCells = 32768;
        config.maximumGridCells = 131072;
        config.maximumAxisCells = 128;
        config.minimumCandidateCoverage = 0.55f;
        config.minimumObservedCoverage = 0.30f;
        config.maximumEdgeError = 8;

        const auto occupancyPredicate =
            [occupied](FirstPersonVec3 point) {
                const int32_t x = std::clamp(
                    int32_t(std::floor(point.x)),
                    0, kCoordsXYStep - 1);
                const int32_t y = std::clamp(
                    int32_t(std::floor(point.y)),
                    0, kCoordsXYStep - 1);
                const uint8_t quarter =
                    FirstPersonSmallSceneryQuarterForPoint(x, y);
                return (occupied & (1u << quarter)) != 0;
            };
        if (allowSilhouetteRefinement)
        {
            const auto projectPoint =
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
                };
            const auto pointSupported =
                [&](const FirstPersonVisualHullView& view,
                    FirstPersonVec3 point) {
                    const auto projected =
                        projectPoint(
                            view.imageDirection, point);
                    const int32_t px =
                        int32_t(std::lround(
                            projected[0]));
                    const int32_t py =
                        int32_t(std::lround(
                            projected[1]));
                    for (int32_t dy = -1;
                         dy <= 1; ++dy)
                    for (int32_t dx = -1;
                         dx <= 1; ++dx)
                    {
                        if (view.observed.contains(
                                px + dx, py + dy))
                            return true;
                    }
                    return false;
                };
            auto carved = BuildFirstPersonVisualHull(
                views, bounds, config,
                projectPoint, occupancyPredicate,
                pointSupported);
            if (carved.valid)
                return carved;
        }

        return BuildFirstPersonOccupancyHull(
            bounds, config, textureViews, occupancyPredicate);
    }

    inline constexpr uint8_t kFirstPersonSmallSceneryUnsplitArtworkQuarter = 0xFF;

    struct FirstPersonSmallSceneryVisualFace
    {
        FirstPersonVisualHullFace face{};
        uint8_t artworkQuarter = kFirstPersonSmallSceneryUnsplitArtworkQuarter;
        bool syntheticInterior = false;
    };

    [[nodiscard]] inline bool FirstPersonSmallScenerySameFaceGeometry(
        const FirstPersonVisualHullFace& a,
        const FirstPersonVisualHullFace& b)
    {
        const auto samePoint = [](FirstPersonVec3 lhs, FirstPersonVec3 rhs) {
            return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
        };
        return std::all_of(
            a.corners.begin(), a.corners.end(),
            [&](FirstPersonVec3 point) {
                return std::any_of(
                    b.corners.begin(), b.corners.end(),
                    [&](FirstPersonVec3 other) {
                        return samePoint(point, other);
                    });
            });
    }

    [[nodiscard]] inline std::vector<FirstPersonSmallSceneryVisualFace>
        BuildFirstPersonSmallSceneryVisualFaces(
            const FirstPersonVisualHull& hull, bool splitTreeArtwork)
    {
        std::vector<FirstPersonSmallSceneryVisualFace> result;
        if (!splitTreeArtwork)
        {
            const auto faces = BuildFirstPersonVisualHullBoundaryFaces(hull);
            result.reserve(faces.size());
            for (const auto& face : faces)
            {
                result.push_back({
                    face,
                    kFirstPersonSmallSceneryUnsplitArtworkQuarter,
                    false
                });
            }
            return result;
        }
        if (!hull.valid || hull.occupied.empty())
            return result;

        const float minForward = hull.minForward;
        const float minRight = hull.minRight;
        const float maxForward =
            hull.minForward + float(hull.sizeForward) * hull.step;
        const float maxRight =
            hull.minRight + float(hull.sizeRight) * hull.step;
        const float splitForward =
            (minForward + maxForward) * 0.5f;
        const float splitRight =
            (minRight + maxRight) * 0.5f;

        const auto quarterForPoint =
            [&](FirstPersonVec3 point) {
                const bool highForward =
                    point.x >= splitForward;
                const bool highRight =
                    point.y >= splitRight;
                if (highForward)
                    return uint8_t(highRight ? 0 : 1);
                return uint8_t(highRight ? 3 : 2);
            };
        const auto isSyntheticInterior =
            [&](const FirstPersonVisualHullFace& face) {
                switch (face.kind)
                {
                    case FirstPersonVisualHullFaceKind::minForward:
                        return face.corners[0].x != minForward;
                    case FirstPersonVisualHullFaceKind::maxForward:
                        return face.corners[0].x != maxForward;
                    case FirstPersonVisualHullFaceKind::minRight:
                        return face.corners[0].y != minRight;
                    case FirstPersonVisualHullFaceKind::maxRight:
                        return face.corners[0].y != maxRight;
                    case FirstPersonVisualHullFaceKind::bottom:
                    case FirstPersonVisualHullFaceKind::top:
                        return false;
                }
                return false;
            };

        for (uint8_t quarter = 0; quarter < 4; ++quarter)
        {
            auto quarterHull = hull;
            for (int32_t up = 0; up < hull.sizeUp; ++up)
            for (int32_t right = 0; right < hull.sizeRight; ++right)
            for (int32_t forward = 0;
                 forward < hull.sizeForward; ++forward)
            {
                if (!hull.contains(forward, right, up))
                    continue;
                if (quarterForPoint(
                        hull.centre(forward, right, up))
                    == quarter)
                    continue;
                const size_t index =
                    (size_t(up) * size_t(hull.sizeRight)
                        + size_t(right))
                        * size_t(hull.sizeForward)
                    + size_t(forward);
                if (index < quarterHull.occupied.size())
                    quarterHull.occupied[index] = 0;
            }

            const auto quarterFaces =
                BuildFirstPersonVisualHullBoundaryFaces(
                    quarterHull);
            for (const auto& face : quarterFaces)
            {
                const bool synthetic =
                    isSyntheticInterior(face);
                const auto duplicate =
                    std::find_if(
                        result.begin(), result.end(),
                        [&](const auto& existing) {
                            return synthetic
                                && existing.syntheticInterior
                                && FirstPersonSmallScenerySameFaceGeometry(
                                    existing.face, face);
                        });
                if (duplicate != result.end())
                    continue;
                result.push_back({
                    face, quarter, synthetic
                });
            }
        }
        return result;
    }

    [[nodiscard]] inline FirstPersonVec3
        FirstPersonSmallSceneryTreeArtworkPoint(
            const FirstPersonVisualHull& hull,
            uint8_t artworkQuarter,
            FirstPersonVec3 point)
    {
        if (artworkQuarter >= 4)
            return point;

        const float minForward = hull.minForward;
        const float minRight = hull.minRight;
        const float maxForward =
            hull.minForward + float(hull.sizeForward) * hull.step;
        const float maxRight =
            hull.minRight + float(hull.sizeRight) * hull.step;
        const float splitForward =
            (minForward + maxForward) * 0.5f;
        const float splitRight =
            (minRight + maxRight) * 0.5f;

        const bool highForward =
            artworkQuarter == 0 || artworkQuarter == 1;
        const bool highRight =
            artworkQuarter == 0 || artworkQuarter == 3;
        const float quarterMinForward =
            highForward ? splitForward : minForward;
        const float quarterMaxForward =
            highForward ? maxForward : splitForward;
        const float quarterMinRight =
            highRight ? splitRight : minRight;
        const float quarterMaxRight =
            highRight ? maxRight : splitRight;
        const float quarterCentreForward =
            (quarterMinForward + quarterMaxForward) * 0.5f;
        const float quarterCentreRight =
            (quarterMinRight + quarterMaxRight) * 0.5f;

        // Rotate only the artwork domain, never the physical/collision hull.
        // Each quarter's former outer corner becomes its inward corner at the
        // shared tree centre, moving the repeated trunk material inward.
        point.x =
            2.0f * quarterCentreForward - point.x;
        point.y =
            2.0f * quarterCentreRight - point.y;
        return point;
    }

    [[nodiscard]] inline FirstPersonVisualHull BuildFirstPersonSmallSceneryVisualHull(
        const SmallSceneryEntry& entry, const SmallSceneryElement& element)
    {
        return BuildFirstPersonSmallSceneryVisualHullFromSnapshot(entry, element,
            [](ImageIndex image) { return GfxGetG1Element(image); });
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
            uint64_t generation{};
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
        const auto generation =
            FirstPersonSmallSceneryReconstructionGeneration();
        if (cached.sourceIdentity != first->offset
            || cached.generation != generation)
        {
            cached.sourceIdentity = first->offset;
            cached.generation = generation;
            cached.hull =
                BuildFirstPersonSmallSceneryVisualHull(
                    entry, element);
        }
        return cached.hull.valid
            ? &cached.hull : nullptr;
    }

    [[nodiscard]] inline FirstPersonSmallSceneryWalkingMask
        BuildFirstPersonSmallSceneryWalkingMaskFromHull(
            const FirstPersonVisualHull& hull,
            int32_t visualHeight)
    {
        FirstPersonSmallSceneryWalkingMask result{};
        if (!hull.valid || visualHeight <= 0)
            return result;

        const int32_t collisionTop =
            std::min(
                visualHeight,
                kFirstPersonSmallSceneryCollisionMaxHeight);
        if (collisionTop <= 0)
            return result;

        constexpr int32_t kLayerHeight = 4;
        result.layerCount = uint8_t(std::min<int32_t>(
            FirstPersonSmallSceneryWalkingMask::kMaxZLayers,
            (collisionTop + kLayerHeight - 1) / kLayerHeight));
        for (size_t layer = 0; layer < result.layerCount; ++layer)
        {
            const int32_t low = int32_t(layer) * kLayerHeight;
            const int32_t high =
                std::min(collisionTop, low + kLayerHeight);
            result.layerLowZ[layer] = int16_t(low);
            result.layerHighZ[layer] = int16_t(high);
        }

        constexpr int32_t cellSize =
            FirstPersonSmallSceneryWalkingMask::kCellSize;
        constexpr int32_t cells =
            FirstPersonSmallSceneryWalkingMask::kCellsPerAxis;
        for (int32_t yCell = 0; yCell < cells; ++yCell)
        for (int32_t xCell = 0; xCell < cells; ++xCell)
        {
            const float x =
                float(xCell * cellSize) + float(cellSize) * 0.5f;
            const float y =
                float(yCell * cellSize) + float(cellSize) * 0.5f;
            for (size_t layer = 0; layer < result.layerCount; ++layer)
            {
                const int32_t low = result.layerLowZ[layer];
                const int32_t high = result.layerHighZ[layer];
                bool occupied = false;
                for (float z = float(low) + 1.0f;
                     z < float(high); z += 2.0f)
                {
                    if (hull.containsPoint({ x, y, z }))
                    {
                        occupied = true;
                        break;
                    }
                }
                if (occupied)
                    result.add(layer, xCell, yCell);
            }
        }
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
            uint64_t generation{};
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
        const auto generation =
            FirstPersonSmallSceneryReconstructionGeneration();
        if (cached.sourceIdentity != first->offset
            || cached.generation != generation)
        {
            cached.sourceIdentity = first->offset;
            cached.generation = generation;
            if (const auto* hull =
                    GetFirstPersonSmallSceneryVisualHull(
                        entry, element);
                hull != nullptr)
            {
                cached.mask =
                    BuildFirstPersonSmallSceneryWalkingMaskFromHull(
                        *hull, entry.height);
            }
            else
            {
                cached.mask =
                    BuildFirstPersonSmallSceneryWalkingMask(
                        entry, element);
            }
        }
        return cached.mask.valid ? &cached.mask : nullptr;
    }
} // namespace OpenRCT2::Paint
