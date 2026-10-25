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

    template<typename SpriteLookup>
    [[nodiscard]] inline std::vector<FirstPersonVehicleBodyView>
        CollectFirstPersonVehicleBodyViews(const CarEntry& entry, SpriteLookup&& lookup)
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
            const auto* g1 = lookup(image);
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

    template<typename SpriteLookup>
    [[nodiscard]] inline FirstPersonVehicleBodyHull
        BuildFirstPersonVehicleBodyHullFromSnapshot(const CarEntry& entry, SpriteLookup&& lookup)
    {
        const auto views =
            CollectFirstPersonVehicleBodyViews(entry, lookup);
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


    // Asset-local faces are independent of vehicle location, interpolation and
    // recolouring. Construct them on the reconstruction worker, not once per
    // vehicle per frame. Keep the original 6-neighbour exposed-voxel topology
    // and the exact native-view texture-choice scoring.
    struct FirstPersonVehicleBodyFace
    {
        std::array<FirstPersonVec3, 4> local{};
        std::array<std::array<float, 2>, 4> uv{};
        ImageIndex image{};
        bool textured = false;
        bool top = false;
    };

    template<typename SpriteLookup>
    [[nodiscard]] inline std::vector<FirstPersonVehicleBodyFace>
        BuildFirstPersonVehicleBodyFaces(
            const FirstPersonVehicleBodyHull& hull, SpriteLookup&& lookup)
    {
        struct Source
        {
            ImageIndex image{};
            const G1Element* g1{};
            uint8_t direction{};
            float forward{}, right{};
        };
        std::vector<Source> sources;
        sources.reserve(hull.textureViews.size());
        constexpr float twoPi = 6.28318530717958647692f;
        for (const auto& view : hull.textureViews)
        {
            const auto* g1 = lookup(view.image);
            if (g1 == nullptr || g1->width <= 0 || g1->height <= 0)
                continue;
            const float theta = float(view.imageDirection & 31u) * (twoPi / 32.0f);
            const float c = std::cos(theta), s = std::sin(theta);
            sources.push_back({ view.image, g1, view.imageDirection,
                -c + s, -s - c });
        }

        std::vector<FirstPersonVehicleBodyFace> faces;
        if (!hull.valid)
            return faces;
        const float half = hull.step * 0.5f;
        const auto append = [&](const std::array<FirstPersonVec3, 4>& local,
                                FirstPersonVec3 normal) {
            FirstPersonVehicleBodyFace face{};
            face.local = local;
            face.top = normal.z > 0.5f;
            if (normal.z >= -0.5f)
            {
                const FirstPersonVec3 centre{
                    0.25f * (local[0].x + local[1].x + local[2].x + local[3].x),
                    0.25f * (local[0].y + local[1].y + local[2].y + local[3].y),
                    0.25f * (local[0].z + local[1].z + local[2].z + local[3].z)
                };
                float bestScore = -std::numeric_limits<float>::infinity();
                for (const auto& source : sources)
                {
                    const float horizontal = normal.x * source.forward
                        + normal.y * source.right;
                    const float facing = face.top ? 1.0f : horizontal;
                    if (facing <= 0.05f)
                        continue;
                    const auto centreUv = ProjectFirstPersonVehicleLocalPoint(
                        source.direction, centre);
                    const int32_t cx = int32_t(std::lround(centreUv[0])) - source.g1->xOffset;
                    const int32_t cy = int32_t(std::lround(centreUv[1])) - source.g1->yOffset;
                    bool supported = false;
                    for (int32_t dy = -1; dy <= 1 && !supported; ++dy)
                    for (int32_t dx = -1; dx <= 1; ++dx)
                    {
                        if (FirstPersonVehicleBodyPixelOpaque(
                                *source.g1, cx + dx, cy + dy))
                        {
                            supported = true;
                            break;
                        }
                    }
                    if (!supported)
                        continue;
                    std::array<std::array<float, 2>, 4> projected{};
                    for (size_t i = 0; i < 4; ++i)
                        projected[i] = ProjectFirstPersonVehicleLocalPoint(
                            source.direction, local[i]);
                    float twiceArea = 0.0f;
                    for (size_t i = 0; i < 4; ++i)
                    {
                        const size_t j = (i + 1) & 3u;
                        twiceArea += projected[i][0] * projected[j][1]
                            - projected[j][0] * projected[i][1];
                    }
                    const float score = facing * std::max(0.25f, 0.5f * std::abs(twiceArea));
                    if (score > bestScore)
                    {
                        bestScore = score;
                        face.textured = true;
                        face.image = source.image;
                        for (size_t i = 0; i < 4; ++i)
                        {
                            face.uv[i] = {
                                projected[i][0] - float(source.g1->xOffset),
                                projected[i][1] - float(source.g1->yOffset)
                            };
                        }
                    }
                }
            }
            faces.push_back(std::move(face));
        };

        for (int32_t up = 0; up < hull.sizeUp; ++up)
        for (int32_t right = 0; right < hull.sizeRight; ++right)
        for (int32_t forward = 0; forward < hull.sizeForward; ++forward)
        {
            if (!hull.contains(forward, right, up))
                continue;
            const auto centre = hull.centre(forward, right, up);
            const float f0 = centre.x - half, f1 = centre.x + half;
            const float r0 = centre.y - half, r1 = centre.y + half;
            const float u0 = centre.z - half, u1 = centre.z + half;
            if (!hull.contains(forward - 1, right, up))
                append({{{f0,r1,u0},{f0,r0,u0},{f0,r0,u1},{f0,r1,u1}}}, {-1,0,0});
            if (!hull.contains(forward + 1, right, up))
                append({{{f1,r0,u0},{f1,r1,u0},{f1,r1,u1},{f1,r0,u1}}}, {1,0,0});
            if (!hull.contains(forward, right - 1, up))
                append({{{f0,r0,u0},{f1,r0,u0},{f1,r0,u1},{f0,r0,u1}}}, {0,-1,0});
            if (!hull.contains(forward, right + 1, up))
                append({{{f1,r1,u0},{f0,r1,u0},{f0,r1,u1},{f1,r1,u1}}}, {0,1,0});
            if (!hull.contains(forward, right, up - 1))
                append({{{f0,r0,u0},{f0,r1,u0},{f1,r1,u0},{f1,r0,u0}}}, {0,0,-1});
            if (!hull.contains(forward, right, up + 1))
                append({{{f0,r1,u1},{f0,r0,u1},{f1,r0,u1},{f1,r1,u1}}}, {0,0,1});
        }
        return faces;
    }

    [[nodiscard]] inline FirstPersonVehicleBodyHull BuildFirstPersonVehicleBodyHull(const CarEntry& entry)
    {
        return BuildFirstPersonVehicleBodyHullFromSnapshot(entry, [](ImageIndex image) { return GfxGetG1Element(image); });
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
