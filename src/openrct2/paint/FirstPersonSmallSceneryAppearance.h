/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonVisualHull.h"

#include "../object/SmallSceneryEntry.h"
#include "../world/Location.hpp"
#include "../world/tile_element/SmallSceneryElement.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace OpenRCT2::Paint
{
    enum class FirstPersonSmallSceneryAppearanceKind : uint8_t
    {
        reconstructedHull,
        crossedTree,
    };

    struct FirstPersonSmallSceneryFootprint
    {
        bool valid = false;
        float minForward{};
        float maxForward{};
        float minRight{};
        float maxRight{};
        float minUp{};
        float maxUp{};

        [[nodiscard]] float centreForward() const
        {
            return (minForward + maxForward) * 0.5f;
        }

        [[nodiscard]] float centreRight() const
        {
            return (minRight + maxRight) * 0.5f;
        }
    };

    struct FirstPersonSmallSceneryAppearancePlan
    {
        FirstPersonSmallSceneryAppearanceKind kind =
            FirstPersonSmallSceneryAppearanceKind::reconstructedHull;
        std::vector<FirstPersonVisualHullFace> faces;
        // Reconstructed solids use native-view physical ownership to decide
        // which boundary receives each observed texel. Crossed tree planes are
        // authored appearance carriers: every arm receives the complete
        // directional tree artwork rather than partitioning one sprite between
        // intersecting planes.
        bool usesDepthOwnership = true;
        // Only a reconstructed closed hull can receive a material-closure
        // certificate. Crossed artwork is deliberately open geometry.
        bool certifiesHullMaterial = true;
        // Crossed planes carry separate front/back directional materials on
        // coincident geometry. The renderer's existing outward-half-space rule
        // makes exactly one side contribute from any camera position.
        bool oneSidedArtwork = false;
        // Solid reconstruction may combine compatible observations to cover a
        // physical boundary. A directional tree plane instead preserves one
        // native sprite's transparent silhouette exactly.
        bool mergesMaterialViews = true;

        [[nodiscard]] bool valid() const
        {
            return !faces.empty();
        }
    };

    [[nodiscard]] inline FirstPersonSmallSceneryAppearanceKind
        FirstPersonSmallSceneryAppearanceFor(
            const SmallSceneryEntry& entry)
    {
        return entry.flags.has(SmallSceneryFlag::isTree)
            ? FirstPersonSmallSceneryAppearanceKind::crossedTree
            : FirstPersonSmallSceneryAppearanceKind::reconstructedHull;
    }

    [[nodiscard]] inline bool
        FirstPersonSmallSceneryUsesCrossedTreeAppearance(
            const SmallSceneryEntry& entry)
    {
        return FirstPersonSmallSceneryAppearanceFor(entry)
            == FirstPersonSmallSceneryAppearanceKind::crossedTree;
    }

    [[nodiscard]] inline FirstPersonSmallSceneryFootprint
        BuildFirstPersonSmallSceneryFootprint(
            uint8_t occupiedQuadrants, int32_t height)
    {
        FirstPersonSmallSceneryFootprint result{};
        const uint8_t occupied = occupiedQuadrants & 0x0Fu;
        if (occupied == 0 || height <= 0)
            return result;

        float minForward = float(kCoordsXYStep);
        float minRight = float(kCoordsXYStep);
        float maxForward = 0.0f;
        float maxRight = 0.0f;
        const auto include =
            [&](float x0, float y0, float x1, float y1) {
                minForward = std::min(minForward, x0);
                minRight = std::min(minRight, y0);
                maxForward = std::max(maxForward, x1);
                maxRight = std::max(maxRight, y1);
            };

        const float half = float(kCoordsXYHalfTile);
        const float full = float(kCoordsXYStep);
        if ((occupied & (1u << 0)) != 0)
            include(half, half, full, full);
        if ((occupied & (1u << 1)) != 0)
            include(half, 0.0f, full, half);
        if ((occupied & (1u << 2)) != 0)
            include(0.0f, 0.0f, half, half);
        if ((occupied & (1u << 3)) != 0)
            include(0.0f, half, half, full);

        if (!(maxForward > minForward)
            || !(maxRight > minRight))
            return {};

        result.valid = true;
        result.minForward = minForward;
        result.maxForward = maxForward;
        result.minRight = minRight;
        result.maxRight = maxRight;
        result.minUp = 0.0f;
        result.maxUp = float(height);
        return result;
    }

    [[nodiscard]] inline FirstPersonSmallSceneryFootprint
        BuildFirstPersonSmallSceneryFootprint(
            const SmallSceneryEntry& entry,
            const SmallSceneryElement& element)
    {
        return BuildFirstPersonSmallSceneryFootprint(
            element.getOccupiedQuadrants(), entry.height);
    }

    [[nodiscard]] inline std::vector<FirstPersonVisualHullFace>
        BuildFirstPersonCrossedTreeFaces(
            const FirstPersonSmallSceneryFootprint& footprint)
    {
        std::vector<FirstPersonVisualHullFace> result;
        if (!footprint.valid)
            return result;

        const float cx = footprint.centreForward();
        const float cy = footprint.centreRight();
        const float x0 = footprint.minForward;
        const float x1 = footprint.maxForward;
        const float y0 = footprint.minRight;
        const float y1 = footprint.maxRight;
        const float z0 = footprint.minUp;
        const float z1 = footprint.maxUp;

        result.reserve(4);
        // X-centred arm, visible from -X.
        result.push_back({ { {
            { cx, y1, z0 }, { cx, y0, z0 },
            { cx, y0, z1 }, { cx, y1, z1 },
        } }, { -1, 0, 0 }, FirstPersonVisualHullFaceKind::minForward });
        // Same physical arm, opposite native material, visible from +X.
        result.push_back({ { {
            { cx, y0, z0 }, { cx, y1, z0 },
            { cx, y1, z1 }, { cx, y0, z1 },
        } }, { 1, 0, 0 }, FirstPersonVisualHullFaceKind::maxForward });
        // Y-centred arm, visible from -Y.
        result.push_back({ { {
            { x0, cy, z0 }, { x1, cy, z0 },
            { x1, cy, z1 }, { x0, cy, z1 },
        } }, { 0, -1, 0 }, FirstPersonVisualHullFaceKind::minRight });
        // Same physical arm, opposite native material, visible from +Y.
        result.push_back({ { {
            { x1, cy, z0 }, { x0, cy, z0 },
            { x0, cy, z1 }, { x1, cy, z1 },
        } }, { 0, 1, 0 }, FirstPersonVisualHullFaceKind::maxRight });
        return result;
    }

    struct FirstPersonTreeArtworkGrounding
    {
        bool valid = false;
        int32_t contactU = 0;
        int32_t contactV = 0;
        int32_t shiftU = 0;
        int32_t shiftV = 0;
    };

    [[nodiscard]] inline FirstPersonTreeArtworkGrounding
        FindFirstPersonTreeArtworkGrounding(
            std::span<const uint8_t> pixels,
            int32_t width, int32_t height,
            int32_t nominalGroundU,
            int32_t nominalGroundV,
            bool hasTransparency)
    {
        FirstPersonTreeArtworkGrounding result{};
        if (width <= 0 || height <= 0
            || pixels.size()
                < size_t(width) * size_t(height))
            return result;

        // The native sprite's lowest painted row is direct evidence of where
        // that directional artwork meets the ground. Crossed tree planes all
        // share one physical z=0; calibrating each source view to its own
        // contact row prevents G1 offsets/directional framing from making the
        // four sides appear to start at different heights.
        int32_t lowestOpaqueRow = -1;
        int64_t contactUSum = 0;
        int32_t contactUCount = 0;
        for (int32_t y = height - 1;
             y >= 0 && lowestOpaqueRow < 0; --y)
        {
            for (int32_t x = 0; x < width; ++x)
            {
                const uint8_t pixel =
                    pixels[size_t(y) * size_t(width)
                        + size_t(x)];
                if (!hasTransparency || pixel != 0)
                {
                    lowestOpaqueRow = y;
                    contactUSum += x;
                    ++contactUCount;
                }
            }
        }
        if (lowestOpaqueRow < 0
            || contactUCount == 0)
            return result;

        result.valid = true;
        result.contactU =
            int32_t(std::lround(
                double(contactUSum)
                / double(contactUCount)));
        result.contactV = lowestOpaqueRow;
        result.shiftU =
            result.contactU - nominalGroundU;
        result.shiftV =
            lowestOpaqueRow - nominalGroundV;
        return result;
    }

    [[nodiscard]] inline FirstPersonSmallSceneryAppearancePlan
        BuildFirstPersonSmallSceneryAppearancePlan(
            const SmallSceneryEntry& entry,
            const SmallSceneryElement& element,
            const FirstPersonVisualHull* reconstructedHull)
    {
        FirstPersonSmallSceneryAppearancePlan result{};
        result.kind = FirstPersonSmallSceneryAppearanceFor(entry);
        if (result.kind
            == FirstPersonSmallSceneryAppearanceKind::crossedTree)
        {
            result.faces = BuildFirstPersonCrossedTreeFaces(
                BuildFirstPersonSmallSceneryFootprint(entry, element));
            result.usesDepthOwnership = false;
            result.certifiesHullMaterial = false;
            result.oneSidedArtwork = true;
            result.mergesMaterialViews = false;
            return result;
        }

        if (reconstructedHull == nullptr
            || !reconstructedHull->valid)
            return result;
        result.faces =
            BuildFirstPersonVisualHullBoundaryFaces(
                *reconstructedHull);
        return result;
    }

    [[nodiscard]] inline std::vector<FirstPersonVisualHullTextureView>
        BuildFirstPersonSmallSceneryTextureViews(
            const SmallSceneryEntry& entry,
            const SmallSceneryElement& element,
            uint8_t witherStage)
    {
        std::vector<FirstPersonVisualHullTextureView> result;
        result.reserve(4);
        for (uint8_t rotation = 0; rotation < 4; ++rotation)
        {
            const uint8_t direction =
                element.getDirectionWithOffset(rotation) & 3u;
            result.push_back({
                rotation,
                entry.image + direction
                    + uint32_t(witherStage) * 4u,
            });
        }
        return result;
    }
} // namespace OpenRCT2::Paint
