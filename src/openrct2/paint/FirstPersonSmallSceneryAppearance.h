/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonSmallSceneryCollision.h"
#include "FirstPersonVisualHull.h"

#include "../object/SmallSceneryEntry.h"
#include "../world/Location.hpp"
#include "../world/tile_element/SmallSceneryElement.h"

#include <algorithm>
#include <array>
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

    struct FirstPersonTreeArtworkBounds
    {
        bool valid = false;
        int32_t minU = 0;
        int32_t minV = 0;
        int32_t maxU = 0;
        int32_t maxV = 0;
    };

    [[nodiscard]] inline FirstPersonTreeArtworkBounds
        FindFirstPersonTreeArtworkBounds(
            std::span<const uint8_t> pixels,
            int32_t width, int32_t height,
            bool hasTransparency)
    {
        FirstPersonTreeArtworkBounds result{};
        if (width <= 0 || height <= 0
            || pixels.size()
                < size_t(width) * size_t(height))
            return result;

        result.minU = width;
        result.minV = height;
        result.maxU = -1;
        result.maxV = -1;
        for (int32_t y = 0; y < height; ++y)
        for (int32_t x = 0; x < width; ++x)
        {
            const uint8_t pixel =
                pixels[size_t(y) * size_t(width)
                    + size_t(x)];
            if (hasTransparency && pixel == 0)
                continue;
            result.minU = std::min(result.minU, x);
            result.minV = std::min(result.minV, y);
            result.maxU = std::max(result.maxU, x);
            result.maxV = std::max(result.maxV, y);
        }
        result.valid =
            result.maxU >= result.minU
            && result.maxV >= result.minV;
        return result;
    }

    struct FirstPersonTreeArtworkMapping
    {
        bool valid = false;
        float groundU = 0.0f;
        float groundV = 0.0f;
        float alongUPerUnit = 0.0f;
        float upVPerUnit = 0.0f;

        [[nodiscard]] std::array<float, 2> sourcePixel(
            float along, float up) const
        {
            return {
                groundU + along * alongUPerUnit,
                groundV + up * upVPerUnit,
            };
        }
    };

    [[nodiscard]] inline FirstPersonTreeArtworkMapping
        BuildFirstPersonTreeArtworkMapping(
            float groundU, float groundV,
            float alongUPerUnit, float upVPerUnit)
    {
        FirstPersonTreeArtworkMapping result{};
        if (!std::isfinite(groundU)
            || !std::isfinite(groundV)
            || !std::isfinite(alongUPerUnit)
            || !std::isfinite(upVPerUnit)
            || std::abs(alongUPerUnit) < 1.0e-5f
            || std::abs(upVPerUnit) < 1.0e-5f)
            return result;

        result.valid = true;
        result.groundU = groundU;
        result.groundV = groundV;
        result.alongUPerUnit = alongUPerUnit;
        result.upVPerUnit = upVPerUnit;
        return result;
    }

    struct FirstPersonTreePlaneExtent
    {
        bool valid = false;
        float minAlong = 0.0f;
        float maxAlong = 0.0f;
        float maxUp = 0.0f;

        void include(
            const FirstPersonTreeArtworkMapping& mapping,
            const FirstPersonTreeArtworkBounds& bounds)
        {
            if (!mapping.valid || !bounds.valid)
                return;

            const float along0 =
                (float(bounds.minU) - mapping.groundU)
                / mapping.alongUPerUnit;
            const float along1 =
                (float(bounds.maxU + 1) - mapping.groundU)
                / mapping.alongUPerUnit;
            const float up0 =
                (float(bounds.minV) - mapping.groundV)
                / mapping.upVPerUnit;
            const float up1 =
                (float(bounds.maxV + 1) - mapping.groundV)
                / mapping.upVPerUnit;
            if (!std::isfinite(along0)
                || !std::isfinite(along1)
                || !std::isfinite(up0)
                || !std::isfinite(up1))
                return;

            const float localMinAlong =
                std::min(along0, along1);
            const float localMaxAlong =
                std::max(along0, along1);
            const float localMaxUp =
                std::max(
                    0.0f,
                    std::max(up0, up1));
            if (!valid)
            {
                minAlong = localMinAlong;
                maxAlong = localMaxAlong;
                maxUp = localMaxUp;
                valid = true;
                return;
            }
            minAlong = std::min(
                minAlong, localMinAlong);
            maxAlong = std::max(
                maxAlong, localMaxAlong);
            maxUp = std::max(
                maxUp, localMaxUp);
        }
    };

    [[nodiscard]] inline std::vector<FirstPersonVisualHullFace>
        BuildFirstPersonCrossedTreeFaces(
            const FirstPersonSmallSceneryFootprint& footprint,
            const FirstPersonTreePlaneExtent* xCentredExtent,
            const FirstPersonTreePlaneExtent* yCentredExtent)
    {
        std::vector<FirstPersonVisualHullFace> result;
        if (!footprint.valid)
            return result;

        const float cx = footprint.centreForward();
        const float cy = footprint.centreRight();
        const float x0 =
            yCentredExtent != nullptr
                && yCentredExtent->valid
            ? cx + yCentredExtent->minAlong
            : footprint.minForward;
        const float x1 =
            yCentredExtent != nullptr
                && yCentredExtent->valid
            ? cx + yCentredExtent->maxAlong
            : footprint.maxForward;
        const float y0 =
            xCentredExtent != nullptr
                && xCentredExtent->valid
            ? cy + xCentredExtent->minAlong
            : footprint.minRight;
        const float y1 =
            xCentredExtent != nullptr
                && xCentredExtent->valid
            ? cy + xCentredExtent->maxAlong
            : footprint.maxRight;
        const float z0 = footprint.minUp;
        const float xTop =
            xCentredExtent != nullptr
                && xCentredExtent->valid
            ? z0 + xCentredExtent->maxUp
            : footprint.maxUp;
        const float yTop =
            yCentredExtent != nullptr
                && yCentredExtent->valid
            ? z0 + yCentredExtent->maxUp
            : footprint.maxUp;
        const float z1 =
            std::max(
                footprint.maxUp,
                std::max(xTop, yTop));

        result.reserve(4);
        result.push_back({ { {
            { cx, y1, z0 }, { cx, y0, z0 },
            { cx, y0, z1 }, { cx, y1, z1 },
        } }, { -1, 0, 0 }, FirstPersonVisualHullFaceKind::minForward });
        result.push_back({ { {
            { cx, y0, z0 }, { cx, y1, z0 },
            { cx, y1, z1 }, { cx, y0, z1 },
        } }, { 1, 0, 0 }, FirstPersonVisualHullFaceKind::maxForward });
        result.push_back({ { {
            { x0, cy, z0 }, { x1, cy, z0 },
            { x1, cy, z1 }, { x0, cy, z1 },
        } }, { 0, -1, 0 }, FirstPersonVisualHullFaceKind::minRight });
        result.push_back({ { {
            { x1, cy, z0 }, { x0, cy, z0 },
            { x0, cy, z1 }, { x1, cy, z1 },
        } }, { 0, 1, 0 }, FirstPersonVisualHullFaceKind::maxRight });
        return result;
    }

    [[nodiscard]] inline std::vector<FirstPersonVisualHullFace>
        BuildFirstPersonCrossedTreeFaces(
            const FirstPersonSmallSceneryFootprint& footprint)
    {
        return BuildFirstPersonCrossedTreeFaces(
            footprint, nullptr, nullptr);
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

        // The native object's occupied footprint and height are physical
        // evidence for collision, NOT evidence that all six sides of that
        // envelope are visible matter. Do not bake an unverified collision
        // envelope as the object's rendered shape. Stable small scenery is
        // eligible for silhouette carving; if its four-view certificate
        // fails, let the existing renderer use native artwork as a last
        // resort, after the asynchronous reconstruction attempt completes.
        // Stateful/animated/glass objects retain their separate policy.
        if (FirstPersonSmallSceneryCanUseSilhouetteRefinement(entry)
            && reconstructedHull->roundTripViewCount < 4)
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
