/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonMath.h"

#include "../Identifiers.h"
#include "../ride/TrackData.h"
#include "../ride/ted/TrackElementDescriptor.h"
#include "../world/Footpath.h"
#include "../world/Map.h"
#include "../world/TileElementsView.h"
#include "../world/tile_element/EntranceElement.h"
#include "../world/tile_element/PathElement.h"
#include "../world/tile_element/Slope.h"
#include "../world/tile_element/TrackElement.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

namespace OpenRCT2::Paint
{
    enum class FirstPersonWalkabilityKind : uint8_t
    {
        none,
        path,
        queue,
        parkEntrance,
        rideEntrance,
        rideExit,
        trackPortal,
    };

    enum class FirstPersonPassageAxis : uint8_t
    {
        none,
        x,
        y,
    };

    // This is deliberately navigation semantics rather than render geometry.
    // It describes where the game says traversal is possible. Render/collision
    // reconstruction may use it as a negative constraint ("solid geometry must
    // not close this corridor"), but it must never be treated as evidence for
    // decorative geometry outside that corridor.
    struct FirstPersonWalkabilityConstraint
    {
        FirstPersonWalkabilityKind kind =
            FirstPersonWalkabilityKind::none;
        CoordsXY tile{};
        int32_t baseZ = 0;
        uint8_t connectedSides = 0;
        uint8_t corners = 0;
        uint8_t slopeDirection = 0;
        bool sloped = false;
        bool wide = false;
        bool walkableFloor = false;
        bool guaranteedThroughPassage = false;
        bool visualDeckShouldReachTileEdges = false;

        // Path identity is retained so stacked/replaced path elements do not
        // silently become one another while the first-person walker moves.
        ObjectEntryIndex surface{};
        ObjectEntryIndex railings{};
    };

    inline constexpr float kFirstPersonPathDeckInset = 8.0f;
    inline constexpr float kFirstPersonPathDeckMax =
        float(kCoordsXYStep) - kFirstPersonPathDeckInset;

    [[nodiscard]] constexpr bool FirstPersonDoorBlocksWalking(
        uint8_t animationFrame)
    {
        // Native MapAnimation keeps frame 5 as the stable open state and
        // returns closing frame 15 to stable closed frame 0. Transitional
        // frames remain conservatively blocking until their panel aperture is
        // calibrated from the door artwork.
        return animationFrame != 5;
    }

    [[nodiscard]] constexpr uint8_t
        RotateFirstPersonConnectionMask(
            uint8_t mask, uint8_t quarterTurns)
    {
        mask &= 0x0F;
        quarterTurns &= 3;
        if (quarterTurns == 0)
            return mask;
        return uint8_t(
            ((uint32_t(mask) << quarterTurns)
             | (uint32_t(mask) >> (4u - quarterTurns)))
            & 0x0Fu);
    }

    [[nodiscard]] constexpr FirstPersonPassageAxis
        FirstPersonPassageAxisForConnections(
            uint8_t connectedSides)
    {
        connectedSides &= 0x0F;
        if ((connectedSides & 0x05u) == 0x05u)
            return FirstPersonPassageAxis::x;
        if ((connectedSides & 0x0Au) == 0x0Au)
            return FirstPersonPassageAxis::y;
        return FirstPersonPassageAxis::none;
    }

    [[nodiscard]] constexpr uint8_t
        FirstPersonPathLandSlope(uint8_t direction)
    {
        switch (direction & 3u)
        {
            case 0:
                return kTileSlopeSWSideUp;
            case 1:
                return kTileSlopeNWSideUp;
            case 2:
                return kTileSlopeNESideUp;
            default:
                return kTileSlopeSESideUp;
        }
    }

    [[nodiscard]] inline bool
        FirstPersonPathUsesFullTileDeck(
            const PathElement& path)
    {
        // Wide-path routing may deliberately omit logical edges inside a broad
        // plaza. Those routing edges are not visual holes. Keep this semantic
        // narrow: queues and slopes still use their native edge/slope shape.
        return path.isWide()
            && !path.isSloped()
            && !path.isQueue();
    }

    [[nodiscard]] inline FirstPersonWalkabilityConstraint
        FirstPersonWalkabilityFromPath(
            CoordsXY tile, const PathElement& path)
    {
        FirstPersonWalkabilityConstraint result{};
        result.kind = path.isQueue()
            ? FirstPersonWalkabilityKind::queue
            : FirstPersonWalkabilityKind::path;
        result.tile = tile.toTileStart();
        result.baseZ = path.getBaseZ();
        result.connectedSides =
            path.getEdges() & 0x0F;
        result.corners =
            path.isQueue()
            ? 0 : (path.getCorners() & 0x0F);
        result.sloped = path.isSloped();
        result.slopeDirection =
            static_cast<uint8_t>(
                path.getSlopeDirection()) & 3u;
        result.wide = path.isWide();
        result.walkableFloor = true;
        result.visualDeckShouldReachTileEdges =
            FirstPersonPathUsesFullTileDeck(path);
        result.surface = path.getSurfaceEntryIndex();
        result.railings = path.getRailingsEntryIndex();
        return result;
    }

    [[nodiscard]] inline FirstPersonWalkabilityConstraint
        FirstPersonWalkabilityFromEntrance(
            CoordsXY tile,
            const EntranceElement& entrance)
    {
        FirstPersonWalkabilityConstraint result{};
        switch (entrance.getEntranceType())
        {
            case EntranceType::parkEntrance:
                result.kind =
                    FirstPersonWalkabilityKind::
                        parkEntrance;
                break;
            case EntranceType::rideEntrance:
                result.kind =
                    FirstPersonWalkabilityKind::
                        rideEntrance;
                break;
            case EntranceType::rideExit:
                result.kind =
                    FirstPersonWalkabilityKind::
                        rideExit;
                break;
        }

        result.tile = tile.toTileStart();
        result.baseZ = entrance.getBaseZ();
        result.connectedSides =
            RotateFirstPersonConnectionMask(
                uint8_t(entrance.getDirections()),
                static_cast<uint8_t>(
                    entrance.getDirection()));
        // Only the centre park-entrance sequence owns the two-sided route.
        // Ride entrances/exits remain graph endpoints, not through-corridors.
        result.guaranteedThroughPassage =
            result.kind
                == FirstPersonWalkabilityKind::
                    parkEntrance
            && entrance.getSequenceIndex()
                == ParkEntranceSequence::centre
            && FirstPersonPassageAxisForConnections(
                    result.connectedSides)
                != FirstPersonPassageAxis::none;
        result.walkableFloor =
            result.guaranteedThroughPassage;
        result.visualDeckShouldReachTileEdges =
            result.guaranteedThroughPassage;
        if (entrance.getPathSurfaceDescriptor()
            != nullptr)
        {
            result.surface =
                entrance.getSurfaceEntryIndex();
        }
        return result;
    }

    [[nodiscard]] inline std::optional<
        FirstPersonWalkabilityConstraint>
        FirstPersonWalkabilityFromTrackPortal(
            CoordsXY tile, const TrackElement& track)
    {
        const auto& ted =
            TrackMetadata::GetTrackElementDescriptor(
                track.getTrackType());
        const size_t sequence =
            track.getSequenceIndex();
        if (sequence
                >= ted.sequenceData.numSequences
            || sequence
                >= ted.sequenceData.sequences.size())
            return std::nullopt;
        const auto& sequenceData =
            ted.sequenceData.sequences[sequence];
        if (!sequenceData.flags.has(
                TrackMetadata::SequenceFlag::
                    connectsToPath))
            return std::nullopt;

        const uint8_t localSides =
            uint8_t(
                sequenceData
                    .getEntranceConnectionSides())
            & 0x0F;
        if (localSides == 0)
            return std::nullopt;

        FirstPersonWalkabilityConstraint result{};
        result.kind =
            FirstPersonWalkabilityKind::trackPortal;
        result.tile = tile.toTileStart();
        result.baseZ = track.getBaseZ();
        result.connectedSides =
            RotateFirstPersonConnectionMask(
                localSides,
                static_cast<uint8_t>(
                    track.getDirection()));
        // This is connectivity evidence only. Track geometry is not a walking
        // floor and is never carved merely because it exposes a path portal.
        return result;
    }

    [[nodiscard]] inline std::vector<
        FirstPersonWalkabilityConstraint>
        CollectFirstPersonWalkabilityConstraints(
            CoordsXY tile)
    {
        std::vector<
            FirstPersonWalkabilityConstraint> result;
        tile = tile.toTileStart();
        if (!MapIsLocationValid(tile))
            return result;

        for (const auto* path :
             TileElementsView<PathElement>(tile))
        {
            if (path == nullptr
                || path->isGhost()
                || path->isInvisible())
                continue;
            result.emplace_back(
                FirstPersonWalkabilityFromPath(
                    tile, *path));
        }

        for (const auto* entrance :
             TileElementsView<EntranceElement>(tile))
        {
            if (entrance == nullptr
                || entrance->isGhost()
                || entrance->isInvisible())
                continue;
            const auto constraint =
                FirstPersonWalkabilityFromEntrance(
                    tile, *entrance);
            if (constraint.connectedSides != 0)
            {
                result.emplace_back(
                    constraint);
            }
        }

        for (const auto* track :
             TileElementsView<TrackElement>(tile))
        {
            if (track == nullptr
                || track->isGhost()
                || track->isInvisible())
                continue;
            const auto constraint =
                FirstPersonWalkabilityFromTrackPortal(
                    tile, *track);
            if (constraint.has_value())
            {
                result.emplace_back(
                    *constraint);
            }
        }
        return result;
    }

    [[nodiscard]] inline bool
        FirstPersonWalkabilityContainsPoint(
            const FirstPersonWalkabilityConstraint&
                constraint,
            const CoordsXY& position,
            float inset = 0.0f)
    {
        const int32_t localXi =
            position.x - constraint.tile.x;
        const int32_t localYi =
            position.y - constraint.tile.y;
        const float x = float(localXi);
        const float y = float(localYi);
        const float tileMin = inset;
        const float tileMax =
            float(kCoordsXYStep) - inset;
        if (x < tileMin || x > tileMax
            || y < tileMin || y > tileMax)
            return false;

        if (constraint.visualDeckShouldReachTileEdges
            && (constraint.kind
                    == FirstPersonWalkabilityKind::path
                || constraint.kind
                    == FirstPersonWalkabilityKind::queue))
        {
            return true;
        }

        if (constraint.kind
                == FirstPersonWalkabilityKind::path
            || constraint.kind
                == FirstPersonWalkabilityKind::queue)
        {
            const float low =
                kFirstPersonPathDeckInset + inset;
            const float high =
                kFirstPersonPathDeckMax - inset;
            if (low > high)
                return false;

            if (x >= low && x <= high
                && y >= low && y <= high)
                return true;

            const auto has =
                [&](uint8_t direction) {
                    return (constraint.connectedSides
                            & (1u << direction))
                        != 0;
                };
            if (has(0)
                && x <= float(kCoordsXYHalfTile)
                && y >= low && y <= high)
                return true;
            if (has(1)
                && y >= float(kCoordsXYHalfTile)
                && x >= low && x <= high)
                return true;
            if (has(2)
                && x >= float(kCoordsXYHalfTile)
                && y >= low && y <= high)
                return true;
            if (has(3)
                && y <= float(kCoordsXYHalfTile)
                && x >= low && x <= high)
                return true;

            if (constraint.kind
                == FirstPersonWalkabilityKind::queue)
                return false;

            if ((constraint.corners & (1u << 0)) != 0
                && x <= float(kCoordsXYHalfTile)
                && y >= float(kCoordsXYHalfTile))
                return true;
            if ((constraint.corners & (1u << 1)) != 0
                && x >= float(kCoordsXYHalfTile)
                && y >= float(kCoordsXYHalfTile))
                return true;
            if ((constraint.corners & (1u << 2)) != 0
                && x >= float(kCoordsXYHalfTile)
                && y <= float(kCoordsXYHalfTile))
                return true;
            if ((constraint.corners & (1u << 3)) != 0
                && x <= float(kCoordsXYHalfTile)
                && y <= float(kCoordsXYHalfTile))
                return true;
            return false;
        }

        if (constraint.guaranteedThroughPassage)
        {
            const float low =
                kFirstPersonPathDeckInset + inset;
            const float high =
                kFirstPersonPathDeckMax - inset;
            if (low > high)
                return false;

            switch (
                FirstPersonPassageAxisForConnections(
                    constraint.connectedSides))
            {
                case FirstPersonPassageAxis::x:
                    return y >= low && y <= high;
                case FirstPersonPassageAxis::y:
                    return x >= low && x <= high;
                case FirstPersonPassageAxis::none:
                    break;
            }
        }
        return false;
    }

    [[nodiscard]] inline float
        FirstPersonWalkabilityHeightAt(
            const FirstPersonWalkabilityConstraint&
                constraint,
            const CoordsXY& position)
    {
        if (!constraint.sloped)
            return float(constraint.baseZ);

        const auto slopeCorners =
            GetSlopeCornerHeights(
                constraint.baseZ,
                FirstPersonPathLandSlope(
                    constraint.slopeDirection));
        const float localX = float(
            std::clamp(
                position.x - constraint.tile.x,
                0, kCoordsXYStep));
        const float localY = float(
            std::clamp(
                position.y - constraint.tile.y,
                0, kCoordsXYStep));
        return FirstPersonPathHeight(
            float(slopeCorners.south),
            float(slopeCorners.east),
            float(slopeCorners.north),
            float(slopeCorners.west),
            localX, localY);
    }

    [[nodiscard]] inline bool
        SameFirstPersonWalkabilitySupport(
            const FirstPersonWalkabilityConstraint& a,
            const FirstPersonWalkabilityConstraint& b)
    {
        if (a.kind != b.kind
            || a.tile.x != b.tile.x
            || a.tile.y != b.tile.y
            || a.baseZ != b.baseZ)
            return false;

        if (a.kind
                == FirstPersonWalkabilityKind::path
            || a.kind
                == FirstPersonWalkabilityKind::queue)
        {
            return a.surface == b.surface
                && a.railings == b.railings;
        }
        return true;
    }

    [[nodiscard]] inline bool
        FirstPersonWalkabilitySupportsConnect(
            const FirstPersonWalkabilityConstraint& from,
            const FirstPersonWalkabilityConstraint& to)
    {
        if (SameFirstPersonWalkabilitySupport(
                from, to))
            return true;

        const int32_t dx =
            to.tile.x - from.tile.x;
        const int32_t dy =
            to.tile.y - from.tile.y;
        uint8_t direction = 0xFF;
        if (dx == -kCoordsXYStep && dy == 0)
            direction = 0;
        else if (dx == 0
            && dy == kCoordsXYStep)
            direction = 1;
        else if (dx == kCoordsXYStep
            && dy == 0)
            direction = 2;
        else if (dx == 0
            && dy == -kCoordsXYStep)
            direction = 3;
        if (direction >= 4)
            return false;

        const uint8_t reverse =
            (direction + 2u) & 3u;
        return (from.connectedSides
                    & (1u << direction))
                != 0
            && (to.connectedSides
                    & (1u << reverse))
                != 0;
    }

    [[nodiscard]] inline std::optional<
        FirstPersonWalkabilityConstraint>
        FindFirstPersonVisualDeckConstraint(
            CoordsXY tile, int32_t baseZ)
    {
        for (const auto& constraint :
             CollectFirstPersonWalkabilityConstraints(
                 tile))
        {
            if (!constraint
                    .visualDeckShouldReachTileEdges
                || constraint.baseZ != baseZ)
                continue;
            return constraint;
        }
        return std::nullopt;
    }

    [[nodiscard]] inline std::optional<
        FirstPersonWalkabilityConstraint>
        FindFirstPersonGuaranteedPassage(
            CoordsXY tile, int32_t baseZ)
    {
        for (const auto& constraint :
             CollectFirstPersonWalkabilityConstraints(
                 tile))
        {
            if (!constraint
                    .guaranteedThroughPassage
                || constraint.baseZ != baseZ)
                continue;
            return constraint;
        }
        return std::nullopt;
    }

    [[nodiscard]] inline bool
        FirstPersonWalkStepUsesGuaranteedPassage(
            CoordsXY tile,
            FirstPersonVec3 from,
            FirstPersonVec3 to,
            float radius = 2.0f)
    {
        const auto constraints =
            CollectFirstPersonWalkabilityConstraints(
                tile);
        if (constraints.empty())
            return false;

        const auto tileOrigin =
            tile.toTileStart();
        const CoordsXY fromXY{
            int32_t(std::lround(from.x)),
            int32_t(std::lround(from.y)),
        };
        const CoordsXY toXY{
            int32_t(std::lround(to.x)),
            int32_t(std::lround(to.y)),
        };
        const auto fromTile =
            fromXY.toTileStart();
        const auto toTile =
            toXY.toTileStart();
        const bool fromOnTile =
            fromTile.x == tileOrigin.x
            && fromTile.y == tileOrigin.y;
        const bool toOnTile =
            toTile.x == tileOrigin.x
            && toTile.y == tileOrigin.y;
        if (!fromOnTile && !toOnTile)
            return false;

        for (const auto& constraint :
             constraints)
        {
            if (!constraint
                    .guaranteedThroughPassage)
                continue;
            const float lowZ =
                float(constraint.baseZ) - 0.5f;
            const float highZ =
                float(constraint.baseZ
                    + kPathClearance) + 0.5f;
            if (std::min(from.z, to.z) < lowZ
                || std::max(from.z, to.z) > highZ)
                continue;

            if (fromOnTile
                && !FirstPersonWalkabilityContainsPoint(
                    constraint, fromXY, radius))
                continue;
            if (toOnTile
                && !FirstPersonWalkabilityContainsPoint(
                    constraint, toXY, radius))
                continue;
            return true;
        }
        return false;
    }

    [[nodiscard]] constexpr bool
        FirstPersonWalkingHeightTransitionAllowed(
            float fromZ, float toZ, float maximumStep)
    {
        const float delta =
            fromZ >= toZ
            ? fromZ - toZ
            : toZ - fromZ;
        return delta <= maximumStep;
    }
} // namespace OpenRCT2::Paint
