/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonSmallSceneryCollision.h"
#include "FirstPersonTrackTrajectory.h"

#include "../ride/CarEntry.h"
#include "../ride/Ride.h"
#include "../ride/RideData.h"
#include "../ride/RideEntry.h"
#include "../ride/Track.h"
#include "../ride/TrackStyle.h"
#include "../ride/ted/TrackElementDescriptor.h"
#include "../ride/ted/TrackElemType.h"
#include "../object/FootpathEntry.h"
#include "../object/PathAdditionEntry.h"
#include "../object/StationObject.h"
#include "../world/Map.h"
#include "../world/tile_element/PathElement.h"
#include "../world/tile_element/SmallSceneryElement.h"
#include "../world/tile_element/Slope.h"
#include "../world/tile_element/SurfaceElement.h"
#include "../world/tile_element/TrackElement.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

namespace OpenRCT2::Paint
{
    enum class FirstPersonPhysicalProxyProvenance : uint8_t
    {
        verifiedTrackArtwork,
        nativeStationGeometry,
        nativePathGeometry,
        calibratedSceneryArtwork,
    };

    enum class FirstPersonPhysicalProxyCapability : uint8_t
    {
        render = 1u << 0,
        collide = 1u << 1,
    };

    struct FirstPersonPhysicalBoxProxy
    {
        FirstPersonVec3 low{};
        FirstPersonVec3 high{};
        FirstPersonPhysicalProxyProvenance provenance =
            FirstPersonPhysicalProxyProvenance::nativePathGeometry;
        uint8_t capabilities =
            static_cast<uint8_t>(FirstPersonPhysicalProxyCapability::render)
            | static_cast<uint8_t>(FirstPersonPhysicalProxyCapability::collide);
        uint64_t sourceKey = 0;
    };

    inline void AppendFirstPersonPhysicalBoxProxy(
        std::vector<FirstPersonPhysicalBoxProxy>& result,
        CoordsXY tile, int32_t x, int32_t y, int32_t z,
        int32_t sizeX, int32_t sizeY, int32_t sizeZ,
        FirstPersonPhysicalProxyProvenance provenance,
        uint64_t sourceKey = 0)
    {
        if (sizeZ <= 0)
            return;
        float lowX = float(tile.x + x);
        float highX = float(tile.x + x + sizeX);
        float lowY = float(tile.y + y);
        float highY = float(tile.y + y + sizeY);
        if (sizeX == 0)
        {
            lowX -= 0.5f;
            highX += 0.5f;
        }
        if (sizeY == 0)
        {
            lowY -= 0.5f;
            highY += 0.5f;
        }
        if (highX < lowX)
            std::swap(lowX, highX);
        if (highY < lowY)
            std::swap(lowY, highY);
        result.push_back({
            { lowX, lowY, float(z) },
            { highX, highY, float(z + sizeZ) },
            provenance,
            static_cast<uint8_t>(
                FirstPersonPhysicalProxyCapability::render)
                | static_cast<uint8_t>(
                    FirstPersonPhysicalProxyCapability::collide),
            sourceKey,
        });
    }

    [[nodiscard]] inline bool AppendFirstPersonSmallSceneryProxies(
        std::vector<FirstPersonPhysicalBoxProxy>& result,
        CoordsXY tile, const SmallSceneryElement& small)
    {
        const auto* entry = small.getEntry();
        if (entry == nullptr)
            return false;
        const auto* mask =
            GetFirstPersonSmallSceneryWalkingMask(*entry, small);
        if (mask == nullptr || !mask->valid)
            return false;

        const size_t start = result.size();
        constexpr int32_t cellSize =
            FirstPersonSmallSceneryWalkingMask::kCellSize;
        constexpr int32_t cells =
            FirstPersonSmallSceneryWalkingMask::kCellsPerAxis;
        constexpr size_t kMaxProxyBoxes = 96;

        for (size_t layer = 0;
             layer < mask->layerCount; ++layer)
        {
            const int32_t z0 =
                small.getBaseZ() + mask->layerLowZ[layer];
            const int32_t z1 = std::min(
                small.getClearanceZ(),
                small.getBaseZ()
                    + mask->layerHighZ[layer]);
            if (z1 <= z0)
                continue;

            for (int32_t yCell = 0;
                 yCell < cells; ++yCell)
            {
                int32_t xCell = 0;
                while (xCell < cells)
                {
                    while (xCell < cells
                        && !mask->contains(
                            layer, xCell, yCell))
                        ++xCell;
                    if (xCell >= cells)
                        break;
                    const int32_t runStart = xCell;
                    while (xCell < cells
                        && mask->contains(
                            layer, xCell, yCell))
                        ++xCell;
                    AppendFirstPersonPhysicalBoxProxy(
                        result, tile,
                        runStart * cellSize,
                        yCell * cellSize,
                        z0,
                        (xCell - runStart) * cellSize,
                        cellSize,
                        z1 - z0,
                        FirstPersonPhysicalProxyProvenance::
                            calibratedSceneryArtwork,
                        uint64_t(entry->image));
                    if (result.size() - start
                        > kMaxProxyBoxes)
                    {
                        result.erase(
                            result.begin() + start,
                            result.end());
                        return false;
                    }
                }
            }
        }
        return result.size() > start;
    }

    [[nodiscard]] inline bool FirstPersonPathArtworkIsPhysical(
        const PathElement& path, ImageIndex image)
    {
        const auto* railings = path.getRailingsDescriptor();
        if (railings != nullptr
            && image >= railings->railingsImage
            && image < railings->railingsImage + 36)
            return true;

        const auto* addition = path.getAdditionEntry();
        if (addition == nullptr
            || addition->draw_type
                == PathAdditionDrawType::jumpingFountain)
            return false;
        // Native light/bin/bench state variants occupy at most twelve
        // directional images after the object base image.
        return image > addition->image
            && image <= addition->image + 12;
    }

    [[nodiscard]] inline bool FirstPersonPathHasSupports(
        CoordsXY tile, const PathElement& path)
    {
        const auto* surface = MapGetSurfaceElementAt(tile);
        if (surface == nullptr)
            return true;

        const int32_t height = path.getBaseZ();
        if (surface->getBaseZ() != height)
        {
            const auto* descriptor =
                path.getSurfaceDescriptor();
            const bool showUndergroundRailings =
                descriptor == nullptr
                || !(descriptor->flags
                    & FOOTPATH_ENTRY_FLAG_NO_SLOPE_RAILINGS);
            if (surface->getBaseZ() < height
                || showUndergroundRailings)
                return true;
        }
        else if (path.isSloped())
        {
            static constexpr std::array<uint8_t, 4>
                kPathSlopeToLandSlope{ {
                    kTileSlopeSWSideUp,
                    kTileSlopeNWSideUp,
                    kTileSlopeNESideUp,
                    kTileSlopeSESideUp,
                } };
            const uint8_t direction =
                path.getSlopeDirection() & 3u;
            if (surface->getSlope()
                != kPathSlopeToLandSlope[direction])
                return true;
        }
        else if (surface->getSlope() != kTileSlopeFlat)
        {
            return true;
        }
        return false;
    }

    inline void AppendFirstPersonPathRailingProxies(
        std::vector<FirstPersonPhysicalBoxProxy>& result,
        CoordsXY tile, const PathElement& path)
    {
        if (path.getRailingsDescriptor() == nullptr)
            return;

        const bool hasSupports =
            FirstPersonPathHasSupports(tile, path);
        const bool slopeRailingsSupported =
            path.getSurfaceDescriptor() == nullptr
            || !(path.getSurfaceDescriptor()->flags
                & FOOTPATH_ENTRY_FLAG_NO_SLOPE_RAILINGS);
        if (!path.isQueue() && !hasSupports
            && !(path.isSloped()
                && slopeRailingsSupported))
            return;

        const int32_t height = path.getBaseZ() + 2;
        const auto addHorizontal =
            [&](int32_t x, int32_t y, int32_t length,
                int32_t heightExtent) {
                AppendFirstPersonPhysicalBoxProxy(
                    result, tile, x, y, height,
                    length, 1, heightExtent,
                    FirstPersonPhysicalProxyProvenance::
                        nativePathGeometry);
            };
        const auto addVertical =
            [&](int32_t x, int32_t y, int32_t length,
                int32_t heightExtent) {
                AppendFirstPersonPhysicalBoxProxy(
                    result, tile, x, y, height,
                    1, length, heightExtent,
                    FirstPersonPhysicalProxyProvenance::
                        nativePathGeometry);
            };
        const auto addPost =
            [&](int32_t x, int32_t y) {
                AppendFirstPersonPhysicalBoxProxy(
                    result, tile, x, y, height,
                    4, 4, 7,
                    FirstPersonPhysicalProxyProvenance::
                        nativePathGeometry);
            };

        if (path.isSloped())
        {
            if ((path.getSlopeDirection() & 1u) == 0)
            {
                addHorizontal(0, 4, 32, 23);
                addHorizontal(0, 27, 32, 23);
            }
            else
            {
                addVertical(4, 0, 32, 23);
                addVertical(27, 0, 32, 23);
            }
            return;
        }

        // Geometry copied from the native flat railing painter at viewport
        // rotation zero. Queue and non-queue variants use different images,
        // but these authored physical strips are the same.
        const uint8_t edges = path.getEdges() & 0x0Fu;
        const auto* railings = path.getRailingsDescriptor();
        uint8_t drawnCorners = 0;
        if (!path.isQueue() && railings != nullptr
            && (railings->flags
                & RAILING_ENTRY_FLAG_DRAW_PATH_OVER_SUPPORTS))
        {
            drawnCorners = path.getCorners() & 0x0Fu;
        }
        const auto addCornerIfVisible =
            [&](uint8_t corner, int32_t x, int32_t y) {
                if (path.isQueue())
                {
                    addPost(x, y);
                }
                else if ((drawnCorners & (1u << corner)) == 0)
                {
                    addPost(x, y);
                }
            };
        switch (edges)
        {
            case 0:
                break;
            case 1:
            case 4:
                addHorizontal(0, 4, 27, 7);
                addHorizontal(0, 27, 27, 7);
                break;
            case 2:
            case 8:
                addVertical(4, 0, 27, 7);
                addVertical(27, 0, 27, 7);
                break;
            case 5:
                addHorizontal(0, 4, 32, 7);
                addHorizontal(0, 27, 32, 7);
                break;
            case 10:
                addVertical(4, 0, 32, 7);
                addVertical(27, 0, 32, 7);
                break;
            case 3:
                addHorizontal(0, 4, 26, 7);
                addVertical(27, 4, 27, 7);
                addCornerIfVisible(0, 0, 27);
                break;
            case 6:
                addVertical(4, 0, 27, 7);
                addHorizontal(0, 4, 27, 7);
                addCornerIfVisible(1, 27, 27);
                break;
            case 9:
                addVertical(27, 0, 27, 7);
                addHorizontal(0, 27, 27, 7);
                addCornerIfVisible(3, 0, 0);
                break;
            case 12:
                addVertical(4, 0, 26, 7);
                addHorizontal(4, 27, 27, 7);
                addCornerIfVisible(2, 27, 0);
                break;
            case 7:
                addHorizontal(0, 4, 32, 7);
                if (path.isQueue())
                {
                    if (path.hasJunctionRailings())
                    {
                        addPost(0, 27);
                        addPost(27, 27);
                    }
                }
                else
                {
                    addCornerIfVisible(0, 0, 27);
                    addCornerIfVisible(1, 27, 27);
                }
                break;
            case 13:
                addHorizontal(0, 27, 32, 7);
                if (path.isQueue())
                {
                    if (path.hasJunctionRailings())
                    {
                        addPost(27, 0);
                        addPost(0, 0);
                    }
                }
                else
                {
                    addCornerIfVisible(2, 27, 0);
                    addCornerIfVisible(3, 0, 0);
                }
                break;
            case 11:
                addVertical(27, 0, 32, 7);
                if (path.isQueue())
                {
                    if (path.hasJunctionRailings())
                    {
                        addPost(0, 0);
                        addPost(0, 27);
                    }
                }
                else
                {
                    addCornerIfVisible(0, 0, 27);
                    addCornerIfVisible(3, 0, 0);
                }
                break;
            case 14:
                addVertical(4, 0, 32, 7);
                if (path.isQueue())
                {
                    if (path.hasJunctionRailings())
                    {
                        addPost(27, 27);
                        addPost(27, 0);
                    }
                }
                else
                {
                    addCornerIfVisible(1, 27, 27);
                    addCornerIfVisible(2, 27, 0);
                }
                break;
            case 15:
                if (path.isQueue())
                {
                    if (path.hasJunctionRailings())
                    {
                        addPost(0, 27);
                        addPost(27, 27);
                        addPost(27, 0);
                        addPost(0, 0);
                    }
                }
                else
                {
                    addCornerIfVisible(0, 0, 27);
                    addCornerIfVisible(1, 27, 27);
                    addCornerIfVisible(2, 27, 0);
                    addCornerIfVisible(3, 0, 0);
                }
                break;
        }
    }

    inline void AppendFirstPersonPathFixtureProxies(
        std::vector<FirstPersonPhysicalBoxProxy>& result,
        CoordsXY tile, const PathElement& path)
    {
        if (!path.hasAddition() || path.additionIsGhost())
            return;
        const auto* entry = path.getAdditionEntry();
        if (entry == nullptr
            || entry->draw_type == PathAdditionDrawType::jumpingFountain)
            return;

        const uint8_t edges = uint8_t(path.getEdges() ^ 0x0Fu);
        int32_t height = path.getBaseZ();
        if (path.isSloped()
            && (entry->draw_type == PathAdditionDrawType::light
                || entry->draw_type == PathAdditionDrawType::bin))
        {
            height += 8;
        }

        const auto add = [&](uint8_t bit, int32_t x, int32_t y,
                             int32_t sizeX, int32_t sizeY,
                             int32_t sizeZ) {
            if ((edges & bit) == 0)
                return;
            AppendFirstPersonPhysicalBoxProxy(
                result, tile, x, y, height + 2,
                sizeX, sizeY, sizeZ,
                FirstPersonPhysicalProxyProvenance::
                    nativePathGeometry,
                uint64_t(entry->image));
        };

        switch (entry->draw_type)
        {
            case PathAdditionDrawType::light:
                add(1u << 0, 3, 8, 0, 16, 23);
                add(1u << 1, 2, 29, 22, 0, 23);
                add(1u << 2, 29, 2, 0, 22, 23);
                add(1u << 3, 8, 3, 16, 0, 23);
                break;
            case PathAdditionDrawType::bin:
            case PathAdditionDrawType::bench:
                add(1u << 0, 6, 8, 0, 16, 7);
                add(1u << 1, 8, 23, 16, 0, 7);
                add(1u << 2, 23, 8, 0, 16, 7);
                add(1u << 3, 8, 6, 16, 0, 7);
                break;
            case PathAdditionDrawType::jumpingFountain:
                break;
        }
    }

    inline void AppendFirstPersonStationFenceProxies(
        std::vector<FirstPersonPhysicalBoxProxy>& result,
        CoordsXY tile, const TrackElement& track,
        const Ride& ride)
    {
        if (!trackTypeIsStation(track.getTrackType()))
            return;
        const auto* stationObject = ride.getStationObject();
        if (stationObject != nullptr
            && stationObject->Flags.has(
                StationObjectFlag::noPlatforms))
            return;

        const auto& station =
            ride.getStation(track.getStationIndex());
        const auto hasFence = [&](int32_t dx, int32_t dy) {
            const auto adjacent =
                TileCoordsXY(tile) + TileCoordsXY{ dx, dy };
            return adjacent != station.entrance
                && adjacent != station.exit;
        };

        const int32_t baseZ = track.getBaseZ() + 7;
        const uint64_t sourceKey =
            (uint64_t(ride.id.ToUnderlying()) << 32)
            | uint32_t(track.getStationIndex().ToUnderlying());
        const auto add = [&](int32_t x, int32_t y,
                             int32_t sizeX, int32_t sizeY) {
            AppendFirstPersonPhysicalBoxProxy(
                result, tile, x, y, baseZ,
                sizeX, sizeY, 7,
                FirstPersonPhysicalProxyProvenance::
                    nativeStationGeometry,
                sourceKey);
        };

        const uint8_t direction = track.getDirection() & 3u;
        if ((direction & 1u) == 0)
        {
            // Direction 0/2 station painters place the two long platform
            // fences on the NW/SE sides. At viewport rotation zero those are
            // y=0 and y=31 in world-tile coordinates.
            if (hasFence(0, -1))
                add(0, 0, 32, 1);

            const bool farFence = hasFence(0, 1);
            if (farFence)
            {
                add(0, 31, 32, 1);
            }
            else if ((track.getTrackType() == TrackElemType::beginStation
                         && direction == 0)
                || (track.getTrackType() == TrackElemType::endStation
                    && direction == 2))
            {
                // TrackPaintUtilDrawStationImpl deliberately leaves the SE
                // platform edge open at an entrance/exit, but retains two
                // 1x8 end-cap fence fragments.
                add(31, 23, 1, 8);
                add(31, 0, 1, 8);
            }
        }
        else
        {
            // Direction 1/3 mirrors the same geometry onto the NE/SW sides.
            if (hasFence(-1, 0))
                add(0, 0, 1, 32);

            const bool farFence = hasFence(1, 0);
            if (farFence)
            {
                add(31, 0, 1, 32);
            }
            else if ((track.getTrackType() == TrackElemType::beginStation
                         && direction == 3)
                || (track.getTrackType() == TrackElemType::endStation
                    && direction == 1))
            {
                add(23, 31, 8, 1);
                add(0, 31, 8, 1);
            }
        }
    }

    struct FirstPersonRailProxySegment
    {
        FirstPersonVec3 a{};
        FirstPersonVec3 b{};
        FirstPersonBasis basisA{};
        FirstPersonBasis basisB{};
        float halfWidth = 0.0f;
        float halfHeight = 0.0f;
        FirstPersonPhysicalProxyProvenance provenance =
            FirstPersonPhysicalProxyProvenance::verifiedTrackArtwork;
    };

    struct FirstPersonVerifiedTrackProfileEvidence
    {
        FirstPersonTrackRailProfile profile{};
        uint64_t sourceFingerprint = 0;
        uint8_t holdoutKinds = 0;
        uint8_t holdoutCount = 0;
    };

    inline std::unordered_map<uint8_t, FirstPersonVerifiedTrackProfileEvidence>
        gFirstPersonVerifiedTrackProfiles;

    [[nodiscard]] inline std::optional<TrackStyle>
        FirstPersonTrackStyleFor(
            const Ride& ride, const TrackElement& track)
    {
        const auto& rtd = ride.getRideTypeDescriptor();
        const auto& painters = track.isInverted()
            ? rtd.InvertedTrackPaintFunctions
            : rtd.TrackPaintFunctions;
        const auto& painter = trackTypeIsCovered(track.getTrackType())
            ? painters.Covered : painters.Regular;
        if (painter.trackStyle == TrackStyle::null)
            return std::nullopt;
        return painter.trackStyle;
    }

    inline void PublishFirstPersonVerifiedTrackProfile(
        TrackStyle style, const FirstPersonTrackRailProfile& profile,
        uint64_t sourceFingerprint, uint8_t holdoutKinds,
        uint8_t holdoutCount)
    {
        if (style == TrackStyle::null || !profile.verified)
            return;
        gFirstPersonVerifiedTrackProfiles[static_cast<uint8_t>(style)] = {
            profile, sourceFingerprint, holdoutKinds, holdoutCount
        };
    }

    inline void WithdrawFirstPersonVerifiedTrackProfile(TrackStyle style)
    {
        if (style != TrackStyle::null)
            gFirstPersonVerifiedTrackProfiles.erase(
                static_cast<uint8_t>(style));
    }

    inline void ClearFirstPersonVerifiedTrackProfiles()
    {
        gFirstPersonVerifiedTrackProfiles.clear();
    }

    [[nodiscard]] inline bool FirstPersonHasVerifiedTrackProfiles()
    {
        return !gFirstPersonVerifiedTrackProfiles.empty();
    }

    [[nodiscard]] inline std::optional<FirstPersonTrackRailProfile>
        FirstPersonVerifiedTrackRailProfile(
            const Ride& ride, const TrackElement& track)
    {
        const auto style = FirstPersonTrackStyleFor(ride, track);
        if (!style.has_value())
            return std::nullopt;
        const auto found = gFirstPersonVerifiedTrackProfiles.find(
            static_cast<uint8_t>(*style));
        if (found == gFirstPersonVerifiedTrackProfiles.end()
            || !found->second.profile.verified)
            return std::nullopt;
        return found->second.profile;
    }

    [[nodiscard]] inline bool RideUsesStandardFirstPersonTrajectory(
        const Ride& ride)
    {
        const auto& rtd = ride.getRideTypeDescriptor();
        if (!rtd.flags.has(RtdFlag::hasTrack)
            || rtd.flags.has(RtdFlag::isFlatRide)
            || rtd.flags.has(RtdFlag::layeredVehiclePreview)
            || rtd.specialType != RtdSpecialType::none)
            return false;

        const auto* rideEntry = ride.getRideEntry();
        const auto* car =
            rideEntry != nullptr ? rideEntry->GetDefaultCar() : nullptr;
        if (car == nullptr)
            return true;
        return !car->flags.hasAny(
            CarEntryFlag::isChairlift,
            CarEntryFlag::isGoKart,
            CarEntryFlag::isMiniGolf,
            CarEntryFlag::isReverserCoasterBogie,
            CarEntryFlag::isReverserCoasterPassengerCar);
    }

    [[nodiscard]] inline std::optional<CoordsXYZ>
        FirstPersonTrackSampleOrigin(
            CoordsXY tile, TileElement* element)
    {
        const auto* track =
            element != nullptr ? element->asTrack() : nullptr;
        const auto origin =
            GetTrackSegmentOrigin(CoordsXYE{ tile, element });
        if (track == nullptr || !origin.has_value())
            return std::nullopt;

        const auto& ted =
            TrackMetadata::GetTrackElementDescriptor(
                track->getTrackType());
        if (ted.sequenceData.numSequences == 0)
            return std::nullopt;
        const auto& block0 =
            ted.sequenceData.sequences[0].clearance;
        CoordsXY sequence0{ origin->x, origin->y };
        sequence0 +=
            CoordsXY{ block0.x, block0.y }.rotate(
                track->getDirection());
        return CoordsXYZ{
            sequence0, origin->z + block0.z
        };
    }

    [[nodiscard]] inline FirstPersonVec3
        FirstPersonRailProxyCentre(
            const FirstPersonTrackTrajectoryPoint& point,
            const FirstPersonTrackRailProfile& profile,
            float gaugeSide)
    {
        return {
            point.position.x
                + point.basis.right.x * gaugeSide
                + point.basis.up.x * profile.verticalOffset,
            point.position.y
                + point.basis.right.y * gaugeSide
                + point.basis.up.y * profile.verticalOffset,
            point.position.z
                + point.basis.right.z * gaugeSide
                + point.basis.up.z * profile.verticalOffset,
        };
    }

    inline void AppendFirstPersonRailProxySegments(
        std::vector<FirstPersonRailProxySegment>& result,
        const FirstPersonTrackTrajectory& trajectory,
        const FirstPersonTrackRailProfile& profile)
    {
        if (!profile.verified || trajectory.points.size() < 2)
            return;

        size_t previous = 0;
        for (size_t i = 1; i < trajectory.points.size(); ++i)
        {
            const auto& a = trajectory.points[previous];
            const auto& b = trajectory.points[i];
            const float distance =
                FirstPersonTrackTrajectoryPointDistance(a, b);
            const float forwardDot =
                a.basis.forward.x * b.basis.forward.x
                + a.basis.forward.y * b.basis.forward.y
                + a.basis.forward.z * b.basis.forward.z;
            const float upDot =
                a.basis.up.x * b.basis.up.x
                + a.basis.up.y * b.basis.up.y
                + a.basis.up.z * b.basis.up.z;
            const bool turns =
                forwardDot < 0.9914449f || upDot < 0.9914449f;
            const bool last =
                i + 1 == trajectory.points.size();
            if (!last && distance < 3.0f && !turns)
                continue;

            if (distance > 0.05f)
            {
                for (const float gaugeSide :
                    { -profile.halfGauge, profile.halfGauge })
                {
                    result.push_back({
                        FirstPersonRailProxyCentre(
                            a, profile, gaugeSide),
                        FirstPersonRailProxyCentre(
                            b, profile, gaugeSide),
                        a.basis,
                        b.basis,
                        profile.halfWidth,
                        profile.halfHeight,
                        FirstPersonPhysicalProxyProvenance::
                            verifiedTrackArtwork,
                    });
                }
            }
            previous = i;
        }
    }

    [[nodiscard]] inline std::vector<FirstPersonRailProxySegment>
        BuildFirstPersonRailProxySegments(
            const FirstPersonTrackTrajectory& trajectory,
            const FirstPersonTrackRailProfile& profile)
    {
        std::vector<FirstPersonRailProxySegment> result;
        result.reserve(trajectory.points.size() * 2);
        AppendFirstPersonRailProxySegments(
            result, trajectory, profile);
        return result;
    }

    [[nodiscard]] inline bool
        FirstPersonRailProxyIntersectsWalkStep(
            const FirstPersonRailProxySegment& rail,
            FirstPersonVec3 from, FirstPersonVec3 to,
            float eyeHeight = 20.0f, float radius = 2.0f)
    {
        const float railLow = std::min(rail.a.z, rail.b.z)
            - rail.halfHeight;
        const float railHigh = std::max(rail.a.z, rail.b.z)
            + rail.halfHeight;
        const float walkLow = std::min(from.z, to.z);
        const float walkHigh =
            std::max(from.z, to.z) + eyeHeight;
        if (walkHigh <= railLow || walkLow >= railHigh)
            return false;

        const float threshold =
            std::max(0.25f, rail.halfWidth) + radius;
        const float threshold2 = threshold * threshold;

        const float fromDistance =
            FirstPersonPointSegmentDistance2(
                from.x, from.y,
                rail.a.x, rail.a.y, rail.b.x, rail.b.y);
        const float toDistance =
            FirstPersonPointSegmentDistance2(
                to.x, to.y,
                rail.a.x, rail.a.y, rail.b.x, rail.b.y);

        // Preserve the established "do not trap an already-overlapping
        // walker" rule before any exact segment-crossing early return.
        if (fromDistance <= threshold2
            && toDistance > fromDistance)
            return false;

        const float railX = rail.b.x - rail.a.x;
        const float railY = rail.b.y - rail.a.y;
        const float stepX = to.x - from.x;
        const float stepY = to.y - from.y;
        const float det = stepX * railY - stepY * railX;
        if (std::abs(det) > 1e-7f)
        {
            const float stepT =
                ((rail.a.x - from.x) * railY
                    - (rail.a.y - from.y) * railX)
                / det;
            const float railT =
                ((rail.a.x - from.x) * stepY
                    - (rail.a.y - from.y) * stepX)
                / det;
            if (stepT >= 0.0f && stepT <= 1.0f
                && railT >= 0.0f && railT <= 1.0f)
                return true;
        }

        const float railADistance =
            FirstPersonPointSegmentDistance2(
                rail.a.x, rail.a.y,
                from.x, from.y, to.x, to.y);
        const float railBDistance =
            FirstPersonPointSegmentDistance2(
                rail.b.x, rail.b.y,
                from.x, from.y, to.x, to.y);

        return toDistance <= threshold2
            || railADistance <= threshold2
            || railBDistance <= threshold2;
    }
} // namespace OpenRCT2::Paint
