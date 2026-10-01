/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "Paint.h"
#include "FirstPersonSmallSceneryCollision.h"
#include "FirstPersonTrackTrajectory.h"

#include "../ride/CarEntry.h"
#include "../ride/Ride.h"
#include "../ride/RideData.h"
#include "../ride/RideEntry.h"
#include "../ride/Track.h"
#include "../ride/TrackData.h"
#include "../ride/TrackStyle.h"
#include "../ride/ted/TrackElementDescriptor.h"
#include "../ride/ted/TrackElemType.h"
#include "../object/FootpathEntry.h"
#include "../object/PathAdditionEntry.h"
#include "../object/StationObject.h"
#include "../world/Footpath.h"
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
        authoritativeTrackTrajectory,
        nativeStationGeometry,
        nativePathGeometry,
        authoritativeSceneryOccupancy,
        authoritativeLargeSceneryOccupancy,
        semanticComponentGeometry,
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

    [[nodiscard]] inline uint64_t FirstPersonPhysicalTileKey(
        CoordsXY tile)
    {
        const auto origin = tile.toTileStart();
        return (uint64_t(uint32_t(
                    origin.x / kCoordsXYStep))
                << 32)
            | uint32_t(
                origin.y / kCoordsXYStep);
    }

    [[nodiscard]] inline FirstPersonVec3
        FirstPersonSemanticWorldPoint(
            const FirstPersonPaintSemanticTransform& transform,
            FirstPersonPaintSemanticVec3 point)
    {
        return {
            transform.origin.x
                + transform.axisX.x * point.x
                + transform.axisY.x * point.y
                + transform.axisZ.x * point.z,
            transform.origin.y
                + transform.axisX.y * point.x
                + transform.axisY.y * point.y
                + transform.axisZ.y * point.z,
            transform.origin.z
                + transform.axisX.z * point.x
                + transform.axisY.z * point.y
                + transform.axisZ.z * point.z,
        };
    }

    [[nodiscard]] inline bool
        FirstPersonSemanticComponentBounds(
            const FirstPersonPaintSemanticComponent& component,
            FirstPersonVec3& low, FirstPersonVec3& high)
    {
        const auto& geometry = component.geometry;
        if (geometry.kind
                == FirstPersonPaintSemanticPrimitiveKind::opening
            || geometry.pointCount == 0)
            return false;

        std::array<FirstPersonPaintSemanticVec3, 8>
            local{};
        size_t count = 0;
        switch (geometry.kind)
        {
            case FirstPersonPaintSemanticPrimitiveKind::box:
            case FirstPersonPaintSemanticPrimitiveKind::localHull:
            {
                if (geometry.pointCount < 2)
                    return false;
                const auto a = geometry.points[0];
                const auto b = geometry.points[1];
                local = { {
                    { a.x, a.y, a.z },
                    { b.x, a.y, a.z },
                    { b.x, b.y, a.z },
                    { a.x, b.y, a.z },
                    { a.x, a.y, b.z },
                    { b.x, a.y, b.z },
                    { b.x, b.y, b.z },
                    { a.x, b.y, b.z },
                } };
                count = local.size();
                break;
            }
            case FirstPersonPaintSemanticPrimitiveKind::beam:
            {
                if (geometry.pointCount < 2)
                    return false;
                const float extent =
                    std::max(
                        geometry.halfWidth,
                        geometry.halfHeight);
                const auto a = geometry.points[0];
                const auto b = geometry.points[1];
                local[0] = {
                    std::min(a.x, b.x) - extent,
                    std::min(a.y, b.y) - extent,
                    std::min(a.z, b.z) - extent,
                };
                local[1] = {
                    std::max(a.x, b.x) + extent,
                    std::max(a.y, b.y) + extent,
                    std::max(a.z, b.z) + extent,
                };
                const auto min = local[0];
                const auto max = local[1];
                local = { {
                    { min.x, min.y, min.z },
                    { max.x, min.y, min.z },
                    { max.x, max.y, min.z },
                    { min.x, max.y, min.z },
                    { min.x, min.y, max.z },
                    { max.x, min.y, max.z },
                    { max.x, max.y, max.z },
                    { min.x, max.y, max.z },
                } };
                count = local.size();
                break;
            }
            case FirstPersonPaintSemanticPrimitiveKind::plane:
            case FirstPersonPaintSemanticPrimitiveKind::footprint:
            {
                count = std::min<size_t>(
                    geometry.pointCount, 4);
                for (size_t i = 0; i < count; ++i)
                    local[i] = geometry.points[i];
                break;
            }
            case FirstPersonPaintSemanticPrimitiveKind::opening:
                return false;
        }

        if (count == 0)
            return false;
        low = high = FirstPersonSemanticWorldPoint(
            component.transform, local[0]);
        for (size_t i = 1; i < count; ++i)
        {
            const auto point = FirstPersonSemanticWorldPoint(
                component.transform, local[i]);
            low.x = std::min(low.x, point.x);
            low.y = std::min(low.y, point.y);
            low.z = std::min(low.z, point.z);
            high.x = std::max(high.x, point.x);
            high.y = std::max(high.y, point.y);
            high.z = std::max(high.z, point.z);
        }
        if (geometry.kind
                == FirstPersonPaintSemanticPrimitiveKind::plane
            || geometry.kind
                == FirstPersonPaintSemanticPrimitiveKind::footprint)
        {
            constexpr float kPlaneThickness = 0.5f;
            low.x -= kPlaneThickness;
            low.y -= kPlaneThickness;
            low.z -= kPlaneThickness;
            high.x += kPlaneThickness;
            high.y += kPlaneThickness;
            high.z += kPlaneThickness;
        }
        return true;
    }

    inline auto& FirstPersonSemanticComponentRegistry()
    {
        static std::unordered_map<
            uint64_t,
            std::vector<FirstPersonPaintSemanticComponent>>
            registry;
        return registry;
    }

    inline void PublishFirstPersonSemanticComponents(
        CoordsXY tile,
        std::vector<FirstPersonPaintSemanticComponent>
            components)
    {
        auto& registry =
            FirstPersonSemanticComponentRegistry();
        const auto key =
            FirstPersonPhysicalTileKey(tile);
        if (components.empty())
            registry.erase(key);
        else
            registry[key] = std::move(components);
    }

    [[nodiscard]] inline const std::vector<
        FirstPersonPaintSemanticComponent>*
        GetFirstPersonSemanticComponents(
            CoordsXY tile)
    {
        const auto& registry =
            FirstPersonSemanticComponentRegistry();
        const auto found = registry.find(
            FirstPersonPhysicalTileKey(tile));
        return found != registry.end()
            ? &found->second : nullptr;
    }

    inline void WithdrawFirstPersonSemanticComponents(
        CoordsXY tile)
    {
        FirstPersonSemanticComponentRegistry().erase(
            FirstPersonPhysicalTileKey(tile));
    }

    inline void ClearFirstPersonSemanticComponents()
    {
        FirstPersonSemanticComponentRegistry().clear();
    }

    [[nodiscard]] inline bool
        FirstPersonSemanticComponentIntersectsWalkStep(
            const FirstPersonPaintSemanticComponent& component,
            FirstPersonVec3 from, FirstPersonVec3 to,
            float eyeHeight = 20.0f, float radius = 2.0f)
    {
        if (!component.collidable)
            return false;
        FirstPersonVec3 low{}, high{};
        if (!FirstPersonSemanticComponentBounds(
                component, low, high))
            return false;
        return FirstPersonBoxIntersectsWalkStep(
            from, to, low, high, eyeHeight, radius);
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
                            authoritativeSceneryOccupancy,
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

    inline std::unordered_map<
        uint64_t, std::vector<FirstPersonPhysicalBoxProxy>>
        gFirstPersonLargeSceneryPhysicalProxies;

    inline void PublishFirstPersonLargeSceneryPhysicalProxies(
        uint64_t groupKey,
        const std::vector<FirstPersonPhysicalBoxProxy>& proxies)
    {
        if (groupKey == 0 || proxies.empty())
        {
            gFirstPersonLargeSceneryPhysicalProxies.erase(
                groupKey);
            return;
        }
        gFirstPersonLargeSceneryPhysicalProxies[groupKey] =
            proxies;
    }

    inline void WithdrawFirstPersonLargeSceneryPhysicalProxies(
        uint64_t groupKey)
    {
        gFirstPersonLargeSceneryPhysicalProxies.erase(
            groupKey);
    }

    inline void ClearFirstPersonLargeSceneryPhysicalProxies()
    {
        gFirstPersonLargeSceneryPhysicalProxies.clear();
    }

    [[nodiscard]] inline bool
        FirstPersonLargeSceneryProxyGroupIntersectsWalkStep(
            uint64_t groupKey,
            FirstPersonVec3 from, FirstPersonVec3 to,
            float eyeHeight = 20.0f, float radius = 2.0f)
    {
        const auto found =
            gFirstPersonLargeSceneryPhysicalProxies.find(
                groupKey);
        if (found
            == gFirstPersonLargeSceneryPhysicalProxies.end())
            return false;

        for (const auto& proxy : found->second)
        {
            if ((proxy.capabilities
                    & static_cast<uint8_t>(
                        FirstPersonPhysicalProxyCapability::
                            collide))
                == 0)
                continue;
            if (FirstPersonBoxIntersectsWalkStep(
                    from, to, proxy.low, proxy.high,
                    eyeHeight, radius))
                return true;
        }
        return false;
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
            FirstPersonPhysicalProxyProvenance::authoritativeTrackTrajectory;
    };

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

    [[nodiscard]] inline FirstPersonTrackRailProfile
        FirstPersonDefaultTrackRailProfile(TrackStyle style)
    {
        // Deterministic structural baselines in RCT world units. Stable
        // guideway geometry comes from TrackStyle; native artwork never changes
        // these dimensions at runtime.
        FirstPersonTrackRailProfile profile{};
        profile.railCount = 2;
        profile.halfGauge = 6.0f;
        profile.halfWidth = 1.0f;
        profile.halfHeight = 1.0f;
        profile.verticalOffset = 0.0f;
        switch (style)
        {
            case TrackStyle::singleRailRollerCoaster:
                profile.railCount = 1;
                profile.halfGauge = 0.0f;
                profile.halfWidth = 2.5f;
                profile.halfHeight = 1.75f;
                break;

            case TrackStyle::monorail:
            case TrackStyle::suspendedMonorail:
                profile.railCount = 1;
                profile.halfGauge = 0.0f;
                profile.halfWidth = 2.75f;
                profile.halfHeight = 2.0f;
                break;

            case TrackStyle::bobsleighCoaster:
            case TrackStyle::dinghySlide:
            case TrackStyle::dinghySlideCovered:
            case TrackStyle::logFlume:
            case TrackStyle::riverRapids:
            case TrackStyle::splashBoats:
            case TrackStyle::submarineRide:
                profile.railCount = 1;
                profile.halfGauge = 0.0f;
                profile.halfWidth = 3.0f;
                profile.halfHeight = 1.5f;
                break;

            case TrackStyle::miniatureRailway:
                profile.halfGauge = 5.0f;
                profile.halfWidth = 0.75f;
                profile.halfHeight = 0.75f;
                break;

            default:
                break;
        }
        return profile;
    }

    [[nodiscard]] inline std::optional<FirstPersonTrackRailProfile>
        FirstPersonTrackRailProfileFor(
            const Ride& ride, const TrackElement& track)
    {
        const auto style = FirstPersonTrackStyleFor(ride, track);
        if (!style.has_value())
            return std::nullopt;
        return FirstPersonDefaultTrackRailProfile(*style);
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
        if (trajectory.points.size() < 2
            || profile.railCount == 0
            || profile.halfWidth <= 0.0f
            || profile.halfHeight <= 0.0f)
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
                const auto appendRail = [&](float gaugeSide) {
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
                            authoritativeTrackTrajectory,
                    });
                };
                if (profile.railCount == 1)
                {
                    appendRail(0.0f);
                }
                else
                {
                    appendRail(-profile.halfGauge);
                    appendRail(profile.halfGauge);
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
        result.reserve(
            trajectory.points.size()
            * std::max<uint8_t>(profile.railCount, 1));
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
        struct SectionExtent
        {
            float horizontal = 0.0f;
            float vertical = 0.0f;
        };
        const auto sectionExtent =
            [&](const FirstPersonBasis& basis) {
                SectionExtent result{};
                result.vertical =
                    std::abs(basis.right.z) * rail.halfWidth
                    + std::abs(basis.up.z) * rail.halfHeight;
                for (const float rightSign :
                    { -1.0f, 1.0f })
                for (const float upSign :
                    { -1.0f, 1.0f })
                {
                    const float x =
                        basis.right.x * rail.halfWidth
                            * rightSign
                        + basis.up.x * rail.halfHeight
                            * upSign;
                    const float y =
                        basis.right.y * rail.halfWidth
                            * rightSign
                        + basis.up.y * rail.halfHeight
                            * upSign;
                    result.horizontal = std::max(
                        result.horizontal,
                        std::hypot(x, y));
                }
                return result;
            };

        const auto extentA = sectionExtent(rail.basisA);
        const auto extentB = sectionExtent(rail.basisB);
        const float railLow = std::min(
            rail.a.z - extentA.vertical,
            rail.b.z - extentB.vertical);
        const float railHigh = std::max(
            rail.a.z + extentA.vertical,
            rail.b.z + extentB.vertical);
        const float walkLow = std::min(from.z, to.z);
        const float walkHigh =
            std::max(from.z, to.z) + eyeHeight;
        if (walkHigh <= railLow || walkLow >= railHigh)
            return false;

        const float threshold =
            std::max({
                0.25f,
                extentA.horizontal,
                extentB.horizontal,
            }) + radius;
        const float threshold2 = threshold * threshold;

        const float fromDistance =
            FirstPersonPointSegmentDistance2(
                from.x, from.y,
                rail.a.x, rail.a.y,
                rail.b.x, rail.b.y);
        const float toDistance =
            FirstPersonPointSegmentDistance2(
                to.x, to.y,
                rail.a.x, rail.a.y,
                rail.b.x, rail.b.y);

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
