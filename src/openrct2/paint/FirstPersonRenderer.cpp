/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#include "FirstPersonRenderer.h"
#include "FirstPersonAssetReconstruction.h"
#include "FirstPersonLargeSceneryReconstruction.h"
#include "FirstPersonPhysicalProxy.h"
#include "FirstPersonTrackTrajectory.h"
#include "FirstPersonTunnelGeometry.h"
#include "FirstPersonVehicleBodyHull.h"
#include "FirstPersonVehiclePose.h"
#include "Paint.h"
#include "Paint.SessionFlags.h"
#include "tile_element/Paint.Surface.h"
#include "tile_element/Paint.Path.h"
#include "tile_element/Paint.TileElement.h"
#include "Paint.Entity.h"

#include "../Context.h"
#include "../GameState.h"
#include "../drawing/Drawing.Sprite.h"
#include "../drawing/Colour.h"
#include "../drawing/ColourMap.h"
#include "../drawing/PaletteIndex.h"
#include "../drawing/IDrawingContext.h"
#include "../drawing/IDrawingEngine.h"
#include "../drawing/RenderTarget.h"
#include "../drawing/ScrollingText.h"
#include "../entity/EntityBase.h"
#include "../entity/EntityTweener.h"
#include "../ride/CarEntry.h"
#include "../ride/Ride.h"
#include "../ride/RideData.h"
#include "../ride/RideEntry.h"
#include "../ride/TrackData.h"
#include "../ride/TrackDesign.h"
#include "../ride/TrackPaint.h"
#include "../ride/TrackIteration.h"
#include "../ride/Vehicle.h"
#include "../ride/ted/TrackElementDescriptor.h"
#include "../interface/Viewport.h"
#include "../profiling/Profiling.h"
#include "../world/Footpath.h"
#include "../world/Map.h"
#include "../world/MapAnimation.h"
#include "../world/TileInspector.h"
#include "../world/Wall.h"
#include "../ride/Track.h"
#include "../world/tile_element/SurfaceElement.h"
#include "../world/tile_element/PathElement.h"
#include "../world/tile_element/TrackElement.h"
#include "../world/tile_element/SmallSceneryElement.h"
#include "../world/tile_element/LargeSceneryElement.h"
#include "../world/tile_element/Slope.h"
#include "../world/tile_element/TileElement.h"
#include "../world/tile_element/TileElementType.h"
#include "../world/tile_element/WallElement.h"
#include "../object/WallSceneryEntry.h"
#include "../object/LargeSceneryEntry.h"
#include "../object/SmallSceneryEntry.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace OpenRCT2::Paint
{
    namespace
    {
        #include "FirstPersonRenderer.Core.inc"
        #include "FirstPersonRenderer.World.inc"
        #include "FirstPersonRenderer.Paint.inc"
    } // namespace

    std::optional<PassengerPaintAnchor>
        CaptureFirstPersonPassengerPaintAnchor(
            const Vehicle& vehicle, uint8_t seatIndex)
    {
        return CaptureFirstPersonPassengerPaintAnchor(
            vehicle, seatIndex,
            FirstPersonPassengerPaintInterpolation{});
    }

    std::optional<PassengerPaintAnchor>
        CaptureFirstPersonPassengerPaintAnchor(
            const Vehicle& vehicle, uint8_t seatIndex,
            const FirstPersonPassengerPaintInterpolation& interpolation)
    {
        if (seatIndex >= 32)
            return std::nullopt;
        const auto* ride = vehicle.GetRide();
        if (ride == nullptr
            || !ride->getRideTypeDescriptor().flags.has(
                RtdFlag::isFlatRide))
            return std::nullopt;

        const auto tryTile =
            [&](CoordsXY tile)
                -> std::optional<PassengerPaintAnchor> {
                if (!MapIsLocationValid(tile))
                    return std::nullopt;

                const auto projected =
                    Translate3DTo2DWithZ(
                        0,
                        {
                            tile.x + kCoordsXYHalfTile,
                            tile.y + kCoordsXYHalfTile,
                            vehicle.TrackLocation.z,
                        });
                Drawing::RenderTarget target{};
                target.x = projected.x - 2048;
                target.y = projected.y - 2048;
                target.width = 4096;
                target.height = 4096;
                target.cullingX = target.x;
                target.cullingY = target.y;
                target.cullingWidth = target.width;
                target.cullingHeight = target.height;
                target.zoom_level = ZoomLevel{ 0 };

                auto* session =
                    PaintSessionAlloc(target, 0, 0);
                if (session == nullptr)
                    return std::nullopt;
                PassengerPaintAnchor anchor{};
                std::vector<FirstPersonPaintSemanticComponent>
                    semanticComponents;
                session->CurrentSource =
                    PaintStructSource::tile;
                session->FirstPersonSemanticComponentSink =
                    &semanticComponents;
                session->FirstPersonPassengerAnchorSink =
                    &anchor;
                session->FirstPersonPassengerAnchorEntity =
                    const_cast<Vehicle*>(&vehicle);
                session->FirstPersonPassengerAnchorSeatIndex =
                    seatIndex;
                session->FirstPersonPassengerInterpolation =
                    interpolation;
                TileElementPaintSetup(*session, tile);
                PaintSessionFree(session);

                if (anchor.Entity != &vehicle
                    || (anchor.seatMask
                        & (uint32_t{ 1 }
                            << seatIndex))
                        == 0)
                    return std::nullopt;
                return anchor;
            };

        const uint16_t key = vehicle.id.ToUnderlying();
        if (const auto cached =
                _passengerAnchorSourceTiles.find(key);
            cached != _passengerAnchorSourceTiles.end())
        {
            if (const auto anchor =
                    tryTile(cached->second);
                anchor.has_value())
                return anchor;
            _passengerAnchorSourceTiles.erase(cached);
        }

        const CoordsXY origin{
            vehicle.TrackLocation.x,
            vehicle.TrackLocation.y
        };
        const auto originTile = origin.toTileStart();
        constexpr int32_t kSearchRadius = 4;
        for (int32_t radius = 0;
             radius <= kSearchRadius; ++radius)
        {
            for (int32_t dy = -radius;
                 dy <= radius; ++dy)
            for (int32_t dx = -radius;
                 dx <= radius; ++dx)
            {
                if (radius != 0
                    && std::max(
                        std::abs(dx),
                        std::abs(dy)) != radius)
                    continue;
                const CoordsXY tile{
                    originTile.x
                        + dx * kCoordsXYStep,
                    originTile.y
                        + dy * kCoordsXYStep,
                };
                if (const auto anchor =
                        tryTile(tile);
                    anchor.has_value())
                {
                    _passengerAnchorSourceTiles[key] =
                        tile;
                    return anchor;
                }
            }
        }
        return std::nullopt;
    }

    std::optional<uint64_t>
        EnsureFirstPersonLargeSceneryPhysicalProxy(
            CoordsXY tile, const LargeSceneryElement& large)
    {
        if (large.isGhost() || large.isInvisible())
            return std::nullopt;
        const auto* entry = large.getEntry();
        if (entry == nullptr)
            return std::nullopt;

        auto* element = reinterpret_cast<TileElement*>(
            const_cast<LargeSceneryElement*>(&large));
        const auto group =
            GetReconstructionGroup(tile, element);
        if (!group.has_value()
            || group->type != TileElementType::largeScenery)
            return std::nullopt;

        if (const auto found =
                gFirstPersonLargeSceneryPhysicalProxies.find(
                    group->key);
            found != gFirstPersonLargeSceneryPhysicalProxies.end()
                && !found->second.empty())
            return group->key;

        if (const auto cached =
                _largeSceneryGeometryCache.find(group->key);
            cached != _largeSceneryGeometryCache.end()
                && !cached->second.dirty
                && !cached->second.collisionProxies.empty())
        {
            PublishFirstPersonLargeSceneryPhysicalProxies(
                group->key, cached->second.collisionProxies);
            return group->key;
        }

        const auto* model =
            GetLargeSceneryAssetModel(*entry);
        if (model == nullptr || !model->usable)
            return std::nullopt;

        const uint8_t direction =
            static_cast<uint8_t>(large.getDirection()) & 3u;
        if (!LargeSceneryInstanceComplete(
                *entry, *group, direction))
            return std::nullopt;

        auto proxies =
            BuildFirstPersonLargeSceneryCollisionProxies(
                *entry, *model, *group, direction);
        if (proxies.empty())
            return std::nullopt;
        PublishFirstPersonLargeSceneryPhysicalProxies(
            group->key, proxies);
        return group->key;
    }

    uint8_t GetFirstPersonTerrainSourceRotation(uint8_t slope)
    {
        return ChooseTerrainSourceRotation(slope);
    }

    bool IsFirstPersonEntityPaintRoot(const ::PaintStruct& root)
    {
        return root.Source == PaintStructSource::entity;
    }

    bool FirstPersonVerticalTunnelCutsTerrain(
        int32_t terrainBaseZ, uint8_t verticalTunnelHeight)
    {
        return verticalTunnelHeight != 0xFF
            && int32_t(verticalTunnelHeight) * kCoordsZPerTinyZ == terrainBaseZ;
    }

    FirstPersonWallPlane BuildFirstPersonWallPlane(
        CoordsXY tileOrigin, int32_t baseZ, uint8_t direction, uint8_t slope, int32_t height)
    {
        const auto local =
            FirstPersonWallSemanticCorners(
                direction, slope, height);
        FirstPersonWallPlane plane{};
        for (size_t i = 0; i < local.size(); ++i)
        {
            plane.corners[i] = {
                float(tileOrigin.x) + local[i].x,
                float(tileOrigin.y) + local[i].y,
                float(baseZ) + local[i].z,
            };
        }
        return plane;
    }

    std::optional<FirstPersonProjection> ProjectFirstPersonPoint(
        const FirstPersonCamera& c, const FirstPersonVec3& p, const ScreenSize& size,
        float fov, float nearClip)
    {
        return ProjectFirstPersonMath(c, p, size.width, size.height, fov, nearClip);
    }
    FirstPersonScene CollectFirstPersonScene(
        const FirstPersonRenderOptions& opt, const ScreenSize& dimensions)
    {
        FirstPersonScene scene{};
        scene.sceneEpoch = _sceneEpoch;
        scene.options = opt;
        scene.dimensions = dimensions;
        const auto map = getGameState().mapSize;
        scene.resolvedView = ResolveFirstPersonView(
            opt.camera, dimensions.width, dimensions.height, map.x, map.y,
            opt.fieldOfViewDegrees, opt.nearClip, opt.farClip);
        scene.options.farClip = scene.resolvedView.farClip;
        // Independent of the overhead paint collector: geometry is derived from live map state.
        DiscoverVisibleTiles(scene);
        CollectTerrain(scene);
        CollectTrackTrajectories(scene);
        // This remains an explicitly identified compatibility bridge for complex sprite selection.
        return scene;
    }
    void ResetFirstPersonPresentationCache()
    {
        _preparedFrame.active = false;
        _preparedFrame.valid = false;
        _preparedFrame.drawingEngine = nullptr;
        _preparedFrame.scene = {};
        _entityRotations.clear();
        _passengerAnchorSourceTiles.clear();
        _dynamicEntitySpatialCache = {};
    }

    void ClearFirstPersonSceneCache()
    {
        ResetFirstPersonPresentationCache();
        _preparedFrame.scene = {};
        ++_sceneEpoch;
        if (_sceneEpoch == 0)
            _sceneEpoch = 1;
        _terrainCache.entries.clear();
        _terrainCache.frame = 0;
        _regionBounds.clear();
        _staticPaintCache.clear();
        _reconstructionRotations.clear();
        _entityRotations.clear();
        _passengerAnchorSourceTiles.clear();
        _dynamicEntitySpatialCache = {};
        _trackTrajectoryCache.clear();
        _trackTrajectoryGroupsByRegion.clear();
        _trackTrajectoryGroupsByBoundsRegion.clear();
        ClearLargeSceneryAssetModelCache();
        ClearFirstPersonSmallSceneryReconstructionCache();
        _largeSceneryGeometryCache.clear();
        ClearFirstPersonLargeSceneryPhysicalProxies();
        _largeSceneryGroupsByRegion.clear();
        _largeSceneryGroupsByBoundsRegion.clear();
        _activeLargeSceneryRegions.clear();
        _largeSceneryGeometryEnabled = false;
        ClearFirstPersonSemanticComponents();
        _staticRegionPackets.clear();
    }
    void InvalidateFirstPersonSceneRegion(CoordsXY low, CoordsXY high)
    {
        ClearFirstPersonLargeSceneryPhysicalProxies();
        if (_regionBounds.empty() && _terrainCache.entries.empty()
            && _staticPaintCache.empty() && _trackTrajectoryCache.empty()
            && _largeSceneryGeometryCache.empty() && _staticRegionPackets.empty()) return;
        const auto floorTile = [](int32_t x) {return int32_t(std::floor(float(x)/kCoordsXYStep));};
        const auto x0 = floorTile(std::min(low.x,high.x));
        const auto y0 = floorTile(std::min(low.y,high.y));
        const auto x1 = floorTile(std::max(low.x,high.x));
        const auto y1 = floorTile(std::max(low.y,high.y));
        std::unordered_set<uint64_t> trackCandidates;
        std::unordered_set<uint64_t> largeSceneryCandidates;
        for (int32_t regionY = y0 / 32; regionY <= y1 / 32; ++regionY)
        for (int32_t regionX = x0 / 32; regionX <= x1 / 32; ++regionX)
        {
            const uint64_t regionKey =
                FirstPersonGpuRegionKey(
                    regionX * 32, regionY * 32);
            _staticRegionPackets[regionKey].dirty = true;

            if (const auto tracks =
                    _trackTrajectoryGroupsByBoundsRegion.find(
                        regionKey);
                tracks
                    != _trackTrajectoryGroupsByBoundsRegion.end())
            {
                trackCandidates.insert(
                    tracks->second.begin(),
                    tracks->second.end());
            }
            if (const auto scenery =
                    _largeSceneryGroupsByBoundsRegion.find(
                        regionKey);
                scenery
                    != _largeSceneryGroupsByBoundsRegion.end())
            {
                largeSceneryCandidates.insert(
                    scenery->second.begin(),
                    scenery->second.end());
            }
        }
        InvalidateFirstPersonRegionBounds(
            x0, y0, x1, y1);
        for (int32_t ty = y0; ty <= y1; ++ty)
        for (int32_t tx = x0; tx <= x1; ++tx)
        {
            WithdrawFirstPersonSemanticComponents(
                { tx * kCoordsXYStep,
                  ty * kCoordsXYStep });

            // Both caches are already addressed by the exact tile key. A local
            // native invalidation should therefore cost O(invalidated tiles),
            // not O(every terrain/static tile first person has ever cached).
            const uint64_t key = TerrainKey(tx, ty);
            if (auto terrain = _terrainCache.entries.find(key);
                terrain != _terrainCache.entries.end())
            {
                terrain->second.dirty = true;
            }
            if (auto cached = _staticPaintCache.find(key);
                cached != _staticPaintCache.end())
            {
                cached->second.dirty = true;
                cached->second.visibilityDirty = true;
            }
        }
        for (const auto groupKey : trackCandidates)
        {
            const auto found =
                _trackTrajectoryCache.find(groupKey);
            if (found == _trackTrajectoryCache.end())
                continue;
            auto& trajectory = found->second;
            if (!trajectory.hasBounds)
                continue;
            if (x0 <= trajectory.maxTileX
                && x1 >= trajectory.minTileX
                && y0 <= trajectory.maxTileY
                && y1 >= trajectory.minTileY)
            {
                MarkTrackTrajectoryRegionsDirty(trajectory);
                trajectory.dirty = true;
            }
        }
        for (const auto groupKey : largeSceneryCandidates)
        {
            const auto found =
                _largeSceneryGeometryCache.find(groupKey);
            if (found == _largeSceneryGeometryCache.end())
                continue;
            auto& geometry = found->second;
            if (!geometry.hasBounds)
                continue;
            if (x0 <= geometry.maxTileX
                && x1 >= geometry.minTileX
                && y0 <= geometry.maxTileY
                && y1 >= geometry.minTileY)
            {
                MarkLargeSceneryGeometryRegionsDirty(geometry);
                geometry.dirty = true;
            }
        }
        // Generic native invalidation also covers shadows and moving objects,
        // so retain the cached data but mark it unfit for a region packet until
        // the tile is next admitted and its live terrain/static state is
        // revalidated. This prevents stale region VBOs without forcing a full
        // terrain reconstruction on every unrelated invalidation.
    }
    void InvalidateFirstPersonSceneTile(CoordsXY world)
    {
        InvalidateFirstPersonSceneRegion(world,world);
    }
    void BeginFirstPersonPresentationFrame()
    {
        _preparedFrame.active = true;
        _preparedFrame.valid = false;
        _preparedFrame.scene = {};
        ++_preparedFrame.serial;
        if (_preparedFrame.serial == 0)
            ++_preparedFrame.serial;
    }

    void EndFirstPersonPresentationFrame()
    {
        _preparedFrame.active = false;
        _preparedFrame.valid = false;
        _preparedFrame.drawingEngine = nullptr;
        _preparedFrame.scene = {};
    }

    void RenderFirstPerson(
        Drawing::RenderTarget& rt, const FirstPersonRenderOptions& opt,
        const ScreenRect& viewport)
    {
        PROFILED_FUNCTION();
        const ScreenSize dimensions{
            viewport.getWidth(), viewport.getHeight()
        };
        if (dimensions.width <= 0 || dimensions.height <= 0
            || rt.DrawingEngine == nullptr)
            return;

        const ScreenCoordsXY screenOrigin{
            viewport.getLeft(), viewport.getTop()
        };
        const bool reusePrepared =
            _preparedFrame.active
            && _preparedFrame.valid
            && _preparedFrame.drawingEngine == rt.DrawingEngine
            && _preparedFrame.dimensions.width == dimensions.width
            && _preparedFrame.dimensions.height == dimensions.height
            && _preparedFrame.screenOrigin.x == screenOrigin.x
            && _preparedFrame.screenOrigin.y == screenOrigin.y
            && SameFirstPersonRenderOptions(
                _preparedFrame.options, opt);

        FirstPersonScene localScene{};
        FirstPersonScene* scene = nullptr;
        if (reusePrepared)
        {
            scene = &_preparedFrame.scene;
        }
        else
        {
            // A second, genuinely different scene inside the same presentation
            // frame must not alias the GPU's cached first scene.
            if (_preparedFrame.active
                && _preparedFrame.valid)
            {
                ++_preparedFrame.serial;
                if (_preparedFrame.serial == 0)
                    ++_preparedFrame.serial;
            }

            localScene =
                CollectFirstPersonScene(opt, dimensions);
            localScene.screenOrigin = screenOrigin;
            localScene.presentationFrameSerial =
                _preparedFrame.active
                    ? _preparedFrame.serial : 0;
            CollectPaintSprites(localScene, rt);

            if (_preparedFrame.active)
            {
                _preparedFrame.options = opt;
                _preparedFrame.dimensions = dimensions;
                _preparedFrame.screenOrigin = screenOrigin;
                _preparedFrame.drawingEngine =
                    rt.DrawingEngine;
                _preparedFrame.scene =
                    std::move(localScene);
                _preparedFrame.valid = true;
                scene = &_preparedFrame.scene;
            }
            else
            {
                scene = &localScene;
            }
        }

        auto* context =
            rt.DrawingEngine->GetDrawingContext();
        if (context != nullptr)
            context->DrawFirstPersonScene(rt, *scene);

    }
} // namespace OpenRCT2::Paint

