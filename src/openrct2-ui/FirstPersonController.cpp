/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "FirstPersonController.h"

#include <SDL_keyboard.h>
#include <SDL_mouse.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <unordered_set>
#include <chrono>
#include <openrct2/drawing/IDrawingEngine.h>
#include <openrct2/audio/Audio.h>
#include <openrct2/ride/CarEntry.h>
#include <openrct2/ride/Ride.h>
#include <openrct2/ride/RideData.h>
#include <openrct2-ui/interface/Window.h>
#include <openrct2/Context.h>
#include <openrct2/GameState.h>
#include <openrct2/entity/Yaw.hpp>
#include <openrct2/entity/EntityTweener.h>
#include <openrct2/interface/Viewport.h>
#include <openrct2/paint/FirstPersonRenderer.h>
#include <openrct2/paint/FirstPersonPhysicalProxy.h>
#include <openrct2/paint/FirstPersonSmallSceneryCollision.h>
#include <openrct2/paint/FirstPersonVehiclePose.h>
#include <openrct2/paint/FirstPersonWalkingSemantics.h>
#include <openrct2/ride/Vehicle.h>
#include <openrct2/world/Map.h>
#include <openrct2/world/TileElementsView.h>
#include <openrct2/world/tile_element/PathElement.h>
#include <openrct2/world/tile_element/LargeSceneryElement.h>
#include <openrct2/world/tile_element/SmallSceneryElement.h>
#include <openrct2/world/tile_element/TrackElement.h>
#include <openrct2/world/tile_element/Slope.h>
#include <openrct2/world/Footpath.h>
#include <openrct2/world/tile_element/WallElement.h>
#include <openrct2/object/WallSceneryEntry.h>
#include <openrct2/object/LargeSceneryEntry.h>
#include <openrct2/object/SmallSceneryEntry.h>

namespace OpenRCT2::Ui::FirstPerson
{
    namespace
    {
        constexpr float kPi = 3.14159265358979323846f;
        constexpr float kTwoPi = 2.0f * kPi;
        constexpr float kEyeHeight = 20.0f;
        constexpr float kWalkSpeed = 160.0f;
        constexpr float kFastWalkSpeed = 320.0f;
        constexpr float kMouseSensitivity = 0.0035f;
        constexpr float kMaxPitch = 1.35f;

        struct WalkingSupportIdentity
        {
            bool path = false;
            CoordsXY tile{};
            int32_t baseZ = 0;
            ObjectEntryIndex surface{};
            ObjectEntryIndex railings{};
        };

        struct WalkingFloorSample
        {
            float z{};
            WalkingSupportIdentity support{};
            uint8_t edges = 0;
            uint8_t corners = 0;

            [[nodiscard]] bool IsPath() const
            {
                return support.path;
            }
        };

        struct State
        {
            Mode mode = Mode::off;
            Paint::FirstPersonCamera camera{};
            EntityId attachedVehicle = EntityId::GetNull();
            RideId attachedRide = RideId::GetNull();
            uint8_t attachedSeat = 0;
            float headYaw = 0.0f;
            float headPitch = 0.0f;
            bool previousResetDown = false;
            std::chrono::steady_clock::time_point lastUpdate = std::chrono::steady_clock::now();
            bool previousEscapeDown = false;
            SDL_bool previousRelativeMouseMode = SDL_FALSE;
            bool ownsRelativeMouseMode = false;
            WalkingFloorSample previousFloor{};
        };

        State _state{};

        // One authoritative ear/eye transform. On a ride this is the already
        // interpolated passenger camera, including independent head orientation;
        // walking and riding never create separate audio attachment positions.
        void PublishAudioListener()
        {
            const auto basis = Paint::GetFirstPersonBasis(_state.camera);
            const auto& eye = _state.camera.position;
            const auto& right = basis.right;
            Audio::SetFirstPersonAudioListener({
                { eye.x, eye.y, eye.z }, { right.x, right.y, right.z }
            });
        }

        void PublishTweenView(int32_t width = 0, int32_t height = 0)
        {
            if (_state.mode == Mode::off)
            {
                EntityTweener::get().clearFirstPersonView();
                return;
            }

            if (width <= 0 || height <= 0)
            {
                if (auto* mainWindow = WindowGetMain();
                    mainWindow != nullptr && mainWindow->viewport != nullptr)
                {
                    width = mainWindow->viewport->ViewWidth();
                    height = mainWindow->viewport->ViewHeight();
                }
            }
            const auto map = getGameState().mapSize;
            EntityTweener::get().setFirstPersonView(Paint::ResolveFirstPersonView(
                _state.camera, width, height, map.x, map.y));
        }

        void CaptureMouse()
        {
            if (_state.ownsRelativeMouseMode)
                return;

            _state.previousRelativeMouseMode = SDL_GetRelativeMouseMode();
            SDL_SetRelativeMouseMode(SDL_TRUE);
            SDL_GetRelativeMouseState(nullptr, nullptr);
            _state.ownsRelativeMouseMode = true;
        }

        void ReleaseMouse()
        {
            if (!_state.ownsRelativeMouseMode)
                return;

            SDL_SetRelativeMouseMode(_state.previousRelativeMouseMode);
            _state.ownsRelativeMouseMode = false;
        }

        [[nodiscard]] bool SameWalkingPath(
            const WalkingSupportIdentity& a, const WalkingSupportIdentity& b)
        {
            return a.path && b.path
                && a.tile.x == b.tile.x && a.tile.y == b.tile.y
                && a.baseZ == b.baseZ && a.surface == b.surface && a.railings == b.railings;
        }

        [[nodiscard]] bool PointOnPathDeck(const PathElement& path, const CoordsXY& position)
        {
            const auto tile = position.toTileStart();
            const int32_t x = position.x - tile.x;
            const int32_t y = position.y - tile.y;
            const auto in = [](int32_t value, int32_t low, int32_t high) {
                return value >= low && value <= high;
            };

            // Central deck plus native edge arms and corner fills. This avoids
            // treating the entire tile as floor for a narrow raised path.
            if (in(x, 8, 24) && in(y, 8, 24))
                return true;

            const uint8_t edges = path.getEdges();
            if ((edges & (1u << 0)) != 0 && x <= 16 && in(y, 8, 24))
                return true; // -X
            if ((edges & (1u << 1)) != 0 && y >= 16 && in(x, 8, 24))
                return true; // +Y
            if ((edges & (1u << 2)) != 0 && x >= 16 && in(y, 8, 24))
                return true; // +X
            if ((edges & (1u << 3)) != 0 && y <= 16 && in(x, 8, 24))
                return true; // -Y

            const uint8_t corners = path.getCorners();
            if ((corners & (1u << 0)) != 0 && x <= 16 && y >= 16)
                return true;
            if ((corners & (1u << 1)) != 0 && x >= 16 && y >= 16)
                return true;
            if ((corners & (1u << 2)) != 0 && x >= 16 && y <= 16)
                return true;
            if ((corners & (1u << 3)) != 0 && x <= 16 && y <= 16)
                return true;
            return false;
        }

        [[nodiscard]] WalkingFloorSample WalkingPathSampleAt(
            const PathElement& path, const CoordsXY& position)
        {
            auto pathZ = static_cast<float>(path.getBaseZ());
            if (path.isSloped())
            {
                const auto slopeCorners = GetSlopeCornerHeights(
                    path.getBaseZ(),
                    kPathSlopeToLandSlope[path.getSlopeDirection()]);
                pathZ = Paint::FirstPersonPathHeight(
                    float(slopeCorners.south), float(slopeCorners.east),
                    float(slopeCorners.north), float(slopeCorners.west),
                    float(position.x & (kCoordsXYStep - 1)),
                    float(position.y & (kCoordsXYStep - 1)));
            }

            WalkingFloorSample sample{};
            sample.z = pathZ;
            sample.support.path = true;
            sample.support.tile = position.toTileStart();
            sample.support.baseZ = path.getBaseZ();
            sample.support.surface = path.getSurfaceEntryIndex();
            sample.support.railings = path.getRailingsEntryIndex();
            sample.edges = path.getEdges();
            sample.corners = path.getCorners();
            return sample;
        }

        [[nodiscard]] bool WalkingPathsConnected(
            const WalkingFloorSample& from, const WalkingFloorSample& to)
        {
            if (!from.IsPath() || !to.IsPath())
                return false;
            if (SameWalkingPath(from.support, to.support))
                return true;

            const int32_t dx = to.support.tile.x - from.support.tile.x;
            const int32_t dy = to.support.tile.y - from.support.tile.y;
            uint8_t direction = 0xFF;
            if (dx == -kCoordsXYStep && dy == 0)
                direction = 0;
            else if (dx == 0 && dy == kCoordsXYStep)
                direction = 1;
            else if (dx == kCoordsXYStep && dy == 0)
                direction = 2;
            else if (dx == 0 && dy == -kCoordsXYStep)
                direction = 3;
            if (direction == 0xFF)
                return false;

            const uint8_t reverse = (direction + 2) & 3;
            return (from.edges & (1u << direction)) != 0
                && (to.edges & (1u << reverse)) != 0;
        }

        [[nodiscard]] WalkingFloorSample WalkingTerrainSampleAt(
            const CoordsXY& position)
        {
            WalkingFloorSample terrain{};
            terrain.z = static_cast<float>(TileElementHeight(position));
            terrain.support.tile = position.toTileStart();
            return terrain;
        }

        [[nodiscard]] std::optional<WalkingFloorSample>
            RevalidateWalkingStartFloor(
                const CoordsXY& position,
                const WalkingFloorSample& previous)
        {
            if (!MapIsLocationValid(position))
                return std::nullopt;

            if (previous.IsPath())
            {
                for (auto* path :
                    TileElementsView<PathElement>(position))
                {
                    if (path == nullptr || path->isGhost()
                        || path->isInvisible()
                        || !PointOnPathDeck(*path, position))
                        continue;
                    const auto candidate =
                        WalkingPathSampleAt(*path, position);
                    if (SameWalkingPath(
                            previous.support,
                            candidate.support))
                    {
                        return candidate;
                    }
                }

                // The support itself was removed or replaced by a map edit.
                // Recover explicitly to terrain at the current XY instead of
                // silently acquiring some unrelated nearby raised path.
                return WalkingTerrainSampleAt(position);
            }

            // Terrain is the same semantic support even if terraforming changed
            // its height beneath the player; follow that edited support rather
            // than resolving upward to another surface.
            return WalkingTerrainSampleAt(position);
        }

        WalkingFloorSample ResolveWalkingFloorSample(
            const CoordsXY& position, const WalkingFloorSample& previous)
        {
            const auto terrain = WalkingTerrainSampleAt(position);

            float bestDelta = std::numeric_limits<float>::max();
            std::optional<WalkingFloorSample> bestPath;

            for (auto* path : TileElementsView<PathElement>(position))
            {
                if (path == nullptr || path->isGhost() || path->isInvisible())
                    continue;
                if (!PointOnPathDeck(*path, position))
                    continue;

                const auto candidate =
                    WalkingPathSampleAt(*path, position);
                const auto pathZ = candidate.z;

                if (previous.IsPath()
                    && !SameWalkingPath(previous.support, candidate.support)
                    && !WalkingPathsConnected(previous, candidate))
                    continue;

                const auto delta = std::abs(pathZ - previous.z);
                if (delta < bestDelta)
                {
                    bestDelta = delta;
                    bestPath = candidate;
                }
            }

            if (bestPath.has_value() && bestDelta <= 2.0f * kCoordsZStep)
                return *bestPath;
            return terrain;
        }

        std::optional<WalkingFloorSample> ResolveWalkingTraversal(
            float fromX, float fromY, float toX, float toY, const WalkingFloorSample& startFloor)
        {
            constexpr float kSampleSpacing = 4.0f;
            constexpr float kMaximumStep = float(kCoordsZStep);
            const float distance = std::hypot(toX - fromX, toY - fromY);
            const int32_t samples = std::max(1, int32_t(std::ceil(distance / kSampleSpacing)));
            const CoordsXY startPosition{
                int32_t(std::lround(fromX)), int32_t(std::lround(fromY))
            };
            const auto revalidatedStart =
                RevalidateWalkingStartFloor(
                    startPosition, startFloor);
            if (!revalidatedStart.has_value())
                return std::nullopt;
            auto floor = *revalidatedStart;

            if (!floor.IsPath())
            {
                const float waterZ =
                    float(TileElementWaterHeight(startPosition));
                if (waterZ > floor.z + 0.5f)
                    return std::nullopt;
            }

            // A zero-length traversal is support preservation, not a request to
            // resolve a different nearby floor. This keeps terrain beneath a
            // raised path from snapping upward before movement begins.
            if (distance <= std::numeric_limits<float>::epsilon())
                return floor;

            for (int32_t i = 1; i <= samples; ++i)
            {
                const float t = float(i) / float(samples);
                const float x = fromX + (toX - fromX) * t;
                const float y = fromY + (toY - fromY) * t;
                const CoordsXY position{ int32_t(std::lround(x)), int32_t(std::lround(y)) };
                if (!MapIsLocationValid(position))
                    return std::nullopt;

                const auto nextFloor = ResolveWalkingFloorSample(position, floor);
                if (!nextFloor.IsPath())
                {
                    const float waterZ = float(TileElementWaterHeight(position));
                    if (waterZ > nextFloor.z + 0.5f)
                        return std::nullopt;

                    // Do not step sideways from a raised deck onto the terrain
                    // beneath its unused tile area / railing boundary.
                    if (floor.IsPath() && floor.z > nextFloor.z + 0.5f)
                        return std::nullopt;
                }

                if (floor.IsPath() && nextFloor.IsPath()
                    && !WalkingPathsConnected(floor, nextFloor))
                    return std::nullopt;

                if (!Paint::FirstPersonWalkingHeightTransitionAllowed(
                        floor.z, nextFloor.z, kMaximumStep))
                    return std::nullopt;
                floor = nextFloor;
            }
            return floor;
        }

        // Walls have authoritative positions and heights in the native map;
        // arbitrary billboard scenery does not. Collide against real static
        // wall planes without pretending that an artwork sorting box is solid.
        // Doors use native semantic state for the two stable endpoints:
        // closed frame 0 keeps the wall plane and stable-open frame 5 removes
        // it. Transitional frames remain conservatively blocked until the
        // moving panel aperture is calibrated from door artwork.
        bool WalkBlockedByWall(
            const Paint::FirstPersonVec3& from, const Paint::FirstPersonVec3& to)
        {
            const int32_t minX=std::max(0,int32_t(std::floor(std::min(from.x,to.x)/kCoordsXYStep))-1);
            const int32_t minY=std::max(0,int32_t(std::floor(std::min(from.y,to.y)/kCoordsXYStep))-1);
            const int32_t maxX=int32_t(std::floor(std::max(from.x,to.x)/kCoordsXYStep))+1;
            const int32_t maxY=int32_t(std::floor(std::max(from.y,to.y)/kCoordsXYStep))+1;
            for(int32_t ty=minY;ty<=maxY;++ty)
            for(int32_t tx=minX;tx<=maxX;++tx)
            {
                const CoordsXY tile{tx*kCoordsXYStep,ty*kCoordsXYStep};
                if(!MapIsLocationValid(tile)) continue;
                for(const auto* wall:TileElementsView<WallElement>(tile))
                {
                    if(wall==nullptr || wall->isGhost() || wall->isInvisible()) continue;
                    const auto* entry=wall->getEntry();
                    if(entry==nullptr || entry->height==0) continue;
                    if(entry->flags.has(WallSceneryFlag::isDoor)
                        && !Paint::FirstPersonDoorBlocksWalking(
                            wall->getAnimationFrame()))
                        continue;
                    const auto plane = Paint::BuildFirstPersonWallPlane(
                        tile, wall->getBaseZ(), wall->getDirection(), wall->getSlope(),
                        int32_t(entry->height) * kCoordsZStep);
                    const auto& lowerA = plane.corners[0];
                    const auto& lowerB = plane.corners[1];
                    const float wallHeight = float(int32_t(entry->height) * kCoordsZStep);
                    if(Paint::FirstPersonSlopedWallIntersectsWalkStep(
                           from,to,lowerA,lowerB,wallHeight,kEyeHeight))
                        return true;
                }
            }
            return false;
        }

        [[nodiscard]] uint8_t RotateQuarterMask(uint8_t mask, uint8_t direction)
        {
            mask &= 0x0F;
            direction &= 3;
            if(direction==0) return mask;
            return uint8_t(((uint32_t(mask)<<direction)|(uint32_t(mask)>>(4-direction)))&0x0F);
        }

        // Large scenery has native per-sequence quarter occupancy and vertical
        // clearance used by OpenRCT2's own construction checks. Those semantics
        // are strong enough for a conservative walking blocker; the visual art
        // remains the original sprite and is not replaced by these boxes.
        bool WalkBlockedByLargeScenery(
            const Paint::FirstPersonVec3& from,
            const Paint::FirstPersonVec3& to)
        {
            const int32_t minX = std::max(
                0, int32_t(std::floor(
                    std::min(from.x, to.x)
                    / kCoordsXYStep)) - 1);
            const int32_t minY = std::max(
                0, int32_t(std::floor(
                    std::min(from.y, to.y)
                    / kCoordsXYStep)) - 1);
            const int32_t maxX = int32_t(std::floor(
                std::max(from.x, to.x)
                / kCoordsXYStep)) + 1;
            const int32_t maxY = int32_t(std::floor(
                std::max(from.y, to.y)
                / kCoordsXYStep)) + 1;
            static constexpr std::array<
                std::array<float, 4>, 4>
                kQuarterBounds{ {
                    { { 16.0f, 16.0f, 32.0f, 32.0f } },
                    { { 16.0f, 0.0f, 32.0f, 16.0f } },
                    { { 0.0f, 0.0f, 16.0f, 16.0f } },
                    { { 0.0f, 16.0f, 16.0f, 32.0f } },
                } };

            for (int32_t ty = minY; ty <= maxY; ++ty)
            for (int32_t tx = minX; tx <= maxX; ++tx)
            {
                const CoordsXY tilePos{
                    tx * kCoordsXYStep,
                    ty * kCoordsXYStep
                };
                if (!MapIsLocationValid(tilePos))
                    continue;

                for (const auto* large :
                     TileElementsView<LargeSceneryElement>(
                         tilePos))
                {
                    if (large == nullptr
                        || large->isGhost()
                        || large->isInvisible())
                        continue;
                    const auto* entry = large->getEntry();
                    const size_t sequence =
                        large->getSequenceIndex();
                    if (entry == nullptr
                        || sequence >= entry->tiles.size())
                        continue;

                    const auto& tile =
                        entry->tiles[sequence];
                    const uint8_t occupied = uint8_t(
                        RotateQuarterMask(
                            tile.corners,
                            large->getDirection())
                        | large->getOccupiedQuadrants());
                    const float lowZ =
                        float(large->getBaseZ());
                    const float highZ = float(std::max(
                        large->getClearanceZ(),
                        large->getBaseZ()
                            + std::max(
                                0, tile.zClearance)));
                    if (highZ <= lowZ)
                        continue;

                    bool broadPhaseHit = false;
                    for (uint8_t q = 0; q < 4; ++q)
                    {
                        if ((occupied & (1u << q)) == 0)
                            continue;
                        const auto& b =
                            kQuarterBounds[q];
                        if (Paint::FirstPersonBoxIntersectsWalkStep(
                                from, to,
                                {
                                    float(tilePos.x) + b[0],
                                    float(tilePos.y) + b[1],
                                    lowZ,
                                },
                                {
                                    float(tilePos.x) + b[2],
                                    float(tilePos.y) + b[3],
                                    highZ,
                                },
                                kEyeHeight))
                        {
                            broadPhaseHit = true;
                            break;
                        }
                    }
                    if (!broadPhaseHit)
                        continue;

                    const auto groupKey =
                        Paint::EnsureFirstPersonLargeSceneryPhysicalProxy(
                            tilePos, *large);
                    if (groupKey.has_value()
                        && Paint::
                            FirstPersonLargeSceneryProxyGroupIntersectsWalkStep(
                                *groupKey, from, to,
                                kEyeHeight))
                    {
                        return true;
                    }
                }
            }

            // Construction reservation alone is never collision evidence.
            return false;
        }

        bool WalkBlockedBySmallScenery(
            const Paint::FirstPersonVec3& from,
            const Paint::FirstPersonVec3& to)
        {
            const int32_t minX = std::max(
                0, int32_t(std::floor(
                    std::min(from.x, to.x) / kCoordsXYStep)) - 1);
            const int32_t minY = std::max(
                0, int32_t(std::floor(
                    std::min(from.y, to.y) / kCoordsXYStep)) - 1);
            const int32_t maxX = int32_t(std::floor(
                std::max(from.x, to.x) / kCoordsXYStep)) + 1;
            const int32_t maxY = int32_t(std::floor(
                std::max(from.y, to.y) / kCoordsXYStep)) + 1;

            std::vector<Paint::FirstPersonPhysicalBoxProxy> proxies;
            proxies.reserve(32);
            for (int32_t ty = minY; ty <= maxY; ++ty)
            for (int32_t tx = minX; tx <= maxX; ++tx)
            {
                const CoordsXY tilePos{
                    tx * kCoordsXYStep,
                    ty * kCoordsXYStep
                };
                if (!MapIsLocationValid(tilePos))
                    continue;

                for (const auto* small :
                    TileElementsView<SmallSceneryElement>(tilePos))
                {
                    if (small == nullptr || small->isGhost()
                        || small->isInvisible())
                        continue;
                    proxies.clear();
                    if (!Paint::AppendFirstPersonSmallSceneryProxies(
                            proxies, tilePos, *small))
                        continue;
                    for (const auto& proxy : proxies)
                    {
                        if (Paint::FirstPersonBoxIntersectsWalkStep(
                                from, to, proxy.low, proxy.high,
                                kEyeHeight))
                            return true;
                    }
                }
            }
            return false;
        }

        bool WalkBlockedBySemanticSupports(
            const Paint::FirstPersonVec3& from,
            const Paint::FirstPersonVec3& to)
        {
            const int32_t minX = std::max(
                0, int32_t(std::floor(
                    std::min(from.x, to.x)
                    / kCoordsXYStep)) - 1);
            const int32_t minY = std::max(
                0, int32_t(std::floor(
                    std::min(from.y, to.y)
                    / kCoordsXYStep)) - 1);
            const int32_t maxX = int32_t(std::floor(
                std::max(from.x, to.x)
                / kCoordsXYStep)) + 1;
            const int32_t maxY = int32_t(std::floor(
                std::max(from.y, to.y)
                / kCoordsXYStep)) + 1;

            for (int32_t ty = minY; ty <= maxY; ++ty)
            for (int32_t tx = minX; tx <= maxX; ++tx)
            {
                const CoordsXY tilePos{
                    tx * kCoordsXYStep,
                    ty * kCoordsXYStep
                };
                const auto* proxies =
                    Paint::GetFirstPersonSemanticSupportProxies(
                        tilePos);
                if (proxies == nullptr)
                    continue;
                for (const auto& proxy : *proxies)
                {
                    if (Paint::FirstPersonBoxIntersectsWalkStep(
                            from, to,
                            proxy.low, proxy.high,
                            kEyeHeight))
                        return true;
                }
            }
            return false;
        }

        bool WalkBlockedByPathFixtures(
            const Paint::FirstPersonVec3& from,
            const Paint::FirstPersonVec3& to)
        {
            const int32_t minX = std::max(
                0, int32_t(std::floor(
                    std::min(from.x, to.x) / kCoordsXYStep)) - 1);
            const int32_t minY = std::max(
                0, int32_t(std::floor(
                    std::min(from.y, to.y) / kCoordsXYStep)) - 1);
            const int32_t maxX = int32_t(std::floor(
                std::max(from.x, to.x) / kCoordsXYStep)) + 1;
            const int32_t maxY = int32_t(std::floor(
                std::max(from.y, to.y) / kCoordsXYStep)) + 1;

            std::vector<Paint::FirstPersonPhysicalBoxProxy> proxies;
            proxies.reserve(16);
            for (int32_t ty = minY; ty <= maxY; ++ty)
            for (int32_t tx = minX; tx <= maxX; ++tx)
            {
                const CoordsXY tilePos{
                    tx * kCoordsXYStep, ty * kCoordsXYStep
                };
                if (!MapIsLocationValid(tilePos))
                    continue;
                for (const auto* path :
                    TileElementsView<PathElement>(tilePos))
                {
                    if (path == nullptr || path->isGhost()
                        || path->isInvisible())
                        continue;
                    proxies.clear();
                    Paint::AppendFirstPersonPathRailingProxies(
                        proxies, tilePos, *path);
                    Paint::AppendFirstPersonPathFixtureProxies(
                        proxies, tilePos, *path);
                    for (const auto& proxy : proxies)
                    {
                        if (Paint::FirstPersonBoxIntersectsWalkStep(
                                from, to, proxy.low, proxy.high,
                                kEyeHeight))
                            return true;
                    }
                }
            }
            return false;
        }

        bool WalkBlockedByStationFences(
            const Paint::FirstPersonVec3& from,
            const Paint::FirstPersonVec3& to)
        {
            const int32_t minX = std::max(
                0, int32_t(std::floor(
                    std::min(from.x, to.x) / kCoordsXYStep)) - 1);
            const int32_t minY = std::max(
                0, int32_t(std::floor(
                    std::min(from.y, to.y) / kCoordsXYStep)) - 1);
            const int32_t maxX = int32_t(std::floor(
                std::max(from.x, to.x) / kCoordsXYStep)) + 1;
            const int32_t maxY = int32_t(std::floor(
                std::max(from.y, to.y) / kCoordsXYStep)) + 1;

            std::vector<Paint::FirstPersonPhysicalBoxProxy> proxies;
            proxies.reserve(8);
            for (int32_t ty = minY; ty <= maxY; ++ty)
            for (int32_t tx = minX; tx <= maxX; ++tx)
            {
                const CoordsXY tilePos{
                    tx * kCoordsXYStep, ty * kCoordsXYStep
                };
                if (!MapIsLocationValid(tilePos))
                    continue;
                for (const auto* track :
                    TileElementsView<TrackElement>(tilePos))
                {
                    if (track == nullptr || track->isGhost()
                        || track->isInvisible()
                        || !trackTypeIsStation(track->getTrackType()))
                        continue;
                    const auto* ride =
                        GetRide(track->getRideIndex());
                    if (ride == nullptr)
                        continue;
                    proxies.clear();
                    Paint::AppendFirstPersonStationFenceProxies(
                        proxies, tilePos, *track, *ride);
                    for (const auto& proxy : proxies)
                    {
                        if (Paint::FirstPersonBoxIntersectsWalkStep(
                                from, to, proxy.low, proxy.high,
                                kEyeHeight))
                            return true;
                    }
                }
            }
            return false;
        }

        bool WalkBlockedByVerifiedTrackRails(
            const Paint::FirstPersonVec3& from,
            const Paint::FirstPersonVec3& to)
        {
            if (!Paint::FirstPersonHasVerifiedTrackProfiles())
                return false;

            const int32_t minX = std::max(
                0, int32_t(std::floor(
                    std::min(from.x, to.x) / kCoordsXYStep)) - 2);
            const int32_t minY = std::max(
                0, int32_t(std::floor(
                    std::min(from.y, to.y) / kCoordsXYStep)) - 2);
            const int32_t maxX = int32_t(std::floor(
                std::max(from.x, to.x) / kCoordsXYStep)) + 2;
            const int32_t maxY = int32_t(std::floor(
                std::max(from.y, to.y) / kCoordsXYStep)) + 2;
            std::unordered_set<uint64_t> seenPieces;

            for (int32_t ty = minY; ty <= maxY; ++ty)
            for (int32_t tx = minX; tx <= maxX; ++tx)
            {
                const CoordsXY tilePos{
                    tx * kCoordsXYStep, ty * kCoordsXYStep
                };
                if (!MapIsLocationValid(tilePos))
                    continue;

                for (auto* track :
                    TileElementsView<TrackElement>(tilePos))
                {
                    if (track == nullptr || track->isGhost()
                        || track->isInvisible())
                        continue;
                    const auto* ride =
                        GetRide(track->getRideIndex());
                    if (ride == nullptr
                        || !Paint::RideUsesStandardFirstPersonTrajectory(
                            *ride))
                        continue;
                    const auto profile =
                        Paint::FirstPersonVerifiedTrackRailProfile(
                            *ride, *track);
                    if (!profile.has_value()
                        || !profile->verified)
                        continue;

                    const auto sampleOrigin =
                        Paint::FirstPersonTrackSampleOrigin(
                            tilePos, track->as<TileElement>());
                    if (!sampleOrigin.has_value())
                        continue;

                    uint64_t pieceKey =
                        uint64_t(track->getRideIndex().ToUnderlying());
                    pieceKey = pieceKey * 1099511628211ull
                        ^ uint64_t(uint16_t(track->getTrackType()));
                    pieceKey = pieceKey * 1099511628211ull
                        ^ uint64_t(uint32_t(sampleOrigin->x));
                    pieceKey = pieceKey * 1099511628211ull
                        ^ uint64_t(uint32_t(sampleOrigin->y));
                    pieceKey = pieceKey * 1099511628211ull
                        ^ uint64_t(uint32_t(sampleOrigin->z));
                    if (!seenPieces.insert(pieceKey).second)
                        continue;

                    const auto trajectory =
                        Paint::BuildFirstPersonTrackTrajectory(
                            track->getTrackType(),
                            track->getDirection(),
                            {
                                float(sampleOrigin->x),
                                float(sampleOrigin->y),
                                float(sampleOrigin->z),
                            });
                    if (!trajectory.has_value()
                        || !Paint::FirstPersonTrackTrajectorySamplesContinuous(
                            *trajectory))
                        continue;

                    const auto rails =
                        Paint::BuildFirstPersonRailProxySegments(
                            *trajectory, *profile);
                    for (const auto& rail : rails)
                    {
                        if (Paint::FirstPersonRailProxyIntersectsWalkStep(
                                rail, from, to, kEyeHeight))
                            return true;
                    }
                }
            }
            return false;
        }

        void UpdateMouseLook(bool riding)
        {
            int32_t dx = 0;
            int32_t dy = 0;
            SDL_GetRelativeMouseState(&dx, &dy);
            float& yaw = riding ? _state.headYaw : _state.camera.yaw;
            float& pitch = riding ? _state.headPitch : _state.camera.pitch;
            yaw += static_cast<float>(dx) * kMouseSensitivity;
            pitch = std::clamp(pitch - static_cast<float>(dy) * kMouseSensitivity, -kMaxPitch, kMaxPitch);
            if (yaw > kTwoPi || yaw < -kTwoPi)
                yaw = std::fmod(yaw, kTwoPi);
        }

        float DeltaSeconds()
        {
            const auto now = std::chrono::steady_clock::now();
            const auto seconds = std::chrono::duration<float>(now - _state.lastUpdate).count();
            _state.lastUpdate = now;
            return std::clamp(seconds, 0.0f, 0.05f);
        }
        void PublishRideAudioAttachment()
        {
            Audio::SetFirstPersonRideAudioListener(
                _state.attachedVehicle, _state.attachedRide, _state.attachedSeat,
                _state.headYaw, _state.headPitch);
        }

        // Relative mouse deltas are presentation input. Consume them from the
        // render path so a 144 Hz display does not quantise head motion to the
        // 40 Hz simulation tick. Walking translation remains fixed-step.
        void UpdatePresentationInput()
        {
            if (!ContextHasFocus())
                return;
            if (_state.mode == Mode::rideAttached)
            {
                UpdateMouseLook(true);
                PublishRideAudioAttachment();
            }
            else if (_state.mode == Mode::walking)
            {
                UpdateMouseLook(false);
                PublishAudioListener();
            }
        }

        bool HandleEscape()
        {
            int count = 0;
            const auto* keys = SDL_GetKeyboardState(&count);
            const bool escapeDown = SDL_SCANCODE_ESCAPE < count && keys[SDL_SCANCODE_ESCAPE] != 0;
            const bool pressed = escapeDown && !_state.previousEscapeDown;
            _state.previousEscapeDown = escapeDown;
            if (pressed)
                Exit();
            return pressed;
        }

        void UpdateWalking()
        {
            if (!ContextHasFocus())
                return;

            const float dt = DeltaSeconds();

            int count = 0;
            const auto* keys = SDL_GetKeyboardState(&count);
            auto pressed = [&](SDL_Scancode code) {
                return static_cast<int>(code) < count && keys[code] != 0;
            };

            float forwardAxis = 0.0f;
            float strafeAxis = 0.0f;
            if (pressed(SDL_SCANCODE_W) || pressed(SDL_SCANCODE_UP))
                forwardAxis += 1.0f;
            if (pressed(SDL_SCANCODE_S) || pressed(SDL_SCANCODE_DOWN))
                forwardAxis -= 1.0f;
            if (pressed(SDL_SCANCODE_D) || pressed(SDL_SCANCODE_RIGHT))
                strafeAxis += 1.0f;
            if (pressed(SDL_SCANCODE_A) || pressed(SDL_SCANCODE_LEFT))
                strafeAxis -= 1.0f;

            const auto magnitude = std::sqrt(forwardAxis * forwardAxis + strafeAxis * strafeAxis);
            if (magnitude > 1.0f)
            {
                forwardAxis /= magnitude;
                strafeAxis /= magnitude;
            }

            const auto speed = (pressed(SDL_SCANCODE_LSHIFT) || pressed(SDL_SCANCODE_RSHIFT)) ? kFastWalkSpeed : kWalkSpeed;
            const auto cosYaw = std::cos(_state.camera.yaw);
            const auto sinYaw = std::sin(_state.camera.yaw);

            const auto nextX = _state.camera.position.x
                + dt * speed * (forwardAxis * cosYaw - strafeAxis * sinYaw);
            const auto nextY = _state.camera.position.y
                + dt * speed * (forwardAxis * sinYaw + strafeAxis * cosYaw);
            const Paint::FirstPersonVec3 from{
                _state.camera.position.x,_state.camera.position.y,_state.previousFloor.z};
            auto tryMove = [&](float x,float y) {
                const auto floor = ResolveWalkingTraversal(
                    from.x, from.y, x, y, _state.previousFloor);
                if (!floor.has_value()) return false;
                const Paint::FirstPersonVec3 destination{
                    x, y, floor->z
                };
                if (WalkBlockedByWall(from, destination)
                    || WalkBlockedByLargeScenery(from, destination)
                    || WalkBlockedBySmallScenery(from, destination)
                    || WalkBlockedBySemanticSupports(from, destination)
                    || WalkBlockedByPathFixtures(from, destination)
                    || WalkBlockedByStationFences(from, destination)
                    || WalkBlockedByVerifiedTrackRails(
                        from, destination))
                    return false;
                _state.camera.position.x=x;
                _state.camera.position.y=y;
                _state.previousFloor=*floor;
                _state.camera.position.z=floor->z+kEyeHeight;
                return true;
            };
            if(!tryMove(nextX,nextY))
            {
                // Slide along an axis-aligned physical obstacle rather than stopping
                // the player completely on a diagonal movement command.
                if(!tryMove(nextX,from.y))
                    tryMove(from.x,nextY);
            }
        }

        void UpdateRideAttached()
        {
            auto* vehicle = getGameState().entities.getEntity<Vehicle>(_state.attachedVehicle);
            if (vehicle == nullptr || vehicle->ride != _state.attachedRide)
            {
                Exit();
                return;
            }
            int count = 0;
            const auto* keys = SDL_GetKeyboardState(&count);
            const bool resetDown = SDL_SCANCODE_R < count && keys[SDL_SCANCODE_R] != 0;
            if (resetDown && !_state.previousResetDown)
            {
                _state.headYaw = _state.headPitch = 0.0f;
                PublishRideAudioAttachment();
            }
            _state.previousResetDown = resetDown;
        }
    } // namespace

    Mode GetMode()
    {
        return _state.mode;
    }

    bool IsActive()
    {
        return _state.mode != Mode::off;
    }

    void ToggleWalking()
    {
        if (_state.mode == Mode::walking)
            Exit();
        else
            EnterWalking();
    }

    bool EnterWalking()
    {
        if (GetContext()->GetDrawingEngineType() != DrawingEngine::openGL)
            return false;
        auto* mainWindow = WindowGetMain();
        if (mainWindow == nullptr || mainWindow->viewport == nullptr)
            return false;

        const auto* viewport = mainWindow->viewport;
        const auto centre = viewport->viewPos
            + ScreenCoordsXY{
                viewport->ViewWidth() / 2,
                viewport->ViewHeight() / 2
            };
        const auto screenCentre = viewport->pos
            + ScreenCoordsXY{
                viewport->width / 2,
                viewport->height / 2
            };

        CoordsXYZ spawn =
            ViewportAdjustForMapHeight(
                centre, viewport->rotation);
        WalkingFloorSample initialFloor{};
        bool pickedWalkableSupport = false;
        const auto picked = GetMapCoordinatesFromPosWindow(
            mainWindow, screenCentre,
            ViewportInteractionItems{
                ViewportInteractionItem::terrain,
                ViewportInteractionItem::footpath
            });

        if (picked.interactionType
                == ViewportInteractionItem::footpath
            && picked.Element != nullptr)
        {
            const auto* path = picked.Element->asPath();
            if (path != nullptr && !path->isGhost()
                && !path->isInvisible())
            {
                auto pathPoint = ScreenGetMapXYWithZ(
                    screenCentre, path->getBaseZ());
                CoordsXY position = pathPoint.value_or(
                    picked.Loc.toTileCentre());
                position.x = std::clamp(
                    position.x, picked.Loc.x,
                    picked.Loc.x + kCoordsXYStep - 1);
                position.y = std::clamp(
                    position.y, picked.Loc.y,
                    picked.Loc.y + kCoordsXYStep - 1);
                if (!PointOnPathDeck(*path, position))
                    position = picked.Loc.toTileCentre();

                initialFloor =
                    WalkingPathSampleAt(*path, position);
                spawn = {
                    position.x, position.y,
                    int32_t(std::lround(initialFloor.z))
                };
                pickedWalkableSupport = true;
            }
        }
        else if (picked.interactionType
            == ViewportInteractionItem::terrain)
        {
            initialFloor.z = float(spawn.z);
            initialFloor.support.tile =
                CoordsXY{ spawn.x, spawn.y }.toTileStart();
            pickedWalkableSupport = true;
        }

        if (!MapIsLocationValid(spawn))
            return false;

        if (!pickedWalkableSupport)
        {
            initialFloor.z = float(spawn.z);
            initialFloor.support.tile =
                CoordsXY{ spawn.x, spawn.y }.toTileStart();
            initialFloor = ResolveWalkingFloorSample(
                CoordsXY{ spawn.x, spawn.y }, initialFloor);
        }

        Exit();

        _state.mode = Mode::walking;
        _state.camera.position = {
            float(spawn.x),
            float(spawn.y),
            initialFloor.z + kEyeHeight,
        };
        _state.camera.yaw =
            float(viewport->rotation) * (kPi * 0.5f);
        _state.camera.pitch = 0.0f;
        _state.previousFloor = initialFloor;
        _state.previousEscapeDown = false;
        PublishTweenView();
        PublishAudioListener();
        CaptureMouse();
        mainWindow->invalidate();
        return true;
    }

    bool EnterRide(EntityId vehicleId)
    {
        if (GetContext()->GetDrawingEngineType() != DrawingEngine::openGL)
            return false;
        auto* vehicle = getGameState().entities.getEntity<Vehicle>(vehicleId);
        if (vehicle == nullptr)
            return false;

        const uint8_t seatIndex =
            Paint::FirstPersonPassengerSeatIndex(*vehicle);
        const auto nativeAnchor =
            Paint::CaptureFirstPersonPassengerPaintAnchor(
                *vehicle, seatIndex);
        const auto initialPassenger =
            Paint::FirstPersonVehicleSimulationPassengerPose(
                *vehicle,
                nativeAnchor.has_value()
                    ? &*nativeAnchor : nullptr,
                seatIndex);
        if (!initialPassenger.supported)
            return false;
        const auto initialOrientation =
            Paint::FirstPersonVehicleSimulationOrientation(*vehicle);

        Exit();

        _state.mode = Mode::rideAttached;
        EntityTweener::get().setTrackedVehicle(vehicleId);
        _state.attachedVehicle = vehicleId;
        _state.attachedRide = vehicle->ride;
        _state.attachedSeat = seatIndex;
        _state.camera.position = initialPassenger.position;
        _state.camera.yaw = initialOrientation.yaw;
        _state.camera.pitch = initialOrientation.pitch;
        _state.camera.roll = initialOrientation.roll;
        _state.camera.hasExplicitBasis = true;
        _state.camera.explicitBasis = initialPassenger.basis;
        _state.previousEscapeDown = false;
        PublishTweenView();
        PublishRideAudioAttachment();
        CaptureMouse();

        if (auto* mainWindow = WindowGetMain(); mainWindow != nullptr)
            mainWindow->invalidate();
        return true;
    }

    void Exit()
    {
        const bool wasActive = IsActive();
        Audio::ClearFirstPersonAudioListener();
        ReleaseMouse();
        EntityTweener::get().clearFirstPersonView();
        EntityTweener::get().setTrackedVehicle(EntityId::GetNull());
        Paint::ClearFirstPersonSceneCache();
        _state = State{};

        if (wasActive)
        {
            if (auto* mainWindow = WindowGetMain(); mainWindow != nullptr)
                mainWindow->invalidate();
        }
    }

    void Update()
    {
        if (!IsActive())
            return;
        if (GetContext()->GetDrawingEngineType() != DrawingEngine::openGL)
        {
            Exit();
            return;
        }

        if (HandleEscape())
            return;

        switch (_state.mode)
        {
            case Mode::walking:
                UpdateWalking();
                PublishAudioListener();
                break;
            case Mode::rideAttached:
                UpdateRideAttached();
                PublishRideAudioAttachment();
                break;
            case Mode::off:
                break;
        }
        PublishTweenView();
    }

    void Render(Drawing::RenderTarget& rt)
    {
        if (!IsActive())
            return;

        UpdatePresentationInput();

        // The original EntityTweener temporarily interpolates tracked vehicle positions
        // just before drawing. Re-read the camera's car position at presentation time,
        // not only during the earlier window update callback.
        if (_state.mode == Mode::rideAttached)
        {
            auto* car = getGameState().entities.getEntity<Vehicle>(_state.attachedVehicle);
            if (car == nullptr || car->ride != _state.attachedRide)
            {
                Exit();
                return;
            }
            {
                const auto tracked =
                    EntityTweener::get().trackedVehicleVisuals(
                        _state.attachedVehicle);
                const auto presentation =
                    Paint::BuildFirstPersonVehiclePresentationState(
                        *car, tracked);
                Paint::FirstPersonPassengerPaintInterpolation
                    interpolation{};
                if (tracked.has_value())
                {
                    interpolation.enabled = true;
                    interpolation.primaryBefore =
                        tracked->flatPrimaryBefore;
                    interpolation.primaryAfter =
                        tracked->flatPrimaryAfter;
                    interpolation.secondaryBefore =
                        tracked->flatSecondaryBefore;
                    interpolation.secondaryAfter =
                        tracked->flatSecondaryAfter;
                    interpolation.alpha = tracked->alpha;
                }
                const auto nativeAnchor =
                    Paint::CaptureFirstPersonPassengerPaintAnchor(
                        *car, _state.attachedSeat,
                        interpolation);
                const auto passenger =
                    Paint::BuildFirstPersonPassengerPoseWithCarriage(
                        *car, presentation.vehicleOrigin,
                        presentation.carriage,
                        nativeAnchor.has_value()
                            ? &*nativeAnchor : nullptr,
                        _state.attachedSeat);
                if (!passenger.supported)
                {
                    Exit();
                    return;
                }
                const auto headBasis =
                    Paint::GetPassengerHeadBasis(
                        passenger.basis,
                        _state.headYaw,
                        _state.headPitch);

                // The rider camera and every first-person consumer now share
                // one presentation-time carriage transform. Scalar yaw remains
                // only a fallback for consumers that cannot use the explicit
                // basis.
                _state.camera.yaw = std::atan2(
                    passenger.basis.forward.y,
                    passenger.basis.forward.x)
                    + _state.headYaw;
                _state.camera.position = passenger.position;
                _state.camera.hasExplicitBasis = true;
                _state.camera.explicitBasis = headBasis;
            }
        }
        ScreenRect renderViewport{
            { 0, 0 }, { rt.width, rt.height }
        };
        auto* mainWindow = WindowGetMain();
        if (mainWindow != nullptr && mainWindow->viewport != nullptr)
        {
            const auto* viewport = mainWindow->viewport;
            renderViewport = {
                viewport->pos,
                viewport->pos
                    + ScreenSize{ viewport->width, viewport->height }
            };
        }

        // A clipped/dirty RenderTarget is only a redraw constraint. Camera
        // projection always uses the complete main viewport.
        PublishTweenView(
            renderViewport.getWidth(), renderViewport.getHeight());

        // Ride audio intentionally remains in simulation time. The renderer may
        // temporarily tween Vehicle::position for presentation, but the audio
        // layer stores the vehicle attachment and resolves it during game audio
        // updates instead of accepting this transient rendered transform.
        Paint::FirstPersonRenderOptions options{};
        options.camera = _state.camera;
        options.hiddenEntity = _state.mode == Mode::rideAttached ? _state.attachedVehicle : EntityId::GetNull();
        options.hiddenSeatIndex = _state.mode == Mode::rideAttached ? _state.attachedSeat : 0xFF;
        options.radiusTiles = 64;
        if (mainWindow != nullptr && mainWindow->viewport != nullptr)
            options.viewFlags = mainWindow->viewport->flags;
        Paint::RenderFirstPerson(rt, options, renderViewport);
    }
} // namespace OpenRCT2::Ui::FirstPerson

