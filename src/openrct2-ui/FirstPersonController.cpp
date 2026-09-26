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
#include <openrct2/paint/FirstPersonVehiclePose.h>
#include <openrct2/ride/Vehicle.h>
#include <openrct2/world/Map.h>
#include <openrct2/world/TileElementsView.h>
#include <openrct2/world/tile_element/PathElement.h>
#include <openrct2/world/tile_element/LargeSceneryElement.h>
#include <openrct2/world/tile_element/Slope.h>
#include <openrct2/world/Footpath.h>
#include <openrct2/world/tile_element/WallElement.h>
#include <openrct2/object/WallSceneryEntry.h>
#include <openrct2/object/LargeSceneryEntry.h>

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

        struct State
        {
            Mode mode = Mode::off;
            Paint::FirstPersonCamera camera{};
            EntityId attachedVehicle = EntityId::GetNull();
            RideId attachedRide = RideId::GetNull();
            float headYaw = 0.0f;
            float headPitch = 0.0f;
            bool previousResetDown = false;
            std::chrono::steady_clock::time_point lastUpdate = std::chrono::steady_clock::now();
            bool previousEscapeDown = false;
            SDL_bool previousRelativeMouseMode = SDL_FALSE;
            bool ownsRelativeMouseMode = false;
            float previousFloorZ = 0.0f;
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

        struct WalkingFloorSample
        {
            float z{};
            bool path = false;
        };

        WalkingFloorSample ResolveWalkingFloorSample(const CoordsXY& position, float previousFloorZ)
        {
            const auto terrainZ = static_cast<float>(TileElementHeight(position));
            float bestPathZ = terrainZ;
            float bestDelta = std::numeric_limits<float>::max();
            bool foundPath = false;

            for (auto* path : TileElementsView<PathElement>(position))
            {
                auto pathZ = static_cast<float>(path->getBaseZ());
                if (path->isSloped())
                {
                    const auto corners = GetSlopeCornerHeights(
                        path->getBaseZ(), kPathSlopeToLandSlope[path->getSlopeDirection()]);
                    pathZ = Paint::FirstPersonPathHeight(
                        float(corners.south), float(corners.east), float(corners.north), float(corners.west),
                        float(position.x & (kCoordsXYStep - 1)),
                        float(position.y & (kCoordsXYStep - 1)));
                }

                const auto delta = std::abs(pathZ - previousFloorZ);
                if (delta < bestDelta)
                {
                    bestDelta = delta;
                    bestPathZ = pathZ;
                    foundPath = true;
                }
            }

            // Path choice is still continuity-based, but final legality is
            // decided by the swept traversal below rather than by teleporting
            // to whichever destination surface happened to be nearest.
            if (foundPath && bestDelta <= 2.0f * kCoordsZStep)
                return { bestPathZ, true };
            return { terrainZ, false };
        }

        std::optional<float> ResolveWalkingTraversal(
            float fromX, float fromY, float toX, float toY, float startFloorZ)
        {
            constexpr float kSampleSpacing = 4.0f;
            constexpr float kMaximumStep = float(kCoordsZStep);
            const float distance = std::hypot(toX - fromX, toY - fromY);
            const int32_t samples = std::max(1, int32_t(std::ceil(distance / kSampleSpacing)));
            float floorZ = startFloorZ;

            for (int32_t i = 1; i <= samples; ++i)
            {
                const float t = float(i) / float(samples);
                const float x = fromX + (toX - fromX) * t;
                const float y = fromY + (toY - fromY) * t;
                const CoordsXY position{ int32_t(std::lround(x)), int32_t(std::lround(y)) };
                if (!MapIsLocationValid(position))
                    return std::nullopt;

                const auto floor = ResolveWalkingFloorSample(position, floorZ);
                if (!floor.path)
                {
                    const float waterZ = float(TileElementWaterHeight(position));
                    if (waterZ > floor.z + 0.5f)
                        return std::nullopt;
                }

                // Continuous legal terrain/path slopes change gradually across
                // these samples. A larger discontinuity is a cliff/ledge and is
                // intentionally non-walkable in either direction.
                if (!Paint::FirstPersonWalkingHeightTransitionAllowed(
                        floorZ, floor.z, kMaximumStep))
                    return std::nullopt;
                floorZ = floor.z;
            }
            return floorZ;
        }

        // Walls have authoritative positions and heights in the native map;
        // arbitrary billboard scenery does not. Collide against real static
        // wall planes without pretending that an artwork sorting box is solid.
        // Animated doors are excluded until their moving/open panels have
        // calibrated physical geometry; sealing their full footprint would
        // incorrectly block an open doorway.
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
                    if(entry==nullptr || entry->height==0 ||
                       entry->flags.has(WallSceneryFlag::isDoor)) continue;
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
            const Paint::FirstPersonVec3& from, const Paint::FirstPersonVec3& to)
        {
            const int32_t minX=std::max(0,int32_t(std::floor(std::min(from.x,to.x)/kCoordsXYStep))-1);
            const int32_t minY=std::max(0,int32_t(std::floor(std::min(from.y,to.y)/kCoordsXYStep))-1);
            const int32_t maxX=int32_t(std::floor(std::max(from.x,to.x)/kCoordsXYStep))+1;
            const int32_t maxY=int32_t(std::floor(std::max(from.y,to.y)/kCoordsXYStep))+1;
            static constexpr std::array<std::array<float,4>,4> kQuarterBounds{{
                {{16.0f,16.0f,32.0f,32.0f}},
                {{16.0f, 0.0f,32.0f,16.0f}},
                {{ 0.0f, 0.0f,16.0f,16.0f}},
                {{ 0.0f,16.0f,16.0f,32.0f}},
            }};
            for(int32_t ty=minY;ty<=maxY;++ty)
            for(int32_t tx=minX;tx<=maxX;++tx)
            {
                const CoordsXY tilePos{tx*kCoordsXYStep,ty*kCoordsXYStep};
                if(!MapIsLocationValid(tilePos)) continue;
                for(const auto* large:TileElementsView<LargeSceneryElement>(tilePos))
                {
                    if(large==nullptr || large->isGhost() || large->isInvisible()) continue;
                    const auto* entry=large->getEntry();
                    const size_t sequence=large->getSequenceIndex();
                    if(entry==nullptr || sequence>=entry->tiles.size()) continue;
                    const auto& tile=entry->tiles[sequence];
                    const uint8_t occupied=uint8_t(
                        RotateQuarterMask(tile.corners,large->getDirection()) | large->getOccupiedQuadrants());
                    const float lowZ=float(large->getBaseZ());
                    const float highZ=float(std::max(
                        large->getClearanceZ(),large->getBaseZ()+std::max(0,tile.zClearance)));
                    if(highZ<=lowZ) continue;
                    for(uint8_t q=0;q<4;++q)
                    {
                        if((occupied&(1u<<q))==0) continue;
                        const auto& b=kQuarterBounds[q];
                        if(Paint::FirstPersonBoxIntersectsWalkStep(
                            from,to,
                            {float(tilePos.x)+b[0],float(tilePos.y)+b[1],lowZ},
                            {float(tilePos.x)+b[2],float(tilePos.y)+b[3],highZ},
                            kEyeHeight))
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
                _state.attachedVehicle, _state.attachedRide, _state.headYaw, _state.headPitch);
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
                _state.camera.position.x,_state.camera.position.y,_state.previousFloorZ};
            auto tryMove = [&](float x,float y) {
                const auto floorZ = ResolveWalkingTraversal(
                    from.x, from.y, x, y, _state.previousFloorZ);
                if (!floorZ.has_value()) return false;
                if(WalkBlockedByWall(from,{x,y,*floorZ}) || WalkBlockedByLargeScenery(from,{x,y,*floorZ})) return false;
                _state.camera.position.x=x;
                _state.camera.position.y=y;
                _state.previousFloorZ=*floorZ;
                _state.camera.position.z=*floorZ+kEyeHeight;
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
            + ScreenCoordsXY{ viewport->ViewWidth() / 2, viewport->ViewHeight() / 2 };
        const auto spawn = ViewportAdjustForMapHeight(centre, viewport->rotation);
        if (!MapIsLocationValid(spawn))
            return false;

        Exit();

        _state.mode = Mode::walking;
        _state.camera.position = {
            static_cast<float>(spawn.x),
            static_cast<float>(spawn.y),
            static_cast<float>(spawn.z) + kEyeHeight,
        };
        _state.camera.yaw = static_cast<float>(viewport->rotation) * (kPi * 0.5f);
        _state.camera.pitch = 0.0f;
        _state.previousFloorZ = ResolveWalkingFloorSample(
            spawn, static_cast<float>(spawn.z)).z;
        _state.camera.position.z = _state.previousFloorZ + kEyeHeight;
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

        Exit();

        _state.mode = Mode::rideAttached;
        EntityTweener::get().setTrackedVehicle(vehicleId);
        _state.attachedVehicle = vehicleId;
        _state.attachedRide = vehicle->ride;
        const auto initialOrientation = Paint::FirstPersonVehicleSimulationOrientation(*vehicle);
        const auto initialPassenger = Paint::FirstPersonVehicleSimulationPassengerPose(*vehicle);
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
                Paint::FirstPersonCamera carOrientation{};
                carOrientation.yaw = Paint::FirstPersonVehicleYawRadians(car->orientation);
                const auto interpolated = EntityTweener::get().trackedVehicleVisuals(_state.attachedVehicle);
                if (interpolated.has_value())
                {
                    carOrientation.yaw = Paint::FirstPersonLerpAngle(
                        Paint::FirstPersonVehicleYawRadians(interpolated->yawBefore),
                        Paint::FirstPersonVehicleYawRadians(interpolated->yawAfter),
                        interpolated->alpha);
                }
                // Vehicle::pitch and Vehicle::roll UNION with flat-ride animation bytes.
                // Interpreting a Ferris wheel's frame index as 75-degree track pitch
                // would make the passenger lurch arbitrarily as the animation advances.
                const auto* ride = car->GetRide();
                const bool flatRide = ride != nullptr &&
                    ride->getRideTypeDescriptor().flags.has(RtdFlag::isFlatRide);
                carOrientation.pitch = flatRide ? 0.0f : Paint::FirstPersonVehiclePitchRadians(car->pitch);
                carOrientation.roll = flatRide ? 0.0f : Paint::FirstPersonVehicleRollRadians(car->roll);
                if (interpolated.has_value() && !flatRide)
                {
                    carOrientation.pitch = Paint::FirstPersonLerpAngle(
                        Paint::FirstPersonVehiclePitchRadians(static_cast<VehiclePitch>(interpolated->pitchBefore)),
                        Paint::FirstPersonVehiclePitchRadians(static_cast<VehiclePitch>(interpolated->pitchAfter)),
                        interpolated->alpha);
                    carOrientation.roll = Paint::FirstPersonLerpAngle(
                        Paint::FirstPersonVehicleRollRadians(static_cast<VehicleRoll>(interpolated->rollBefore)),
                        Paint::FirstPersonVehicleRollRadians(static_cast<VehicleRoll>(interpolated->rollAfter)),
                        interpolated->alpha);
                }
                // RCT2 stores the spinning carriage angle on an eight-bit turn;
                // preserve it even if the underlying track and car are stationary.
                const auto* entry = car->Entry();
                if (entry != nullptr && entry->flags.has(CarEntryFlag::hasSpinning))
                {
                    const float spin = interpolated.has_value()
                        ? Paint::FirstPersonLerpAngle(
                              Paint::SpinSpriteYawRadians(interpolated->spinBefore),
                              Paint::SpinSpriteYawRadians(interpolated->spinAfter), interpolated->alpha)
                        : Paint::SpinSpriteYawRadians(car->spin_sprite);
                    carOrientation.yaw += spin;
                }
                const auto cb = Paint::GetFirstPersonBasis(carOrientation);
                float flatPrimaryFrame = float(car->flatRideAnimationFrame);
                float flatSecondaryFrame = float(car->flatRideSecondaryAnimationFrame);
                if (interpolated.has_value())
                {
                    flatPrimaryFrame =
                        float(interpolated->flatPrimaryBefore)
                        + (float(interpolated->flatPrimaryAfter)
                           - float(interpolated->flatPrimaryBefore)) * interpolated->alpha;
                    const float beforeAngle =
                        float(interpolated->flatSecondaryBefore & 0x0F)
                        * (kTwoPi / 16.0f);
                    const float afterAngle =
                        float(interpolated->flatSecondaryAfter & 0x0F)
                        * (kTwoPi / 16.0f);
                    flatSecondaryFrame = Paint::FirstPersonLerpAngle(
                        beforeAngle, afterAngle, interpolated->alpha) / (kTwoPi / 16.0f);
                }

                const auto loc = car->getLocation();
                const auto passenger = Paint::BuildFirstPersonPassengerPose(
                    *car, { float(loc.x), float(loc.y), float(loc.z) }, cb,
                    flatPrimaryFrame, flatSecondaryFrame);
                const auto headBasis = Paint::GetPassengerHeadBasis(
                    passenger.basis, _state.headYaw, _state.headPitch);

                // Keep scalar yaw as a fallback for consumers that cannot use
                // the explicit basis; the actual camera orientation is the
                // passenger/cabin basis plus independent head look.
                _state.camera.yaw = std::atan2(
                    passenger.basis.forward.y, passenger.basis.forward.x) + _state.headYaw;
                _state.camera.position = passenger.position;
                _state.camera.hasExplicitBasis = true;
                _state.camera.explicitBasis = headBasis;
            }
        }
        // Publish the presentation-rate first-person frustum for interpolation
        // admission on the next simulation tick. This is independent of the
        // overhead viewport's bounds and zoom.
        PublishTweenView(rt.width, rt.height);

        // Ride audio intentionally remains in simulation time. The renderer may
        // temporarily tween Vehicle::position for presentation, but the audio
        // layer stores the vehicle attachment and resolves it during game audio
        // updates instead of accepting this transient rendered transform.
        Paint::FirstPersonRenderOptions options{};
        options.camera = _state.camera;
        options.hiddenEntity = _state.mode == Mode::rideAttached ? _state.attachedVehicle : EntityId::GetNull();
        options.radiusTiles = 64;
        if (auto* mainWindow = WindowGetMain(); mainWindow != nullptr && mainWindow->viewport != nullptr)
            options.viewFlags = mainWindow->viewport->flags;
        Paint::RenderFirstPerson(rt, options);
    }
} // namespace OpenRCT2::Ui::FirstPerson

