/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#include "EntityTweener.h"

#include "../entity/Guest.h"
#include "../entity/Staff.h"
#include "../interface/Viewport.h"
#include "../interface/WindowTypes.h"
#include "../ride/Vehicle.h"
#include "../ride/CarEntry.h"
#include "EntityList.h"

#include <algorithm>
#include <cmath>

namespace OpenRCT2
{
    static inline ViewportList GetUnzoomedViewports() noexcept
    {
        ViewportList viewports;
        WindowVisitEach([&](WindowBase* w) {
            if (auto* vp = WindowGetViewport(w); vp != nullptr)
            {
                if (!vp->isVisible)
                {
                    // Ignore viewports that are not visible.
                    return;
                }
                if (vp->zoom > ZoomLevel{ 0 })
                {
                    // Ignore viewports that are zoomed out, interpolation wouldn't have much of an effect
                    // due to the loss of detail.
                    return;
                }
                viewports.push_back(vp);
            }
        });
        return viewports;
    }

    bool EntityTweener::isEntityVisible(const ViewportList& vpList, const EntityBase* entity) const noexcept
    {
        const auto worldLoc = entity->getLocation();

        for (const auto* vp : vpList)
        {
            const auto screenPos = Translate3DTo2DWithZ(vp->rotation, worldLoc);
            if (vp->Contains(screenPos))
            {
                // Entity is visible in at least one overhead viewport.
                return true;
            }
        }

        if (_firstPersonView.has_value())
        {
            const auto& view = *_firstPersonView;

            // The first-person renderer maps native entity sprite pixels onto
            // upright world-space impostors. Use the same native sprite bounds
            // as a conservative visual sphere instead of assuming every entity
            // fits inside 64 world units.
            float halfWidth = std::max(1.0f, float(entity->spriteData.width));
            float verticalExtent = std::max(
                float(entity->spriteData.heightMin),
                float(entity->spriteData.heightMax));
            if (const auto* vehicle = entity->as<Vehicle>(); vehicle != nullptr)
            {
                if (const auto* entry = vehicle->Entry(); entry != nullptr)
                {
                    halfWidth = std::max(halfWidth, float(entry->spriteWidth));
                    verticalExtent = std::max(verticalExtent,
                        float(std::max(entry->spriteHeightNegative, entry->spriteHeightPositive)));
                }
            }

            const float radius = std::max(
                32.0f, std::hypot(halfWidth, verticalExtent) + 16.0f);
            const Paint::FirstPersonVec3 centre{
                float(worldLoc.x), float(worldLoc.y), float(worldLoc.z)
            };
            if (Paint::FirstPersonSphereVisible(
                    view.camera, centre, radius, view.fieldOfViewDegrees,
                    view.aspect, view.nearClip, view.farClip))
                return true;
        }

        return false;
    }

    void EntityTweener::addEntity(const ViewportList& vpList, EntityBase* entity)
    {
        if (!isEntityVisible(vpList, entity))
        {
            return;
        }

        entities.push_back(entity);
        prePos.emplace_back(entity->getLocation());
    }

    void EntityTweener::populateEntities()
    {
        const auto vpList = GetUnzoomedViewports();
        if (vpList.empty() && _trackedVehicle.IsNull() && !_firstPersonView.has_value())
        {
            // No overhead or first-person view needs interpolation.
            return;
        }

        for (auto ent : EntityList<Guest>())
        {
            addEntity(vpList, ent);
        }
        for (auto ent : EntityList<Staff>())
        {
            addEntity(vpList, ent);
        }
        for (auto ent : EntityList<Vehicle>())
        {
            // A first-person passenger can be far outside the last overhead viewport.
            // Include their car in the SAME official render-time interpolator.
            if (!_trackedVehicle.IsNull() && ent->id == _trackedVehicle)
            {
                entities.push_back(ent);
                prePos.emplace_back(ent->getLocation());
            }
            else
            {
                addEntity(vpList, ent);
            }
        }
    }

    void EntityTweener::preTick()
    {
        restore();
        reset();
        populateEntities();
        _trackedVisuals.reset();
        if (!_trackedVehicle.IsNull())
        {
            if (auto* vehicle = getGameState().entities.getEntity<Vehicle>(_trackedVehicle))
            {
                FirstPersonTrackedVehicleVisuals v{};
                v.yawBefore = v.yawAfter = vehicle->orientation;
                v.spinBefore = v.spinAfter = vehicle->spin_sprite;
                v.pitchBefore = v.pitchAfter = static_cast<uint8_t>(vehicle->pitch);
                v.rollBefore = v.rollAfter = static_cast<uint8_t>(vehicle->roll);
                v.flatPrimaryBefore = v.flatPrimaryAfter = vehicle->flatRideAnimationFrame;
                v.flatSecondaryBefore = v.flatSecondaryAfter = vehicle->flatRideSecondaryAnimationFrame;
                v.swingPositionBefore = v.swingPositionAfter = vehicle->SwingPosition;
                v.swingSpriteBefore = v.swingSpriteAfter = vehicle->SwingSprite;
                _trackedVisuals = v;
            }
        }
    }

    void EntityTweener::postTick()
    {
        if (_trackedVisuals.has_value())
        {
            if (auto* vehicle = getGameState().entities.getEntity<Vehicle>(_trackedVehicle))
            {
                _trackedVisuals->yawAfter = vehicle->orientation;
                _trackedVisuals->spinAfter = vehicle->spin_sprite;
                _trackedVisuals->pitchAfter = static_cast<uint8_t>(vehicle->pitch);
                _trackedVisuals->rollAfter = static_cast<uint8_t>(vehicle->roll);
                _trackedVisuals->flatPrimaryAfter = vehicle->flatRideAnimationFrame;
                _trackedVisuals->flatSecondaryAfter = vehicle->flatRideSecondaryAnimationFrame;
                _trackedVisuals->swingPositionAfter = vehicle->SwingPosition;
                _trackedVisuals->swingSpriteAfter = vehicle->SwingSprite;
                _trackedVisuals->alpha = 1.0f;
            }
            else _trackedVisuals.reset();
        }
        for (auto* ent : entities)
        {
            if (ent == nullptr)
            {
                // Sprite was removed, add a dummy position to keep the index aligned.
                postPos.emplace_back(0, 0, 0);
            }
            else
            {
                postPos.emplace_back(ent->getLocation());
            }
        }
    }

    static bool CanTweenEntity(EntityBase* ent)
    {
        if (ent->is<Guest>() || ent->is<Staff>() || ent->is<Vehicle>())
            return true;
        return false;
    }

    void EntityTweener::removeEntity(EntityBase* entity)
    {
        if (!CanTweenEntity(entity))
        {
            // Only peeps and vehicles are tweened, bail if type is incorrect.
            return;
        }

        if (!_trackedVehicle.IsNull() && entity->id == _trackedVehicle)
            _trackedVisuals.reset();
        auto it = std::find(entities.begin(), entities.end(), entity);
        if (it != entities.end())
            *it = nullptr;
    }

    void EntityTweener::tween(float alpha)
    {
        if (_trackedVisuals.has_value())
            _trackedVisuals->alpha = std::clamp(alpha, 0.0f, 1.0f);
        const float inv = (1.0f - alpha);
        for (size_t i = 0; i < entities.size(); ++i)
        {
            auto* ent = entities[i];
            if (ent == nullptr)
                continue;

            auto& posA = prePos[i];
            auto& posB = postPos[i];

            if (posA == posB)
                continue;

            ent->moveTo(
                { static_cast<int32_t>(std::round(posB.x * alpha + posA.x * inv)),
                  static_cast<int32_t>(std::round(posB.y * alpha + posA.y * inv)),
                  static_cast<int32_t>(std::round(posB.z * alpha + posA.z * inv)) });
        }
    }

    void EntityTweener::restore()
    {
        for (size_t i = 0; i < entities.size(); ++i)
        {
            auto* ent = entities[i];
            if (ent == nullptr || prePos[i] == postPos[i])
                continue;

            ent->moveTo(postPos[i]);
        }
    }

    void EntityTweener::reset()
    {
        entities.clear();
        prePos.clear();
        postPos.clear();
    }

    static EntityTweener tweener;

    EntityTweener& EntityTweener::get()
    {
        return tweener;
    }

} // namespace OpenRCT2
