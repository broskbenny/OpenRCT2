/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../interface/Window.h"
#include "../paint/FirstPersonStreaming.h"
#include "../Identifiers.h"

#include <sfl/static_vector.hpp>
#include <optional>
#include <vector>

namespace OpenRCT2
{
    struct EntityBase;
    struct Viewport;

    // TODO: Move this to somewhere else, currently filters also by zoom.
    using ViewportList = sfl::static_vector<Viewport*, kWindowLimitMax>;

    struct FirstPersonTrackedVehicleVisuals
    {
        uint8_t yawBefore{}, yawAfter{}, spinBefore{}, spinAfter{};
        uint8_t pitchBefore{}, pitchAfter{}, rollBefore{}, rollAfter{};
        uint8_t flatPrimaryBefore{}, flatPrimaryAfter{};
        uint8_t flatSecondaryBefore{}, flatSecondaryAfter{};
        float alpha = 1.0f;
    };

    class EntityTweener
    {
        std::vector<EntityBase*> entities;
        std::vector<CoordsXYZ> prePos;
        std::vector<CoordsXYZ> postPos;
        EntityId _trackedVehicle = EntityId::GetNull();
        std::optional<FirstPersonTrackedVehicleVisuals> _trackedVisuals;
        std::optional<Paint::FirstPersonResolvedView> _firstPersonView;

    private:
        void populateEntities();
        void addEntity(const ViewportList& vp, EntityBase* entity);
        [[nodiscard]] bool isEntityVisible(const ViewportList& vp, const EntityBase* entity) const noexcept;

    public:
        static EntityTweener& get();

        void setFirstPersonView(const Paint::FirstPersonResolvedView& view)
        {
            _firstPersonView = view;
        }
        void clearFirstPersonView()
        {
            _firstPersonView.reset();
        }

        void setTrackedVehicle(EntityId id)
        {
            if (_trackedVehicle != id) _trackedVisuals.reset();
            _trackedVehicle = id;
        }
        std::optional<FirstPersonTrackedVehicleVisuals> trackedVehicleVisuals(EntityId id) const
        {
            return id == _trackedVehicle ? _trackedVisuals : std::nullopt;
        }
        void preTick();
        void postTick();
        void removeEntity(EntityBase* entity);
        void tween(float alpha);
        void restore();
        void reset();
    };

} // namespace OpenRCT2
