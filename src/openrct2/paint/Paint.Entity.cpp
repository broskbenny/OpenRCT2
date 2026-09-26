/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Paint.Entity.h"

#include "../drawing/LightFX.h"
#include "../entity/EntityList.h"
#include "../entity/Staff.h"
#include "../interface/Viewport.h"
#include "../profiling/Profiling.h"
#include "../ride/TrackDesign.h"
#include "../world/Map.h"
#include "Paint.h"
#include "entity/Paint.Balloon.h"
#include "entity/Paint.CrashSplashParticle.h"
#include "entity/Paint.Duck.h"
#include "entity/Paint.ExplosionCloud.h"
#include "entity/Paint.ExplosionFlare.h"
#include "entity/Paint.Guest.h"
#include "entity/Paint.JumpingFountain.h"
#include "entity/Paint.Litter.h"
#include "entity/Paint.MoneyEffect.h"
#include "entity/Paint.Staff.h"
#include "entity/Paint.SteamParticle.h"
#include "entity/Paint.Vehicle.h"
#include "entity/Paint.VehicleCrashParticle.h"

#include <cassert>

using namespace OpenRCT2;
using namespace OpenRCT2::Drawing;

namespace
{
    bool EntityPassesViewportFilters(const PaintSession& session, EntityBase& entity)
    {
        if (gTrackDesignSaveMode || (session.ViewFlags & VIEWPORT_FLAG_HIDE_ENTITIES))
            return false;
        if (session.rt.zoom_level > ZoomLevel{ 2 })
            return false;

        if (session.ViewFlags & VIEWPORT_FLAG_HIGHLIGHT_PATH_ISSUES)
        {
            const auto staff = entity.as<Staff>();
            if (staff != nullptr)
            {
                if (staff->assignedStaffType != StaffType::handyman)
                    return false;
            }
            else if (entity.type != EntityType::litter)
            {
                return false;
            }
        }

        const auto entityPos = entity.getLocation();
        if (session.ViewFlags & VIEWPORT_FLAG_CLIP_VIEW)
        {
            if (entityPos.z > (gClipHeight * kCoordsZStep)
                && (session.ViewFlags & VIEWPORT_FLAG_CLIP_VIEW_SEE_THROUGH) == 0)
                return false;
            if (entityPos.x < gClipSelectionA.x || entityPos.x > (gClipSelectionB.x + kCoordsXYStep - 1))
                return false;
            if (entityPos.y < gClipSelectionA.y || entityPos.y > (gClipSelectionB.y + kCoordsXYStep - 1))
                return false;
        }

        const auto screenCoords = Translate3DTo2DWithZ(session.CurrentRotation, entityPos);
        const auto spriteRect = ScreenRect(
            screenCoords - ScreenCoordsXY{ entity.spriteData.width, entity.spriteData.heightMin },
            screenCoords + ScreenCoordsXY{ entity.spriteData.width, entity.spriteData.heightMax });
        const ZoomLevel zoom = session.rt.zoom_level;
        return !(session.rt.y + session.rt.height <= zoom.ApplyInversedTo(spriteRect.getTop())
            || zoom.ApplyInversedTo(spriteRect.getBottom()) <= session.rt.y
            || session.rt.x + session.rt.width <= zoom.ApplyInversedTo(spriteRect.getLeft())
            || zoom.ApplyInversedTo(spriteRect.getRight()) <= session.rt.x);
    }

    void PaintOneEntity(PaintSession& session, EntityBase& entity)
    {
        if (!EntityPassesViewportFilters(session, entity))
            return;

        const auto entityPos = entity.getLocation();
        int32_t imageDirection = session.CurrentRotation;
        imageDirection <<= 3;
        imageDirection += entity.orientation;
        imageDirection &= 0x1F;

        session.CurrentlyDrawnEntity = &entity;
        session.SpritePosition.x = entityPos.x;
        session.SpritePosition.y = entityPos.y;
        session.InteractionType = ViewportInteractionItem::entity;

        switch (entity.type)
        {
            case EntityType::vehicle:
                PaintVehicle(session, *entity.cast<Vehicle>(), imageDirection);
                if (LightFx::ForVehiclesIsAvailable())
                    LightFx::AddLightsMagicVehicle(entity.cast<Vehicle>());
                break;
            case EntityType::guest:
                PaintGuest(session, *entity.cast<Guest>(), imageDirection);
                break;
            case EntityType::staff:
                PaintStaff(session, *entity.cast<Staff>(), imageDirection);
                break;
            case EntityType::steamParticle:
                PaintSteamParticle(session, *entity.cast<SteamParticle>());
                break;
            case EntityType::moneyEffect:
                PaintMoneyEffect(session, *entity.cast<MoneyEffect>());
                break;
            case EntityType::crashedVehicleParticle:
                PaintVehicleCrashParticle(session, *entity.cast<VehicleCrashParticle>());
                break;
            case EntityType::explosionCloud:
                PaintExplosionCloud(session, *entity.cast<ExplosionCloud>());
                break;
            case EntityType::crashSplash:
                PaintCrashSplashParticle(session, *entity.cast<CrashSplashParticle>());
                break;
            case EntityType::explosionFlare:
                PaintExplosionFlare(session, *entity.cast<ExplosionFlare>());
                break;
            case EntityType::jumpingFountain:
                PaintJumpingFountain(session, *entity.cast<JumpingFountain>(), imageDirection);
                break;
            case EntityType::balloon:
                PaintBalloon(session, *entity.cast<Balloon>(), imageDirection);
                break;
            case EntityType::duck:
                PaintDuck(session, *entity.cast<Duck>(), imageDirection);
                break;
            case EntityType::litter:
                PaintLitter(session, *entity.cast<Litter>(), imageDirection);
                break;
            default:
                assert(false);
                break;
        }
    }
}

/**
 * Paint Quadrant
 *  rct2: 0x0069E8B0
 */
void EntityPaintSetup(PaintSession& session, const CoordsXY& pos)
{
    PROFILED_FUNCTION();
    if (!MapIsLocationValid(pos))
        return;

    for (auto* entity : EntityTileList(pos))
        PaintOneEntity(session, *entity);
}

void EntityPaintSetupEntity(PaintSession& session, EntityBase& entity)
{
    PaintOneEntity(session, entity);
}
