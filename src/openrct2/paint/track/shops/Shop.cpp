/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "../../../ride/Ride.h"
#include "../../../ride/RideEntry.h"
#include "../../../ride/TrackPaint.h"
#include "../../../world/tile_element/TrackElement.h"
#include "../../Boundbox.h"
#include "../../Paint.h"
#include "../../support/WoodenSupports.hpp"
#include "../../tile_element/Segment.h"

using namespace OpenRCT2;

static constexpr TunnelGroup kTunnelGroup = TunnelGroup::square;

static void PaintShop(
    PaintSession& session, const Ride& ride, uint8_t trackSequence, uint8_t direction, int32_t height,
    const TrackElement& trackElement, SupportType supportType)
{
    bool hasSupports = DrawSupportForSequenceA<TrackElemType::flatTrack1x1A>(
        session, supportType.wooden, trackSequence, direction, height, GetShopSupportColourScheme(session, trackElement));

    auto rideEntry = ride.getRideEntry();
    if (rideEntry == nullptr)
        return;

    auto firstCarEntry = &rideEntry->Cars[0];
    if (firstCarEntry == nullptr)
        return;

    CoordsXYZ offset(0, 0, height);
    BoundBoxXYZ bb = { { 2, 2, height }, { 28, 28, trackElement.getClearanceZ() - trackElement.getBaseZ() - 3 } };

    auto imageFlags = session.TrackColours.WithoutSecondary();
    auto imageIndex = firstCarEntry->baseImageId + direction;
    const auto bodyImageId = imageFlags.WithIndex(imageIndex);
    const float bodyTop = float(trackElement.getClearanceZ() - trackElement.getBaseZ() - 3 + height);
    FirstPersonPaintSemanticTransform semanticTransform{};
    semanticTransform.origin = {
        float(session.MapPosition.x), float(session.MapPosition.y), 0.0f
    };
    const uint32_t bodyGroup = PaintSessionBeginFirstPersonSemanticArtworkGroup(session);
    PaintSessionAddFirstPersonSemanticOrientedBox(
        session, FirstPersonPaintSemanticRole::structureBody,
        semanticTransform,
        { 2.0f, 2.0f, float(height) },
        { 30.0f, 30.0f, bodyTop },
        bodyImageId, offset, bodyGroup);

    if (hasSupports)
    {
        auto foundationImageTemplate = GetShopSupportColourScheme(session, trackElement);
        auto foundationImageIndex = (direction & 1) ? SPR_FLOOR_PLANKS_90_DEG : SPR_FLOOR_PLANKS;
        auto foundationImageId = foundationImageTemplate.WithIndex(foundationImageIndex);
        const uint32_t floorGroup = PaintSessionBeginFirstPersonSemanticArtworkGroup(session);
        PaintSessionAddFirstPersonSemanticOrientedQuad(
            session, FirstPersonPaintSemanticRole::structureFloor,
            FirstPersonPaintSemanticPrimitiveKind::footprint,
            semanticTransform,
            { {
                { 2.0f, 2.0f, float(height) },
                { 30.0f, 2.0f, float(height) },
                { 30.0f, 30.0f, float(height) },
                { 2.0f, 30.0f, float(height) },
            } },
            foundationImageId, offset, floorGroup, false, true);
        {
            FirstPersonPaintSemanticScope floorScope(
                session, FirstPersonPaintSemanticRole::structureFloor, floorGroup);
            PaintAddImageAsParent(session, foundationImageId, offset, bb);
        }
        {
            FirstPersonPaintSemanticScope bodyScope(
                session, FirstPersonPaintSemanticRole::structureBody, bodyGroup);
            PaintAddImageAsChild(session, bodyImageId, offset, bb);
        }
    }
    else
    {
        FirstPersonPaintSemanticScope bodyScope(
            session, FirstPersonPaintSemanticRole::structureBody, bodyGroup);
        PaintAddImageAsParent(session, bodyImageId, offset, bb);
    }

    PaintUtilSetSegmentSupportHeight(session, kSegmentsAll, 0xFFFF, 0);
    PaintUtilSetGeneralSupportHeight(session, height + 48);

    if (direction == 1 || direction == 2)
        PaintUtilPushTunnelRotated(session, direction, height, kTunnelGroup, TunnelSubType::flat);
}

TrackPaintFunction GetTrackPaintFunctionShop(TrackElemType trackType)
{
    switch (trackType)
    {
        case TrackElemType::flatTrack1x1A:
        case TrackElemType::flatTrack1x1B:
            return PaintShop;
        default:
            return TrackPaintFunctionDummy;
    }
}
