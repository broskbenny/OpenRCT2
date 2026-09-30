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
#include "../../track/Support.h"

using namespace OpenRCT2;

static constexpr TunnelGroup kTunnelGroup = TunnelGroup::square;

static void PaintFacility(
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

    const auto lengthZ = trackElement.getClearanceZ() - trackElement.getBaseZ() - 3;
    const CoordsXYZ offset(0, 0, height);
    const BoundBoxXYZ bb = (direction == 0 || direction == 3) ? BoundBoxXYZ{ { 2, 2, height + lengthZ }, { 28, 28, 1 } }
                                                              : BoundBoxXYZ{ { 2, 2, height }, { 28, 8, lengthZ } };

    auto imageTemplate = session.TrackColours;
    auto imageIndex = firstCarEntry->baseImageId + ((direction + 2) & 3);
    auto imageId = imageTemplate.WithIndex(imageIndex);

    FirstPersonPaintSemanticTransform semanticTransform{};
    semanticTransform.origin = {
        float(session.MapPosition.x), float(session.MapPosition.y), 0.0f
    };
    const uint32_t bodyGroup = PaintSessionBeginFirstPersonSemanticArtworkGroup(session);
    PaintSessionAddFirstPersonSemanticOrientedBox(
        session, FirstPersonPaintSemanticRole::structureBody,
        semanticTransform,
        { 2.0f, 2.0f, float(height) },
        { 30.0f, 30.0f, float(height + lengthZ) },
        imageId, offset, bodyGroup);

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
            PaintAddImageAsChildRotated(session, direction, imageId, offset, bb);
        }
    }
    else
    {
        FirstPersonPaintSemanticScope bodyScope(
            session, FirstPersonPaintSemanticRole::structureBody, bodyGroup);
        PaintAddImageAsParentRotated(session, direction, imageId, offset, bb);
    }

    // The additional door image is a horizontal top surface in the native
    // painter. Keep it as artwork on the authoritative roof plane instead of
    // allowing its 2-D sorting slab to become physical geometry.
    if (direction == 1 || direction == 2)
    {
        const auto roofImage = imageId.WithIndexOffset(direction == 1 ? 2 : 4);
        const uint32_t roofGroup = PaintSessionBeginFirstPersonSemanticArtworkGroup(session);
        PaintSessionAddFirstPersonSemanticOrientedQuad(
            session, FirstPersonPaintSemanticRole::structureFloor,
            FirstPersonPaintSemanticPrimitiveKind::plane,
            semanticTransform,
            { {
                { 2.0f, 2.0f, float(height + lengthZ) },
                { 30.0f, 2.0f, float(height + lengthZ) },
                { 30.0f, 30.0f, float(height + lengthZ) },
                { 2.0f, 30.0f, float(height + lengthZ) },
            } },
            roofImage, offset, roofGroup, false, false);
        FirstPersonPaintSemanticScope roofScope(
            session, FirstPersonPaintSemanticRole::structureFloor, roofGroup);
        PaintAddImageAsParent(
            session, roofImage, offset,
            { { 2, 2, height + lengthZ }, { 28, 28, 1 } });
    }

    PaintUtilSetSegmentSupportHeight(session, kSegmentsAll, 0xFFFF, 0);
    PaintUtilSetGeneralSupportHeight(session, height + kDefaultGeneralSupportHeight);

    if (direction == 1 || direction == 2)
        PaintUtilPushTunnelRotated(session, direction, height, kTunnelGroup, TunnelSubType::flat);
}

/* 0x00762D44 */
TrackPaintFunction GetTrackPaintFunctionFacility(TrackElemType trackType)
{
    switch (trackType)
    {
        case TrackElemType::flatTrack1x1A:
            return PaintFacility;
        default:
            return TrackPaintFunctionDummy;
    }
}
