/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "../../../ride/Ride.h"
#include "../../../ride/TrackPaint.h"
#include "../../../ride/ted/TrackElemType.h"
#include "../../../world/tile_element/TileElement.h"
#include "../../../world/tile_element/TrackElement.h"
#include "../../Paint.h"
#include "../../support/WoodenSupports.h"
#include "../../tile_element/Segment.h"
#include "../../track/Support.h"

using namespace OpenRCT2;

enum
{
    SprObservationTowerSegmentBase = 14986,
    SprObservationTowerSegment = 14987,
    SprObservationTowerSegmentTop = 14988,
};

static PaintStruct* PaintSemanticTowerSectionAsParent(
    PaintSession& session, ImageId image,
    const CoordsXYZ& offset, const BoundBoxXYZ& physicalBox,
    uint64_t localHullKey, uint16_t repetitionIndex = 0)
{
    auto low = physicalBox.offset;
    auto high = physicalBox.offset + physicalBox.length;
    if (physicalBox.length.x == 0)
        high.x = low.x + 1;
    if (physicalBox.length.y == 0)
        high.y = low.y + 1;
    if (physicalBox.length.z == 0)
        high.z = low.z + 1;
    const uint32_t group =
        PaintSessionBeginFirstPersonSemanticArtworkGroup(session);
    PaintSessionAddFirstPersonSemanticBox(
        session, FirstPersonPaintSemanticRole::towerSection,
        low, high, image, offset, group,
        localHullKey, repetitionIndex, true);
    FirstPersonPaintSemanticScope scope(
        session, FirstPersonPaintSemanticRole::towerSection,
        group);
    auto* result =
        PaintAddImageAsParent(
            session, image, offset, physicalBox);
    session.FirstPersonSemanticArtworkGroup = 0;
    return result;
}

/** rct2: 0x0070DD6C */
static void PaintObservationTowerBase(
    PaintSession& session, const Ride& ride, uint8_t trackSequence, uint8_t direction, int32_t height,
    const TrackElement& trackElement, SupportType supportType)
{
    trackSequence = kTrackMap3x3[direction][trackSequence];

    int32_t edges = kEdges3x3[trackSequence];
    CoordsXY position = session.MapPosition;

    WoodenASupportsPaintSetupRotated(
        session, WoodenSupportType::truss, WoodenSupportSubType::neSw, direction, height,
        GetStationColourScheme(session, trackElement));

    const StationObject* stationObject = ride.getStationObject();

    TrackPaintUtilPaintFloor(session, edges, session.SupportColours, height, kFloorSpritesTileDiamond, stationObject);

    TrackPaintUtilPaintFences(
        session, edges, position, trackElement, ride, session.TrackColours, height, kFenceSpritesPicketDuplicate,
        session.CurrentRotation);

    if (trackSequence == 0)
    {
        auto imageId = session.TrackColours.WithIndex(SprObservationTowerSegmentBase);
        PaintSemanticTowerSectionAsParent(session, imageId, { 0, 0, height }, { { 8, 8, height + 3 }, { 2, 2, 27 } }, uint64_t(SprObservationTowerSegment));

        imageId = session.TrackColours.WithIndex(SprObservationTowerSegment);
        PaintSemanticTowerSectionAsParent(session, imageId, { 0, 0, height + 32 }, { { 8, 8, height + 32 }, { 2, 2, 30 } }, uint64_t(SprObservationTowerSegment));

        imageId = session.TrackColours.WithIndex(SprObservationTowerSegment);
        PaintSemanticTowerSectionAsParent(session, imageId, { 0, 0, height + 64 }, { { 8, 8, height + 64 }, { 2, 2, 30 } }, uint64_t(SprObservationTowerSegment));

        PaintUtilSetVerticalTunnel(session, height + 96);
        PaintUtilSetSegmentSupportHeight(session, kSegmentsAll, 0xFFFF, 0);

        PaintUtilSetGeneralSupportHeight(session, height + 96);

        return;
    }

    int32_t blockedSegments = 0;
    switch (trackSequence)
    {
        case 1:
            blockedSegments = EnumsToFlags(
                PaintSegment::left, PaintSegment::topLeft, PaintSegment::top, PaintSegment::topRight, PaintSegment::right);
            break;
        case 2:
            blockedSegments = EnumsToFlags(PaintSegment::top, PaintSegment::topRight, PaintSegment::right);
            break;
        case 3:
            blockedSegments = EnumsToFlags(
                PaintSegment::top, PaintSegment::topRight, PaintSegment::right, PaintSegment::bottomRight,
                PaintSegment::bottom);
            break;
        case 4:
            blockedSegments = EnumsToFlags(PaintSegment::top, PaintSegment::topLeft, PaintSegment::left);
            break;
        case 5:
            blockedSegments = EnumsToFlags(PaintSegment::right, PaintSegment::bottomRight, PaintSegment::bottom);
            break;
        case 6:
            blockedSegments = EnumsToFlags(
                PaintSegment::top, PaintSegment::topLeft, PaintSegment::left, PaintSegment::bottomLeft, PaintSegment::bottom);
            break;
        case 7:
            blockedSegments = EnumsToFlags(
                PaintSegment::left, PaintSegment::bottomLeft, PaintSegment::bottom, PaintSegment::bottomRight,
                PaintSegment::right);
            break;
        case 8:
            blockedSegments = EnumsToFlags(PaintSegment::left, PaintSegment::bottomLeft, PaintSegment::bottom);
            break;
    }
    PaintUtilSetSegmentSupportHeight(session, blockedSegments, 0xFFFF, 0);
    PaintUtilSetSegmentSupportHeight(session, kSegmentsAll & ~blockedSegments, height + 2, 0x20);
    PaintUtilSetGeneralSupportHeight(session, height + kDefaultGeneralSupportHeight);
}

/** rct2: 0x0070DD7C */
static void PaintObservationTowerSection(
    PaintSession& session, const Ride& ride, uint8_t trackSequence, uint8_t direction, int32_t height,
    const TrackElement& trackElement, SupportType supportType)
{
    if (trackSequence == 1)
    {
        return;
    }

    auto imageId = session.TrackColours.WithIndex(SprObservationTowerSegment);
    PaintSemanticTowerSectionAsParent(session, imageId, { 0, 0, height }, { { 8, 8, height }, { 2, 2, 30 } }, uint64_t(SprObservationTowerSegment));

    const TileElement* nextTileElement = reinterpret_cast<const TileElement*>(&trackElement) + 1;
    if (trackElement.isLastForTile() || trackElement.getClearanceZ() != nextTileElement->getBaseZ())
    {
        imageId = session.TrackColours.WithIndex(SprObservationTowerSegmentTop);
        PaintAddImageAsChild(session, imageId, { 0, 0, height }, { { 8, 8, height }, { 2, 2, 30 } });
    }

    PaintUtilSetSegmentSupportHeight(session, kSegmentsAll, 0xFFFF, 0);

    PaintUtilSetVerticalTunnel(session, height + 32);
    PaintUtilSetGeneralSupportHeight(session, height + kDefaultGeneralSupportHeight);
}

/**
 * rct2: 0x0070DC5C
 */
TrackPaintFunction GetTrackPaintFunctionObservationTower(TrackElemType trackType)
{
    switch (trackType)
    {
        case TrackElemType::towerBase:
            return PaintObservationTowerBase;

        case TrackElemType::towerSection:
            return PaintObservationTowerSection;
        default:
            return TrackPaintFunctionDummy;
    }
}
