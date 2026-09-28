/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "../../../GameState.h"
#include "../../../entity/EntityRegistry.h"
#include "../../../interface/Viewport.h"
#include "../../../ride/TrackPaint.h"
#include "../../../ride/Vehicle.h"
#include "../../Boundbox.h"
#include "../../Paint.h"
#include "../../support/WoodenSupports.h"
#include "../../tile_element/Segment.h"

#include <algorithm>
#include <cmath>

using namespace OpenRCT2;

static int16_t TopSpinSeatHeightOffset[] = {
    -10, -10, -9, -7, -4, -1, 2,  6,  11, 16, 21, 26, 31, 37, 42, 47, 52, 57, 61, 64, 67, 70, 72, 73,
    73,  73,  72, 70, 67, 64, 61, 57, 52, 47, 42, 37, 31, 26, 21, 16, 11, 6,  2,  -1, -4, -7, -9, -10,
};

/**
 * Can be calculated as Rounddown(34*sin(x)+0.5)
 * where x is in 7.5 deg segments.
 */
static int8_t TopSpinSeatPositionOffset[] = {
    0, 4,  9,  13,  17,  21,  24,  27,  29,  31,  33,  34,  34,  34,  33,  31,  29,  27,  24,  21,  17,  13,  9,  4,
    0, -3, -8, -12, -16, -20, -23, -26, -28, -30, -32, -33, -33, -33, -32, -30, -28, -26, -23, -20, -16, -12, -8, -3,
};

static float FirstPersonResolvePaintFrame(
    const PaintSession& session, uint8_t current,
    bool secondary, float frameCount)
{
    if (!session.FirstPersonPassengerInterpolation.enabled
        || !(frameCount > 0.0f))
        return float(current);

    const auto& interpolation =
        session.FirstPersonPassengerInterpolation;
    float before = float(
        secondary ? interpolation.secondaryBefore
                  : interpolation.primaryBefore);
    float after = float(
        secondary ? interpolation.secondaryAfter
                  : interpolation.primaryAfter);
    float delta = std::fmod(after - before, frameCount);
    if (delta > frameCount * 0.5f)
        delta -= frameCount;
    else if (delta < -frameCount * 0.5f)
        delta += frameCount;
    float frame = before + delta
        * std::clamp(interpolation.alpha, 0.0f, 1.0f);
    frame = std::fmod(frame, frameCount);
    if (frame < 0.0f)
        frame += frameCount;
    return frame;
}

static void PaintTopSpinRiders(
    PaintSession& session, const Vehicle& vehicle, ImageIndex seatImageIndex, const CoordsXYZ& seatCoords,
    const BoundBoxXYZ& bb)
{
    if (session.rt.zoom_level >= ZoomLevel{ 2 })
        return;

    for (int i = 0; i < 4; i++)
    {
        auto peepIndex = i * 2;
        if (vehicle.num_peeps > peepIndex)
        {
            auto imageIndex = seatImageIndex + ((i + 1) * 76);
            auto imageId = ImageId(
                imageIndex, vehicle.peep_tshirt_colours[peepIndex], vehicle.peep_tshirt_colours[peepIndex + 1]);
            const uint32_t previousSeatMask =
                session.FirstPersonPassengerSeatMask;
            session.FirstPersonPassengerSeatMask =
                uint32_t{ 3 } << peepIndex;
            PaintAddImageAsChild(session, imageId, seatCoords, bb);
            session.FirstPersonPassengerSeatMask =
                previousSeatMask;
        }
        else
        {
            break;
        }
    }
}

static void PaintTopSpinSeat(
    PaintSession& session, const Ride& ride, const RideObjectEntry& rideEntry, const Vehicle* vehicle, Direction direction,
    uint32_t armRotation, uint32_t seatRotation, const CoordsXYZ& offset, const BoundBoxXYZ& bb, ImageId stationColour)
{
    if (armRotation >= std::size(TopSpinSeatHeightOffset))
        return;

    const auto& carEntry = rideEntry.Cars[0];

    ImageIndex seatImageIndex;
    if (vehicle != nullptr && vehicle->restraints_position >= 64)
    {
        // Open Restraints
        seatImageIndex = carEntry.baseImageId + 64;
        seatImageIndex += (vehicle->restraints_position - 64) >> 6;
        seatImageIndex += direction * 3;
    }
    else
    {
        // Var_20 Rotation of seats
        seatImageIndex = carEntry.baseImageId;
        seatImageIndex += direction * 16;
        seatImageIndex += seatRotation;
    }

    auto seatCoords = offset;
    seatCoords.z += TopSpinSeatHeightOffset[armRotation];
    switch (direction)
    {
        case 0:
            seatCoords.x -= TopSpinSeatPositionOffset[armRotation];
            break;
        case 1:
            seatCoords.y += TopSpinSeatPositionOffset[armRotation];
            break;
        case 2:
            seatCoords.x += TopSpinSeatPositionOffset[armRotation];
            break;
        case 3:
            seatCoords.y -= TopSpinSeatPositionOffset[armRotation];
            break;
    }

    auto imageTemplate = ImageId(0, ride.vehicleColours[0].Body, ride.vehicleColours[0].Trim);
    if (stationColour != TrackStationColour)
    {
        imageTemplate = stationColour;
    }

    if (vehicle != nullptr)
    {
        const float armFrame = FirstPersonResolvePaintFrame(
            session, vehicle->flatRideAnimationFrame,
            false, 48.0f);
        const int32_t arm0 =
            int32_t(std::floor(armFrame)) % 48;
        const int32_t arm1 = (arm0 + 1) % 48;
        const float armAlpha =
            armFrame - std::floor(armFrame);
        const float seatPosition =
            float(TopSpinSeatPositionOffset[arm0])
            + (float(TopSpinSeatPositionOffset[arm1])
                - float(TopSpinSeatPositionOffset[arm0]))
                * armAlpha;
        const float seatHeight =
            float(TopSpinSeatHeightOffset[arm0])
            + (float(TopSpinSeatHeightOffset[arm1])
                - float(TopSpinSeatHeightOffset[arm0]))
                * armAlpha;

        float anchorX = float(offset.x);
        float anchorY = float(offset.y);
        switch (direction)
        {
            case 0: anchorX -= seatPosition; break;
            case 1: anchorY += seatPosition; break;
            case 2: anchorX += seatPosition; break;
            case 3: anchorY -= seatPosition; break;
        }
        const float seatFrame = FirstPersonResolvePaintFrame(
            session, vehicle->flatRideSecondaryAnimationFrame,
            true, 16.0f);
        constexpr float kTwoPi =
            6.28318530717958647692f;
        const float localPitch =
            seatFrame * (kTwoPi / 16.0f);
        const uint32_t seatMask =
            vehicle->num_seats >= 32
            ? 0xffffffffu
            : ((uint32_t{ 1 } << vehicle->num_seats) - 1u);
        PaintSessionPublishFirstPersonPassengerLocalAnchor(
            session, *vehicle, seatMask,
            anchorX, anchorY,
            float(offset.z) + seatHeight,
            false, 0.0f, 0.0f, 0.0f,
            true, localPitch);
    }

    PaintAddImageAsChild(session, imageTemplate.WithIndex(seatImageIndex), seatCoords, bb);
    if (vehicle != nullptr)
    {
        PaintTopSpinRiders(session, *vehicle, seatImageIndex, seatCoords, bb);
    }
}

static void PaintTopSpinVehicle(
    PaintSession& session, int32_t al, int32_t cl, const Ride& ride, uint8_t direction, int32_t height, ImageId stationColour)
{
    const auto* rideEntry = GetRideEntryByIndex(ride.subtype);
    if (rideEntry == nullptr)
        return;

    const auto& carEntry = rideEntry->Cars[0];

    height += 3;
    uint8_t seatRotation = 0;
    uint8_t armRotation = 0;
    auto* vehicle = getGameState().entities.getEntity<Vehicle>(ride.vehicles[0]);
    if (ride.flags.has(RideFlag::onTrack) && vehicle != nullptr)
    {
        session.InteractionType = ViewportInteractionItem::entity;
        session.CurrentlyDrawnEntity = vehicle;

        armRotation = vehicle->flatRideAnimationFrame;
        seatRotation = vehicle->flatRideSecondaryAnimationFrame;
    }

    int32_t armImageOffset = armRotation;
    if (direction & 2)
    {
        armImageOffset = -armImageOffset;
        if (armImageOffset != 0)
            armImageOffset += 48;
    }

    CoordsXYZ offset = { al, cl, height };
    BoundBoxXYZ bb = { { al + 16, cl + 16, height }, { 24, 24, 90 } };

    auto supportImageTemplate = ImageId(0, ride.trackColours[0].main, ride.trackColours[0].supports);
    auto armImageTemplate = ImageId(0, ride.trackColours[0].main, ride.trackColours[0].additional);
    if (stationColour != TrackStationColour)
    {
        supportImageTemplate = stationColour;
        armImageTemplate = supportImageTemplate;
    }

    // Left back bottom support
    auto imageIndex = carEntry.baseImageId + 572 + ((direction & 1) << 1);
    PaintAddImageAsParent(session, supportImageTemplate.WithIndex(imageIndex), offset, bb);

    // Left hand arm
    imageIndex = carEntry.baseImageId + 380 + armImageOffset + ((direction & 1) * 48);
    PaintAddImageAsChild(session, armImageTemplate.WithIndex(imageIndex), offset, bb);

    // Seat
    PaintTopSpinSeat(session, ride, *rideEntry, vehicle, direction, armRotation, seatRotation, offset, bb, stationColour);

    // Right hand arm
    imageIndex = carEntry.baseImageId + 476 + armImageOffset + ((direction & 1) * 48);
    PaintAddImageAsChild(session, armImageTemplate.WithIndex(imageIndex), offset, bb);

    // Right back bottom support
    imageIndex = carEntry.baseImageId + 573 + ((direction & 1) << 1);
    PaintAddImageAsChild(session, supportImageTemplate.WithIndex(imageIndex), offset, bb);

    session.CurrentlyDrawnEntity = nullptr;
    session.InteractionType = ViewportInteractionItem::ride;
}

static void PaintTopSpin(
    PaintSession& session, const Ride& ride, uint8_t trackSequence, uint8_t direction, int32_t height,
    const TrackElement& trackElement, SupportType supportType)
{
    trackSequence = kTrackMap3x3[direction][trackSequence];

    int32_t edges = kEdges3x3[trackSequence];

    auto stationColour = GetStationColourScheme(session, trackElement);
    WoodenASupportsPaintSetupRotated(
        session, WoodenSupportType::truss, WoodenSupportSubType::neSw, direction, height, stationColour);

    const StationObject* stationObject = ride.getStationObject();

    TrackPaintUtilPaintFloor(session, edges, session.TrackColours, height, kFloorSpritesMulch, stationObject);

    TrackPaintUtilPaintFences(
        session, edges, session.MapPosition, trackElement, ride, stationColour, height, kFenceSpritesRope,
        session.CurrentRotation);

    switch (trackSequence)
    {
        case 1:
            PaintTopSpinVehicle(session, 32, 32, ride, direction, height, stationColour);
            break;
        case 3:
            PaintTopSpinVehicle(session, 32, -32, ride, direction, height, stationColour);
            break;
        case 5:
            PaintTopSpinVehicle(session, 0, -32, ride, direction, height, stationColour);
            break;
        case 6:
            PaintTopSpinVehicle(session, -32, 32, ride, direction, height, stationColour);
            break;
        case 7:
            PaintTopSpinVehicle(session, -32, -32, ride, direction, height, stationColour);
            break;
        case 8:
            PaintTopSpinVehicle(session, -32, 0, ride, direction, height, stationColour);
            break;
    }

    int32_t cornerSegments = 0;
    switch (trackSequence)
    {
        case 1:
            // top
            cornerSegments = EnumsToFlags(PaintSegment::top, PaintSegment::topLeft, PaintSegment::topRight);
            break;
        case 3:
            // right
            cornerSegments = EnumsToFlags(PaintSegment::topRight, PaintSegment::right, PaintSegment::bottomRight);
            break;
        case 6:
            // left
            cornerSegments = EnumsToFlags(PaintSegment::topLeft, PaintSegment::left, PaintSegment::bottomLeft);
            break;
        case 7:
            // bottom
            cornerSegments = EnumsToFlags(PaintSegment::bottomLeft, PaintSegment::bottom, PaintSegment::bottomRight);
            break;
    }

    PaintUtilSetSegmentSupportHeight(session, cornerSegments, height + 2, 0x20);
    PaintUtilSetSegmentSupportHeight(session, kSegmentsAll & ~cornerSegments, 0xFFFF, 0);
    PaintUtilSetGeneralSupportHeight(session, height + 112);
}

TrackPaintFunction GetTrackPaintFunctionTopspin(TrackElemType trackType)
{
    if (trackType != TrackElemType::flatTrack3x3)
    {
        return TrackPaintFunctionDummy;
    }

    return PaintTopSpin;
}
