/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "../Paint.h"

#include "../../Context.h"
#include "../../GameState.h"
#include "../../SpriteIds.h"
#include "../../drawing/LightFX.h"
#include "../../drawing/PaletteIndex.h"
#include "../../drawing/ScrollingText.h"
#include "../../interface/Viewport.h"
#include "../../localisation/StringIds.h"
#include "../../object/EntranceObject.h"
#include "../../object/ObjectManager.h"
#include "../../object/StationObject.h"
#include "../../profiling/Profiling.h"
#include "../../ride/TrackDesign.h"
#include "../../world/Footpath.h"
#include "../../world/tile_element/EntranceElement.h"
#include "../support/WoodenSupports.h"
#include "Paint.Entrance.h"
#include "Paint.TileElement.h"
#include "Segment.h"

#include <array>
#include <utility>

using namespace OpenRCT2;
using namespace OpenRCT2::Drawing;

using OpenRCT2::Drawing::LightFx::LightType;

static void PaintRideEntranceExitScrollingText(
    PaintSession& session, const EntranceElement& entranceEl, const StationObject& stationObj, Direction direction,
    int32_t height)
{
    PROFILED_FUNCTION();

    if (stationObj.ScrollingMode == kScrollingModeNone)
        return;

    if (entranceEl.getEntranceType() == EntranceType::rideExit)
        return;

    const auto* ride = GetRide(entranceEl.getRideIndex());
    if (ride == nullptr)
        return;

    u8string bannerText;
    if (ride->status == RideStatus::open && !ride->flags.has(RideFlag::brokenDown))
    {
        bannerText = ScrollingText::kRideBannerColourPrefix + ride->getName();
    }
    else
    {
        bannerText = LanguageGetString(STR_RIDE_ENTRANCE_CLOSED);
    }

    PaintAddImageAsChild(
        session, ScrollingText::setup(session, bannerText, stationObj.ScrollingMode, PaletteIndex::transparent),
        { 0, 0, height + stationObj.Height }, { { 2, 2, height + stationObj.Height }, { 28, 28, 51 } });
}

static void PaintRideEntranceExitLightEffects(PaintSession& session, int32_t height, const EntranceElement& entranceEl)
{
    PROFILED_FUNCTION();

    if (LightFx::IsAvailable())
    {
        if (entranceEl.getEntranceType() == EntranceType::rideEntrance)
        {
            LightFx::Add3DLightMagicFromDrawingTile(session.MapPosition, 0, 0, height + 45, LightType::lantern3);
        }

        switch (entranceEl.getDirection())
        {
            case 0:
                LightFx::Add3DLightMagicFromDrawingTile(session.MapPosition, 16, 0, height + 16, LightType::lantern2);
                break;
            case 1:
                LightFx::Add3DLightMagicFromDrawingTile(session.MapPosition, 0, -16, height + 16, LightType::lantern2);
                break;
            case 2:
                LightFx::Add3DLightMagicFromDrawingTile(session.MapPosition, -16, 0, height + 16, LightType::lantern2);
                break;
            case 3:
                LightFx::Add3DLightMagicFromDrawingTile(session.MapPosition, 0, 16, height + 16, LightType::lantern2);
                break;
        }
    }
}

static void PaintRideEntranceExit(PaintSession& session, uint8_t direction, int32_t height, const EntranceElement& entranceEl)
{
    PROFILED_FUNCTION();

    auto rideIndex = entranceEl.getRideIndex();
    if ((session.ViewFlags & VIEWPORT_FLAG_HIGHLIGHT_PATH_ISSUES)
        || (gTrackDesignSaveMode && rideIndex != gTrackDesignSaveRideIndex))
    {
        return;
    }

    auto ride = GetRide(rideIndex);
    if (ride == nullptr)
    {
        return;
    }

    auto stationObj = ride->getStationObject();
    if (stationObj == nullptr || stationObj->entranceBackIndex == kImageIndexUndefined)
    {
        return;
    }

    session.InteractionType = ViewportInteractionItem::ride;

    PaintRideEntranceExitLightEffects(session, height, entranceEl);

    auto hasGlass = stationObj->Flags.has(StationObjectFlag::isTransparent);
    auto colourPrimary = ride->trackColours[0].main;
    auto imageTemplate = ImageId(0);
    ImageId glassImageTemplate;
    if (hasGlass)
    {
        glassImageTemplate = ImageId().WithTransparency(colourPrimary);
    }

    if (entranceEl.isGhost())
    {
        session.InteractionType = ViewportInteractionItem::none;
        imageTemplate = ImageId().WithRemap(FilterPaletteID::paletteGhost);
    }
    else if (session.SelectedElement == reinterpret_cast<const TileElement*>(&entranceEl))
    {
        imageTemplate = ImageId().WithRemap(FilterPaletteID::paletteGhost);
    }
    else
    {
        if (stationObj->Flags.has(StationObjectFlag::hasPrimaryColour))
        {
            imageTemplate = imageTemplate.WithPrimary(colourPrimary);
        }
        if (stationObj->Flags.has(StationObjectFlag::hasSecondaryColour))
        {
            auto colourSecondary = ride->trackColours[0].additional;
            imageTemplate = imageTemplate.WithSecondary(colourSecondary);
        }
    }

    // Format modified to stop repeated code

    // Each entrance is split into 2 images for drawing
    // Certain entrance styles have another 2 images to draw for coloured windows

    auto isExit = entranceEl.getEntranceType() == EntranceType::rideExit;

    // The native painter knows this is a gate, not a pair of sorting slabs.
    // Publish that structure before its directional artwork is flattened into
    // PaintStructs. The opening remains physically open between the two posts.
    FirstPersonPaintSemanticTransform structureTransform{};
    structureTransform.origin = {
        float(session.MapPosition.x + 16),
        float(session.MapPosition.y + 16),
        0.0f,
    };
    const auto worldDirection =
        static_cast<uint8_t>(entranceEl.getDirection()) & 3u;
    switch (worldDirection)
    {
        case 1:
            structureTransform.axisX = { 0.0f, 1.0f, 0.0f };
            structureTransform.axisY = { -1.0f, 0.0f, 0.0f };
            break;
        case 2:
            structureTransform.axisX = { -1.0f, 0.0f, 0.0f };
            structureTransform.axisY = { 0.0f, -1.0f, 0.0f };
            break;
        case 3:
            structureTransform.axisX = { 0.0f, -1.0f, 0.0f };
            structureTransform.axisY = { 1.0f, 0.0f, 0.0f };
            break;
        default:
            break;
    }

    // Back artwork owns the stable frame body.
    ImageIndex backImageIndex = (isExit ? stationObj->exitBackIndex : stationObj->entranceBackIndex) + direction;
    const auto backImage = imageTemplate.WithIndex(backImageIndex);
    const uint32_t bodyGroup = PaintSessionBeginFirstPersonSemanticArtworkGroup(session);
    for (const auto& bounds : std::array{
             std::pair{ FirstPersonPaintSemanticVec3{ -14.0f, -4.0f, float(height) },
                        FirstPersonPaintSemanticVec3{ -9.0f, 4.0f, float(height + 30) } },
             std::pair{ FirstPersonPaintSemanticVec3{ 9.0f, -4.0f, float(height) },
                        FirstPersonPaintSemanticVec3{ 14.0f, 4.0f, float(height + 30) } },
             std::pair{ FirstPersonPaintSemanticVec3{ -9.0f, -4.0f, float(height + 22) },
                        FirstPersonPaintSemanticVec3{ 9.0f, 4.0f, float(height + 30) } },
         })
    {
        PaintSessionAddFirstPersonSemanticOrientedBox(
            session, FirstPersonPaintSemanticRole::structureBody,
            structureTransform, bounds.first, bounds.second,
            backImage, { 0, 0, height }, bodyGroup);
    }
    {
        FirstPersonPaintSemanticScope bodyScope(
            session, FirstPersonPaintSemanticRole::structureBody, bodyGroup);
        PaintAddImageAsParentRotated(
            session, direction, backImage, { 0, 0, height },
            { { 2, 2, height }, { 28, 8, 30 } });
    }

    if (hasGlass)
    {
        ImageIndex backGlassImageIndex = (isExit ? stationObj->exitBackGlassIndex : stationObj->entranceBackGlassIndex)
            + direction;
        const auto backGlass = glassImageTemplate.WithIndex(backGlassImageIndex);
        const uint32_t glassGroup = PaintSessionBeginFirstPersonSemanticArtworkGroup(session);
        PaintSessionAddFirstPersonSemanticOrientedQuad(
            session, FirstPersonPaintSemanticRole::structureBody,
            FirstPersonPaintSemanticPrimitiveKind::plane,
            structureTransform,
            { {
                { -14.0f, -4.05f, float(height) },
                { 14.0f, -4.05f, float(height) },
                { 14.0f, -4.05f, float(height + 30) },
                { -14.0f, -4.05f, float(height + 30) },
            } },
            backGlass, { 0, 0, height }, glassGroup, true, false);
        FirstPersonPaintSemanticScope glassScope(
            session, FirstPersonPaintSemanticRole::structureBody, glassGroup);
        PaintAddImageAsChildRotated(
            session, direction, backGlass, { 0, 0, height },
            { { 2, 2, height }, { 28, 8, 30 } });
    }

    // Front layers are appearance attached to the frame, not another solid.
    const auto frontBoundBoxZ = isExit ? 1 : 17;
    ImageIndex frontImageIndex = (isExit ? stationObj->exitFrontIndex : stationObj->entranceFrontIndex) + direction;
    const auto frontImage = imageTemplate.WithIndex(frontImageIndex);
    const uint32_t frontGroup = PaintSessionBeginFirstPersonSemanticArtworkGroup(session);
    PaintSessionAddFirstPersonSemanticOrientedQuad(
        session, FirstPersonPaintSemanticRole::structureBody,
        FirstPersonPaintSemanticPrimitiveKind::plane,
        structureTransform,
        { {
            { -14.0f, 4.05f, float(height) },
            { 14.0f, 4.05f, float(height) },
            { 14.0f, 4.05f, float(height + 30) },
            { -14.0f, 4.05f, float(height + 30) },
        } },
        frontImage, { 0, 0, height }, frontGroup, true, false);
    {
        FirstPersonPaintSemanticScope frontScope(
            session, FirstPersonPaintSemanticRole::structureBody, frontGroup);
        PaintAddImageAsParent(
            session, frontImage, { 0, 0, height },
            { { 2, 2, height + 30 }, { 28, 28, frontBoundBoxZ } });
    }
    if (hasGlass)
    {
        ImageIndex frontGlassImageIndex = (isExit ? stationObj->exitFrontGlassIndex : stationObj->entranceFrontGlassIndex)
            + direction;
        const auto frontGlass = glassImageTemplate.WithIndex(frontGlassImageIndex);
        const uint32_t frontGlassGroup = PaintSessionBeginFirstPersonSemanticArtworkGroup(session);
        PaintSessionAddFirstPersonSemanticOrientedQuad(
            session, FirstPersonPaintSemanticRole::structureBody,
            FirstPersonPaintSemanticPrimitiveKind::plane,
            structureTransform,
            { {
                { -14.0f, 4.10f, float(height) },
                { 14.0f, 4.10f, float(height) },
                { 14.0f, 4.10f, float(height + 30) },
                { -14.0f, 4.10f, float(height + 30) },
            } },
            frontGlass, { 0, 0, height }, frontGlassGroup, true, false);
        FirstPersonPaintSemanticScope glassScope(
            session, FirstPersonPaintSemanticRole::structureBody, frontGlassGroup);
        PaintAddImageAsChild(
            session, frontGlass, { 0, 0, height },
            { { 2, 2, height + 30 }, { 28, 28, frontBoundBoxZ } });
    }

    PaintUtilPushTunnelRotated(session, direction, height, TunnelType::squareFlat);

    if (!entranceEl.isGhost())
        PaintRideEntranceExitScrollingText(session, entranceEl, *stationObj, direction, height);

    auto supportsImageTemplate = imageTemplate;
    if (!entranceEl.isGhost())
    {
        supportsImageTemplate = ImageId().WithPrimary(OpenRCT2::Drawing::Colour::saturatedBrown);
    }
    WoodenASupportsPaintSetupRotated(
        session, WoodenSupportType::truss, WoodenSupportSubType::neSw, direction, height, supportsImageTemplate);

    height += isExit ? 40 : 56;
    PaintUtilSetSegmentSupportHeight(session, kSegmentsAll, 0xFFFF, 0);
    PaintUtilSetGeneralSupportHeight(session, height);
}

static FirstPersonPaintSemanticTransform
    MakeFirstPersonParkEntranceTransform(
        const PaintSession& session,
        const EntranceElement& entranceEl)
{
    FirstPersonPaintSemanticTransform transform{};
    transform.origin = {
        float(session.MapPosition.x + kCoordsXYHalfTile),
        float(session.MapPosition.y + kCoordsXYHalfTile),
        0.0f,
    };
    switch (static_cast<uint8_t>(
                entranceEl.getDirection()) & 3u)
    {
        case 1:
            transform.axisX = { 0.0f, 1.0f, 0.0f };
            transform.axisY = { -1.0f, 0.0f, 0.0f };
            break;
        case 2:
            transform.axisX = { -1.0f, 0.0f, 0.0f };
            transform.axisY = { 0.0f, -1.0f, 0.0f };
            break;
        case 3:
            transform.axisX = { 0.0f, -1.0f, 0.0f };
            transform.axisY = { 1.0f, 0.0f, 0.0f };
            break;
        default:
            break;
    }
    return transform;
}

static uint32_t PublishFirstPersonParkEntranceGatePlane(
    PaintSession& session, const EntranceElement& entranceEl,
    ImageId image, const CoordsXYZ& artworkOffset,
    uint32_t artworkGroup,
    FirstPersonPaintSemanticRole role =
        FirstPersonPaintSemanticRole::structureBody)
{
    if (session.FirstPersonSemanticComponentSink == nullptr)
        return 0;
    const float bottom = float(entranceEl.getBaseZ());
    const float top = float(entranceEl.getClearanceZ());
    return PaintSessionAddFirstPersonSemanticOrientedQuad(
        session, role,
        FirstPersonPaintSemanticPrimitiveKind::plane,
        MakeFirstPersonParkEntranceTransform(
            session, entranceEl),
        { {
            { -float(kCoordsXYHalfTile), 0.0f, bottom },
            { float(kCoordsXYHalfTile), 0.0f, bottom },
            { float(kCoordsXYHalfTile), 0.0f, top },
            { -float(kCoordsXYHalfTile), 0.0f, top },
        } },
        image, artworkOffset, artworkGroup,
        true, false);
}

static uint32_t PublishFirstPersonParkEntranceCentreBody(
    PaintSession& session, const EntranceElement& entranceEl,
    ImageId image, const CoordsXYZ& artworkOffset,
    uint32_t artworkGroup)
{
    if (session.FirstPersonSemanticComponentSink == nullptr)
        return 0;

    // Match the native centre PaintStruct bounds exactly:
    //   origin=(2,2,height+32), size=(28,28,47).
    // In the entrance-local frame that is a 28x28x47 upper body spanning
    // z=[base+32, base+79]. The walk-through opening below is therefore
    // physical empty space instead of transparency on a full-height plane.
    const float bottom = float(entranceEl.getBaseZ());
    const auto transform =
        MakeFirstPersonParkEntranceTransform(
            session, entranceEl);
    return PaintSessionAddFirstPersonSemanticOrientedBox(
        session, FirstPersonPaintSemanticRole::structureBody,
        transform,
        { -14.0f, -14.0f, bottom + 32.0f },
        { 14.0f, 14.0f, bottom + 79.0f },
        image, artworkOffset, artworkGroup,
        0, 0, false, false);
}

static uint32_t PublishFirstPersonParkEntranceSideBody(
    PaintSession& session, const EntranceElement& entranceEl,
    ImageId image, const CoordsXYZ& artworkOffset,
    uint32_t artworkGroup)
{
    if (session.FirstPersonSemanticComponentSink == nullptr)
        return 0;
    // Match the native side PaintStruct bounds exactly:
    //   origin=(3,3,height), size=(26,26,79).
    // These bounds are materially tighter than the tile/clearance envelope
    // and allow the semantic visual hull to stay at 1-world-unit resolution.
    const float bottom = float(entranceEl.getBaseZ());
    const auto transform =
        MakeFirstPersonParkEntranceTransform(
            session, entranceEl);
    return PaintSessionAddFirstPersonSemanticOrientedBox(
        session, FirstPersonPaintSemanticRole::structureBody,
        transform,
        { -13.0f, -13.0f, bottom },
        { 13.0f, 13.0f, bottom + 79.0f },
        image, artworkOffset, artworkGroup);
}

static uint32_t PublishFirstPersonParkEntrancePathDeck(
    PaintSession& session, const EntranceElement& entranceEl,
    ImageId image, const CoordsXYZ& artworkOffset,
    uint32_t artworkGroup)
{
    if (session.FirstPersonSemanticComponentSink == nullptr)
        return 0;
    FirstPersonPaintSemanticTransform transform{};
    transform.origin = {
        float(session.MapPosition.x),
        float(session.MapPosition.y),
        0.0f,
    };
    const float z = float(entranceEl.getBaseZ());
    return PaintSessionAddFirstPersonSemanticOrientedQuad(
        session, FirstPersonPaintSemanticRole::pathDeck,
        FirstPersonPaintSemanticPrimitiveKind::footprint,
        transform,
        { {
            { 0.0f, 0.0f, z },
            { float(kCoordsXYStep), 0.0f, z },
            { float(kCoordsXYStep), float(kCoordsXYStep), z },
            { 0.0f, float(kCoordsXYStep), z },
        } },
        image, artworkOffset, artworkGroup,
        false, false);
}

static void PaintParkEntranceScrollingText(
    PaintSession& session, const EntranceObject& entrance,
    const EntranceElement& entranceEl, Direction direction,
    int32_t height, uint32_t artworkGroup)
{
    PROFILED_FUNCTION();

    const auto publishEmpty = [&]() {
        PublishFirstPersonParkEntranceGatePlane(
            session, entranceEl, {},
            { 0, 0, height + entrance.GetTextHeight() },
            artworkGroup,
            FirstPersonPaintSemanticRole::sign);
    };

    if ((direction + 1) & (1 << 1))
    {
        publishEmpty();
        return;
    }

    auto scrollingMode = entrance.GetScrollingMode();
    if (scrollingMode == kScrollingModeNone)
    {
        publishEmpty();
        return;
    }

    auto& gameState = getGameState();
    u8string bannerText;
    if (gameState.park.flags.has(ParkFlag::parkOpen))
    {
        const auto& park = gameState.park;
        bannerText = ScrollingText::kParkBannerColourPrefix + park.name;
    }
    else
    {
        bannerText = LanguageGetString(STR_BANNER_TEXT_CLOSED);
    }

    auto imageIndex = ScrollingText::setup(
        session, bannerText,
        scrollingMode + direction / 2,
        PaletteIndex::transparent);
    auto textHeight = height + entrance.GetTextHeight();
    PublishFirstPersonParkEntranceGatePlane(
        session, entranceEl, imageIndex,
        { 0, 0, textHeight }, artworkGroup,
        FirstPersonPaintSemanticRole::sign);
    FirstPersonPaintSemanticScope scope(
        session, FirstPersonPaintSemanticRole::sign,
        artworkGroup);
    PaintAddImageAsChild(
        session, imageIndex,
        { 0, 0, textHeight },
        { { 2, 2, textHeight }, { 28, 28, 47 } });
}

static void PaintParkEntranceLightEffects(PaintSession& session)
{
    PROFILED_FUNCTION();

    if (LightFx::IsAvailable())
    {
        LightFx::Add3DLightMagicFromDrawingTile(session.MapPosition, 0, 0, 155, LightType::lantern3);
    }
}

static void PaintParkEntrance(PaintSession& session, uint8_t direction, int32_t height, const EntranceElement& entranceEl)
{
    PROFILED_FUNCTION();

    if (gTrackDesignSaveMode || (session.ViewFlags & VIEWPORT_FLAG_HIGHLIGHT_PATH_ISSUES))
        return;

    PaintParkEntranceLightEffects(session);

    session.InteractionType = ViewportInteractionItem::parkEntrance;

    ImageId imageTemplate;
    if (entranceEl.isGhost())
    {
        session.InteractionType = ViewportInteractionItem::none;
        imageTemplate = ImageId().WithRemap(FilterPaletteID::paletteGhost);
    }
    else if (session.SelectedElement == reinterpret_cast<const TileElement*>(&entranceEl))
    {
        imageTemplate = ImageId().WithRemap(FilterPaletteID::paletteGhost);
    }

    auto& objManager = GetContext()->GetObjectManager();
    const auto* entrance = objManager.GetLoadedObject<EntranceObject>(entranceEl.getEntryIndex());
    auto sequence = entranceEl.getSequenceIndex();
    switch (sequence)
    {
        case ParkEntranceSequence::centre:
        {
            // Footpath
            auto surfaceDescriptor = entranceEl.getPathSurfaceDescriptor();
            if (surfaceDescriptor != nullptr)
            {
                auto imageIndex =
                    surfaceDescriptor->image
                    + 5 * (1 + (direction & 1));
                const auto pathImage =
                    imageTemplate.WithIndex(imageIndex);
                const uint32_t pathGroup =
                    PaintSessionBeginFirstPersonSemanticArtworkGroup(
                        session);
                PublishFirstPersonParkEntrancePathDeck(
                    session, entranceEl, pathImage,
                    { 0, 0, height }, pathGroup);
                FirstPersonPaintSemanticScope pathScope(
                    session,
                    FirstPersonPaintSemanticRole::pathDeck,
                    pathGroup);
                PaintAddImageAsParent(
                    session, pathImage,
                    { 0, 0, height },
                    { { 0, 2, height }, { 32, 28, 0 } });
            }

            // Entrance: the centre is an oriented transparent gate plane,
            // preserving the walk-through opening instead of fabricating a
            // solid sorting box.
            if (entrance != nullptr)
            {
                auto imageIndex =
                    entrance->GetImage(sequence, direction);
                const auto entranceImage =
                    imageTemplate.WithIndex(imageIndex);
                const uint32_t bodyGroup =
                    PaintSessionBeginFirstPersonSemanticArtworkGroup(
                        session);
                PublishFirstPersonParkEntranceCentreBody(
                    session, entranceEl, entranceImage,
                    { 0, 0, height }, bodyGroup);
                {
                    FirstPersonPaintSemanticScope bodyScope(
                        session,
                        FirstPersonPaintSemanticRole::structureBody,
                        bodyGroup);
                    PaintAddImageAsParent(
                        session, entranceImage,
                        { 0, 0, height },
                        { { 2, 2, height + 32 },
                          { 28, 28, 47 } });
                }

                if (!entranceEl.isGhost())
                    PaintParkEntranceScrollingText(
                        session, *entrance, entranceEl,
                        direction, height, bodyGroup);
                else
                    PublishFirstPersonParkEntranceGatePlane(
                        session, entranceEl, {},
                        { 0, 0,
                          height + entrance->GetTextHeight() },
                        bodyGroup,
                        FirstPersonPaintSemanticRole::sign);
            }
            break;
        }
        case ParkEntranceSequence::left:
        case ParkEntranceSequence::right:
            if (entrance != nullptr)
            {
                auto imageIndex =
                    entrance->GetImage(sequence, direction);
                const auto entranceImage =
                    imageTemplate.WithIndex(imageIndex);
                const uint32_t bodyGroup =
                    PaintSessionBeginFirstPersonSemanticArtworkGroup(
                        session);
                PublishFirstPersonParkEntranceSideBody(
                    session, entranceEl, entranceImage,
                    { 0, 0, height }, bodyGroup);
                FirstPersonPaintSemanticScope bodyScope(
                    session,
                    FirstPersonPaintSemanticRole::structureBody,
                    bodyGroup);
                PaintAddImageAsParent(
                    session, entranceImage,
                    { 0, 0, height },
                    { { 3, 3, height },
                      { 26, 26, 79 } });
            }
            break;
    }

    auto supportsImageTemplate = imageTemplate;
    if (!entranceEl.isGhost())
    {
        supportsImageTemplate = ImageId().WithPrimary(OpenRCT2::Drawing::Colour::saturatedBrown);
    }
    WoodenASupportsPaintSetupRotated(
        session, WoodenSupportType::truss, WoodenSupportSubType::neSw, direction, height, supportsImageTemplate);

    PaintUtilSetSegmentSupportHeight(session, kSegmentsAll, 0xFFFF, 0);
    PaintUtilSetGeneralSupportHeight(session, height + 80);
}

static void PaintHeightMarkers(PaintSession& session, const EntranceElement& entranceEl, int32_t height)
{
    PROFILED_FUNCTION();

    if (PaintShouldShowHeightMarkers(session, VIEWPORT_FLAG_PATH_HEIGHTS))
    {
        if (entranceEl.getDirections() & 0xF)
        {
            auto heightMarkerBaseZ = entranceEl.getBaseZ() + 3;
            ImageIndex baseImageIndex = SPR_HEIGHT_MARKER_BASE;
            baseImageIndex += heightMarkerBaseZ / 16;
            baseImageIndex += GetHeightMarkerOffset();
            baseImageIndex -= kMapBaseZ;
            auto imageId = ImageId(baseImageIndex, OpenRCT2::Drawing::Colour::grey);
            PaintAddImageAsParent(session, imageId, { 16, 16, height }, { { 31, 31, heightMarkerBaseZ + 64 }, { 1, 1, 0 } });
        }
    }
}

void PaintEntrance(PaintSession& session, uint8_t direction, int32_t height, const EntranceElement& entranceElement)
{
    PROFILED_FUNCTION();

    session.InteractionType = ViewportInteractionItem::label;

    PaintHeightMarkers(session, entranceElement, height);
    switch (entranceElement.getEntranceType())
    {
        case EntranceType::rideEntrance:
        case EntranceType::rideExit:
            PaintRideEntranceExit(session, direction, height, entranceElement);
            break;
        case EntranceType::parkEntrance:
            PaintParkEntrance(session, direction, height, entranceElement);
            break;
    }
}
