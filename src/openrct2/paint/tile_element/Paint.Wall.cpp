/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "../Paint.h"

#include "../../GameState.h"
#include "../../drawing/ColourMap.h"
#include "../../drawing/ScrollingText.h"
#include "../../interface/Viewport.h"
#include "../../object/WallSceneryEntry.h"
#include "../../profiling/Profiling.h"
#include "../../ride/TrackDesign.h"
#include "../../world/tile_element/WallElement.h"
#include "Paint.TileElement.h"
#include "Paint.Wall.h"

using namespace OpenRCT2;
using namespace OpenRCT2::Drawing;

static constexpr uint8_t DirectionToDoorImageOffset0[] = {
    2, 2, 22, 26, 30, 34, 34, 34, 34, 34, 30, 26, 22, 2, 6, 2, 2, 2, 6, 10, 14, 18, 18, 18, 18, 18, 14, 10, 6, 2, 22, 2,
};

static constexpr uint8_t DirectionToDoorImageOffset1[] = {
    0, 0, 4, 8, 12, 16, 16, 16, 16, 16, 12, 8, 4, 0, 20, 0, 0, 0, 20, 24, 28, 32, 32, 32, 32, 32, 28, 24, 20, 0, 4, 0,
};

static constexpr uint8_t DirectionToDoorImageOffset2[] = {
    2, 2, 6, 10, 14, 18, 18, 18, 18, 18, 14, 10, 6, 2, 22, 2, 2, 2, 22, 26, 30, 34, 34, 34, 34, 34, 30, 26, 22, 2, 6, 2,
};

static constexpr uint8_t DirectionToDoorImageOffset3[] = {
    0, 0, 20, 24, 28, 32, 32, 32, 32, 32, 28, 24, 20, 0, 4, 0, 0, 0, 4, 8, 12, 16, 16, 16, 16, 16, 12, 8, 4, 0, 20, 0,
};

static constexpr const uint8_t* DirectionToDoorImageOffset[] = { DirectionToDoorImageOffset0, DirectionToDoorImageOffset1,
                                                                 DirectionToDoorImageOffset2, DirectionToDoorImageOffset3 };

static uint32_t PublishFirstPersonWallSemanticPlane(
    PaintSession& session, const WallSceneryEntry& wallEntry,
    const WallElement& wallElement, ImageId image,
    const CoordsXYZ& artworkOffset, uint32_t artworkGroup,
    bool decal, bool collidable);

static void PaintWallDoor(
    PaintSession& session, const WallSceneryEntry& wallEntry,
    const WallElement& wallElement, ImageId imageId, CoordsXYZ offset,
    BoundBoxXYZ bbR1, BoundBoxXYZ bbR2, BoundBoxXYZ bbL)
{
    PROFILED_FUNCTION();

    const auto newImageId0 = imageId;
    const auto newImageId1 = imageId.WithIndexOffset(1);
    const uint32_t artworkGroup =
        PaintSessionBeginFirstPersonSemanticArtworkGroup(session);

    // A door is still authored on one known wall plane. Its animation may
    // create a transparent opening, so the semantic plane is visual evidence
    // rather than a solid collision slab. Both native layers remain attached
    // to the same fixed world plane instead of becoming passenger-facing cards.
    PublishFirstPersonWallSemanticPlane(
        session, wallEntry, wallElement, newImageId0,
        offset, artworkGroup, true, false);
    PublishFirstPersonWallSemanticPlane(
        session, wallEntry, wallElement, newImageId1,
        offset, artworkGroup, true, false);

    FirstPersonPaintSemanticScope scope(
        session, FirstPersonPaintSemanticRole::wall,
        artworkGroup);
    if (wallEntry.flags.has(WallSceneryFlag::cannotBuildOnSlope))
    {
        PaintAddImageAsParent(session, newImageId0, offset, bbR1);
        PaintAddImageAsParent(session, newImageId1, offset, bbR2);
    }
    else
    {
        PaintAddImageAsParent(session, newImageId0, offset, bbL);
        PaintAddImageAsChild(session, newImageId1, offset, bbL);
    }
}

static void PaintWallDoor(
    PaintSession& session, const WallSceneryEntry& wallEntry, const WallElement& wallElement, ImageId imageTemplate,
    Direction direction, int32_t height)
{
    PROFILED_FUNCTION();

    auto bbHeight = wallEntry.height * 8 - 2;
    auto animationFrame = wallElement.getAnimationFrame();

    // Add the direction as well
    if (wallElement.animationIsBackwards())
        animationFrame |= (1 << 4);

    auto imageId = wallEntry.image + DirectionToDoorImageOffset[direction & 3][animationFrame];
    switch (direction)
    {
        case 0:
        {
            BoundBoxXYZ bbR1 = { { 1, 1, height + 1 }, { 1, 3, bbHeight - 5 } };
            BoundBoxXYZ bbR2 = { { 1, 1, height + bbHeight - 4 }, { 1, 28, 3 } };

            BoundBoxXYZ bbL = { { 1, 1, height + 1 }, { 1, 28, bbHeight } };

            CoordsXYZ offset = { 0, 0, height };

            PaintWallDoor(
                session, wallEntry, wallElement,
                imageTemplate.WithIndex(imageId),
                offset, bbR1, bbR2, bbL);
            break;
        }
        case 1:
        {
            BoundBoxXYZ bbR1 = { { 1, 30, height + 1 }, { 3, 3, bbHeight - 5 } };
            BoundBoxXYZ bbR2 = { { 1, 30, height + bbHeight - 3 }, { 29, 3, 2 } };
            BoundBoxXYZ bbL = { { 2, 30, height + 1 }, { 29, 1, bbHeight } };

            CoordsXYZ offset = { 1, 31, height };

            PaintWallDoor(
                session, wallEntry, wallElement,
                imageTemplate.WithIndex(imageId),
                offset, bbR1, bbR2, bbL);
            break;
        }
        case 2:
        {
            BoundBoxXYZ bbR1 = { { 30, 1, height + 1 }, { 3, 3, bbHeight - 5 } };
            BoundBoxXYZ bbR2 = { { 30, 1, height + bbHeight - 3 }, { 3, 29, 2 } };
            BoundBoxXYZ bbL = { { 30, 2, height + 1 }, { 1, 29, bbHeight } };

            CoordsXYZ offset = { 31, 0, height };

            PaintWallDoor(
                session, wallEntry, wallElement,
                imageTemplate.WithIndex(imageId),
                offset, bbR1, bbR2, bbL);
            break;
        }
        case 3:
        {
            BoundBoxXYZ bbR1 = { { 1, 1, height + 1 }, { 3, 1, bbHeight - 5 } };
            BoundBoxXYZ bbR2 = { { 1, 1, height + bbHeight - 4 }, { 28, 1, 3 } };
            BoundBoxXYZ bbL = { { 1, 1, height + 1 }, { 28, 1, bbHeight } };

            CoordsXYZ offset = { 2, 1, height };

            PaintWallDoor(
                session, wallEntry, wallElement,
                imageTemplate.WithIndex(imageId),
                offset, bbR1, bbR2, bbL);
            break;
        }
    }
}

static std::array<FirstPersonPaintSemanticVec3, 4>
    GetFirstPersonWallSemanticCorners(
        const WallSceneryEntry& wallEntry,
        const WallElement& wallElement)
{
    const float h =
        float(int32_t(wallEntry.height) * kCoordsZStep);
    const float step =
        float(2 * kCoordsZStep);
    FirstPersonPaintSemanticVec3 a{};
    FirstPersonPaintSemanticVec3 b{};
    switch (static_cast<uint8_t>(
                wallElement.getDirection()) & 3u)
    {
        case 0:
            a = { 0.0f, 0.0f, 0.0f };
            b = { 0.0f, float(kCoordsXYStep), 0.0f };
            break;
        case 1:
            a = { 0.0f, float(kCoordsXYStep), 0.0f };
            b = { float(kCoordsXYStep), float(kCoordsXYStep), 0.0f };
            break;
        case 2:
            a = { float(kCoordsXYStep), float(kCoordsXYStep), 0.0f };
            b = { float(kCoordsXYStep), 0.0f, 0.0f };
            break;
        default:
            a = { float(kCoordsXYStep), 0.0f, 0.0f };
            b = { 0.0f, 0.0f, 0.0f };
            break;
    }

    // Native wall slope values are 1 = upward along the authored edge,
    // 2 = downward along it. Keep the exact physical edge in world space;
    // the current paint rotation must never rotate the wall itself.
    if (wallElement.getSlope() == 1)
        b.z += step;
    else if (wallElement.getSlope() == 2)
        a.z += step;

    return { {
        a,
        b,
        { b.x, b.y, b.z + h },
        { a.x, a.y, a.z + h },
    } };
}

static uint32_t PublishFirstPersonWallSemanticPlane(
    PaintSession& session, const WallSceneryEntry& wallEntry,
    const WallElement& wallElement, ImageId image,
    const CoordsXYZ& artworkOffset, uint32_t artworkGroup,
    bool decal, bool collidable)
{
    if (session.FirstPersonSemanticComponentSink == nullptr
        || !image.HasValue())
        return 0;

    FirstPersonPaintSemanticTransform transform{};
    transform.origin = {
        float(session.MapPosition.x),
        float(session.MapPosition.y),
        float(wallElement.getBaseZ()),
    };
    return PaintSessionAddFirstPersonSemanticOrientedQuad(
        session, FirstPersonPaintSemanticRole::wall,
        FirstPersonPaintSemanticPrimitiveKind::plane,
        transform,
        GetFirstPersonWallSemanticCorners(
            wallEntry, wallElement),
        image, artworkOffset, artworkGroup,
        decal, collidable);
}

static void PaintWallWall(
    PaintSession& session, const WallSceneryEntry& wallEntry,
    const WallElement& wallElement, ImageId imageTemplate,
    uint32_t imageOffset, CoordsXYZ offset,
    BoundBoxXYZ boundBox, bool isGhost)
{
    PROFILED_FUNCTION();

    auto frameNum = wallEntry.flags2.has(WallSceneryFlag2::isAnimated) ? (getGameState().currentTicks & 7) * 2 : 0;
    auto imageIndex = wallEntry.image + imageOffset + frameNum;
    const auto wallImage = imageTemplate.WithIndex(imageIndex);
    const uint32_t artworkGroup =
        PaintSessionBeginFirstPersonSemanticArtworkGroup(session);
    const bool transparentBody =
        wallEntry.flags.has(WallSceneryFlag::hasGlass)
        || wallEntry.flags2.has(
            WallSceneryFlag2::isTransparent);
    PublishFirstPersonWallSemanticPlane(
        session, wallEntry, wallElement, wallImage,
        offset, artworkGroup, transparentBody, true);
    {
        FirstPersonPaintSemanticScope scope(
            session, FirstPersonPaintSemanticRole::wall,
            artworkGroup);
        PaintAddImageAsParent(
            session, wallImage, offset, boundBox);
    }

    if ((wallEntry.flags.has(WallSceneryFlag::hasGlass)) && !isGhost)
    {
        auto glassImageId =
            ImageId(imageIndex + 6)
                .WithTransparency(imageTemplate.GetPrimary());
        PublishFirstPersonWallSemanticPlane(
            session, wallEntry, wallElement,
            glassImageId, offset, artworkGroup,
            true, false);
        FirstPersonPaintSemanticScope scope(
            session, FirstPersonPaintSemanticRole::wall,
            artworkGroup);
        PaintAddImageAsChild(
            session, glassImageId, offset, boundBox);
    }
}

static void PaintWallScrollingText(
    PaintSession& session, const WallSceneryEntry& wallEntry, const WallElement& wallElement, Direction direction,
    int32_t height, const CoordsXYZ& boundsOffset, bool isGhost)
{
    PROFILED_FUNCTION();

    if (direction != 0 && direction != 3)
        return;

    auto scrollingMode = wallEntry.scrolling_mode;
    if (scrollingMode == kScrollingModeNone)
        return;

    scrollingMode = wallEntry.scrolling_mode + ((direction + 1) & 3);
    if (scrollingMode >= ScrollingText::kMaxModes)
        return;

    auto banner = wallElement.getBanner();
    if (banner == nullptr)
        return;

    auto textColour = isGhost ? static_cast<OpenRCT2::Drawing::Colour>(OpenRCT2::Drawing::Colour::grey)
                              : wallElement.getSecondaryColour();
    auto textPaletteIndex = direction == 0 ? getColourMap(textColour).midDark : getColourMap(textColour).light;

    auto bannerText = banner->getText();
    auto imageId = ScrollingText::setup(
        session, bannerText, scrollingMode,
        textPaletteIndex);
    const CoordsXYZ artworkOffset{
        0, 0, height + 8
    };
    const uint32_t artworkGroup =
        PaintSessionBeginFirstPersonSemanticArtworkGroup(
            session);
    PublishFirstPersonWallSemanticPlane(
        session, wallEntry, wallElement,
        imageId, artworkOffset, artworkGroup,
        true, false);
    FirstPersonPaintSemanticScope scope(
        session, FirstPersonPaintSemanticRole::wall,
        artworkGroup);
    PaintAddImageAsChild(
        session, imageId, artworkOffset,
        { boundsOffset, { 1, 1, 13 } });
}

static void PaintWallWall(
    PaintSession& session, const WallSceneryEntry& wallEntry, const WallElement& wallElement, ImageId imageTemplate,
    Direction direction, int32_t height, bool isGhost)
{
    PROFILED_FUNCTION();

    uint8_t bbHeight = wallEntry.height * 8 - 2;
    ImageIndex imageOffset = 0;
    CoordsXYZ offset;
    BoundBoxXYZ boundBox;
    switch (direction)
    {
        case 0:
            if (wallElement.getSlope() == 2)
            {
                imageOffset = 3;
            }
            else if (wallElement.getSlope() == 1)
            {
                imageOffset = 5;
            }
            else
            {
                imageOffset = 1;
            }

            offset = { 0, 0, height };
            boundBox = { { 1, 1, height + 1 }, { 1, 28, bbHeight } };
            break;

        case 1:
            if (wallElement.getSlope() == 2)
            {
                imageOffset = 2;
            }
            else if (wallElement.getSlope() == 1)
            {
                imageOffset = 4;
            }
            else
            {
                imageOffset = 0;
            }

            if (wallEntry.flags.has(WallSceneryFlag::hasGlass))
            {
                if (wallEntry.flags.has(WallSceneryFlag::isDoubleSided))
                {
                    imageOffset += 12;
                }
            }
            else
            {
                if (wallEntry.flags.has(WallSceneryFlag::isDoubleSided))
                {
                    imageOffset += 6;
                }
            }

            offset = { 1, 31, height };
            boundBox = { { 2, 30, height + 1 }, { 29, 1, bbHeight } };
            break;

        case 2:
            if (wallElement.getSlope() == 2)
            {
                imageOffset = 5;
            }
            else if (wallElement.getSlope() == 1)
            {
                imageOffset = 3;
            }
            else
            {
                imageOffset = 1;
            }

            if (wallEntry.flags.has(WallSceneryFlag::isDoubleSided))
            {
                imageOffset += 6;
            }

            offset = { 31, 0, height };
            boundBox = { { 30, 2, height + 1 }, { 1, 29, bbHeight } };
            break;

        case 3:
            if (wallElement.getSlope() == 2)
            {
                imageOffset = 4;
            }
            else if (wallElement.getSlope() == 1)
            {
                imageOffset = 2;
            }
            else
            {
                imageOffset = 0;
            }

            offset = { 2, 1, height };
            boundBox = { { 1, 1, height + 1 }, { 28, 1, bbHeight } };
            break;
    }

    PaintWallWall(
        session, wallEntry, wallElement, imageTemplate,
        imageOffset, offset, boundBox, isGhost);
    PaintWallScrollingText(session, wallEntry, wallElement, direction, height, boundBox.offset, isGhost);
}

void PaintWall(PaintSession& session, uint8_t direction, int32_t height, const WallElement& wallElement)
{
    PROFILED_FUNCTION();

    if (session.ViewFlags & VIEWPORT_FLAG_HIGHLIGHT_PATH_ISSUES)
    {
        return;
    }

    auto* wallEntry = wallElement.getEntry();
    if (wallEntry == nullptr)
    {
        return;
    }

    session.InteractionType = ViewportInteractionItem::wall;

    ImageId imageTemplate;
    if (wallEntry->flags.has(WallSceneryFlag::hasPrimaryColour))
    {
        imageTemplate = imageTemplate.WithPrimary(wallElement.getPrimaryColour());
    }
    if (wallEntry->flags.has(WallSceneryFlag::hasSecondaryColour))
    {
        imageTemplate = imageTemplate.WithSecondary(wallElement.getSecondaryColour());
    }
    if (wallEntry->flags.has(WallSceneryFlag::hasTertiaryColour))
    {
        imageTemplate = imageTemplate.WithTertiary(wallElement.getTertiaryColour());
    }

    PaintUtilSetGeneralSupportHeight(session, 8 * wallElement.clearanceHeight);

    auto isGhost = false;
    if (gTrackDesignSaveMode)
    {
        if (!TrackDesignSaveContainsTileElement(reinterpret_cast<const TileElement*>(&wallElement)))
        {
            imageTemplate = ImageId().WithRemap(FilterPaletteID::palette46);
            isGhost = true;
        }
    }

    if (wallElement.isGhost())
    {
        session.InteractionType = ViewportInteractionItem::none;
        imageTemplate = ImageId().WithRemap(FilterPaletteID::paletteGhost);
        isGhost = true;
    }
    else if (session.SelectedElement == reinterpret_cast<const TileElement*>(&wallElement))
    {
        imageTemplate = ImageId().WithRemap(FilterPaletteID::paletteGhost);
        isGhost = true;
    }

    if (wallEntry->flags.has(WallSceneryFlag::isDoor))
    {
        PaintWallDoor(session, *wallEntry, wallElement, imageTemplate, direction, height);
    }
    else
    {
        PaintWallWall(session, *wallEntry, wallElement, imageTemplate, direction, height, isGhost);
    }
}
