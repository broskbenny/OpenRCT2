/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "../Paint.h"

#include "../../drawing/PaletteIndex.h"
#include "../../drawing/ScrollingText.h"
#include "../../interface/Viewport.h"
#include "../../object/BannerSceneryEntry.h"
#include "../../object/ObjectEntryManager.h"
#include "../../profiling/Profiling.h"
#include "../../ride/TrackDesign.h"
#include "../../world/Banner.h"
#include "../../world/tile_element/BannerElement.h"
#include "Paint.Banner.h"
#include "Paint.TileElement.h"

#include <algorithm>

using namespace OpenRCT2;
using namespace OpenRCT2::Drawing;

// kBannerBoundBoxes[rotation][0] is for the pole in the back
// kBannerBoundBoxes[rotation][1] is for the pole and the banner in the front
const CoordsXY kBannerBoundBoxes[][2] = {
    { { 1, 2 }, { 1, 29 } },
    { { 2, 32 }, { 29, 32 } },
    { { 32, 2 }, { 32, 29 } },
    { { 2, 1 }, { 29, 1 } },
};

static uint32_t PublishFirstPersonBannerSemanticPlane(
    PaintSession& session, const BannerElement& bannerElement,
    ImageId image, const CoordsXYZ& artworkOffset,
    uint32_t artworkGroup)
{
    if (session.FirstPersonSemanticComponentSink == nullptr)
        return 0;

    const float bottom =
        float(bannerElement.getBaseZ() - (2 * kCoordsZStep));
    const float top =
        float(std::max(
            bannerElement.getClearanceZ(),
            bannerElement.getBaseZ()));
    const float tile = float(kCoordsXYStep);
    FirstPersonPaintSemanticVec3 a{};
    FirstPersonPaintSemanticVec3 b{};
    switch (bannerElement.getPosition() & 3u)
    {
        case 0:
            a = { 0.0f, 0.0f, bottom };
            b = { 0.0f, tile, bottom };
            break;
        case 1:
            a = { 0.0f, tile, bottom };
            b = { tile, tile, bottom };
            break;
        case 2:
            a = { tile, tile, bottom };
            b = { tile, 0.0f, bottom };
            break;
        default:
            a = { tile, 0.0f, bottom };
            b = { 0.0f, 0.0f, bottom };
            break;
    }

    FirstPersonPaintSemanticTransform transform{};
    transform.origin = {
        float(session.MapPosition.x),
        float(session.MapPosition.y),
        0.0f,
    };
    return PaintSessionAddFirstPersonSemanticOrientedQuad(
        session, FirstPersonPaintSemanticRole::sign,
        FirstPersonPaintSemanticPrimitiveKind::plane,
        transform,
        { {
            a,
            b,
            { b.x, b.y, top },
            { a.x, a.y, top },
        } },
        image, artworkOffset, artworkGroup,
        true, false);
}

static void PaintBannerScrollingText(
    PaintSession& session, const BannerSceneryEntry& bannerEntry,
    Banner& banner, const BannerElement& bannerElement,
    Direction direction, int32_t height, const CoordsXYZ& bbOffset,
    uint32_t artworkGroup)
{
    PROFILED_FUNCTION();

    // Keep one third semantic layer in every native rotation so the multi-view
    // grouper pairs scrolling text consistently even when that text is hidden
    // in a particular source view.
    direction = DirectionReverse(direction) - 1;
    if (direction >= 2 || bannerElement.isGhost())
    {
        PublishFirstPersonBannerSemanticPlane(
            session, bannerElement, {},
            { 0, 0, height + 22 }, artworkGroup);
        return;
    }

    auto scrollingMode = bannerEntry.scrolling_mode + (direction & 3);
    if (scrollingMode >= ScrollingText::kMaxModes)
    {
        PublishFirstPersonBannerSemanticPlane(
            session, bannerElement, {},
            { 0, 0, height + 22 }, artworkGroup);
        return;
    }

    auto bannerText = banner.getTextWithColour();
    auto imageId = ScrollingText::setup(
        session, bannerText, scrollingMode,
        PaletteIndex::transparent);
    PublishFirstPersonBannerSemanticPlane(
        session, bannerElement, imageId,
        { 0, 0, height + 22 }, artworkGroup);
    FirstPersonPaintSemanticScope scope(
        session, FirstPersonPaintSemanticRole::sign,
        artworkGroup);
    PaintAddImageAsChild(
        session, imageId, { 0, 0, height + 22 },
        { bbOffset, { 1, 1, 21 } });
}

void PaintBanner(PaintSession& session, uint8_t direction, int32_t height, const BannerElement& bannerElement)
{
    PROFILED_FUNCTION();

    if (session.rt.zoom_level > ZoomLevel{ 1 } || gTrackDesignSaveMode
        || (session.ViewFlags & VIEWPORT_FLAG_HIGHLIGHT_PATH_ISSUES))
        return;

    auto banner = bannerElement.getBanner();
    if (banner == nullptr)
    {
        return;
    }

    auto* bannerEntry = OpenRCT2::ObjectEntryManager::GetObjectEntry<BannerSceneryEntry>(banner->type);
    if (bannerEntry == nullptr)
    {
        return;
    }

    session.InteractionType = ViewportInteractionItem::banner;

    height -= 16;

    direction += bannerElement.getPosition();
    direction &= 3;

    ImageId imageTemplate;
    if (bannerElement.isGhost())
    {
        session.InteractionType = ViewportInteractionItem::none;
        imageTemplate = ImageId().WithRemap(FilterPaletteID::paletteGhost);
    }
    else if (session.SelectedElement == reinterpret_cast<const TileElement*>(&bannerElement))
    {
        imageTemplate = ImageId().WithRemap(FilterPaletteID::paletteGhost);
    }
    else
    {
        imageTemplate = ImageId().WithPrimary(banner->colour);
    }

    auto imageIndex = (direction << 1) + bannerEntry->image;
    auto imageId = imageTemplate.WithIndex(imageIndex);
    const uint32_t artworkGroup =
        PaintSessionBeginFirstPersonSemanticArtworkGroup(session);
    PublishFirstPersonBannerSemanticPlane(
        session, bannerElement, imageId,
        { 0, 0, height }, artworkGroup);
    PublishFirstPersonBannerSemanticPlane(
        session, bannerElement, imageId.WithIndexOffset(1),
        { 0, 0, height }, artworkGroup);

    FirstPersonPaintSemanticScope scope(
        session, FirstPersonPaintSemanticRole::sign,
        artworkGroup);
    auto bbOffset = CoordsXYZ(kBannerBoundBoxes[direction][0], height + 2);
    PaintAddImageAsParent(
        session, imageId, { 0, 0, height },
        { bbOffset, { 1, 1, 21 } });

    bbOffset = CoordsXYZ(kBannerBoundBoxes[direction][1], height + 2);
    PaintAddImageAsParent(
        session, imageId.WithIndexOffset(1),
        { 0, 0, height },
        { bbOffset, { 1, 1, 21 } });

    PaintBannerScrollingText(
        session, *bannerEntry, *banner, bannerElement,
        direction, height, bbOffset, artworkGroup);
}
