/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "Paint.h"

#include "../Context.h"
#include "../config/Config.h"
#include "../core/Money.hpp"
#include "../core/Numerics.hpp"
#include "../drawing/Drawing.Sprite.h"
#include "../drawing/Drawing.String.h"
#include "../drawing/ScrollingText.h"
#include "../drawing/Drawing.h"
#include "../drawing/Font.h"
#include "../drawing/Line.h"
#include "../interface/Viewport.h"
#include "../localisation/Currency.h"
#include "../localisation/Formatting.h"
#include "../localisation/LocalisationService.h"
#include "../paint/Painter.h"
#include "../profiling/Profiling.h"
#include "Boundbox.h"
#include "Paint.Entity.h"
#include "tile_element/Paint.TileElement.h"

#include <algorithm>
#include <array>
#include <cmath>

using namespace OpenRCT2;
using namespace OpenRCT2::Drawing;
using namespace OpenRCT2::Numerics;

// Globals for paint clipping
uint8_t gClipHeight = 128; // Default to middle value
CoordsXY gClipSelectionA = { 0, 0 };
CoordsXY gClipSelectionB = { kMaximumTileStartXY, kMaximumTileStartXY };

static constexpr PaletteIndex kBoundBoxDebugColours[] = {
    PaletteIndex::transparent, // NONE
    PaletteIndex::pi102,       // TERRAIN
    PaletteIndex::pi114,       // SPRITE
    PaletteIndex::pi229,       // RIDE
    PaletteIndex::pi126,       // WATER
    PaletteIndex::pi138,       // SCENERY
    PaletteIndex::pi150,       // FOOTPATH
    PaletteIndex::pi162,       // FOOTPATH_ITEM
    PaletteIndex::pi174,       // PARK
    PaletteIndex::pi186,       // WALL
    PaletteIndex::pi198,       // LARGE_SCENERY
    PaletteIndex::hotPink8,    // LABEL
    PaletteIndex::pi222,       // BANNER
};

bool gShowDirtyVisuals;
bool gPaintBoundingBoxes;
bool gPaintBlockedTiles;
bool gPaintStableSort;

static void PaintPSImageWithBoundingBoxes(PaintSession& session, PaintStruct* ps, ImageId imageId, int32_t x, int32_t y);
static ImageId PaintPSColourifyImage(const PaintStruct* ps, ImageId imageId, uint32_t viewFlags);

static int32_t RemapPositionToQuadrant(const PaintStruct& ps, uint8_t rotation)
{
    constexpr auto MapRangeMax = MaxPaintQuadrants * kCoordsXYStep;
    constexpr auto MapRangeCenter = MapRangeMax / 2;

    const auto x = ps.Bounds.x;
    const auto y = ps.Bounds.y;
    // NOTE: We are not calling CoordsXY::Rotate on purpose to mix in the additional
    // value without a secondary switch.
    switch (rotation & 3)
    {
        case 0:
            return x + y;
        case 1:
            // Because one component may be the maximum we add the center to be a positive value.
            return (y - x) + MapRangeCenter;
        case 2:
            // If both components would be the maximum it would be the negative xy, to be positive add max.
            return (-(y + x)) + MapRangeMax;
        case 3:
            // Same as 1 but inverted.
            return (x - y) + MapRangeCenter;
    }
    return 0;
}

static void PaintSessionAddPSToQuadrant(PaintSession& session, PaintStruct* ps)
{
    const auto positionHash = RemapPositionToQuadrant(*ps, session.CurrentRotation);

    // Values below zero or above MaxPaintQuadrants are void, corners also share the same quadrant as void.
    const uint32_t paintQuadrantIndex = std::clamp(positionHash / kCoordsXYStep, 0, MaxPaintQuadrants - 1);

    ps->QuadrantIndex = paintQuadrantIndex;
    ps->NextQuadrantEntry = session.Quadrants[paintQuadrantIndex];
    session.Quadrants[paintQuadrantIndex] = ps;

    session.QuadrantBackIndex = std::min(session.QuadrantBackIndex, paintQuadrantIndex);
    session.QuadrantFrontIndex = std::max(session.QuadrantFrontIndex, paintQuadrantIndex);
}

static constexpr bool imageWithinRT(const ScreenCoordsXY& imagePos, const G1Element& g1, const RenderTarget& rt)
{
    const int32_t left = imagePos.x + g1.xOffset;
    const int32_t bottom = imagePos.y + g1.yOffset;

    const int32_t right = left + g1.width;
    const int32_t top = bottom + g1.height;

    // mber: It is possible to use only the bottom else block here if you change <= and >= to simply < and >.
    // However, since this is used to cull paint structs, I'd prefer to keep the condition strict and calculate
    // the culling differently for minifying and magnifying.
    const auto zoom = rt.zoom_level;
    if (zoom > ZoomLevel{ 0 })
    {
        const int32_t x = zoom.ApplyTo(rt.cullingX);
        const int32_t y = zoom.ApplyTo(rt.cullingY);
        if (right <= x)
            return false;
        if (top <= y)
            return false;
        if (left >= x + zoom.ApplyTo(rt.cullingWidth))
            return false;
        if (bottom >= y + zoom.ApplyTo(rt.cullingHeight))
            return false;
    }
    else
    {
        if (zoom.ApplyInversedTo(right) <= rt.cullingX)
            return false;
        if (zoom.ApplyInversedTo(top) <= rt.cullingY)
            return false;
        if (zoom.ApplyInversedTo(left) >= rt.cullingX + rt.cullingWidth)
            return false;
        if (zoom.ApplyInversedTo(bottom) >= rt.cullingY + rt.cullingHeight)
            return false;
    }
    return true;
}

static constexpr CoordsXYZ RotateBoundBoxSize(const CoordsXYZ& bbSize, const uint8_t rotation)
{
    auto output = bbSize;
    // This probably rotates the variables so they're relative to rotation 0.
    switch (rotation)
    {
        case 0:
            output.x--;
            output.y--;
            output = { output.rotate(0), output.z };
            break;
        case 1:
            output.x--;
            output = { output.rotate(3), output.z };
            break;
        case 2:
            output = { output.rotate(2), output.z };
            break;
        case 3:
            output.y--;
            output = { output.rotate(1), output.z };
            break;
    }
    return output;
}

/**
 * Extracted from 0x0098196c, 0x0098197c, 0x0098198c, 0x0098199c
 */
static PaintStruct* CreateNormalPaintStruct(
    PaintSession& session, ImageId image_id, const CoordsXYZ& offset, const BoundBoxXYZ& boundBox)
{
    auto* const g1 = GfxGetG1Element(image_id);
    if (g1 == nullptr)
    {
        return nullptr;
    }

    const auto swappedRotation = DirectionFlipXAxis(session.CurrentRotation);
    auto swappedRotCoord = CoordsXYZ{ offset.rotate(swappedRotation), offset.z };
    swappedRotCoord += session.SpritePosition;

    const auto imagePos = Translate3DTo2DWithZ(session.CurrentRotation, swappedRotCoord);

    if (!imageWithinRT(imagePos, *g1, session.rt))
    {
        return nullptr;
    }

    const auto rotBoundBoxOffset = CoordsXYZ{ boundBox.offset.rotate(swappedRotation), boundBox.offset.z };
    const auto rotBoundBoxSize = RotateBoundBoxSize(boundBox.length, session.CurrentRotation);

    auto* ps = session.AllocateNormalPaintEntry();
    if (ps == nullptr)
    {
        return nullptr;
    }

    ps->image_id = image_id;
    ps->FirstPersonSnapshot = Drawing::ScrollingText::CaptureFirstPersonSnapshot(image_id);
    ps->ScreenPos = imagePos;
    ps->Bounds.x_end = rotBoundBoxSize.x + rotBoundBoxOffset.x + session.SpritePosition.x;
    ps->Bounds.y_end = rotBoundBoxSize.y + rotBoundBoxOffset.y + session.SpritePosition.y;
    ps->Bounds.z_end = rotBoundBoxSize.z + rotBoundBoxOffset.z;
    ps->Bounds.x = rotBoundBoxOffset.x + session.SpritePosition.x;
    ps->Bounds.y = rotBoundBoxOffset.y + session.SpritePosition.y;
    ps->Bounds.z = rotBoundBoxOffset.z;
    ps->Attached = nullptr;
    ps->Children = nullptr;
    ps->NextQuadrantEntry = nullptr;
    ps->InteractionItem = session.InteractionType;
    ps->MapPos = session.MapPosition;
    ps->Element = session.CurrentlyDrawnTileElement;
    ps->Entity = session.CurrentlyDrawnEntity;
    ps->Source = session.CurrentSource;
    ps->FirstPersonSemanticRole = session.FirstPersonSemanticRole;
    ps->FirstPersonPassengerSeatMask =
        session.FirstPersonPassengerSeatMask;
    ps->FirstPersonSemanticArtworkGroup =
        session.FirstPersonSemanticArtworkGroup;

    return ps;
}

static PaintStruct* CreateNormalPaintStructHeight(
    PaintSession& session, const ImageId imageId, const int32_t height, const CoordsXYZ& offset, const BoundBoxXYZ& boundBox)
{
    auto* const g1 = GfxGetG1Element(imageId);
    if (g1 == nullptr)
    {
        return nullptr;
    }

    const auto swappedRotation = DirectionFlipXAxis(session.CurrentRotation);
    auto swappedRotCoord = CoordsXYZ{ offset.rotate(swappedRotation), offset.z + height };
    swappedRotCoord += session.SpritePosition;

    const auto imagePos = Translate3DTo2DWithZ(session.CurrentRotation, swappedRotCoord);

    if (!imageWithinRT(imagePos, *g1, session.rt))
    {
        return nullptr;
    }

    const auto rotBoundBoxOffset = CoordsXYZ{ boundBox.offset.rotate(swappedRotation), boundBox.offset.z + height };
    const auto rotBoundBoxSize = RotateBoundBoxSize(boundBox.length, session.CurrentRotation);

    auto* ps = session.AllocateNormalPaintEntry();
    if (ps == nullptr)
    {
        return nullptr;
    }

    ps->image_id = imageId;
    ps->FirstPersonSnapshot = Drawing::ScrollingText::CaptureFirstPersonSnapshot(imageId);
    ps->ScreenPos = imagePos;
    ps->Bounds.x_end = rotBoundBoxSize.x + rotBoundBoxOffset.x + session.SpritePosition.x;
    ps->Bounds.y_end = rotBoundBoxSize.y + rotBoundBoxOffset.y + session.SpritePosition.y;
    ps->Bounds.z_end = rotBoundBoxSize.z + rotBoundBoxOffset.z;
    ps->Bounds.x = rotBoundBoxOffset.x + session.SpritePosition.x;
    ps->Bounds.y = rotBoundBoxOffset.y + session.SpritePosition.y;
    ps->Bounds.z = rotBoundBoxOffset.z;
    ps->Attached = nullptr;
    ps->Children = nullptr;
    ps->NextQuadrantEntry = nullptr;
    ps->InteractionItem = session.InteractionType;
    ps->MapPos = session.MapPosition;
    ps->Element = session.CurrentlyDrawnTileElement;
    ps->Entity = session.CurrentlyDrawnEntity;
    ps->Source = session.CurrentSource;
    ps->FirstPersonSemanticRole = session.FirstPersonSemanticRole;
    ps->FirstPersonPassengerSeatMask =
        session.FirstPersonPassengerSeatMask;
    ps->FirstPersonSemanticArtworkGroup =
        session.FirstPersonSemanticArtworkGroup;

    return ps;
}

template<uint8_t direction>
void PaintSessionGenerateRotate(PaintSession& session)
{
    // Optimised modified version of ViewportPosToMapPos
    ScreenCoordsXY screenCoord = { floor2(session.rt.WorldX(), 32), floor2((session.rt.WorldY() - 16), 32) };
    CoordsXY mapTile = { screenCoord.y - screenCoord.x / 2, screenCoord.y + screenCoord.x / 2 };
    mapTile = mapTile.rotate(direction);

    if constexpr (direction & 1)
    {
        mapTile.y -= 16;
    }
    mapTile = mapTile.toTileStart();

    uint16_t numVerticalTiles = (session.rt.WorldHeight() + 2128) >> 5;

    // Adjacent tiles to also check due to overlapping of sprites
    constexpr CoordsXY adjacentTiles[] = {
        CoordsXY{ -32, 32 }.rotate(direction),
        CoordsXY{ 0, 32 }.rotate(direction),
        CoordsXY{ 32, 0 }.rotate(direction),
    };
    constexpr CoordsXY nextVerticalTile = CoordsXY{ 32, 32 }.rotate(direction);

    for (; numVerticalTiles > 0; --numVerticalTiles)
    {
        TileElementPaintSetup(session, mapTile);
        EntityPaintSetup(session, mapTile);

        const auto loc1 = mapTile + adjacentTiles[0];
        EntityPaintSetup(session, loc1);

        const auto loc2 = mapTile + adjacentTiles[1];
        TileElementPaintSetup(session, loc2);
        EntityPaintSetup(session, loc2);

        const auto loc3 = mapTile + adjacentTiles[2];
        EntityPaintSetup(session, loc3);

        mapTile += nextVerticalTile;
    }
}

/**
 *
 *  rct2: 0x0068B6C2
 */
void PaintSessionGenerate(PaintSession& session)
{
    switch (DirectionFlipXAxis(session.CurrentRotation))
    {
        case 0:
            PaintSessionGenerateRotate<0>(session);
            break;
        case 1:
            PaintSessionGenerateRotate<1>(session);
            break;
        case 2:
            PaintSessionGenerateRotate<2>(session);
            break;
        case 3:
            PaintSessionGenerateRotate<3>(session);
            break;
    }
}

template<uint8_t TRotation>
static bool CheckBoundingBox(const PaintStructBoundBox& initialBBox, const PaintStructBoundBox& currentBBox)
{
    if constexpr (TRotation == 0)
    {
        if (initialBBox.z_end >= currentBBox.z && initialBBox.y_end >= currentBBox.y && initialBBox.x_end >= currentBBox.x
            && !(initialBBox.z < currentBBox.z_end && initialBBox.y < currentBBox.y_end && initialBBox.x < currentBBox.x_end))
        {
            return true;
        }
    }
    else if constexpr (TRotation == 1)
    {
        if (initialBBox.z_end >= currentBBox.z && initialBBox.y_end >= currentBBox.y && initialBBox.x_end < currentBBox.x
            && !(initialBBox.z < currentBBox.z_end && initialBBox.y < currentBBox.y_end && initialBBox.x >= currentBBox.x_end))
        {
            return true;
        }
    }
    else if constexpr (TRotation == 2)
    {
        if (initialBBox.z_end >= currentBBox.z && initialBBox.y_end < currentBBox.y && initialBBox.x_end < currentBBox.x
            && !(initialBBox.z < currentBBox.z_end && initialBBox.y >= currentBBox.y_end && initialBBox.x >= currentBBox.x_end))
        {
            return true;
        }
    }
    else if constexpr (TRotation == 3)
    {
        if (initialBBox.z_end >= currentBBox.z && initialBBox.y_end < currentBBox.y && initialBBox.x_end >= currentBBox.x
            && !(initialBBox.z < currentBBox.z_end && initialBBox.y >= currentBBox.y_end && initialBBox.x < currentBBox.x_end))
        {
            return true;
        }
    }
    return false;
}

namespace OpenRCT2::PaintSortFlags
{
    static constexpr uint8_t None = 0;
    static constexpr uint8_t PendingVisit = (1u << 0);
    static constexpr uint8_t Neighbour = (1u << 1);
    static constexpr uint8_t OutsideQuadrant = (1u << 7);
} // namespace OpenRCT2::PaintSortFlags

static PaintStruct* PaintStructsFirstInQuadrant(PaintStruct* psNext, uint16_t quadrantIndex)
{
    PaintStruct* ps;
    do
    {
        ps = psNext;
        psNext = psNext->NextQuadrantEntry;
        if (psNext == nullptr)
            return ps;
    } while (quadrantIndex > psNext->QuadrantIndex);
    return ps;
}

// Initializes sorting flags for all entries in the specified quadrant by quadrantIndex.
// Sorting flags specify whether a node needs to be traversed, is a neighbour, or is outside the
// quadrant range.
static void PaintStructsInitializeSort(PaintStruct* ps, uint16_t quadrantIndex, uint8_t flag)
{
    do
    {
        ps = ps->NextQuadrantEntry;
        if (ps == nullptr)
            break;

        if (ps->QuadrantIndex > quadrantIndex + 1)
        {
            // Outside of the range.
            ps->SortFlags = PaintSortFlags::OutsideQuadrant;
        }
        else if (ps->QuadrantIndex == quadrantIndex + 1)
        {
            // Is neighbour and requires a visit.
            ps->SortFlags = PaintSortFlags::Neighbour | PaintSortFlags::PendingVisit;
        }
        else if (ps->QuadrantIndex == quadrantIndex)
        {
            // In specified quadrant, requires visit.
            ps->SortFlags = flag | PaintSortFlags::PendingVisit;
        }
    } while (ps->QuadrantIndex <= quadrantIndex + 1);
}

// Returns a pair of parent and child where child is the next node that requires traversal.
// Because this structure uses a singly linked list we need to keep track of the parent in order
// to be able to re-order the list.
static std::pair<PaintStruct*, PaintStruct*> PaintStructsGetNextPending(PaintStruct* ps)
{
    PaintStruct* ps_next;
    while (true)
    {
        ps_next = ps->NextQuadrantEntry;
        if (ps_next == nullptr)
        {
            // End of the current list.
            return { nullptr, nullptr };
        }
        if (ps_next->SortFlags & PaintSortFlags::OutsideQuadrant)
        {
            // Reached point outside of specified quadrant.
            return { nullptr, nullptr };
        }
        if (ps_next->SortFlags & PaintSortFlags::PendingVisit)
        {
            // Found node to check on.
            break;
        }
        ps = ps_next;
    }
    return { ps, ps_next };
}

// Re-orders all nodes after the specified child node and marks the child node as traversed. The resulting
// order of the children is the depth based on rotation and dimensions of the bounding box.
template<uint8_t TRotation>
static void PaintStructsSortQuadrantLegacy(PaintStruct* parent, PaintStruct* child)
{
    // Mark visited.
    child->SortFlags &= ~PaintSortFlags::PendingVisit;

    // Compare all the children below the first child and move them up in the list if they intersect.
    const PaintStructBoundBox& initialBBox = child->Bounds;

    for (;;)
    {
        auto* ps = child;
        child = child->NextQuadrantEntry;

        if (child == nullptr || child->SortFlags & PaintSortFlags::OutsideQuadrant)
        {
            break;
        }

        if (!(child->SortFlags & PaintSortFlags::Neighbour))
        {
            continue;
        }

        if (CheckBoundingBox<TRotation>(initialBBox, child->Bounds))
        {
            // Child node intersects with current node, move behind.
            ps->NextQuadrantEntry = child->NextQuadrantEntry;

            auto* psTemp = parent->NextQuadrantEntry;
            parent->NextQuadrantEntry = child;

            child->NextQuadrantEntry = psTemp;
            child = ps;
        }
    }
}

// Re-orders all nodes after the specified child node and marks the child node as traversed. The resulting
// order of the children is the depth based on rotation and dimensions of the bounding box.
template<uint8_t TRotation>
static void PaintStructsSortQuadrantStable(PaintStruct* parent, PaintStruct* child)
{
    // Mark visited.
    child->SortFlags &= ~PaintSortFlags::PendingVisit;

    // Compare all the children below the first child and move them up in the list if they intersect.
    const PaintStructBoundBox& initialBBox = child->Bounds;

    // Create a temporary list to collect sorted nodes in stable order.
    PaintStruct* sortedHead = nullptr;
    PaintStruct* sortedTail = nullptr;

    // Traverse the list and reorder based on intersection.
    for (;;)
    {
        PaintStruct* next = child->NextQuadrantEntry;

        // Stop if at the end of the list or outside the quadrant range.
        if (next == nullptr || next->SortFlags & PaintSortFlags::OutsideQuadrant)
        {
            break;
        }

        // Ignore nodes that are not neighbors.
        if (!(next->SortFlags & PaintSortFlags::Neighbour))
        {
            child = next;
            continue;
        }

        // Detach the current node from the list if it intersects.
        if (CheckBoundingBox<TRotation>(initialBBox, next->Bounds))
        {
            child->NextQuadrantEntry = next->NextQuadrantEntry;

            if (sortedHead == nullptr)
            {
                sortedHead = next;
                sortedTail = next;
                next->NextQuadrantEntry = nullptr;
            }
            else
            {
                sortedTail->NextQuadrantEntry = next;
                sortedTail = next;
                next->NextQuadrantEntry = nullptr;
            }
        }
        else
        {
            child = next;
        }
    }

    // Merge the sorted list back into the main list after parent.
    if (sortedHead != nullptr)
    {
        PaintStruct* originalNext = parent->NextQuadrantEntry;
        parent->NextQuadrantEntry = sortedHead;
        sortedTail->NextQuadrantEntry = originalNext;
    }
}

template<bool TStableSort, uint8_t TRotation>
static PaintStruct* PaintArrangeStructsHelperRotation(PaintStruct* psQuadrantEntry, uint16_t quadrantIndex, uint8_t flag)
{
    // We keep track of the first node in the quadrant so the next call with a higher quadrant index
    // can use this node to skip some iterations.
    psQuadrantEntry = PaintStructsFirstInQuadrant(psQuadrantEntry, quadrantIndex);

    // Visit all nodes in the linked quadrant list and determine their current
    // sorting relevancy.
    PaintStructsInitializeSort(psQuadrantEntry, quadrantIndex, flag);

    // Iterate all nodes in the current list and re-order them based on
    // the current rotation and their bounding box.
    for (auto* ps = psQuadrantEntry; ps != nullptr;)
    {
        const auto [parent, child] = PaintStructsGetNextPending(ps);
        if (parent == nullptr)
        {
            break;
        }

        if constexpr (TStableSort)
        {
            PaintStructsSortQuadrantStable<TRotation>(parent, child);
        }
        else
        {
            PaintStructsSortQuadrantLegacy<TRotation>(parent, child);
        }

        ps = parent;
    }

    return psQuadrantEntry;
}

// Iterates over all the quadrant lists and links them together as a
// singly linked list.
// The paint session has a head member which is the first entry.
static void PaintStructsLinkQuadrants(PaintSessionCore& session, PaintStruct& psHead)
{
    PaintStruct* ps = &psHead;
    ps->NextQuadrantEntry = nullptr;

    uint32_t quadrantIndex = session.QuadrantBackIndex;
    do
    {
        PaintStruct* psNext = session.Quadrants[quadrantIndex];
        if (psNext != nullptr)
        {
            ps->NextQuadrantEntry = psNext;
            do
            {
                ps = psNext;
                psNext = psNext->NextQuadrantEntry;

            } while (psNext != nullptr);
        }
    } while (++quadrantIndex <= session.QuadrantFrontIndex);
}

template<bool TStableSort, int TRotation>
static void PaintSessionArrangeImpl(PaintSessionCore& session)
{
    uint32_t quadrantIndex = session.QuadrantBackIndex;
    if (quadrantIndex == UINT32_MAX)
    {
        return;
    }

    // psHead is an intermediate node that is used to link all the quadrant lists together,
    // this was previously stored in PaintSession but only the NextQuadrantEntry is relevant here.
    // The head node is not part of the linked list and just serves as an entry point.
    PaintStruct psHead{};
    PaintStructsLinkQuadrants(session, psHead);

    PaintStruct* psNextQuadrant = PaintArrangeStructsHelperRotation<TStableSort, TRotation>(
        &psHead, session.QuadrantBackIndex, PaintSortFlags::Neighbour);

    while (++quadrantIndex < session.QuadrantFrontIndex)
    {
        psNextQuadrant = PaintArrangeStructsHelperRotation<TStableSort, TRotation>(
            psNextQuadrant, quadrantIndex, PaintSortFlags::None);
    }

    session.PaintHead = psHead.NextQuadrantEntry;
}

using PaintArrangeWithRotation = void (*)(PaintSessionCore& session);

constexpr std::array _paintArrangeFuncsLegacy = {
    PaintSessionArrangeImpl<false, 0>,
    PaintSessionArrangeImpl<false, 1>,
    PaintSessionArrangeImpl<false, 2>,
    PaintSessionArrangeImpl<false, 3>,
};

constexpr std::array _paintArrangeFuncsStable = {
    PaintSessionArrangeImpl<true, 0>,
    PaintSessionArrangeImpl<true, 1>,
    PaintSessionArrangeImpl<true, 2>,
    PaintSessionArrangeImpl<true, 3>,
};

/**
 *
 *  rct2: 0x00688217
 */
void PaintSessionArrange(PaintSessionCore& session)
{
    PROFILED_FUNCTION();
    if (gPaintStableSort)
    {
        return _paintArrangeFuncsStable[session.CurrentRotation](session);
    }
    return _paintArrangeFuncsLegacy[session.CurrentRotation](session);
}

static inline void PaintAttachedPS(RenderTarget& rt, PaintStruct* ps, uint32_t viewFlags)
{
    AttachedPaintStruct* attached_ps = ps->Attached;
    for (; attached_ps != nullptr; attached_ps = attached_ps->NextEntry)
    {
        const auto screenCoords = ps->ScreenPos + attached_ps->RelativePos;

        auto imageId = PaintPSColourifyImage(ps, attached_ps->image_id, viewFlags);
        if (attached_ps->IsMasked)
        {
            GfxDrawSpriteRawMasked(rt, screenCoords, imageId, attached_ps->ColourImageId);
        }
        else
        {
            GfxDrawSprite(rt, imageId, screenCoords);
        }
    }
}

static inline void PaintDrawStruct(PaintSession& session, PaintStruct* ps)
{
    auto screenPos = ps->ScreenPos;
    if (ps->InteractionItem == ViewportInteractionItem::entity)
    {
        if (session.rt.zoom_level >= ZoomLevel{ 1 })
        {
            screenPos.x = floor2(screenPos.x, 2);
            screenPos.y = floor2(screenPos.y, 2);
            if (session.rt.zoom_level >= ZoomLevel{ 2 })
            {
                screenPos.x = floor2(screenPos.x, 4);
                screenPos.y = floor2(screenPos.y, 4);
            }
        }
    }
    auto imageId = PaintPSColourifyImage(ps, ps->image_id, session.ViewFlags);
    if (gPaintBoundingBoxes)
    {
        PaintPSImageWithBoundingBoxes(session, ps, imageId, screenPos.x, screenPos.y);
    }
    else
    {
        GfxDrawSprite(session.rt, imageId, screenPos);
    }

    if (ps->Children != nullptr)
    {
        PaintDrawStruct(session, ps->Children);
    }
    else
    {
        PaintAttachedPS(session.rt, ps, session.ViewFlags);
    }
}

/**
 *
 *  rct2: 0x00688485
 */
void PaintDrawStructs(PaintSession& session)
{
    PROFILED_FUNCTION();

    for (PaintStruct* ps = session.PaintHead; ps != nullptr; ps = ps->NextQuadrantEntry)
    {
        PaintDrawStruct(session, ps);
    }
}

static void PaintPSImageWithBoundingBoxes(PaintSession& session, PaintStruct* ps, ImageId imageId, int32_t x, int32_t y)
{
    auto& rt = session.rt;

    const PaletteIndex colour = kBoundBoxDebugColours[EnumValue(ps->InteractionItem)];
    const uint8_t rotation = session.CurrentRotation;

    const CoordsXYZ frontTop = {
        ps->Bounds.x_end,
        ps->Bounds.y_end,
        ps->Bounds.z_end,
    };
    const auto screenCoordFrontTop = Translate3DTo2DWithZ(rotation, frontTop);

    const CoordsXYZ frontBottom = {
        ps->Bounds.x_end,
        ps->Bounds.y_end,
        ps->Bounds.z,
    };
    const auto screenCoordFrontBottom = Translate3DTo2DWithZ(rotation, frontBottom);

    const CoordsXYZ leftTop = {
        ps->Bounds.x,
        ps->Bounds.y_end,
        ps->Bounds.z_end,
    };
    const auto screenCoordLeftTop = Translate3DTo2DWithZ(rotation, leftTop);

    const CoordsXYZ leftBottom = {
        ps->Bounds.x,
        ps->Bounds.y_end,
        ps->Bounds.z,
    };
    const auto screenCoordLeftBottom = Translate3DTo2DWithZ(rotation, leftBottom);

    const CoordsXYZ rightTop = {
        ps->Bounds.x_end,
        ps->Bounds.y,
        ps->Bounds.z_end,
    };
    const auto screenCoordRightTop = Translate3DTo2DWithZ(rotation, rightTop);

    const CoordsXYZ rightBottom = {
        ps->Bounds.x_end,
        ps->Bounds.y,
        ps->Bounds.z,
    };
    const auto screenCoordRightBottom = Translate3DTo2DWithZ(rotation, rightBottom);

    const CoordsXYZ backTop = {
        ps->Bounds.x,
        ps->Bounds.y,
        ps->Bounds.z_end,
    };
    const auto screenCoordBackTop = Translate3DTo2DWithZ(rotation, backTop);

    const CoordsXYZ backBottom = {
        ps->Bounds.x,
        ps->Bounds.y,
        ps->Bounds.z,
    };
    const auto screenCoordBackBottom = Translate3DTo2DWithZ(rotation, backBottom);

    // bottom square
    GfxDrawLine(rt, { screenCoordFrontBottom, screenCoordLeftBottom }, colour);
    GfxDrawLine(rt, { screenCoordBackBottom, screenCoordLeftBottom }, colour);
    GfxDrawLine(rt, { screenCoordBackBottom, screenCoordRightBottom }, colour);
    GfxDrawLine(rt, { screenCoordFrontBottom, screenCoordRightBottom }, colour);

    // vertical back + sides
    GfxDrawLine(rt, { screenCoordBackTop, screenCoordBackBottom }, colour);
    GfxDrawLine(rt, { screenCoordLeftTop, screenCoordLeftBottom }, colour);
    GfxDrawLine(rt, { screenCoordRightTop, screenCoordRightBottom }, colour);

    // top square back
    GfxDrawLine(rt, { screenCoordBackTop, screenCoordLeftTop }, colour);
    GfxDrawLine(rt, { screenCoordBackTop, screenCoordRightTop }, colour);

    GfxDrawSprite(rt, imageId, { x, y });

    // vertical front
    GfxDrawLine(rt, { screenCoordFrontTop, screenCoordFrontBottom }, colour);

    // top square
    GfxDrawLine(rt, { screenCoordFrontTop, screenCoordLeftTop }, colour);
    GfxDrawLine(rt, { screenCoordFrontTop, screenCoordRightTop }, colour);
}

static ImageId PaintPSColourifyImage(const PaintStruct* ps, ImageId imageId, uint32_t viewFlags)
{
    auto visibility = GetPaintStructVisibility(ps, viewFlags);
    switch (visibility)
    {
        case VisibilityKind::partial:
            return imageId.WithTransparency(FilterPaletteID::paletteDarken1);
        case VisibilityKind::hidden:
            return ImageId();
        default:
            return imageId;
    }
}

PaintSession* PaintSessionAlloc(RenderTarget& rt, uint32_t viewFlags, uint8_t rotation)
{
    return GetContext()->GetPainter()->CreateSession(rt, viewFlags, rotation);
}

void PaintSessionFree(PaintSession* session)
{
    GetContext()->GetPainter()->ReleaseSession(session);
}

void PaintSessionPublishFirstPersonPassengerAnchor(
    PaintSession& session, EntityBase& entity,
    uint32_t seatMask, float worldX, float worldY, float worldZ,
    bool hasEyeOffset, float eyeForward,
    float eyeRight, float eyeUp,
    bool hasLocalPitch, float localPitch)
{
    if (session.FirstPersonPassengerAnchorSink == nullptr
        || session.FirstPersonPassengerAnchorEntity != &entity
        || session.FirstPersonPassengerAnchorSeatIndex >= 32
        || (seatMask
            & (uint32_t{ 1 }
                << session.FirstPersonPassengerAnchorSeatIndex))
            == 0)
        return;

    *session.FirstPersonPassengerAnchorSink = {
        &entity,
        seatMask,
        worldX,
        worldY,
        worldZ,
        hasEyeOffset,
        eyeForward,
        eyeRight,
        eyeUp,
        hasLocalPitch,
        localPitch,
    };
}

void PaintSessionPublishFirstPersonPassengerLocalAnchor(
    PaintSession& session, EntityBase& entity,
    uint32_t seatMask, float localX, float localY, float localZ,
    bool hasEyeOffset, float eyeForward,
    float eyeRight, float eyeUp,
    bool hasLocalPitch, float localPitch)
{
    const uint8_t rotation =
        DirectionFlipXAxis(session.CurrentRotation) & 3u;
    float rotatedX = localX;
    float rotatedY = localY;
    switch (rotation)
    {
        case 1:
            rotatedX = localY;
            rotatedY = -localX;
            break;
        case 2:
            rotatedX = -localX;
            rotatedY = -localY;
            break;
        case 3:
            rotatedX = -localY;
            rotatedY = localX;
            break;
        default:
            break;
    }
    PaintSessionPublishFirstPersonPassengerAnchor(
        session, entity, seatMask,
        float(session.SpritePosition.x) + rotatedX,
        float(session.SpritePosition.y) + rotatedY,
        localZ,
        hasEyeOffset, eyeForward, eyeRight, eyeUp,
        hasLocalPitch, localPitch);
}

static FirstPersonPaintSemanticVec3 TransformFirstPersonSemanticPoint(
    const FirstPersonPaintSemanticTransform& transform,
    FirstPersonPaintSemanticVec3 point)
{
    return {
        transform.origin.x
            + transform.axisX.x * point.x
            + transform.axisY.x * point.y
            + transform.axisZ.x * point.z,
        transform.origin.y
            + transform.axisX.y * point.x
            + transform.axisY.y * point.y
            + transform.axisZ.y * point.z,
        transform.origin.z
            + transform.axisX.z * point.x
            + transform.axisY.z * point.y
            + transform.axisZ.z * point.z,
    };
}

static ScreenCoordsXY FirstPersonSemanticArtworkScreenPos(
    const PaintSession& session, const CoordsXYZ& offset)
{
    const auto swappedRotation =
        DirectionFlipXAxis(session.CurrentRotation);
    auto source = CoordsXYZ{
        offset.rotate(swappedRotation), offset.z
    };
    source += session.SpritePosition;
    return Translate3DTo2DWithZ(
        session.CurrentRotation, source);
}

uint32_t PaintSessionBeginFirstPersonSemanticArtworkGroup(
    PaintSession& session)
{
    uint32_t group =
        session.FirstPersonSemanticNextArtworkGroup++;
    if (group == 0)
        group = session.FirstPersonSemanticNextArtworkGroup++;
    return group;
}

FirstPersonPaintSemanticTransform
    PaintSessionMakeFirstPersonSemanticTransform(
        const PaintSession& session,
        FirstPersonPaintSemanticVec3 localOrigin)
{
    const uint8_t rotation =
        DirectionFlipXAxis(session.CurrentRotation) & 3u;
    FirstPersonPaintSemanticTransform transform{};
    transform.origin = {
        float(session.SpritePosition.x),
        float(session.SpritePosition.y),
        0.0f,
    };
    switch (rotation)
    {
        case 1:
            transform.axisX = { 0.0f, -1.0f, 0.0f };
            transform.axisY = { 1.0f, 0.0f, 0.0f };
            break;
        case 2:
            transform.axisX = { -1.0f, 0.0f, 0.0f };
            transform.axisY = { 0.0f, -1.0f, 0.0f };
            break;
        case 3:
            transform.axisX = { 0.0f, 1.0f, 0.0f };
            transform.axisY = { -1.0f, 0.0f, 0.0f };
            break;
        default:
            break;
    }
    const auto worldOrigin =
        TransformFirstPersonSemanticPoint(
            transform, localOrigin);
    transform.origin = worldOrigin;
    return transform;
}

static FirstPersonPaintSemanticArtwork
    MakeFirstPersonSemanticArtwork(
        const PaintSession& session, ImageId image,
        const CoordsXYZ& artworkOffset, uint32_t artworkGroup,
        bool decal = false)
{
    FirstPersonPaintSemanticArtwork artwork{};
    artwork.image = image;
    artwork.screenPos =
        FirstPersonSemanticArtworkScreenPos(
            session, artworkOffset);
    const uint32_t snapshotHandle =
        Drawing::ScrollingText::CaptureFirstPersonSnapshot(image);
    if (snapshotHandle != 0)
    {
        const auto* snapshot =
            Drawing::ScrollingText::GetFirstPersonSnapshot(
                snapshotHandle);
        if (snapshot != nullptr
            && !snapshot->pixels.empty())
        {
            artwork.immutablePixels =
                snapshot->pixels;
            artwork.immutableWidth =
                snapshot->width;
            artwork.immutableHeight =
                snapshot->height;
        }
    }
    artwork.group = artworkGroup != 0
        ? artworkGroup
        : session.FirstPersonSemanticArtworkGroup;
    artwork.sourceRotation = session.CurrentRotation;
    artwork.decal = decal;
    return artwork;
}

static uint32_t PublishFirstPersonSemanticComponent(
    PaintSession& session,
    FirstPersonPaintSemanticComponent component)
{
    if (session.FirstPersonSemanticComponentSink == nullptr)
        return 0;
    uint32_t id = session.FirstPersonSemanticNextComponentId++;
    if (id == 0)
        id = session.FirstPersonSemanticNextComponentId++;
    component.id = id;
    component.mapPosition = session.MapPosition;
    session.FirstPersonSemanticComponentSink->push_back(
        std::move(component));
    return id;
}

uint32_t PaintSessionAddFirstPersonSemanticOrientedBox(
    PaintSession& session, FirstPersonPaintSemanticRole role,
    const FirstPersonPaintSemanticTransform& transform,
    FirstPersonPaintSemanticVec3 localLow,
    FirstPersonPaintSemanticVec3 localHigh,
    ImageId image, const CoordsXYZ& artworkOffset,
    uint32_t artworkGroup, uint64_t localHullKey,
    uint16_t repetitionIndex, bool collidable, bool decal)
{
    if (!(localHigh.x > localLow.x)
        || !(localHigh.y > localLow.y)
        || !(localHigh.z > localLow.z))
        return 0;

    FirstPersonPaintSemanticComponent component{};
    component.role = role;
    component.geometry.kind =
        localHullKey != 0
        ? FirstPersonPaintSemanticPrimitiveKind::localHull
        : FirstPersonPaintSemanticPrimitiveKind::box;
    component.geometry.points[0] = localLow;
    component.geometry.points[1] = localHigh;
    component.geometry.pointCount = 2;
    component.geometry.localHullKey = localHullKey;
    component.transform = transform;
    component.artwork = MakeFirstPersonSemanticArtwork(
        session, image, artworkOffset, artworkGroup, decal);
    component.repetitionIndex = repetitionIndex;
    component.collidable = collidable;
    return PublishFirstPersonSemanticComponent(
        session, std::move(component));
}

uint32_t PaintSessionAddFirstPersonSemanticBox(
    PaintSession& session, FirstPersonPaintSemanticRole role,
    const CoordsXYZ& localLow, const CoordsXYZ& localHigh,
    ImageId image, const CoordsXYZ& artworkOffset,
    uint32_t artworkGroup, uint64_t localHullKey,
    uint16_t repetitionIndex, bool collidable, bool decal)
{
    return PaintSessionAddFirstPersonSemanticOrientedBox(
        session, role,
        PaintSessionMakeFirstPersonSemanticTransform(session),
        { float(localLow.x), float(localLow.y),
          float(localLow.z) },
        { float(localHigh.x), float(localHigh.y),
          float(localHigh.z) },
        image, artworkOffset, artworkGroup, localHullKey,
        repetitionIndex, collidable, decal);
}

uint32_t PaintSessionAddFirstPersonSemanticBeam(
    PaintSession& session, FirstPersonPaintSemanticRole role,
    const CoordsXYZ& localA, const CoordsXYZ& localB,
    int32_t halfWidth, ImageId image,
    const CoordsXYZ& artworkOffset,
    uint32_t artworkGroup, bool collidable)
{
    if (localA.x == localB.x
        && localA.y == localB.y
        && localA.z == localB.z)
        return 0;
    FirstPersonPaintSemanticComponent component{};
    component.role = role;
    component.geometry.kind =
        FirstPersonPaintSemanticPrimitiveKind::beam;
    component.geometry.points[0] = {
        float(localA.x), float(localA.y), float(localA.z)
    };
    component.geometry.points[1] = {
        float(localB.x), float(localB.y), float(localB.z)
    };
    component.geometry.pointCount = 2;
    component.geometry.halfWidth =
        float(std::max(1, halfWidth));
    component.geometry.halfHeight =
        float(std::max(1, halfWidth));
    component.transform =
        PaintSessionMakeFirstPersonSemanticTransform(session);
    component.artwork = MakeFirstPersonSemanticArtwork(
        session, image, artworkOffset, artworkGroup);
    component.collidable = collidable;
    return PublishFirstPersonSemanticComponent(
        session, std::move(component));
}

uint32_t PaintSessionAddFirstPersonSemanticQuad(
    PaintSession& session, FirstPersonPaintSemanticRole role,
    FirstPersonPaintSemanticPrimitiveKind kind,
    const std::array<CoordsXYZ, 4>& localCorners,
    ImageId image, const CoordsXYZ& artworkOffset,
    uint32_t artworkGroup, bool decal, bool collidable)
{
    if (kind != FirstPersonPaintSemanticPrimitiveKind::plane
        && kind !=
            FirstPersonPaintSemanticPrimitiveKind::footprint
        && kind != FirstPersonPaintSemanticPrimitiveKind::opening)
        return 0;

    FirstPersonPaintSemanticComponent component{};
    component.role = role;
    component.geometry.kind = kind;
    component.geometry.pointCount = 4;
    for (size_t i = 0;
         i < component.geometry.points.size(); ++i)
    {
        component.geometry.points[i] = {
            float(localCorners[i].x),
            float(localCorners[i].y),
            float(localCorners[i].z),
        };
    }
    component.transform =
        PaintSessionMakeFirstPersonSemanticTransform(session);
    component.artwork = MakeFirstPersonSemanticArtwork(
        session, image, artworkOffset, artworkGroup, decal);
    component.collidable = collidable;
    return PublishFirstPersonSemanticComponent(
        session, std::move(component));
}

uint32_t PaintSessionAddFirstPersonSemanticOrientedQuad(
    PaintSession& session, FirstPersonPaintSemanticRole role,
    FirstPersonPaintSemanticPrimitiveKind kind,
    const FirstPersonPaintSemanticTransform& transform,
    const std::array<FirstPersonPaintSemanticVec3, 4>& localCorners,
    ImageId image, const CoordsXYZ& artworkOffset,
    uint32_t artworkGroup, bool decal, bool collidable)
{
    if (kind != FirstPersonPaintSemanticPrimitiveKind::plane
        && kind !=
            FirstPersonPaintSemanticPrimitiveKind::footprint
        && kind != FirstPersonPaintSemanticPrimitiveKind::opening)
        return 0;

    FirstPersonPaintSemanticComponent component{};
    component.role = role;
    component.geometry.kind = kind;
    component.geometry.points = localCorners;
    component.geometry.pointCount = 4;
    component.transform = transform;
    component.artwork = MakeFirstPersonSemanticArtwork(
        session, image, artworkOffset, artworkGroup, decal);
    component.collidable = collidable;
    return PublishFirstPersonSemanticComponent(
        session, std::move(component));
}


void PaintSessionPublishFirstPersonPassengerComponentAnchor(
    PaintSession& session, EntityBase& entity,
    uint32_t seatMask, uint32_t componentId,
    const FirstPersonPaintSemanticTransform& transform,
    FirstPersonPaintSemanticVec3 localAnchor,
    bool hasEyeOffset, float eyeForward,
    float eyeRight, float eyeUp,
    bool hasLocalPitch, float localPitch)
{
    const auto world =
        TransformFirstPersonSemanticPoint(
            transform, localAnchor);
    PaintSessionPublishFirstPersonPassengerAnchor(
        session, entity, seatMask,
        world.x, world.y, world.z,
        hasEyeOffset, eyeForward, eyeRight, eyeUp,
        hasLocalPitch, localPitch);

    if (session.FirstPersonPassengerAnchorSink != nullptr
        && session.FirstPersonPassengerAnchorSink->Entity
            == &entity
        && (session.FirstPersonPassengerAnchorSink->seatMask
            & seatMask) != 0)
    {
        session.FirstPersonPassengerAnchorSink
            ->semanticComponentId = componentId;
        session.FirstPersonPassengerAnchorSink
            ->componentLocalAnchor = localAnchor;
    }
}

/**
 *  rct2: 0x00686806, 0x006869B2, 0x00686B6F, 0x00686D31, 0x0098197C
 *
 * @param image_id (ebx)
 * @param x_offset (al)
 * @param y_offset (cl)
 * @param bound_box_length_x (di)
 * @param bound_box_length_y (si)
 * @param bound_box_length_z (ah)
 * @param z_offset (dx)
 * @param bound_box_offset_x (0x009DEA52)
 * @param bound_box_offset_y (0x009DEA54)
 * @param bound_box_offset_z (0x009DEA56)
 * @return (ebp) PaintStruct on success (CF == 0), nullptr on failure (CF == 1)
 */
// Track Pieces, Shops.
PaintStruct* PaintAddImageAsParent(
    PaintSession& session, const ImageId image_id, const CoordsXYZ& offset, const BoundBoxXYZ& boundBox)
{
    session.LastPS = nullptr;
    session.LastAttachedPS = nullptr;

    auto* ps = CreateNormalPaintStruct(session, image_id, offset, boundBox);
    if (ps == nullptr)
    {
        return nullptr;
    }

    PaintSessionAddPSToQuadrant(session, ps);

    return ps;
}

/**
 *
 *  rct2: 0x00686EF0, 0x00687056, 0x006871C8, 0x0068733C, 0x0098198C
 *
 * @param image_id (ebx)
 * @param x_offset (al)
 * @param y_offset (cl)
 * @param bound_box_length_x (di)
 * @param bound_box_length_y (si)
 * @param bound_box_length_z (ah)
 * @param z_offset (dx)
 * @param bound_box_offset_x (0x009DEA52)
 * @param bound_box_offset_y (0x009DEA54)
 * @param bound_box_offset_z (0x009DEA56)
 * @return (ebp) PaintStruct on success (CF == 0), nullptr on failure (CF == 1)
 * Creates a paint struct but does not allocate to a paint quadrant. Result cannot be ignored!
 */
[[nodiscard]] PaintStruct* PaintAddImageAsOrphan(
    PaintSession& session, const ImageId imageId, const CoordsXYZ& offset, const BoundBoxXYZ& boundBox)
{
    session.LastPS = nullptr;
    session.LastAttachedPS = nullptr;
    return CreateNormalPaintStruct(session, imageId, offset, boundBox);
}

/**
 *
 *  rct2: 0x006874B0, 0x00687618, 0x0068778C, 0x00687902, 0x0098199C
 *
 * @param image_id (ebx)
 * @param x_offset (al)
 * @param y_offset (cl)
 * @param bound_box_length_x (di)
 * @param bound_box_length_y (si)
 * @param bound_box_length_z (ah)
 * @param z_offset (dx)
 * @param bound_box_offset_x (0x009DEA52)
 * @param bound_box_offset_y (0x009DEA54)
 * @param bound_box_offset_z (0x009DEA56)
 * @return (ebp) PaintStruct on success (CF == 0), nullptr on failure (CF == 1)
 * If there is no parent paint struct then image is added as a parent
 */
PaintStruct* PaintAddImageAsChild(
    PaintSession& session, const ImageId image_id, const CoordsXYZ& offset, const BoundBoxXYZ& boundBox)
{
    PaintStruct* parentPS = session.LastPS;
    if (parentPS == nullptr)
    {
        return PaintAddImageAsParent(session, image_id, offset, boundBox);
    }

    auto* ps = CreateNormalPaintStruct(session, image_id, offset, boundBox);
    if (ps == nullptr)
    {
        return nullptr;
    }

    parentPS->Children = ps;

    return ps;
}

PaintStruct* PaintAddImageAsParentHeight(
    PaintSession& session, const ImageId imageId, const int32_t height, const CoordsXYZ& offset, const BoundBoxXYZ& boundBox)
{
    session.LastPS = nullptr;
    session.LastAttachedPS = nullptr;

    auto* const ps = CreateNormalPaintStructHeight(session, imageId, height, offset, boundBox);
    if (ps == nullptr)
    {
        return nullptr;
    }

    PaintSessionAddPSToQuadrant(session, ps);

    return ps;
}

/**
 * rct2: 0x006881D0
 *
 * @param image_id (ebx)
 * @param x (ax)
 * @param y (cx)
 * @return (!CF) success
 */
bool PaintAttachToPreviousAttach(PaintSession& session, const ImageId imageId, int32_t x, int32_t y)
{
    auto* previousAttachedPS = session.LastAttachedPS;
    if (previousAttachedPS == nullptr)
    {
        return PaintAttachToPreviousPS(session, imageId, x, y);
    }

    auto* ps = session.AllocateAttachedPaintEntry();
    if (ps == nullptr)
    {
        return false;
    }

    ps->image_id = imageId;
    ps->FirstPersonSnapshot = Drawing::ScrollingText::CaptureFirstPersonSnapshot(imageId);
    ps->RelativePos = { x, y };
    ps->IsMasked = false;
    ps->NextEntry = nullptr;

    previousAttachedPS->NextEntry = ps;

    return true;
}

/**
 * rct2: 0x0068818E
 *
 * @param image_id (ebx)
 * @param x (ax)
 * @param y (cx)
 * @return (!CF) success
 */
bool PaintAttachToPreviousPS(PaintSession& session, const ImageId image_id, int32_t x, int32_t y)
{
    auto* masterPs = session.LastPS;
    if (masterPs == nullptr)
    {
        return false;
    }

    auto* ps = session.AllocateAttachedPaintEntry();
    if (ps == nullptr)
    {
        return false;
    }

    ps->image_id = image_id;
    ps->FirstPersonSnapshot = Drawing::ScrollingText::CaptureFirstPersonSnapshot(image_id);
    ps->RelativePos = { x, y };
    ps->IsMasked = false;

    AttachedPaintStruct* oldFirstAttached = masterPs->Attached;
    masterPs->Attached = ps;
    ps->NextEntry = oldFirstAttached;

    return true;
}

/**
 * rct2: 0x00685EBC, 0x00686046, 0x00685FC8, 0x00685F4A, 0x00685ECC
 * @param amount (eax)
 * @param string_id (bx)
 * @param y (cx)
 * @param z (dx)
 * @param offset_x (si)
 * @param y_offsets (di)
 * @param rotation (ebp)
 */
void PaintFloatingMoneyEffect(
    PaintSession& session, money64 amount, StringId string_id, int32_t y, int32_t z, int8_t y_offsets[], int32_t offset_x,
    uint32_t rotation)
{
    auto* ps = session.AllocateStringPaintEntry();
    if (ps == nullptr)
    {
        return;
    }

    const CoordsXYZ position = {
        session.SpritePosition.x,
        session.SpritePosition.y,
        z,
    };
    const auto coord = Translate3DTo2DWithZ(rotation, position);

    ps->string_id = string_id;
    ps->NextEntry = nullptr;
    std::memcpy(ps->args, &amount, sizeof(amount));
    ps->args[2] = 0;
    ps->args[3] = 0;
    ps->y_offsets = reinterpret_cast<uint8_t*>(y_offsets);
    ps->ScreenPos = ScreenCoordsXY{ coord.x + offset_x, coord.y };
}

/**
 *
 *  rct2: 0x006860C3
 */
void PaintDrawMoneyStructs(RenderTarget& rt, PaintStringStruct* ps)
{
    do
    {
        char buffer[256]{};
        FormatStringLegacy(buffer, sizeof(buffer), ps->string_id, &ps->args);

        // Use sprite font unless the currency contains characters unsupported by the sprite font
        auto forceSpriteFont = false;
        const auto& currencyDesc = CurrencyDescriptors[EnumValue(Config::Get().general.currencyFormat)];
        if (LocalisationService_UseTrueTypeFont() && FontSupportsStringSprite(currencyDesc.symbol_unicode))
        {
            forceSpriteFont = true;
        }

        drawStringWithYOffsets(
            rt, buffer, { Drawing::Colour::black }, ps->ScreenPos, reinterpret_cast<int8_t*>(ps->y_offsets), forceSpriteFont,
            FontStyle::medium);
    } while ((ps = ps->NextEntry) != nullptr);
}
