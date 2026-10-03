/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../core/Money.hpp"
#include "../drawing/FilterPaletteIds.h"
#include "../drawing/ImageId.hpp"
#include "../drawing/RenderTarget.h"
#include "../localisation/StringIdType.h"
#include "../world/Location.hpp"
#include "../world/MapLimits.h"
#include "Boundbox.h"
#include "tile_element/Paint.Tunnel.h"

#include <array>
#include <optional>
#include <vector>
#include <sfl/segmented_vector.hpp>
#include <sfl/static_vector.hpp>

enum class ViewportInteractionItem : uint8_t;

namespace OpenRCT2
{
    struct EntityBase;

    struct TileElement;
    struct SurfaceElement;

} // namespace OpenRCT2

struct AttachedPaintStruct
{
    AttachedPaintStruct* NextEntry;
    ImageId image_id;
    ImageId ColourImageId;
    // This is relative to the parent where we are attached to.
    ScreenCoordsXY RelativePos;
    bool IsMasked;
    uint32_t FirstPersonSnapshot = 0;
};

struct PaintStructBoundBox
{
    int32_t x;
    int32_t y;
    int32_t z;
    int32_t x_end;
    int32_t y_end;
    int32_t z_end;
};

enum class PaintStructSource : uint8_t
{
    unknown,
    tile,
    entity,
};

enum class FirstPersonPaintSemanticRole : uint8_t
{
    none,
    support,
    pathDeck,
    railing,
    pathFixture,
    stationFloor,
    stationFence,
    stationCover,
    wall,
    terrainEdge,
    structureBody,
    structureFloor,
    structureRoof,
    pier,
    towerSection,
    movingMachinery,
    seat,
    sign,
};

enum class FirstPersonPaintSemanticPrimitiveKind : uint8_t
{
    plane,
    box,
    beam,
    footprint,
    opening,
    localHull,
};

struct FirstPersonPaintSemanticVec3
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

[[nodiscard]] inline std::array<FirstPersonPaintSemanticVec3, 4>
    FirstPersonWallSemanticCorners(
        uint8_t direction, uint8_t slope, int32_t height)
{
    const float h = float(height > 0 ? height : 0);
    const float step = float(2 * kCoordsZStep);
    FirstPersonPaintSemanticVec3 a{};
    FirstPersonPaintSemanticVec3 b{};
    switch (direction & 3u)
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
    if ((slope & 1u) != 0)
        b.z += step;
    else if ((slope & 2u) != 0)
        a.z += step;

    return { {
        a,
        b,
        { b.x, b.y, b.z + h },
        { a.x, a.y, a.z + h },
    } };
}

struct FirstPersonPaintSemanticGeometry
{
    FirstPersonPaintSemanticPrimitiveKind kind =
        FirstPersonPaintSemanticPrimitiveKind::box;
    std::array<FirstPersonPaintSemanticVec3, 4> points{};
    uint8_t pointCount = 0;
    float halfWidth = 0.0f;
    float halfHeight = 0.0f;
    uint64_t localHullKey = 0;
};

struct FirstPersonPaintSemanticTransform
{
    FirstPersonPaintSemanticVec3 origin{};
    FirstPersonPaintSemanticVec3 axisX{ 1.0f, 0.0f, 0.0f };
    FirstPersonPaintSemanticVec3 axisY{ 0.0f, 1.0f, 0.0f };
    FirstPersonPaintSemanticVec3 axisZ{ 0.0f, 0.0f, 1.0f };
};

struct FirstPersonPaintSemanticArtwork
{
    ImageId image{};
    ImageId mask{};
    ScreenCoordsXY screenPos{};
    uint32_t group = 0;
    // Dynamic native artwork (notably scrolling text) must survive beyond the
    // paint session that produced it. Own the captured indexed pixels here
    // rather than retaining a temporary global snapshot handle.
    std::vector<uint8_t> immutablePixels;
    int16_t immutableWidth = 0;
    int16_t immutableHeight = 0;
    uint8_t sourceRotation = 0;
    // Logical order assigned from the arranged native PaintStruct stream.
    // This is used only to resolve surfaces at the same physical depth.
    uint64_t nativePaintOrdinal = 0;
    bool decal = false;
};

struct FirstPersonPaintSemanticComponent
{
    uint32_t id = 0;
    FirstPersonPaintSemanticRole role =
        FirstPersonPaintSemanticRole::none;
    CoordsXY mapPosition{};
    FirstPersonPaintSemanticGeometry geometry{};
    FirstPersonPaintSemanticTransform transform{};
    FirstPersonPaintSemanticArtwork artwork{};
    uint16_t repetitionIndex = 0;
    bool collidable = true;
};

struct PassengerPaintAnchor
{
    OpenRCT2::EntityBase* Entity = nullptr;
    uint32_t seatMask = 0;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    bool hasEyeOffset = false;
    float eyeForward = 0.0f;
    float eyeRight = 0.0f;
    float eyeUp = 0.0f;
    bool hasLocalPitch = false;
    float localPitch = 0.0f;
    uint32_t semanticComponentId = 0;
    FirstPersonPaintSemanticVec3 componentLocalAnchor{};
};

struct FirstPersonPassengerPaintInterpolation
{
    bool enabled = false;
    uint8_t primaryBefore = 0;
    uint8_t primaryAfter = 0;
    uint8_t secondaryBefore = 0;
    uint8_t secondaryAfter = 0;
    float alpha = 0.0f;
};

struct PaintStruct
{
    PaintStructBoundBox Bounds;
    AttachedPaintStruct* Attached;
    PaintStruct* Children;
    PaintStruct* NextQuadrantEntry;
    OpenRCT2::TileElement* Element;
    OpenRCT2::EntityBase* Entity;
    // Which native painter produced this artwork. Entity remains interaction
    // ownership and is not evidence that the entity painter produced the art.
    PaintStructSource Source = PaintStructSource::unknown;
    FirstPersonPaintSemanticRole FirstPersonSemanticRole =
        FirstPersonPaintSemanticRole::none;
    uint32_t FirstPersonPassengerSeatMask = 0;
    uint32_t FirstPersonSemanticArtworkGroup = 0;
    ImageId image_id;
    ScreenCoordsXY ScreenPos;
    CoordsXY MapPos;
    uint16_t QuadrantIndex;
    uint8_t SortFlags;
    ViewportInteractionItem InteractionItem;
    uint32_t FirstPersonSnapshot = 0;
};

struct PaintStringStruct
{
    StringId string_id;
    PaintStringStruct* NextEntry;
    ScreenCoordsXY ScreenPos;
    uint32_t args[4];
    uint8_t* y_offsets;
};

struct PaintEntry
{
private:
    // Avoid including expensive <algorithm> for std::max. Manually ensure we use the largest type.
    static_assert(sizeof(PaintStruct) >= sizeof(AttachedPaintStruct));
    static_assert(sizeof(PaintStruct) >= sizeof(PaintStringStruct));
    std::array<uint8_t, sizeof(PaintStruct)> data;

public:
    PaintStruct* AsBasic()
    {
        auto* res = reinterpret_cast<PaintStruct*>(data.data());
        ::new (res) PaintStruct();
        return res;
    }
    AttachedPaintStruct* AsAttached()
    {
        auto* res = reinterpret_cast<AttachedPaintStruct*>(data.data());
        ::new (res) AttachedPaintStruct();
        return res;
    }
    PaintStringStruct* AsString()
    {
        auto* res = reinterpret_cast<PaintStringStruct*>(data.data());
        ::new (res) PaintStringStruct();
        return res;
    }
};
static_assert(sizeof(PaintEntry) >= sizeof(PaintStruct));
static_assert(sizeof(PaintEntry) >= sizeof(AttachedPaintStruct));
static_assert(sizeof(PaintEntry) >= sizeof(PaintStringStruct));

struct SpriteBb
{
    uint32_t sprite_id;
    CoordsXYZ offset;
    CoordsXYZ bb_offset;
    CoordsXYZ bb_size;
};

struct SupportHeight
{
    uint16_t height;
    uint8_t slope;
    uint8_t pad;
};

// The maximum size must be kMaximumMapSizeTechnical multiplied by 2 because
// the quadrant index is based on the x and y components combined.
static constexpr int32_t MaxPaintQuadrants = kMaximumMapSizeTechnical * 2;

struct PaintSessionCore
{
    PaintStruct* PaintHead;
    PaintStruct* Quadrants[MaxPaintQuadrants];
    PaintStruct* LastPS;
    PaintStringStruct* PSStringHead;
    PaintStringStruct* LastPSString;
    AttachedPaintStruct* LastAttachedPS;
    const OpenRCT2::SurfaceElement* Surface;
    OpenRCT2::EntityBase* CurrentlyDrawnEntity;
    OpenRCT2::TileElement* CurrentlyDrawnTileElement;
    PaintStructSource CurrentSource = PaintStructSource::unknown;
    FirstPersonPaintSemanticRole FirstPersonSemanticRole =
        FirstPersonPaintSemanticRole::none;
    uint32_t FirstPersonPassengerSeatMask = 0;
    PassengerPaintAnchor* FirstPersonPassengerAnchorSink = nullptr;
    OpenRCT2::EntityBase* FirstPersonPassengerAnchorEntity = nullptr;
    uint8_t FirstPersonPassengerAnchorSeatIndex = 0xFF;
    FirstPersonPassengerPaintInterpolation FirstPersonPassengerInterpolation{};
    uint32_t FirstPersonSemanticArtworkGroup = 0;
    uint32_t FirstPersonSemanticNextArtworkGroup = 1;
    uint32_t FirstPersonSemanticNextComponentId = 1;
    std::vector<FirstPersonPaintSemanticComponent>*
        FirstPersonSemanticComponentSink = nullptr;
    const OpenRCT2::TileElement* PathElementOnSameHeight;
    const OpenRCT2::TileElement* TrackElementOnSameHeight;
    const OpenRCT2::TileElement* SelectedElement;
    PaintStruct* WoodenSupportsPrependTo;
    CoordsXY SpritePosition;
    CoordsXY MapPosition;
    uint32_t ViewFlags;
    uint32_t QuadrantBackIndex;
    uint32_t QuadrantFrontIndex;
    ImageId TrackColours;
    ImageId SupportColours;
    SupportHeight SupportSegments[9];
    SupportHeight Support;
    uint16_t WaterHeight;
    sfl::static_vector<TunnelEntry, kTunnelMaxCount> LeftTunnels;
    sfl::static_vector<TunnelEntry, kTunnelMaxCount> RightTunnels;
    uint8_t VerticalTunnelHeight;
    uint8_t CurrentRotation;
    uint8_t Flags;
    ViewportInteractionItem InteractionType;
};

struct PaintNodeStorage
{
    // 1024 is typically enough to cover the column, after its full it will use dynamicPaintEntries.
    sfl::static_vector<PaintEntry, 1024> fixedPaintEntries;

    // This has to be wrapped in optional as it allocates memory before it is used.
    std::optional<sfl::segmented_vector<PaintEntry, 256>> dynamicPaintEntries;

    PaintEntry* allocate()
    {
        if (!fixedPaintEntries.full())
        {
            return &fixedPaintEntries.emplace_back();
        }

        if (!dynamicPaintEntries.has_value())
        {
            dynamicPaintEntries.emplace();
        }

        return &dynamicPaintEntries->emplace_back();
    }

    void clear()
    {
        fixedPaintEntries.clear();
        dynamicPaintEntries.reset();
    }
};

struct PaintSession : public PaintSessionCore
{
    OpenRCT2::Drawing::RenderTarget rt;
    PaintNodeStorage paintEntries;

    PaintStruct* AllocateNormalPaintEntry() noexcept
    {
        auto* entry = paintEntries.allocate();
        LastPS = entry->AsBasic();
        return LastPS;
    }

    AttachedPaintStruct* AllocateAttachedPaintEntry() noexcept
    {
        auto* entry = paintEntries.allocate();
        LastAttachedPS = entry->AsAttached();
        return LastAttachedPS;
    }

    PaintStringStruct* AllocateStringPaintEntry() noexcept
    {
        auto* entry = paintEntries.allocate();

        auto* string = entry->AsString();
        if (LastPSString == nullptr)
        {
            PSStringHead = string;
        }
        else
        {
            LastPSString->NextEntry = string;
        }

        LastPSString = string;
        return LastPSString;
    }
};

extern PaintSession gPaintSession;

void PaintSessionPublishFirstPersonPassengerAnchor(
    PaintSession& session, OpenRCT2::EntityBase& entity,
    uint32_t seatMask, float worldX, float worldY, float worldZ,
    bool hasEyeOffset = false, float eyeForward = 0.0f,
    float eyeRight = 0.0f, float eyeUp = 0.0f,
    bool hasLocalPitch = false, float localPitch = 0.0f);

void PaintSessionPublishFirstPersonPassengerLocalAnchor(
    PaintSession& session, OpenRCT2::EntityBase& entity,
    uint32_t seatMask, float localX, float localY, float localZ,
    bool hasEyeOffset = false, float eyeForward = 0.0f,
    float eyeRight = 0.0f, float eyeUp = 0.0f,
    bool hasLocalPitch = false, float localPitch = 0.0f);

uint32_t PaintSessionBeginFirstPersonSemanticArtworkGroup(
    PaintSession& session);

FirstPersonPaintSemanticTransform
    PaintSessionMakeFirstPersonSemanticTransform(
        const PaintSession& session,
        FirstPersonPaintSemanticVec3 localOrigin = {});

uint32_t PaintSessionAddFirstPersonSemanticBox(
    PaintSession& session, FirstPersonPaintSemanticRole role,
    const CoordsXYZ& localLow, const CoordsXYZ& localHigh,
    ImageId image = {}, const CoordsXYZ& artworkOffset = {},
    uint32_t artworkGroup = 0, uint64_t localHullKey = 0,
    uint16_t repetitionIndex = 0, bool collidable = true,
    bool decal = false);

uint32_t PaintSessionAddFirstPersonSemanticBeam(
    PaintSession& session, FirstPersonPaintSemanticRole role,
    const CoordsXYZ& localA, const CoordsXYZ& localB,
    int32_t halfWidth, ImageId image = {},
    const CoordsXYZ& artworkOffset = {},
    uint32_t artworkGroup = 0, bool collidable = true);

uint32_t PaintSessionAddFirstPersonSemanticQuad(
    PaintSession& session, FirstPersonPaintSemanticRole role,
    FirstPersonPaintSemanticPrimitiveKind kind,
    const std::array<CoordsXYZ, 4>& localCorners,
    ImageId image = {}, const CoordsXYZ& artworkOffset = {},
    uint32_t artworkGroup = 0, bool decal = false,
    bool collidable = true);

uint32_t PaintSessionAddFirstPersonSemanticOrientedQuad(
    PaintSession& session, FirstPersonPaintSemanticRole role,
    FirstPersonPaintSemanticPrimitiveKind kind,
    const FirstPersonPaintSemanticTransform& transform,
    const std::array<FirstPersonPaintSemanticVec3, 4>& localCorners,
    ImageId image = {}, const CoordsXYZ& artworkOffset = {},
    uint32_t artworkGroup = 0, bool decal = false,
    bool collidable = true);

uint32_t PaintSessionAddFirstPersonSemanticOrientedBox(
    PaintSession& session, FirstPersonPaintSemanticRole role,
    const FirstPersonPaintSemanticTransform& transform,
    FirstPersonPaintSemanticVec3 localLow,
    FirstPersonPaintSemanticVec3 localHigh,
    ImageId image = {}, const CoordsXYZ& artworkOffset = {},
    uint32_t artworkGroup = 0, uint64_t localHullKey = 0,
    uint16_t repetitionIndex = 0, bool collidable = true,
    bool decal = false);

void PaintSessionPublishFirstPersonPassengerComponentAnchor(
    PaintSession& session, OpenRCT2::EntityBase& entity,
    uint32_t seatMask, uint32_t componentId,
    const FirstPersonPaintSemanticTransform& transform,
    FirstPersonPaintSemanticVec3 localAnchor = {},
    bool hasEyeOffset = false, float eyeForward = 0.0f,
    float eyeRight = 0.0f, float eyeUp = 0.0f,
    bool hasLocalPitch = false, float localPitch = 0.0f);

struct FirstPersonPaintSemanticScope
{
    PaintSession& session;
    FirstPersonPaintSemanticRole previousRole;
    uint32_t previousArtworkGroup;

    FirstPersonPaintSemanticScope(
        PaintSession& s, FirstPersonPaintSemanticRole role,
        uint32_t artworkGroup = 0)
        : session(s)
        , previousRole(s.FirstPersonSemanticRole)
        , previousArtworkGroup(s.FirstPersonSemanticArtworkGroup)
    {
        session.FirstPersonSemanticRole = role;
        session.FirstPersonSemanticArtworkGroup = artworkGroup;
    }

    ~FirstPersonPaintSemanticScope()
    {
        session.FirstPersonSemanticRole = previousRole;
        session.FirstPersonSemanticArtworkGroup =
            previousArtworkGroup;
    }
};

// Globals for paint clipping
extern uint8_t gClipHeight;
extern CoordsXY gClipSelectionA;
extern CoordsXY gClipSelectionB;

/** rct2: 0x00993CC4. The white ghost that indicates not-yet-built elements. */
constexpr ImageId ConstructionMarker = ImageId(0).WithRemap(OpenRCT2::Drawing::FilterPaletteID::paletteGhost);
constexpr ImageId HighlightMarker = ImageId(0).WithRemap(OpenRCT2::Drawing::FilterPaletteID::paletteGhost);
constexpr ImageId TrackStationColour = ImageId(0, OpenRCT2::Drawing::Colour::black);
constexpr ImageId ShopSupportColour = ImageId(0, OpenRCT2::Drawing::Colour::darkBrown);

extern bool gShowDirtyVisuals;
extern bool gPaintBoundingBoxes;
extern bool gPaintBlockedTiles;
extern bool gPaintWidePathsAsGhost;
extern bool gPaintStableSort;

PaintStruct* PaintAddImageAsParent(
    PaintSession& session, ImageId image_id, const CoordsXYZ& offset, const BoundBoxXYZ& boundBox);
/**
 *  rct2: 0x006861AC, 0x00686337, 0x006864D0, 0x0068666B, 0x0098196C
 *
 * @param image_id (ebx)
 * @param x_offset (al)
 * @param y_offset (cl)
 * @param bound_box_length_x (di)
 * @param bound_box_length_y (si)
 * @param bound_box_length_z (ah)
 * @param z_offset (dx)
 * @return (ebp) PaintStruct on success (CF == 0), nullptr on failure (CF == 1)
 */
inline PaintStruct* PaintAddImageAsParent(
    PaintSession& session, ImageId image_id, const CoordsXYZ& offset, const CoordsXYZ& boundBoxSize)
{
    return PaintAddImageAsParent(session, image_id, offset, { offset, boundBoxSize });
}

[[nodiscard]] PaintStruct* PaintAddImageAsOrphan(
    PaintSession& session, ImageId imageId, const CoordsXYZ& offset, const BoundBoxXYZ& boundBox);
PaintStruct* PaintAddImageAsChild(
    PaintSession& session, ImageId image_id, const CoordsXYZ& offset, const BoundBoxXYZ& boundBox);

PaintStruct* PaintAddImageAsChildRotated(
    PaintSession& session, uint8_t direction, ImageId image_id, const CoordsXYZ& offset, const BoundBoxXYZ& boundBox);

PaintStruct* PaintAddImageAsParentRotated(
    PaintSession& session, uint8_t direction, ImageId imageId, const CoordsXYZ& offset, const BoundBoxXYZ& boundBox);

inline PaintStruct* PaintAddImageAsParentRotated(
    PaintSession& session, const uint8_t direction, const ImageId imageId, const CoordsXYZ& offset,
    const CoordsXYZ& boundBoxSize)
{
    return PaintAddImageAsParentRotated(session, direction, imageId, offset, { offset, boundBoxSize });
}

PaintStruct* PaintAddImageAsParentHeight(
    PaintSession& session, ImageId imageId, int32_t height, const CoordsXYZ& offset, const BoundBoxXYZ& boundBox);

bool PaintAttachToPreviousAttach(PaintSession& session, ImageId imageId, int32_t x, int32_t y);
bool PaintAttachToPreviousPS(PaintSession& session, ImageId image_id, int32_t x, int32_t y);
void PaintFloatingMoneyEffect(
    PaintSession& session, money64 amount, StringId string_id, int32_t y, int32_t z, int8_t y_offsets[], int32_t offset_x,
    uint32_t rotation);

PaintSession* PaintSessionAlloc(OpenRCT2::Drawing::RenderTarget& rt, uint32_t viewFlags, uint8_t rotation);
void PaintSessionFree(PaintSession* session);
void PaintSessionGenerate(PaintSession& session);
void PaintSessionArrange(PaintSessionCore& session);
void PaintDrawStructs(PaintSession& session);
void PaintDrawMoneyStructs(OpenRCT2::Drawing::RenderTarget& rt, PaintStringStruct* ps);
