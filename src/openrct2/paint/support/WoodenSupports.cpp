/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "WoodenSupports.h"

#include "../../SpriteIds.h"
#include "../../interface/Viewport.h"
#include "../../ride/TrackData.h"
#include "../../ride/ted/TrackElementDescriptor.h"
#include "../../world/Footpath.h"
#include "../../world/tile_element/Slope.h"
#include "../Boundbox.h"
#include "../Paint.SessionFlags.h"
#include "../Paint.h"

#include <cassert>

using namespace OpenRCT2;
using namespace OpenRCT2::Drawing;
using namespace OpenRCT2::Numerics;

constexpr auto kNumWoodenSupportTypes = 2;
constexpr auto kNumWoodenSupportSubTypes = 6;

struct SupportsIdDescriptor
{
    ImageIndex Full;
    ImageIndex Half;
    ImageIndex Flat;
    ImageIndex Slope;
};

/* 0x0097B1C4 */
static constexpr SupportsIdDescriptor WoodenSupportImageIds[kNumWoodenSupportTypes][kNumWoodenSupportSubTypes] = {
    // WoodenSupportType::truss
    {
        { 3392, 3393, 3394, 3536 }, // WoodenSupportSubType::neSw
        { 3390, 3391, 3394, 3514 }, // WoodenSupportSubType::nwSe
        { 3558, 3559, 3560, 3570 }, // WoodenSupportSubType::corner0
        { 3561, 3562, 3563, 3592 }, // WoodenSupportSubType::corner1
        { 3564, 3565, 3566, 3614 }, // WoodenSupportSubType::corner2
        { 3567, 3568, 3569, 3636 }, // WoodenSupportSubType::corner3
    },
    // WoodenSupportType::mine
    {
        { 3677, 3678, 3680, 3739 }, // WoodenSupportSubType::neSw
        { 3675, 3676, 3679, 3717 }, // WoodenSupportSubType::nwSe
        { 3761, 3762, 3763, 3773 }, // WoodenSupportSubType::corner0
        { 3764, 3765, 3766, 3795 }, // WoodenSupportSubType::corner1
        { 3767, 3768, 3769, 3817 }, // WoodenSupportSubType::corner2
        { 3770, 3771, 3772, 3839 }, // WoodenSupportSubType::corner3
    },
};

constexpr SupportsIdDescriptor GetWoodenSupportIds(WoodenSupportType supportType, WoodenSupportSubType subType)
{
    return WoodenSupportImageIds[EnumValue(supportType)][EnumValue(subType)];
}

using ImagesByTransitionTypeArray = std::array<
    std::array<ImageIndex, kNumOrthogonalDirections>, kWoodenSupportTransitionTypeCount>;

static constexpr ImagesByTransitionTypeArray WoodenCurveSupportImageIds0 = { {
    { 3465, 3466, 3467, 3468 }, // flatToUp25Deg
    { 3469, 3470, 3471, 3472 }, // up25DegToFlat
    { 3473, 3474, 3475, 3476 }, // up25Deg
    { 3477, 3478, 3479, 3480 }, // up25DegToUp60Deg
    { 3481, 3482, 3483, 3484 }, // up60DegToUp25Deg
    { 3485, 3486, 3487, 3488 }, // up60Deg
    { SPR_TRACKS_SUPPORT_WOODEN_TRUSS_UP_25_EVEN, SPR_TRACKS_SUPPORT_WOODEN_TRUSS_UP_25_EVEN + 1,
      SPR_TRACKS_SUPPORT_WOODEN_TRUSS_UP_25_EVEN + 2, SPR_TRACKS_SUPPORT_WOODEN_TRUSS_UP_25_EVEN + 3 }, // up25even
    { 3493, 3494, 3495, 3496 },                                                                         // flatToUp60Deg
    { 3497, 3498, 3499, 3500 },                                                                         // up60DegToFlat
    { 3501, 3502, 3503, 3504 },                                                                         // flatToUp25DegRailway
    { 3505, 3506, 3507, 3508 },                                                                         // up25DegToFlatRailway
    { 3509, 3510, 3511, 3512 },                                                                         // up25DegRailway
    { 3513, 3513, 3513, 3513 },                                                                         // scenery
    { SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_FLAT_TO_STEEP, SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_FLAT_TO_STEEP + 1,
      SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_FLAT_TO_STEEP + 2,
      SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_FLAT_TO_STEEP + 3 }, // flatToUp60DegLongBaseSeq0
    { SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_FLAT_TO_STEEP + 4, SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_FLAT_TO_STEEP + 5,
      SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_FLAT_TO_STEEP + 6,
      SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_FLAT_TO_STEEP + 7 }, // flatToUp60DegLongBaseSeq1
    { SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_FLAT_TO_STEEP + 8, SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_FLAT_TO_STEEP + 9,
      SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_FLAT_TO_STEEP + 10,
      SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_FLAT_TO_STEEP + 11 }, // flatToUp60DegLongBaseSeq2
    { SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_FLAT_TO_STEEP + 12, SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_FLAT_TO_STEEP + 13,
      SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_FLAT_TO_STEEP + 14,
      SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_FLAT_TO_STEEP + 15 }, // flatToUp60DegLongBaseSeq3
    { SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_STEEP_TO_FLAT, SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_STEEP_TO_FLAT + 1,
      SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_STEEP_TO_FLAT + 2,
      SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_STEEP_TO_FLAT + 3 }, // up60DegToFlatLongBaseSeq0
    { SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_STEEP_TO_FLAT + 4, SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_STEEP_TO_FLAT + 5,
      SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_STEEP_TO_FLAT + 6,
      SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_STEEP_TO_FLAT + 7 }, // up60DegToFlatLongBaseSeq1
    { SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_STEEP_TO_FLAT + 8, SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_STEEP_TO_FLAT + 9,
      SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_STEEP_TO_FLAT + 10,
      SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_STEEP_TO_FLAT + 11 }, // up60DegToFlatLongBaseSeq2
    { SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_STEEP_TO_FLAT + 12, SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_STEEP_TO_FLAT + 13,
      SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_STEEP_TO_FLAT + 14,
      SPR_TRACKS_SUPPORT_WOODEN_TRUSS_LONG_STEEP_TO_FLAT + 15 }, // up60DegToFlatLongBaseSeq3
} };

static constexpr ImagesByTransitionTypeArray WoodenCurveSupportImageIds1 = { {
    { 3681, 3682, 3683, 3684 }, // flatToUp25Deg
    { 3685, 3686, 3687, 3688 }, // up25DegToFlat
    { 3689, 3690, 3691, 3692 }, // up25Deg
    { 3693, 3694, 3695, 3696 }, // up25DegToUp60Deg
    { 3697, 3698, 3699, 3700 }, // up60DegToUp25Deg
    { 3701, 3702, 3703, 3704 }, // up60Deg
    { SPR_TRACKS_SUPPORT_WOODEN_MINE_UP_25_EVEN, SPR_TRACKS_SUPPORT_WOODEN_MINE_UP_25_EVEN + 1,
      SPR_TRACKS_SUPPORT_WOODEN_MINE_UP_25_EVEN + 2, SPR_TRACKS_SUPPORT_WOODEN_MINE_UP_25_EVEN + 3 }, // up25even
    { 3709, 3710, 3711, 3712 },                                                                       // flatToUp60Deg
    { 3713, 3714, 3715, 3716 },                                                                       // up60DegToFlat
    { 3717, 3718, 3719, 3720 },                                                                       // flatToUp25DegRailway
    { 3721, 3722, 3723, 3724 },                                                                       // up25DegToFlatRailway
    { 3725, 3726, 3727, 3728 },                                                                       // up25DegRailway
    { 3729, 3729, 3729, 3729 },                                                                       // scenery
    { SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_FLAT_TO_STEEP, SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_FLAT_TO_STEEP + 1,
      SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_FLAT_TO_STEEP + 2,
      SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_FLAT_TO_STEEP + 3 }, // flatToUp60DegLongBaseSeq0
    { SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_FLAT_TO_STEEP + 4, SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_FLAT_TO_STEEP + 5,
      SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_FLAT_TO_STEEP + 6,
      SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_FLAT_TO_STEEP + 7 }, // flatToUp60DegLongBaseSeq1
    { SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_FLAT_TO_STEEP + 8, SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_FLAT_TO_STEEP + 9,
      SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_FLAT_TO_STEEP + 10,
      SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_FLAT_TO_STEEP + 11 }, // flatToUp60DegLongBaseSeq2
    { SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_FLAT_TO_STEEP + 12, SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_FLAT_TO_STEEP + 13,
      SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_FLAT_TO_STEEP + 14,
      SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_FLAT_TO_STEEP + 15 }, // flatToUp60DegLongBaseSeq3
    { SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_STEEP_TO_FLAT, SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_STEEP_TO_FLAT + 1,
      SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_STEEP_TO_FLAT + 2,
      SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_STEEP_TO_FLAT + 3 }, // up60DegToFlatLongBaseSeq0
    { SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_STEEP_TO_FLAT + 4, SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_STEEP_TO_FLAT + 5,
      SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_STEEP_TO_FLAT + 6,
      SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_STEEP_TO_FLAT + 7 }, // up60DegToFlatLongBaseSeq1
    { SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_STEEP_TO_FLAT + 8, SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_STEEP_TO_FLAT + 9,
      SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_STEEP_TO_FLAT + 10,
      SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_STEEP_TO_FLAT + 11 }, // up60DegToFlatLongBaseSeq2
    { SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_STEEP_TO_FLAT + 12, SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_STEEP_TO_FLAT + 13,
      SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_STEEP_TO_FLAT + 14,
      SPR_TRACKS_SUPPORT_WOODEN_MINE_LONG_STEEP_TO_FLAT + 15 }, // up60DegToFlatLongBaseSeq3
} };

// clang-format off
/* 0x0097B224 */
static constexpr const ImagesByTransitionTypeArray::const_pointer WoodenCurveSupportImageIds[kNumWoodenSupportTypes][kNumWoodenSupportSubTypes] = {
    // WoodenSupportType::truss
    { 
        WoodenCurveSupportImageIds0.data(), // WoodenSupportSubType::neSw
        WoodenCurveSupportImageIds0.data(), // WoodenSupportSubType::nwSe
        nullptr,                            // WoodenSupportSubType::corner0
        nullptr,                            // WoodenSupportSubType::corner1
        nullptr,                            // WoodenSupportSubType::corner2
        nullptr,                            // WoodenSupportSubType::corner3
    }, 
    // WoodenSupportType::mine
    { 
        WoodenCurveSupportImageIds1.data(), // WoodenSupportSubType::neSw
        WoodenCurveSupportImageIds1.data(), // WoodenSupportSubType::nwSe
        nullptr,                            // WoodenSupportSubType::corner0
        nullptr,                            // WoodenSupportSubType::corner1
        nullptr,                            // WoodenSupportSubType::corner2
        nullptr,                            // WoodenSupportSubType::corner3
    }, 
};

struct SlopedSupportsDescriptor {
    BoundBoxXYZ BoundingBox;
    bool AsOrphan;
};

/* 0x0097B23C */
static constexpr auto kSupportsDescriptors = std::to_array<SlopedSupportsDescriptor>({
    {{{0,  0,  0}, {1,  1,  8}},  false}, // Flat to gentle
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false}, // Gentle to flat
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false}, // Gentle slope
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false}, // Gentle to steep
    {{{10, 0,  0}, {10, 32, 44}}, true},
    {{{0,  10, 0}, {32, 10, 44}}, true},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false}, // Steep to gentle
    {{{10, 0,  0}, {10, 32, 44}}, true},
    {{{0,  10, 0}, {32, 10, 44}}, true},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false}, // Steep slope
    {{{10, 0,  2}, {10, 32, 76}}, true},
    {{{0,  10, 2}, {32, 10, 76}}, true},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  4}},  false}, // Slope
    {{{0,  0,  0}, {1,  1,  4}},  false},
    {{{0,  0,  0}, {1,  1,  4}},  false},
    {{{0,  0,  0}, {1,  1,  4}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false}, // Flat to steep small
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false}, // Steep to flat small
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false}, // ?
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false}, // ?
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false}, // ?
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{2,  2,  1}, {28, 28, 2}},  false}, // Large scenery
    {{{2,  2,  1}, {28, 28, 2}},  false},
    {{{2,  2,  1}, {28, 28, 2}},  false},
    {{{2,  2,  1}, {28, 28, 2}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false}, // Flat to steep large 1
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false}, // Flat to steep large 2
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false}, // Flat to steep large 3
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false}, // Flat to steep large 4
    {{{10, 0,  2}, {10, 32, 52}}, true},
    {{{0,  10, 2}, {32, 10, 52}}, true},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false}, // Steep to flat large 1
    {{{10, 0,  2}, {10, 32, 76}}, true},
    {{{0,  10, 2}, {32, 10, 76}}, true},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false}, // Steep to flat large 2
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false}, // Steep to flat large 3
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false}, // Steep to flat large 4
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
    {{{0,  0,  0}, {1,  1,  8}},  false},
});
static_assert(std::size(kSupportsDescriptors) == kWoodenSupportTransitionTypeCount * kNumOrthogonalDirections);

/* 0x0098D8D4 */
static constexpr SlopedSupportsDescriptor kSlopedPathSupportsDescriptor = {{{0, 0, 0}, {1, 1, 4}}, false};

/* 0x0097B3C4 */
static constexpr uint16_t word_97B3C4[] = {
    0,
    0,
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    10,
    11,
    12,
    13,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    14,
    0,
    0,
    0,
    17,
    0,
    16,
    15,
    0,
};
// clang-format on

static WoodenSupportSubType rotatedWoodenSupportSubTypes[kNumWoodenSupportSubTypes][kNumOrthogonalDirections] = {
    {
        WoodenSupportSubType::neSw,
        WoodenSupportSubType::nwSe,
        WoodenSupportSubType::neSw,
        WoodenSupportSubType::nwSe,
    },
    {
        WoodenSupportSubType::nwSe,
        WoodenSupportSubType::neSw,
        WoodenSupportSubType::nwSe,
        WoodenSupportSubType::neSw,
    },
    {
        WoodenSupportSubType::corner0,
        WoodenSupportSubType::corner1,
        WoodenSupportSubType::corner2,
        WoodenSupportSubType::corner3,
    },
    {
        WoodenSupportSubType::corner1,
        WoodenSupportSubType::corner2,
        WoodenSupportSubType::corner3,
        WoodenSupportSubType::corner0,
    },
    {
        WoodenSupportSubType::corner2,
        WoodenSupportSubType::corner3,
        WoodenSupportSubType::corner0,
        WoodenSupportSubType::corner1,
    },
    {
        WoodenSupportSubType::corner3,
        WoodenSupportSubType::corner0,
        WoodenSupportSubType::corner1,
        WoodenSupportSubType::corner2,
    },
};

static std::array<CoordsXY, 2>
    FirstPersonWoodenSupportPosts(
        WoodenSupportSubType subType)
{
    switch (subType)
    {
        case WoodenSupportSubType::neSw:
            return { CoordsXY{ 4, 28 },
                     CoordsXY{ 28, 4 } };
        case WoodenSupportSubType::nwSe:
            return { CoordsXY{ 4, 4 },
                     CoordsXY{ 28, 28 } };
        case WoodenSupportSubType::corner0:
            return { CoordsXY{ 4, 4 },
                     CoordsXY{ 16, 16 } };
        case WoodenSupportSubType::corner1:
            return { CoordsXY{ 28, 4 },
                     CoordsXY{ 16, 16 } };
        case WoodenSupportSubType::corner2:
            return { CoordsXY{ 28, 28 },
                     CoordsXY{ 16, 16 } };
        case WoodenSupportSubType::corner3:
            return { CoordsXY{ 4, 28 },
                     CoordsXY{ 16, 16 } };
        default:
            return { CoordsXY{}, CoordsXY{} };
    }
}

static void PublishFirstPersonWoodenSupportGeometry(
    PaintSession& session, WoodenSupportSubType subType,
    int32_t lowZ, int32_t highZ, ImageId image)
{
    if (highZ <= lowZ
        || subType == WoodenSupportSubType::null)
        return;
    const auto posts =
        FirstPersonWoodenSupportPosts(subType);
    for (const auto point : posts)
    {
        PaintSessionAddFirstPersonSemanticBox(
            session, FirstPersonPaintSemanticRole::support,
            { point.x - 2, point.y - 2, lowZ },
            { point.x + 2, point.y + 2, highZ },
            image);
    }

    for (int32_t sectionLow = lowZ;
         sectionLow < highZ;
         sectionLow += 16)
    {
        const int32_t sectionHigh =
            std::min(sectionLow + 16, highZ);
        PaintSessionAddFirstPersonSemanticBeam(
            session, FirstPersonPaintSemanticRole::support,
            { posts[0].x, posts[0].y, sectionHigh },
            { posts[1].x, posts[1].y, sectionHigh },
            1, image);
        PaintSessionAddFirstPersonSemanticBeam(
            session, FirstPersonPaintSemanticRole::support,
            { posts[0].x, posts[0].y, sectionLow },
            { posts[1].x, posts[1].y, sectionHigh },
            1, image);
        PaintSessionAddFirstPersonSemanticBeam(
            session, FirstPersonPaintSemanticRole::support,
            { posts[1].x, posts[1].y, sectionLow },
            { posts[0].x, posts[0].y, sectionHigh },
            1, image);
    }
}

/**
 * Draw repeated supports for left over space
 */
static void PaintRepeatedWoodenSupports(
    const SupportsIdDescriptor supportImages, const ImageId& imageTemplate, int16_t heightSteps, PaintSession& session,
    uint16_t& baseHeight, bool& hasSupports)
{
    // All call sites are inside the shared wooden-support semantic scope.
    while (heightSteps > 0)
    {
        const bool isHalf = baseHeight & 0x10 || heightSteps == 1 || baseHeight + kWaterHeightStep == session.WaterHeight;
        if (isHalf)
        {
            // Half support
            auto imageId = imageTemplate.WithIndex(supportImages.Half);
            uint8_t boundBoxHeight = (heightSteps == 1) ? 7 : 12;
            PaintAddImageAsParent(session, imageId, { 0, 0, baseHeight }, { 32, 32, boundBoxHeight });
            baseHeight += 16;
            heightSteps -= 1;
        }
        else
        {
            // Full support
            auto imageId = imageTemplate.WithIndex(supportImages.Full);
            uint8_t boundBoxHeight = (heightSteps == 2) ? 23 : 28;
            PaintAddImageAsParent(session, imageId, { 0, 0, baseHeight }, { 32, 32, boundBoxHeight });
            baseHeight += 32;
            heightSteps -= 2;
        }

        hasSupports = true;
    }
}

/**
 * Draw special pieces, e.g. curved supports.
 */
static void PaintSlopeTransitions(
    const SlopedSupportsDescriptor& supportsDesc, ImageIndex imageIndex, PaintSession& session, const ImageId& imageTemplate,
    uint16_t baseHeight)
{
    // Own the transition where its native paint struct is created. This also
    // covers orphaned transition images attached to a track paint chain and
    // future callers that do not already establish a support semantic scope.
    FirstPersonPaintSemanticScope firstPersonRole(
        session, FirstPersonPaintSemanticRole::support);
    auto imageId = imageTemplate.WithIndex(imageIndex);

    auto boundBox = supportsDesc.BoundingBox;
    boundBox.offset.z += baseHeight;

    if (supportsDesc.AsOrphan == false || session.WoodenSupportsPrependTo == nullptr)
    {
        PaintAddImageAsParent(session, imageId, { 0, 0, baseHeight }, boundBox);
    }
    else
    {
        auto* paintStruct = PaintAddImageAsOrphan(session, imageId, { 0, 0, baseHeight }, boundBox);
        if (paintStruct != nullptr)
        {
            session.WoodenSupportsPrependTo->Children = paintStruct;
        }
    }
}

static bool WoodenABPaintSlopeTransitions(
    PaintSession& session, WoodenSupportType supportType, WoodenSupportSubType subType,
    WoodenSupportTransitionType transitionType, Direction direction, const ImageId& imageTemplate, uint16_t baseHeight)
{
    const uint16_t supportsDescriptorIndex = (EnumValue(transitionType) * kNumOrthogonalDirections) + direction;
    const SlopedSupportsDescriptor& supportsDesc = kSupportsDescriptors[supportsDescriptorIndex];
    const auto* imageIds = WoodenCurveSupportImageIds[EnumValue(supportType)][EnumValue(subType)];

    if (imageIds == nullptr || imageIds[EnumValue(transitionType)][direction] == 0)
        return false;

    PaintSlopeTransitions(supportsDesc, imageIds[EnumValue(transitionType)][direction], session, imageTemplate, baseHeight);

    return true;
}

template<uint8_t zOffset, bool doHeightStepsCheck>
static inline bool WoodenSupportsPaintSetupCommon(
    PaintSession& session, const SupportsIdDescriptor& supportImages, int32_t height, ImageId& imageTemplate, bool& hasSupports,
    uint16_t& baseHeight)
{
    // These native images are timber-support artwork, not freestanding objects.
    // The fixed post/brace geometry is published by the owning support painter.
    // Give ground bases and slope-adapting pieces the same physical ownership
    // as the repeated support sprites, rather than allowing view-facing fallback.
    FirstPersonPaintSemanticScope firstPersonRole(
        session, FirstPersonPaintSemanticRole::support);
    if (!(session.Flags & PaintSessionFlags::PassedSurface))
    {
        return false;
    }

    if (session.ViewFlags & VIEWPORT_FLAG_HIDE_SUPPORTS)
    {
        if (session.ViewFlags & VIEWPORT_FLAG_INVISIBLE_SUPPORTS)
        {
            return false;
        }
        imageTemplate = ImageId().WithTransparency(FilterPaletteID::paletteDarken1);
    }

    baseHeight = ceil2(session.Support.height, 16);
    int16_t supportLength = height - baseHeight;

    if (supportLength < 0)
    {
        return false;
    }

    int16_t heightSteps = supportLength / 16;

    hasSupports = false;
    bool drawFlatPiece = false;

    // Draw base support (usually shaped to the slope)
    auto slope = session.Support.slope;
    if (slope & kTileSlopeAboveTrackOrScenery)
    {
        // Above scenery (just put a base piece above it)
        drawFlatPiece = true;
    }
    else if (slope & kTileSlopeDiagonalFlag)
    {
        // Steep diagonal (place the correct shaped support for the slope)
        heightSteps -= 2;
        if (heightSteps < 0)
        {
            return false;
        }

        ImageIndex imageIndex = supportImages.Slope;
        if (imageIndex == 0)
        {
            drawFlatPiece = true;
        }
        else
        {
            auto imageId = imageTemplate.WithIndex(imageIndex + word_97B3C4[slope & kTileSlopeMask]);

            PaintAddImageAsParent(session, imageId, { 0, 0, baseHeight }, { { 0, 0, baseHeight + 2 }, { 32, 32, 11 } });
            PaintAddImageAsParent(
                session, imageId.WithIndexOffset(4), { 0, 0, baseHeight + 16 },
                { { 0, 0, baseHeight + 16 + 2 }, { 32, 32, zOffset } });

            hasSupports = true;
        }

        baseHeight += 32;
    }
    else if ((slope & kTileSlopeRaisedCornersMask) != 0)
    {
        // 1 to 3 quarters up
        heightSteps--;
        if (heightSteps < 0)
        {
            return false;
        }

        ImageIndex imageIndex = supportImages.Slope;
        if (imageIndex == 0)
        {
            drawFlatPiece = true;
        }
        else
        {
            auto imageId = imageTemplate.WithIndex(imageIndex + word_97B3C4[slope & kTileSlopeMask]);
            PaintAddImageAsParent(session, imageId, { 0, 0, baseHeight }, { { 0, 0, baseHeight + 2 }, { 32, 32, zOffset } });
            hasSupports = true;
        }

        baseHeight += 16;
    }

    // Draw flat base support
    if (drawFlatPiece)
    {
        bool shouldDraw = true;
        if constexpr (doHeightStepsCheck)
        {
            shouldDraw = heightSteps > 0;
        }

        if (shouldDraw)
        {
            auto imageId = imageTemplate.WithIndex(supportImages.Flat);
            PaintAddImageAsParent(session, imageId, { 0, 0, baseHeight - 2 }, { 32, 32, 0 });
            hasSupports = true;
        }
    }

    PaintRepeatedWoodenSupports(supportImages, imageTemplate, heightSteps, session, baseHeight, hasSupports);
    return true;
}

template<uint8_t zOffset, bool doHeightStepsCheck>
inline bool WoodenABSupportsPaintSetupCommon(
    PaintSession& session, WoodenSupportType supportType, WoodenSupportSubType subType, int32_t height, ImageId imageTemplate,
    WoodenSupportTransitionType transitionType, Direction direction)
{
    assert(subType != WoodenSupportSubType::null);

    uint16_t baseHeight = 0;
    bool hasSupports = false;
    auto supportIds = GetWoodenSupportIds(supportType, subType);
    const int32_t firstPersonBase =
        ceil2(session.Support.height, 16);

    if (!WoodenSupportsPaintSetupCommon<zOffset, doHeightStepsCheck>(
            session, supportIds, height, imageTemplate, hasSupports, baseHeight))
    {
        return false;
    }

    if (hasSupports)
    {
        PublishFirstPersonWoodenSupportGeometry(
            session, subType,
            firstPersonBase, height,
            imageTemplate.WithIndex(supportIds.Full));
    }

    if (transitionType != WoodenSupportTransitionType::none)
    {
        // Native paint may report a zero-height flat base as drawn. That
        // alone does not produce a non-degenerate semantic post/brace body.
        const bool hadStructuralBody =
            hasSupports && height > firstPersonBase;
        const bool hasTransition = WoodenABPaintSlopeTransitions(
            session, supportType, subType, transitionType, direction, imageTemplate, baseHeight);
        if (hasTransition && !hadStructuralBody)
        {
            // A track can paint its upper wooden transition even when there
            // is no vertical support length at this tile. The usual post/brace
            // publisher therefore has nothing to emit. Retain a small,
            // non-colliding physical tie at the *known* post anchors rather
            // than silently losing the entire native transition or inventing
            // its shape from the native sorting bounding box.
            const auto posts = FirstPersonWoodenSupportPosts(subType);
            const auto* images = WoodenCurveSupportImageIds[
                EnumValue(supportType)][EnumValue(subType)];
            const ImageId transitionImage = imageTemplate.WithIndex(
                images[EnumValue(transitionType)][direction]);
            PaintSessionAddFirstPersonSemanticBeam(
                session, FirstPersonPaintSemanticRole::support,
                { posts[0].x, posts[0].y, height },
                { posts[1].x, posts[1].y, height },
                1, transitionImage, {}, 0, false);
        }
        // Preserve the native success/return contract.
        hasSupports = hasTransition;
    }

    return hasSupports;
}

/**
 * Adds paint structs for wooden supports.
 *  rct2: 0x006629BC
 * @param supportType (edi) Type and direction of supports.
 * @param special (ax) Used for curved supports.
 * @param height (dx) The height of the supports.
 * @param imageTemplate (ebp) The colour and palette flags for the support sprites.
 * @param[out] underground (Carry flag) true if underground.
 * @returns (al) true if any supports have been drawn, otherwise false.
 */
bool WoodenASupportsPaintSetup(
    PaintSession& session, WoodenSupportType supportType, WoodenSupportSubType subType, int32_t height, ImageId imageTemplate,
    WoodenSupportTransitionType transitionType, Direction direction)
{
    return WoodenABSupportsPaintSetupCommon<11, false>(
        session, supportType, subType, height, imageTemplate, transitionType, direction);
}

bool WoodenASupportsPaintSetupRotated(
    PaintSession& session, WoodenSupportType supportType, WoodenSupportSubType subType, Direction direction, int32_t height,
    ImageId imageTemplate, WoodenSupportTransitionType transitionType)
{
    assert(subType != WoodenSupportSubType::null);
    subType = rotatedWoodenSupportSubTypes[EnumValue(subType)][direction];
    return WoodenASupportsPaintSetup(session, supportType, subType, height, imageTemplate, transitionType, direction);
}

/**
 * Wooden supports
 *  rct2: 0x00662D5C
 *
 * @param supportType (edi)
 * @param special (ax)
 * @param height (dx)
 * @param imageTemplate (ebp)
 * @param[out] underground (Carry Flag)
 *
 * @return (al) whether supports have been drawn
 */
bool WoodenBSupportsPaintSetup(
    PaintSession& session, WoodenSupportType supportType, WoodenSupportSubType subType, int32_t height, ImageId imageTemplate,
    WoodenSupportTransitionType transitionType, Direction direction)
{
    return WoodenABSupportsPaintSetupCommon<3, true>(
        session, supportType, subType, height, imageTemplate, transitionType, direction);
}

bool WoodenBSupportsPaintSetupRotated(
    PaintSession& session, WoodenSupportType supportType, WoodenSupportSubType subType, Direction direction, int32_t height,
    ImageId imageTemplate, WoodenSupportTransitionType transitionType)
{
    assert(subType != WoodenSupportSubType::null);
    subType = rotatedWoodenSupportSubTypes[EnumValue(subType)][direction];
    return WoodenBSupportsPaintSetup(session, supportType, subType, height, imageTemplate, transitionType, direction);
}

/**
 *  rct2: 0x006A2ECC
 *
 * @param supportType (edi)
 * @param special (ax)
 * @param height (dx)
 * @param imageTemplate (ebp)
 * @param railingsDescriptor (0x00F3EF6C)
 * @param[out] underground (Carry Flag)
 *
 * @return Whether supports were drawn
 */
bool PathBoxSupportsPaintSetup(
    PaintSession& session, WoodenSupportSubType supportType, bool isSloped, Direction slopeRotation, int32_t height,
    ImageId imageTemplate, const PathRailingsDescriptor& railings)
{
    // Every native sprite emitted by this path-specific support painter is
    // presentation for fixed support structure that first person publishes
    // semantically below. Keep all of it out of the generic billboard path,
    // including terrain bases and slope transitions.
    FirstPersonPaintSemanticScope firstPersonRole(
        session, FirstPersonPaintSemanticRole::support);

    auto supportOrientationOffset = (supportType == WoodenSupportSubType::nwSe) ? 24 : 0;

    uint16_t baseHeight = 0;
    bool hasSupports = false;
    const int32_t firstPersonBase =
        ceil2(session.Support.height, 16);
    SupportsIdDescriptor supportIds = {
        .Full = railings.bridgeImage + 22 + supportOrientationOffset,
        .Half = railings.bridgeImage + 23 + supportOrientationOffset,
        .Flat = railings.bridgeImage + 48,
        .Slope = railings.bridgeImage + supportOrientationOffset,
    };

    if (!WoodenSupportsPaintSetupCommon<11, false>(session, supportIds, height, imageTemplate, hasSupports, baseHeight))
    {
        return false;
    }

    if (hasSupports)
    {
        PublishFirstPersonWoodenSupportGeometry(
            session, supportType,
            firstPersonBase, height,
            imageTemplate.WithIndex(supportIds.Full));
    }

    if (isSloped)
    {
        ImageIndex imageIndex = railings.bridgeImage + 55 + slopeRotation;

        PaintSlopeTransitions(
            kSlopedPathSupportsDescriptor, imageIndex,
            session, imageTemplate, baseHeight);
        hasSupports = true;
    }

    return hasSupports;
}

bool DrawSupportForSequenceA(
    PaintSession& session, const WoodenSupportType supportType, const TrackElemType trackType, const uint8_t sequence,
    const Direction direction, const int32_t height, const ImageId imageTemplate)
{
    const auto& ted = TrackMetadata::GetTrackElementDescriptor(trackType);
    const auto& sequenceDesc = ted.sequenceData.sequences[sequence];
    const auto& desc = sequenceDesc.woodenSupports;

    if (desc.subType == WoodenSupportSubType::null)
        return false;

    const Direction supportRotation = (direction + sequenceDesc.extraSupportRotation) & 3;

    return WoodenASupportsPaintSetupRotated(
        session, supportType, desc.subType, supportRotation, height, imageTemplate, desc.transitionType);
}

bool DrawSupportForSequenceB(
    PaintSession& session, const WoodenSupportType supportType, const TrackElemType trackType, const uint8_t sequence,
    const Direction direction, const int32_t height, const ImageId imageTemplate)
{
    const auto& ted = TrackMetadata::GetTrackElementDescriptor(trackType);
    const auto& sequenceDesc = ted.sequenceData.sequences[sequence];
    const auto& desc = sequenceDesc.woodenSupports;

    if (desc.subType == WoodenSupportSubType::null)
        return false;

    const Direction supportRotation = (direction + sequenceDesc.extraSupportRotation) & 3;

    return WoodenBSupportsPaintSetupRotated(
        session, supportType, desc.subType, supportRotation, height, imageTemplate, desc.transitionType);
}
