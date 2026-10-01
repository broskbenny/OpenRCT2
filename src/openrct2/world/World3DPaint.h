/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "../drawing/ImageId.hpp"
#include "../paint/Boundbox.h"
#include "Location.hpp"
#include "tile_element/TileElementType.h"

#include <cstdint>
#include <vector>

struct PaintSession;
struct PaintStruct;

namespace OpenRCT2::World3D
{
    /**
     * Evidence emitted by the existing isometric paint code.
     *
     * A declared paint bounding box is useful for locating a component, but is
     * intentionally not promoted to mesh geometry here: sorting bounds describe
     * occupied space for the painter and need not match the object's literal shape.
     */
    struct PaintEvidence
    {
        ImageId imageId{};
        CoordsXY mapPosition{};
        CoordsXY spritePosition{};
        CoordsXYZ imageOffset{};
        BoundBoxXYZ declaredBounds{};
        int32_t heightOffset{};
        ScreenCoordsXY screenPosition{};
        CoordsXYZ paintBoundsMin{};
        CoordsXYZ paintBoundsMax{};
        uint8_t rotation{};

        bool hasTileElement{};
        TileElementType elementType{};
        int32_t elementBaseZ{};
        int32_t elementClearanceZ{};
        uint8_t elementDirection{};
        uint8_t occupiedQuadrants{};
    };

    /**
     * RAII capture scope. Paint calls only append evidence while a scope is active,
     * so normal viewport rendering pays only a null-pointer branch.
     */
    class PaintEvidenceCapture
    {
    public:
        explicit PaintEvidenceCapture(std::vector<PaintEvidence>& output) noexcept;
        ~PaintEvidenceCapture();

        PaintEvidenceCapture(const PaintEvidenceCapture&) = delete;
        PaintEvidenceCapture& operator=(const PaintEvidenceCapture&) = delete;

    private:
        std::vector<PaintEvidence>* _previous{};
    };

    void RecordPaintEvidence(
        const PaintSession& session, ImageId imageId, const CoordsXYZ& imageOffset, const BoundBoxXYZ& bounds,
        const PaintStruct& paintStruct, int32_t heightOffset = 0) noexcept;
} // namespace OpenRCT2::World3D
