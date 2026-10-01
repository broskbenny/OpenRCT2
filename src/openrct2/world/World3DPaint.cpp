/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "World3DPaint.h"

#include "../paint/Paint.h"
#include "tile_element/TileElement.h"

namespace OpenRCT2::World3D
{
    namespace
    {
        thread_local std::vector<PaintEvidence>* _activePaintEvidence = nullptr;
    }

    PaintEvidenceCapture::PaintEvidenceCapture(std::vector<PaintEvidence>& output) noexcept
        : _previous(_activePaintEvidence)
    {
        _activePaintEvidence = &output;
    }

    PaintEvidenceCapture::~PaintEvidenceCapture()
    {
        _activePaintEvidence = _previous;
    }

    void RecordPaintEvidence(
        const PaintSession& session, ImageId imageId, const CoordsXYZ& imageOffset, const BoundBoxXYZ& bounds,
        const PaintStruct& paintStruct, int32_t heightOffset) noexcept
    {
        if (_activePaintEvidence == nullptr)
        {
            return;
        }

        PaintEvidence evidence{};
        evidence.imageId = imageId;
        evidence.mapPosition = session.MapPosition;
        evidence.spritePosition = session.SpritePosition;
        evidence.imageOffset = imageOffset;
        evidence.declaredBounds = bounds;
        evidence.heightOffset = heightOffset;
        evidence.screenPosition = paintStruct.ScreenPos;
        evidence.paintBoundsMin = { paintStruct.Bounds.x, paintStruct.Bounds.y, paintStruct.Bounds.z };
        evidence.paintBoundsMax = { paintStruct.Bounds.x_end, paintStruct.Bounds.y_end, paintStruct.Bounds.z_end };
        evidence.rotation = session.CurrentRotation;

        if (const auto* element = session.CurrentlyDrawnTileElement)
        {
            evidence.hasTileElement = true;
            evidence.elementType = element->getType();
            evidence.elementBaseZ = element->getBaseZ();
            evidence.elementClearanceZ = element->getClearanceZ();
            evidence.elementDirection = static_cast<uint8_t>(element->getDirection());
            evidence.occupiedQuadrants = element->getOccupiedQuadrants();
        }

        _activePaintEvidence->push_back(evidence);
    }
} // namespace OpenRCT2::World3D
