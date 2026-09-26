/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include <cstdint>

struct PaintSession;

namespace OpenRCT2
{
    struct PathElement;
}

void PaintPath(PaintSession& session, uint16_t height, const OpenRCT2::PathElement& tileElement);

// Returns the native surface-art offset for a path at a specific paint rotation.
// First-person rendering uses the same selection even when supported paths omit
// the separate surface sprite and put the visible deck inside bridge artwork.
uint8_t GetPathSurfaceImageOffset(const OpenRCT2::PathElement& pathElement, uint8_t rotation);
