/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

struct PaintSession;
struct CoordsXY;
namespace OpenRCT2 { struct EntityBase; }

void EntityPaintSetup(PaintSession& session, const CoordsXY& pos);
void EntityPaintSetupEntity(PaintSession& session, OpenRCT2::EntityBase& entity);
