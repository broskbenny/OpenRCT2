/*****************************************************************************/
/* Copyright (c) 2014-2026 OpenRCT2 developers                             */
/* OpenRCT2 is licensed under the GNU General Public License version 3.     */
/*****************************************************************************/
#pragma once

namespace OpenRCT2::Paint
{
    // A source sprite's alpha describes one *projected image*, not an
    // independently verified hole in an already reconstructed 3D surface.
    // Geometry (including known negative passages) decides where faces exist.
    // A lone transparent native projection is unresolved APPEARANCE evidence
    // on a solid boundary. Preserve cutouts with agreement from two views.
    // The caller must retain its separate unknown-material diagnostic/fallback.
    template<typename TDecision>
    [[nodiscard]] constexpr TDecision ResolveFirstPersonBoundaryMaterial(
        TDecision decision, bool acceptedOccupancyBoundary)
    {
        if (acceptedOccupancyBoundary
            && decision.kind == decltype(decision.kind)::transparent
            && decision.transparentViews < 2)
        {
            decision.kind = decltype(decision.kind)::unknown;
            decision.pixel = 0;
        }
        return decision;
    }
} // namespace OpenRCT2::Paint
