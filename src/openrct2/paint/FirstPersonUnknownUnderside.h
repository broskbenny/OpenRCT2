/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

namespace OpenRCT2::Paint
{
    // Native isometric artwork cannot observe the bottom of an elevated
    // reconstructed boundary. Make such an existing face visibly unknown,
    // rather than deleting it. Ground-contact bottoms are deliberately left
    // out to avoid coincident surfaces with the ground. This creates NO new
    // geometry: the face must already belong to the accepted hull.
    template<typename Face>
    [[nodiscard]] constexpr bool FirstPersonRevealUnknownUnderside(
        const Face& face, float hullMinimumUp, float hullStep,
        bool hullValid, bool opaqueStructuralMode)
    {
        if (!hullValid || opaqueStructuralMode || !(hullStep > 0.0f)
            || face.normal.z >= -0.5f)
            return false;
        float lowest = face.corners[0].z;
        for (const auto& corner : face.corners)
            if (corner.z < lowest)
                lowest = corner.z;
        return lowest > hullMinimumUp + hullStep * 0.5f;
    }
} // namespace OpenRCT2::Paint
