/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace OpenRCT2::Paint
{
    // Repair only isolated, weakly transparent raster texels embedded in
    // demonstrably native-coloured material. A real opening, an edge cut-out,
    // an unknown/checkerboard region or a multi-view-confirmed transparent
    // pixel must not be overwritten.
    //
    // provenance[i] == 1: a nonzero palette index directly observed in native
    // artwork. eligible[i] == 1: exactly one native view explicitly observed
    // transparency over the full footprint; other views offered no evidence.
    // A caller must not mark a pixel eligible where multiple views confirm
    // transparency or where any authored negative-passage constraint applies.
    [[nodiscard]] inline size_t RepairFirstPersonIsolatedMaterialPinhole(
        std::vector<uint8_t>& pixels,
        const std::vector<uint8_t>& provenance,
        const std::vector<uint8_t>& eligible,
        int32_t width, int32_t height)
    {
        if (width < 3 || height < 3 || width > 256 || height > 256)
            return 0;
        const size_t count = size_t(width) * size_t(height);
        if (pixels.size() != count || provenance.size() != count
            || eligible.size() != count)
            return 0;

        size_t repairs = 0;
        for (int32_t y = 1; y < height - 1; ++y)
        for (int32_t x = 1; x < width - 1; ++x)
        {
            const size_t index = size_t(y) * size_t(width) + size_t(x);
            if (pixels[index] != 0 || eligible[index] == 0)
                continue;

            uint8_t palette[8]{};
            size_t available = 0;
            for (int32_t dy = -1; dy <= 1; ++dy)
            for (int32_t dx = -1; dx <= 1; ++dx)
            {
                if (dx == 0 && dy == 0)
                    continue;
                const size_t neighbour = size_t(y + dy) * size_t(width)
                    + size_t(x + dx);
                if (provenance[neighbour] == 0 || pixels[neighbour] == 0)
                    continue;
                palette[available++] = pixels[neighbour];
            }
            // Do not bridge an adjacent gap, unknown material or an outer
            // silhouette boundary: all eight neighbours must be authentic.
            if (available != 8)
                continue;
            uint8_t chosen = 0;
            size_t agreement = 0;
            for (const uint8_t value : palette)
            {
                size_t matches = 0;
                for (const uint8_t other : palette)
                    matches += value == other;
                if (matches > agreement)
                {
                    chosen = value;
                    agreement = matches;
                }
            }
            // Palette indices represent categories, not linear RGB. Fill
            // only when at least 6/8 source samples agree exactly.
            if (agreement < 6)
                continue;
            pixels[index] = chosen;
            ++repairs;
            // Keep provenance false: one repair cannot support another.
        }
        return repairs;
    }
} // namespace OpenRCT2::Paint
