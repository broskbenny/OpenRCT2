/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonMaterialInference.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace OpenRCT2::Paint
{
    struct FirstPersonTerrainEdgeMaterial
    {
        std::vector<uint8_t> pixels;
        int32_t width = 0;
        int32_t height = 0;
        int32_t padding = 0;
    };

    // An RCT2 terrain sprite is a cropped isometric raster. Its transparent
    // border is NOT a real hole through solid terrain. When mapped onto the
    // physical tile, that border currently becomes a uniform fallback-colour
    // wedge. Extend only the observed native indexed colours to nearby UV
    // samples, with a shared per-image resident bitmap. Actual scenery
    // cut-outs continue to use their original transparent sprites unchanged.
    [[nodiscard]] inline std::optional<FirstPersonTerrainEdgeMaterial>
        BuildFirstPersonTerrainEdgeMaterial(
            const std::vector<uint8_t>& nativePixels,
            int32_t nativeWidth, int32_t nativeHeight,
            int32_t padding = 16)
    {
        if (nativeWidth <= 0 || nativeHeight <= 0 || padding < 0
            || nativePixels.size() != size_t(nativeWidth) * size_t(nativeHeight))
            return std::nullopt;
        const int32_t width = nativeWidth + 2 * padding;
        const int32_t height = nativeHeight + 2 * padding;
        constexpr int64_t kMaxTerrainMaterialPixels = 32768;
        if (int64_t(width) * int64_t(height) > kMaxTerrainMaterialPixels)
            return std::nullopt;

        FirstPersonTerrainEdgeMaterial material;
        material.width = width;
        material.height = height;
        material.padding = padding;
        material.pixels.assign(size_t(width) * size_t(height), 0);
        for (int32_t y = 0; y < nativeHeight; ++y)
        {
            std::copy_n(
                nativePixels.data() + size_t(y) * size_t(nativeWidth),
                nativeWidth,
                material.pixels.data() + size_t(y + padding) * size_t(width)
                    + size_t(padding));
        }
        // This fill only affects a tile's *material*. The authoritative
        // terrain triangles, water clipping, and physical voids are unchanged.
        if (FillFirstPersonOpaqueStructuralMaterial(
                material.pixels, width, height) == 0)
            return std::nullopt;
        return material;
    }
    // An isometric native material is not periodic. Instead of changing any
    // palette pixels, reflect its physical tile UV basis on alternating map
    // coordinates. Along an edge between equal native sprites, both tiles
    // then fetch EXACTLY the same original texels. A mirror changes neither
    // the geometry nor the authored alpha/colour and adds no atlas entries.
    // Only call this on flat, symmetric tile art: a sloped or asymmetric
    // edge-specific sprite would otherwise acquire the wrong orientation.
    template<typename Vertex>
    inline void MirrorFirstPersonFlatTileUV(
        std::array<Vertex, 4>& vertices, bool mirrorX, bool mirrorY)
    {
        if (!mirrorX && !mirrorY)
            return;
        constexpr std::array<size_t, 4> flipX{ 1, 0, 3, 2 };
        constexpr std::array<size_t, 4> flipY{ 3, 2, 1, 0 };
        const auto original = vertices;
        for (size_t i = 0; i < 4; ++i)
        {
            size_t src = mirrorX ? flipX[i] : i;
            if (mirrorY)
                src = flipY[src];
            vertices[i].u = original[src].u;
            vertices[i].v = original[src].v;
        }
    }

} // namespace OpenRCT2::Paint
