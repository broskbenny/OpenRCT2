/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonMaterialInference.h"
#include <algorithm>
#include <array>
#include <cmath>
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
    // Periodise one complete, flat native tile in world-tile coordinates.
    // A native diamond sprite is not intrinsically periodic: even its opaque
    // pixels may contain different left/right edge colours. Extending its
    // transparent border therefore cannot remove visible 32-unit seams.
    // Preserve the source palette and projective artwork in the interior,
    // but make the four physical edges share the same indexed texels.
    // This is a one-time, per-image material operation, never a frame pass.
    // Partial paths, walls, slopes and source cut-outs must not call it.
    template<typename Vertex>
    [[nodiscard]] inline bool PeriodizeFirstPersonFlatTileMaterial(
        FirstPersonTerrainEdgeMaterial& material,
        const std::array<Vertex, 4>& corners)
    {
        constexpr int32_t side = 32;
        constexpr float invSide = 1.0f / float(side);
        const float ux = corners[1].u - corners[0].u;
        const float uy = corners[1].v - corners[0].v;
        const float vx = corners[3].u - corners[0].u;
        const float vy = corners[3].v - corners[0].v;
        const float determinant = ux * vy - uy * vx;
        if (std::abs(determinant) < 0.01f
            || material.width <= 0 || material.height <= 0
            || material.pixels.size()
                != size_t(material.width) * size_t(material.height))
            return false;

        std::array<uint8_t, side * side> periodic{};
        const auto index = [](int32_t x, int32_t y) {
            return size_t(y * side + x);
        };
        const auto nativeSample = [&](float u, float v) {
            const int32_t x = std::clamp(
                int32_t(std::floor(u + float(material.padding))),
                0, material.width - 1);
            const int32_t y = std::clamp(
                int32_t(std::floor(v + float(material.padding))),
                0, material.height - 1);
            return material.pixels[size_t(y) * size_t(material.width)
                + size_t(x)];
        };
        for (int32_t y = 0; y < side; ++y)
        for (int32_t x = 0; x < side; ++x)
        {
            const float s = (float(x) + 0.5f) * invSide;
            const float t = (float(y) + 0.5f) * invSide;
            periodic[index(x, y)] = nativeSample(
                corners[0].u + ux * s + vx * t,
                corners[0].v + uy * s + vy * t);
        }

        // Material IDs are palette indices, not RGB. Use deterministic
        // ordered index mixing (never numerical averaging of palette IDs)
        // across a narrow band and make opposing boundary texels identical.
        // Work in X first, then Y; the second pass preserves the first pass's
        // boundary equality, including the four corners.
        const auto hash = [](int32_t x, int32_t y) {
            return uint32_t(x * 73 + y * 151 + (x ^ y) * 31) & 15u;
        };
        for (int32_t y = 0; y < side; ++y)
        {
            const uint8_t seam = (hash(0, y) & 1u) != 0
                ? periodic[index(1, y)] : periodic[index(side - 2, y)];
            periodic[index(0, y)] = periodic[index(side - 1, y)] = seam;
            for (int32_t offset = 1; offset <= 3; ++offset)
            {
                if (hash(offset, y) < uint32_t((4 - offset) * 4))
                    periodic[index(offset, y)] = seam;
                if (hash(side - 1 - offset, y) < uint32_t((4 - offset) * 4))
                    periodic[index(side - 1 - offset, y)] = seam;
            }
        }
        for (int32_t x = 0; x < side; ++x)
        {
            const int32_t wrappedX = x == side - 1 ? 0 : x;
            const uint8_t seam = (hash(wrappedX, 0) & 1u) != 0
                ? periodic[index(x, 1)] : periodic[index(x, side - 2)];
            periodic[index(x, 0)] = periodic[index(x, side - 1)] = seam;
            for (int32_t offset = 1; offset <= 3; ++offset)
            {
                if (hash(wrappedX, offset) < uint32_t((4 - offset) * 4))
                    periodic[index(x, offset)] = seam;
                if (hash(wrappedX, side - 1 - offset) < uint32_t((4 - offset) * 4))
                    periodic[index(x, side - 1 - offset)] = seam;
            }
        }

        // Retain the existing atlas dimensions and native UVs. Inverting the
        // projected diamond places the periodic physical-tile material back
        // into precisely the texels that its original triangles will fetch.
        const float inverse = 1.0f / determinant;
        for (int32_t y = 0; y < material.height; ++y)
        for (int32_t x = 0; x < material.width; ++x)
        {
            const float dx = float(x) + 0.5f
                - float(material.padding) - corners[0].u;
            const float dy = float(y) + 0.5f
                - float(material.padding) - corners[0].v;
            const float s = (dx * vy - dy * vx) * inverse;
            const float t = (ux * dy - uy * dx) * inverse;
            if (s < -invSide || s > 1.0f + invSide
                || t < -invSide || t > 1.0f + invSide)
                continue;
            const auto px = std::clamp(
                int32_t(std::floor(s * float(side))), 0, side - 1);
            const auto py = std::clamp(
                int32_t(std::floor(t * float(side))), 0, side - 1);
            material.pixels[size_t(y) * size_t(material.width)
                + size_t(x)] = periodic[index(px, py)];
        }
        return true;
    }

} // namespace OpenRCT2::Paint
