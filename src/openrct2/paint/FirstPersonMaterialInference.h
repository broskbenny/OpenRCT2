/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <deque>
#include <vector>

namespace OpenRCT2::Paint
{
    enum class FirstPersonMaterialEvidenceKind : uint8_t
    {
        unknown,
        transparent,
        colour,
    };

    struct FirstPersonMaterialEvidenceSample
    {
        FirstPersonMaterialEvidenceKind kind =
            FirstPersonMaterialEvidenceKind::unknown;
        uint8_t pixel = 0;
    };

    struct FirstPersonMaterialPatchDecision
    {
        FirstPersonMaterialEvidenceKind kind =
            FirstPersonMaterialEvidenceKind::unknown;
        uint8_t pixel = 0;
        size_t observedViews = 0;
        size_t opaqueViews = 0;
        size_t transparentViews = 0;
        size_t fullyTransparentViews = 0;
        size_t bestOpaqueSamples = 0;
    };

    class FirstPersonMaterialPatchAccumulator
    {
    public:
        template<typename SampleRange>
        void addView(const SampleRange& samples)
        {
            size_t known = 0;
            size_t opaque = 0;
            uint8_t firstOpaque = 0;
            for (const auto& sample : samples)
            {
                if (sample.kind
                    == FirstPersonMaterialEvidenceKind::unknown)
                    continue;
                ++known;
                if (sample.kind
                    == FirstPersonMaterialEvidenceKind::colour
                    && sample.pixel != 0)
                {
                    ++opaque;
                    if (firstOpaque == 0)
                        firstOpaque = sample.pixel;
                }
            }
            if (known == 0)
                return;

            ++_result.observedViews;
            if (opaque != 0)
            {
                ++_result.opaqueViews;
                if (opaque > _result.bestOpaqueSamples)
                {
                    _result.bestOpaqueSamples = opaque;
                    _result.pixel = firstOpaque;
                }
                return;
            }

            ++_result.transparentViews;
            if (known == samples.size())
                ++_result.fullyTransparentViews;
        }

        [[nodiscard]] FirstPersonMaterialPatchDecision
            finish() const
        {
            auto result = _result;

            // A single opaque sub-sample is positive material evidence. The
            // candidate surface already exists geometrically; sparse native
            // raster gaps must not override a directly observed colour.
            if (result.opaqueViews != 0
                && result.pixel != 0)
            {
                result.kind =
                    FirstPersonMaterialEvidenceKind::colour;
                return result;
            }

            // Transparency is a stronger claim than colour because it removes
            // visible material from an otherwise reconstructed surface. Demand
            // either agreement from two views, or one view whose whole sampled
            // projected footprint is known and transparent. This preserves real
            // cut-outs without promoting isolated raster misses to 3D holes.
            if (result.transparentViews >= 2
                || result.fullyTransparentViews >= 1)
            {
                result.kind =
                    FirstPersonMaterialEvidenceKind::transparent;
                return result;
            }

            result.kind =
                FirstPersonMaterialEvidenceKind::unknown;
            return result;
        }

    private:
        FirstPersonMaterialPatchDecision _result{};
    };

    [[nodiscard]] constexpr size_t
        FirstPersonMaterialPatchSampleCount()
    {
        return 9;
    }

    struct FirstPersonMaterialPatchOffset
    {
        float s{};
        float t{};
    };

    [[nodiscard]] inline const std::array<
        FirstPersonMaterialPatchOffset,
        FirstPersonMaterialPatchSampleCount()>&
        FirstPersonMaterialPatchOffsets()
    {
        // Centre first so the selected palette index stays as close as
        // possible to the original point sample when several opaque samples
        // are available. The remaining samples cover the texel footprint.
        static constexpr std::array<
            FirstPersonMaterialPatchOffset,
            FirstPersonMaterialPatchSampleCount()> offsets{ {
            { 0.5f, 0.5f },
            { 1.0f / 6.0f, 1.0f / 6.0f },
            { 0.5f, 1.0f / 6.0f },
            { 5.0f / 6.0f, 1.0f / 6.0f },
            { 1.0f / 6.0f, 0.5f },
            { 5.0f / 6.0f, 0.5f },
            { 1.0f / 6.0f, 5.0f / 6.0f },
            { 0.5f, 5.0f / 6.0f },
            { 5.0f / 6.0f, 5.0f / 6.0f },
        } };
        return offsets;
    }

    // Reconstructed structural boundary geometry has already decided where
    // visible matter exists. Once a boundary rectangle survives carving,
    // transparent raster samples are missing appearance evidence, not a second
    // independent permission to punch holes through that rectangle. Propagate
    // the nearest directly observed palette colour across the face. The caller
    // retains an explicit unknown-material fallback when a face has no colour
    // evidence at all.
    [[nodiscard]] inline size_t
        FillFirstPersonOpaqueStructuralMaterial(
            std::vector<uint8_t>& pixels,
            int32_t width, int32_t height)
    {
        if (width <= 0 || height <= 0
            || pixels.size()
                != size_t(width) * size_t(height))
            return 0;

        constexpr int32_t kUnvisited =
            std::numeric_limits<int32_t>::max();
        std::vector<int32_t> distance(
            pixels.size(), kUnvisited);
        std::deque<size_t> queue;
        for (size_t i = 0; i < pixels.size(); ++i)
        {
            if (pixels[i] == 0)
                continue;
            distance[i] = 0;
            queue.push_back(i);
        }
        if (queue.empty())
            return 0;

        constexpr std::array<int32_t, 4> dx{
            -1, 1, 0, 0
        };
        constexpr std::array<int32_t, 4> dy{
            0, 0, -1, 1
        };
        while (!queue.empty())
        {
            const size_t current = queue.front();
            queue.pop_front();
            const int32_t x =
                int32_t(current % size_t(width));
            const int32_t y =
                int32_t(current / size_t(width));
            for (size_t direction = 0;
                 direction < dx.size(); ++direction)
            {
                const int32_t nx = x + dx[direction];
                const int32_t ny = y + dy[direction];
                if (nx < 0 || ny < 0
                    || nx >= width || ny >= height)
                    continue;
                const size_t next =
                    size_t(ny) * size_t(width)
                    + size_t(nx);
                if (distance[next] != kUnvisited)
                    continue;
                distance[next] =
                    distance[current] + 1;
                pixels[next] = pixels[current];
                queue.push_back(next);
            }
        }
        return pixels.size();
    }

} // namespace OpenRCT2::Paint
