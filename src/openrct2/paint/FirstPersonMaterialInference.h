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
} // namespace OpenRCT2::Paint
