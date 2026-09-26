/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include <openrct2/paint/FirstPersonRenderer.h>
#include <cstdint>

namespace OpenRCT2::Ui
{
    // Stable material identity is used only for streamed-texture deduplication.
    // Persistent static-region reuse is generation/dependency based; it no longer
    // fingerprints resident geometry every frame.
    [[nodiscard]] inline uint64_t FirstPersonMaterialFingerprint(const ImageId& id)
    {
        uint64_t h = uint64_t(id.GetIndex());
        h |= uint64_t(uint8_t(id.GetPrimary())) << 32;
        h |= uint64_t(uint8_t(id.GetSecondary())) << 40;
        h |= uint64_t(uint8_t(id.GetTertiary())) << 48;
        h |= uint64_t(id.IsBlended()) << 56;
        h |= uint64_t(id.IsRemap()) << 57;
        h |= uint64_t(id.HasSecondary()) << 58;
        h |= uint64_t(id.HasTertiary()) << 59;
        return h;
    }
    inline void ExtendFirstPersonFingerprint(uint64_t& fingerprint, uint64_t value)
    {
        // Encode each 64-bit value byte-wise. This remains deterministic across
        // padding/alignment differences and detects UV and placement changes.
        for (unsigned i=0; i<8; ++i)
        {
            fingerprint ^= (value >> (8u*i)) & 255u;
            fingerprint *= 1099511628211ull;
        }
    }
} // namespace OpenRCT2::Ui

