/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include <openrct2/paint/FirstPersonRenderer.h>
#include <bit>
#include <cstdint>
#include <vector>

namespace OpenRCT2::Ui
{
    // Fingerprint ONLY the actual vertex, material and mask data. Never hash
    // struct padding or pointers into temporary native paint sessions.
    // Camera-facing geometry has no stable GPU-region provenance, by design.
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
    inline void ExtendFirstPersonSurfaceFingerprint(uint64_t& fingerprint, const Paint::FirstPersonSurface& s)
    {
        ExtendFirstPersonFingerprint(fingerprint,FirstPersonMaterialFingerprint(s.image));
        ExtendFirstPersonFingerprint(fingerprint,s.mask.HasValue() ? FirstPersonMaterialFingerprint(s.mask) : 0);
        ExtendFirstPersonFingerprint(fingerprint,s.depthBias ? 1u : 0u);
        ExtendFirstPersonFingerprint(fingerprint,s.edgeCoverage ? 1u : 0u);
        ExtendFirstPersonFingerprint(fingerprint,s.immutableFingerprint);
        for (const auto& v : s.triangles)
        {
            ExtendFirstPersonFingerprint(fingerprint,std::bit_cast<uint32_t>(v.world.x));
            ExtendFirstPersonFingerprint(fingerprint,std::bit_cast<uint32_t>(v.world.y));
            ExtendFirstPersonFingerprint(fingerprint,std::bit_cast<uint32_t>(v.world.z));
            ExtendFirstPersonFingerprint(fingerprint,std::bit_cast<uint32_t>(v.u));
            ExtendFirstPersonFingerprint(fingerprint,std::bit_cast<uint32_t>(v.v));
        }
    }
    [[nodiscard]] inline uint64_t FirstPersonRegionFingerprint(
        const std::vector<const Paint::FirstPersonSurface*>& surfaces)
    {
        uint64_t fingerprint=14695981039346656037ull;
        ExtendFirstPersonFingerprint(fingerprint,surfaces.size());
        for (const auto* s : surfaces)
            ExtendFirstPersonSurfaceFingerprint(fingerprint,*s);
        return fingerprint;
    }
} // namespace OpenRCT2::Ui

