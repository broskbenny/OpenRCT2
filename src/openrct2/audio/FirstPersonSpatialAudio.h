/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace OpenRCT2::Audio
{
    // Listener and emitters use OpenRCT2 world units (32 XY units per tile).
    // The visual camera supplies the position and head-relative RIGHT vector;
    // no viewport rotation, screen pixels or sprite rectangles are involved.
    struct FirstPersonAudioVec3
    {
        float x{}, y{}, z{};
    };

    struct FirstPersonAudioListener
    {
        FirstPersonAudioVec3 position{};
        FirstPersonAudioVec3 right{};
    };

    struct FirstPersonSpatialParams
    {
        bool inRange = false;
        int32_t volume = -10000;       // Native DirectSound-style attenuation.
        int32_t pan = 0;               // -10000 left, +10000 right.
        uint8_t vehicleVolume = 0;     // For OpenRCT2's existing vehicle mixer.
    };

    [[nodiscard]] inline FirstPersonSpatialParams CalculateFirstPersonSpatialParams(
        const FirstPersonAudioListener& listener, FirstPersonAudioVec3 emitter)
    {
        // Deliberate, testable tuning constants, not invented building acoustics.
        constexpr float kMaxDistance = 1536.0f; // 48 map tiles
        constexpr float kRolloffDistance = 256.0f;
        const float dx = emitter.x - listener.position.x;
        const float dy = emitter.y - listener.position.y;
        const float dz = emitter.z - listener.position.z;
        const float distance2 = dx * dx + dy * dy + dz * dz;
        const float rightLength2 = listener.right.x * listener.right.x
            + listener.right.y * listener.right.y + listener.right.z * listener.right.z;
        if (!std::isfinite(distance2) || !std::isfinite(rightLength2)
            || rightLength2 < 1e-8f || distance2 >= kMaxDistance * kMaxDistance)
            return {};

        const float distance = std::sqrt(distance2);
        const float fade = 1.0f - distance / kMaxDistance;
        const float gain = fade * fade / (1.0f + distance2 / (kRolloffDistance * kRolloffDistance));
        if (gain <= 0.0f)
            return {};

        // A conventional stereo mixer has no front/back HRTF. Use head-right
        // dot product, including pitch/roll, and retain 3D distance for volume.
        const float dotRight = dx * listener.right.x + dy * listener.right.y + dz * listener.right.z;
        const float relative = distance * std::sqrt(rightLength2);
        // Source crossing the ears must NOT jump full-left to full-right.
        // Within half a tile, smoothly collapse stereo width towards centre;
        // outside it, preserve legacy far-field separation exactly.
        constexpr float kNearFieldRadius = 16.0f;
        const float nearFieldWidth = std::min(distance / kNearFieldRadius, 1.0f);
        const float lateral = relative > 1e-5f ? (dotRight / relative) * nearFieldWidth : 0.0f;
        FirstPersonSpatialParams params{};
        params.inRange = true;
        params.pan = static_cast<int32_t>(std::lround(std::clamp(lateral, -1.0f, 1.0f) * 10000.0f));
        params.volume = std::clamp(static_cast<int32_t>(std::lround(2000.0f * std::log10(gain))), -10000, 0);
        params.vehicleVolume = static_cast<uint8_t>(std::lround(std::clamp(gain, 0.0f, 1.0f) * 255.0f));
        return params;
    }

    // Priority retains train speed/mass and channel-continuity as tie breakers,
    // but one quantized gain step outranks all legacy priority differences.
    // Inaudible vehicles MUST be rejected before consuming one of 14 slots.
    [[nodiscard]] inline uint16_t FirstPersonVehiclePriority(
        uint32_t nativePriority, bool alreadyPlaying, uint8_t audibleGain)
    {
        const uint32_t tieBreaker = std::min(nativePriority / 128u, 127u)
            + (alreadyPlaying ? 128u : 0u);
        return static_cast<uint16_t>(uint32_t(audibleGain) * 256u + tieBreaker);
    }

    // Continue existing ride music down to essentially silent level rather
    // than deleting a channel at -4000 (one-percent amplitude). -8000 is
    // 0.01 percent amplitude and occurs inside the absolute 48-tile cutoff.
    [[nodiscard]] inline bool IsFirstPersonRideMusicAudible(const FirstPersonSpatialParams& params)
    {
        return params.inRange && params.volume > -8000;
    }

    [[nodiscard]] inline uint16_t FirstPersonCrowdWeight(uint8_t audibleGain, bool queuing)
    {
        return static_cast<uint16_t>((queuing ? 1u : 2u) * audibleGain);
    }

} // namespace OpenRCT2::Audio

