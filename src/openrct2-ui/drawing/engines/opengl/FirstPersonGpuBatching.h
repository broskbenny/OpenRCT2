/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include <openrct2/paint/FirstPersonRenderer.h>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

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

    // Return the exact maximum number of half-open screen rectangles that
    // overlap at any pixel. Coverage changes only at rectangle edges, so a
    // coordinate-compressed sweep avoids rebuilding a full per-pixel tile grid
    // every frame while preserving the same conservative layer bound.
    template<typename RectRange>
    [[nodiscard]] inline size_t FirstPersonMaximumRectangleOverlap(
        const RectRange& rectangles)
    {
        struct Event
        {
            int32_t x{};
            int32_t y0{};
            int32_t y1{};
            int32_t delta{};
        };

        std::vector<Event> events;
        std::vector<int32_t> yEdges;
        for (const auto& rect : rectangles)
        {
            if (rect.x1 <= rect.x0 || rect.y1 <= rect.y0)
                continue;
            events.push_back({ rect.x0, rect.y0, rect.y1, 1 });
            events.push_back({ rect.x1, rect.y0, rect.y1, -1 });
            yEdges.push_back(rect.y0);
            yEdges.push_back(rect.y1);
        }
        if (events.empty())
            return 0;

        std::sort(yEdges.begin(), yEdges.end());
        yEdges.erase(
            std::unique(yEdges.begin(), yEdges.end()),
            yEdges.end());
        if (yEdges.size() < 2)
            return 0;

        const size_t segmentCount = yEdges.size() - 1;
        size_t treeBase = 1;
        while (treeBase < segmentCount)
            treeBase <<= 1;
        std::vector<int32_t> maximum(treeBase * 2, 0);
        std::vector<int32_t> lazy(treeBase * 2, 0);

        const auto addRange =
            [&](auto&& self, size_t node,
                size_t left, size_t right,
                size_t queryLeft, size_t queryRight,
                int32_t delta) -> void {
                if (queryLeft <= left && right <= queryRight)
                {
                    maximum[node] += delta;
                    lazy[node] += delta;
                    return;
                }
                const size_t middle = left + (right - left) / 2;
                if (queryLeft < middle)
                {
                    self(
                        self, node * 2, left, middle,
                        queryLeft, queryRight, delta);
                }
                if (queryRight > middle)
                {
                    self(
                        self, node * 2 + 1, middle, right,
                        queryLeft, queryRight, delta);
                }
                maximum[node] =
                    lazy[node]
                    + std::max(
                        maximum[node * 2],
                        maximum[node * 2 + 1]);
            };

        std::sort(
            events.begin(), events.end(),
            [](const Event& a, const Event& b) {
                return a.x < b.x;
            });

        size_t result = 0;
        for (size_t first = 0; first < events.size();)
        {
            const int32_t x = events[first].x;
            size_t next = first;
            while (next < events.size()
                && events[next].x == x)
            {
                const auto& event = events[next];
                const size_t y0 = size_t(
                    std::lower_bound(
                        yEdges.begin(), yEdges.end(),
                        event.y0) - yEdges.begin());
                const size_t y1 = size_t(
                    std::lower_bound(
                        yEdges.begin(), yEdges.end(),
                        event.y1) - yEdges.begin());
                if (y0 < y1)
                {
                    addRange(
                        addRange, 1, 0, treeBase,
                        y0, y1, event.delta);
                }
                ++next;
            }
            result = std::max(
                result,
                size_t(std::max(maximum[1], 0)));
            first = next;
        }
        return result;
    }
} // namespace OpenRCT2::Ui

