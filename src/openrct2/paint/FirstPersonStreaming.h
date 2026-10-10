/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonMath.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

namespace OpenRCT2::Paint
{
    // Broad phase for coplanar diagnostics. A plane can span a whole park;
    // sharing its normal and distance does not make two surfaces neighbours.
    // Large bounds use a separate list so one unusual object cannot allocate
    // an unbounded number of cells. Every inspected entry consumes the caller's
    // diagnostic budget, including rejected large-bound candidates.
    template<typename Probe>
    class FirstPersonPlaneSpatialIndex
    {
        struct Range { int32_t x0, y0, x1, y1; };
        static constexpr float kCellSize = 32.0f;
        static constexpr int64_t kMaximumCells = 64;
        std::vector<Probe> _probes;
        std::vector<uint64_t> _seen;
        std::vector<size_t> _large;
        std::unordered_map<uint64_t, std::vector<size_t>> _cells;
        uint64_t _serial = 0;

        static std::optional<Range> range(const Probe& p)
        {
            const std::array<float, 4> values{ p.minU, p.minV, p.maxU, p.maxV };
            for (const auto value : values)
                if (!std::isfinite(value) || std::abs(value) > 1.0e9f) return std::nullopt;
            if (p.maxU < p.minU || p.maxV < p.minV) return std::nullopt;
            const Range r{
                int32_t(std::floor(p.minU / kCellSize)), int32_t(std::floor(p.minV / kCellSize)),
                int32_t(std::floor(p.maxU / kCellSize)), int32_t(std::floor(p.maxV / kCellSize))
            };
            if ((int64_t(r.x1) - r.x0 + 1) * (int64_t(r.y1) - r.y0 + 1) > kMaximumCells)
                return std::nullopt;
            return r;
        }
        static uint64_t key(int32_t x, int32_t y)
        {
            return (uint64_t(uint32_t(x)) << 32) | uint32_t(y);
        }
    public:
        void insert(const Probe& probe)
        {
            const size_t index = _probes.size();
            _probes.push_back(probe);
            _seen.push_back(0);
            const auto cells = range(probe);
            if (!cells) { _large.push_back(index); return; }
            for (int32_t y = cells->y0; y <= cells->y1; ++y)
            for (int32_t x = cells->x0; x <= cells->x1; ++x)
                _cells[key(x, y)].push_back(index);
        }

        std::vector<const Probe*> query(const Probe& probe, size_t& remainingChecks)
        {
            if (++_serial == 0)
            {
                std::fill(_seen.begin(), _seen.end(), 0);
                ++_serial;
            }
            std::vector<const Probe*> result;
            const auto visit = [&](size_t index) {
                if (_seen[index] == _serial || remainingChecks == 0) return;
                _seen[index] = _serial;
                --remainingChecks;
                const auto& p = _probes[index];
                if (p.maxU >= probe.minU && p.minU <= probe.maxU
                    && p.maxV >= probe.minV && p.minV <= probe.maxV)
                    result.push_back(&p);
            };
            const auto cells = range(probe);
            if (!cells)
            {
                for (size_t i = 0; i < _probes.size() && remainingChecks != 0; ++i) visit(i);
                return result;
            }
            for (const auto index : _large)
            {
                if (remainingChecks == 0) return result;
                visit(index);
            }
            for (int32_t y = cells->y0; y <= cells->y1 && remainingChecks != 0; ++y)
            for (int32_t x = cells->x0; x <= cells->x1 && remainingChecks != 0; ++x)
            {
                const auto found = _cells.find(key(x, y));
                if (found == _cells.end()) continue;
                for (const auto index : found->second)
                {
                    if (remainingChecks == 0) return result;
                    visit(index);
                }
            }
            return result;
        }
    };

    // Cached camera planes: do not recalculate the camera's trigonometry for
    // each park tile. A region is rejected only if its WHOLE bounding sphere
    // lies outside a plane (including near/far). Camera roll is supported.
    struct FirstPersonFrustum
    {
        FirstPersonVec3 eye{};
        FirstPersonBasis basis{};
        float tanHalfHorizontal{}, tanHalfVertical{};
        float horizontalRadiusScale{}, verticalRadiusScale{};
        float nearDistance{}, farDistance{};

        FirstPersonFrustum(
            const FirstPersonCamera& camera, float horizontalFov, float aspect, float nearClip, float farClip)
            : eye(camera.position)
            , basis(GetFirstPersonBasis(camera))
            , tanHalfHorizontal(std::tan(std::clamp(horizontalFov, 30.0f, 120.0f) * 0.00872664625997f))
            , tanHalfVertical(tanHalfHorizontal / std::max(aspect, 0.01f))
            , horizontalRadiusScale(std::sqrt(1.0f + tanHalfHorizontal * tanHalfHorizontal))
            , verticalRadiusScale(std::sqrt(1.0f + tanHalfVertical * tanHalfVertical))
            , nearDistance(nearClip)
            , farDistance(farClip)
        {
        }

        [[nodiscard]] bool visible(FirstPersonVec3 center, float radius) const
        {
            const FirstPersonVec3 d{ center.x - eye.x, center.y - eye.y, center.z - eye.z };
            const float x = FpDot(d, basis.right);
            const float y = FpDot(d, basis.up);
            const float z = FpDot(d, basis.forward);
            if (z + radius < nearDistance || z - radius > farDistance)
                return false;
            if (x > z * tanHalfHorizontal + radius * horizontalRadiusScale)
                return false;
            if (-x > z * tanHalfHorizontal + radius * horizontalRadiusScale)
                return false;
            if (y > z * tanHalfVertical + radius * verticalRadiusScale)
                return false;
            if (-y > z * tanHalfVertical + radius * verticalRadiusScale)
                return false;
            return true;
        }

        // Exact plane support test for an axis-aligned world box. This is
        // tighter than testing the box's enclosing sphere, especially for
        // large rectangular map regions, while remaining fully conservative.
        [[nodiscard]] bool visibleAabb(
            FirstPersonVec3 center,
            FirstPersonVec3 halfExtents) const
        {
            const FirstPersonVec3 d{
                center.x - eye.x,
                center.y - eye.y,
                center.z - eye.z,
            };
            const float x = FpDot(d, basis.right);
            const float y = FpDot(d, basis.up);
            const float z = FpDot(d, basis.forward);
            const auto support =
                [&](FirstPersonVec3 normal) {
                    return std::abs(normal.x)
                            * halfExtents.x
                        + std::abs(normal.y)
                            * halfExtents.y
                        + std::abs(normal.z)
                            * halfExtents.z;
                };

            const float forwardRadius =
                support(basis.forward);
            if (z + forwardRadius < nearDistance
                || z - forwardRadius > farDistance)
                return false;

            const FirstPersonVec3 rightPlane{
                basis.right.x
                    - basis.forward.x
                        * tanHalfHorizontal,
                basis.right.y
                    - basis.forward.y
                        * tanHalfHorizontal,
                basis.right.z
                    - basis.forward.z
                        * tanHalfHorizontal,
            };
            const FirstPersonVec3 leftPlane{
                -basis.right.x
                    - basis.forward.x
                        * tanHalfHorizontal,
                -basis.right.y
                    - basis.forward.y
                        * tanHalfHorizontal,
                -basis.right.z
                    - basis.forward.z
                        * tanHalfHorizontal,
            };
            if (x - z * tanHalfHorizontal
                    > support(rightPlane)
                || -x - z * tanHalfHorizontal
                    > support(leftPlane))
                return false;

            const FirstPersonVec3 topPlane{
                basis.up.x
                    - basis.forward.x
                        * tanHalfVertical,
                basis.up.y
                    - basis.forward.y
                        * tanHalfVertical,
                basis.up.z
                    - basis.forward.z
                        * tanHalfVertical,
            };
            const FirstPersonVec3 bottomPlane{
                -basis.up.x
                    - basis.forward.x
                        * tanHalfVertical,
                -basis.up.y
                    - basis.forward.y
                        * tanHalfVertical,
                -basis.up.z
                    - basis.forward.z
                        * tanHalfVertical,
            };
            if (y - z * tanHalfVertical
                    > support(topPlane)
                || -y - z * tanHalfVertical
                    > support(bottomPlane))
                return false;
            return true;
        }
    };

    // The default 32,768-unit far plane cannot include the opposite corner
    // of a 1,024 x 1,024 tile park (the planar diagonal exceeds 46,000).
    // Derive coverage from the loaded map, the actual observer and a vertical
    // source-art allowance; do NOT collapse it in response to a low FPS.
    [[nodiscard]] inline float CompleteParkFarClip(
        FirstPersonVec3 eye, int32_t mapTilesX, int32_t mapTilesY,
        float requestedFar = 32768.0f)
    {
        const float width = float(std::max(0,mapTilesX)) * 32.0f;
        const float height = float(std::max(0,mapTilesY)) * 32.0f;
        float farthest = std::max(0.0f,requestedFar);
        for (const float x : {0.0f,width})
        for (const float y : {0.0f,height})
        for (const float z : {-512.0f,4096.0f})
            farthest=std::max(farthest,
                std::hypot(std::hypot(x-eye.x,y-eye.y),z-eye.z)+512.0f);
        return farthest;
    }

    struct FirstPersonResolvedView
    {
        FirstPersonCamera camera{};
        float fieldOfViewDegrees = 70.0f;
        float aspect = 1.0f;
        float nearClip = 2.0f;
        float farClip = 32768.0f;
    };

    [[nodiscard]] inline FirstPersonResolvedView ResolveFirstPersonView(
        const FirstPersonCamera& camera, int32_t width, int32_t height,
        int32_t mapTilesX, int32_t mapTilesY,
        float fieldOfViewDegrees = 70.0f, float nearClip = 2.0f,
        float requestedFar = 32768.0f)
    {
        return {
            camera,
            fieldOfViewDegrees,
            height > 0 ? float(std::max(width, 1)) / float(height) : 1.0f,
            nearClip,
            CompleteParkFarClip(camera.position, mapTilesX, mapTilesY, requestedFar),
        };
    }

    // Periodic fallback repaint protects against game-side art changes that do
    // not edit tile bytes and do not emit a map invalidation. Spread a cold
    // park's subsequent repaints across frames instead of repainting every
    // tile in the SAME frame, which would cause a predictable ride hitch.
    // Immediate native invalidation and tile-signature changes bypass this.
    [[nodiscard]] constexpr uint64_t FirstPersonRefreshPhase(uint64_t key, uint64_t interval)
    {
        if (interval == 0) return 0;
        key ^= key >> 30;
        key *= 0xbf58476d1ce4e5b9ull;
        key ^= key >> 27;
        key *= 0x94d049bb133111ebull;
        key ^= key >> 31;
        return key % interval;
    }
    [[nodiscard]] constexpr bool FirstPersonRefreshDue(
        uint64_t key, uint64_t frame, uint64_t lastPainted, uint64_t interval)
    {
        if (interval == 0 || frame <= lastPainted || frame - lastPainted <= interval)
            return false;
        return (frame + FirstPersonRefreshPhase(key, interval)) % interval == 0;
    }

    // Tile-map aligned region partition: a full-park visibility traversal, not
    // a fixed camera-centred ring. Bounds are deliberately conservative until
    // OpenRCT2 exposes authoritative region invalidation and accurate bounds.
    // In particular, raised track is NOT culled by the terrain height below it.
    struct FirstPersonRegionBounds
    {
        FirstPersonVec3 center{};
        FirstPersonVec3 halfExtents{};
        bool populated = true;
    };

    template<class Visitor, class Bounds>
    void VisitFirstPersonRegions(
        const FirstPersonFrustum& frustum, int32_t minX, int32_t minY,
        int32_t maxX, int32_t maxY, Visitor& visitor, Bounds& bounds)
    {
        if (minX >= maxX || minY >= maxY)
            return;
        const FirstPersonRegionBounds region =
            bounds(minX, minY, maxX, maxY);
        if (!region.populated
            || !frustum.visibleAabb(
                region.center, region.halfExtents))
            return;
        if (maxX - minX <= 16 && maxY - minY <= 16)
        {
            visitor(minX, minY, maxX, maxY);
            return;
        }
        // Split only on tile boundaries, so fine and coarse blocks remain
        // mutually aligned and a full-park traversal never overlaps regions.
        if (maxY - minY >= maxX - minX)
        {
            const int32_t mid = minY + (maxY - minY) / 2;
            VisitFirstPersonRegions(frustum, minX, minY, maxX, mid, visitor, bounds);
            VisitFirstPersonRegions(frustum, minX, mid, maxX, maxY, visitor, bounds);
        }
        else
        {
            const int32_t mid = minX + (maxX - minX) / 2;
            VisitFirstPersonRegions(frustum, minX, minY, mid, maxY, visitor, bounds);
            VisitFirstPersonRegions(frustum, mid, minY, maxX, maxY, visitor, bounds);
        }
    }
    template<class Visitor>
    void VisitFirstPersonRegions(
        const FirstPersonFrustum& frustum, int32_t minX, int32_t minY,
        int32_t maxX, int32_t maxY, Visitor& visitor)
    {
        auto conservative = [](int32_t x0, int32_t y0, int32_t x1, int32_t y1) {
            constexpr float kTile = 32.0f;
            const float halfX = float(x1-x0)*0.5f*kTile;
            const float halfY = float(y1-y0)*0.5f*kTile;
            const float halfZ = (4096.0f + 512.0f)*0.5f;
            constexpr float kArtworkHalo = 512.0f;
            return FirstPersonRegionBounds{
                {
                    float(x0+x1)*0.5f*kTile,
                    float(y0+y1)*0.5f*kTile,
                    (4096.0f-512.0f)*0.5f
                },
                {
                    halfX + kArtworkHalo,
                    halfY + kArtworkHalo,
                    halfZ + kArtworkHalo
                }
            };
        };
        VisitFirstPersonRegions(frustum,minX,minY,maxX,maxY,visitor,conservative);
    }
} // namespace OpenRCT2::Paint
