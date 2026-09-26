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

namespace OpenRCT2::Paint
{
    // Cached camera planes: do not recalculate the camera's trigonometry for
    // each park tile. A region is rejected only if its WHOLE bounding sphere
    // lies outside a plane (including near/far). Camera roll is supported.
    struct FirstPersonFrustum
    {
        FirstPersonVec3 eye{};
        FirstPersonBasis basis{};
        float tanHalfHorizontal{}, tanHalfVertical{}, nearDistance{}, farDistance{};

        FirstPersonFrustum(
            const FirstPersonCamera& camera, float horizontalFov, float aspect, float nearClip, float farClip)
            : eye(camera.position)
            , basis(GetFirstPersonBasis(camera))
            , tanHalfHorizontal(std::tan(std::clamp(horizontalFov, 30.0f, 120.0f) * 0.00872664625997f))
            , tanHalfVertical(tanHalfHorizontal / std::max(aspect, 0.01f))
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
            if (x > z * tanHalfHorizontal + radius * std::sqrt(1.0f + tanHalfHorizontal * tanHalfHorizontal))
                return false;
            if (-x > z * tanHalfHorizontal + radius * std::sqrt(1.0f + tanHalfHorizontal * tanHalfHorizontal))
                return false;
            if (y > z * tanHalfVertical + radius * std::sqrt(1.0f + tanHalfVertical * tanHalfVertical))
                return false;
            if (-y > z * tanHalfVertical + radius * std::sqrt(1.0f + tanHalfVertical * tanHalfVertical))
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
    struct FirstPersonRegionSphere
    {
        FirstPersonVec3 center{};
        float radius{};
        bool populated = true;
    };

    template<class Visitor, class Bounds>
    void VisitFirstPersonRegions(
        const FirstPersonFrustum& frustum, int32_t minX, int32_t minY,
        int32_t maxX, int32_t maxY, Visitor& visitor, Bounds& bounds)
    {
        if (minX >= maxX || minY >= maxY)
            return;
        const FirstPersonRegionSphere region = bounds(minX,minY,maxX,maxY);
        if (!region.populated || !frustum.visible(region.center,region.radius))
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
            return FirstPersonRegionSphere{
                {float(x0+x1)*0.5f*kTile,float(y0+y1)*0.5f*kTile,(4096.0f-512.0f)*0.5f},
                std::sqrt(halfX*halfX + halfY*halfY + halfZ*halfZ)};
        };
        VisitFirstPersonRegions(frustum,minX,minY,maxX,maxY,visitor,conservative);
    }
} // namespace OpenRCT2::Paint

