/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

namespace OpenRCT2::Paint
{
    struct FirstPersonVec3 { float x{}, y{}, z{}; };
    struct FirstPersonBasis
    {
        FirstPersonVec3 forward{}, right{}, up{};
    };
    struct FirstPersonCamera
    {
        FirstPersonVec3 position{};
        float yaw{}, pitch{}, roll{};
        bool hasExplicitBasis = false;
        FirstPersonBasis explicitBasis{};
    };
    struct FirstPersonProjection
    {
        float x{}, y{}, depth{}, scale{};
    };
    inline FirstPersonVec3 FpCross(FirstPersonVec3 a, FirstPersonVec3 b)
    {
        return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
    }
    inline float FpDot(FirstPersonVec3 a, FirstPersonVec3 b)
    {
        return a.x*b.x + a.y*b.y + a.z*b.z;
    }
    inline FirstPersonBasis GetFirstPersonBasis(const FirstPersonCamera& c)
    {
        if (c.hasExplicitBasis) return c.explicitBasis;
        const auto cy = std::cos(c.yaw), sy = std::sin(c.yaw);
        const auto cp = std::cos(c.pitch), sp = std::sin(c.pitch);
        FirstPersonBasis b{};
        b.forward = { cy*cp, sy*cp, sp };
        const FirstPersonVec3 right0{ -sy, cy, 0.0f };
        const auto up0 = FpCross(b.forward, right0);
        const float cr = std::cos(c.roll), sr = std::sin(c.roll);
        b.right = {
            right0.x*cr + up0.x*sr,
            right0.y*cr + up0.y*sr,
            right0.z*cr + up0.z*sr,
        };
        b.up = {
            up0.x*cr - right0.x*sr,
            up0.y*cr - right0.y*sr,
            up0.z*cr - right0.z*sr,
        };
        return b;
    }
    // Interpolate a signed angular displacement around the SHORTEST arc.
    // Raw yaw is a 32-step turn, spinning-car sprite orientation a 256-step
    // turn; direct numeric lerp would spin nearly 360 degrees at wraparound.
    [[nodiscard]] inline float FirstPersonLerpAngle(float a, float b, float alpha)
    {
        constexpr float kTwoPi = 6.28318530717958647692f;
        constexpr float kPi = 3.14159265358979323846f;
        float delta = std::remainder(b - a, kTwoPi);
        if (delta <= -kPi) delta += kTwoPi;
        return a + std::clamp(alpha, 0.0f, 1.0f) * delta;
    }
    // The native footpath and terrain tiles share 32-unit, axis-aligned
    // coordinates. Path ramps have two high and two low corners. Bilinear
    // interpolation gives their exact height at an arbitrary walking point,
    // rather than jumping 8 units to the ramp midpoint on tile entry.
    [[nodiscard]] inline float FirstPersonPathHeight(
        float south, float east, float north, float west,
        float localX, float localY)
    {
        const float x = std::clamp(localX / 32.0f, 0.0f, 1.0f);
        const float y = std::clamp(localY / 32.0f, 0.0f, 1.0f);
        const float alongSouth = south + (east - south) * x;
        const float alongNorth = west + (north - west) * x;
        return alongSouth + (alongNorth - alongSouth) * y;
    }
    // Authored cabin/seat positions use the physical car basis, so pitch,
    // inversion and spinning can move the passenger without moving the park.
    // This helper does not pretend that RCT2 loading paths encode eye seats.
    [[nodiscard]] inline FirstPersonVec3 FirstPersonPassengerEye(
        FirstPersonVec3 carPosition, const FirstPersonBasis& car,
        FirstPersonVec3 localForwardRightUp)
    {
        const auto& o = localForwardRightUp;
        return { carPosition.x + car.forward.x*o.x + car.right.x*o.y + car.up.x*o.z,
                 carPosition.y + car.forward.y*o.x + car.right.y*o.y + car.up.y*o.z,
                 carPosition.z + car.forward.z*o.x + car.right.z*o.y + car.up.z*o.z };
    }
    // Swept circular walking collision against a physical wall segment.
    // Check the WHOLE step, not only its destination: a 320 world-unit/s
    // sprint can otherwise tunnel through a 1-unit-thick wall in one frame.
    [[nodiscard]] inline float FirstPersonPointSegmentDistance2(
        float px, float py, float ax, float ay, float bx, float by)
    {
        const float dx = bx-ax, dy = by-ay;
        const float length2 = dx*dx+dy*dy;
        const float t = length2 > 1e-8f
            ? std::clamp(((px-ax)*dx+(py-ay)*dy)/length2,0.0f,1.0f) : 0.0f;
        const float ex = px-ax-t*dx, ey = py-ay-t*dy;
        return ex*ex+ey*ey;
    }
    [[nodiscard]] inline float FirstPersonPointSegmentParameter2(
        float px, float py, float ax, float ay, float bx, float by)
    {
        const float dx = bx - ax;
        const float dy = by - ay;
        const float length2 = dx * dx + dy * dy;
        if (length2 <= 1e-8f)
            return 0.0f;
        return std::clamp(((px - ax) * dx + (py - ay) * dy) / length2, 0.0f, 1.0f);
    }

    // Swept circular walking collision against a wall whose lower and upper
    // edges may slope. The wall endpoints come directly from semantic wall
    // geometry; evaluate vertical overlap at the actual XY contact point rather
    // than expanding a sloped wall into one conservative rectangular prism.
    [[nodiscard]] inline bool FirstPersonSlopedWallIntersectsWalkStep(
        FirstPersonVec3 from, FirstPersonVec3 to,
        FirstPersonVec3 wallA, FirstPersonVec3 wallB,
        float wallHeight, float eyeHeight = 20.0f, float radius = 2.0f)
    {
        if (wallHeight <= 0.0f)
            return false;

        const float rad2 = radius * radius;
        const float wallX = wallB.x - wallA.x;
        const float wallY = wallB.y - wallA.y;
        const float stepX = to.x - from.x;
        const float stepY = to.y - from.y;

        const float fromDist = FirstPersonPointSegmentDistance2(
            from.x, from.y, wallA.x, wallA.y, wallB.x, wallB.y);
        const float toDist = FirstPersonPointSegmentDistance2(
            to.x, to.y, wallA.x, wallA.y, wallB.x, wallB.y);
        const float crossFrom = wallX * (from.y - wallA.y) - wallY * (from.x - wallA.x);
        const float crossTo = wallX * (to.y - wallA.y) - wallY * (to.x - wallA.x);

        // Let someone already overlapping a wall move away from it instead of
        // becoming trapped by a map edit or mode transition.
        if (fromDist <= rad2 && toDist > fromDist && crossFrom * crossTo >= 0.0f)
            return false;

        const auto verticalOverlap = [&](float stepT, float wallT) {
            const float walkBase = from.z + (to.z - from.z) * stepT;
            const float wallBase = wallA.z + (wallB.z - wallA.z) * wallT;
            return walkBase + eyeHeight > wallBase && walkBase < wallBase + wallHeight;
        };
        const auto distance2 = [](float ax, float ay, float bx, float by) {
            const float dx = ax - bx;
            const float dy = ay - by;
            return dx * dx + dy * dy;
        };

        // Exact finite-segment crossing.
        const float det = stepX * wallY - stepY * wallX;
        if (std::abs(det) > 1e-7f)
        {
            const float stepT = ((wallA.x - from.x) * wallY - (wallA.y - from.y) * wallX) / det;
            const float wallT = ((wallA.x - from.x) * stepY - (wallA.y - from.y) * stepX) / det;
            if (stepT >= 0.0f && stepT <= 1.0f && wallT >= 0.0f && wallT <= 1.0f
                && verticalOverlap(stepT, wallT))
                return true;
        }

        // If the finite segments do not cross, their closest pair contains at
        // least one endpoint. Check all four endpoint-to-segment candidates and
        // evaluate the sloped wall height at the corresponding parameter.
        const float wallFromT = FirstPersonPointSegmentParameter2(
            from.x, from.y, wallA.x, wallA.y, wallB.x, wallB.y);
        const float wallFromX = wallA.x + wallX * wallFromT;
        const float wallFromY = wallA.y + wallY * wallFromT;
        if (distance2(from.x, from.y, wallFromX, wallFromY) <= rad2
            && verticalOverlap(0.0f, wallFromT))
            return true;

        const float wallToT = FirstPersonPointSegmentParameter2(
            to.x, to.y, wallA.x, wallA.y, wallB.x, wallB.y);
        const float wallToX = wallA.x + wallX * wallToT;
        const float wallToY = wallA.y + wallY * wallToT;
        if (distance2(to.x, to.y, wallToX, wallToY) <= rad2
            && verticalOverlap(1.0f, wallToT))
            return true;

        const float stepAT = FirstPersonPointSegmentParameter2(
            wallA.x, wallA.y, from.x, from.y, to.x, to.y);
        const float stepAX = from.x + stepX * stepAT;
        const float stepAY = from.y + stepY * stepAT;
        if (distance2(wallA.x, wallA.y, stepAX, stepAY) <= rad2
            && verticalOverlap(stepAT, 0.0f))
            return true;

        const float stepBT = FirstPersonPointSegmentParameter2(
            wallB.x, wallB.y, from.x, from.y, to.x, to.y);
        const float stepBX = from.x + stepX * stepBT;
        const float stepBY = from.y + stepY * stepBT;
        return distance2(wallB.x, wallB.y, stepBX, stepBY) <= rad2
            && verticalOverlap(stepBT, 1.0f);
    }

    [[nodiscard]] inline bool FirstPersonWallIntersectsWalkStep(
        FirstPersonVec3 from, FirstPersonVec3 to,
        FirstPersonVec3 wallA, FirstPersonVec3 wallB,
        float wallTop, float eyeHeight = 20.0f, float radius = 2.0f)
    {
        // Compatibility wrapper for flat-bottom callers/tests.
        wallB.z = wallA.z;
        return FirstPersonSlopedWallIntersectsWalkStep(
            from, to, wallA, wallB, wallTop - wallA.z, eyeHeight, radius);
    }
    // Swept walking collision against an authoritative axis-aligned occupancy
    // prism. Large-scenery object data exposes occupied quarter-tiles and an
    // actual clearance height, so unlike sprite sorting bounds these boxes are
    // safe to use as conservative physical blockers.
    [[nodiscard]] inline bool FirstPersonSegmentIntersectsRect(
        FirstPersonVec3 from, FirstPersonVec3 to,
        float minX, float minY, float maxX, float maxY)
    {
        float enter = 0.0f;
        float leave = 1.0f;
        auto clipAxis = [&](float start, float delta, float low, float high) {
            if (std::abs(delta) <= 1e-7f)
                return start >= low && start <= high;
            float a = (low - start) / delta;
            float b = (high - start) / delta;
            if (a > b) std::swap(a, b);
            enter = std::max(enter, a);
            leave = std::min(leave, b);
            return enter <= leave;
        };
        return clipAxis(from.x, to.x - from.x, minX, maxX)
            && clipAxis(from.y, to.y - from.y, minY, maxY)
            && leave >= 0.0f && enter <= 1.0f;
    }
    [[nodiscard]] inline bool FirstPersonBoxIntersectsWalkStep(
        FirstPersonVec3 from, FirstPersonVec3 to,
        FirstPersonVec3 lowCorner, FirstPersonVec3 highCorner,
        float eyeHeight = 20.0f, float radius = 2.0f)
    {
        const float walkLow = std::min(from.z, to.z);
        const float walkHigh = std::max(from.z, to.z) + eyeHeight;
        if (walkHigh <= lowCorner.z || walkLow >= highCorner.z)
            return false;

        const float minX = std::min(lowCorner.x, highCorner.x) - radius;
        const float maxX = std::max(lowCorner.x, highCorner.x) + radius;
        const float minY = std::min(lowCorner.y, highCorner.y) - radius;
        const float maxY = std::max(lowCorner.y, highCorner.y) + radius;
        const auto inside = [&](FirstPersonVec3 p) {
            return p.x >= minX && p.x <= maxX && p.y >= minY && p.y <= maxY;
        };
        const bool fromInside = inside(from);
        const bool toInside = inside(to);
        if (fromInside)
        {
            // Map edits or a mode transition can leave the eye already inside a
            // conservative volume. Never trap it: allow motion toward the
            // nearest boundary, including a complete step out of the volume.
            if (!toInside)
                return false;
            const auto penetration = [&](FirstPersonVec3 p) {
                return std::min({ p.x - minX, maxX - p.x, p.y - minY, maxY - p.y });
            };
            if (penetration(to) + 1e-4f < penetration(from))
                return false;
            return true;
        }
        if (toInside)
            return true;
        return FirstPersonSegmentIntersectsRect(from, to, minX, minY, maxX, maxY);
    }

    // RCT2's 8-bit spinning carriage orientation is one full turn / 256 steps.
    inline float SpinSpriteYawRadians(uint8_t spinSprite)
    {
        constexpr float kTwoPi = 6.28318530717958647692f;
        return float(spinSprite) * (kTwoPi / 256.0f);
    }
    // Head motion is relative to the car's right/up axes, even upside down.
    inline FirstPersonBasis GetPassengerHeadBasis(const FirstPersonBasis& car, float yaw, float pitch)
    {
        const float cy = std::cos(yaw), sy = std::sin(yaw);
        const float cp = std::cos(pitch), sp = std::sin(pitch);
        const FirstPersonVec3 right{
            car.right.x*cy - car.forward.x*sy,
            car.right.y*cy - car.forward.y*sy,
            car.right.z*cy - car.forward.z*sy,
        };
        const FirstPersonVec3 facing{
            car.forward.x*cy + car.right.x*sy,
            car.forward.y*cy + car.right.y*sy,
            car.forward.z*cy + car.right.z*sy,
        };
        const FirstPersonVec3 forward{
            facing.x*cp + car.up.x*sp,
            facing.y*cp + car.up.y*sp,
            facing.z*cp + car.up.z*sp,
        };
        return { forward, right, FpCross(forward, right) };
    }
    // Conservative test against the six perspective view planes. Treat the bounds
    // as a sphere, so a tall structure remains visible even when its base is
    // outside the picture. Unlike flat horizontal tile-cones this works for
    // steep looks, full inversions and camera roll.
    inline bool FirstPersonSphereVisible(
        const FirstPersonCamera& camera, FirstPersonVec3 center, float radius,
        float horizontalFovDegrees, float aspect, float nearClip, float farClip)
    {
        const auto basis = GetFirstPersonBasis(camera);
        const FirstPersonVec3 d{
            center.x - camera.position.x, center.y - camera.position.y, center.z - camera.position.z };
        const float x = FpDot(d, basis.right);
        const float y = FpDot(d, basis.up);
        const float z = FpDot(d, basis.forward);
        const float r = std::max(0.0f, radius);
        if (z + r < nearClip || z - r > farClip) return false;
        constexpr float kPi = 3.14159265358979323846f;
        const float h = std::tan(
            std::clamp(horizontalFovDegrees, 30.0f, 120.0f) * kPi / 360.0f);
        const float v = h / std::max(0.01f, aspect);
        // Point-plane distances scaled by the plane's normal length. The sphere
        // may straddle a plane and is then intentionally retained.
        return std::abs(x) <= z*h + r*std::sqrt(1.0f+h*h)
            && std::abs(y) <= z*v + r*std::sqrt(1.0f+v*v);
    }
    inline std::optional<FirstPersonProjection> ProjectFirstPersonMath(
        const FirstPersonCamera& c, const FirstPersonVec3& p,
        int32_t width, int32_t height, float fovDegrees, float nearClip)
    {
        if (width <= 0 || height <= 0) return std::nullopt;
        const auto b = GetFirstPersonBasis(c);
        const FirstPersonVec3 d{ p.x-c.position.x, p.y-c.position.y, p.z-c.position.z };
        const float depth = FpDot(d, b.forward);
        if (depth <= nearClip) return std::nullopt;
        constexpr float kPi = 3.14159265358979323846f;
        const float focal = float(width) / (2.0f * std::tan(
            std::clamp(fovDegrees, 30.0f, 120.0f) * kPi / 360.0f));
        return FirstPersonProjection{
            width*0.5f + focal * FpDot(d, b.right) / depth,
            height*0.5f - focal * FpDot(d, b.up) / depth,
            depth, focal / depth,
        };
    }
} // namespace OpenRCT2::Paint

