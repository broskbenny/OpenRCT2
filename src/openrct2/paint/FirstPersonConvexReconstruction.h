/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonVisualHull.h"

#include <limits>
#include <utility>

namespace OpenRCT2::Paint
{
    struct FirstPersonConvexAttempt
    {
        uint8_t stage = 0;
        size_t faces = 0;
        FirstPersonRoundTripCertificate certificate;
    };
    // Fit compact planar solids before carving voxels. Metadata supplies only
    // a footprint hypothesis: all four native silhouettes must certify it.
    // Openings, disconnected artwork and unsupported shapes retain carving.
    namespace FirstPersonConvexDetail
    {
        using Point = std::array<float, 2>;
        struct Plane { FirstPersonVec3 normal; float distance; };
        inline constexpr float kEpsilon = 1.0e-4f;

        inline FirstPersonVec3 Subtract(FirstPersonVec3 a, FirstPersonVec3 b)
        {
            return { a.x - b.x, a.y - b.y, a.z - b.z };
        }

        inline bool SolidSilhouette(const FirstPersonSilhouette& source)
        {
            const int64_t width = int64_t(source.maxX) - source.minX;
            const int64_t height = int64_t(source.maxY) - source.minY;
            if (source.empty() || source.overflowed || width <= 0 || height <= 0
                || width > 512 || height > 512 || source.size() > 65536)
                return false;
            // Any gap within a row or column disproves the solid convex
            // hypothesis, including one-pixel windows and separate components.
            for (int32_t y = source.minY; y < source.maxY; ++y)
            {
                int32_t lo = source.maxX, hi = source.minX - 1;
                size_t count = 0;
                for (int32_t x = source.minX; x < source.maxX; ++x)
                    if (source.contains(x, y)) { lo = std::min(lo, x); hi = x; ++count; }
                if (count == 0 || count != size_t(hi - lo + 1))
                    return false;
            }
            for (int32_t x = source.minX; x < source.maxX; ++x)
            {
                int32_t lo = source.maxY, hi = source.minY - 1;
                size_t count = 0;
                for (int32_t y = source.minY; y < source.maxY; ++y)
                    if (source.contains(x, y)) { lo = std::min(lo, y); hi = y; ++count; }
                if (count == 0 || count != size_t(hi - lo + 1))
                    return false;
            }
            return true;
        }

        inline std::vector<FirstPersonVisualHullFace> Profile(
            const std::array<FirstPersonVec3, 4>& corners, float base,
            const std::optional<FirstPersonVec3>& peak)
        {
            std::vector<FirstPersonVisualHullFace> faces;
            faces.reserve(9);
            const auto append = [&](FirstPersonVec3 a, FirstPersonVec3 b,
                                    FirstPersonVec3 c, FirstPersonVec3 d) {
                auto normal = FpCross(Subtract(b, a), Subtract(c, a));
                const float length = std::sqrt(FpDot(normal, normal));
                if (length < kEpsilon) return;
                normal = { normal.x / length, normal.y / length, normal.z / length };
                const auto kind = normal.z < -0.999f ? FirstPersonVisualHullFaceKind::bottom
                    : normal.z > 0 ? FirstPersonVisualHullFaceKind::top
                    : std::abs(normal.x) > std::abs(normal.y)
                        ? (normal.x < 0 ? FirstPersonVisualHullFaceKind::minForward
                                        : FirstPersonVisualHullFaceKind::maxForward)
                        : (normal.y < 0 ? FirstPersonVisualHullFaceKind::minRight
                                        : FirstPersonVisualHullFaceKind::maxRight);
                faces.push_back({ { a, b, c, d }, normal, kind });
            };
            auto bottom = corners;
            for (auto& p : bottom) p.z = base;
            append(bottom[3], bottom[2], bottom[1], bottom[0]);
            for (size_t i = 0; i < 4; ++i)
            {
                const size_t j = (i + 1) % 4;
                // Order from the elevated endpoint so a triangular side never
                // starts with two coincident vertices.
                if (corners[j].z > base + kEpsilon)
                    append(bottom[i], bottom[j], corners[j],
                        corners[i].z > base + kEpsilon ? corners[i] : corners[j]);
                else if (corners[i].z > base + kEpsilon)
                    append(corners[i], bottom[i], bottom[j], bottom[j]);
                if (peak) append(corners[i], corners[j], *peak, *peak);
            }
            if (!peak) append(corners[0], corners[1], corners[2], corners[3]);
            return faces;
        }

        struct Columns
        {
            int32_t first = 0;
            std::vector<std::array<int32_t, 2>> spans;
        };

        // Rank a fixed number of hypotheses using convex silhouette column
        // spans. This is O(candidate count * sprite width * face count), with
        // no per-candidate raster allocation or hash table. Admission still
        // uses the independent production four-view raster certificate.
        template<typename ProjectPoint>
        inline uint64_t Error(const std::vector<FirstPersonVisualHullFace>& faces,
            const std::vector<FirstPersonVisualHullView>& views,
            const std::array<Columns, 4>& columns, ProjectPoint&& projectPoint)
        {
            uint64_t error = 0;
            for (size_t v = 0; v < 4; ++v)
            {
                std::array<Point, 36> points{};
                size_t pointIndex = 0;
                for (const auto& face : faces)
                    for (const auto p : face.corners)
                        points[pointIndex++] = projectPoint(views[v].imageDirection, p);
                const auto& source = columns[v];
                for (size_t column = 0; column < source.spans.size(); ++column)
                {
                    const float x = float(source.first + int32_t(column)) + 0.5f;
                    float low = std::numeric_limits<float>::infinity(), high = -low;
                    for (size_t f = 0; f < faces.size(); ++f)
                    for (size_t i = 0; i < 4; ++i)
                    {
                        const auto a = points[f * 4 + i], b = points[f * 4 + (i + 1) % 4];
                        if (x < std::min(a[0], b[0]) || x > std::max(a[0], b[0]) || a[0] == b[0])
                            continue;
                        const float y = a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0]);
                        low = std::min(low, y); high = std::max(high, y);
                    }
                    const auto observed = source.spans[column];
                    if (!std::isfinite(low)) { error += uint64_t(observed[1] - observed[0] + 1); continue; }
                    const int32_t lo = int32_t(std::ceil(low - 0.50001f));
                    const int32_t hi = int32_t(std::floor(high - 0.49999f));
                    const int32_t intersection = std::max(0, std::min(hi, observed[1]) - std::max(lo, observed[0]) + 1);
                    error += uint64_t(std::max(0, hi - lo + 1) + observed[1] - observed[0] + 1 - 2 * intersection);
                }
            }
            return error;
        }
    }

    template<typename ProjectPoint>
    [[nodiscard]] inline FirstPersonVisualHull BuildFirstPersonConvexVisualHull(
        const std::vector<FirstPersonVisualHullView>& views,
        const FirstPersonVisualHullBounds& bounds, const FirstPersonVisualHullConfig& config,
        ProjectPoint&& projectPoint, FirstPersonConvexAttempt* attempt = nullptr)
    {
        using namespace FirstPersonConvexDetail;
        if (attempt) { *attempt = {}; attempt->stage = 1; }
        const std::array<float, 7> dimensions{ bounds.minForward, bounds.maxForward,
            bounds.minRight, bounds.maxRight, bounds.minUp, bounds.maxUp, bounds.step };
        for (const float d : dimensions) if (!std::isfinite(d) || std::abs(d) > 4096) return {};
        if (views.size() != 4 || !(bounds.step > 0)
            || bounds.maxForward <= bounds.minForward || bounds.maxRight <= bounds.minRight
            || bounds.maxUp <= bounds.minUp
            || bounds.maxForward - bounds.minForward > 64 || bounds.maxRight - bounds.minRight > 64)
            return {};
        // Validate before any float-to-integer grid conversion, including
        // arbitrarily small positive steps supplied by a malformed caller.
        for (const float extent : { bounds.maxForward - bounds.minForward,
                 bounds.maxRight - bounds.minRight, bounds.maxUp - bounds.minUp })
            if (extent / bounds.step > float(std::min(config.maximumAxisCells, 255))) return {};
        std::array<Columns, 4> columns;
        std::array<bool, 4> seen{};
        for (size_t i = 0; i < 4; ++i)
        {
            const auto& view = views[i];
            if (view.imageDirection >= 4 || seen[view.imageDirection] || !SolidSilhouette(view.observed)) return {};
            seen[view.imageDirection] = true;
            columns[i].first = view.observed.minX;
            for (int32_t x = view.observed.minX; x < view.observed.maxX; ++x)
            {
                int32_t lo = view.observed.maxY, hi = view.observed.minY;
                for (int32_t y = view.observed.minY; y < view.observed.maxY; ++y)
                    if (view.observed.contains(x, y)) { lo = std::min(lo, y); hi = y; }
                columns[i].spans.push_back({ lo, hi });
            }
        }
        if (attempt) attempt->stage = 2;
        const auto upper = [&](float x, float y) {
            float z = bounds.maxUp;
            for (size_t i = 0; i < 4; ++i)
            {
                const auto p = projectPoint(views[i].imageDirection, FirstPersonVec3{ x, y, bounds.minUp });
                const auto up = projectPoint(views[i].imageDirection, FirstPersonVec3{ x, y, bounds.minUp + 32 });
                if (std::abs(up[0] - p[0]) > kEpsilon || std::abs(up[1] - p[1] + 32) > kEpsilon)
                    return bounds.minUp;
                const auto& source = columns[i];
                const int32_t column = std::clamp(int32_t(std::floor(p[0])) - source.first,
                    0, int32_t(source.spans.size()) - 1);
                z = std::min(z, bounds.minUp + p[1] - float(source.spans[column][0]) - 0.5f);
            }
            return z;
        };
        std::array<FirstPersonVec3, 4> corners{{
            {bounds.minForward,bounds.minRight,0}, {bounds.maxForward,bounds.minRight,0},
            {bounds.maxForward,bounds.maxRight,0}, {bounds.minForward,bounds.maxRight,0} }};
        // Infer the bottom plane from native lower silhouette extrema. The
        // metadata's ground level is only a bound, not a visible base plane.
        std::array<float, 4> bases{};
        for (size_t i = 0; i < 4; ++i)
        {
            float bottom = -std::numeric_limits<float>::infinity();
            for (const auto p : corners)
                bottom = std::max(bottom, projectPoint(views[i].imageDirection,
                    FirstPersonVec3{ p.x, p.y, bounds.minUp })[1]);
            bases[i] = bounds.minUp + bottom - float(views[i].observed.maxY);
        }
        std::sort(bases.begin(), bases.end());
        if (bases.back() - bases.front() > 1) return {};
        const float base = std::clamp(std::round((bases[1] + bases[2]) * 0.5f), bounds.minUp, bounds.maxUp);
        for (auto& p : corners) p.z = upper(p.x, p.y);
        float highest = bounds.minUp, peakX = 0, peakY = 0;
        size_t peakCount = 0;
        // At most 65x65 horizontal samples, never a finer 3D voxel grid.
        for (float y = bounds.minRight; y <= bounds.maxRight; y += 1)
        for (float x = bounds.minForward; x <= bounds.maxForward; x += 1)
        {
            const float z = upper(x, y);
            if (z > highest + kEpsilon) { highest = z; peakX = x; peakY = y; peakCount = 1; }
            else if (std::abs(z - highest) <= kEpsilon) { peakX += x; peakY += y; ++peakCount; }
        }
        if (peakCount == 0) return {};
        peakX = std::round(peakX / float(peakCount)); peakY = std::round(peakY / float(peakCount));
        std::vector<FirstPersonVisualHullFace> best;
        uint64_t bestError = std::numeric_limits<uint64_t>::max();
        const auto consider = [&](const auto& top, const std::optional<FirstPersonVec3>& peak) {
            auto faces = Profile(top, base, peak);
            if (faces.size() < 4) return;
            const uint64_t error = Error(faces, views, columns, projectPoint);
            if (error < bestError || (error == bestError && faces.size() < best.size()))
            { bestError = error; best = std::move(faces); }
        };
        // Quantisation uncertainty is restricted to two world units. These
        // corner-height combinations cover flat and sloped planar solids.
        for (uint32_t mask = 0; mask < 81; ++mask)
        {
            auto top = corners;
            uint32_t digits = mask;
            for (auto& p : top)
            {
                p.z = std::clamp(std::round(p.z) + float(int(digits % 3) - 1), base, bounds.maxUp);
                digits /= 3;
            }
            if (std::abs(top[0].z + top[2].z - top[1].z - top[3].z) > kEpsilon) continue;
            consider(top, std::nullopt);
        }
        // Interior-peak solids have four planar top faces, independent of
        // their height, and are never digital silhouette stair meshes.
        for (int bias = 0; bias <= 1; ++bias)
        {
            auto top = corners;
            for (auto& p : top) p.z = std::clamp(std::floor(p.z) + float(bias), base, bounds.maxUp);
            for (int dy = -2; dy <= 2; ++dy)
            for (int dx = -2; dx <= 2; ++dx)
            for (int dz = -1; dz <= 2; ++dz)
            {
                const FirstPersonVec3 peak{ peakX + float(dx), peakY + float(dy), std::round(highest) + float(dz) };
                if (peak.x <= bounds.minForward || peak.x >= bounds.maxForward
                    || peak.y <= bounds.minRight || peak.y >= bounds.maxRight || peak.z > bounds.maxUp) continue;
                bool above = true;
                for (const auto p : top) above &= peak.z > p.z;
                if (above) consider(top, peak);
            }
        }
        if (best.empty()) return {};
        const auto triangles = TriangulateFirstPersonReconstructionFaces(best);
        const auto certificate = CertifyFirstPersonReconstruction(triangles, views, projectPoint);
        if (attempt) { attempt->stage = 5; attempt->faces = best.size(); attempt->certificate = certificate; }
        if (!certificate.valid || certificate.viewCount != 4
            || certificate.minimumCandidateCoverage < 0.97f || certificate.minimumObservedCoverage < 0.94f
            || certificate.minimumIntersectionOverUnion < 0.92f || certificate.maximumEdgeError > 1) return {};
        // Every face is a supporting plane of the solid. A folded or concave
        // profile is rejected before collision or material extraction.
        std::vector<Plane> planes;
        for (const auto& face : best)
        {
            const Plane plane{ face.normal, FpDot(face.normal, face.corners[0]) };
            for (const auto& other : best)
            for (const auto p : other.corners)
                if (FpDot(plane.normal, p) > plane.distance + kEpsilon) return {};
            planes.push_back(plane);
        }
        std::vector<FirstPersonVisualHullTextureView> textures;
        for (const auto& view : views) textures.push_back({ view.imageDirection, view.image });
        auto result = BuildFirstPersonOccupancyHull(bounds, config, textures, [&](FirstPersonVec3 point) {
            for (const auto& plane : planes)
                if (FpDot(plane.normal, point) > plane.distance + kEpsilon) return false;
            return true;
        });
        if (!result.valid) return {};
        result.refinedFaces = std::move(best);
        result.continuousSurfaceRefined = true;
        result.directConvexSurface = true;
        result.averageIntersectionOverUnion = certificate.averageIntersectionOverUnion;
        result.minimumIntersectionOverUnion = certificate.minimumIntersectionOverUnion;
        result.minimumCandidateCoverage = certificate.minimumCandidateCoverage;
        result.minimumObservedCoverage = certificate.minimumObservedCoverage;
        result.maximumEdgeError = certificate.maximumEdgeError;
        result.roundTripViewCount = certificate.viewCount;
        result.roundTripSymmetricDifference = certificate.symmetricDifferencePixels;
        result.attempt.stage = 6;
        result.attempt.nativeViews = 4;
        result.attempt.occupiedCells = uint32_t(std::count(result.occupied.begin(), result.occupied.end(), uint8_t{ 1 }));
        result.attempt.certificateViews = 4;
        result.attempt.minimumIoU = certificate.minimumIntersectionOverUnion;
        result.attempt.candidateCoverage = certificate.minimumCandidateCoverage;
        result.attempt.observedCoverage = certificate.minimumObservedCoverage;
        result.attempt.maximumEdgeError = certificate.maximumEdgeError;
        result.attempt.disagreementPixels = certificate.symmetricDifferencePixels;
        if (attempt) attempt->stage = 6;
        return result;
    }
}
