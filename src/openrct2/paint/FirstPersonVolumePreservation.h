/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <utility>
#include <vector>

namespace OpenRCT2::Paint
{
    struct FirstPersonVolumePreservationCertificate
    {
        bool measured = false;
        size_t falselyFilledEmptyCells = 0;
        size_t lostOccupiedCells = 0;
        size_t ambiguousColumns = 0;
        size_t nonManifoldOrOpenEdges = 0;

        [[nodiscard]] bool retainsAllCellCentres() const
        {
            return measured && falselyFilledEmptyCells == 0
                && lostOccupiedCells == 0 && ambiguousColumns == 0
                && nonManifoldOrOpenEdges == 0;
        }
    };

    // Geometry-only certificate for a *closed* visual candidate. Compare the
    // enclosed volume with both solid and empty original occupancy cells.
    // The four points of each face follow EmitQuad's real triangle diagonal.
    // A failed/budget-limited certificate cannot authorize simplification.
    // This does not independently establish source sprite accuracy or real
    // architectural openings absent from the initial occupancy.
    template<typename Hull, typename FaceRange>
    [[nodiscard]] inline FirstPersonVolumePreservationCertificate
        CertifyFirstPersonCandidateVolume(
            const Hull& hull, const FaceRange& faces,
            size_t maximumFaceColumnPairs = 2000000)
    {
        FirstPersonVolumePreservationCertificate answer{};
        const size_t nx = hull.sizeForward, ny = hull.sizeRight;
        const size_t nz = hull.sizeUp;
        if (!hull.valid || !(hull.step > 0.0f) || !std::isfinite(hull.step)
            || nx == 0 || ny == 0 || nz == 0 || faces.empty()
            || nx * ny > maximumFaceColumnPairs / faces.size())
            return answer;

        // A parity probe alone cannot detect a missing VERTICAL wall.
        // Require every boundary edge to occur exactly twice with opposite
        // winding. Planar candidates are expected to have welded face edges.
        using VertexKey = std::array<int64_t, 3>;
        using EdgeKey = std::pair<VertexKey, VertexKey>;
        std::map<EdgeKey, std::pair<size_t, int32_t>> edgeCounts;
        const auto vertexKey = [](const auto& p) -> VertexKey {
            return { std::llround(double(p.x) * 4096.0),
                     std::llround(double(p.y) * 4096.0),
                     std::llround(double(p.z) * 4096.0) };
        };
        for (const auto& face : faces)
        {
            for (const auto& point : face.corners)
            {
                if (!std::isfinite(point.x) || !std::isfinite(point.y)
                    || !std::isfinite(point.z)
                    || std::abs(point.x) > 1000000.0
                    || std::abs(point.y) > 1000000.0
                    || std::abs(point.z) > 1000000.0)
                    return answer;
            }
            for (size_t i = 0; i < face.corners.size(); ++i)
            {
                const auto a = vertexKey(face.corners[i]);
                const auto b = vertexKey(face.corners[(i + 1) % 4]);
                if (a == b)
                    continue; // Degenerate quad encoding a triangle.
                const bool forward = a < b;
                auto& count = edgeCounts[forward ? EdgeKey{ a, b } : EdgeKey{ b, a }];
                ++count.first;
                count.second += forward ? 1 : -1;
            }
        }
        answer.measured = true;
        for (const auto& [edge, count] : edgeCounts)
            answer.nonManifoldOrOpenEdges +=
                count.first != 2 || count.second != 0;
        if (edgeCounts.empty() || answer.nonManifoldOrOpenEdges != 0)
            return answer;

        const double epsilon = double(hull.step) * 0.013;
        const double duplicateTolerance = double(hull.step) * 0.0001;
        std::vector<double> hits;
        hits.reserve(faces.size() * 2);
        const auto intersect = [&](const auto& a, const auto& b,
                                   const auto& c, double x, double y) {
            const double abx = double(b.x) - double(a.x);
            const double aby = double(b.y) - double(a.y);
            const double acx = double(c.x) - double(a.x);
            const double acy = double(c.y) - double(a.y);
            const double determinant = abx * acy - aby * acx;
            if (std::abs(determinant) < 1.0e-10)
                return;
            const double px = x - double(a.x), py = y - double(a.y);
            const double u = (px * acy - py * acx) / determinant;
            const double v = (abx * py - aby * px) / determinant;
            if (u < -1.0e-8 || v < -1.0e-8 || u + v > 1.0 + 1.0e-8)
                return;
            const double z = double(a.z)
                + u * (double(b.z) - double(a.z))
                + v * (double(c.z) - double(a.z));
            if (std::isfinite(z))
                hits.push_back(z);
        };
        for (size_t y = 0; y < ny; ++y)
        for (size_t x = 0; x < nx; ++x)
        {
            hits.clear();
            const double px = double(hull.minForward)
                + (double(x) + 0.5) * double(hull.step) + epsilon;
            const double py = double(hull.minRight)
                + (double(y) + 0.5) * double(hull.step) - epsilon;
            for (const auto& face : faces)
            {
                intersect(face.corners[0], face.corners[1],
                          face.corners[2], px, py);
                intersect(face.corners[0], face.corners[2],
                          face.corners[3], px, py);
            }
            std::sort(hits.begin(), hits.end());
            size_t unique = 0;
            for (size_t i = 0; i < hits.size(); ++i)
            {
                if (unique == 0
                    || hits[i] - hits[unique - 1] > duplicateTolerance)
                    hits[unique++] = hits[i];
            }
            hits.resize(unique);
            if ((hits.size() & 1) != 0)
            {
                ++answer.ambiguousColumns;
                continue;
            }
            for (size_t z = 0; z < nz; ++z)
            {
                const double probeZ = double(hull.minUp)
                    + (double(z) + 0.5) * double(hull.step);
                const bool inside =
                    (size_t(std::upper_bound(hits.begin(), hits.end(), probeZ)
                        - hits.begin()) & 1) != 0;
                const bool occupied = hull.contains(
                    int32_t(x), int32_t(y), int32_t(z));
                answer.falselyFilledEmptyCells += inside && !occupied;
                answer.lostOccupiedCells += occupied && !inside;
            }
        }
        return answer;
    }
} // namespace OpenRCT2::Paint
