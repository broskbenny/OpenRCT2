/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonVisualHull.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace OpenRCT2::Paint
{
    // An optional visual-only replacement for voxel roofs with a provably
    // contiguous rectangular occupancy support. The mesh has six faces for
    // a planar top or nine for a single straight ridge, with shared edges.
    // Its silhouette and source material are independently certified by the
    // caller before the existing reconstruction can be replaced.
    //
    // No attempt is made for cavities, disconnected columns, missing footprint
    // cells or complex non-heightfield scenery. Those remain on the general
    // reconstruction path rather than being turned into solid boxes.
    [[nodiscard]] inline std::vector<FirstPersonVisualHullFace>
        BuildFirstPersonPlanarSolidCandidate(const FirstPersonVisualHull& hull)
    {
        if (!hull.valid || hull.occupied.empty()
            || !(hull.step > 0.0f) || !std::isfinite(hull.step))
            return {};
        const int nx = hull.sizeForward, ny = hull.sizeRight, nz = hull.sizeUp;
        if (nx < 2 || ny < 2 || nz < 1 || size_t(nx) * size_t(ny) > 4096)
            return {};

        std::vector<double> tops(size_t(nx) * size_t(ny));
        const auto topAt = [&](int x, int y) -> double& {
            return tops[size_t(y) * size_t(nx) + size_t(x)];
        };
        for (int y = 0; y < ny; ++y)
        for (int x = 0; x < nx; ++x)
        {
            int top = -1;
            for (int z = 0; z < nz; ++z)
            {
                if (!hull.contains(x, y, z))
                    continue;
                if (top != z - 1)
                    return {}; // a cavity or separated layer, not a solid roof
                top = z;
            }
            if (top < 0)
                return {}; // a footprint opening cannot be inferred as a wall
            topAt(x, y) = double(top + 1) * double(hull.step);
        }

        struct Profile
        {
            bool valid = false;
            bool ridgeAlongX = false;
            int split = 0;
            double error = std::numeric_limits<double>::infinity();
            double a{}, b{}, c{}; // z = a*x + b*y + c, local world coordinates
            double nearHeight{}, ridgeHeight{}, farHeight{};
        };

        const double step = hull.step;
        Profile selected{};
        // The voxel ceiling quantises every column to the next whole cell.
        // A true continuous plane should fit within one cell of that ceiling.
        const double mx = double(nx) * step * 0.5;
        const double my = double(ny) * step * 0.5;
        double meanZ = 0.0, xx = 0.0, yy = 0.0, xz = 0.0, yz = 0.0;
        for (int y = 0; y < ny; ++y)
        for (int x = 0; x < nx; ++x)
        {
            const double dx = (double(x) + 0.5) * step - mx;
            const double dy = (double(y) + 0.5) * step - my;
            const double z = topAt(x, y);
            meanZ += z;
            xx += dx * dx;
            yy += dy * dy;
            xz += dx * z;
            yz += dy * z;
        }
        meanZ /= double(nx * ny);
        if (xx > 0.0 && yy > 0.0)
        {
            Profile plane{};
            plane.valid = true;
            plane.a = xz / xx;
            plane.b = yz / yy;
            plane.c = meanZ - plane.a * mx - plane.b * my;
            plane.error = 0.0;
            double largest = 0.0;
            for (int y = 0; y < ny; ++y)
            for (int x = 0; x < nx; ++x)
            {
                const double px = (double(x) + 0.5) * step;
                const double py = (double(y) + 0.5) * step;
                const double error = plane.a * px + plane.b * py + plane.c - topAt(x, y);
                largest = std::max(largest, std::abs(error));
                plane.error += error * error;
            }
            // Reject staircases, ornaments and complex roofs rather than
            // blindly approximating them with an attractive but false plane.
            if (largest <= step * 0.80)
                selected = plane;
        }

        // Search a straight ridge in either horizontal direction. The model
        // has one shared ridge height and two independently fitted slopes.
        // Unlike a generic plane fit, a roof peak is allowed to be nonconvex
        // in the voxel-ceiling observations without inventing extra faces.
        for (int axis = 0; axis < 2; ++axis)
        {
            const int count = axis == 0 ? nx : ny;
            if (count < 4)
                continue;
            std::vector<double> averages(size_t(count), 0.0);
            for (int y = 0; y < ny; ++y)
            for (int x = 0; x < nx; ++x)
                averages[size_t(axis == 0 ? x : y)] += topAt(x, y);
            for (double& v : averages)
                v /= double(axis == 0 ? ny : nx);

            for (int split = 2; split <= count - 2; ++split)
            {
                const double ridge = double(split) * step;
                const auto regress = [&](int begin, int end) {
                    double n = 0.0, sumX = 0.0, sumZ = 0.0;
                    double sumXX = 0.0, sumXZ = 0.0;
                    for (int k = begin; k < end; ++k)
                    {
                        const double p = (double(k) + 0.5) * step;
                        const double z = averages[size_t(k)];
                        n += 1.0;
                        sumX += p;
                        sumZ += z;
                        sumXX += p * p;
                        sumXZ += p * z;
                    }
                    const double divisor = n * sumXX - sumX * sumX;
                    const double slope = divisor > 0.0
                        ? (n * sumXZ - sumX * sumZ) / divisor : 0.0;
                    return std::array<double, 2>{ slope, (sumZ - slope * sumX) / n };
                };
                const auto left = regress(0, split);
                const auto right = regress(split, count);
                // A real ridge rises on one side and falls on the other.
                if (left[0] < step * 0.01 || right[0] > -step * 0.01)
                    continue;
                const double leftRidge = left[0] * ridge + left[1];
                const double rightRidge = right[0] * ridge + right[1];
                if (std::abs(leftRidge - rightRidge) > step * 1.5)
                    continue;
                const double peak = (leftRidge + rightRidge) * 0.5;
                const double low = left[1] + (peak - leftRidge);
                const double high = right[0] * double(count) * step
                    + right[1] + (peak - rightRidge);
                if (peak - std::max(low, high) < step * 0.50)
                    continue;

                Profile profile{};
                profile.valid = true;
                profile.error = 0.0;
                profile.ridgeAlongX = axis == 0;
                profile.split = split;
                profile.nearHeight = low;
                profile.ridgeHeight = peak;
                profile.farHeight = high;
                double largest = 0.0;
                for (int y = 0; y < ny; ++y)
                for (int x = 0; x < nx; ++x)
                {
                    const double p = (double(axis == 0 ? x : y) + 0.5) * step;
                    const double predicted = p <= ridge
                        ? low + (peak - low) * p / ridge
                        : peak + (high - peak) * (p - ridge)
                            / (double(count) * step - ridge);
                    const double error = predicted - topAt(x, y);
                    largest = std::max(largest, std::abs(error));
                    profile.error += error * error;
                }
                if (largest <= step * 0.85
                    && (!selected.valid || profile.error + step * step * 0.1 < selected.error))
                    selected = profile;
            }
        }
        if (!selected.valid)
            return {};

        const double x0 = hull.minForward;
        const double y0 = hull.minRight;
        const double x1 = x0 + double(nx) * step;
        const double y1 = y0 + double(ny) * step;
        const double base = hull.minUp;
        const double maxZ = base + double(nz) * step;
        const auto height = [&](double x, double y) {
            if (selected.split == 0)
                return selected.a * (x - x0) + selected.b * (y - y0)
                    + selected.c + base;
            const double position = selected.ridgeAlongX ? x - x0 : y - y0;
            const double total = double(selected.ridgeAlongX ? nx : ny) * step;
            const double ridge = double(selected.split) * step;
            return base + (position <= ridge
                ? selected.nearHeight + (selected.ridgeHeight - selected.nearHeight)
                    * position / ridge
                : selected.ridgeHeight + (selected.farHeight - selected.ridgeHeight)
                    * (position - ridge) / (total - ridge));
        };
        // Boundary vertices must be inside the original occupancy envelope.
        for (double x : { x0, x1 })
        for (double y : { y0, y1 })
        {
            const double z = height(x, y);
            if (!std::isfinite(z) || z < base + 0.1 * step
                || z > maxZ + step * 0.1)
                return {};
        }

        std::vector<FirstPersonVisualHullFace> result;
        result.reserve(selected.split == 0 ? 6 : 9);
        const auto point = [&](double x, double y, double z) {
            return FirstPersonVec3{ float(x), float(y), float(z) };
        };
        const auto add = [&](FirstPersonVisualHullFaceKind kind,
            std::array<FirstPersonVec3, 4> corners) {
            const FirstPersonVec3 a{
                corners[1].x - corners[0].x,
                corners[1].y - corners[0].y,
                corners[1].z - corners[0].z
            };
            const FirstPersonVec3 b{
                corners[2].x - corners[0].x,
                corners[2].y - corners[0].y,
                corners[2].z - corners[0].z
            };
            auto normal = FpCross(a, b);
            const float sq = FpDot(normal, normal);
            if (!(sq > 1.0e-8f) || !std::isfinite(sq))
                return false;
            normal.x /= std::sqrt(sq);
            normal.y /= std::sqrt(sq);
            normal.z /= std::sqrt(sq);
            result.push_back({corners, normal, kind});
            return true;
        };
        const auto addTop = [&](double ax, double ay, double bx, double by) {
            return add(FirstPersonVisualHullFaceKind::top, { {
                point(ax, ay, height(ax, ay)), point(bx, ay, height(bx, ay)),
                point(bx, by, height(bx, by)), point(ax, by, height(ax, by))
            } });
        };
        const auto addYWall = [&](double ax, double bx, double y, bool positive) {
            const auto a = point(ax, y, base);
            const auto b = point(bx, y, base);
            const auto c = point(bx, y, height(bx, y));
            const auto d = point(ax, y, height(ax, y));
            return positive
                ? add(FirstPersonVisualHullFaceKind::maxRight, {{b, a, d, c}})
                : add(FirstPersonVisualHullFaceKind::minRight, {{a, b, c, d}});
        };
        const auto addXWall = [&](double x, double ay, double by, bool positive) {
            const auto a = point(x, ay, base);
            const auto b = point(x, by, base);
            const auto c = point(x, by, height(x, by));
            const auto d = point(x, ay, height(x, ay));
            return positive
                ? add(FirstPersonVisualHullFaceKind::maxForward, {{a, b, c, d}})
                : add(FirstPersonVisualHullFaceKind::minForward, {{b, a, d, c}});
        };
        const double ridge = double(selected.split) * step;
        const double midX = selected.split != 0 && selected.ridgeAlongX ? x0 + ridge : x1;
        const double midY = selected.split != 0 && !selected.ridgeAlongX ? y0 + ridge : y1;
        if (!addTop(x0, y0, midX, midY)
            || (selected.split != 0 && !(selected.ridgeAlongX
                ? addTop(midX, y0, x1, y1)
                : addTop(x0, midY, x1, y1))))
            return {};
        if (!(selected.split != 0 && selected.ridgeAlongX
                ? (addYWall(x0, midX, y0, false) && addYWall(midX, x1, y0, false)
                    && addYWall(x0, midX, y1, true) && addYWall(midX, x1, y1, true))
                : (addYWall(x0, x1, y0, false) && addYWall(x0, x1, y1, true))))
            return {};
        if (!(selected.split != 0 && !selected.ridgeAlongX
                ? (addXWall(x0, y0, midY, false) && addXWall(x0, midY, y1, false)
                    && addXWall(x1, y0, midY, true) && addXWall(x1, midY, y1, true))
                : (addXWall(x0, y0, y1, false) && addXWall(x1, y0, y1, true))))
            return {};
        if (!add(FirstPersonVisualHullFaceKind::bottom, { {
                point(x0, y1, base), point(x1, y1, base),
                point(x1, y0, base), point(x0, y0, base)
            } }))
            return {};
        return result;
    }
} // namespace OpenRCT2::Paint
