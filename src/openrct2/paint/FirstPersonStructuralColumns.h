/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonVisualHull.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace OpenRCT2::Paint
{
    struct FirstPersonStructuralColumnConfig
    {
        size_t directSupportViews = 4;
        size_t intervalSupportViews = 3;
        size_t bridgeSupportViews = 2;
        size_t hardNegativeMaximumViews = 1;
        int32_t hardNegativeRunCells = 3;
        int32_t maximumFacadeNotchCells = 2;
        bool facadeAxisIsForward = false;
    };

    struct FirstPersonStructuralColumnResult
    {
        FirstPersonVisualHull hull{};
        size_t addedCells = 0;
        size_t removedCells = 0;
        size_t regularizedColumns = 0;
    };

    template<typename AllowedCell, typename SupportCount>
    [[nodiscard]] FirstPersonStructuralColumnResult
        RegularizeFirstPersonStructuralColumns(
            const FirstPersonVisualHull& source,
            const FirstPersonStructuralColumnConfig& config,
            AllowedCell&& allowedCell,
            SupportCount&& supportCount)
    {
        FirstPersonStructuralColumnResult result{};
        result.hull = source;
        result.hull.refinedFaces.clear();
        result.hull.continuousSurfaceRefined = false;
        if (!source.valid || source.occupied.empty()
            || source.sizeForward == 0
            || source.sizeRight == 0
            || source.sizeUp == 0)
            return result;

        const size_t cellCount = source.occupied.size();
        std::vector<uint8_t> allowed(cellCount, 0);
        std::vector<uint8_t> support(cellCount, 0);
        std::vector<uint8_t> barrier(cellCount, 0);
        result.hull.occupied.assign(cellCount, 0);

        const auto indexOf =
            [&](int32_t forward, int32_t right, int32_t up) {
                return (size_t(up) * size_t(source.sizeRight)
                           + size_t(right))
                    * size_t(source.sizeForward)
                    + size_t(forward);
            };
        const auto validHorizontal =
            [&](int32_t forward, int32_t right) {
                return forward >= 0 && right >= 0
                    && forward < source.sizeForward
                    && right < source.sizeRight;
            };

        for (int32_t up = 0; up < source.sizeUp; ++up)
        for (int32_t right = 0; right < source.sizeRight; ++right)
        for (int32_t forward = 0; forward < source.sizeForward; ++forward)
        {
            const size_t index = indexOf(forward, right, up);
            const auto point = source.centre(forward, right, up);
            if (!allowedCell(point))
                continue;
            allowed[index] = 1;
            support[index] = uint8_t(std::min<size_t>(
                255, supportCount(point)));
        }

        for (int32_t right = 0; right < source.sizeRight; ++right)
        for (int32_t forward = 0; forward < source.sizeForward; ++forward)
        {
            int32_t up = 0;
            while (up < source.sizeUp)
            {
                const size_t startIndex =
                    indexOf(forward, right, up);
                if (allowed[startIndex] == 0
                    || support[startIndex]
                        > config.hardNegativeMaximumViews)
                {
                    ++up;
                    continue;
                }
                const int32_t start = up;
                while (up < source.sizeUp)
                {
                    const size_t index =
                        indexOf(forward, right, up);
                    if (allowed[index] == 0
                        || support[index]
                            > config.hardNegativeMaximumViews)
                        break;
                    ++up;
                }
                if (up - start
                    < config.hardNegativeRunCells)
                    continue;
                for (int32_t u = start; u < up; ++u)
                    barrier[indexOf(forward, right, u)] = 1;
            }
        }

        for (int32_t right = 0; right < source.sizeRight; ++right)
        for (int32_t forward = 0; forward < source.sizeForward; ++forward)
        {
            bool changed = false;
            int32_t up = 0;
            while (up < source.sizeUp)
            {
                while (up < source.sizeUp)
                {
                    const size_t index =
                        indexOf(forward, right, up);
                    if (allowed[index] != 0
                        && barrier[index] == 0)
                        break;
                    ++up;
                }
                if (up >= source.sizeUp)
                    break;
                const int32_t runStart = up;
                while (up < source.sizeUp)
                {
                    const size_t index =
                        indexOf(forward, right, up);
                    if (allowed[index] == 0
                        || barrier[index] != 0)
                        break;
                    ++up;
                }
                const int32_t runEnd = up;

                bool anchored = false;
                int32_t low = runEnd;
                int32_t high = runStart - 1;
                for (int32_t u = runStart; u < runEnd; ++u)
                {
                    const size_t index =
                        indexOf(forward, right, u);
                    if (support[index]
                        >= config.directSupportViews)
                        anchored = true;
                    if (support[index]
                        >= config.intervalSupportViews)
                    {
                        low = std::min(low, u);
                        high = std::max(high, u);
                    }
                }
                if (!anchored || high < low)
                    continue;

                for (int32_t u = low; u <= high; ++u)
                {
                    const size_t index =
                        indexOf(forward, right, u);
                    if (result.hull.occupied[index] == 0)
                    {
                        result.hull.occupied[index] = 1;
                        changed = true;
                    }
                }
            }
            if (changed)
                ++result.regularizedColumns;
        }

        const auto interval =
            [&](int32_t forward, int32_t right) {
                std::array<int32_t, 2> value{
                    int32_t(source.sizeUp), -1
                };
                if (!validHorizontal(forward, right))
                    return value;
                for (int32_t up = 0; up < source.sizeUp; ++up)
                {
                    if (result.hull.occupied[
                            indexOf(forward, right, up)]
                        == 0)
                        continue;
                    value[0] = std::min(value[0], up);
                    value[1] = std::max(value[1], up);
                }
                return value;
            };
        const auto facadeNeighbour =
            [&](int32_t forward, int32_t right, int32_t delta) {
                return config.facadeAxisIsForward
                    ? std::array<int32_t, 2>{
                          forward + delta, right
                      }
                    : std::array<int32_t, 2>{
                          forward, right + delta
                      };
            };

        auto coherent = result.hull.occupied;
        for (int32_t right = 0; right < source.sizeRight; ++right)
        for (int32_t forward = 0; forward < source.sizeForward; ++forward)
        {
            const auto prevPos =
                facadeNeighbour(forward, right, -1);
            const auto nextPos =
                facadeNeighbour(forward, right, 1);
            if (!validHorizontal(prevPos[0], prevPos[1])
                || !validHorizontal(nextPos[0], nextPos[1]))
                continue;

            const auto previous =
                interval(prevPos[0], prevPos[1]);
            const auto current =
                interval(forward, right);
            const auto next =
                interval(nextPos[0], nextPos[1]);
            if (previous[1] < previous[0]
                || next[1] < next[0])
                continue;

            const int32_t neighbourLow =
                std::max(previous[0], next[0]);
            const int32_t neighbourHigh =
                std::min(previous[1], next[1]);
            if (neighbourHigh < neighbourLow
                || std::abs(previous[0] - next[0])
                    > config.maximumFacadeNotchCells
                || std::abs(previous[1] - next[1])
                    > config.maximumFacadeNotchCells)
                continue;

            int32_t fillLow = neighbourLow;
            int32_t fillHigh = neighbourHigh;
            if (current[1] >= current[0])
            {
                fillLow = std::min(fillLow, current[0]);
                fillHigh = std::max(fillHigh, current[1]);
            }

            bool touched = false;
            for (int32_t up = fillLow; up <= fillHigh; ++up)
            {
                const size_t index =
                    indexOf(forward, right, up);
                if (allowed[index] == 0
                    || barrier[index] != 0
                    || support[index]
                        < config.bridgeSupportViews)
                    continue;
                if (coherent[index] == 0)
                {
                    coherent[index] = 1;
                    touched = true;
                }
            }
            if (touched)
                ++result.regularizedColumns;
        }
        result.hull.occupied = std::move(coherent);

        auto pruned = result.hull.occupied;
        for (int32_t right = 0; right < source.sizeRight; ++right)
        for (int32_t forward = 0; forward < source.sizeForward; ++forward)
        {
            const auto prevPos =
                facadeNeighbour(forward, right, -1);
            const auto nextPos =
                facadeNeighbour(forward, right, 1);
            if (!validHorizontal(prevPos[0], prevPos[1])
                || !validHorizontal(nextPos[0], nextPos[1]))
                continue;
            const auto previous =
                interval(prevPos[0], prevPos[1]);
            const auto current =
                interval(forward, right);
            const auto next =
                interval(nextPos[0], nextPos[1]);
            if (current[1] < current[0]
                || previous[1] < previous[0]
                || next[1] < next[0])
                continue;

            if (std::abs(previous[0] - next[0]) <= 1
                && current[0]
                    < std::min(previous[0], next[0]))
            {
                const int32_t target =
                    std::min(previous[0], next[0]);
                for (int32_t up = current[0];
                     up < target; ++up)
                {
                    const size_t index =
                        indexOf(forward, right, up);
                    if (support[index]
                        >= config.directSupportViews)
                        break;
                    pruned[index] = 0;
                }
            }
            if (std::abs(previous[1] - next[1]) <= 1
                && current[1]
                    > std::max(previous[1], next[1]))
            {
                const int32_t target =
                    std::max(previous[1], next[1]);
                for (int32_t up = current[1];
                     up > target; --up)
                {
                    const size_t index =
                        indexOf(forward, right, up);
                    if (support[index]
                        >= config.directSupportViews)
                        break;
                    pruned[index] = 0;
                }
            }
        }
        result.hull.occupied = std::move(pruned);

        for (size_t i = 0; i < cellCount; ++i)
        {
            const bool before = source.occupied[i] != 0;
            const bool after = result.hull.occupied[i] != 0;
            if (!before && after)
                ++result.addedCells;
            else if (before && !after)
                ++result.removedCells;
        }
        return result;
    }
} // namespace OpenRCT2::Paint
