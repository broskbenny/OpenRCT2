/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace OpenRCT2::Paint
{
    // Preserve source order within a region, without scanning unrelated faces.
    using FirstPersonRegionSurfaceIndex =
        std::unordered_map<uint64_t, std::vector<size_t>>;

    template<typename Surfaces>
    [[nodiscard]] FirstPersonRegionSurfaceIndex
        IndexFirstPersonSurfacesByGpuRegion(const Surfaces& surfaces)
    {
        FirstPersonRegionSurfaceIndex index;
        for (size_t i = 0; i < surfaces.size(); ++i)
        {
            const uint64_t region = surfaces[i].gpuRegion;
            if (region != 0)
                index[region].push_back(i);
        }
        return index;
    }

    // A resumable group-relative offset keeps exact-once source face order.
    template<typename Cache, typename Budget, typename Consume>
    [[nodiscard]] bool ResumeFirstPersonIndexedRegionSurfaces(
        const Cache& cache, uint64_t region, size_t& offset,
        Budget& budget, Consume&& consume)
    {
        const auto found = cache.regionSurfaceIndices.find(region);
        if (found == cache.regionSurfaceIndices.end())
            return true;
        const auto& indices = found->second;
        while (offset < indices.size())
        {
            if (!budget.take())
                return false;
            consume(cache.surfaces[indices[offset]]);
            ++offset;
        }
        return true;
    }
} // namespace OpenRCT2::Paint
