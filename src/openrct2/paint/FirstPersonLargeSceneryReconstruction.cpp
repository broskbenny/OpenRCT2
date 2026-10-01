/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "FirstPersonLargeSceneryReconstruction.h"

#include "../object/LargeSceneryEntry.h"

#include <array>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

namespace OpenRCT2::Paint
{
    bool LargeSceneryAssetEligible(
        const LargeSceneryEntry& entry)
    {
        return !entry.tiles.empty()
            && entry.tiles.size() <= 256;
    }

    namespace
    {
        struct LargeSceneryAssetCell
        {
            int32_t qx{};
            int32_t qy{};
            int32_t lowZ{};
            int32_t highZ{};
            uint16_t sequence{};
        };

        static std::unordered_map<
            const LargeSceneryEntry*, LargeSceneryAssetModel>
            _largeSceneryAssetModels;

        [[nodiscard]] std::optional<
            std::vector<LargeSceneryAssetCell>>
            BuildLargeSceneryAssetCells(
                const LargeSceneryEntry& entry)
        {
            static constexpr std::array<CoordsXY, 4>
                kQuarterCellOffsets{ {
                    { 1, 1 }, // SW
                    { 1, 0 }, // NW
                    { 0, 0 }, // NE
                    { 0, 1 }, // SE
                } };

            std::vector<LargeSceneryAssetCell> cells;
            for (size_t sequence = 0;
                 sequence < entry.tiles.size();
                 ++sequence)
            {
                const auto& tile =
                    entry.tiles[sequence];
                if ((tile.offset.x % 16) != 0
                    || (tile.offset.y % 16) != 0
                    || tile.zClearance <= 0
                    || (tile.corners & 0x0Fu) == 0)
                {
                    return std::nullopt;
                }

                const int32_t tileQx =
                    tile.offset.x / 16;
                const int32_t tileQy =
                    tile.offset.y / 16;
                for (uint8_t quarter = 0;
                     quarter < 4; ++quarter)
                {
                    if ((tile.corners
                            & (1u << quarter))
                        == 0)
                        continue;
                    const int32_t qx =
                        tileQx
                        + kQuarterCellOffsets[
                              quarter]
                              .x;
                    const int32_t qy =
                        tileQy
                        + kQuarterCellOffsets[
                              quarter]
                              .y;
                    cells.push_back({
                        qx,
                        qy,
                        tile.offset.z,
                        tile.offset.z
                            + tile.zClearance,
                        uint16_t(sequence),
                    });
                }
            }
            if (cells.empty())
                return std::nullopt;
            return cells;
        }

        [[nodiscard]] LargeSceneryAssetModel
            BuildLargeSceneryAssetModel(
                const LargeSceneryEntry& entry)
        {
            LargeSceneryAssetModel model{};
            model.bodyImageFirst =
                entry.image + 4;
            model.bodyImageLast =
                model.bodyImageFirst
                + uint32_t(
                    entry.tiles.size() * 4);
            model.attempted = true;
            if (!LargeSceneryAssetEligible(entry))
                return model;

            const auto cells =
                BuildLargeSceneryAssetCells(entry);
            if (!cells.has_value())
                return model;

            // Authoritative occupancy/clearance is the geometry contract.
            // Native sprites are consumed later only as multi-view texture
            // evidence for these fixed faces.
            model.faces =
                BuildFirstPersonQuarterCellOccupancyFaces(
                    *cells);
            model.usable = !model.faces.empty();
            return model;
        }
    }

    bool LargeSceneryAssetModelAttempted(
        const LargeSceneryEntry& entry)
    {
        const auto found =
            _largeSceneryAssetModels.find(&entry);
        return found
                != _largeSceneryAssetModels.end()
            && found->second.attempted;
    }

    const LargeSceneryAssetModel*
        GetLargeSceneryAssetModel(
            const LargeSceneryEntry& entry,
            bool allowBuild)
    {
        auto [it, inserted] =
            _largeSceneryAssetModels.try_emplace(
                &entry);
        if (inserted)
            it->second = {};

        if (!it->second.attempted)
        {
            if (!allowBuild)
                return nullptr;
            it->second =
                BuildLargeSceneryAssetModel(entry);
        }
        return &it->second;
    }

    void ClearLargeSceneryAssetModelCache()
    {
        _largeSceneryAssetModels.clear();
    }
}
