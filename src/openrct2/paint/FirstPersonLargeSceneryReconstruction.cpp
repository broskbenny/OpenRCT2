/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#include "FirstPersonLargeSceneryReconstruction.h"

#include "FirstPersonVisualHull.h"
#include "tile_element/Paint.TileElement.h"

#include "../drawing/Drawing.Sprite.h"
#include "../interface/Viewport.h"
#include "../object/LargeSceneryEntry.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <unordered_map>
#include <vector>

namespace OpenRCT2::Paint
{
    bool LargeSceneryAssetEligible(
        const LargeSceneryEntry& entry)
    {
        return !entry.flags.has(LargeSceneryFlag::isTree)
            && !entry.tiles.empty() && entry.tiles.size() <= 256;
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

        template<typename TCallback>
        bool ForEachLargeSceneryOpaquePixel(const G1Element& g1, TCallback&& callback)
        {
            if (g1.offset == nullptr || g1.width <= 0 || g1.height <= 0
                || g1.flags.has(G1Flag::isPalette))
                return false;

            bool any = false;
            if (g1.flags.has(G1Flag::hasRLECompression))
            {
                for (int32_t y = 0; y < g1.height; ++y)
                {
                    const uint16_t lineOffset =
                        uint16_t(g1.offset[y * 2]) | (uint16_t(g1.offset[y * 2 + 1]) << 8);
                    const uint8_t* run = g1.offset + lineOffset;
                    bool endOfLine = false;
                    size_t runGuard = 0;
                    while (!endOfLine && runGuard++ < 256)
                    {
                        uint8_t length = *run++;
                        const int32_t x = *run++;
                        endOfLine = (length & 0x80u) != 0;
                        length &= 0x7Fu;
                        for (uint8_t n = 0; n < length; ++n)
                        {
                            // RLE drawing always treats palette index zero as
                            // transparent, even inside a stored run.
                            if (run[n] == 0)
                                continue;
                            callback(x + n, y);
                            any = true;
                        }
                        run += length;
                    }
                }
                return any;
            }

            const bool hasTransparency = g1.flags.has(G1Flag::hasTransparency);
            for (int32_t y = 0; y < g1.height; ++y)
            for (int32_t x = 0; x < g1.width; ++x)
            {
                const uint8_t pixel = g1.offset[size_t(y) * size_t(g1.width) + size_t(x)];
                if (hasTransparency && pixel == 0)
                    continue;
                callback(x, y);
                any = true;
            }
            return any;
        }

        struct LargeSceneryObservedViews
        {
            bool valid = false;
            std::array<FirstPersonSilhouette, 4> combined;
            std::vector<std::array<FirstPersonSilhouette, 4>> bySequence;
        };

        [[nodiscard]] LargeSceneryObservedViews CollectLargeSceneryObservedViews(
            const LargeSceneryEntry& entry)
        {
            LargeSceneryObservedViews result{};
            result.bySequence.resize(entry.tiles.size());
            // Custom objects can legally contain very large sprites. A fit is
            // optional evidence, so cap one asset's first-use work rather than
            // allowing reconstruction to become a new frame hitch.
            constexpr size_t kMaxSilhouetteSourcePixels = 262144;
            size_t sourcePixels = 0;

            for (size_t sequence = 0; sequence < entry.tiles.size(); ++sequence)
            {
                const auto& tile = entry.tiles[sequence];
                for (uint8_t rotation = 0; rotation < 4; ++rotation)
                {
                    const ImageIndex imageIndex =
                        entry.image + 4 + (ImageIndex(sequence) << 2) + rotation;
                    const auto* g1 = GfxGetG1Element(imageIndex);
                    if (g1 == nullptr || g1->width <= 0 || g1->height <= 0)
                        return result;
                    sourcePixels += size_t(g1->width) * size_t(g1->height);
                    if (sourcePixels > kMaxSilhouetteSourcePixels)
                        return result;

                    const auto spriteOrigin = GetTileElementPaintSpritePosition(
                        { tile.offset.x, tile.offset.y }, rotation);
                    const auto spritePos = Translate3DTo2DWithZ(
                        rotation, { spriteOrigin, tile.offset.z });
                    auto& sequenceSilhouette = result.bySequence[sequence][rotation];
                    auto& combined = result.combined[rotation];
                    const bool any = ForEachLargeSceneryOpaquePixel(
                        *g1, [&](int32_t x, int32_t y) {
                            const int32_t sx = spritePos.x + g1->xOffset + x;
                            const int32_t sy = spritePos.y + g1->yOffset + y;
                            sequenceSilhouette.add(sx, sy);
                            combined.add(sx, sy);
                        });
                    if (!any)
                        return result;
                }
            }

            result.valid = std::all_of(
                result.combined.begin(), result.combined.end(),
                [](const FirstPersonSilhouette& silhouette) { return !silhouette.empty(); });
            return result;
        }

        [[nodiscard]] std::optional<std::vector<LargeSceneryAssetCell>> BuildLargeSceneryAssetCells(
            const LargeSceneryEntry& entry)
        {
            static constexpr std::array<CoordsXY, 4> kQuarterCellOffsets{ {
                { 1, 1 }, // SW
                { 1, 0 }, // NW
                { 0, 0 }, // NE
                { 0, 1 }, // SE
            } };

            std::vector<LargeSceneryAssetCell> cells;
            for (size_t sequence = 0; sequence < entry.tiles.size(); ++sequence)
            {
                const auto& tile = entry.tiles[sequence];
                if ((tile.offset.x % 16) != 0 || (tile.offset.y % 16) != 0
                    || tile.zClearance <= 0 || (tile.corners & 0x0F) == 0)
                    return std::nullopt;

                const int32_t tileQx = tile.offset.x / 16;
                const int32_t tileQy = tile.offset.y / 16;
                for (uint8_t quarter = 0; quarter < 4; ++quarter)
                {
                    if ((tile.corners & (1u << quarter)) == 0)
                        continue;
                    const int32_t qx =
                        tileQx + kQuarterCellOffsets[quarter].x;
                    const int32_t qy =
                        tileQy + kQuarterCellOffsets[quarter].y;
                    cells.push_back({
                        qx, qy, tile.offset.z,
                        tile.offset.z + tile.zClearance,
                        uint16_t(sequence),
                    });
                }
            }
            if (cells.empty())
                return std::nullopt;
            return cells;
        }

        [[nodiscard]] bool LargeSceneryCellContainsPoint(
            const LargeSceneryAssetCell& cell, FirstPersonVec3 point)
        {
            const float x0 = float(cell.qx * 16);
            const float y0 = float(cell.qy * 16);
            return point.x >= x0 && point.x < x0 + 16.0f
                && point.y >= y0 && point.y < y0 + 16.0f
                && point.z >= float(cell.lowZ)
                && point.z < float(cell.highZ);
        }

        [[nodiscard]] FirstPersonVisualHull BuildLargeSceneryAssetHull(
            const std::vector<LargeSceneryAssetCell>& cells,
            const LargeSceneryObservedViews& observed)
        {
            if (cells.empty() || !observed.valid)
                return {};

            FirstPersonVisualHullBounds bounds{};
            bounds.minForward = float(cells.front().qx * 16);
            bounds.maxForward = bounds.minForward + 16.0f;
            bounds.minRight = float(cells.front().qy * 16);
            bounds.maxRight = bounds.minRight + 16.0f;
            bounds.minUp = float(cells.front().lowZ);
            bounds.maxUp = float(cells.front().highZ);
            bounds.step = 4.0f;
            for (const auto& cell : cells)
            {
                bounds.minForward = std::min(
                    bounds.minForward, float(cell.qx * 16));
                bounds.maxForward = std::max(
                    bounds.maxForward, float((cell.qx + 1) * 16));
                bounds.minRight = std::min(
                    bounds.minRight, float(cell.qy * 16));
                bounds.maxRight = std::max(
                    bounds.maxRight, float((cell.qy + 1) * 16));
                bounds.minUp = std::min(
                    bounds.minUp, float(cell.lowZ));
                bounds.maxUp = std::max(
                    bounds.maxUp, float(cell.highZ));
            }

            FirstPersonVisualHullConfig config{};
            config.minimumViews = 4;
            config.minimumOccupiedCells = 4;
            config.maximumOccupiedCells = 8192;
            config.maximumGridCells = 131072;
            config.maximumAxisCells = 96;
            config.minimumCandidateCoverage = 0.70f;
            config.minimumObservedCoverage = 0.50f;
            config.maximumEdgeError = 8;

            const auto projectPoint =
                [](uint8_t rotation, FirstPersonVec3 point) {
                    const auto projected =
                        Translate3DTo2DWithZ(
                            rotation,
                            {
                                int32_t(std::lround(point.x)),
                                int32_t(std::lround(point.y)),
                                int32_t(std::lround(point.z)),
                            });
                    return std::array<float, 2>{
                        float(projected.x), float(projected.y)
                    };
                };
            const auto occupancyPredicate =
                [&](FirstPersonVec3 point) {
                    return std::any_of(
                        cells.begin(), cells.end(),
                        [&](const auto& cell) {
                            return LargeSceneryCellContainsPoint(
                                cell, point);
                        });
                };
            const auto pointSupported =
                [&](const FirstPersonVisualHullView& view,
                    FirstPersonVec3 point) {
                    const auto projected =
                        projectPoint(view.imageDirection, point);
                    const int32_t x =
                        int32_t(std::lround(projected[0]));
                    const int32_t y =
                        int32_t(std::lround(projected[1]));
                    for (int32_t dy = -1; dy <= 1; ++dy)
                    for (int32_t dx = -1; dx <= 1; ++dx)
                    {
                        if (view.observed.contains(x + dx, y + dy))
                            return true;
                    }
                    return false;
                };

            std::vector<FirstPersonVisualHullView> views;
            views.reserve(4);
            for (uint8_t rotation = 0; rotation < 4; ++rotation)
            {
                FirstPersonVisualHullView view{};
                view.imageDirection = rotation;
                view.observed = observed.combined[rotation];
                views.push_back(std::move(view));
            }
            return BuildFirstPersonVisualHull(
                views, bounds, config,
                projectPoint, occupancyPredicate,
                pointSupported);
        }

        template<typename SampleFace, typename BuildCorners>
        void AppendLargeSceneryGreedyFaceSlices(
            std::vector<LargeSceneryAssetFace>& result,
            int32_t sliceCount, int32_t axisACount,
            int32_t axisBCount, LargeSceneryAssetFaceKind kind,
            SampleFace&& sampleFace, BuildCorners&& buildCorners)
        {
            std::vector<int32_t> mask(
                size_t(axisACount) * size_t(axisBCount), -1);
            for (int32_t slice = 0; slice < sliceCount; ++slice)
            {
                std::fill(mask.begin(), mask.end(), -1);
                for (int32_t b = 0; b < axisBCount; ++b)
                for (int32_t a = 0; a < axisACount; ++a)
                    mask[size_t(b) * size_t(axisACount) + size_t(a)] =
                        sampleFace(slice, a, b);

                for (int32_t b = 0; b < axisBCount; ++b)
                for (int32_t a = 0; a < axisACount; ++a)
                {
                    const size_t index =
                        size_t(b) * size_t(axisACount) + size_t(a);
                    const int32_t owner = mask[index];
                    if (owner < 0)
                        continue;

                    int32_t width = 1;
                    while (a + width < axisACount
                        && mask[size_t(b) * size_t(axisACount)
                            + size_t(a + width)] == owner)
                        ++width;

                    int32_t height = 1;
                    for (; b + height < axisBCount; ++height)
                    {
                        bool same = true;
                        for (int32_t x = 0; x < width; ++x)
                        {
                            if (mask[
                                    size_t(b + height)
                                        * size_t(axisACount)
                                    + size_t(a + x)] != owner)
                            {
                                same = false;
                                break;
                            }
                        }
                        if (!same)
                            break;
                    }

                    for (int32_t y = 0; y < height; ++y)
                    for (int32_t x = 0; x < width; ++x)
                        mask[size_t(b + y) * size_t(axisACount)
                            + size_t(a + x)] = -2;
                    result.push_back({
                        buildCorners(slice, a, b, width, height),
                        uint16_t(owner), kind, 0
                    });
                }
            }
        }

        [[nodiscard]] std::vector<LargeSceneryAssetFace>
            BuildLargeSceneryAssetFaces(
                const FirstPersonVisualHull& hull,
                const std::vector<LargeSceneryAssetCell>& cells,
                const LargeSceneryObservedViews& observed)
        {
            std::vector<LargeSceneryAssetFace> result;
            const auto projectedPoint =
                [](uint8_t rotation, FirstPersonVec3 point) {
                    return Translate3DTo2DWithZ(
                        rotation,
                        {
                            int32_t(std::lround(point.x)),
                            int32_t(std::lround(point.y)),
                            int32_t(std::lround(point.z)),
                        });
                };
            std::vector<int16_t> owners(
                hull.occupied.size(), int16_t(-1));
            for (int32_t up = 0; up < hull.sizeUp; ++up)
            for (int32_t right = 0; right < hull.sizeRight; ++right)
            for (int32_t forward = 0; forward < hull.sizeForward; ++forward)
            {
                if (!hull.contains(forward, right, up))
                    continue;
                const auto point =
                    hull.centre(forward, right, up);
                std::array<ScreenCoordsXY, 4> projected{};
                for (uint8_t rotation = 0; rotation < 4; ++rotation)
                    projected[rotation] =
                        projectedPoint(rotation, point);

                int32_t bestSequence = -1;
                int32_t bestScore = -1;
                for (const auto& cell : cells)
                {
                    if (!LargeSceneryCellContainsPoint(cell, point))
                        continue;
                    int32_t score = 0;
                    for (uint8_t rotation = 0; rotation < 4; ++rotation)
                    {
                        bool supported = false;
                        for (int32_t dy = -1; dy <= 1
                             && !supported; ++dy)
                        for (int32_t dx = -1; dx <= 1; ++dx)
                        {
                            if (observed.bySequence[
                                    cell.sequence][rotation]
                                    .contains(
                                        projected[rotation].x + dx,
                                        projected[rotation].y + dy))
                            {
                                supported = true;
                                break;
                            }
                        }
                        if (supported)
                            ++score;
                    }
                    if (score > bestScore
                        || (score == bestScore
                            && (bestSequence < 0
                                || cell.sequence < bestSequence)))
                    {
                        bestScore = score;
                        bestSequence = cell.sequence;
                    }
                }
                const size_t index =
                    (size_t(up) * size_t(hull.sizeRight)
                        + size_t(right))
                        * size_t(hull.sizeForward)
                        + size_t(forward);
                owners[index] = int16_t(bestSequence);
            }
            const auto ownerAt =
                [&](int32_t forward, int32_t right,
                    int32_t up) -> int32_t {
                    if (!hull.contains(forward, right, up))
                        return -1;
                    const size_t index =
                        (size_t(up) * size_t(hull.sizeRight)
                            + size_t(right))
                            * size_t(hull.sizeForward)
                            + size_t(forward);
                    return owners[index];
                };
            const auto coord =
                [&](float base, int32_t cell) {
                    return int32_t(std::lround(
                        base + float(cell) * hull.step));
                };

            AppendLargeSceneryGreedyFaceSlices(
                result, hull.sizeForward, hull.sizeRight, hull.sizeUp,
                LargeSceneryAssetFaceKind::minX,
                [&](int32_t f, int32_t r, int32_t u) {
                    const int32_t owner = ownerAt(f, r, u);
                    return owner >= 0 && !hull.contains(f - 1, r, u)
                        ? owner : -1;
                },
                [&](int32_t f, int32_t r, int32_t u,
                    int32_t width, int32_t height) {
                    const int32_t x = coord(hull.minForward, f);
                    const int32_t y0 = coord(hull.minRight, r);
                    const int32_t y1 = coord(hull.minRight, r + width);
                    const int32_t z0 = coord(hull.minUp, u);
                    const int32_t z1 = coord(hull.minUp, u + height);
                    return std::array<CoordsXYZ, 4>{ {
                        { x, y0, z0 }, { x, y1, z0 },
                        { x, y1, z1 }, { x, y0, z1 },
                    } };
                });
            AppendLargeSceneryGreedyFaceSlices(
                result, hull.sizeForward, hull.sizeRight, hull.sizeUp,
                LargeSceneryAssetFaceKind::maxX,
                [&](int32_t f, int32_t r, int32_t u) {
                    const int32_t owner = ownerAt(f, r, u);
                    return owner >= 0 && !hull.contains(f + 1, r, u)
                        ? owner : -1;
                },
                [&](int32_t f, int32_t r, int32_t u,
                    int32_t width, int32_t height) {
                    const int32_t x = coord(hull.minForward, f + 1);
                    const int32_t y0 = coord(hull.minRight, r);
                    const int32_t y1 = coord(hull.minRight, r + width);
                    const int32_t z0 = coord(hull.minUp, u);
                    const int32_t z1 = coord(hull.minUp, u + height);
                    return std::array<CoordsXYZ, 4>{ {
                        { x, y1, z0 }, { x, y0, z0 },
                        { x, y0, z1 }, { x, y1, z1 },
                    } };
                });
            AppendLargeSceneryGreedyFaceSlices(
                result, hull.sizeRight, hull.sizeForward, hull.sizeUp,
                LargeSceneryAssetFaceKind::minY,
                [&](int32_t r, int32_t f, int32_t u) {
                    const int32_t owner = ownerAt(f, r, u);
                    return owner >= 0 && !hull.contains(f, r - 1, u)
                        ? owner : -1;
                },
                [&](int32_t r, int32_t f, int32_t u,
                    int32_t width, int32_t height) {
                    const int32_t y = coord(hull.minRight, r);
                    const int32_t x0 = coord(hull.minForward, f);
                    const int32_t x1 = coord(hull.minForward, f + width);
                    const int32_t z0 = coord(hull.minUp, u);
                    const int32_t z1 = coord(hull.minUp, u + height);
                    return std::array<CoordsXYZ, 4>{ {
                        { x1, y, z0 }, { x0, y, z0 },
                        { x0, y, z1 }, { x1, y, z1 },
                    } };
                });
            AppendLargeSceneryGreedyFaceSlices(
                result, hull.sizeRight, hull.sizeForward, hull.sizeUp,
                LargeSceneryAssetFaceKind::maxY,
                [&](int32_t r, int32_t f, int32_t u) {
                    const int32_t owner = ownerAt(f, r, u);
                    return owner >= 0 && !hull.contains(f, r + 1, u)
                        ? owner : -1;
                },
                [&](int32_t r, int32_t f, int32_t u,
                    int32_t width, int32_t height) {
                    const int32_t y = coord(hull.minRight, r + 1);
                    const int32_t x0 = coord(hull.minForward, f);
                    const int32_t x1 = coord(hull.minForward, f + width);
                    const int32_t z0 = coord(hull.minUp, u);
                    const int32_t z1 = coord(hull.minUp, u + height);
                    return std::array<CoordsXYZ, 4>{ {
                        { x0, y, z0 }, { x1, y, z0 },
                        { x1, y, z1 }, { x0, y, z1 },
                    } };
                });
            AppendLargeSceneryGreedyFaceSlices(
                result, hull.sizeUp, hull.sizeForward, hull.sizeRight,
                LargeSceneryAssetFaceKind::bottom,
                [&](int32_t u, int32_t f, int32_t r) {
                    const int32_t owner = ownerAt(f, r, u);
                    return owner >= 0 && !hull.contains(f, r, u - 1)
                        ? owner : -1;
                },
                [&](int32_t u, int32_t f, int32_t r,
                    int32_t width, int32_t height) {
                    const int32_t z = coord(hull.minUp, u);
                    const int32_t x0 = coord(hull.minForward, f);
                    const int32_t x1 = coord(hull.minForward, f + width);
                    const int32_t y0 = coord(hull.minRight, r);
                    const int32_t y1 = coord(hull.minRight, r + height);
                    return std::array<CoordsXYZ, 4>{ {
                        { x0, y1, z }, { x1, y1, z },
                        { x1, y0, z }, { x0, y0, z },
                    } };
                });
            AppendLargeSceneryGreedyFaceSlices(
                result, hull.sizeUp, hull.sizeForward, hull.sizeRight,
                LargeSceneryAssetFaceKind::top,
                [&](int32_t u, int32_t f, int32_t r) {
                    const int32_t owner = ownerAt(f, r, u);
                    return owner >= 0 && !hull.contains(f, r, u + 1)
                        ? owner : -1;
                },
                [&](int32_t u, int32_t f, int32_t r,
                    int32_t width, int32_t height) {
                    const int32_t z = coord(hull.minUp, u + 1);
                    const int32_t x0 = coord(hull.minForward, f);
                    const int32_t x1 = coord(hull.minForward, f + width);
                    const int32_t y0 = coord(hull.minRight, r);
                    const int32_t y1 = coord(hull.minRight, r + height);
                    return std::array<CoordsXYZ, 4>{ {
                        { x0, y0, z }, { x1, y0, z },
                        { x1, y1, z }, { x0, y1, z },
                    } };
                });
            return result;
        }

        [[nodiscard]] bool LargeSceneryFaceVisibleFromDirection(
            LargeSceneryAssetFaceKind kind, uint8_t direction)
        {
            if (kind == LargeSceneryAssetFaceKind::top)
                return true;
            if (kind == LargeSceneryAssetFaceKind::bottom)
                return false;
            CoordsXY normal{};
            switch (kind)
            {
                case LargeSceneryAssetFaceKind::minX: normal = { -1, 0 }; break;
                case LargeSceneryAssetFaceKind::maxX: normal = { 1, 0 }; break;
                case LargeSceneryAssetFaceKind::minY: normal = { 0, -1 }; break;
                case LargeSceneryAssetFaceKind::maxY: normal = { 0, 1 }; break;
                case LargeSceneryAssetFaceKind::top: return true;
                case LargeSceneryAssetFaceKind::bottom: return false;
            }
            return FirstPersonFaceVisibleFromNativeView(normal, direction);
        }

        [[nodiscard]] FirstPersonSilhouette RasterizeLargeSceneryAssetFace(
            const LargeSceneryAssetFace& face, uint8_t rotation)
        {
            std::array<ScreenCoordsXY, 4> projected{};
            for (size_t i = 0; i < face.corners.size(); ++i)
                projected[i] = Translate3DTo2DWithZ(rotation, face.corners[i]);
            FirstPersonSilhouette result{};
            AddFirstPersonSilhouetteQuad(result, projected);
            return result;
        }

        [[nodiscard]] std::optional<size_t>
            EstimateLargeSceneryAssetRasterWork(
                const std::vector<LargeSceneryAssetFace>& faces)
        {
            size_t work = 0;
            constexpr size_t kMaxFaceRasterWork = 32768;
            constexpr size_t kMaxCandidateRasterWork = 131072;
            for (uint8_t rotation = 0; rotation < 4; ++rotation)
            for (const auto& face : faces)
            {
                if (!LargeSceneryFaceVisibleFromDirection(
                        face.kind, rotation))
                    continue;
                std::array<ScreenCoordsXY, 4> projected{};
                for (size_t n = 0; n < face.corners.size(); ++n)
                {
                    projected[n] =
                        Translate3DTo2DWithZ(
                            rotation, face.corners[n]);
                }
                int32_t minX = projected[0].x;
                int32_t maxX = projected[0].x;
                int32_t minY = projected[0].y;
                int32_t maxY = projected[0].y;
                for (size_t n = 1; n < projected.size(); ++n)
                {
                    minX = std::min(minX, projected[n].x);
                    maxX = std::max(maxX, projected[n].x);
                    minY = std::min(minY, projected[n].y);
                    maxY = std::max(maxY, projected[n].y);
                }
                const int64_t width =
                    int64_t(maxX) - int64_t(minX);
                const int64_t height =
                    int64_t(maxY) - int64_t(minY);
                if (width <= 0 || height <= 0)
                    continue;
                const uint64_t faceWork =
                    uint64_t(width) * uint64_t(height);
                if (faceWork > kMaxFaceRasterWork)
                    return std::nullopt;
                work += size_t(faceWork);
                if (work > kMaxCandidateRasterWork)
                    return std::nullopt;
            }
            return work;
        }

        [[nodiscard]] std::array<FirstPersonDepthOwnerMap, 4>
            BuildLargeSceneryAssetDepthOwners(
                const std::vector<LargeSceneryAssetFace>& faces)
        {
            std::array<FirstPersonDepthOwnerMap, 4> result{};
            for (uint8_t rotation = 0; rotation < 4; ++rotation)
            {
                for (size_t faceIndex = 0; faceIndex < faces.size(); ++faceIndex)
                {
                    const auto& face = faces[faceIndex];
                    if (!LargeSceneryFaceVisibleFromDirection(face.kind, rotation))
                        continue;

                    std::array<ScreenCoordsXY, 4> screen{};
                    std::array<float, 4> depth{};
                    for (size_t i = 0; i < face.corners.size(); ++i)
                    {
                        screen[i] = Translate3DTo2DWithZ(rotation, face.corners[i]);
                        depth[i] = FirstPersonIsoDepth(rotation, face.corners[i]);
                    }
                    AddFirstPersonDepthTriangle(
                        result[rotation], uint32_t(faceIndex),
                        { screen[0], screen[1], screen[2] },
                        { depth[0], depth[1], depth[2] });
                    AddFirstPersonDepthTriangle(
                        result[rotation], uint32_t(faceIndex),
                        { screen[0], screen[2], screen[3] },
                        { depth[0], depth[2], depth[3] });
                }
            }
            return result;
        }

        [[nodiscard]] LargeSceneryAssetModel BuildLargeSceneryAssetModel(
            const LargeSceneryEntry& entry)
        {
            LargeSceneryAssetModel model{};
            model.bodyImageFirst = entry.image + 4;
            model.bodyImageLast =
                model.bodyImageFirst + uint32_t(entry.tiles.size() * 4);
            model.attempted = true;
            if (!LargeSceneryAssetEligible(entry))
                return model;

            const auto cells = BuildLargeSceneryAssetCells(entry);
            if (!cells.has_value())
                return model;

            constexpr size_t kMaxCarvedCells = 48;
            LargeSceneryObservedViews observed{};
            FirstPersonVisualHull hull{};
            std::vector<LargeSceneryAssetFace> faces;
            if (cells->size() <= kMaxCarvedCells)
            {
                observed = CollectLargeSceneryObservedViews(entry);
                hull = BuildLargeSceneryAssetHull(*cells, observed);
                if (hull.valid)
                    faces = BuildLargeSceneryAssetFaces(
                        hull, *cells, observed);
            }

            constexpr size_t kMaxCarvedFaces = 1024;
            const bool carved =
                !faces.empty()
                && faces.size() <= kMaxCarvedFaces
                && EstimateLargeSceneryAssetRasterWork(faces)
                    .has_value();
            if (!carved)
            {
                faces =
                    BuildFirstPersonQuarterCellOccupancyFaces(
                        *cells);
                if (faces.empty())
                    return model;

                const auto depthOwners =
                    BuildLargeSceneryAssetDepthOwners(faces);
                float minimumOwnership = 1.0f;
                for (size_t faceIndex = 0;
                     faceIndex < faces.size(); ++faceIndex)
                {
                    auto& face = faces[faceIndex];
                    float bestOwnership = -1.0f;
                    std::optional<uint8_t> bestDirection;
                    for (uint8_t direction = 0;
                         direction < 4; ++direction)
                    {
                        if (!LargeSceneryFaceVisibleFromDirection(
                                face.kind, direction))
                            continue;
                        const ImageIndex sourceImage =
                            entry.image + 4
                            + (ImageIndex(face.sequence) << 2)
                            + direction;
                        const auto* g1 =
                            GfxGetG1Element(sourceImage);
                        if (g1 == nullptr
                            || g1->width <= 0
                            || g1->height <= 0)
                            continue;
                        const auto projected =
                            RasterizeLargeSceneryAssetFace(
                                face, direction);
                        const float ownership =
                            FirstPersonDepthOwnerCoverage(
                                depthOwners[direction],
                                uint32_t(faceIndex),
                                projected);
                        if (!bestDirection.has_value()
                            || ownership > bestOwnership)
                        {
                            bestDirection = direction;
                            bestOwnership = ownership;
                        }
                    }
                    if (!bestDirection.has_value())
                    {
                        bestDirection =
                            ChooseFirstPersonOccupancyFaceSourceDirection(
                                face.kind,
                                [&](uint8_t direction) {
                                    const ImageIndex sourceImage =
                                        entry.image + 4
                                        + (ImageIndex(face.sequence) << 2)
                                        + direction;
                                    const auto* g1 =
                                        GfxGetG1Element(sourceImage);
                                    return g1 != nullptr
                                        && g1->width > 0
                                        && g1->height > 0;
                                });
                        bestOwnership = 0.0f;
                    }
                    if (!bestDirection.has_value())
                        return model;

                    face.sourceDirection = *bestDirection;
                    face.textureFallbackOnly =
                        face.kind == LargeSceneryAssetFaceKind::bottom
                        || bestOwnership < 0.98f;
                    if (face.kind != LargeSceneryAssetFaceKind::bottom)
                    {
                        minimumOwnership =
                            std::min(
                                minimumOwnership,
                                std::max(0.0f, bestOwnership));
                    }
                }
                model.minimumCandidateCoverage = 0.0f;
                model.minimumFaceOwnership = minimumOwnership;
                model.faces = std::move(faces);
                model.usable = true;
                return model;
            }

            const auto depthOwners =
                BuildLargeSceneryAssetDepthOwners(faces);
            float minimumFaceOwnership = 1.0f;
            for (size_t faceIndex = 0; faceIndex < faces.size(); ++faceIndex)
            {
                auto& face = faces[faceIndex];
                float bestFaceScore = -1.0f;
                float bestOwnership = 0.0f;
                uint8_t bestDirection = 0;
                for (uint8_t direction = 0; direction < 4; ++direction)
                {
                    if (!LargeSceneryFaceVisibleFromDirection(
                            face.kind, direction))
                        continue;
                    const ImageIndex sourceImage =
                        entry.image + 4
                        + (ImageIndex(face.sequence) << 2)
                        + direction;
                    const auto* g1 = GfxGetG1Element(sourceImage);
                    if (g1 == nullptr || g1->width <= 0
                        || g1->height <= 0)
                        continue;
                    const auto projectedFace =
                        RasterizeLargeSceneryAssetFace(face, direction);
                    if (projectedFace.empty())
                        continue;

                    const float ownership =
                        FirstPersonDepthOwnerCoverage(
                            depthOwners[direction], uint32_t(faceIndex),
                            projectedFace);
                    const auto& source =
                        observed.bySequence[face.sequence][direction];
                    const auto fit =
                        CompareFirstPersonSilhouettes(
                            source, projectedFace);
                    const float coverage =
                        fit.valid ? fit.candidateCoverage : 0.0f;
                    const float score =
                        ownership * 4.0f
                        + (fit.valid ? coverage : 0.0f);
                    if (score <= bestFaceScore)
                        continue;
                    bestFaceScore = score;
                    bestOwnership = ownership;
                    bestDirection = direction;
                }
                if (bestFaceScore < 0.0f)
                {
                    for (uint8_t direction = 0; direction < 4; ++direction)
                    {
                        const ImageIndex sourceImage =
                            entry.image + 4
                            + (ImageIndex(face.sequence) << 2)
                            + direction;
                        const auto* g1 = GfxGetG1Element(sourceImage);
                        if (g1 != nullptr && g1->width > 0
                            && g1->height > 0)
                        {
                            bestDirection = direction;
                            bestFaceScore = 0.0f;
                            break;
                        }
                    }
                }
                if (bestFaceScore < 0.0f)
                    return model;
                face.sourceDirection = bestDirection;
                face.textureFallbackOnly =
                    face.kind == LargeSceneryAssetFaceKind::bottom
                    || bestOwnership < 0.98f;
                if (face.kind != LargeSceneryAssetFaceKind::bottom)
                {
                    minimumFaceOwnership =
                        std::min(
                            minimumFaceOwnership,
                            bestOwnership);
                }
            }

            model.minimumCandidateCoverage =
                hull.minimumCandidateCoverage;
            model.minimumFaceOwnership = minimumFaceOwnership;
            model.faces = std::move(faces);
            model.usable = true;
            return model;
        }


    }

    bool LargeSceneryAssetModelAttempted(
        const LargeSceneryEntry& entry)
    {
        const auto found =
            _largeSceneryAssetModels.find(&entry);
        return found != _largeSceneryAssetModels.end()
            && found->second.attempted;
    }

    const LargeSceneryAssetModel*
        GetLargeSceneryAssetModel(
            const LargeSceneryEntry& entry, bool allowBuild)
    {
        auto [it, inserted] =
            _largeSceneryAssetModels.try_emplace(&entry);
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
