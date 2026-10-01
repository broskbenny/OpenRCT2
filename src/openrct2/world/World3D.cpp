/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "World3D.h"

#include "Map.h"
#include "MapLimits.h"
#include "TileElementsView.h"
#include "tile_element/EntranceElement.h"
#include "tile_element/LargeSceneryElement.h"
#include "tile_element/PathElement.h"
#include "tile_element/SmallSceneryElement.h"
#include "tile_element/SurfaceElement.h"
#include "tile_element/TrackElement.h"
#include "tile_element/WallElement.h"

#include <array>
#include <cstdint>

namespace OpenRCT2::World3D
{
    namespace
    {
        constexpr int32_t kPathShellThickness = 2;
        constexpr int32_t kWallProxyThickness = 2;

        uint32_t AddVertex(WorldSnapshot& world, int32_t x, int32_t y, int32_t z)
        {
            world.vertices.push_back({ x, y, z });
            return static_cast<uint32_t>(world.vertices.size() - 1);
        }

        void AddTriangle(
            WorldSnapshot& world, uint32_t a, uint32_t b, uint32_t c, uint32_t elementIndex, SurfaceKind kind,
            GeometrySource source, bool collidable = true)
        {
            world.triangles.push_back({ a, b, c, elementIndex, kind, source, collidable });
        }

        void AddQuad(
            WorldSnapshot& world, const std::array<Vertex, 4>& vertices, uint32_t elementIndex, SurfaceKind kind,
            GeometrySource source, bool collidable = true)
        {
            const auto a = AddVertex(world, vertices[0].x, vertices[0].y, vertices[0].z);
            const auto b = AddVertex(world, vertices[1].x, vertices[1].y, vertices[1].z);
            const auto c = AddVertex(world, vertices[2].x, vertices[2].y, vertices[2].z);
            const auto d = AddVertex(world, vertices[3].x, vertices[3].y, vertices[3].z);
            AddTriangle(world, a, b, c, elementIndex, kind, source, collidable);
            AddTriangle(world, a, c, d, elementIndex, kind, source, collidable);
        }

        void AddBox(
            WorldSnapshot& world, const Vertex& min, const Vertex& max, uint32_t elementIndex, SurfaceKind kind,
            GeometrySource source)
        {
            const std::array<Vertex, 8> v = {
                Vertex{ min.x, min.y, min.z },
                Vertex{ max.x, min.y, min.z },
                Vertex{ max.x, max.y, min.z },
                Vertex{ min.x, max.y, min.z },
                Vertex{ min.x, min.y, max.z },
                Vertex{ max.x, min.y, max.z },
                Vertex{ max.x, max.y, max.z },
                Vertex{ min.x, max.y, max.z },
            };

            const auto base = static_cast<uint32_t>(world.vertices.size());
            world.vertices.insert(world.vertices.end(), v.begin(), v.end());

            const auto tri = [&](uint32_t a, uint32_t b, uint32_t c) {
                AddTriangle(world, base + a, base + b, base + c, elementIndex, kind, source);
            };

            tri(0, 2, 1);
            tri(0, 3, 2);
            tri(4, 5, 6);
            tri(4, 6, 7);
            tri(0, 1, 5);
            tri(0, 5, 4);
            tri(1, 2, 6);
            tri(1, 6, 5);
            tri(2, 3, 7);
            tri(2, 7, 6);
            tri(3, 0, 4);
            tri(3, 4, 7);
        }

        void AddTerrain(
            WorldSnapshot& world, const CoordsXY& tile, const SurfaceElement& surface, uint32_t elementIndex)
        {
            // MapGetCornerHeight works in legacy height units, while the world uses big z coordinates.
            const auto baseHeight = surface.getBaseZ() / kCoordsZStep;
            const auto slope = surface.getSlope();

            // The named RCT corners map to these square corners in map coordinates:
            // S=(0,0), E=(32,0), N=(32,32), W=(0,32).
            const auto zS = MapGetCornerHeight(baseHeight, slope, 2) * kCoordsZStep;
            const auto zE = MapGetCornerHeight(baseHeight, slope, 1) * kCoordsZStep;
            const auto zN = MapGetCornerHeight(baseHeight, slope, 0) * kCoordsZStep;
            const auto zW = MapGetCornerHeight(baseHeight, slope, 3) * kCoordsZStep;
            const auto zCentre = TileElementHeight({ tile.x + kCoordsXYHalfTile, tile.y + kCoordsXYHalfTile });

            const auto s = AddVertex(world, tile.x, tile.y, zS);
            const auto e = AddVertex(world, tile.x + kCoordsXYStep, tile.y, zE);
            const auto n = AddVertex(world, tile.x + kCoordsXYStep, tile.y + kCoordsXYStep, zN);
            const auto w = AddVertex(world, tile.x, tile.y + kCoordsXYStep, zW);
            const auto centre = AddVertex(
                world, tile.x + kCoordsXYHalfTile, tile.y + kCoordsXYHalfTile, zCentre);

            // The centre fan preserves the diagonal creases used by one-corner slopes and valleys.
            AddTriangle(world, s, e, centre, elementIndex, SurfaceKind::terrain, GeometrySource::authoritative);
            AddTriangle(world, e, n, centre, elementIndex, SurfaceKind::terrain, GeometrySource::authoritative);
            AddTriangle(world, n, w, centre, elementIndex, SurfaceKind::terrain, GeometrySource::authoritative);
            AddTriangle(world, w, s, centre, elementIndex, SurfaceKind::terrain, GeometrySource::authoritative);

            const auto waterZ = surface.getWaterHeight();
            if (waterZ > 0)
            {
                AddQuad(
                    world,
                    { Vertex{ tile.x, tile.y, waterZ },
                      Vertex{ tile.x + kCoordsXYStep, tile.y, waterZ },
                      Vertex{ tile.x + kCoordsXYStep, tile.y + kCoordsXYStep, waterZ },
                      Vertex{ tile.x, tile.y + kCoordsXYStep, waterZ } },
                    elementIndex, SurfaceKind::water, GeometrySource::authoritative, false);
            }
        }

        std::array<int32_t, 4> GetPathCornerZ(const PathElement& path)
        {
            // Corner order: S, E, N, W.
            std::array<int32_t, 4> z = { path.getBaseZ(), path.getBaseZ(), path.getBaseZ(), path.getBaseZ() };
            if (!path.isSloped())
            {
                return z;
            }

            const auto highZ = path.getBaseZ() + kLandHeightStep;
            switch (path.getSlopeDirection())
            {
                case 0: // west edge
                    z[0] = highZ;
                    z[3] = highZ;
                    break;
                case 1: // north edge
                    z[2] = highZ;
                    z[3] = highZ;
                    break;
                case 2: // east edge
                    z[1] = highZ;
                    z[2] = highZ;
                    break;
                case 3: // south edge
                    z[0] = highZ;
                    z[1] = highZ;
                    break;
            }
            return z;
        }

        void AddPath(WorldSnapshot& world, const CoordsXY& tile, const PathElement& path, uint32_t elementIndex)
        {
            const auto z = GetPathCornerZ(path);
            const std::array<Vertex, 4> top = {
                Vertex{ tile.x, tile.y, z[0] },
                Vertex{ tile.x + kCoordsXYStep, tile.y, z[1] },
                Vertex{ tile.x + kCoordsXYStep, tile.y + kCoordsXYStep, z[2] },
                Vertex{ tile.x, tile.y + kCoordsXYStep, z[3] },
            };

            // The walkable path elevation is structural data.
            AddQuad(world, top, elementIndex, SurfaceKind::path, GeometrySource::reconstructed);

            // The save format does not define literal pavement thickness. Keep it deliberately thin so
            // bridges do not become filled columns and under-path openings remain open.
            std::array<Vertex, 4> bottom = top;
            for (auto& vertex : bottom)
            {
                vertex.z -= kPathShellThickness;
            }

            AddQuad(world, { bottom[3], bottom[2], bottom[1], bottom[0] }, elementIndex, SurfaceKind::pathSide,
                GeometrySource::approximation);

            for (size_t i = 0; i < 4; i++)
            {
                const auto next = (i + 1) & 3;
                AddQuad(
                    world, { top[i], bottom[i], bottom[next], top[next] }, elementIndex, SurfaceKind::pathSide,
                    GeometrySource::approximation);
            }
        }

        void AddWall(WorldSnapshot& world, const CoordsXY& tile, const WallElement& wall, uint32_t elementIndex)
        {
            const auto baseZ = wall.getBaseZ();
            const auto clearanceZ = wall.getClearanceZ();
            if (clearanceZ <= baseZ)
            {
                return;
            }

            Vertex min{ tile.x, tile.y, baseZ };
            Vertex max{ tile.x + kCoordsXYStep, tile.y + kCoordsXYStep, clearanceZ };

            // Wall edge placement and height are structural. Thickness is not, so use a small explicit proxy.
            switch (wall.getDirection())
            {
                case 0:
                    max.x = min.x + kWallProxyThickness;
                    break;
                case 1:
                    min.y = max.y - kWallProxyThickness;
                    break;
                case 2:
                    min.x = max.x - kWallProxyThickness;
                    break;
                case 3:
                    max.y = min.y + kWallProxyThickness;
                    break;
            }

            AddBox(world, min, max, elementIndex, SurfaceKind::wall, GeometrySource::approximation);
        }

        ElementSnapshot SnapshotElement(const CoordsXY& tile, const TileElement& element)
        {
            ElementSnapshot result{};
            result.type = element.getType();
            result.tileOrigin = tile;
            result.baseZ = element.getBaseZ();
            result.clearanceZ = element.getClearanceZ();
            result.direction = static_cast<uint8_t>(element.getDirection());
            result.occupiedQuadrants = element.getOccupiedQuadrants();

            if (const auto* surface = element.asSurface())
            {
                result.objectIndex = static_cast<uint16_t>(surface->getSurfaceObjectIndex());
                result.slope = surface->getSlope();
            }
            else if (const auto* path = element.asPath())
            {
                result.objectIndex = static_cast<uint16_t>(path->getSurfaceEntryIndex());
                result.pathEdges = path->getEdges();
                result.pathCorners = path->getCorners();
                result.pathIsSloped = path->isSloped();
                result.pathSlopeDirection = static_cast<uint8_t>(path->getSlopeDirection());
            }
            else if (const auto* track = element.asTrack())
            {
                result.trackType = static_cast<uint16_t>(track->getTrackType());
                result.sequenceIndex = track->getSequenceIndex();
            }
            else if (const auto* scenery = element.asSmallScenery())
            {
                result.objectIndex = static_cast<uint16_t>(scenery->getEntryIndex());
            }
            else if (const auto* entrance = element.asEntrance())
            {
                result.objectIndex = static_cast<uint16_t>(entrance->getEntryIndex());
                result.sequenceIndex = static_cast<uint8_t>(entrance->getSequenceIndex());
            }
            else if (const auto* wall = element.asWall())
            {
                result.objectIndex = wall->getEntryIndex();
                result.slope = wall->getSlope();
            }
            else if (const auto* scenery = element.asLargeScenery())
            {
                result.objectIndex = static_cast<uint16_t>(scenery->getEntryIndex());
                result.sequenceIndex = scenery->getSequenceIndex();
            }

            return result;
        }
    } // namespace

    WorldSnapshot BuildParkSnapshot()
    {
        WorldSnapshot world;
        const auto mapSize = GetMapSizeUnits();

        for (int32_t y = 0; y <= mapSize.y; y += kCoordsXYStep)
        {
            for (int32_t x = 0; x <= mapSize.x; x += kCoordsXYStep)
            {
                const CoordsXY tile{ x, y };
                for (auto* element : TileElementsView<>(tile))
                {
                    if (element == nullptr || element->isGhost())
                    {
                        continue;
                    }

                    const auto elementIndex = static_cast<uint32_t>(world.elements.size());
                    world.elements.push_back(SnapshotElement(tile, *element));

                    if (const auto* surface = element->asSurface())
                    {
                        AddTerrain(world, tile, *surface, elementIndex);
                        world.elements[elementIndex].geometryStatus = GeometryStatus::generated;
                    }
                    else if (const auto* path = element->asPath())
                    {
                        AddPath(world, tile, *path, elementIndex);
                        world.elements[elementIndex].geometryStatus = GeometryStatus::generated;
                    }
                    else if (const auto* wall = element->asWall())
                    {
                        AddWall(world, tile, *wall, elementIndex);
                        world.elements[elementIndex].geometryStatus = GeometryStatus::generated;
                    }
                }
            }
        }

        return world;
    }
} // namespace OpenRCT2::World3D
