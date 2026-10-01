/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#pragma once

#include "Location.hpp"
#include "tile_element/TileElementType.h"

#include <cstdint>
#include <limits>
#include <vector>

namespace OpenRCT2::World3D
{
    enum class SurfaceKind : uint8_t
    {
        terrain,
        water,
        path,
        pathSide,
        wall,
    };

    enum class GeometrySource : uint8_t
    {
        authoritative,
        reconstructed,
        approximation,
    };

    enum class GeometryStatus : uint8_t
    {
        generated,
        unresolved,
    };

    struct Vertex
    {
        int32_t x{};
        int32_t y{};
        int32_t z{};
    };

    struct ElementSnapshot
    {
        static constexpr uint16_t kNoObject = std::numeric_limits<uint16_t>::max();

        TileElementType type{};
        CoordsXY tileOrigin{};
        int32_t baseZ{};
        int32_t clearanceZ{};
        uint8_t direction{};
        uint8_t occupiedQuadrants{};

        // Type-specific structural facts. Values stay unset when they do not apply.
        uint16_t objectIndex{ kNoObject };
        uint16_t trackType{ kNoObject };
        uint8_t sequenceIndex{};
        uint8_t slope{};
        uint8_t pathEdges{};
        uint8_t pathCorners{};
        uint8_t pathSlopeDirection{};
        bool pathIsSloped{};

        GeometryStatus geometryStatus{ GeometryStatus::unresolved };
    };

    struct Triangle
    {
        uint32_t a{};
        uint32_t b{};
        uint32_t c{};
        uint32_t elementIndex{};
        SurfaceKind kind{};
        GeometrySource source{};
        bool collidable{ true };
    };

    struct WorldSnapshot
    {
        std::vector<Vertex> vertices;
        std::vector<Triangle> triangles;
        std::vector<ElementSnapshot> elements;
    };

    /**
     * Builds persistent world-space geometry from structural map data.
     *
     * Coordinates are kept in OpenRCT2's native "big" coordinate system
     * (32 x/y units per tile, 8 z units per height step) so the resulting mesh
     * can be related back to sprites without another lossy coordinate transform.
     *
     * Terrain is reconstructed from the exact slope height function. Path top
     * surfaces use their specified base/slope elevations; their thin side shell
     * is explicitly marked as an approximation. Walls are represented as thin
     * prisms because map occupancy gives their edge and height but not literal
     * artwork thickness.
     *
     * Elements whose exact physical shape is not yet derivable are still copied
     * into ElementSnapshot with GeometryStatus::unresolved. Their occupied
     * base/clearance range is evidence, not a generated solid.
     */
    WorldSnapshot BuildParkSnapshot();
} // namespace OpenRCT2::World3D
