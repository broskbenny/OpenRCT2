/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonMath.h"
#include "FirstPersonStreaming.h"
#include "../Identifiers.h"
#include "../drawing/ImageId.hpp"
#include "../interface/ScreenCoords.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace OpenRCT2::Drawing { struct RenderTarget; }

namespace OpenRCT2::Paint
{
    struct FirstPersonRenderOptions
    {
        FirstPersonCamera camera{};
        EntityId hiddenEntity = EntityId::GetNull();
        uint32_t viewFlags{};
        // Retained for compatibility with the MVP caller; this is no longer a
        // maximum scene radius. Visibility covers the entire actual map.
        int32_t radiusTiles = 64;
        float fieldOfViewDegrees = 70.0f;
        float nearClip = 2.0f;
        // Minimum requested range; the collector extends it to contain the
        // loaded park (including the far diagonal of large maps).
        float farClip = 32768.0f;
    };
    struct FirstPersonVertex
    {
        FirstPersonVec3 world{};
        float u{}, v{}; // Image pixel coordinates, not normalised or atlas coordinates.
    };
    // Partition world-fixed geometry into 32 x 32 source-map tiles.
    // This bounds per-turn region membership churn and per-edit VBO uploads.
    // Zero means dynamic or camera-oriented geometry; it is always streamed.
    // Reserve zero even for the origin region by adding one to both axes.
    [[nodiscard]] constexpr uint64_t FirstPersonGpuRegionKey(int32_t tileX, int32_t tileY)
    {
        return (uint64_t(uint32_t(tileX / 32 + 1)) << 32) | uint32_t(tileY / 32 + 1);
    }
    struct FirstPersonSurface
    {
        ImageId image{};
        ImageId mask{}; // When present, mask zeroes out corresponding colour-image pixels.
        std::array<FirstPersonVertex, 6> triangles{}; // One quad: no heap allocation for every tile/sprite.
        // Cached upright sprites must always face the CURRENT camera: keep their
        // original native image-space offsets and world anchor, not old vertices.
        bool viewFacing = false;
        // Semantic decks stay at their true physical height. A small depth-only
        // bias resolves coplanar terrain without moving collision/ground geometry.
        bool depthBias = false;
        // Terrain source sprites are diamonds with transparent pixels outside
        // their silhouette. Permit a one-pixel inward coverage fallback at tile
        // edges so exact shared world vertices cannot expose background seams.
        bool edgeCoverage = false;
        // Some native sprite IDs (notably scrolling text) reference mutable
        // bitmap slots. When present, these bytes are the immutable content
        // captured when the native PaintStruct was created.
        std::vector<uint8_t> immutablePixels;
        int16_t immutableWidth = 0;
        int16_t immutableHeight = 0;
        uint64_t immutableFingerprint = 0;
        // Zero means tile-local/unconnected artwork. Nonzero identifies a
        // multi-tile reconstruction group whose native source rotation is
        // selected from one canonical object/track origin.
        uint64_t reconstructionGroup = 0;
        // Stable native painter order for palette-filter tie breaking. Physical
        // depth remains authoritative; this ordinal is consulted only when two
        // transparent fragments occupy the same physical depth.
        uint32_t nativePaintOrdinal = 0;
        uint64_t gpuRegion = 0; // Nonzero only for static WORLD-FIXED source art.
        FirstPersonVec3 billboardAnchor{};
        float billboardLeft{}, billboardTop{}, billboardWidth{}, billboardHeight{};
        // Some native object types expose authoritative physical occupancy that
        // is larger/more useful than their isometric sprite plane. Retain a
        // baked semantic sphere so cached source art is not culled merely
        // because a painter sorting bound or billboard happens to face away.
        bool hasSemanticBounds = false;
        FirstPersonVec3 semanticCenter{};
        float semanticRadius = 0.0f;
        // Masked/blended image IDs must be handled separately from opaque cutouts.
    };
    struct FirstPersonStaticRegion
    {
        uint64_t key{};
        uint64_t generation{};
        FirstPersonVec3 center{};
        float radius{};
        const std::vector<FirstPersonSurface>* surfaces = nullptr;
        const std::vector<ImageIndex>* textureDependencies = nullptr;
    };

    struct FirstPersonScene
    {
        FirstPersonRenderOptions options{};
        FirstPersonResolvedView resolvedView{};
        ScreenSize dimensions{};
        // Dynamic, camera-facing, transparent and animated surfaces only.
        std::vector<FirstPersonSurface> surfaces;
        // Persistent fixed opaque geometry is submitted by region descriptor;
        // individual static surfaces are exposed only when that region rebuilds.
        std::vector<FirstPersonStaticRegion> staticRegions;
        std::vector<CoordsXY> visibleTiles;
        // Diagnostics count native tile-paint work, not merely submitted quads.
        uint32_t staticTilePaints = 0;
        uint32_t staticTileCacheHits = 0;
        uint32_t dynamicTileQueries = 0; // O(1) occupancy probes on admitted tiles.
        uint32_t dynamicTilesPainted = 0; // Nonempty native entity tile paints.
        // Component timings from the application process, never synthetic FPS.
        // Retained as diagnostics for native traversal, terrain projection and
        // source-art collection; they do not drive a hidden quality approximation.
        float visibilityCpuMs = 0.0f;
        float terrainCpuMs = 0.0f;
        float paintCpuMs = 0.0f;
        float prepareCpuMs = 0.0f;
    };

    struct FirstPersonWallPlane
    {
        // Clockwise physical wall quad: lower A, lower B, upper B, upper A.
        std::array<FirstPersonVec3, 4> corners{};
    };
    // Authoritative full-tile wall geometry shared by rendering and walking
    // collision. Wall slope values are the native EDGE_SLOPE values stored in
    // WallElement (0, upwards, downwards).
    [[nodiscard]] FirstPersonWallPlane BuildFirstPersonWallPlane(
        CoordsXY tileOrigin, int32_t baseZ, uint8_t direction, uint8_t slope, int32_t height);

    [[nodiscard]] std::optional<FirstPersonProjection> ProjectFirstPersonPoint(
        const FirstPersonCamera& camera, const FirstPersonVec3& worldPoint,
        const ScreenSize& screenSize, float fieldOfViewDegrees = 70.0f,
        float nearClip = 2.0f);
    // A single authoritative park-state -> perspective-scene collector.
    FirstPersonScene CollectFirstPersonScene(
        const FirstPersonRenderOptions& options, const ScreenSize& dimensions);
    // Release derived terrain geometry on exiting POV or closing/reloading a park.
    void ClearFirstPersonSceneCache();
    // OpenRCT2 map redraw/invalidation points invalidate geometry/height bounds.
    // Both APIs are no-ops when POV is inactive and no cache exists.
    void InvalidateFirstPersonSceneTile(CoordsXY position);
    void InvalidateFirstPersonSceneRegion(CoordsXY minPosition, CoordsXY maxPosition);
    void RenderFirstPerson(Drawing::RenderTarget& rt, const FirstPersonRenderOptions& options);
} // namespace OpenRCT2::Paint

