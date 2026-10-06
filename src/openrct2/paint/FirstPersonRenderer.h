/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonMath.h"
#include "Paint.h"
#include "FirstPersonStreaming.h"
#include "../Identifiers.h"
#include "../drawing/ImageId.hpp"
#include "../interface/ScreenCoords.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <memory>
#include <vector>

namespace OpenRCT2::Drawing { struct RenderTarget; }
namespace OpenRCT2
{
    struct LargeSceneryElement;
    struct Vehicle;
}
struct PaintStruct;

namespace OpenRCT2::Paint
{
    enum class FirstPersonHiddenComponentDisposition : uint8_t
    {
        emit,
        suppressSelfContinueChain,
        suppressSubtree,
    };

    enum class FirstPersonDiagnosticCategory : uint32_t
    {
        horizontalArtwork = 1u << 0,
        horizontalPhysical = 1u << 1,
        horizontalMixed = 1u << 2,
        planes = 1u << 3,
    };

    inline constexpr uint32_t kFirstPersonDiagnosticDefaultMask =
        uint32_t(FirstPersonDiagnosticCategory::horizontalArtwork)
        | uint32_t(FirstPersonDiagnosticCategory::horizontalPhysical)
        | uint32_t(FirstPersonDiagnosticCategory::planes);
    inline constexpr uint32_t kFirstPersonDiagnosticAllMask =
        kFirstPersonDiagnosticDefaultMask
        | uint32_t(FirstPersonDiagnosticCategory::horizontalMixed);

    void SetFirstPersonDiagnosticMask(uint32_t mask);
    [[nodiscard]] uint32_t GetFirstPersonDiagnosticMask();
    // One-shot diagnostic request. The next prepared first-person scene casts
    // a ray through the centre of the view and prints the rendered hit plus
    // authoritative map/cache provenance. No work is done on ordinary frames.
    void RequestFirstPersonInspectorPick();

    // Perspective interaction is captured with the rendered surface, but tile
    // ownership is stored as a stale-safe locator rather than a TileElement*
    // that could outlive a map edit or an asynchronous reconstruction.
    struct FirstPersonInteractionTarget
    {
        ViewportInteractionItem interactionType =
            static_cast<ViewportInteractionItem>(0);
        CoordsXY mapPosition{};
        EntityId entity = EntityId::GetNull();
        uint16_t tileElementIndex = 0xFFFF;
        uint64_t tileSignature = 0;

        [[nodiscard]] bool hasValue() const
        {
            return static_cast<uint8_t>(interactionType) != 0;
        }
    };

    void RequestFirstPersonInteractionPick();
    [[nodiscard]] std::optional<FirstPersonInteractionTarget>
        ConsumeFirstPersonInteractionPick();
    [[nodiscard]] bool IsFirstPersonInteractionTargetCurrent(
        const FirstPersonInteractionTarget& target);

    [[nodiscard]] constexpr FirstPersonHiddenComponentDisposition
        FirstPersonHiddenComponentPolicy(
            bool matchesHiddenEntity, bool entityPainted,
            bool selectedTilePassenger)
    {
        if (!matchesHiddenEntity)
            return FirstPersonHiddenComponentDisposition::emit;
        if (entityPainted)
            return FirstPersonHiddenComponentDisposition::suppressSubtree;
        if (selectedTilePassenger)
            return FirstPersonHiddenComponentDisposition::suppressSelfContinueChain;
        return FirstPersonHiddenComponentDisposition::emit;
    }

    struct FirstPersonRenderOptions
    {
        FirstPersonCamera camera{};
        EntityId hiddenEntity = EntityId::GetNull();
        // A tile painter can use hiddenEntity only as an interaction owner
        // (Ferris wheel structure is the canonical example). Hide the attached
        // passenger component by seat identity without discarding that mechanism.
        uint8_t hiddenSeatIndex = 0xFF;
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
        // Nonzero palette index for semantic geometry that has no sprite
        // texture (for example trajectory-derived physical rails).
        uint8_t solidColour = 0;
        std::array<FirstPersonVertex, 6> triangles{}; // One quad: no heap allocation for every tile/sprite.
        // Cached upright sprites must always face the CURRENT camera: keep their
        // original native image-space offsets and world anchor, not old vertices.
        bool viewFacing = false;
        // Equal-depth ownership is logical paint evidence, not a geometry
        // offset. World-fixed native artwork can be replayed in authoritative
        // paint order against the frozen physical-depth result, so coincident
        // surfaces resolve deterministically without role-specific biases.
        bool coplanarOwner = false;
        // Diagnostic provenance for semantic material reconstruction. These
        // fields do not affect rendering or cache identity; they only explain
        // where a final plane/material came from when FP_PLANE is enabled.
        uint64_t diagnosticSemanticGroup = 0;
        CoordsXY diagnosticSourceTile{};
        CoordsXY diagnosticMaterialTile{};
        uint32_t diagnosticSourceComponent = 0;
        uint32_t diagnosticMaterialComponent = 0;
        uint32_t diagnosticSourceArtworkGroup = 0;
        uint32_t diagnosticMaterialArtworkGroup = 0;
        uint64_t diagnosticMaterialPaintOrdinal = 0;
        uint8_t diagnosticSourceRotation = 0xFF;
        uint8_t diagnosticMaterialRotation = 0xFF;
        uint8_t diagnosticSemanticRole = 0;
        // Visual coverage may deliberately follow known-solid world geometry
        // instead of sprite alpha (terrain, rails, structural faces). This is a
        // rendering/material rule only, never a collision flag: foliage and
        // other cutout artwork must leave this false so transparent pixels stay
        // transparent even when their collision/hull geometry is solid.
        bool physicalCoverage = false;
        // Non-physical appearance carriers follow authoritative geometry but
        // exist only to receive inverse-projected native artwork (for example
        // ties/cross-members around trajectory-derived rails).
        bool artworkCarrier = false;
        // When a face has no trustworthy projective source, retain physical
        // geometry but use only a deliberate material fallback from the sprite.
        bool textureFallbackOnly = false;
        // Some native sprite IDs (notably scrolling text) reference mutable
        // bitmap slots. Semantic reconstruction also bakes persistent per-face
        // textures from several native views. Both store immutable indexed
        // pixels here; persistentBitmap distinguishes resident face materials
        // from per-frame snapshot uploads.
        std::vector<uint8_t> immutablePixels;
        // Persistent baked materials are shared so static transparent faces can
        // enter the per-frame transparency list without copying their bitmap.
        std::shared_ptr<const std::vector<uint8_t>>
            persistentPixels;
        [[nodiscard]] bool hasImmutablePixelData() const
        {
            return persistentPixels != nullptr
                ? !persistentPixels->empty()
                : !immutablePixels.empty();
        }
        [[nodiscard]] const std::vector<uint8_t>&
            immutablePixelData() const
        {
            return persistentPixels != nullptr
                ? *persistentPixels
                : immutablePixels;
        }
        int16_t immutableWidth = 0;
        int16_t immutableHeight = 0;
        uint64_t immutableFingerprint = 0;
        bool persistentBitmap = false;
        // True when geometry/material was reconstructed from authoritative
        // geometry plus all available native observations and therefore does
        // not depend on the passenger-selected native source rotation.
        bool cameraIndependent = false;
        // An authored one-sided plane or a materially closed reconstructed hull
        // defines a visible half-space through its outward normal. Reconstructed
        // artwork must establish closure after baking, not from occupancy alone.
        bool exteriorOnly = false;
        FirstPersonVec3 outwardNormal{};
        // Post-bake coverage of a reconstructed hull. Zero boundary faces means
        // this surface has no hull-material certificate (e.g. an authored wall).
        uint32_t diagnosticHullBoundaryFaces = 0;
        uint32_t diagnosticHullMaterialFaces = 0;
        uint32_t diagnosticHullOpaqueFaces = 0;
        // Zero means tile-local/unconnected artwork. Nonzero identifies a
        // multi-tile reconstruction group whose native source rotation is
        // selected from one canonical object/track origin.
        uint64_t reconstructionGroup = 0;
        // Diagnostic-only provenance for grouped reconstruction. Kind uses
        // 0=none, 1=large scenery, 2=track, 3=grouped diagnostic fallback.
        // The anchor is the canonical physical group origin used to build the
        // reconstruction key.
        uint8_t diagnosticReconstructionKind = 0;
        FirstPersonVec3 diagnosticReconstructionAnchor{};
        // Inverse-reconstruction evidence retained for the click inspector.
        // These do not affect rendering or cache identity.
        bool diagnosticContinuousSurfaceRefined = false;
        bool diagnosticStructuralEvidenceUsed = false;
        uint32_t diagnosticStructuralPairedSamples = 0;
        uint32_t diagnosticStructuralBaselineMismatches = 0;
        uint32_t diagnosticStructuralSelectedMismatches = 0;
        uint64_t diagnosticVoxelRoundTripPixelDisagreement = 0;
        uint64_t diagnosticRoundTripPixelDisagreement = 0;
        // Stable native painter order for logical tie breaking. Physical depth
        // remains authoritative; this ordinal is consulted only when surfaces
        // occupy the same physical layer (opaque owners or transparency).
        uint64_t nativePaintOrdinal = 0;
        // Native left-click ownership follows the visible first-person surface.
        // Tile targets are validated against live map state before dispatch.
        FirstPersonInteractionTarget interactionTarget{};
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
        uint64_t sceneEpoch{};
        uint64_t generation{};
        FirstPersonVec3 center{};
        float radius{};
        const std::vector<FirstPersonSurface>* surfaces = nullptr;
        const std::vector<ImageIndex>* textureDependencies = nullptr;
        std::shared_ptr<const std::vector<FirstPersonSurface>> surfaceStorage{};
        uint64_t sourceRevision{};
        size_t vertexCount{};
    };

    struct FirstPersonScene
    {
        // Increments whenever the authoritative first-person world cache is
        // cleared (park reload/teardown). GPU-resident baked face textures use
        // this to drop assets that no longer belong to the current park.
        uint64_t sceneEpoch = 0;
        FirstPersonRenderOptions options{};
        FirstPersonResolvedView resolvedView{};
        ScreenSize dimensions{};
        ScreenCoordsXY screenOrigin{};
        // Nonzero only while the drawing engine is presenting one frame.
        // Every dirty clip for that presentation observes the same scene.
        uint64_t presentationFrameSerial = 0;
        // Dynamic, camera-facing, transparent and animated surfaces only.
        std::vector<FirstPersonSurface> surfaces;
        // Persistent fixed opaque geometry is submitted by region descriptor;
        // individual static surfaces are exposed only when that region rebuilds.
        std::vector<FirstPersonStaticRegion> staticRegions;
        std::vector<CoordsXY> visibleTiles;
        // Region keys belonging to track groups actually encountered during
        // this frame's visible-tile traversal. Carrying them forward avoids a
        // park-wide scan of the persistent trajectory cache during submission.
        std::vector<uint64_t> activeTrackRegions;
    };

    struct FirstPersonWallPlane
    {
        // Clockwise physical wall quad: lower A, lower B, upper B, upper A.
        std::array<FirstPersonVec3, 4> corners{};
    };
    // Stable native source view chosen from physical terrain geometry so every
    // non-degenerate triangle retains two-dimensional texture information.
    [[nodiscard]] uint8_t GetFirstPersonTerrainSourceRotation(uint8_t slope);
    // Paint ownership and paint provenance are separate: a tile painter may set
    // Entity solely so interaction points at a ride vehicle.
    [[nodiscard]] bool IsFirstPersonEntityPaintRoot(const ::PaintStruct& root);
    [[nodiscard]] bool FirstPersonVerticalTunnelCutsTerrain(
        int32_t terrainBaseZ, uint8_t verticalTunnelHeight);
    [[nodiscard]] constexpr bool
        FirstPersonTerrainRevalidationNeedsRegionRebuild(
            bool wasDirty, bool geometryChanged)
    {
        return wasDirty || geometryChanged;
    }

    [[nodiscard]] constexpr ScreenRect
        IntersectFirstPersonScreenRects(
            const ScreenRect& viewport, const ScreenRect& dirty)
    {
        const int32_t left = viewport.getLeft() > dirty.getLeft()
            ? viewport.getLeft() : dirty.getLeft();
        const int32_t top = viewport.getTop() > dirty.getTop()
            ? viewport.getTop() : dirty.getTop();
        const int32_t right = viewport.getRight() < dirty.getRight()
            ? viewport.getRight() : dirty.getRight();
        const int32_t bottom = viewport.getBottom() < dirty.getBottom()
            ? viewport.getBottom() : dirty.getBottom();
        return { left, top, right, bottom };
    }

    // Authoritative full-tile wall geometry shared by rendering and walking
    // collision. Wall slope values are the native EDGE_SLOPE values stored in
    // WallElement (0, upwards, downwards).
    [[nodiscard]] FirstPersonWallPlane BuildFirstPersonWallPlane(
        CoordsXY tileOrigin, int32_t baseZ, uint8_t direction, uint8_t slope, int32_t height);

    [[nodiscard]] std::optional<FirstPersonProjection> ProjectFirstPersonPoint(
        const FirstPersonCamera& camera, const FirstPersonVec3& worldPoint,
        const ScreenSize& screenSize, float fieldOfViewDegrees = 70.0f,
        float nearClip = 2.0f);
    [[nodiscard]] std::optional<PassengerPaintAnchor>
        CaptureFirstPersonPassengerPaintAnchor(
            const Vehicle& vehicle, uint8_t seatIndex);
    [[nodiscard]] std::optional<PassengerPaintAnchor>
        CaptureFirstPersonPassengerPaintAnchor(
            const Vehicle& vehicle, uint8_t seatIndex,
            const FirstPersonPassengerPaintInterpolation& interpolation);

    // A single authoritative park-state -> perspective-scene collector.
    FirstPersonScene CollectFirstPersonScene(
        const FirstPersonRenderOptions& options, const ScreenSize& dimensions);
    // One immutable prepared scene is shared by all dirty redraw clips in an
    // OpenGL presentation frame.
    void BeginFirstPersonPresentationFrame();
    void EndFirstPersonPresentationFrame();

    // End one POV presentation session without discarding the park-bounded
    // static world caches. Full clearing remains for park reload/teardown.
    void ResetFirstPersonPresentationCache();
    [[nodiscard]] std::optional<uint64_t>
        EnsureFirstPersonLargeSceneryPhysicalProxy(
            CoordsXY tile, const LargeSceneryElement& large);
    void ClearFirstPersonSceneCache();
    // OpenRCT2 map redraw/invalidation points invalidate geometry/height bounds.
    // Both APIs are no-ops when POV is inactive and no cache exists.
    void InvalidateFirstPersonSceneTile(CoordsXY position);
    void InvalidateFirstPersonSceneRegion(CoordsXY minPosition, CoordsXY maxPosition);
    void RenderFirstPerson(
        Drawing::RenderTarget& rt, const FirstPersonRenderOptions& options,
        const ScreenRect& viewport);
} // namespace OpenRCT2::Paint

