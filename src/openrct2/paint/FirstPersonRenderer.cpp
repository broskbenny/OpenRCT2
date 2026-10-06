/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#include "FirstPersonRenderer.h"
#include "FirstPersonSpriteSnapshot.h"
#include "FirstPersonAssetReconstruction.h"
#include "FirstPersonLargeSceneryReconstruction.h"
#include "FirstPersonPhysicalProxy.h"
#include "FirstPersonSmallSceneryAppearance.h"
#include "FirstPersonTrackTrajectory.h"
#include "FirstPersonTunnelGeometry.h"
#include "FirstPersonVehicleBodyHull.h"
#include "FirstPersonVehiclePose.h"
#include "FirstPersonWalkingSemantics.h"
#include "Paint.h"
#include "Paint.SessionFlags.h"
#include "tile_element/Paint.Surface.h"
#include "tile_element/Paint.Path.h"
#include "tile_element/Paint.TileElement.h"
#include "Paint.Entity.h"

#include "../Context.h"
#include "../Diagnostic.h"
#include "../core/Console.hpp"
#include "../GameState.h"
#include "../drawing/Drawing.Sprite.h"
#include "../drawing/Colour.h"
#include "../drawing/ColourMap.h"
#include "../drawing/PaletteIndex.h"
#include "../drawing/IDrawingContext.h"
#include "../drawing/IDrawingEngine.h"
#include "../drawing/RenderTarget.h"
#include "../drawing/ScrollingText.h"
#include "../entity/EntityBase.h"
#include "../entity/EntityTweener.h"
#include "../ride/CarEntry.h"
#include "../ride/Ride.h"
#include "../ride/RideData.h"
#include "../ride/RideEntry.h"
#include "../ride/TrackData.h"
#include "../ride/TrackDesign.h"
#include "../ride/TrackPaint.h"
#include "../ride/TrackIteration.h"
#include "../ride/Vehicle.h"
#include "../ride/ted/TrackElementDescriptor.h"
#include "../interface/Viewport.h"
#include "../profiling/Profiling.h"
#include "../world/Footpath.h"
#include "../world/Map.h"
#include "../world/MapAnimation.h"
#include "../world/TileInspector.h"
#include "../world/Wall.h"
#include "../ride/Track.h"
#include "../world/tile_element/SurfaceElement.h"
#include "../world/tile_element/EntranceElement.h"
#include "../world/tile_element/PathElement.h"
#include "../world/tile_element/TrackElement.h"
#include "../world/tile_element/SmallSceneryElement.h"
#include "../world/tile_element/LargeSceneryElement.h"
#include "../world/tile_element/Slope.h"
#include "../world/tile_element/TileElement.h"
#include "../world/tile_element/TileElementType.h"
#include "../world/tile_element/WallElement.h"
#include "../object/WallSceneryEntry.h"
#include "../object/Object.h"
#include "../object/ObjectList.h"
#include "../object/EntranceObject.h"
#include "../object/ObjectManager.h"
#include "../object/LargeSceneryEntry.h"
#include "../object/SmallSceneryEntry.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace OpenRCT2::Paint
{
    namespace
    {
        uint32_t gFirstPersonDiagnosticMask =
            kFirstPersonDiagnosticDefaultMask;
        uint64_t gFirstPersonDiagnosticGeneration = 1;
        bool gFirstPersonInspectorPickRequested = false;
        bool gFirstPersonInteractionPickRequested = false;
        std::optional<FirstPersonInteractionTarget>
            gFirstPersonInteractionPickResult;

        [[nodiscard]] std::string
            FirstPersonDiagnosticImageLabel(uint32_t imageIndex)
        {
            std::string label = std::to_string(imageIndex);
            auto* context = GetContext();
            if (context == nullptr)
                return label;

            auto& objectManager = context->GetObjectManager();
            for (const auto objectType : getAllObjectTypes())
            {
                const size_t limit =
                    getObjectTypeLimit(objectType);
                for (size_t index = 0;
                     index < limit; ++index)
                {
                    auto* object =
                        objectManager.GetLoadedObject(
                            objectType, index);
                    if (object == nullptr)
                        continue;

                    const ImageIndex base =
                        object->GetBaseImageId();
                    if (base == kImageIndexUndefined)
                        continue;
                    const uint64_t begin = base;
                    const uint64_t end =
                        begin + object->GetNumImages();
                    if (uint64_t(imageIndex) < begin
                        || uint64_t(imageIndex) >= end)
                        continue;

                    const auto identifier =
                        object->GetIdentifier();
                    const auto name = object->GetName();
                    label += "{";
                    if (!identifier.empty())
                        label += identifier;
                    else
                        label +=
                            object->GetDescriptor().ToString();
                    if (!name.empty())
                    {
                        label += "|";
                        label += name;
                    }
                    label += "+";
                    label += std::to_string(
                        uint64_t(imageIndex) - begin);
                    label += "}";
                    return label;
                }
            }
            return label;
        }

        #include "FirstPersonRenderer.Core.inc"
        #include "FirstPersonRenderer.World.inc"
        #include "FirstPersonRenderer.Paint.inc"
    } // namespace

    void SetFirstPersonDiagnosticMask(uint32_t mask)
    {
        mask &= kFirstPersonDiagnosticAllMask;
        if (gFirstPersonDiagnosticMask == mask)
            return;
        gFirstPersonDiagnosticMask = mask;
        ++gFirstPersonDiagnosticGeneration;
        if (gFirstPersonDiagnosticGeneration == 0)
            gFirstPersonDiagnosticGeneration = 1;
    }

    uint32_t GetFirstPersonDiagnosticMask()
    {
        return gFirstPersonDiagnosticMask;
    }

    void RequestFirstPersonInspectorPick()
    {
        gFirstPersonInspectorPickRequested = true;
    }

    void RequestFirstPersonInteractionPick()
    {
        gFirstPersonInteractionPickRequested = true;
        gFirstPersonInteractionPickResult.reset();
    }

    std::optional<FirstPersonInteractionTarget>
        ConsumeFirstPersonInteractionPick()
    {
        auto result =
            gFirstPersonInteractionPickResult;
        gFirstPersonInteractionPickResult.reset();
        return result;
    }

    bool IsFirstPersonInteractionTargetCurrent(
        const FirstPersonInteractionTarget& target)
    {
        if (!target.hasValue())
            return false;
        if (target.interactionType
            == ViewportInteractionItem::entity)
        {
            return !target.entity.IsNull()
                && getGameState().entities
                    .tryGetEntity<EntityBase>(
                        target.entity) != nullptr;
        }

        if (target.tileElementIndex == 0xFFFF
            || !MapIsLocationValid(
                target.mapPosition))
            return false;
        if (NativeTileSignature(
                target.mapPosition)
            != target.tileSignature)
            return false;

        auto* element =
            MapGetFirstElementAt(
                target.mapPosition);
        for (uint16_t index = 0;
             element != nullptr
                 && index
                     < target.tileElementIndex;
             ++index)
        {
            if (element->isLastForTile())
                return false;
            ++element;
        }
        return element != nullptr
            && !element->isGhost()
            && !element->isInvisible();
    }

    namespace
    {
        struct FirstPersonInspectorHit
        {
            const FirstPersonSurface* surface = nullptr;
            const FirstPersonStaticRegion* region = nullptr;
            size_t surfaceIndex = 0;
            float distance = 0.0f;
            float u = 0.0f;
            float v = 0.0f;
            FirstPersonVec3 point{};
        };

        [[nodiscard]] bool IntersectFirstPersonInspectorTriangle(
            const FirstPersonVec3& origin, const FirstPersonVec3& direction,
            const FirstPersonVertex& a, const FirstPersonVertex& b,
            const FirstPersonVertex& c, float& distance, float& u, float& v)
        {
            constexpr float kEpsilon = 0.00001f;
            const FirstPersonVec3 edge1{
                b.world.x - a.world.x,
                b.world.y - a.world.y,
                b.world.z - a.world.z,
            };
            const FirstPersonVec3 edge2{
                c.world.x - a.world.x,
                c.world.y - a.world.y,
                c.world.z - a.world.z,
            };
            const auto p = FpCross(direction, edge2);
            const float determinant = FpDot(edge1, p);
            if (std::abs(determinant) <= kEpsilon)
                return false;

            const float inverse = 1.0f / determinant;
            const FirstPersonVec3 fromA{
                origin.x - a.world.x,
                origin.y - a.world.y,
                origin.z - a.world.z,
            };
            const float baryB = FpDot(fromA, p) * inverse;
            if (baryB < -kEpsilon || baryB > 1.0f + kEpsilon)
                return false;

            const auto q = FpCross(fromA, edge1);
            const float baryC = FpDot(direction, q) * inverse;
            if (baryC < -kEpsilon || baryB + baryC > 1.0f + kEpsilon)
                return false;

            const float t = FpDot(edge2, q) * inverse;
            if (t <= kEpsilon)
                return false;

            const float baryA = 1.0f - baryB - baryC;
            distance = t;
            u = baryA * a.u + baryB * b.u + baryC * c.u;
            v = baryA * a.v + baryB * b.v + baryC * c.v;
            return true;
        }

        [[nodiscard]] bool IntersectFirstPersonInspectorSurface(
            const FirstPersonSurface& surface, const FirstPersonVec3& origin,
            const FirstPersonVec3& direction, FirstPersonInspectorHit& hit)
        {
            bool found = false;
            float bestDistance = std::numeric_limits<float>::max();
            float bestU = 0.0f;
            float bestV = 0.0f;
            for (const size_t first : { size_t(0), size_t(3) })
            {
                float distance = 0.0f;
                float u = 0.0f;
                float v = 0.0f;
                if (!IntersectFirstPersonInspectorTriangle(
                        origin, direction,
                        surface.triangles[first],
                        surface.triangles[first + 1],
                        surface.triangles[first + 2],
                        distance, u, v))
                    continue;
                if (distance < bestDistance)
                {
                    found = true;
                    bestDistance = distance;
                    bestU = u;
                    bestV = v;
                }
            }
            if (!found)
                return false;

            const FirstPersonVec3 point{
                origin.x + direction.x * bestDistance,
                origin.y + direction.y * bestDistance,
                origin.z + direction.z * bestDistance,
            };
            if (surface.exteriorOnly)
            {
                const FirstPersonVec3 towardEye{
                    origin.x - point.x,
                    origin.y - point.y,
                    origin.z - point.z,
                };
                if (FpDot(surface.outwardNormal, towardEye) <= 0.0f)
                    return false;
            }

            hit.surface = &surface;
            hit.distance = bestDistance;
            hit.u = bestU;
            hit.v = bestV;
            hit.point = point;
            return true;
        }

        [[nodiscard]] bool SampleFirstPersonInspectorPixels(
            const std::vector<uint8_t>& pixels, int32_t width, int32_t height,
            float u, float v, uint8_t& pixel)
        {
            const int32_t x = int32_t(std::floor(u));
            const int32_t y = int32_t(std::floor(v));
            if (x < 0 || y < 0 || x >= width || y >= height
                || width <= 0 || height <= 0)
                return false;
            const size_t index =
                size_t(y) * size_t(width) + size_t(x);
            if (index >= pixels.size())
                return false;
            pixel = pixels[index];
            return true;
        }

        [[nodiscard]] bool FirstPersonInspectorSurfaceVisibleAtHit(
            const FirstPersonInspectorHit& hit)
        {
            const auto& surface = *hit.surface;
            if (surface.solidColour != 0)
                return true;
            if (!surface.image.HasValue())
                return false;

            bool colourInside = false;
            uint8_t colourPixel = 0;
            if (surface.hasImmutablePixelData())
            {
                colourInside = SampleFirstPersonInspectorPixels(
                    surface.immutablePixelData(),
                    surface.immutableWidth, surface.immutableHeight,
                    hit.u, hit.v, colourPixel);
            }
            else if (const auto* g1 = GfxGetG1Element(surface.image);
                     g1 != nullptr)
            {
                const auto pixels =
                    DecodeFirstPersonSpritePixels(*g1);
                if (!pixels.has_value())
                    return false;
                colourInside = SampleFirstPersonInspectorPixels(
                    *pixels, g1->width, g1->height,
                    hit.u, hit.v, colourPixel);
            }
            else
            {
                return false;
            }

            if (!colourInside && !surface.physicalCoverage)
                return false;
            if (colourInside && colourPixel == 0
                && !surface.physicalCoverage)
                return false;

            if (surface.mask.HasValue())
            {
                const auto* mask = GfxGetG1Element(surface.mask);
                if (mask == nullptr)
                    return false;
                const auto maskPixels =
                    DecodeFirstPersonSpritePixels(*mask);
                if (!maskPixels.has_value())
                    return false;
                uint8_t maskPixel = 0;
                if (!SampleFirstPersonInspectorPixels(
                        *maskPixels, mask->width, mask->height,
                        hit.u, hit.v, maskPixel)
                    || maskPixel == 0)
                    return false;
            }
            return true;
        }

        [[nodiscard]] const char*
            FirstPersonInspectorWalkabilityKindName(
                FirstPersonWalkabilityKind kind)
        {
            switch (kind)
            {
                case FirstPersonWalkabilityKind::path:
                    return "path";
                case FirstPersonWalkabilityKind::queue:
                    return "queue";
                case FirstPersonWalkabilityKind::parkEntrance:
                    return "parkEntrance";
                case FirstPersonWalkabilityKind::rideEntrance:
                    return "rideEntrance";
                case FirstPersonWalkabilityKind::rideExit:
                    return "rideExit";
                case FirstPersonWalkabilityKind::trackPortal:
                    return "trackPortal";
                case FirstPersonWalkabilityKind::none:
                default:
                    return "none";
            }
        }

        [[nodiscard]] const char* FirstPersonInspectorTileElementName(
            TileElementType type)
        {
            switch (type)
            {
                case TileElementType::surface:
                    return "surface";
                case TileElementType::path:
                    return "path";
                case TileElementType::track:
                    return "track";
                case TileElementType::smallScenery:
                    return "smallScenery";
                case TileElementType::entrance:
                    return "entrance";
                case TileElementType::wall:
                    return "wall";
                case TileElementType::largeScenery:
                    return "largeScenery";
                case TileElementType::banner:
                    return "banner";
                default:
                    return "unknown";
            }
        }

        void DumpFirstPersonInspectorSurface(
            const FirstPersonInspectorHit& hit, const char* label,
            bool detailed)
        {
            const auto& surface = *hit.surface;
            Console::WriteLine(
                "%s distance=%.3f point=(%.3f,%.3f,%.3f) uv=(%.3f,%.3f)",
                label, double(hit.distance),
                double(hit.point.x), double(hit.point.y),
                double(hit.point.z), double(hit.u), double(hit.v));
            const auto imageLabel =
                surface.image.HasValue()
                ? FirstPersonDiagnosticImageLabel(
                    surface.image.GetIndex())
                : std::string("none");
            const auto maskLabel =
                surface.mask.HasValue()
                ? FirstPersonDiagnosticImageLabel(
                    surface.mask.GetIndex())
                : std::string("none");
            Console::WriteLine(
                "  source=%s surfaceIndex=%zu region=%llu image=%s mask=%s",
                hit.region != nullptr ? "staticRegion" : "streamed",
                hit.surfaceIndex,
                hit.region != nullptr
                    ? static_cast<unsigned long long>(hit.region->key)
                    : 0ull,
                imageLabel.c_str(),
                maskLabel.c_str());
            Console::WriteLine(
                "  gpuRegion=%llu cameraIndependent=%d viewFacing=%d physicalCoverage=%d "
                "coplanarOwner=%d nativePaintOrdinal=%llu exteriorOnly=%d",
                static_cast<unsigned long long>(surface.gpuRegion),
                surface.cameraIndependent ? 1 : 0,
                surface.viewFacing ? 1 : 0,
                surface.physicalCoverage ? 1 : 0,
                surface.coplanarOwner ? 1 : 0,
                static_cast<unsigned long long>(
                    surface.nativePaintOrdinal),
                surface.exteriorOnly ? 1 : 0);
            Console::WriteLine(
                "  solidColour=%u persistentBitmap=%d immutableFingerprint=%llu "
                "reconstructionGroup=%llu reconstructionKind=%u",
                unsigned(surface.solidColour),
                surface.persistentBitmap ? 1 : 0,
                static_cast<unsigned long long>(
                    surface.immutableFingerprint),
                static_cast<unsigned long long>(
                    surface.reconstructionGroup),
                unsigned(surface.diagnosticReconstructionKind));
            if (surface.diagnosticReconstructionKind == 1)
            {
                Console::WriteLine(
                    "  reconstructionEvidence continuous=%d structural=%d "
                    "structuralSamples=%u structuralMismatch=%u->%u "
                    "roundTripPixels=%llu->%llu",
                    surface.diagnosticContinuousSurfaceRefined ? 1 : 0,
                    surface.diagnosticStructuralEvidenceUsed ? 1 : 0,
                    surface.diagnosticStructuralPairedSamples,
                    surface.diagnosticStructuralBaselineMismatches,
                    surface.diagnosticStructuralSelectedMismatches,
                    static_cast<unsigned long long>(
                        surface.diagnosticVoxelRoundTripPixelDisagreement),
                    static_cast<unsigned long long>(
                        surface.diagnosticRoundTripPixelDisagreement));
            }
            Console::WriteLine(
                "  semanticRole=%u semanticGroup=%llu sourceTile=(%d,%d) "
                "materialTile=(%d,%d) sourceComponent=%u materialComponent=%u",
                unsigned(surface.diagnosticSemanticRole),
                static_cast<unsigned long long>(
                    surface.diagnosticSemanticGroup),
                surface.diagnosticSourceTile.x,
                surface.diagnosticSourceTile.y,
                surface.diagnosticMaterialTile.x,
                surface.diagnosticMaterialTile.y,
                surface.diagnosticSourceComponent,
                surface.diagnosticMaterialComponent);
            if (surface.diagnosticHullBoundaryFaces != 0)
            {
                Console::WriteLine(
                    "  hullMaterial=%s boundaryFaces=%u materialFaces=%u opaqueFaces=%u",
                    surface.exteriorOnly ? "closed-shell" : "open-artwork",
                    surface.diagnosticHullBoundaryFaces,
                    surface.diagnosticHullMaterialFaces,
                    surface.diagnosticHullOpaqueFaces);
            }
            Console::WriteLine(
                "  outwardNormal=(%.3f,%.3f,%.3f) semanticBounds=%d "
                "semanticCenter=(%.3f,%.3f,%.3f) semanticRadius=%.3f",
                double(surface.outwardNormal.x),
                double(surface.outwardNormal.y),
                double(surface.outwardNormal.z),
                surface.hasSemanticBounds ? 1 : 0,
                double(surface.semanticCenter.x),
                double(surface.semanticCenter.y),
                double(surface.semanticCenter.z),
                double(surface.semanticRadius));

            if (hit.region != nullptr)
            {
                Console::WriteLine(
                    "  region sceneEpoch=%llu generation=%llu sourceRevision=%llu "
                    "vertexCount=%zu storageSurfaces=%zu",
                    static_cast<unsigned long long>(
                        hit.region->sceneEpoch),
                    static_cast<unsigned long long>(
                        hit.region->generation),
                    static_cast<unsigned long long>(
                        hit.region->sourceRevision),
                    hit.region->vertexCount,
                    hit.region->surfaceStorage != nullptr
                        ? hit.region->surfaceStorage->size() : 0);
                const auto packet =
                    _staticRegionPackets.find(hit.region->key);
                if (packet != _staticRegionPackets.end())
                {
                    Console::WriteLine(
                        "  packet dirty=%d generation=%llu sourceRevision=%llu "
                        "publishedSourceRevision=%llu vertexCount=%zu publishedSurfaces=%zu",
                        packet->second.dirty ? 1 : 0,
                        static_cast<unsigned long long>(
                            packet->second.generation),
                        static_cast<unsigned long long>(
                            packet->second.sourceRevision),
                        static_cast<unsigned long long>(
                            packet->second.publishedSourceRevision),
                        packet->second.vertexCount,
                        packet->second.surfaceStorage != nullptr
                            ? packet->second.surfaceStorage->size() : 0);
                }
            }

            if (!detailed)
                return;
            for (size_t i = 0; i < surface.triangles.size(); ++i)
            {
                const auto& vertex = surface.triangles[i];
                Console::WriteLine(
                    "  vertex[%zu] world=(%.3f,%.3f,%.3f) uv=(%.3f,%.3f)",
                    i, double(vertex.world.x), double(vertex.world.y),
                    double(vertex.world.z), double(vertex.u),
                    double(vertex.v));
            }
        }

        void DumpFirstPersonInspectorTileCache(
            int32_t tileX, int32_t tileY)
        {
            const auto key = TerrainKey(tileX, tileY);
            Console::WriteLine(
                "--- FP_PICK CACHE tile=(%d,%d) key=%llu ---",
                tileX, tileY,
                static_cast<unsigned long long>(key));

            const auto terrain = _terrainCache.entries.find(key);
            if (terrain == _terrainCache.entries.end())
            {
                Console::WriteLine("  terrainCache=absent");
            }
            else
            {
                const auto& cached = terrain->second;
                Console::WriteLine(
                    "  terrainCache dirty=%d baseZ=%d waterZ=%d slope=%u "
                    "sourceRotation=%u surfaceStyle=%u verticalOpening=%d "
                    "groundRegion=%llu water=%zu overlay=%zu",
                    cached.dirty ? 1 : 0,
                    cached.baseZ, cached.waterZ,
                    unsigned(cached.slope),
                    unsigned(cached.sourceRotation),
                    unsigned(cached.surfaceStyle),
                    cached.verticalOpening ? 1 : 0,
                    static_cast<unsigned long long>(
                        cached.ground.gpuRegion),
                    cached.water.size(),
                    cached.waterOverlay.size());
            }

            const auto paint = _staticPaintCache.find(key);
            if (paint == _staticPaintCache.end())
            {
                Console::WriteLine("  staticPaintCache=absent");
            }
            else
            {
                const auto& cached = paint->second;
                Console::WriteLine(
                    "  staticPaint valid=%d dirty=%d animated=%d sourceRevision=%llu "
                    "hasSelectedRotation=%d selectedRotation=%u visibilityDirty=%d "
                    "reconstructionGroups=%zu",
                    cached.valid ? 1 : 0,
                    cached.dirty ? 1 : 0,
                    cached.animated ? 1 : 0,
                    static_cast<unsigned long long>(
                        cached.sourceRevision),
                    cached.hasSelectedRotation ? 1 : 0,
                    unsigned(cached.selectedRotation),
                    cached.visibilityDirty ? 1 : 0,
                    cached.reconstructionGroups.size());
                Console::WriteLine(
                    "  staticPaint cameraIndependentResident=%zu "
                    "cameraIndependentStreamed=%zu semanticResident=%zu "
                    "semanticStreamed=%zu movingSemantic=%zu",
                    cached.cameraIndependentResidentSurfaces.size(),
                    cached.cameraIndependentStreamedSurfaces.size(),
                    cached.semanticResidentSurfaces.size(),
                    cached.semanticStreamedSurfaces.size(),
                    cached.semanticMovingMeshes.size());
                for (size_t rotation = 0;
                     rotation < cached.rotations.size(); ++rotation)
                {
                    const auto& variant =
                        cached.rotations[rotation];
                    Console::WriteLine(
                        "  rotation[%zu] valid=%d semanticValid=%d "
                        "resident=%zu streamed=%zu residentFingerprint=%llu "
                        "semanticFingerprint=%llu",
                        rotation,
                        variant.valid ? 1 : 0,
                        variant.semanticValid ? 1 : 0,
                        variant.residentSurfaces.size(),
                        variant.streamedSurfaces.size(),
                        static_cast<unsigned long long>(
                            variant.residentFingerprint),
                        static_cast<unsigned long long>(
                            variant.semanticFingerprint));
                }
            }

            const auto regionKey =
                FirstPersonGpuRegionKey(tileX, tileY);
            const auto packet =
                _staticRegionPackets.find(regionKey);
            if (packet == _staticRegionPackets.end())
            {
                Console::WriteLine(
                    "  regionPacket region=%llu absent",
                    static_cast<unsigned long long>(regionKey));
            }
            else
            {
                Console::WriteLine(
                    "  regionPacket region=%llu dirty=%d generation=%llu "
                    "sourceRevision=%llu publishedSourceRevision=%llu "
                    "vertexCount=%zu publishedSurfaces=%zu",
                    static_cast<unsigned long long>(regionKey),
                    packet->second.dirty ? 1 : 0,
                    static_cast<unsigned long long>(
                        packet->second.generation),
                    static_cast<unsigned long long>(
                        packet->second.sourceRevision),
                    static_cast<unsigned long long>(
                        packet->second.publishedSourceRevision),
                    packet->second.vertexCount,
                    packet->second.surfaceStorage != nullptr
                        ? packet->second.surfaceStorage->size() : 0);
            }
        }

        void DumpFirstPersonInspectorMapTile(
            int32_t tileX, int32_t tileY, const char* reason)
        {
            const CoordsXY tile{
                tileX * kCoordsXYStep,
                tileY * kCoordsXYStep,
            };
            if (!MapIsLocationValid(tile))
                return;

            Console::WriteLine(
                "--- FP_PICK MAP tile=(%d,%d) world=(%d,%d) reason=%s ---",
                tileX, tileY, tile.x, tile.y, reason);
            auto* element = MapGetFirstElementAt(tile);
            if (element == nullptr)
            {
                Console::WriteLine("  no tile elements");
                return;
            }

            size_t index = 0;
            while (true)
            {
                const auto type = element->getType();
                Console::WriteLine(
                    "  element[%zu] type=%s(%u) baseZ=%d clearanceZ=%d "
                    "direction=%u quadrants=0x%02x ghost=%d invisible=%d",
                    index,
                    FirstPersonInspectorTileElementName(type),
                    unsigned(type),
                    element->getBaseZ(),
                    element->getClearanceZ(),
                    unsigned(element->getDirection()),
                    unsigned(element->getOccupiedQuadrants()),
                    element->isGhost() ? 1 : 0,
                    element->isInvisible() ? 1 : 0);

                switch (type)
                {
                    case TileElementType::surface:
                    {
                        const auto* surface =
                            element->asSurface();
                        Console::WriteLine(
                            "    terrain slope=0x%02x waterHeight=%d surfaceObject=%u "
                            "edgeObject=%u grass=%u fences=0x%02x",
                            unsigned(surface->getSlope()),
                            surface->getWaterHeight(),
                            unsigned(surface->getSurfaceObjectIndex()),
                            unsigned(surface->getEdgeObjectIndex()),
                            unsigned(surface->getGrassLength()),
                            unsigned(surface->getParkFences()));
                        break;
                    }
                    case TileElementType::path:
                    {
                        const auto* path = element->asPath();
                        Console::WriteLine(
                            "    path surface=%u railings=%u edges=0x%02x corners=0x%02x "
                            "sloped=%d slopeDirection=%u queue=%d wide=%d addition=%u",
                            unsigned(path->getSurfaceEntryIndex()),
                            unsigned(path->getRailingsEntryIndex()),
                            unsigned(path->getEdges()),
                            unsigned(path->getCorners()),
                            path->isSloped() ? 1 : 0,
                            unsigned(path->getSlopeDirection()),
                            path->isQueue() ? 1 : 0,
                            path->isWide() ? 1 : 0,
                            unsigned(path->getAddition()));
                        break;
                    }
                    case TileElementType::track:
                    {
                        const auto* track = element->asTrack();
                        Console::WriteLine(
                            "    track type=%u sequence=%u ride=%u rideType=%u "
                            "station=%u chain=%d inverted=%d cableLift=%d",
                            unsigned(track->getTrackType()),
                            unsigned(track->getSequenceIndex()),
                            unsigned(track->getRideIndex().ToUnderlying()),
                            unsigned(track->getRideType()),
                            unsigned(track->getStationIndex().ToUnderlying()),
                            track->hasChain() ? 1 : 0,
                            track->isInverted() ? 1 : 0,
                            track->hasCableLift() ? 1 : 0);
                        break;
                    }
                    case TileElementType::smallScenery:
                    {
                        const auto* scenery =
                            element->asSmallScenery();
                        Console::WriteLine(
                            "    smallScenery entry=%u quadrant=%u age=%u needsSupports=%d",
                            unsigned(scenery->getEntryIndex()),
                            unsigned(scenery->getSceneryQuadrant()),
                            unsigned(scenery->getAge()),
                            scenery->needsSupports() ? 1 : 0);
                        break;
                    }
                    case TileElementType::entrance:
                    {
                        const auto* entrance =
                            element->asEntrance();
                        Console::WriteLine(
                            "    entrance type=%u sequence=%u directions=0x%x "
                            "entry=%u surface=%u ride=%u station=%u",
                            unsigned(entrance->getEntranceType()),
                            unsigned(entrance->getSequenceIndex()),
                            unsigned(entrance->getDirections()),
                            unsigned(entrance->getEntryIndex()),
                            unsigned(entrance->getSurfaceEntryIndex()),
                            unsigned(entrance->getRideIndex().ToUnderlying()),
                            unsigned(entrance->getStationIndex().ToUnderlying()));
                        break;
                    }
                    case TileElementType::wall:
                    {
                        const auto* wall = element->asWall();
                        Console::WriteLine(
                            "    wall entry=%u slope=%u animationFrame=%u animating=%d acrossTrack=%d",
                            unsigned(wall->getEntryIndex()),
                            unsigned(wall->getSlope()),
                            unsigned(wall->getAnimationFrame()),
                            wall->isAnimating() ? 1 : 0,
                            wall->isAcrossTrack() ? 1 : 0);
                        break;
                    }
                    case TileElementType::largeScenery:
                    {
                        const auto* scenery =
                            element->asLargeScenery();
                        Console::WriteLine(
                            "    largeScenery entry=%u sequence=%u",
                            unsigned(scenery->getEntryIndex()),
                            unsigned(scenery->getSequenceIndex()));
                        break;
                    }
                    default:
                        break;
                }

                const bool last = element->isLastForTile();
                ++index;
                if (last)
                    break;
                ++element;
            }

            const auto walkability =
                CollectFirstPersonWalkabilityConstraints(
                    tile);
            Console::WriteLine(
                "--- FP_PICK WALKABILITY count=%zu ---",
                walkability.size());
            for (size_t i = 0;
                 i < walkability.size(); ++i)
            {
                const auto& constraint =
                    walkability[i];
                Console::WriteLine(
                    "  constraint[%zu] kind=%s baseZ=%d sides=0x%02x "
                    "corners=0x%02x floor=%d through=%d visualFullDeck=%d "
                    "visualEdges=0x%02x sloped=%d slopeDirection=%u wide=%d",
                    i,
                    FirstPersonInspectorWalkabilityKindName(
                        constraint.kind),
                    constraint.baseZ,
                    unsigned(constraint.connectedSides),
                    unsigned(constraint.corners),
                    constraint.walkableFloor ? 1 : 0,
                    constraint.guaranteedThroughPassage ? 1 : 0,
                    constraint.visualFullTileDeck ? 1 : 0,
                    unsigned(constraint.visualDeckEdgeMask),
                    constraint.sloped ? 1 : 0,
                    unsigned(constraint.slopeDirection),
                    constraint.wide ? 1 : 0);
            }

            DumpFirstPersonInspectorTileCache(tileX, tileY);
        }

        void DumpFirstPersonInspectorRelevantTiles(
            const FirstPersonVec3& point)
        {
            const int32_t tileX =
                int32_t(std::floor(
                    point.x / float(kCoordsXYStep)));
            const int32_t tileY =
                int32_t(std::floor(
                    point.y / float(kCoordsXYStep)));
            DumpFirstPersonInspectorMapTile(
                tileX, tileY, "hit");

            constexpr float kBoundaryTolerance = 0.5f;
            const float localX =
                point.x - float(tileX * kCoordsXYStep);
            const float localY =
                point.y - float(tileY * kCoordsXYStep);
            if (localX <= kBoundaryTolerance)
                DumpFirstPersonInspectorMapTile(
                    tileX - 1, tileY, "hit -X boundary");
            if (localX >=
                float(kCoordsXYStep) - kBoundaryTolerance)
                DumpFirstPersonInspectorMapTile(
                    tileX + 1, tileY, "hit +X boundary");
            if (localY <= kBoundaryTolerance)
                DumpFirstPersonInspectorMapTile(
                    tileX, tileY - 1, "hit -Y boundary");
            if (localY >=
                float(kCoordsXYStep) - kBoundaryTolerance)
                DumpFirstPersonInspectorMapTile(
                    tileX, tileY + 1, "hit +Y boundary");
        }

        struct FirstPersonInspectorSelection
        {
            std::vector<FirstPersonInspectorHit> visible;
            size_t primaryIndex = 0;
        };

        [[nodiscard]] std::optional<
            FirstPersonInspectorSelection>
            SelectFirstPersonInspectorHit(
                const FirstPersonScene& scene)
        {
            const auto& camera =
                scene.resolvedView.camera;
            const auto direction =
                GetFirstPersonBasis(camera).forward;

            std::vector<FirstPersonInspectorHit>
                intersections;
            intersections.reserve(
                scene.surfaces.size()
                + scene.staticRegions.size() * 32);

            for (size_t i = 0;
                 i < scene.surfaces.size(); ++i)
            {
                FirstPersonInspectorHit hit{};
                if (!IntersectFirstPersonInspectorSurface(
                        scene.surfaces[i],
                        camera.position, direction, hit)
                    || hit.distance
                        > scene.resolvedView.farClip)
                    continue;
                hit.surfaceIndex = i;
                intersections.emplace_back(hit);
            }

            for (const auto& region :
                 scene.staticRegions)
            {
                if (region.surfaceStorage == nullptr)
                    continue;
                const auto& surfaces =
                    *region.surfaceStorage;
                for (size_t i = 0;
                     i < surfaces.size(); ++i)
                {
                    FirstPersonInspectorHit hit{};
                    if (!IntersectFirstPersonInspectorSurface(
                            surfaces[i],
                            camera.position, direction, hit)
                        || hit.distance
                            > scene.resolvedView.farClip)
                        continue;
                    hit.region = &region;
                    hit.surfaceIndex = i;
                    intersections.emplace_back(hit);
                }
            }

            std::sort(
                intersections.begin(),
                intersections.end(),
                [](const auto& a, const auto& b) {
                    if (a.distance != b.distance)
                        return a.distance < b.distance;
                    if (a.surface->coplanarOwner
                        != b.surface->coplanarOwner)
                        return a.surface->coplanarOwner;
                    return a.surface->nativePaintOrdinal
                        > b.surface->nativePaintOrdinal;
                });

            FirstPersonInspectorSelection selection;
            constexpr float kCoplanarBand = 0.25f;
            float firstVisibleDistance =
                std::numeric_limits<float>::max();
            for (const auto& hit : intersections)
            {
                if (firstVisibleDistance
                        != std::numeric_limits<float>::max()
                    && hit.distance
                        > firstVisibleDistance
                            + kCoplanarBand)
                    break;
                if (!FirstPersonInspectorSurfaceVisibleAtHit(
                        hit))
                    continue;
                if (firstVisibleDistance
                    == std::numeric_limits<float>::max())
                    firstVisibleDistance = hit.distance;
                selection.visible.push_back(hit);
            }

            if (selection.visible.empty())
                return std::nullopt;

            for (size_t i = 1;
                 i < selection.visible.size(); ++i)
            {
                if (std::abs(
                        selection.visible[i].distance
                        - firstVisibleDistance)
                    > kCoplanarBand)
                    continue;
                const auto& candidate =
                    *selection.visible[i].surface;
                const auto& primary =
                    *selection.visible[
                        selection.primaryIndex].surface;
                if (candidate.coplanarOwner
                    && (!primary.coplanarOwner
                        || candidate.nativePaintOrdinal
                            > primary.nativePaintOrdinal))
                {
                    selection.primaryIndex = i;
                }
            }
            return selection;
        }

        void DumpFirstPersonInspectorPick(
            const FirstPersonScene& scene)
        {
            const auto& camera =
                scene.resolvedView.camera;
            const auto direction =
                GetFirstPersonBasis(camera).forward;
            const auto selection =
                SelectFirstPersonInspectorHit(scene);

            Console::WriteLine("========== FP_PICK ==========");
            Console::WriteLine(
                "camera=(%.3f,%.3f,%.3f) ray=(%.6f,%.6f,%.6f) "
                "sceneSurfaces=%zu staticRegions=%zu",
                double(camera.position.x),
                double(camera.position.y),
                double(camera.position.z),
                double(direction.x), double(direction.y),
                double(direction.z),
                scene.surfaces.size(),
                scene.staticRegions.size());

            if (!selection.has_value())
            {
                Console::WriteLine(
                    "no visible first-person surface intersects the centre ray");
                Console::WriteLine("=============================");
                return;
            }

            const auto& visible = selection->visible;
            const size_t primaryIndex =
                selection->primaryIndex;
            constexpr float kCoplanarBand = 0.25f;
            DumpFirstPersonInspectorSurface(
                visible[primaryIndex], "PRIMARY", true);
            Console::WriteLine(
                "--- FP_PICK NEAR-LAYER count=%zu band=%.3f ---",
                visible.size(), double(kCoplanarBand));
            for (size_t i = 0; i < visible.size(); ++i)
            {
                if (i == primaryIndex)
                    continue;
                DumpFirstPersonInspectorSurface(
                    visible[i], "COMPETITOR", false);
            }

            DumpFirstPersonInspectorRelevantTiles(
                visible[primaryIndex].point);
            Console::WriteLine("=============================");
        }
    } // namespace

    std::optional<PassengerPaintAnchor>
        CaptureFirstPersonPassengerPaintAnchor(
            const Vehicle& vehicle, uint8_t seatIndex)
    {
        return CaptureFirstPersonPassengerPaintAnchor(
            vehicle, seatIndex,
            FirstPersonPassengerPaintInterpolation{});
    }

    std::optional<PassengerPaintAnchor>
        CaptureFirstPersonPassengerPaintAnchor(
            const Vehicle& vehicle, uint8_t seatIndex,
            const FirstPersonPassengerPaintInterpolation& interpolation)
    {
        if (seatIndex >= 32)
            return std::nullopt;
        const auto* ride = vehicle.GetRide();
        if (ride == nullptr
            || !ride->getRideTypeDescriptor().flags.has(
                RtdFlag::isFlatRide))
            return std::nullopt;

        const auto tryTile =
            [&](CoordsXY tile)
                -> std::optional<PassengerPaintAnchor> {
                if (!MapIsLocationValid(tile))
                    return std::nullopt;

                const auto projected =
                    Translate3DTo2DWithZ(
                        0,
                        {
                            tile.x + kCoordsXYHalfTile,
                            tile.y + kCoordsXYHalfTile,
                            vehicle.TrackLocation.z,
                        });
                Drawing::RenderTarget target{};
                target.x = projected.x - 2048;
                target.y = projected.y - 2048;
                target.width = 4096;
                target.height = 4096;
                target.cullingX = target.x;
                target.cullingY = target.y;
                target.cullingWidth = target.width;
                target.cullingHeight = target.height;
                target.zoom_level = ZoomLevel{ 0 };

                auto* session =
                    PaintSessionAlloc(target, 0, 0);
                if (session == nullptr)
                    return std::nullopt;
                PassengerPaintAnchor anchor{};
                std::vector<FirstPersonPaintSemanticComponent>
                    semanticComponents;
                session->CurrentSource =
                    PaintStructSource::tile;
                session->FirstPersonSemanticComponentSink =
                    &semanticComponents;
                session->FirstPersonPassengerAnchorSink =
                    &anchor;
                session->FirstPersonPassengerAnchorEntity =
                    const_cast<Vehicle*>(&vehicle);
                session->FirstPersonPassengerAnchorSeatIndex =
                    seatIndex;
                session->FirstPersonPassengerInterpolation =
                    interpolation;
                TileElementPaintSetup(*session, tile);
                PaintSessionFree(session);

                if (anchor.Entity != &vehicle
                    || (anchor.seatMask
                        & (uint32_t{ 1 }
                            << seatIndex))
                        == 0)
                    return std::nullopt;
                return anchor;
            };

        const uint16_t key = vehicle.id.ToUnderlying();
        if (const auto cached =
                _passengerAnchorSourceTiles.find(key);
            cached != _passengerAnchorSourceTiles.end())
        {
            if (const auto anchor =
                    tryTile(cached->second);
                anchor.has_value())
                return anchor;
            _passengerAnchorSourceTiles.erase(cached);
        }

        const CoordsXY origin{
            vehicle.TrackLocation.x,
            vehicle.TrackLocation.y
        };
        const auto originTile = origin.toTileStart();
        constexpr int32_t kSearchRadius = 4;
        for (int32_t radius = 0;
             radius <= kSearchRadius; ++radius)
        {
            for (int32_t dy = -radius;
                 dy <= radius; ++dy)
            for (int32_t dx = -radius;
                 dx <= radius; ++dx)
            {
                if (radius != 0
                    && std::max(
                        std::abs(dx),
                        std::abs(dy)) != radius)
                    continue;
                const CoordsXY tile{
                    originTile.x
                        + dx * kCoordsXYStep,
                    originTile.y
                        + dy * kCoordsXYStep,
                };
                if (const auto anchor =
                        tryTile(tile);
                    anchor.has_value())
                {
                    _passengerAnchorSourceTiles[key] =
                        tile;
                    return anchor;
                }
            }
        }
        return std::nullopt;
    }

    std::optional<uint64_t>
        EnsureFirstPersonLargeSceneryPhysicalProxy(
            CoordsXY tile, const LargeSceneryElement& large)
    {
        if (large.isGhost() || large.isInvisible())
            return std::nullopt;
        const auto* entry = large.getEntry();
        if (entry == nullptr)
            return std::nullopt;

        auto* element = reinterpret_cast<TileElement*>(
            const_cast<LargeSceneryElement*>(&large));
        const auto group =
            GetReconstructionGroup(tile, element);
        if (!group.has_value()
            || group->type != TileElementType::largeScenery)
            return std::nullopt;

        if (const auto found =
                gFirstPersonLargeSceneryPhysicalProxies.find(
                    group->key);
            found != gFirstPersonLargeSceneryPhysicalProxies.end()
                && !found->second.empty())
            return group->key;

        if (const auto cached =
                _largeSceneryGeometryCache.find(group->key);
            cached != _largeSceneryGeometryCache.end()
                && !cached->second.dirty
                && !cached->second.collisionProxies.empty())
        {
            PublishFirstPersonLargeSceneryPhysicalProxies(
                group->key, cached->second.collisionProxies);
            return group->key;
        }

        const auto* model =
            GetLargeSceneryAssetModel(*entry);
        if (model == nullptr || !model->usable)
            return std::nullopt;

        const uint8_t direction =
            static_cast<uint8_t>(large.getDirection()) & 3u;
        if (!LargeSceneryInstanceComplete(
                *entry, *group, direction))
            return std::nullopt;

        auto proxies =
            BuildFirstPersonLargeSceneryCollisionProxies(
                *entry, *model, *group, direction);
        if (proxies.empty())
            return std::nullopt;
        PublishFirstPersonLargeSceneryPhysicalProxies(
            group->key, proxies);
        return group->key;
    }

    uint8_t GetFirstPersonTerrainSourceRotation(uint8_t slope)
    {
        return ChooseTerrainSourceRotation(slope);
    }

    bool IsFirstPersonEntityPaintRoot(const ::PaintStruct& root)
    {
        return root.Source == PaintStructSource::entity;
    }

    bool FirstPersonVerticalTunnelCutsTerrain(
        int32_t terrainBaseZ, uint8_t verticalTunnelHeight)
    {
        return verticalTunnelHeight != 0xFF
            && int32_t(verticalTunnelHeight) * kCoordsZPerTinyZ == terrainBaseZ;
    }

    FirstPersonWallPlane BuildFirstPersonWallPlane(
        CoordsXY tileOrigin, int32_t baseZ, uint8_t direction, uint8_t slope, int32_t height)
    {
        const auto local =
            FirstPersonWallSemanticCorners(
                direction, slope, height);
        FirstPersonWallPlane plane{};
        for (size_t i = 0; i < local.size(); ++i)
        {
            plane.corners[i] = {
                float(tileOrigin.x) + local[i].x,
                float(tileOrigin.y) + local[i].y,
                float(baseZ) + local[i].z,
            };
        }
        return plane;
    }

    std::optional<FirstPersonProjection> ProjectFirstPersonPoint(
        const FirstPersonCamera& c, const FirstPersonVec3& p, const ScreenSize& size,
        float fov, float nearClip)
    {
        return ProjectFirstPersonMath(c, p, size.width, size.height, fov, nearClip);
    }
    static void CollectFirstPersonSceneInto(
        FirstPersonScene& scene,
        const FirstPersonRenderOptions& opt,
        const ScreenSize& dimensions)
    {
        FirstPersonFrameBudget publicationBudget(2, std::chrono::microseconds(750));
        _reconstructionWorker.publish(publicationBudget);
        _discoveryBudget = FirstPersonFrameBudget(2048, std::chrono::microseconds(2000));
        // Reuse the prepared scene's backing allocations between presentation
        // frames. Every semantic value is rebuilt below; only vector capacity is
        // retained.
        scene.surfaces.clear();
        scene.staticRegions.clear();
        scene.visibleTiles.clear();
        scene.activeTrackRegions.clear();
        scene.sceneEpoch = _sceneEpoch;
        scene.options = opt;
        scene.resolvedView = {};
        scene.dimensions = dimensions;
        scene.screenOrigin = {};
        scene.presentationFrameSerial = 0;

        const auto map = getGameState().mapSize;
        scene.resolvedView = ResolveFirstPersonView(
            opt.camera, dimensions.width, dimensions.height, map.x, map.y,
            opt.fieldOfViewDegrees, opt.nearClip, opt.farClip);
        scene.options.farClip = scene.resolvedView.farClip;
        // Independent of the overhead paint collector: geometry is derived from live map state.
        DiscoverVisibleTiles(scene);
        CollectTerrain(scene);
        _snapshotBudget = FirstPersonFrameBudget(16, std::chrono::microseconds(1000));
        CollectTrackTrajectories(scene);
    }

    FirstPersonScene CollectFirstPersonScene(
        const FirstPersonRenderOptions& opt, const ScreenSize& dimensions)
    {
        FirstPersonScene scene{};
        CollectFirstPersonSceneInto(
            scene, opt, dimensions);
        // This remains an explicitly identified compatibility bridge for complex sprite selection.
        return scene;
    }
    void ResetFirstPersonPresentationCache()
    {
        gFirstPersonInspectorPickRequested = false;
        gFirstPersonInteractionPickRequested = false;
        gFirstPersonInteractionPickResult.reset();
        _reconstructionWorker.cancel();
        _largeSceneryJobs.clear();
        _parkEntranceJobs.clear();
        _smallSceneryJobs.clear();
        _vehicleHullJobs.clear();
        _semanticJobs.clear();
        _trackMaterialJobs.clear();
        _trackGeometryRequests.clear();
        _preparedFrame.active = false;
        _preparedFrame.valid = false;
        _preparedFrame.drawingEngine = nullptr;
        _preparedFrame.scene = {};
        _entityRotations.clear();
        _passengerAnchorSourceTiles.clear();
        _dynamicEntitySpatialCache = {};
    }

    void ClearFirstPersonSceneCache()
    {
        ResetFirstPersonPresentationCache();
        _preparedFrame.scene = {};
        ++_sceneEpoch;
        if (_sceneEpoch == 0)
            _sceneEpoch = 1;
        _terrainCache.entries.clear();
        _terrainCache.frame = 0;
        _regionBounds.clear();
        _staticPaintCache.clear();
        _reconstructionRotations.clear();
        _entityRotations.clear();
        _passengerAnchorSourceTiles.clear();
        _dynamicEntitySpatialCache = {};
        _trackTrajectoryCache.clear();
        _trackTrajectoryGroupsByRegion.clear();
        _trackTrajectoryGroupsByBoundsRegion.clear();
        ClearLargeSceneryAssetModelCache();
        ClearFirstPersonSmallSceneryReconstructionCache();
        _largeSceneryGeometryCache.clear();
        ClearFirstPersonLargeSceneryPhysicalProxies();
        _largeSceneryGroupsByRegion.clear();
        _largeSceneryGroupsByBoundsRegion.clear();
        _activeLargeSceneryRegions.clear();
        _largeSceneryGeometryEnabled = false;
        _parkEntranceGeometryCache.clear();
        _parkEntranceGroupsByRegion.clear();
        _parkEntranceGroupsByBoundsRegion.clear();
        _parkEntranceGeometryEnabled = false;
        ClearFirstPersonSemanticComponents();
        _staticRegionPackets.clear();
    }
    void InvalidateFirstPersonSceneRegion(CoordsXY low, CoordsXY high)
    {
        ClearFirstPersonLargeSceneryPhysicalProxies();
        if (_regionBounds.empty() && _terrainCache.entries.empty()
            && _staticPaintCache.empty() && _trackTrajectoryCache.empty()
            && _largeSceneryGeometryCache.empty()
            && _parkEntranceGeometryCache.empty()
            && _staticRegionPackets.empty()) return;
        const auto floorTile = [](int32_t x) {return int32_t(std::floor(float(x)/kCoordsXYStep));};
        const auto x0 = floorTile(std::min(low.x,high.x));
        const auto y0 = floorTile(std::min(low.y,high.y));
        const auto x1 = floorTile(std::max(low.x,high.x));
        const auto y1 = floorTile(std::max(low.y,high.y));
        for (auto it = _largeSceneryJobs.begin(); it != _largeSceneryJobs.end();)
        {
            const auto& job = *it->second;
            const bool changed = x0 <= job.maxX && x1 >= job.minX && y0 <= job.maxY && y1 >= job.minY
                && std::any_of(job.tileSignatures.begin(), job.tileSignatures.end(),
                    [](const auto& tile) { return NativeTileSignature(tile.first) != tile.second; });
            if (changed) it = _largeSceneryJobs.erase(it);
            else ++it;
        }
        for (auto it = _parkEntranceJobs.begin();
             it != _parkEntranceJobs.end();)
        {
            const auto& job = *it->second;
            const bool changed =
                std::any_of(
                    job.tileSignatures.begin(),
                    job.tileSignatures.end(),
                    [&](const auto& tile) {
                        const int32_t tx =
                            int32_t(std::floor(
                                float(tile.first.x)
                                / kCoordsXYStep));
                        const int32_t ty =
                            int32_t(std::floor(
                                float(tile.first.y)
                                / kCoordsXYStep));
                        return tx >= x0 && tx <= x1
                            && ty >= y0 && ty <= y1
                            && NativeTileSignature(
                                tile.first) != tile.second;
                    });
            if (changed)
                it = _parkEntranceJobs.erase(it);
            else
                ++it;
        }
        for (auto it = _trackGeometryRequests.begin(); it != _trackGeometryRequests.end();)
        {
            const auto p = it->second.tile;
            if (p.x / kCoordsXYStep >= x0 - 16 && p.x / kCoordsXYStep <= x1 + 16
                && p.y / kCoordsXYStep >= y0 - 16 && p.y / kCoordsXYStep <= y1 + 16)
                it = _trackGeometryRequests.erase(it);
            else ++it;
        }
        std::unordered_set<uint64_t> trackCandidates;
        std::unordered_set<uint64_t> largeSceneryCandidates;
        std::unordered_set<uint64_t> parkEntranceCandidates;
        for (int32_t regionY = y0 / 32; regionY <= y1 / 32; ++regionY)
        for (int32_t regionX = x0 / 32; regionX <= x1 / 32; ++regionX)
        {
            const uint64_t regionKey =
                FirstPersonGpuRegionKey(
                    regionX * 32, regionY * 32);
            _staticRegionPackets[regionKey].dirty = true;
            ++_staticRegionPackets[regionKey].sourceRevision;

            if (const auto tracks =
                    _trackTrajectoryGroupsByBoundsRegion.find(
                        regionKey);
                tracks
                    != _trackTrajectoryGroupsByBoundsRegion.end())
            {
                trackCandidates.insert(
                    tracks->second.begin(),
                    tracks->second.end());
            }
            if (const auto scenery =
                    _largeSceneryGroupsByBoundsRegion.find(
                        regionKey);
                scenery
                    != _largeSceneryGroupsByBoundsRegion.end())
            {
                largeSceneryCandidates.insert(
                    scenery->second.begin(),
                    scenery->second.end());
            }
            if (const auto entrances =
                    _parkEntranceGroupsByBoundsRegion.find(
                        regionKey);
                entrances
                    != _parkEntranceGroupsByBoundsRegion.end())
            {
                parkEntranceCandidates.insert(
                    entrances->second.begin(),
                    entrances->second.end());
            }
        }
        InvalidateFirstPersonRegionBounds(
            x0, y0, x1, y1);
        for (int32_t ty = y0; ty <= y1; ++ty)
        for (int32_t tx = x0; tx <= x1; ++tx)
        {
            WithdrawFirstPersonSemanticComponents(
                { tx * kCoordsXYStep,
                  ty * kCoordsXYStep });

            // Both caches are already addressed by the exact tile key. A local
            // native invalidation should therefore cost O(invalidated tiles),
            // not O(every terrain/static tile first person has ever cached).
            const uint64_t key = TerrainKey(tx, ty);
            if (auto terrain = _terrainCache.entries.find(key);
                terrain != _terrainCache.entries.end())
            {
                terrain->second.dirty = true;
            }
            if (auto cached = _staticPaintCache.find(key);
                cached != _staticPaintCache.end())
            {
                if (NativeTileSignature({tx * kCoordsXYStep, ty * kCoordsXYStep}) != cached->second.signature)
                {
                    ++cached->second.sourceRevision;
                    _semanticJobs.erase(key);
                    cached->second.semanticMovingMeshes.clear();
                    cached->second.semanticResidentSurfaces.clear();
                    cached->second.semanticStreamedSurfaces.clear();
                    cached->second.semanticSurfaceFingerprint = 0;
                }
                cached->second.dirty = true;
                cached->second.visibilityDirty = true;
            }
        }
        for (const auto groupKey : trackCandidates)
        {
            const auto found =
                _trackTrajectoryCache.find(groupKey);
            if (found == _trackTrajectoryCache.end())
                continue;
            auto& trajectory = found->second;
            if (!trajectory.hasBounds)
                continue;
            if (x0 <= trajectory.maxTileX
                && x1 >= trajectory.minTileX
                && y0 <= trajectory.maxTileY
                && y1 >= trajectory.minTileY)
            {
                MarkTrackTrajectoryRegionsDirty(trajectory);
                ++trajectory.materialVersion;
                _trackMaterialJobs.erase(groupKey);
                trajectory.dirty = true;
            }
        }
        for (const auto groupKey : largeSceneryCandidates)
        {
            const auto found =
                _largeSceneryGeometryCache.find(groupKey);
            if (found == _largeSceneryGeometryCache.end())
                continue;
            auto& geometry = found->second;
            if (!geometry.hasBounds)
                continue;
            if (x0 <= geometry.maxTileX
                && x1 >= geometry.minTileX
                && y0 <= geometry.maxTileY
                && y1 >= geometry.minTileY)
            {
                MarkLargeSceneryGeometryRegionsDirty(geometry);
                geometry.dirty = true;
            }
        }
        for (const auto groupKey :
             parkEntranceCandidates)
        {
            const auto found =
                _parkEntranceGeometryCache.find(groupKey);
            if (found
                == _parkEntranceGeometryCache.end())
                continue;
            auto& geometry = found->second;
            if (!geometry.hasBounds)
                continue;
            if (x0 <= geometry.maxTileX
                && x1 >= geometry.minTileX
                && y0 <= geometry.maxTileY
                && y1 >= geometry.minTileY)
            {
                const bool sourceChanged =
                    std::any_of(
                        geometry.tileSignatures.begin(),
                        geometry.tileSignatures.end(),
                        [&](const auto& source) {
                            const int32_t tx =
                                source.first.x
                                / kCoordsXYStep;
                            const int32_t ty =
                                source.first.y
                                / kCoordsXYStep;
                            return tx >= x0 && tx <= x1
                                && ty >= y0 && ty <= y1
                                && NativeTileSignature(
                                    source.first)
                                    != source.second;
                        });
                if (!sourceChanged)
                    continue;
                MarkParkEntranceGeometryRegionsDirty(
                    geometry);
                geometry.dirty = true;
            }
        }
        // Generic native invalidation also covers shadows and moving objects,
        // so retain the cached data but mark it unfit for a region packet until
        // the tile is next admitted and its live terrain/static state is
        // revalidated. This prevents stale region VBOs without forcing a full
        // terrain reconstruction on every unrelated invalidation.
    }
    void InvalidateFirstPersonSceneTile(CoordsXY world)
    {
        InvalidateFirstPersonSceneRegion(world,world);
    }
    void BeginFirstPersonPresentationFrame()
    {
        _preparedFrame.active = true;
        _preparedFrame.valid = false;
        // Keep the previous scene's vector capacities. The first POV render of
        // this presentation repopulates the same buffers from authoritative
        // state.
        ++_preparedFrame.serial;
        if (_preparedFrame.serial == 0)
            ++_preparedFrame.serial;
    }

    void EndFirstPersonPresentationFrame()
    {
        _preparedFrame.active = false;
        _preparedFrame.valid = false;
        _preparedFrame.drawingEngine = nullptr;
        // Retain scene storage for the next presentation. A real POV teardown
        // calls ResetFirstPersonPresentationCache(), which releases it.
    }

    void RenderFirstPerson(
        Drawing::RenderTarget& rt, const FirstPersonRenderOptions& opt,
        const ScreenRect& viewport)
    {
        PROFILED_FUNCTION();
        const ScreenSize dimensions{
            viewport.getWidth(), viewport.getHeight()
        };
        if (dimensions.width <= 0 || dimensions.height <= 0
            || rt.DrawingEngine == nullptr)
            return;

        const ScreenCoordsXY screenOrigin{
            viewport.getLeft(), viewport.getTop()
        };
        const bool reusePrepared =
            _preparedFrame.active
            && _preparedFrame.valid
            && _preparedFrame.drawingEngine == rt.DrawingEngine
            && _preparedFrame.dimensions.width == dimensions.width
            && _preparedFrame.dimensions.height == dimensions.height
            && _preparedFrame.screenOrigin.x == screenOrigin.x
            && _preparedFrame.screenOrigin.y == screenOrigin.y
            && SameFirstPersonRenderOptions(
                _preparedFrame.options, opt);

        FirstPersonScene localScene{};
        FirstPersonScene* scene = nullptr;
        if (reusePrepared)
        {
            scene = &_preparedFrame.scene;
        }
        else
        {
            // A second, genuinely different scene inside the same presentation
            // frame must not alias the GPU's cached first scene.
            if (_preparedFrame.active
                && _preparedFrame.valid)
            {
                ++_preparedFrame.serial;
                if (_preparedFrame.serial == 0)
                    ++_preparedFrame.serial;
            }

            if (_preparedFrame.active)
            {
                scene = &_preparedFrame.scene;
                CollectFirstPersonSceneInto(
                    *scene, opt, dimensions);
            }
            else
            {
                localScene =
                    CollectFirstPersonScene(
                        opt, dimensions);
                scene = &localScene;
            }

            scene->screenOrigin = screenOrigin;
            scene->presentationFrameSerial =
                _preparedFrame.active
                    ? _preparedFrame.serial : 0;
            CollectPaintSprites(*scene, rt);

            if (_preparedFrame.active)
            {
                _preparedFrame.options = opt;
                _preparedFrame.dimensions = dimensions;
                _preparedFrame.screenOrigin = screenOrigin;
                _preparedFrame.drawingEngine =
                    rt.DrawingEngine;
                _preparedFrame.valid = true;
            }
        }

        if (gFirstPersonInteractionPickRequested)
        {
            gFirstPersonInteractionPickRequested = false;
            gFirstPersonInteractionPickResult.reset();
            if (const auto selection =
                    SelectFirstPersonInspectorHit(*scene);
                selection.has_value())
            {
                const auto& target =
                    selection->visible[
                        selection->primaryIndex]
                        .surface->interactionTarget;
                if (target.hasValue())
                    gFirstPersonInteractionPickResult =
                        target;
            }
        }

        if (gFirstPersonInspectorPickRequested)
        {
            gFirstPersonInspectorPickRequested = false;
            DumpFirstPersonInspectorPick(*scene);
        }

        auto* context =
            rt.DrawingEngine->GetDrawingContext();
        if (context != nullptr)
            context->DrawFirstPersonScene(rt, *scene);

    }
} // namespace OpenRCT2::Paint

