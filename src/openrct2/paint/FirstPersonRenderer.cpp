/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#include "FirstPersonRenderer.h"
#include "FirstPersonAssetReconstruction.h"
#include "FirstPersonPhysicalProxy.h"
#include "FirstPersonTrackTrajectory.h"
#include "FirstPersonTrackProfileCalibration.h"
#include "FirstPersonVehicleBodyHull.h"
#include "FirstPersonVehiclePose.h"
#include "Paint.h"
#include "Paint.SessionFlags.h"
#include "tile_element/Paint.Surface.h"
#include "tile_element/Paint.Path.h"
#include "tile_element/Paint.TileElement.h"
#include "Paint.Entity.h"

#include "../Context.h"
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
#include "../world/tile_element/PathElement.h"
#include "../world/tile_element/TrackElement.h"
#include "../world/tile_element/SmallSceneryElement.h"
#include "../world/tile_element/LargeSceneryElement.h"
#include "../world/tile_element/Slope.h"
#include "../world/tile_element/TileElement.h"
#include "../world/tile_element/TileElementType.h"
#include "../world/tile_element/WallElement.h"
#include "../object/WallSceneryEntry.h"
#include "../object/LargeSceneryEntry.h"
#include "../object/SmallSceneryEntry.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace OpenRCT2::Paint
{
    namespace
    {

        float Dot(FirstPersonVec3 a, FirstPersonVec3 b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }
        FirstPersonVec3 Sub(FirstPersonVec3 a, FirstPersonVec3 b)
        {
            return { a.x - b.x, a.y - b.y, a.z - b.z };
        }
        FirstPersonVec3 Add(FirstPersonVec3 a, FirstPersonVec3 b)
        {
            return { a.x + b.x, a.y + b.y, a.z + b.z };
        }
        FirstPersonVec3 Mul(FirstPersonVec3 a, float s)
        {
            return { a.x * s, a.y * s, a.z * s };
        }
        void EmitQuad(FirstPersonSurface& surface, std::array<FirstPersonVertex, 4> v, bool otherDiagonal = false)
        {
            // The diagonal is physically meaningful for RCT2's half-flat,
            // single-corner slopes and valleys. Do not always split 0--2:
            // that creates ramps where the authoritative height function says
            // the land should be flat (and puts raised land in the wrong place).
            surface.triangles = otherDiagonal
                ? std::array<FirstPersonVertex, 6>{ v[0], v[1], v[3], v[1], v[2], v[3] }
                : std::array<FirstPersonVertex, 6>{ v[0], v[1], v[2], v[0], v[2], v[3] };
        }
        [[nodiscard]] bool UsesOppositeTerrainDiagonal(uint8_t slope)
        {
            switch (slope & kTileSlopeRaisedCornersMask)
            {
                // North/South corner up or down, or North/South valley:
                // physical world coordinates of north and south are (32,32)
                // and (0,0), respectively. Isolate those corners with 1--3.
                case kTileSlopeNCornerUp:
                case kTileSlopeSCornerUp:
                case kTileSlopeNSValley:
                case kTileSlopeNCornerDown:
                case kTileSlopeSCornerDown:
                    return true;
                default:
                    return false;
            }
        }
        [[nodiscard]] uint64_t ProjectedTriangleAreaTwice(
            const ScreenCoordsXY& a, const ScreenCoordsXY& b, const ScreenCoordsXY& c)
        {
            const int64_t abx = int64_t(b.x) - a.x;
            const int64_t aby = int64_t(b.y) - a.y;
            const int64_t acx = int64_t(c.x) - a.x;
            const int64_t acy = int64_t(c.y) - a.y;
            const int64_t area = abx * acy - aby * acx;
            return uint64_t(area < 0 ? -area : area);
        }

        [[nodiscard]] uint8_t ChooseTerrainSourceRotation(uint8_t slope)
        {
            const auto corners = GetSlopeCornerHeights(0, slope);
            const std::array<CoordsXYZ, 4> world{ {
                { 0, 0, corners.south },
                { kCoordsXYStep, 0, corners.east },
                { kCoordsXYStep, kCoordsXYStep, corners.north },
                { 0, kCoordsXYStep, corners.west },
            } };
            const bool opposite = UsesOppositeTerrainDiagonal(slope);
            const std::array<std::array<size_t, 3>, 2> triangleIndices = opposite
                ? std::array<std::array<size_t, 3>, 2>{ { { 0, 1, 3 }, { 1, 2, 3 } } }
                : std::array<std::array<size_t, 3>, 2>{ { { 0, 1, 2 }, { 0, 2, 3 } } };

            uint8_t bestRotation = 0;
            uint64_t bestMinimumArea = 0;
            for (uint8_t rotation = 0; rotation < 4; ++rotation)
            {
                std::array<ScreenCoordsXY, 4> projected{};
                for (size_t i = 0; i < world.size(); ++i)
                    projected[i] = Translate3DTo2DWithZ(rotation, world[i]);

                uint64_t minimumArea = std::numeric_limits<uint64_t>::max();
                for (const auto& triangle : triangleIndices)
                {
                    minimumArea = std::min(
                        minimumArea,
                        ProjectedTriangleAreaTwice(
                            projected[triangle[0]], projected[triangle[1]], projected[triangle[2]]));
                }
                // Stable world-space tie break: lower native quarter-turn wins.
                if (minimumArea > bestMinimumArea)
                {
                    bestMinimumArea = minimumArea;
                    bestRotation = rotation;
                }
            }
            return bestRotation;
        }

        [[nodiscard]] uint8_t PaintRotationForPoint(
            const FirstPersonCamera& camera, FirstPersonVec3 anchor, std::optional<uint8_t> previous)
        {
            return FirstPersonSourceRotationForPoint(camera, anchor, previous);
        }

        [[nodiscard]] uint8_t PaintRotationForTile(
            const FirstPersonCamera& camera, CoordsXY tile, std::optional<uint8_t> previous)
        {
            return PaintRotationForPoint(
                camera,
                { float(tile.x + kCoordsXYHalfTile), float(tile.y + kCoordsXYHalfTile), 0.0f },
                previous);
        }

        inline void ExtendStableKey(uint64_t& key, uint64_t value)
        {
            for (unsigned i = 0; i < 8; ++i)
            {
                key ^= (value >> (8u * i)) & 255u;
                key *= 1099511628211ull;
            }
        }

        struct ReconstructionGroupInfo
        {
            uint64_t key{};
            FirstPersonVec3 anchor{};
            CoordsXY sourceTile{};
            TileElementType type = TileElementType::surface;
        };

        [[nodiscard]] std::optional<ReconstructionGroupInfo> GetReconstructionGroup(
            CoordsXY tile, TileElement* element)
        {
            if (element == nullptr)
                return std::nullopt;

            if (element->getType() == TileElementType::largeScenery)
            {
                auto* large = element->asLargeScenery();
                const auto* entry = large != nullptr ? large->getEntry() : nullptr;
                const size_t sequence = large != nullptr ? large->getSequenceIndex() : 0;
                if (entry == nullptr || sequence >= entry->tiles.size()
                    || entry->flags.has(LargeSceneryFlag::isTree))
                    return std::nullopt;

                const auto direction = large->getDirection();
                const auto offset = CoordsXY{ entry->tiles[sequence].offset }.rotate(direction);
                const FirstPersonVec3 anchor{
                    float(tile.x - offset.x),
                    float(tile.y - offset.y),
                    float(large->getBaseZ() - entry->tiles[sequence].offset.z),
                };
                uint64_t key = 14695981039346656037ull;
                ExtendStableKey(key, 1);
                ExtendStableKey(key, uint32_t(int32_t(anchor.x)));
                ExtendStableKey(key, uint32_t(int32_t(anchor.y)));
                ExtendStableKey(key, uint32_t(int32_t(anchor.z)));
                ExtendStableKey(key, direction);
                ExtendStableKey(key, large->getEntryIndex());
                if (key == 0) key = 1;
                return ReconstructionGroupInfo{
                    key, anchor, tile, TileElementType::largeScenery
                };
            }

            if (element->getType() == TileElementType::track)
            {
                auto* track = element->asTrack();
                const auto origin = GetTrackSegmentOrigin(CoordsXYE{ tile, element });
                if (track == nullptr || !origin.has_value())
                    return std::nullopt;

                const FirstPersonVec3 anchor{
                    float(origin->x), float(origin->y), float(origin->z)
                };
                uint64_t key = 14695981039346656037ull;
                ExtendStableKey(key, 2);
                ExtendStableKey(key, uint32_t(origin->x));
                ExtendStableKey(key, uint32_t(origin->y));
                ExtendStableKey(key, uint32_t(origin->z));
                ExtendStableKey(key, origin->direction);
                ExtendStableKey(key, track->getRideIndex().ToUnderlying());
                ExtendStableKey(key, static_cast<uint16_t>(track->getTrackType()));
                if (key == 0) key = 1;
                return ReconstructionGroupInfo{
                    key, anchor, tile, TileElementType::track
                };
            }

            return std::nullopt;
        }
        FirstPersonVec3 Anchor(const PaintStruct& ps)
        {
            if (ps.Entity != nullptr)
            {
                const auto p = ps.Entity->getLocation();
                return { float(p.x), float(p.y), float(p.z) };
            }
            return { float(ps.Bounds.x + ps.Bounds.x_end) * 0.5f,
                     float(ps.Bounds.y + ps.Bounds.y_end) * 0.5f, float(ps.Bounds.z) };
        }
        // An upright impostor faces the PASSENGER'S LOCATION, never the gaze
        // direction. Otherwise looking sideways or rolling the head physically
        // spins the surrounding trees, signs and scenery around their centres.
        FirstPersonVec3 UprightBillboardRight(
            FirstPersonVec3 anchor, FirstPersonVec3 eye, FirstPersonVec3 fallback)
        {
            const float dx=anchor.x-eye.x,dy=anchor.y-eye.y;
            const float len=std::hypot(dx,dy);
            if(len>0.1f) return {-dy/len,dx/len,0.0f};
            const float fallbackLen=std::hypot(fallback.x,fallback.y);
            if(fallbackLen>0.001f)
                return {fallback.x/fallbackLen,fallback.y/fallbackLen,0.0f};
            return {0.0f,1.0f,0.0f};
        }
        void ReorientBillboard(
            FirstPersonSurface& surface, FirstPersonVec3 eye, FirstPersonVec3 fallbackRight)
        {
            if (!surface.viewFacing) return;
            const auto cameraRight=UprightBillboardRight(surface.billboardAnchor,eye,fallbackRight);
            const auto up = FirstPersonVec3{ 0.0f, 0.0f, 1.0f };
            const auto p0 = Add(Add(surface.billboardAnchor, Mul(cameraRight,surface.billboardLeft)),
                                Mul(up,-surface.billboardTop));
            const auto p1 = Add(p0,Mul(cameraRight,surface.billboardWidth));
            const auto p2 = Add(p1,Mul(up,-surface.billboardHeight));
            const auto p3 = Add(p0,Mul(up,-surface.billboardHeight));
            for (size_t i=0;i<6;++i)
            {
                surface.triangles[i].world =
                    std::array<FirstPersonVec3,6>{p0,p1,p2,p0,p2,p3}[i];
            }
        }
        struct SpriteCompositeLayout
        {
            const G1Element* colour{};
            const G1Element* mask{};
            int32_t xOffset{}, yOffset{}, width{}, height{};
        };
        [[nodiscard]] std::optional<SpriteCompositeLayout> GetSpriteCompositeLayout(ImageId image, ImageId mask)
        {
            const auto* colour = image.HasValue() ? GfxGetG1Element(image) : nullptr;
            if (colour == nullptr || colour->width <= 0 || colour->height <= 0)
                return std::nullopt;
            if (!mask.HasValue())
                return SpriteCompositeLayout{ colour, nullptr, colour->xOffset, colour->yOffset, colour->width, colour->height };

            const auto* maskG1 = GfxGetG1Element(mask);
            if (maskG1 == nullptr || maskG1->width <= 0 || maskG1->height <= 0)
                return std::nullopt;
            // DrawSpriteRawMasked positions the composite with MASK offsets and
            // clips both images to the common rectangle. Colour offsets do not
            // participate in placement.
            return SpriteCompositeLayout{
                colour, maskG1, maskG1->xOffset, maskG1->yOffset,
                std::min<int32_t>(colour->width, maskG1->width),
                std::min<int32_t>(colour->height, maskG1->height)
            };
        }
        void ApplyImmutablePaintSnapshot(FirstPersonSurface& surface, uint32_t handle)
        {
            const auto* snapshot = Drawing::ScrollingText::GetFirstPersonSnapshot(handle);
            if (snapshot == nullptr || snapshot->pixels.empty())
                return;

            surface.immutablePixels = snapshot->pixels;
            surface.immutableWidth = snapshot->width;
            surface.immutableHeight = snapshot->height;
            uint64_t fingerprint = 1469598103934665603ULL;
            auto extend = [&](uint64_t value) {
                fingerprint ^= value;
                fingerprint *= 1099511628211ULL;
            };
            extend(uint16_t(snapshot->width));
            extend(uint16_t(snapshot->height));
            for (const auto pixel : snapshot->pixels)
                extend(pixel);
            surface.immutableFingerprint = fingerprint;
        }

        struct SpriteReconstructionFrame
        {
            FirstPersonVec3 anchor{};
            uint64_t groupKey{};
        };

        [[nodiscard]] SpriteReconstructionFrame GetSpriteReconstructionFrame(
            const PaintStruct& ps, FirstPersonVec3 fallbackAnchor)
        {
            SpriteReconstructionFrame frame{ fallbackAnchor, 0 };

            // Connected native sprite fragments remain one visual impostor:
            // they share the canonical object/track origin and source rotation.
            // This preserves their relative native-image placement without
            // pretending the 2-D artwork lies on an arbitrary world-fixed plane.
            if (ps.Element != nullptr)
            {
                if (const auto group = GetReconstructionGroup(ps.MapPos, ps.Element); group.has_value())
                {
                    frame.anchor = group->anchor;
                    frame.groupKey = group->key;
                    return frame;
                }

                if (ps.Element->getType() == TileElementType::smallScenery)
                {
                    const auto* small = ps.Element->asSmallScenery();
                    if (small != nullptr)
                    {
                        frame.anchor = {
                            float(ps.MapPos.x + kCoordsXYHalfTile),
                            float(ps.MapPos.y + kCoordsXYHalfTile),
                            float(small->getBaseZ()),
                        };
                    }
                }
            }
            return frame;
        }

        [[nodiscard]] bool IsPathDeckCarrier(const PaintStruct& ps)
        {
            const auto* path = ps.Element != nullptr ? ps.Element->asPath() : nullptr;
            const auto* surface = path != nullptr ? path->getSurfaceDescriptor() : nullptr;
            const auto* railings = path != nullptr ? path->getRailingsDescriptor() : nullptr;
            if (surface == nullptr || railings == nullptr || !ps.image_id.HasValue())
                return false;

            const auto image = ps.image_id.GetIndex();
            if (image >= surface->image && image < surface->image + 51)
                return true;

            // Supported paths may omit the separate surface sprite. In that
            // case the bridge parent still carries the path image template
            // (ghost/highlight remap included), so it is the authoritative
            // source from which to synthesize the missing semantic deck.
            if (railings->supportType == RailingEntrySupportType::pole)
                return image >= railings->bridgeImage && image < railings->bridgeImage + 20;
            return image >= railings->bridgeImage + 49 && image < railings->bridgeImage + 55;
        }
        void AppendLayer(
            FirstPersonScene& scene, const FirstPersonVec3& anchor, const FirstPersonBasis& basis,
            const ScreenCoordsXY& isoAnchor, ImageId image, const ScreenCoordsXY& spritePos,
            ImageId mask = {})
        {
            const auto layout = GetSpriteCompositeLayout(image, mask);
            if (!layout.has_value())
                return;
            const float left = float(spritePos.x + layout->xOffset - isoAnchor.x);
            const float top = float(spritePos.y + layout->yOffset - isoAnchor.y);
            const auto right = UprightBillboardRight(
                anchor, scene.options.camera.position, basis.right);
            const auto up = FirstPersonVec3{ 0.0f, 0.0f, 1.0f };
            const auto p0 = Add(Add(anchor, Mul(right, left)), Mul(up, -top));
            const auto p1 = Add(p0, Mul(right, float(layout->width)));
            const auto p2 = Add(p1, Mul(up, -float(layout->height)));
            const auto p3 = Add(p0, Mul(up, -float(layout->height)));
            FirstPersonSurface surface{};
            surface.image = image;
            surface.mask = mask;
            surface.viewFacing = true;
            surface.billboardAnchor = anchor;
            surface.billboardLeft = left;
            surface.billboardTop = top;
            surface.billboardWidth = float(layout->width);
            surface.billboardHeight = float(layout->height);
            EmitQuad(surface, { {
                { p0, 0.0f, 0.0f }, { p1, float(layout->width), 0.0f },
                { p2, float(layout->width), float(layout->height) },
                { p3, 0.0f, float(layout->height) },
            } });
            scene.surfaces.emplace_back(std::move(surface));
        }

        // Some native paint helpers expose more than sorting bounds: their boxes
        // are the authored physical geometry itself. This is deliberately
        // restricted to element families whose painters use exact slabs/strips:
        // walls/surfaces, station floors/fences, and path railings/fixtures.
        // Generic track/scenery paint bounds remain sorting evidence only.
        bool AppendPhysicalPlane(
            FirstPersonScene& scene, const PaintStruct& ps, ImageId image,
            const ScreenCoordsXY& spritePos, uint8_t rotation, ImageId mask = {})
        {
            if (ps.Element == nullptr)
                return false;

            const auto type = ps.Element->getType();
            const auto* track = ps.Element->asTrack();
            const auto* path = ps.Element->asPath();
            const bool stationTrack =
                track != nullptr
                && trackTypeIsStation(track->getTrackType());
            const bool pathGeometry =
                path != nullptr && image.HasValue()
                && FirstPersonPathArtworkIsPhysical(
                    *path, image.GetIndex());
            const bool legacyPlanar =
                type == TileElementType::wall
                || type == TileElementType::surface;
            if (!legacyPlanar && !stationTrack && !pathGeometry)
                return false;

            const auto layout = GetSpriteCompositeLayout(image, mask);
            if (!layout.has_value())
                return false;

            const float x0 = float(std::min(ps.Bounds.x, ps.Bounds.x_end));
            const float x1 = float(std::max(ps.Bounds.x, ps.Bounds.x_end));
            const float y0 = float(std::min(ps.Bounds.y, ps.Bounds.y_end));
            const float y1 = float(std::max(ps.Bounds.y, ps.Bounds.y_end));
            const float z0 = float(std::min(ps.Bounds.z, ps.Bounds.z_end));
            const float z1 = float(std::max(ps.Bounds.z, ps.Bounds.z_end));
            const float sx = x1 - x0;
            const float sy = y1 - y0;
            const float sz = z1 - z0;

            const bool alongX =
                sx >= 6.0f && sy <= 4.0f && sz >= 5.0f;
            const bool alongY =
                sy >= 6.0f && sx <= 4.0f && sz >= 5.0f;
            const bool stationSlab =
                stationTrack && sz <= 2.0f
                && sx >= 6.0f && sy >= 6.0f
                // Native station base/floor slabs are authored at track
                // height. Platform strips several units above that can carry
                // an integrated fence in the same sprite; flattening those
                // would turn a visible fence into floor texture.
                && z0 <= float(track->getBaseZ() + 4);
            const bool stationFence =
                stationTrack && (alongX || alongY)
                && z0 >= float(track->getBaseZ() + 1)
                && z1 <= float(track->getBaseZ() + 24);
            const bool verticalPlane =
                (alongX || alongY)
                && (legacyPlanar || pathGeometry || stationFence);
            if (!verticalPlane && !stationSlab)
                return false;

            std::array<FirstPersonVec3, 4> world{};
            if (stationSlab)
            {
                const float fixedZ = 0.5f * (z0 + z1);
                world = { {
                    { x0, y0, fixedZ },
                    { x1, y0, fixedZ },
                    { x1, y1, fixedZ },
                    { x0, y1, fixedZ },
                } };
            }
            else if (alongX)
            {
                const float fixedY = 0.5f * (y0 + y1);
                world = { {
                    { x0, fixedY, z0 },
                    { x1, fixedY, z0 },
                    { x1, fixedY, z1 },
                    { x0, fixedY, z1 },
                } };
            }
            else
            {
                const float fixedX = 0.5f * (x0 + x1);
                world = { {
                    { fixedX, y0, z0 },
                    { fixedX, y1, z0 },
                    { fixedX, y1, z1 },
                    { fixedX, y0, z1 },
                } };
            }

            FirstPersonSurface surface{};
            surface.image = image;
            surface.mask = mask;
            if (stationSlab)
                surface.depthBias = true;
            std::array<FirstPersonVertex, 4> vertices{};
            for (size_t i = 0; i < vertices.size(); ++i)
            {
                const auto& p = world[i];
                const CoordsXYZ loc{
                    int32_t(std::lround(p.x)),
                    int32_t(std::lround(p.y)),
                    int32_t(std::lround(p.z)),
                };
                const auto iso =
                    Translate3DTo2DWithZ(rotation, loc);
                vertices[i] = {
                    p,
                    float(iso.x - spritePos.x - layout->xOffset),
                    float(iso.y - spritePos.y - layout->yOffset),
                };
            }
            EmitQuad(surface, vertices);
            scene.surfaces.emplace_back(std::move(surface));
            return true;
        }
        // Native walls have a REAL footprint, side, base elevation and physical
        // height. A paint sorting bound is not an authoritative world surface:
        // e.g. the bounding box can change with painter rotation or animation.
        // Reconstruct the plane from the placed wall element and its object
        // definition; retain the native selected image, colour and glass child.
        // Doors deliberately take the existing split-panel fallback: a solid
        // full-wall quad would seal an open doorway and misrepresent the game.
        bool AppendSemanticWallPlane(
            FirstPersonScene& scene, const PaintStruct& ps, ImageId image,
            const ScreenCoordsXY& spritePos, uint8_t rotation, ImageId mask = {})
        {
            if (ps.Element == nullptr || ps.Element->getType() != TileElementType::wall || !image.HasValue())
                return false;
            const auto* wall = ps.Element->asWall();
            const auto* entry = wall != nullptr ? wall->getEntry() : nullptr;
            const auto layout = GetSpriteCompositeLayout(image, mask);
            if (entry == nullptr || !layout.has_value() ||
                entry->flags.has(WallSceneryFlag::isDoor) || entry->height == 0)
                return false;

            const auto physical = BuildFirstPersonWallPlane(
                ps.MapPos, wall->getBaseZ(), wall->getDirection(), wall->getSlope(),
                int32_t(entry->height) * kCoordsZStep);
            FirstPersonSurface surface{};
            surface.image = image;
            surface.mask = mask;
            std::array<FirstPersonVertex, 4> vertices{};
            for (size_t i = 0; i < physical.corners.size(); ++i)
            {
                const auto& pos = physical.corners[i];
                const auto iso = Translate3DTo2DWithZ(rotation,
                    { int32_t(std::lround(pos.x)), int32_t(std::lround(pos.y)), int32_t(std::lround(pos.z)) });
                vertices[i] = { pos, float(iso.x - spritePos.x - layout->xOffset),
                                    float(iso.y - spritePos.y - layout->yOffset) };
            }
            EmitQuad(surface, vertices);
            scene.surfaces.emplace_back(std::move(surface));
            return true;
        }
        [[nodiscard]] uint8_t RotateQuarterMask(uint8_t mask, uint8_t direction)
        {
            mask &= 0x0F;
            direction &= 3;
            if (direction == 0) return mask;
            return uint8_t(((uint32_t(mask) << direction) | (uint32_t(mask) >> (4 - direction))) & 0x0F);
        }
        struct FirstPersonSemanticSphere
        {
            FirstPersonVec3 center{};
            float radius{};
        };
        // Large-scenery placement uses exactly these quarter-tile occupancy bits
        // and zClearance values for native construction clearance. Convert them
        // into a conservative physical sphere for view admission while leaving
        // the selected original sprite/remap as the visual surface.
        std::optional<FirstPersonSemanticSphere> LargeScenerySemanticBounds(const PaintStruct& ps)
        {
            if (ps.Element == nullptr || ps.Element->getType() != TileElementType::largeScenery)
                return std::nullopt;
            const auto* large = ps.Element->asLargeScenery();
            const auto* entry = large != nullptr ? large->getEntry() : nullptr;
            if (large == nullptr || entry == nullptr)
                return std::nullopt;
            const size_t sequence = large->getSequenceIndex();
            if (sequence >= entry->tiles.size())
                return std::nullopt;
            const auto& tile = entry->tiles[sequence];
            const uint8_t metadataMask = RotateQuarterMask(tile.corners, large->getDirection());
            // Normally these masks are identical: placement stores the rotated
            // metadata mask in the element. Their union is conservative if a
            // malformed/edited park leaves them temporarily inconsistent.
            const uint8_t occupied = uint8_t(metadataMask | large->getOccupiedQuadrants());
            if ((occupied & 0x0F) == 0)
                return std::nullopt;
            static constexpr std::array<std::array<float, 4>, 4> kQuarterBounds{{
                {{16.0f,16.0f,32.0f,32.0f}}, // SW in native quadrant numbering
                {{16.0f, 0.0f,32.0f,16.0f}}, // NW
                {{ 0.0f, 0.0f,16.0f,16.0f}}, // NE
                {{ 0.0f,16.0f,16.0f,32.0f}}, // SE
            }};
            float minX=32.0f,minY=32.0f,maxX=0.0f,maxY=0.0f;
            for (uint8_t q=0;q<4;++q)
            {
                if ((occupied & (1u << q)) == 0) continue;
                const auto& b=kQuarterBounds[q];
                minX=std::min(minX,b[0]); minY=std::min(minY,b[1]);
                maxX=std::max(maxX,b[2]); maxY=std::max(maxY,b[3]);
            }
            const float lowZ=float(large->getBaseZ());
            const float highZ=float(std::max(
                large->getClearanceZ(), large->getBaseZ() + std::max(0, tile.zClearance)));
            if (highZ <= lowZ)
                return std::nullopt;
            const FirstPersonVec3 low{float(ps.MapPos.x)+minX,float(ps.MapPos.y)+minY,lowZ};
            const FirstPersonVec3 high{float(ps.MapPos.x)+maxX,float(ps.MapPos.y)+maxY,highZ};
            const FirstPersonVec3 center{0.5f*(low.x+high.x),0.5f*(low.y+high.y),0.5f*(low.z+high.z)};
            const float hx=0.5f*(high.x-low.x),hy=0.5f*(high.y-low.y),hz=0.5f*(high.z-low.z);
            return FirstPersonSemanticSphere{center,std::sqrt(hx*hx+hy*hy+hz*hz)+2.0f};
        }
        enum class LargeSceneryAssetFaceKind : uint8_t
        {
            minX,
            maxX,
            minY,
            maxY,
            top,
        };

        struct LargeSceneryAssetCell
        {
            int32_t qx{};
            int32_t qy{};
            int32_t lowZ{};
            int32_t highZ{};
            uint16_t sequence{};
        };

        struct LargeSceneryAssetFace
        {
            std::array<CoordsXYZ, 4> corners{};
            uint16_t sequence{};
            LargeSceneryAssetFaceKind kind{};
            // Native large-scenery image direction, not world viewport rotation.
            uint8_t sourceDirection{};
        };

        struct LargeSceneryAssetModel
        {
            bool attempted = false;
            bool reliable = false;
            int32_t heightTrim = 0;
            uint32_t bodyImageFirst = 0;
            uint32_t bodyImageLast = 0;
            FirstPersonMultiViewFit fit{};
            float minimumFaceSourceCoverage = 0.0f;
            float minimumFaceOwnership = 0.0f;
            FirstPersonVec3 low{};
            FirstPersonVec3 high{};
            std::vector<LargeSceneryAssetFace> faces;
        };
        static std::unordered_map<const LargeSceneryEntry*, LargeSceneryAssetModel> _largeSceneryAssetModels;

        [[nodiscard]] uint64_t LargeSceneryQuarterCellKey(int32_t qx, int32_t qy)
        {
            return (uint64_t(uint32_t(qx)) << 32) | uint32_t(qy);
        }

        [[nodiscard]] bool LargeSceneryAssetEligible(const LargeSceneryEntry& entry)
        {
            return !entry.flags.hasAny(
                       LargeSceneryFlag::isTree,
                       LargeSceneryFlag::isAnimated,
                       LargeSceneryFlag::is3DText)
                && entry.scrolling_mode == kScrollingModeNone
                && !entry.tiles.empty() && entry.tiles.size() <= 64;
        }

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

        [[nodiscard]] std::vector<LargeSceneryAssetFace> BuildLargeSceneryAssetFaces(
            const std::vector<LargeSceneryAssetCell>& cells, int32_t heightTrim)
        {
            std::unordered_map<
                uint64_t, std::vector<const LargeSceneryAssetCell*>> lookup;
            lookup.reserve(cells.size());
            for (const auto& cell : cells)
            {
                lookup[LargeSceneryQuarterCellKey(cell.qx, cell.qy)]
                    .push_back(&cell);
            }

            const auto effectiveHigh =
                [heightTrim](const LargeSceneryAssetCell& cell) {
                    return std::max(
                        cell.lowZ + 1, cell.highZ - heightTrim);
                };
            const auto coverageAt =
                [&](int32_t qx, int32_t qy) {
                    std::vector<FirstPersonVerticalInterval> intervals;
                    const auto found = lookup.find(
                        LargeSceneryQuarterCellKey(qx, qy));
                    if (found == lookup.end())
                        return intervals;
                    intervals.reserve(found->second.size());
                    for (const auto* cell : found->second)
                    {
                        const int32_t high = effectiveHigh(*cell);
                        if (high > cell->lowZ)
                            intervals.push_back(
                                { cell->lowZ, high });
                    }
                    return intervals;
                };

            std::vector<LargeSceneryAssetFace> faces;
            faces.reserve(cells.size() * 5);
            std::unordered_map<
                uint64_t, std::vector<FirstPersonVerticalInterval>> claimed;

            const auto appendSideStrip =
                [&](const LargeSceneryAssetCell& cell,
                    FirstPersonVerticalInterval strip,
                    LargeSceneryAssetFaceKind kind) {
                    if (strip.high <= strip.low)
                        return;
                    const int32_t x0 = cell.qx * 16;
                    const int32_t y0 = cell.qy * 16;
                    const int32_t x1 = x0 + 16;
                    const int32_t y1 = y0 + 16;
                    LargeSceneryAssetFace face{};
                    face.sequence = cell.sequence;
                    face.kind = kind;
                    switch (kind)
                    {
                        case LargeSceneryAssetFaceKind::minX:
                            face.corners = { {
                                { x0, y0, strip.low },
                                { x0, y1, strip.low },
                                { x0, y1, strip.high },
                                { x0, y0, strip.high },
                            } };
                            break;
                        case LargeSceneryAssetFaceKind::maxX:
                            face.corners = { {
                                { x1, y1, strip.low },
                                { x1, y0, strip.low },
                                { x1, y0, strip.high },
                                { x1, y1, strip.high },
                            } };
                            break;
                        case LargeSceneryAssetFaceKind::minY:
                            face.corners = { {
                                { x1, y0, strip.low },
                                { x0, y0, strip.low },
                                { x0, y0, strip.high },
                                { x1, y0, strip.high },
                            } };
                            break;
                        case LargeSceneryAssetFaceKind::maxY:
                            face.corners = { {
                                { x0, y1, strip.low },
                                { x1, y1, strip.low },
                                { x1, y1, strip.high },
                                { x0, y1, strip.high },
                            } };
                            break;
                        case LargeSceneryAssetFaceKind::top:
                            return;
                    }
                    faces.push_back(std::move(face));
                };

            for (const auto& cell : cells)
            {
                const int32_t highZ = effectiveHigh(cell);
                if (highZ <= cell.lowZ)
                    continue;

                const uint64_t key =
                    LargeSceneryQuarterCellKey(cell.qx, cell.qy);
                auto& alreadyClaimed = claimed[key];
                const auto ownedFragments =
                    SubtractFirstPersonVerticalCoverage(
                        { cell.lowZ, highZ }, alreadyClaimed);
                alreadyClaimed.push_back({ cell.lowZ, highZ });

                for (const auto fragment : ownedFragments)
                {
                    const auto appendExposedSides =
                        [&](int32_t dx, int32_t dy,
                            LargeSceneryAssetFaceKind kind) {
                            const auto exposed =
                                SubtractFirstPersonVerticalCoverage(
                                    fragment,
                                    coverageAt(
                                        cell.qx + dx,
                                        cell.qy + dy));
                            for (const auto strip : exposed)
                                appendSideStrip(cell, strip, kind);
                        };

                    appendExposedSides(
                        -1, 0,
                        LargeSceneryAssetFaceKind::minX);
                    appendExposedSides(
                        1, 0,
                        LargeSceneryAssetFaceKind::maxX);
                    appendExposedSides(
                        0, -1,
                        LargeSceneryAssetFaceKind::minY);
                    appendExposedSides(
                        0, 1,
                        LargeSceneryAssetFaceKind::maxY);

                    // Only the actual upper end of this source interval can
                    // own a roof. Internal fragment boundaries created by
                    // overlapping cells are solid union boundaries, not roofs.
                    if (fragment.high != highZ)
                        continue;

                    auto sameColumn = coverageAt(cell.qx, cell.qy);
                    bool coveredAbove = false;
                    for (const auto interval : sameColumn)
                    {
                        if (interval.low <= highZ
                            && interval.high > highZ)
                        {
                            coveredAbove = true;
                            break;
                        }
                    }
                    if (coveredAbove)
                        continue;

                    const int32_t x0 = cell.qx * 16;
                    const int32_t y0 = cell.qy * 16;
                    LargeSceneryAssetFace roof{};
                    roof.sequence = cell.sequence;
                    roof.kind = LargeSceneryAssetFaceKind::top;
                    roof.corners = { {
                        { x0, y0, highZ },
                        { x0 + 16, y0, highZ },
                        { x0 + 16, y0 + 16, highZ },
                        { x0, y0 + 16, highZ },
                    } };
                    faces.push_back(std::move(roof));
                }
            }
            return faces;
        }

        [[nodiscard]] bool LargeSceneryFaceVisibleFromDirection(
            LargeSceneryAssetFaceKind kind, uint8_t direction)
        {
            if (kind == LargeSceneryAssetFaceKind::top)
                return true;
            CoordsXY normal{};
            switch (kind)
            {
                case LargeSceneryAssetFaceKind::minX: normal = { -1, 0 }; break;
                case LargeSceneryAssetFaceKind::maxX: normal = { 1, 0 }; break;
                case LargeSceneryAssetFaceKind::minY: normal = { 0, -1 }; break;
                case LargeSceneryAssetFaceKind::maxY: normal = { 0, 1 }; break;
                case LargeSceneryAssetFaceKind::top: return true;
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

        [[nodiscard]] std::array<FirstPersonSilhouette, 4> RasterizeLargeSceneryAssetViews(
            const std::vector<LargeSceneryAssetFace>& faces)
        {
            std::array<FirstPersonSilhouette, 4> result{};
            for (uint8_t rotation = 0; rotation < 4; ++rotation)
            {
                for (const auto& face : faces)
                {
                    if (!LargeSceneryFaceVisibleFromDirection(face.kind, rotation))
                        continue;
                    std::array<ScreenCoordsXY, 4> projected{};
                    for (size_t i = 0; i < face.corners.size(); ++i)
                        projected[i] = Translate3DTo2DWithZ(rotation, face.corners[i]);
                    AddFirstPersonSilhouetteQuad(result[rotation], projected);
                }
            }
            return result;
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
            model.bodyImageLast = model.bodyImageFirst + uint32_t(entry.tiles.size() * 4);
            model.attempted = true;
            if (!LargeSceneryAssetEligible(entry))
                return model;

            const auto cells = BuildLargeSceneryAssetCells(entry);
            constexpr size_t kMaxReconstructionCells = 48;
            if (!cells.has_value() || cells->size() > kMaxReconstructionCells)
                return model;

            const auto observed = CollectLargeSceneryObservedViews(entry);
            if (!observed.valid)
                return model;

            static constexpr std::array<int32_t, 7> kHeightTrims{ { 0, 2, 4, 6, 8, 12, 16 } };
            float bestScore = -std::numeric_limits<float>::infinity();
            std::vector<LargeSceneryAssetFace> bestFaces;
            for (const int32_t trim : kHeightTrims)
            {
                const auto faces = BuildLargeSceneryAssetFaces(*cells, trim);
                constexpr size_t kMaxReconstructionFaces = 144;
                if (faces.empty() || faces.size() > kMaxReconstructionFaces
                    || !EstimateLargeSceneryAssetRasterWork(faces).has_value())
                    continue;
                const auto candidate = RasterizeLargeSceneryAssetViews(faces);
                size_t candidatePixels = 0;
                for (const auto& view : candidate)
                    candidatePixels += view.size();
                constexpr size_t kMaxCandidatePixels = 393216;
                if (candidatePixels > kMaxCandidatePixels)
                    continue;
                const auto fit = CompareFirstPersonMultiViewSilhouettes(
                    observed.combined, candidate);
                if (!fit.valid)
                    continue;
                const float score = fit.averageIntersectionOverUnion
                    - 0.0025f * float(fit.maximumEdgeError);
                if (score <= bestScore)
                    continue;
                bestScore = score;
                model.heightTrim = trim;
                model.fit = fit;
                bestFaces = faces;
            }
            if (bestFaces.empty() || !IsFirstPersonMultiViewFitReliable(model.fit))
                return model;

            const auto depthOwners =
                BuildLargeSceneryAssetDepthOwners(bestFaces);
            float minimumFaceCoverage = 1.0f;
            float minimumFaceOwnership = 1.0f;
            for (size_t faceIndex = 0; faceIndex < bestFaces.size(); ++faceIndex)
            {
                auto& face = bestFaces[faceIndex];
                float bestFaceScore = -1.0f;
                float bestCoverage = 0.0f;
                float bestOwnership = 0.0f;
                uint8_t bestDirection = 0;
                for (uint8_t direction = 0; direction < 4; ++direction)
                {
                    if (!LargeSceneryFaceVisibleFromDirection(face.kind, direction))
                        continue;
                    const auto projectedFace =
                        RasterizeLargeSceneryAssetFace(face, direction);
                    if (projectedFace.empty())
                        continue;

                    const float ownership = FirstPersonDepthOwnerCoverage(
                        depthOwners[direction], uint32_t(faceIndex), projectedFace);
                    // Silhouette agreement says "something is opaque here";
                    // the depth-owner map additionally says THIS face owns it.
                    // Reject source views where another candidate surface is in
                    // front rather than baking foreground pixels onto a recess.
                    if (ownership < 0.97f)
                        continue;

                    const auto& source =
                        observed.bySequence[face.sequence][direction];
                    const auto fit =
                        CompareFirstPersonSilhouettes(source, projectedFace);
                    if (!fit.valid)
                        continue;
                    const float score =
                        std::min(fit.candidateCoverage, ownership);
                    if (score <= bestFaceScore)
                        continue;
                    bestFaceScore = score;
                    bestCoverage = fit.candidateCoverage;
                    bestOwnership = ownership;
                    bestDirection = direction;
                }
                if (bestFaceScore < 0.0f)
                    return model;
                face.sourceDirection = bestDirection;
                minimumFaceCoverage =
                    std::min(minimumFaceCoverage, bestCoverage);
                minimumFaceOwnership =
                    std::min(minimumFaceOwnership, bestOwnership);
            }
            model.minimumFaceSourceCoverage = minimumFaceCoverage;
            model.minimumFaceOwnership = minimumFaceOwnership;
            if (minimumFaceCoverage < 0.55f || minimumFaceOwnership < 0.97f)
                return model;

            model.low = {
                float((*cells)[0].qx * 16),
                float((*cells)[0].qy * 16),
                float((*cells)[0].lowZ),
            };
            model.high = model.low;
            for (const auto& face : bestFaces)
            for (const auto& corner : face.corners)
            {
                model.low.x = std::min(model.low.x, float(corner.x));
                model.low.y = std::min(model.low.y, float(corner.y));
                model.low.z = std::min(model.low.z, float(corner.z));
                model.high.x = std::max(model.high.x, float(corner.x));
                model.high.y = std::max(model.high.y, float(corner.y));
                model.high.z = std::max(model.high.z, float(corner.z));
            }
            model.faces = std::move(bestFaces);
            model.reliable = true;
            return model;
        }

        [[nodiscard]] const LargeSceneryAssetModel* GetLargeSceneryAssetModel(
            const LargeSceneryEntry& entry, bool allowBuild)
        {
            auto [it, inserted] = _largeSceneryAssetModels.try_emplace(&entry);
            if (inserted)
                it->second = {};
            if (!it->second.attempted)
            {
                if (!allowBuild)
                    return nullptr;
                it->second = BuildLargeSceneryAssetModel(entry);
            }
            return &it->second;
        }

        int32_t SemanticClearanceZ(const TileElement& element)
        {
            int32_t result=element.getClearanceZ();
            const auto* large=element.asLargeScenery();
            if(large==nullptr) return result;
            const auto* entry=large->getEntry();
            const size_t sequence=large->getSequenceIndex();
            if(entry==nullptr || sequence>=entry->tiles.size()) return result;
            return std::max(result,element.getBaseZ()+std::max(0,entry->tiles[sequence].zClearance));
        }

        // Culling on the raw PaintStruct's SORTING bounds is unsafe: its
        // geometry may be far smaller (or in another location) than the art
        // derived from its source. Test the final physical surface, and cache
        // ALL static surfaces even when their present view does not show them.
        bool SurfaceMayBeVisible(const FirstPersonSurface& surface, const FirstPersonFrustum& view)
        {
            if (surface.hasSemanticBounds && view.visible(surface.semanticCenter,surface.semanticRadius))
                return true;
            const auto& v=surface.triangles;
            auto low=v[0].world, high=v[0].world;
            for (const auto& vertex:v)
            {
                const auto& p=vertex.world;
                low.x=std::min(low.x,p.x); low.y=std::min(low.y,p.y); low.z=std::min(low.z,p.z);
                high.x=std::max(high.x,p.x); high.y=std::max(high.y,p.y); high.z=std::max(high.z,p.z);
            }
            const FirstPersonVec3 center{(low.x+high.x)*0.5f,(low.y+high.y)*0.5f,(low.z+high.z)*0.5f};
            const float x=(high.x-low.x)*0.5f,y=(high.y-low.y)*0.5f,z=(high.z-low.z)*0.5f;
            return view.visible(center,std::sqrt(x*x+y*y+z*z)+2.0f);
        }
        void AppendSemanticPathDeck(
            FirstPersonScene& scene, const PaintStruct& ps, ImageId image, uint8_t rotation)
        {
            const auto* g1 = image.HasValue() ? GfxGetG1Element(image) : nullptr;
            const auto* path = ps.Element != nullptr ? ps.Element->asPath() : nullptr;
            if (g1 == nullptr || path == nullptr) return;
            const auto origin = ps.MapPos;
            const int32_t baseZ = path->getBaseZ();
            // The semantic deck may be synthesized from a bridge/support root.
            // Its UV origin must still match the native {0,0,baseZ} surface
            // sprite placement, not whichever root happened to expose PathElement.
            const auto spriteOrigin = GetTileElementPaintSpritePosition(origin, rotation);
            const auto spritePos = Translate3DTo2DWithZ(rotation, { spriteOrigin, baseZ });
            const auto slope = path->isSloped() ? kPathSlopeToLandSlope[path->getSlopeDirection()] : kTileSlopeFlat;
            const auto heights = GetSlopeCornerHeights(baseZ, slope);
            // Geometry is the REAL walking plane. Do not raise it to solve
            // z-fighting; depth separation is a rendering concern.
            const std::array<CoordsXYZ, 4> world = { {
                { origin.x, origin.y, heights.south },
                { origin.x + kCoordsXYStep, origin.y, heights.east },
                { origin.x + kCoordsXYStep, origin.y + kCoordsXYStep, heights.north },
                { origin.x, origin.y + kCoordsXYStep, heights.west },
            } };
            FirstPersonSurface surface{};
            surface.image = image;
            surface.depthBias = true;
            std::array<FirstPersonVertex, 4> v{};
            for (size_t i = 0; i < v.size(); ++i)
            {
                const auto iso = Translate3DTo2DWithZ(rotation, world[i]);
                v[i] = { { float(world[i].x), float(world[i].y), float(world[i].z) },
                         float(iso.x - spritePos.x - g1->xOffset),
                         float(iso.y - spritePos.y - g1->yOffset) };
            }
            EmitQuad(surface, v, UsesOppositeTerrainDiagonal(slope));
            scene.surfaces.emplace_back(std::move(surface));
        }
        enum class AttachedVehicleComponentRole : uint8_t
        {
            other,
            body,
            rider,
        };

        struct AttachedVehicleComponent
        {
            AttachedVehicleComponentRole role =
                AttachedVehicleComponentRole::other;
            uint8_t riderRow = 0xFF;
        };

        [[nodiscard]] AttachedVehicleComponent
            GetAttachedVehicleComponent(
                const Vehicle& vehicle, ImageId image)
        {
            AttachedVehicleComponent result{};
            const auto visual = ResolveVehicleVisualState(vehicle);
            const auto* entry = visual.carEntry;
            if (entry == nullptr || !image.HasValue()
                || entry->numCarImages == 0)
                return result;

            const uint32_t index = image.GetIndex();
            if (index < entry->baseImageId)
                return result;
            const uint32_t delta = index - entry->baseImageId;
            const uint32_t block = delta / entry->numCarImages;
            if (block == 0)
            {
                result.role = AttachedVehicleComponentRole::body;
                return result;
            }
            if (block <= entry->numSeatingRows)
            {
                result.role = AttachedVehicleComponentRole::rider;
                result.riderRow = uint8_t(block - 1);
            }
            return result;
        }

        [[nodiscard]] const FirstPersonVehicleBodyHull*
            GetUsableAttachedVehicleHull(
                const Vehicle& vehicle, uint8_t seatIndex)
        {
            if (seatIndex == 0xFF
                || vehicle.flags.has(
                    VehicleFlag::carIsReversed))
                return nullptr;
            const auto* ride = vehicle.GetRide();
            const auto visual =
                ResolveVehicleVisualState(vehicle);
            const auto* entry = visual.carEntry;
            if (ride == nullptr || entry == nullptr
                || !ride->getRideTypeDescriptor().flags.has(RtdFlag::hasTrack))
                return nullptr;

            const auto* hull =
                GetFirstPersonVehicleBodyHull(*entry);
            const auto* seat =
                GetFirstPersonPassengerAssetSeat(*entry, seatIndex);
            if (hull == nullptr || seat == nullptr
                || hull->containsPoint(seat->localEye))
                return nullptr;
            return hull;
        }

        [[nodiscard]] FirstPersonVec3 TransformVehicleLocalPoint(
            FirstPersonVec3 origin, const FirstPersonBasis& basis,
            FirstPersonVec3 local)
        {
            return {
                origin.x + basis.forward.x * local.x
                    + basis.right.x * local.y
                    + basis.up.x * local.z,
                origin.y + basis.forward.y * local.x
                    + basis.right.y * local.y
                    + basis.up.y * local.z,
                origin.z + basis.forward.z * local.x
                    + basis.right.z * local.y
                    + basis.up.z * local.z,
            };
        }

        void AppendAttachedVehicleHull(
            FirstPersonScene& scene, const Vehicle& vehicle,
            const FirstPersonVehicleBodyHull& hull,
            ImageId bodyImageTemplate)
        {
            const auto presentation =
                BuildFirstPersonVehiclePresentationState(
                    vehicle,
                    EntityTweener::get().trackedVehicleVisuals(
                        vehicle.id));
            const auto& carriage = presentation.carriage;
            FirstPersonVec3 origin{
                presentation.vehicleOrigin.x
                    + carriage.originOffset.x,
                presentation.vehicleOrigin.y
                    + carriage.originOffset.y,
                presentation.vehicleOrigin.z
                    + carriage.originOffset.z,
            };
            const auto& basis = carriage.basis;

            const auto colourMap =
                Drawing::getColourMap(vehicle.colours.Body);
            uint8_t sideColour =
                static_cast<uint8_t>(colourMap.midDark);
            uint8_t topColour =
                static_cast<uint8_t>(colourMap.midLight);
            if (sideColour == 0)
                sideColour = static_cast<uint8_t>(
                    Drawing::PaletteIndex::trackRails1);
            if (topColour == 0)
                topColour = static_cast<uint8_t>(
                    Drawing::PaletteIndex::trackRails2);

            struct VehicleFaceSource
            {
                ImageId image{};
                const G1Element* g1 = nullptr;
                uint8_t direction = 0;
            };

            const auto sourceForFace =
                [&](const std::array<FirstPersonVec3, 4>& local,
                    FirstPersonVec3 normal)
                    -> std::optional<VehicleFaceSource> {
                    // Native vehicle art is always viewed from above. There is
                    // no honest source for the underside, so keep the solid
                    // fallback there rather than reflecting a top texture.
                    if (normal.z < -0.5f
                        || hull.textureViews.empty())
                        return std::nullopt;

                    const FirstPersonVec3 centre{
                        0.25f * (local[0].x + local[1].x
                            + local[2].x + local[3].x),
                        0.25f * (local[0].y + local[1].y
                            + local[2].y + local[3].y),
                        0.25f * (local[0].z + local[1].z
                            + local[2].z + local[3].z),
                    };
                    float bestScore =
                        -std::numeric_limits<float>::infinity();
                    std::optional<VehicleFaceSource> best;
                    constexpr float kTwoPi =
                        6.28318530717958647692f;

                    for (const auto& view : hull.textureViews)
                    {
                        const auto image =
                            bodyImageTemplate.WithIndex(view.image);
                        const auto* g1 = GfxGetG1Element(image);
                        if (g1 == nullptr || g1->width <= 0
                            || g1->height <= 0)
                            continue;

                        const float theta =
                            float(view.imageDirection & 31u)
                            * (kTwoPi / 32.0f);
                        const float cosine = std::cos(theta);
                        const float sine = std::sin(theta);
                        // FirstPersonVehicleLocalPoint's native depth
                        // increases toward this horizontal direction.
                        const float towardForward =
                            -cosine + sine;
                        const float towardRight =
                            -sine - cosine;
                        const float horizontalFacing =
                            normal.x * towardForward
                            + normal.y * towardRight;
                        const float facing =
                            normal.z > 0.5f
                            ? 1.0f
                            : horizontalFacing;
                        if (facing <= 0.05f)
                            continue;

                        const auto projectedCentre =
                            ProjectFirstPersonVehicleLocalPoint(
                                view.imageDirection, centre);
                        const int32_t centreX =
                            int32_t(std::lround(
                                projectedCentre[0]))
                            - g1->xOffset;
                        const int32_t centreY =
                            int32_t(std::lround(
                                projectedCentre[1]))
                            - g1->yOffset;
                        bool supported = false;
                        for (int32_t dy = -1; dy <= 1
                             && !supported; ++dy)
                        for (int32_t dx = -1; dx <= 1; ++dx)
                        {
                            if (FirstPersonVehicleBodyPixelOpaque(
                                    *g1, centreX + dx,
                                    centreY + dy))
                            {
                                supported = true;
                                break;
                            }
                        }
                        if (!supported)
                            continue;

                        std::array<std::array<float, 2>, 4>
                            projected{};
                        for (size_t i = 0;
                             i < projected.size(); ++i)
                        {
                            projected[i] =
                                ProjectFirstPersonVehicleLocalPoint(
                                    view.imageDirection, local[i]);
                        }
                        float twiceArea = 0.0f;
                        for (size_t i = 0; i < 4; ++i)
                        {
                            const size_t j = (i + 1) & 3u;
                            twiceArea +=
                                projected[i][0] * projected[j][1]
                                - projected[j][0]
                                    * projected[i][1];
                        }
                        const float area =
                            0.5f * std::abs(twiceArea);
                        const float score =
                            facing * std::max(0.25f, area);
                        if (!best.has_value()
                            || score > bestScore)
                        {
                            bestScore = score;
                            best = VehicleFaceSource{
                                image, g1,
                                view.imageDirection
                            };
                        }
                    }
                    return best;
                };

            const float half = hull.step * 0.5f;
            const auto appendFace =
                [&](const std::array<FirstPersonVec3, 4>& local,
                    FirstPersonVec3 normal, uint8_t fallbackColour) {
                    FirstPersonSurface surface{};
                    const auto source =
                        sourceForFace(local, normal);
                    std::array<FirstPersonVertex, 4> vertices{};
                    for (size_t i = 0; i < vertices.size(); ++i)
                    {
                        vertices[i].world =
                            TransformVehicleLocalPoint(
                                origin, basis, local[i]);
                        if (source.has_value())
                        {
                            const auto projected =
                                ProjectFirstPersonVehicleLocalPoint(
                                    source->direction, local[i]);
                            vertices[i].u =
                                projected[0]
                                - float(source->g1->xOffset);
                            vertices[i].v =
                                projected[1]
                                - float(source->g1->yOffset);
                        }
                    }
                    if (source.has_value())
                    {
                        surface.image = source->image;
                    }
                    else
                    {
                        surface.solidColour = fallbackColour;
                    }
                    EmitQuad(surface, vertices);
                    scene.surfaces.emplace_back(
                        std::move(surface));
                };

            for (int32_t up = 0; up < hull.sizeUp; ++up)
            for (int32_t right = 0; right < hull.sizeRight; ++right)
            for (int32_t forward = 0;
                 forward < hull.sizeForward; ++forward)
            {
                if (!hull.contains(forward, right, up))
                    continue;
                const auto centre =
                    hull.centre(forward, right, up);
                const float f0 = centre.x - half;
                const float f1 = centre.x + half;
                const float r0 = centre.y - half;
                const float r1 = centre.y + half;
                const float u0 = centre.z - half;
                const float u1 = centre.z + half;

                if (!hull.contains(forward - 1, right, up))
                    appendFace({ {
                        { f0, r1, u0 }, { f0, r0, u0 },
                        { f0, r0, u1 }, { f0, r1, u1 },
                    } }, { -1.0f, 0.0f, 0.0f }, sideColour);
                if (!hull.contains(forward + 1, right, up))
                    appendFace({ {
                        { f1, r0, u0 }, { f1, r1, u0 },
                        { f1, r1, u1 }, { f1, r0, u1 },
                    } }, { 1.0f, 0.0f, 0.0f }, sideColour);
                if (!hull.contains(forward, right - 1, up))
                    appendFace({ {
                        { f0, r0, u0 }, { f1, r0, u0 },
                        { f1, r0, u1 }, { f0, r0, u1 },
                    } }, { 0.0f, -1.0f, 0.0f }, sideColour);
                if (!hull.contains(forward, right + 1, up))
                    appendFace({ {
                        { f1, r1, u0 }, { f0, r1, u0 },
                        { f0, r1, u1 }, { f1, r1, u1 },
                    } }, { 0.0f, 1.0f, 0.0f }, sideColour);
                if (!hull.contains(forward, right, up - 1))
                    appendFace({ {
                        { f0, r0, u0 }, { f0, r1, u0 },
                        { f1, r1, u0 }, { f1, r0, u0 },
                    } }, { 0.0f, 0.0f, -1.0f }, sideColour);
                if (!hull.contains(forward, right, up + 1))
                    appendFace({ {
                        { f0, r1, u1 }, { f0, r0, u1 },
                        { f1, r0, u1 }, { f1, r1, u1 },
                    } }, { 0.0f, 0.0f, 1.0f }, topColour);
            }
        }

        [[nodiscard]] bool ApplyAdjacentRiderMask(
            FirstPersonSurface& surface, ImageId image,
            bool hideSecondary)
        {
            const auto* g1 =
                image.HasValue() ? GfxGetG1Element(image) : nullptr;
            if (g1 == nullptr || g1->offset == nullptr
                || g1->width <= 0 || g1->height <= 0
                || g1->width > 512 || g1->height > 512
                || g1->flags.has(G1Flag::isPalette))
                return false;

            const size_t width = size_t(g1->width);
            const size_t height = size_t(g1->height);
            std::vector<uint8_t> pixels(width * height, 0);
            if (g1->flags.has(G1Flag::hasRLECompression))
            {
                for (int32_t y = 0; y < g1->height; ++y)
                {
                    const uint16_t lineOffset =
                        uint16_t(g1->offset[y * 2])
                        | (uint16_t(g1->offset[y * 2 + 1]) << 8);
                    const uint8_t* run = g1->offset + lineOffset;
                    bool endOfLine = false;
                    size_t guard = 0;
                    while (!endOfLine && guard++ < 256)
                    {
                        uint8_t length = *run++;
                        const int32_t x = *run++;
                        endOfLine = (length & 0x80u) != 0;
                        length &= 0x7Fu;
                        for (uint8_t n = 0; n < length; ++n)
                        {
                            if (x + n >= 0 && x + n < g1->width)
                                pixels[size_t(y) * width
                                    + size_t(x + n)] = run[n];
                        }
                        run += length;
                    }
                    if (!endOfLine)
                        return false;
                }
            }
            else
            {
                std::copy_n(g1->offset, pixels.size(), pixels.begin());
            }

            double primaryX = 0.0;
            double primaryY = 0.0;
            double secondaryX = 0.0;
            double secondaryY = 0.0;
            size_t primaryCount = 0;
            size_t secondaryCount = 0;
            for (int32_t y = 0; y < g1->height; ++y)
            for (int32_t x = 0; x < g1->width; ++x)
            {
                const uint8_t pixel =
                    pixels[size_t(y) * width + size_t(x)];
                if (FirstPersonRiderPixelUsesPrimaryRemap(pixel))
                {
                    primaryX += x;
                    primaryY += y;
                    ++primaryCount;
                }
                if (FirstPersonRiderPixelUsesSecondaryRemap(pixel))
                {
                    secondaryX += x;
                    secondaryY += y;
                    ++secondaryCount;
                }
            }
            if (primaryCount < 3 || secondaryCount < 3)
                return false;
            primaryX /= double(primaryCount);
            primaryY /= double(primaryCount);
            secondaryX /= double(secondaryCount);
            secondaryY /= double(secondaryCount);

            const double selectedX =
                hideSecondary ? secondaryX : primaryX;
            const double selectedY =
                hideSecondary ? secondaryY : primaryY;
            const double otherX =
                hideSecondary ? primaryX : secondaryX;
            const double otherY =
                hideSecondary ? primaryY : secondaryY;
            size_t kept = 0;
            for (int32_t y = 0; y < g1->height; ++y)
            for (int32_t x = 0; x < g1->width; ++x)
            {
                auto& pixel =
                    pixels[size_t(y) * width + size_t(x)];
                if (pixel == 0)
                    continue;
                const double selectedDistance =
                    (double(x) - selectedX) * (double(x) - selectedX)
                    + (double(y) - selectedY) * (double(y) - selectedY);
                const double otherDistance =
                    (double(x) - otherX) * (double(x) - otherX)
                    + (double(y) - otherY) * (double(y) - otherY);
                const bool selectedRemap = hideSecondary
                    ? FirstPersonRiderPixelUsesSecondaryRemap(pixel)
                    : FirstPersonRiderPixelUsesPrimaryRemap(pixel);
                if (selectedRemap
                    || selectedDistance <= otherDistance)
                {
                    pixel = 0;
                }
                else
                {
                    ++kept;
                }
            }
            if (kept < 3)
                return false;

            uint64_t fingerprint = 14695981039346656037ull;
            ExtendStableKey(fingerprint, image.GetIndex());
            ExtendStableKey(fingerprint, hideSecondary ? 1 : 0);
            for (const auto pixel : pixels)
            {
                fingerprint ^= pixel;
                fingerprint *= 1099511628211ull;
            }
            surface.immutablePixels = std::move(pixels);
            surface.immutableWidth = g1->width;
            surface.immutableHeight = g1->height;
            surface.immutableFingerprint = fingerprint;
            return true;
        }

        bool AppendCalibratedSmallSceneryGeometry(
            FirstPersonScene& scene, const PaintStruct& ps,
            ImageId image, const ScreenCoordsXY& spritePos,
            uint8_t rotation, ImageId mask = {})
        {
            const auto* small =
                ps.Element != nullptr
                ? ps.Element->asSmallScenery()
                : nullptr;
            if (small == nullptr || !image.HasValue()
                || mask.HasValue())
                return false;
            const auto* entry = small->getEntry();
            if (entry == nullptr)
                return false;
            const uint8_t direction =
                small->getDirectionWithOffset(rotation) & 3u;
            const ImageIndex expectedBodyImage =
                entry->image + direction
                + uint32_t(FirstPersonSmallSceneryWitherStage(
                    *entry, *small)) * 4u;
            if (image.GetIndex() != expectedBodyImage)
                return false;

            const auto layout =
                GetSpriteCompositeLayout(image, mask);
            if (!layout.has_value())
                return false;

            std::vector<FirstPersonPhysicalBoxProxy> proxies;
            proxies.reserve(32);
            if (!AppendFirstPersonSmallSceneryProxies(
                    proxies, ps.MapPos, *small))
                return false;

            float reconstructedTop =
                -std::numeric_limits<float>::infinity();
            for (const auto& proxy : proxies)
                reconstructedTop =
                    std::max(reconstructedTop, proxy.high.z);
            const int32_t reconstructedHeight =
                int32_t(std::floor(
                    reconstructedTop
                    - float(small->getBaseZ()) + 0.5f));
            if (!FirstPersonSmallSceneryVisualReconstructionCoversHeight(
                    entry->height, reconstructedHeight))
            {
                // Walking collision deliberately samples only the lower body.
                // That is useful collision evidence, not permission to replace
                // taller native artwork with a truncated visual mesh.
                return false;
            }

            const auto emitFace =
                [&](const std::array<FirstPersonVec3, 4>& world) {
                    FirstPersonSurface surface{};
                    surface.image = image;
                    std::array<FirstPersonVertex, 4> vertices{};
                    for (size_t i = 0; i < vertices.size(); ++i)
                    {
                        const auto& p = world[i];
                        const CoordsXYZ loc{
                            int32_t(std::lround(p.x)),
                            int32_t(std::lround(p.y)),
                            int32_t(std::lround(p.z)),
                        };
                        const auto iso =
                            Translate3DTo2DWithZ(rotation, loc);
                        vertices[i] = {
                            p,
                            float(
                                iso.x - spritePos.x
                                - layout->xOffset),
                            float(
                                iso.y - spritePos.y
                                - layout->yOffset),
                        };
                    }
                    EmitQuad(surface, vertices);
                    scene.surfaces.emplace_back(
                        std::move(surface));
                };

            for (const auto& proxy : proxies)
            {
                const auto& low = proxy.low;
                const auto& high = proxy.high;
                // Top and four exposed vertical candidates. Internal faces
                // between adjacent run-length boxes are harmlessly occluded;
                // omitting bottoms avoids inventing artwork that native
                // isometric sprites never observe.
                emitFace({ {
                    { low.x, low.y, high.z },
                    { high.x, low.y, high.z },
                    { high.x, high.y, high.z },
                    { low.x, high.y, high.z },
                } });
                emitFace({ {
                    { low.x, low.y, low.z },
                    { high.x, low.y, low.z },
                    { high.x, low.y, high.z },
                    { low.x, low.y, high.z },
                } });
                emitFace({ {
                    { high.x, low.y, low.z },
                    { high.x, high.y, low.z },
                    { high.x, high.y, high.z },
                    { high.x, low.y, high.z },
                } });
                emitFace({ {
                    { high.x, high.y, low.z },
                    { low.x, high.y, low.z },
                    { low.x, high.y, high.z },
                    { high.x, high.y, high.z },
                } });
                emitFace({ {
                    { low.x, high.y, low.z },
                    { low.x, low.y, low.z },
                    { low.x, low.y, high.z },
                    { low.x, high.y, high.z },
                } });
            }
            return true;
        }

        [[nodiscard]] bool IsHiddenPassengerTileComponent(
            const PaintStruct& ps, EntityId hiddenEntity, uint8_t hiddenSeatIndex)
        {
            if (ps.Source != PaintStructSource::tile || ps.Entity == nullptr
                || hiddenEntity.IsNull() || ps.Entity->id != hiddenEntity
                || hiddenSeatIndex == 0xFF || !ps.image_id.HasValue())
                return false;

            const auto* vehicle = ps.Entity->as<Vehicle>();
            const auto* ride = vehicle != nullptr ? vehicle->GetRide() : nullptr;
            const auto* rideEntry = vehicle != nullptr ? vehicle->GetRideEntry() : nullptr;
            if (vehicle == nullptr || ride == nullptr || rideEntry == nullptr
                || ride->getRideTypeDescriptor().Name != "ferris_wheel")
                return false;

            return FirstPersonFerrisWheelImageMatchesSeatPair(
                rideEntry->Cars[0].baseImageId,
                ps.image_id.GetIndex(),
                vehicle->flatRideAnimationFrame,
                hiddenSeatIndex);
        }

        void AppendRoot(
            FirstPersonScene& scene, const PaintStruct& ps, const FirstPersonVec3& anchor,
            const FirstPersonBasis& basis, const ScreenCoordsXY& isoAnchor,
            uint32_t viewFlags, EntityId hidden, uint8_t hiddenSeatIndex,
            uint8_t rotation, bool emitPathDeck)
        {
            const bool matchesHiddenEntity =
                ps.Entity != nullptr && !hidden.IsNull()
                && ps.Entity->id == hidden;
            const auto* attachedVehicle =
                matchesHiddenEntity
                    && ps.Source == PaintStructSource::entity
                ? ps.Entity->as<Vehicle>() : nullptr;
            const auto* attachedHull =
                attachedVehicle != nullptr
                ? GetUsableAttachedVehicleHull(
                    *attachedVehicle, hiddenSeatIndex)
                : nullptr;
            const bool reconstructAttachedVehicle =
                attachedVehicle != nullptr && attachedHull != nullptr;
            const auto hiddenPolicy = FirstPersonHiddenComponentPolicy(
                matchesHiddenEntity && !reconstructAttachedVehicle,
                ps.Source == PaintStructSource::entity,
                matchesHiddenEntity
                    && IsHiddenPassengerTileComponent(
                        ps, hidden, hiddenSeatIndex));
            if (hiddenPolicy
                == FirstPersonHiddenComponentDisposition::suppressSubtree)
                return;
            if (hiddenPolicy
                == FirstPersonHiddenComponentDisposition::suppressSelfContinueChain)
            {
                // Children is a paint-chain link, not ownership of this one
                // rider component. Skip the selected passenger artwork while
                // still traversing later gondolas and the final support.
                if (ps.Children != nullptr)
                {
                    AppendRoot(
                        scene, *ps.Children, anchor, basis, isoAnchor,
                        viewFlags, hidden, hiddenSeatIndex, rotation, false);
                }
                return;
            }

            const auto visibility = GetPaintStructVisibility(&ps, viewFlags);
            if (visibility == VisibilityKind::hidden)
                return;
            const auto colourify = [&](ImageId id) {
                return visibility == VisibilityKind::partial
                    ? id.WithTransparency(Drawing::FilterPaletteID::paletteDarken1)
                    : id;
            };
            const auto* physicalTrack =
                ps.Element != nullptr ? ps.Element->asTrack() : nullptr;
            const bool physicallyPlanar =
                ps.Element != nullptr
                && (ps.Element->getType() == TileElementType::wall
                    || ps.Element->getType() == TileElementType::surface
                    || ps.Element->getType() == TileElementType::path
                    || (physicalTrack != nullptr
                        && trackTypeIsStation(
                            physicalTrack->getTrackType())));
            const auto* path = ps.Element != nullptr ? ps.Element->asPath() : nullptr;
            const auto* pathSurface = path != nullptr ? path->getSurfaceDescriptor() : nullptr;
            const auto spriteIndex = ps.image_id.GetIndex();
            const bool groundPathArtwork = pathSurface != nullptr && ps.image_id.HasValue()
                && spriteIndex >= pathSurface->image && spriteIndex < pathSurface->image + 51;

            // Emit one semantic walking deck per path element, regardless of
            // whether native bridge painting omitted the separate surface sprite.
            if (emitPathDeck && pathSurface != nullptr && ps.image_id.HasValue())
            {
                auto deckImage = ps.image_id.WithIndex(
                    pathSurface->image + GetPathSurfaceImageOffset(*path, rotation));
                AppendSemanticPathDeck(scene, ps, colourify(deckImage), rotation);
            }

            bool suppressCurrentImage = false;
            if (reconstructAttachedVehicle)
            {
                const auto component =
                    GetAttachedVehicleComponent(
                        *attachedVehicle, ps.image_id);
                if (component.role
                    == AttachedVehicleComponentRole::body)
                {
                    AppendAttachedVehicleHull(
                        scene, *attachedVehicle, *attachedHull,
                        colourify(ps.image_id));
                    suppressCurrentImage = true;
                }
                else if (component.role
                    == AttachedVehicleComponentRole::rider)
                {
                    const uint8_t selectedRow =
                        hiddenSeatIndex / 2;
                    if (component.riderRow == selectedRow)
                    {
                        const uint8_t rowBase =
                            uint8_t(selectedRow * 2);
                        const uint8_t adjacentSeat =
                            (hiddenSeatIndex & 1u) != 0
                            ? rowBase : uint8_t(rowBase + 1);
                        if (attachedVehicle->num_peeps > adjacentSeat)
                        {
                            const auto surfaceStart =
                                scene.surfaces.size();
                            AppendLayer(
                                scene, anchor, basis, isoAnchor,
                                colourify(ps.image_id), ps.ScreenPos);
                            if (scene.surfaces.size() > surfaceStart
                                && !ApplyAdjacentRiderMask(
                                    scene.surfaces.back(),
                                    ps.image_id,
                                    (hiddenSeatIndex & 1u) != 0))
                            {
                                scene.surfaces.pop_back();
                            }
                        }
                        suppressCurrentImage = true;
                    }
                }
                else
                {
                    // Unknown special component of an otherwise reconstructable
                    // attached car: do not let an uncalibrated close billboard
                    // reintroduce self-clipping.
                    suppressCurrentImage = true;
                }
            }

            // Native path surface sprites are represented by the semantic deck
            // above. Bridge/support sprites remain artwork and are never
            // flattened into the walking plane.
            if (!groundPathArtwork && !suppressCurrentImage)
            {
                const auto surfaceStart = scene.surfaces.size();
                const bool smallPhysical =
                    AppendCalibratedSmallSceneryGeometry(
                        scene, ps, colourify(ps.image_id),
                        ps.ScreenPos, rotation);
                if (!smallPhysical
                    && !AppendSemanticWallPlane(
                        scene, ps, colourify(ps.image_id),
                        ps.ScreenPos, rotation)
                    && (!physicallyPlanar
                        || !AppendPhysicalPlane(
                            scene, ps, colourify(ps.image_id),
                            ps.ScreenPos, rotation)))
                {
                    AppendLayer(
                        scene, anchor, basis, isoAnchor,
                        colourify(ps.image_id), ps.ScreenPos);
                }
                if (scene.surfaces.size() > surfaceStart
                    && !smallPhysical)
                {
                    ApplyImmutablePaintSnapshot(
                        scene.surfaces.back(),
                        ps.FirstPersonSnapshot);
                }
            }
            if (ps.Children != nullptr)
            {
                AppendRoot(
                    scene, *ps.Children, anchor, basis, isoAnchor,
                    viewFlags, hidden, hiddenSeatIndex, rotation, false);
            }
            else
            {
                for (auto* a = ps.Attached; a != nullptr; a = a->NextEntry)
                {
                    const auto colourImage = colourify(a->IsMasked ? a->ColourImageId : a->image_id);
                    const auto maskImage = a->IsMasked ? a->image_id : ImageId{};
                    const auto position = ps.ScreenPos + a->RelativePos;
                    const auto surfaceStart = scene.surfaces.size();
                    if (!AppendSemanticWallPlane(scene, ps, colourImage, position, rotation, maskImage) &&
                        (!physicallyPlanar ||
                         !AppendPhysicalPlane(scene, ps, colourImage, position, rotation, maskImage)))
                    {
                        AppendLayer(
                            scene, anchor, basis, isoAnchor, colourImage, position, maskImage);
                    }
                    if (scene.surfaces.size() > surfaceStart)
                        ApplyImmutablePaintSnapshot(scene.surfaces.back(), a->FirstPersonSnapshot);
                }
            }
        }

        // Terrain is static between edits, but changes in grass length, custom
        // terrain images, terraforming and water level are authoritative game
        // state. Validate a cheap semantic signature on EVERY visible tile,
        // rather than maintain an unrelated list of all map mutation hooks.
        struct TerrainCacheEntry
        {
            ImageId source{};
            int32_t baseZ{}, waterZ{};
            uint8_t slope{};
            uint8_t sourceRotation{};
            bool verticalOpening = false;
            int32_t spriteX{}, spriteY{}, spriteWidth{}, spriteHeight{};
            FirstPersonSurface ground{};
            std::optional<FirstPersonSurface> water;
            std::optional<FirstPersonSurface> waterOverlay;
            ImageId waterMaskImage{}, waterOverlayImage{};
            uint64_t lastSeen{};
            bool dirty = true;
        };
        struct TerrainCache
        {
            std::unordered_map<uint64_t, TerrainCacheEntry> entries;
            uint64_t frame{};
        };
        static TerrainCache _terrainCache;
        static uint64_t _sceneEpoch = 1;
        // Persistent NATIVE PAINT results, separated from dynamic entity sprites.
        // A cached surface never retains a PaintStruct/TileElement pointer: all
        // native session pointers expire immediately after PaintSessionFree.
        struct StaticPaintRotationCache
        {
            uint64_t lastPainted{};
            uint32_t lastAnimationGeneration{};
            uint32_t lastSourceProbeGeneration{};
            uint64_t residentFingerprint{};
            bool valid = false;
            uint8_t verticalTunnelHeight = 0xFF;
            std::vector<TunnelEntry> leftTunnels;
            std::vector<TunnelEntry> rightTunnels;
            std::vector<FirstPersonSurface> residentSurfaces;
            std::vector<FirstPersonSurface> streamedSurfaces;
        };
        struct StaticPaintCacheEntry
        {
            uint64_t signature{};
            uint64_t lastSeen{};
            uint32_t lastSemanticGeneration{};
            uint32_t viewFlags{};
            bool valid = false;
            bool dirty = true;
            bool animated = false;
            bool hasSelectedRotation = false;
            uint8_t selectedRotation = 0;
            bool hasUngroupedResident = false;
            bool visibilityBoundValid = false;
            bool visibilityDirty = true;
            int32_t visibilityMinZ = 0;
            int32_t visibilityMaxZ = 0;
            uint32_t lastVisibilityGeneration = 0;
            std::vector<ReconstructionGroupInfo> reconstructionGroups;
            // Keep all four native quarter-turn variants. Crossing a viewpoint
            // boundary can paint a variant once without destroying the previous
            // one, so moving back and forth does not thrash the whole park.
            std::array<StaticPaintRotationCache, 4> rotations;
        };
        static std::unordered_map<uint64_t, StaticPaintCacheEntry> _staticPaintCache;

        [[nodiscard]] std::optional<uint8_t> CachedVerticalTunnelHeight(uint64_t key)
        {
            const auto it = _staticPaintCache.find(key);
            if (it == _staticPaintCache.end())
                return std::nullopt;
            for (const auto& variant : it->second.rotations)
            {
                if (variant.valid)
                    return variant.verticalTunnelHeight;
            }
            return std::nullopt;
        }

        struct ReconstructionRotationState
        {
            uint64_t lastSeen{};
            bool hasSelectedRotation = false;
            uint8_t selectedRotation = 0;
        };
        static std::unordered_map<uint64_t, ReconstructionRotationState> _reconstructionRotations;

        [[nodiscard]] uint64_t
            CurrentReconstructionSelectionStamp(
                const std::vector<uint64_t>& groups)
        {
            uint64_t stamp = 14695981039346656037ull;
            for (const auto groupKey : groups)
            {
                ExtendStableKey(stamp, groupKey);
                const auto found =
                    _reconstructionRotations.find(groupKey);
                const uint64_t selected =
                    found != _reconstructionRotations.end()
                        && found->second.hasSelectedRotation
                    ? found->second.selectedRotation
                    : 0xffu;
                ExtendStableKey(stamp, selected);
            }
            return stamp;
        }

        struct EntityRotationState
        {
            uint64_t lastSeen{};
            bool hasSelectedRotation = false;
            uint8_t selectedRotation = 0;
        };
        static std::unordered_map<uint16_t, EntityRotationState> _entityRotations;

        struct DynamicEntityRegion
        {
            bool hasBounds = false;
            FirstPersonVec3 low{};
            FirstPersonVec3 high{};
            FirstPersonVec3 center{};
            float radius = 0.0f;
            std::vector<EntityId> entities;
        };
        struct DynamicEntitySpatialCache
        {
            bool valid = false;
            uint32_t generation = 0;
            std::vector<DynamicEntityRegion> regions;
        };
        static DynamicEntitySpatialCache _dynamicEntitySpatialCache;

        [[nodiscard]] FirstPersonSemanticSphere EntityVisualBounds(const EntityBase& entity)
        {
            const auto worldLoc = entity.getLocation();
            float halfWidth = std::max(
                1.0f, float(entity.spriteData.width));
            float verticalExtent = std::max(
                float(entity.spriteData.heightMin),
                float(entity.spriteData.heightMax));
            int32_t visualZOffset = 0;
            if (const auto* vehicle =
                    entity.as<Vehicle>();
                vehicle != nullptr)
            {
                const auto visual =
                    ResolveVehicleVisualState(*vehicle);
                visualZOffset = visual.zOffset;
                if (visual.carEntry != nullptr)
                {
                    halfWidth = std::max(
                        halfWidth,
                        float(visual.carEntry->spriteWidth));
                    verticalExtent = std::max(
                        verticalExtent,
                        float(std::max(
                            visual.carEntry->spriteHeightNegative,
                            visual.carEntry->spriteHeightPositive)));
                }
            }
            return {
                {
                    float(worldLoc.x), float(worldLoc.y),
                    float(worldLoc.z + visualZOffset)
                },
                std::max(
                    32.0f,
                    std::hypot(
                        halfWidth, verticalExtent)
                        + 16.0f)
            };
        }

        void RebuildDynamicEntitySpatialCache(uint32_t generation)
        {
            if (_dynamicEntitySpatialCache.valid
                && _dynamicEntitySpatialCache.generation == generation)
                return;

            _dynamicEntitySpatialCache.valid = true;
            _dynamicEntitySpatialCache.generation = generation;
            _dynamicEntitySpatialCache.regions.clear();
            std::unordered_map<uint64_t, size_t> regionSlots;
            constexpr int32_t kDynamicRegionTiles = 16;
            constexpr int32_t kDynamicRegionWorld = kDynamicRegionTiles * kCoordsXYStep;
            const auto floorDiv = [](int32_t value, int32_t divisor) {
                int32_t quotient = value / divisor;
                if (value < 0 && value % divisor != 0)
                    --quotient;
                return quotient;
            };

            for (uint8_t rawType = 0; rawType < static_cast<uint8_t>(EntityType::count); ++rawType)
            {
                const auto type = static_cast<EntityType>(rawType);
                for (const auto entityId : getGameState().entities.getEntityList(type))
                {
                    const auto* entity = getGameState().entities.tryGetEntity<EntityBase>(entityId);
                    if (entity == nullptr)
                        continue;
                    const auto location = entity->getLocation();
                    if (location.x == kLocationNull)
                        continue;

                    const int32_t rx = floorDiv(location.x, kDynamicRegionWorld);
                    const int32_t ry = floorDiv(location.y, kDynamicRegionWorld);
                    const uint64_t key =
                        (uint64_t(uint32_t(rx)) << 32) | uint32_t(ry);
                    auto [slotIt, inserted] = regionSlots.emplace(
                        key, _dynamicEntitySpatialCache.regions.size());
                    if (inserted)
                        _dynamicEntitySpatialCache.regions.emplace_back();
                    auto& region = _dynamicEntitySpatialCache.regions[slotIt->second];
                    region.entities.push_back(entityId);

                    const auto bounds = EntityVisualBounds(*entity);
                    const FirstPersonVec3 low{
                        bounds.center.x - bounds.radius,
                        bounds.center.y - bounds.radius,
                        bounds.center.z - bounds.radius
                    };
                    const FirstPersonVec3 high{
                        bounds.center.x + bounds.radius,
                        bounds.center.y + bounds.radius,
                        bounds.center.z + bounds.radius
                    };
                    if (!region.hasBounds)
                    {
                        region.low = low;
                        region.high = high;
                        region.hasBounds = true;
                    }
                    else
                    {
                        region.low.x = std::min(region.low.x, low.x);
                        region.low.y = std::min(region.low.y, low.y);
                        region.low.z = std::min(region.low.z, low.z);
                        region.high.x = std::max(region.high.x, high.x);
                        region.high.y = std::max(region.high.y, high.y);
                        region.high.z = std::max(region.high.z, high.z);
                    }
                }
            }

            for (auto& region : _dynamicEntitySpatialCache.regions)
            {
                region.center = {
                    0.5f * (region.low.x + region.high.x),
                    0.5f * (region.low.y + region.high.y),
                    0.5f * (region.low.z + region.high.z)
                };
                const float dx = 0.5f * (region.high.x - region.low.x);
                const float dy = 0.5f * (region.high.y - region.low.y);
                const float dz = 0.5f * (region.high.z - region.low.z);
                // Cover render-time tweening between adjacent simulation poses.
                region.radius = std::sqrt(dx * dx + dy * dy + dz * dz) + 256.0f;
            }
        }

        struct StaticRegionPacketCache
        {
            uint64_t generation{};
            uint64_t lastSeen{};
            bool dirty = true;
            FirstPersonVec3 center{};
            float radius{};
            std::vector<FirstPersonSurface> surfaces;
            std::vector<ImageIndex> textureDependencies;
            std::vector<uint64_t> reconstructionGroups;
            uint64_t reconstructionSelectionStamp{};
        };
        static std::unordered_map<uint64_t, StaticRegionPacketCache> _staticRegionPackets;

        struct TrackTrajectoryCacheEntry
        {
            struct MaskedArtwork
            {
                ImageIndex image = kImageIndexUndefined;
                int32_t left = 0;
                int32_t top = 0;
                int16_t width = 0;
                int16_t height = 0;
                uint8_t rotation = 0;
                uint8_t channelMask = 0;
                uint8_t imageChannelFlags = 0;
                bool changed = false;
                uint64_t fingerprint = 0;
                std::vector<uint8_t> pixels;
            };

            uint64_t signature{};
            uint64_t lastSeen{};
            bool dirty = false;
            bool boundaryContinuous = true;
            bool hasBounds = false;
            int32_t minTileX{}, minTileY{}, maxTileX{}, maxTileY{};
            uint8_t sourceChannelMask = 0;
            std::array<FirstPersonSilhouette, 4> railSilhouettes{};
            std::unordered_map<uint64_t, MaskedArtwork> maskedArtwork;
            std::unordered_map<uint64_t, std::vector<size_t>>
                regionSurfaceIndices;
            std::vector<FirstPersonSurface> surfaces;
        };
        static std::unordered_map<uint64_t, TrackTrajectoryCacheEntry>
            _trackTrajectoryCache;
        static std::unordered_map<uint64_t, std::unordered_set<uint64_t>>
            _trackTrajectoryGroupsByRegion;

        uint64_t TerrainKey(int32_t tx, int32_t ty);

        struct LargeSceneryGeometryCacheEntry
        {
            uint64_t signature{};
            uint64_t lastSeen{};
            bool dirty = false;
            uint32_t bodyImageFirst{};
            uint32_t bodyImageLast{};
            bool hasBounds = false;
            int32_t minTileX{}, minTileY{}, maxTileX{}, maxTileY{};
            std::vector<FirstPersonSurface> surfaces;
            std::vector<FirstPersonPhysicalBoxProxy> collisionProxies;
        };
        static std::unordered_map<uint64_t, LargeSceneryGeometryCacheEntry> _largeSceneryGeometryCache;
        static std::unordered_map<uint64_t, std::unordered_set<uint64_t>>
            _largeSceneryGroupsByRegion;
        static std::unordered_set<uint64_t> _activeLargeSceneryRegions;
        static bool _largeSceneryGeometryEnabled = false;

        [[nodiscard]] bool LargeSceneryGeometryAllowedForView(uint32_t viewFlags)
        {
            constexpr uint32_t kIncompatible =
                VIEWPORT_FLAG_CLIP_VIEW
                | VIEWPORT_FLAG_CLIP_VIEW_SEE_THROUGH
                | VIEWPORT_FLAG_HIGHLIGHT_PATH_ISSUES
                | VIEWPORT_FLAG_HIDE_SCENERY
                | VIEWPORT_FLAG_INVISIBLE_SCENERY;
            if ((viewFlags & kIncompatible) != 0 || gTrackDesignSaveMode)
                return false;

            // Native painting can temporarily recolour a selected large-scenery
            // element. Until reconstruction carries that presentation state as
            // part of its own immutable texture key, leave selection rendering
            // entirely on the authoritative native path.
            const auto* selected = TileInspector::GetSelectedElement();
            return selected == nullptr
                || selected->getType() != TileElementType::largeScenery;
        }

        void MarkLargeSceneryGeometryRegionsDirty(const LargeSceneryGeometryCacheEntry& cached)
        {
            for (const auto& surface : cached.surfaces)
            {
                if (surface.gpuRegion != 0)
                    _staticRegionPackets[surface.gpuRegion].dirty = true;
            }
        }

        void UnregisterLargeSceneryRegionMembership(
            uint64_t groupKey, const LargeSceneryGeometryCacheEntry& cached)
        {
            WithdrawFirstPersonLargeSceneryPhysicalProxies(groupKey);
            std::unordered_set<uint64_t> regions;
            for (const auto& surface : cached.surfaces)
            {
                if (surface.gpuRegion != 0)
                    regions.insert(surface.gpuRegion);
            }
            for (const auto region : regions)
            {
                const auto found = _largeSceneryGroupsByRegion.find(region);
                if (found == _largeSceneryGroupsByRegion.end())
                    continue;
                found->second.erase(groupKey);
                if (found->second.empty())
                    _largeSceneryGroupsByRegion.erase(found);
            }
        }

        void RegisterLargeSceneryRegionMembership(
            uint64_t groupKey, const LargeSceneryGeometryCacheEntry& cached)
        {
            PublishFirstPersonLargeSceneryPhysicalProxies(
                groupKey, cached.collisionProxies);
            for (const auto& surface : cached.surfaces)
            {
                if (surface.gpuRegion != 0)
                    _largeSceneryGroupsByRegion[surface.gpuRegion].insert(groupKey);
            }
        }

        void ActivateLargeSceneryRegions(
            const LargeSceneryGeometryCacheEntry& cached)
        {
            for (const auto& surface : cached.surfaces)
            {
                if (surface.gpuRegion != 0)
                    _activeLargeSceneryRegions.insert(surface.gpuRegion);
            }
        }

        [[nodiscard]] bool LargeSceneryInstanceMetadataMatches(
            const LargeSceneryElement& large, const LargeSceneryEntry& entry)
        {
            const size_t sequence = large.getSequenceIndex();
            if (sequence >= entry.tiles.size())
                return false;
            const uint8_t expected =
                RotateQuarterMask(entry.tiles[sequence].corners, large.getDirection());
            const uint8_t actual = large.getOccupiedQuadrants() & 0x0F;
            return actual == 0 || actual == expected;
        }

        [[nodiscard]] bool LargeSceneryInstanceComplete(
            const LargeSceneryEntry& entry, const ReconstructionGroupInfo& group,
            uint8_t objectDirection)
        {
            const int32_t originX = int32_t(std::lround(group.anchor.x));
            const int32_t originY = int32_t(std::lround(group.anchor.y));
            const int32_t originZ = int32_t(std::lround(group.anchor.z));
            for (size_t sequence = 0; sequence < entry.tiles.size(); ++sequence)
            {
                const auto& tile = entry.tiles[sequence];
                const auto offset =
                    CoordsXY{ tile.offset.x, tile.offset.y }.rotate(objectDirection);
                const CoordsXY expectedTile{ originX + offset.x, originY + offset.y };
                const int32_t expectedBaseZ = originZ + tile.offset.z;

                auto* element = MapGetFirstElementAt(expectedTile);
                if (element == nullptr)
                    return false;

                bool found = false;
                do
                {
                    if (element->getType() != TileElementType::largeScenery)
                        continue;
                    const auto* candidate = element->asLargeScenery();
                    if (candidate == nullptr
                        || candidate->getEntry() != &entry
                        || candidate->getSequenceIndex() != sequence
                        || static_cast<uint8_t>(candidate->getDirection()) != objectDirection
                        || candidate->getBaseZ() != expectedBaseZ
                        || !LargeSceneryInstanceMetadataMatches(*candidate, entry))
                        continue;
                    found = true;
                    break;
                } while (!(element++)->isLastForTile());

                if (!found)
                    return false;
            }
            return true;
        }

        [[nodiscard]] ImageId LargeSceneryImageTemplate(
            const LargeSceneryElement& large, const LargeSceneryEntry& entry)
        {
            ImageId result{};
            if (entry.flags.has(LargeSceneryFlag::hasPrimaryColour))
                result = result.WithPrimary(large.getPrimaryColour());
            if (entry.flags.has(LargeSceneryFlag::hasSecondaryColour))
                result = result.WithSecondary(large.getSecondaryColour());
            if (entry.flags.has(LargeSceneryFlag::hasTertiaryColour))
                result = result.WithTertiary(large.getTertiaryColour());
            return result;
        }

        [[nodiscard]] std::vector<FirstPersonPhysicalBoxProxy>
            BuildFirstPersonLargeSceneryCollisionProxies(
                const LargeSceneryEntry& entry,
                const LargeSceneryAssetModel& model,
                const ReconstructionGroupInfo& group,
                uint8_t objectDirection)
        {
            std::vector<FirstPersonPhysicalBoxProxy> result;
            constexpr float kCollisionSkin = 0.5f;
            constexpr float kCellSize = 2.0f;
            constexpr size_t kMaxCollisionCells = 2048;

            for (const auto& face : model.faces)
            {
                if (face.sequence >= entry.tiles.size())
                    continue;
                const auto& tile = entry.tiles[face.sequence];
                const uint8_t sourceRotation =
                    FirstPersonViewportRotationForNativeView(
                        objectDirection, face.sourceDirection);
                const ImageIndex imageIndex =
                    entry.image + 4
                    + (ImageIndex(face.sequence) << 2)
                    + face.sourceDirection;
                const auto* g1 = GfxGetG1Element(imageIndex);
                if (g1 == nullptr || g1->width <= 0
                    || g1->height <= 0)
                    return {};

                const CoordsXY tileOffset =
                    CoordsXY{
                        tile.offset.x, tile.offset.y
                    }.rotate(objectDirection);
                const CoordsXY tileWorld{
                    int32_t(std::lround(group.anchor.x))
                        + tileOffset.x,
                    int32_t(std::lround(group.anchor.y))
                        + tileOffset.y,
                };
                const int32_t tileBaseZ =
                    int32_t(std::lround(group.anchor.z))
                    + tile.offset.z;
                const auto spriteOrigin =
                    GetTileElementPaintSpritePosition(
                        tileWorld, sourceRotation);
                const auto spritePos =
                    Translate3DTo2DWithZ(
                        sourceRotation,
                        { spriteOrigin, tileBaseZ });

                bool havePoint = false;
                FirstPersonVec3 low{}, high{};
                for (const auto& corner : face.corners)
                {
                    const auto localXY =
                        FirstPersonLargeSceneryPlacedPoint(
                            { tile.offset.x, tile.offset.y },
                            { corner.x, corner.y },
                            objectDirection);
                    const FirstPersonVec3 world{
                        group.anchor.x
                            + float(localXY.x),
                        group.anchor.y
                            + float(localXY.y),
                        group.anchor.z
                            + float(corner.z),
                    };
                    if (!havePoint)
                    {
                        low = high = world;
                        havePoint = true;
                    }
                    else
                    {
                        low.x = std::min(low.x, world.x);
                        low.y = std::min(low.y, world.y);
                        low.z = std::min(low.z, world.z);
                        high.x = std::max(high.x, world.x);
                        high.y = std::max(high.y, world.y);
                        high.z = std::max(high.z, world.z);
                    }
                }
                if (!havePoint)
                    continue;

                const int normalAxis =
                    high.x - low.x < 0.01f ? 0
                    : (high.y - low.y < 0.01f ? 1 : 2);
                const std::array<float, 3> lo{
                    low.x, low.y, low.z
                };
                const std::array<float, 3> hi{
                    high.x, high.y, high.z
                };
                const int axisA =
                    normalAxis == 0 ? 1 : 0;
                const int axisB =
                    normalAxis == 2 ? 1 : 2;

                const auto worldPoint =
                    [](const std::array<float, 3>& p) {
                        return FirstPersonVec3{
                            p[0], p[1], p[2]
                        };
                    };
                const auto opaqueAt =
                    [&](const std::array<float, 3>& point) {
                        const auto p = worldPoint(point);
                        const auto source =
                            Translate3DTo2DWithZ(
                                sourceRotation,
                                {
                                    int32_t(std::lround(p.x)),
                                    int32_t(std::lround(p.y)),
                                    int32_t(std::lround(p.z)),
                                });
                        return FirstPersonG1PixelOpaque(
                            *g1,
                            source.x - spritePos.x
                                - g1->xOffset,
                            source.y - spritePos.y
                                - g1->yOffset);
                    };

                for (float a = lo[axisA];
                     a < hi[axisA] - 0.01f;
                     a += kCellSize)
                for (float b = lo[axisB];
                     b < hi[axisB] - 0.01f;
                     b += kCellSize)
                {
                    const float a1 =
                        std::min(a + kCellSize, hi[axisA]);
                    const float b1 =
                        std::min(b + kCellSize, hi[axisB]);
                    std::array<float, 3> centre{
                        0.5f * (lo[0] + hi[0]),
                        0.5f * (lo[1] + hi[1]),
                        0.5f * (lo[2] + hi[2]),
                    };
                    centre[axisA] = 0.5f * (a + a1);
                    centre[axisB] = 0.5f * (b + b1);

                    // Sample the centre plus four quarter-cell points. A cell
                    // becomes physical only when the native face texture
                    // contains opacity there; transparent arch/door pixels do
                    // not acquire collision merely because their enclosing face
                    // was geometrically validated.
                    size_t opaqueSamples = 0;
                    for (const float u :
                        { 0.25f, 0.75f })
                    for (const float v :
                        { 0.25f, 0.75f })
                    {
                        auto sample = centre;
                        sample[axisA] =
                            a + (a1 - a) * u;
                        sample[axisB] =
                            b + (b1 - b) * v;
                        if (opaqueAt(sample))
                            ++opaqueSamples;
                    }
                    if (opaqueAt(centre))
                        ++opaqueSamples;
                    if (opaqueSamples == 0)
                        continue;

                    std::array<float, 3> cellLow = centre;
                    std::array<float, 3> cellHigh = centre;
                    cellLow[axisA] = a;
                    cellHigh[axisA] = a1;
                    cellLow[axisB] = b;
                    cellHigh[axisB] = b1;
                    cellLow[normalAxis] =
                        lo[normalAxis] - kCollisionSkin;
                    cellHigh[normalAxis] =
                        hi[normalAxis] + kCollisionSkin;

                    result.push_back({
                        worldPoint(cellLow),
                        worldPoint(cellHigh),
                        FirstPersonPhysicalProxyProvenance::
                            calibratedLargeSceneryArtwork,
                        static_cast<uint8_t>(
                            FirstPersonPhysicalProxyCapability::
                                collide),
                        group.key,
                    });
                    if (result.size()
                        > kMaxCollisionCells)
                    {
                        // Complexity is not evidence. Fail open rather than
                        // replacing a detailed/hollow asset with a coarse box.
                        return {};
                    }
                }
            }
            return result;
        }

        LargeSceneryGeometryCacheEntry BuildLargeSceneryInstanceGeometry(
            const LargeSceneryElement& large, const LargeSceneryEntry& entry,
            const LargeSceneryAssetModel& model, const ReconstructionGroupInfo& group,
            uint64_t signature, uint64_t frame)
        {
            LargeSceneryGeometryCacheEntry result{};
            result.signature = signature;
            result.lastSeen = frame;
            result.bodyImageFirst = model.bodyImageFirst;
            result.bodyImageLast = model.bodyImageLast;
            const uint8_t objectDirection = static_cast<uint8_t>(large.getDirection()) & 3;
            const auto imageTemplate = LargeSceneryImageTemplate(large, entry);
            result.collisionProxies =
                BuildFirstPersonLargeSceneryCollisionProxies(
                    entry, model, group, objectDirection);

            bool haveBounds = false;
            FirstPersonVec3 low{}, high{};
            for (const auto& face : model.faces)
            {
                if (face.sequence >= entry.tiles.size())
                    continue;
                const auto& tile = entry.tiles[face.sequence];
                const uint8_t sourceRotation =
                    FirstPersonViewportRotationForNativeView(
                        objectDirection, face.sourceDirection);
                const ImageIndex imageIndex =
                    entry.image + 4 + (ImageIndex(face.sequence) << 2) + face.sourceDirection;
                const auto image = imageTemplate.WithIndex(imageIndex);
                const auto* g1 = GfxGetG1Element(image);
                if (g1 == nullptr || g1->width <= 0 || g1->height <= 0)
                    continue;

                const CoordsXY tileOffset =
                    CoordsXY{ tile.offset.x, tile.offset.y }.rotate(objectDirection);
                const CoordsXY tileWorld{
                    int32_t(std::lround(group.anchor.x)) + tileOffset.x,
                    int32_t(std::lround(group.anchor.y)) + tileOffset.y,
                };
                const int32_t tileBaseZ =
                    int32_t(std::lround(group.anchor.z)) + tile.offset.z;
                const auto spriteOrigin =
                    GetTileElementPaintSpritePosition(tileWorld, sourceRotation);
                const auto spritePos = Translate3DTo2DWithZ(
                    sourceRotation, { spriteOrigin, tileBaseZ });

                FirstPersonSurface surface{};
                surface.image = image;
                surface.reconstructionGroup = group.key;
                std::array<FirstPersonVertex, 4> vertices{};
                FirstPersonVec3 faceCenter{};
                for (size_t i = 0; i < face.corners.size(); ++i)
                {
                    const auto localXY =
                        FirstPersonLargeSceneryPlacedPoint(
                            { tile.offset.x, tile.offset.y },
                            { face.corners[i].x, face.corners[i].y },
                            objectDirection);
                    const FirstPersonVec3 world{
                        group.anchor.x + float(localXY.x),
                        group.anchor.y + float(localXY.y),
                        group.anchor.z + float(face.corners[i].z),
                    };
                    const auto source = Translate3DTo2DWithZ(
                        sourceRotation,
                        {
                            int32_t(std::lround(world.x)),
                            int32_t(std::lround(world.y)),
                            int32_t(std::lround(world.z)),
                        });
                    vertices[i] = {
                        world,
                        float(source.x - spritePos.x - g1->xOffset),
                        float(source.y - spritePos.y - g1->yOffset),
                    };
                    faceCenter = Add(faceCenter, world);

                    if (!haveBounds)
                    {
                        low = high = world;
                        haveBounds = true;
                    }
                    else
                    {
                        low.x = std::min(low.x, world.x);
                        low.y = std::min(low.y, world.y);
                        low.z = std::min(low.z, world.z);
                        high.x = std::max(high.x, world.x);
                        high.y = std::max(high.y, world.y);
                        high.z = std::max(high.z, world.z);
                    }
                }
                faceCenter = Mul(faceCenter, 0.25f);
                surface.gpuRegion = FirstPersonGpuRegionKey(
                    int32_t(std::floor(faceCenter.x / float(kCoordsXYStep))),
                    int32_t(std::floor(faceCenter.y / float(kCoordsXYStep))));
                EmitQuad(surface, vertices);
                result.surfaces.emplace_back(std::move(surface));

            }

            if (haveBounds && !result.surfaces.empty())
            {
                result.hasBounds = true;
                result.minTileX = int32_t(std::floor(low.x / float(kCoordsXYStep)));
                result.minTileY = int32_t(std::floor(low.y / float(kCoordsXYStep)));
                result.maxTileX = int32_t(std::floor(high.x / float(kCoordsXYStep)));
                result.maxTileY = int32_t(std::floor(high.y / float(kCoordsXYStep)));
                const FirstPersonVec3 center{
                    0.5f * (low.x + high.x),
                    0.5f * (low.y + high.y),
                    0.5f * (low.z + high.z),
                };
                const float hx = 0.5f * (high.x - low.x);
                const float hy = 0.5f * (high.y - low.y);
                const float hz = 0.5f * (high.z - low.z);
                const float radius = std::sqrt(hx * hx + hy * hy + hz * hz) + 2.0f;
                for (auto& surface : result.surfaces)
                {
                    surface.hasSemanticBounds = true;
                    surface.semanticCenter = center;
                    surface.semanticRadius = radius;
                }
            }
            return result;
        }

        void UpdateLargeSceneryReconstructions(FirstPersonScene& scene, uint64_t frame)
        {
            _activeLargeSceneryRegions.clear();
            const bool enabled =
                LargeSceneryGeometryAllowedForView(scene.options.viewFlags);
            if (_largeSceneryGeometryEnabled != enabled)
            {
                for (const auto& [groupKey, cached] : _largeSceneryGeometryCache)
                {
                    (void)groupKey;
                    MarkLargeSceneryGeometryRegionsDirty(cached);
                }
                _largeSceneryGeometryEnabled = enabled;
                if (!enabled)
                    ClearFirstPersonLargeSceneryPhysicalProxies();
            }
            if (!enabled)
                return;

            // Discovery reuses the reconstruction groups already collected by
            // the static-tile semantic cache. Cold fitting is deterministic:
            // at most one previously unseen asset is attempted per frame, and
            // every success OR fallback decision is cached. No asset restarts
            // identical work forever after missing a wall-clock deadline.
            size_t assetFitBudget = 1;
            std::unordered_set<uint64_t> seenGroups;
            seenGroups.reserve(scene.visibleTiles.size() / 2 + 1);

            for (const auto tile : scene.visibleTiles)
            {
                const auto tileKey = TerrainKey(
                    tile.x / kCoordsXYStep, tile.y / kCoordsXYStep);
                const auto paintCache = _staticPaintCache.find(tileKey);
                if (paintCache == _staticPaintCache.end()
                    || !paintCache->second.valid || paintCache->second.dirty)
                    continue;

                for (const auto& group : paintCache->second.reconstructionGroups)
                {
                    if (group.type != TileElementType::largeScenery
                        || !seenGroups.insert(group.key).second)
                        continue;

                    auto cached = _largeSceneryGeometryCache.find(group.key);
                    if (cached != _largeSceneryGeometryCache.end()
                        && !cached->second.dirty)
                    {
                        cached->second.lastSeen = frame;
                        ActivateLargeSceneryRegions(cached->second);
                        continue;
                    }

                    // Resolve live instance state only on a cold/dirty build.
                    // sourceTile is the tile whose semantic cache produced this
                    // group and is therefore invalidated with that tile.
                    LargeSceneryElement* large = nullptr;
                    auto* element = MapGetFirstElementAt(group.sourceTile);
                    if (element != nullptr)
                    {
                        do
                        {
                            if (element->getType() != TileElementType::largeScenery
                                || element->isGhost() || element->isInvisible())
                                continue;
                            const auto candidateGroup =
                                GetReconstructionGroup(group.sourceTile, element);
                            if (candidateGroup.has_value()
                                && candidateGroup->key == group.key)
                            {
                                large = element->asLargeScenery();
                                break;
                            }
                        } while (!(element++)->isLastForTile());
                    }

                    const auto* entry =
                        large != nullptr ? large->getEntry() : nullptr;
                    if (large == nullptr || entry == nullptr
                        || !LargeSceneryAssetEligible(*entry)
                        || !LargeSceneryInstanceMetadataMatches(*large, *entry))
                    {
                        if (cached != _largeSceneryGeometryCache.end())
                        {
                            MarkLargeSceneryGeometryRegionsDirty(cached->second);
                            UnregisterLargeSceneryRegionMembership(
                                group.key, cached->second);
                            _largeSceneryGeometryCache.erase(cached);
                        }
                        continue;
                    }

                    const auto existingModel =
                        _largeSceneryAssetModels.find(entry);
                    const bool alreadyAttempted =
                        existingModel != _largeSceneryAssetModels.end()
                        && existingModel->second.attempted;
                    const bool allowBuild =
                        alreadyAttempted || assetFitBudget > 0;
                    const auto* model =
                        GetLargeSceneryAssetModel(*entry, allowBuild);
                    if (!alreadyAttempted && model != nullptr)
                        --assetFitBudget;
                    if (model == nullptr || !model->reliable)
                        continue;

                    uint64_t signature = group.key;
                    ExtendStableKey(
                        signature,
                        static_cast<uint8_t>(large->getPrimaryColour()));
                    ExtendStableKey(
                        signature,
                        static_cast<uint8_t>(large->getSecondaryColour()));
                    ExtendStableKey(
                        signature,
                        static_cast<uint8_t>(large->getTertiaryColour()));
                    ExtendStableKey(signature, uint32_t(model->heightTrim));
                    ExtendStableKey(signature, model->faces.size());
                    ExtendStableKey(
                        signature,
                        uint32_t(std::lround(
                            model->minimumFaceOwnership * 1000.0f)));

                    if (cached != _largeSceneryGeometryCache.end()
                        && cached->second.signature == signature
                        && !cached->second.dirty)
                    {
                        cached->second.lastSeen = frame;
                        ActivateLargeSceneryRegions(cached->second);
                        continue;
                    }

                    if (cached != _largeSceneryGeometryCache.end())
                    {
                        MarkLargeSceneryGeometryRegionsDirty(cached->second);
                        UnregisterLargeSceneryRegionMembership(
                            group.key, cached->second);
                    }

                    const uint8_t objectDirection =
                        static_cast<uint8_t>(large->getDirection()) & 3u;
                    if (!LargeSceneryInstanceComplete(
                            *entry, group, objectDirection))
                    {
                        if (cached != _largeSceneryGeometryCache.end())
                            _largeSceneryGeometryCache.erase(cached);
                        continue;
                    }

                    auto rebuilt = BuildLargeSceneryInstanceGeometry(
                        *large, *entry, *model, group, signature, frame);
                    if (rebuilt.surfaces.empty())
                    {
                        if (cached != _largeSceneryGeometryCache.end())
                            _largeSceneryGeometryCache.erase(cached);
                        continue;
                    }

                    MarkLargeSceneryGeometryRegionsDirty(rebuilt);
                    RegisterLargeSceneryRegionMembership(group.key, rebuilt);
                    ActivateLargeSceneryRegions(rebuilt);
                    _largeSceneryGeometryCache[group.key] = std::move(rebuilt);
                }
            }

            // Expiry is maintenance cadence, not presentation cadence. Region
            // membership lets packet rebuilds avoid a full geometry-cache scan.
            if (frame % 120 == 0)
            {
                for (auto it = _largeSceneryGeometryCache.begin();
                     it != _largeSceneryGeometryCache.end();)
                {
                    if (frame - it->second.lastSeen <= 240)
                    {
                        ++it;
                        continue;
                    }
                    MarkLargeSceneryGeometryRegionsDirty(it->second);
                    UnregisterLargeSceneryRegionMembership(
                        it->first, it->second);
                    it = _largeSceneryGeometryCache.erase(it);
                }
            }
        }

        [[nodiscard]] bool IsReconstructedLargeSceneryBody(
            const FirstPersonSurface& surface, uint64_t frame)
        {
            if (!_largeSceneryGeometryEnabled || surface.reconstructionGroup == 0
                || !surface.image.HasValue())
                return false;
            const auto found = _largeSceneryGeometryCache.find(surface.reconstructionGroup);
            if (found == _largeSceneryGeometryCache.end() || found->second.dirty
                || found->second.lastSeen != frame)
                return false;
            const auto image = surface.image.GetIndex();
            return image >= found->second.bodyImageFirst
                && image < found->second.bodyImageLast;
        }

        [[nodiscard]] bool IsResidentStaticSurface(const FirstPersonSurface& surface)
        {
            if (surface.gpuRegion == 0 || surface.viewFacing)
                return false;
            if (surface.solidColour != 0)
                return true;
            return surface.image.HasValue() && !surface.image.IsBlended()
                && surface.immutablePixels.empty();
        }

        [[nodiscard]] uint64_t ResidentStaticSurfaceFingerprint(
            const std::vector<FirstPersonSurface>& surfaces)
        {
            uint64_t fingerprint = 14695981039346656037ull;
            const auto extendFloat =
                [&](float value) {
                    ExtendStableKey(
                        fingerprint,
                        uint64_t(int64_t(std::llround(
                            double(value) * 1024.0))));
                };
            const auto extendImage =
                [&](ImageId image) {
                    ExtendStableKey(
                        fingerprint, image.GetIndex());
                    ExtendStableKey(
                        fingerprint, image.GetRemap());
                    ExtendStableKey(
                        fingerprint,
                        EnumValue(image.GetPrimary()));
                    ExtendStableKey(
                        fingerprint,
                        EnumValue(image.GetSecondary()));
                    ExtendStableKey(
                        fingerprint,
                        EnumValue(image.GetTertiary()));
                    ExtendStableKey(
                        fingerprint,
                        image.HasPrimary() ? 1 : 0);
                    ExtendStableKey(
                        fingerprint,
                        image.HasSecondary() ? 1 : 0);
                    ExtendStableKey(
                        fingerprint,
                        image.HasTertiary() ? 1 : 0);
                    ExtendStableKey(
                        fingerprint,
                        image.IsBlended() ? 1 : 0);
                };

            ExtendStableKey(fingerprint, surfaces.size());
            for (const auto& surface : surfaces)
            {
                extendImage(surface.image);
                extendImage(surface.mask);
                ExtendStableKey(
                    fingerprint, surface.solidColour);
                ExtendStableKey(
                    fingerprint, surface.depthBias ? 1 : 0);
                ExtendStableKey(
                    fingerprint, surface.edgeCoverage ? 1 : 0);
                ExtendStableKey(
                    fingerprint, surface.reconstructionGroup);
                ExtendStableKey(
                    fingerprint, surface.nativePaintOrdinal);
                ExtendStableKey(
                    fingerprint, surface.gpuRegion);
                ExtendStableKey(
                    fingerprint,
                    surface.hasSemanticBounds ? 1 : 0);
                if (surface.hasSemanticBounds)
                {
                    extendFloat(surface.semanticCenter.x);
                    extendFloat(surface.semanticCenter.y);
                    extendFloat(surface.semanticCenter.z);
                    extendFloat(surface.semanticRadius);
                }
                for (const auto& vertex :
                     surface.triangles)
                {
                    extendFloat(vertex.world.x);
                    extendFloat(vertex.world.y);
                    extendFloat(vertex.world.z);
                    extendFloat(vertex.u);
                    extendFloat(vertex.v);
                }
            }
            return fingerprint;
        }

        void MarkStaticRegionDirtyForTile(int32_t tileX, int32_t tileY)
        {
            _staticRegionPackets[FirstPersonGpuRegionKey(tileX, tileY)].dirty = true;
        }

        void MarkTrackTrajectoryRegionsDirty(
            const TrackTrajectoryCacheEntry& cached)
        {
            for (const auto& [regionKey, indices] :
                 cached.regionSurfaceIndices)
            {
                (void)indices;
                _staticRegionPackets[regionKey].dirty = true;
            }
        }

        void UnregisterTrackTrajectoryRegionMembership(
            uint64_t groupKey,
            const TrackTrajectoryCacheEntry& cached)
        {
            for (const auto& [regionKey, indices] :
                 cached.regionSurfaceIndices)
            {
                (void)indices;
                const auto found =
                    _trackTrajectoryGroupsByRegion.find(
                        regionKey);
                if (found
                    == _trackTrajectoryGroupsByRegion.end())
                    continue;
                found->second.erase(groupKey);
                if (found->second.empty())
                {
                    _trackTrajectoryGroupsByRegion.erase(
                        found);
                }
            }
        }

        void RegisterTrackTrajectoryRegionMembership(
            uint64_t groupKey,
            const TrackTrajectoryCacheEntry& cached)
        {
            for (const auto& [regionKey, indices] :
                 cached.regionSurfaceIndices)
            {
                (void)indices;
                _trackTrajectoryGroupsByRegion[
                    regionKey].insert(groupKey);
            }
        }

        struct FirstPersonTrackProfileCalibrationState
        {
            enum class Phase : uint8_t
            {
                captureSource,
                fitSource,
                validateHoldouts,
                verifySource,
                complete,
            };

            Phase phase = Phase::captureSource;
            bool rejected = false;
            FirstPersonTrackRailProfile profile{};
            FirstPersonTrackArtworkObservation sourceArtwork{};
            FirstPersonTrackTrajectory sourceTrajectory{};
            FirstPersonVec3 sourceAnchor{};
            uint64_t sourceFingerprint = 0;
            FirstPersonTrackProfileSearchState search{};
            uint8_t requiredKinds = 0;
            uint8_t passedKinds = 0;
            uint8_t passedHoldouts = 0;
            uint8_t holdoutCategory = 0;
            uint8_t holdoutCandidate = 0;
            bool holdoutCategorySupported = false;
        };
        static std::unordered_map<uint8_t,
            FirstPersonTrackProfileCalibrationState>
            _trackProfileCalibrations;

        [[nodiscard]] uint64_t FirstPersonTrackProfileSignature(
            const Ride& ride, const TrackElement& track)
        {
            const auto profile =
                FirstPersonVerifiedTrackRailProfile(ride, track);
            if (!profile.has_value())
                return 0;
            uint64_t result = 14695981039346656037ull;
            ExtendStableKey(result, 1);
            ExtendStableKey(
                result, uint32_t(std::lround(profile->halfGauge * 100.0f)));
            ExtendStableKey(
                result, uint32_t(std::lround(profile->halfWidth * 100.0f)));
            ExtendStableKey(
                result, uint32_t(std::lround(profile->halfHeight * 100.0f)));
            ExtendStableKey(
                result, uint32_t(int32_t(std::lround(
                    profile->verticalOffset * 100.0f))));
            ExtendStableKey(result, profile->sourceChannelMask);
            ExtendStableKey(result, profile->materialVerified ? 1 : 0);
            ExtendStableKey(result, profile->topMaterialValue);
            ExtendStableKey(result, profile->sideMaterialValue);
            return result;
        }

        [[nodiscard]] TileElement* FindFirstPersonTrackOriginElement(
            const CoordsXYZ& sampleOrigin, const TrackElement& source)
        {
            auto* element = MapGetFirstElementAt(sampleOrigin);
            if (element == nullptr)
                return nullptr;
            do
            {
                if (element->getType() != TileElementType::track
                    || element->getBaseZ() != sampleOrigin.z)
                    continue;
                auto* track = element->asTrack();
                if (track != nullptr
                    && track->getRideIndex() == source.getRideIndex()
                    && track->getTrackType() == source.getTrackType()
                    && track->getDirection() == source.getDirection()
                    && track->getSequenceIndex() == 0)
                    return element;
            } while (!(element++)->isLastForTile());
            return nullptr;
        }

        struct FirstPersonTrackCalibrationInstance
        {
            TrackElement* track = nullptr;
            TileElement* element = nullptr;
            const Ride* ride = nullptr;
            TrackStyle style = TrackStyle::null;
            CoordsXYZ sampleOrigin{};
            FirstPersonVec3 groupAnchor{};
        };

        [[nodiscard]] std::optional<FirstPersonTrackCalibrationInstance>
            ResolveFirstPersonTrackCalibrationInstance(
                const ReconstructionGroupInfo& group)
        {
            if (group.type != TileElementType::track)
                return std::nullopt;

            TileElement* sourceElement = nullptr;
            TrackElement* sourceTrack = nullptr;
            auto* element = MapGetFirstElementAt(group.sourceTile);
            if (element != nullptr)
            {
                do
                {
                    if (element->getType() != TileElementType::track
                        || element->isGhost() || element->isInvisible())
                        continue;
                    const auto candidate =
                        GetReconstructionGroup(group.sourceTile, element);
                    if (!candidate.has_value() || candidate->key != group.key)
                        continue;
                    sourceElement = element;
                    sourceTrack = element->asTrack();
                    break;
                } while (!(element++)->isLastForTile());
            }
            if (sourceElement == nullptr || sourceTrack == nullptr)
                return std::nullopt;

            const auto* ride = GetRide(sourceTrack->getRideIndex());
            if (ride == nullptr
                || !RideUsesStandardFirstPersonTrajectory(*ride))
                return std::nullopt;
            const auto style = FirstPersonTrackStyleFor(*ride, *sourceTrack);
            const auto sampleOrigin =
                FirstPersonTrackSampleOrigin(group.sourceTile, sourceElement);
            if (!style.has_value() || !sampleOrigin.has_value())
                return std::nullopt;

            return FirstPersonTrackCalibrationInstance{
                sourceTrack,
                sourceElement,
                ride,
                *style,
                *sampleOrigin,
                group.anchor,
            };
        }

        [[nodiscard]] bool FirstPersonTrackChannelEnabledForImage(
            const ImageId& image, FirstPersonTrackPixelChannel channel)
        {
            switch (channel)
            {
                case FirstPersonTrackPixelChannel::trackRailPalette:
                    return true;
                case FirstPersonTrackPixelChannel::primaryRemap:
                    return image.HasPrimary();
                case FirstPersonTrackPixelChannel::secondaryRemap:
                    return image.HasSecondary();
                case FirstPersonTrackPixelChannel::tertiaryRemap:
                    return image.HasTertiary();
                default:
                    return false;
            }
        }

        [[nodiscard]] std::optional<std::vector<uint8_t>>
            DecodeFirstPersonTrackSprite(const G1Element& g1)
        {
            if (g1.offset == nullptr || g1.width <= 0 || g1.height <= 0
                || g1.width > 512 || g1.height > 512
                || g1.flags.has(G1Flag::isPalette))
                return std::nullopt;

            const size_t pixelCount =
                size_t(g1.width) * size_t(g1.height);
            if (pixelCount == 0 || pixelCount > 262144)
                return std::nullopt;
            std::vector<uint8_t> pixels(pixelCount, 0);

            if (g1.flags.has(G1Flag::hasRLECompression))
            {
                for (int32_t y = 0; y < g1.height; ++y)
                {
                    const uint16_t lineOffset =
                        uint16_t(g1.offset[y * 2])
                        | (uint16_t(g1.offset[y * 2 + 1]) << 8);
                    const uint8_t* run = g1.offset + lineOffset;
                    bool endOfLine = false;
                    size_t guard = 0;
                    while (!endOfLine && guard++ < 256)
                    {
                        uint8_t length = *run++;
                        const int32_t x = *run++;
                        endOfLine = (length & 0x80u) != 0;
                        length &= 0x7Fu;
                        if (x < 0 || x + int32_t(length) > g1.width)
                            return std::nullopt;
                        for (uint8_t n = 0; n < length; ++n)
                        {
                            pixels[size_t(y) * size_t(g1.width)
                                + size_t(x + n)] = run[n];
                        }
                        run += length;
                    }
                    if (!endOfLine)
                        return std::nullopt;
                }
            }
            else
            {
                std::copy_n(g1.offset, pixelCount, pixels.begin());
                if (!g1.flags.has(G1Flag::hasTransparency))
                    return pixels;
            }
            return pixels;
        }

        void AddFirstPersonSyntheticTrackImageObservation(
            FirstPersonTrackArtworkObservation& observation,
            uint8_t rotation, ImageId image,
            const ScreenCoordsXY& screenPos,
            const ScreenCoordsXY& anchorScreen,
            uint64_t& fingerprint)
        {
            if (rotation >= 4 || !image.HasValue()
                || image.IsBlended())
                return;
            const auto* g1 = GfxGetG1Element(image);
            if (g1 == nullptr)
                return;
            const auto pixels =
                DecodeFirstPersonTrackSprite(*g1);
            if (!pixels.has_value())
                return;

            const int32_t left =
                screenPos.x + g1->xOffset
                - anchorScreen.x;
            const int32_t top =
                screenPos.y + g1->yOffset
                - anchorScreen.y;
            ExtendStableKey(
                fingerprint, image.GetIndex());
            ExtendStableKey(
                fingerprint, uint32_t(left));
            ExtendStableKey(
                fingerprint, uint32_t(top));
            ExtendStableKey(fingerprint, rotation);

            for (int32_t y = 0; y < g1->height; ++y)
            for (int32_t x = 0; x < g1->width; ++x)
            {
                const uint8_t pixel =
                    (*pixels)[size_t(y)
                        * size_t(g1->width)
                        + size_t(x)];
                if (pixel == 0)
                    continue;
                for (size_t channelIndex = 0;
                     channelIndex
                         < kFirstPersonTrackPixelChannelCount;
                     ++channelIndex)
                {
                    const auto channel =
                        static_cast<
                            FirstPersonTrackPixelChannel>(
                            channelIndex);
                    if (!FirstPersonTrackChannelEnabledForImage(
                            image, channel)
                        || !FirstPersonTrackPixelMatchesChannel(
                            pixel, channel))
                        continue;
                    observation
                        .channelViews[channelIndex][rotation]
                        .add(left + x, top + y);
                    observation
                        .channelSamples[channelIndex][rotation]
                        .push_back({
                            left + x, top + y, pixel
                        });
                }
            }
        }

        void CollectFirstPersonSyntheticTrackPaintTree(
            FirstPersonTrackArtworkObservation& observation,
            uint8_t rotation, const PaintStruct& root,
            const ScreenCoordsXY& anchorScreen,
            uint64_t& fingerprint)
        {
            AddFirstPersonSyntheticTrackImageObservation(
                observation, rotation, root.image_id,
                root.ScreenPos, anchorScreen, fingerprint);

            // Match native PaintDrawStruct exactly: a paint node with a child
            // recurses into that child; attached overlays are consumed only on
            // the leaf node. Do not invent an observation from paint records
            // that native rendering itself would not draw.
            if (root.Children != nullptr)
            {
                CollectFirstPersonSyntheticTrackPaintTree(
                    observation, rotation,
                    *root.Children, anchorScreen,
                    fingerprint);
            }
            else
            {
                for (auto* attached = root.Attached;
                     attached != nullptr;
                     attached = attached->NextEntry)
                {
                    if (attached->IsMasked)
                        continue;
                    AddFirstPersonSyntheticTrackImageObservation(
                        observation, rotation,
                        attached->image_id,
                        root.ScreenPos + attached->RelativePos,
                        anchorScreen, fingerprint);
                }
            }
        }

        struct FirstPersonSyntheticTrackObservation
        {
            bool supported = false;
            bool valid = false;
            uint64_t fingerprint =
                14695981039346656037ull;
            FirstPersonVec3 anchor{};
            FirstPersonTrackTrajectory trajectory{};
            FirstPersonTrackArtworkObservation artwork{};
        };

        [[nodiscard]] FirstPersonSyntheticTrackObservation
            PaintFirstPersonCanonicalTrackObservation(
                const FirstPersonTrackCalibrationInstance& instance,
                TrackElemType requestedType)
        {
            FirstPersonSyntheticTrackObservation result{};
            const auto type = uncoverTrackType(requestedType);
            const auto& ted =
                TrackMetadata::GetTrackElementDescriptor(type);
            if (ted.sequenceData.numSequences == 0)
                return result;

            TrackPaintFunction paintFunction =
                GetTrackPaintFunction(instance.style, type);
            if (&paintFunction == &TrackPaintFunctionDummy)
                return result;

            constexpr CoordsXYZ kAnchor{
                1024, 1024, 256
            };
            const auto& sequence0 =
                ted.sequenceData.sequences[0].clearance;
            const CoordsXYZ sampleOrigin{
                kAnchor.x + sequence0.x,
                kAnchor.y + sequence0.y,
                kAnchor.z + sequence0.z,
            };
            if (!FirstPersonTrackTrajectoryTemplateSamplesContinuous(
                    type, 0))
                return result;
            const auto trajectory =
                BuildFirstPersonTrackTrajectory(
                    type, 0,
                    {
                        float(sampleOrigin.x),
                        float(sampleOrigin.y),
                        float(sampleOrigin.z),
                    });
            if (!trajectory.has_value())
                return result;

            result.supported = true;
            result.anchor = {
                float(kAnchor.x), float(kAnchor.y),
                float(kAnchor.z)
            };
            result.trajectory = *trajectory;
            ExtendStableKey(
                result.fingerprint,
                static_cast<uint8_t>(instance.style));
            ExtendStableKey(
                result.fingerprint,
                static_cast<uint16_t>(type));

            const auto& rtd =
                instance.ride->getRideTypeDescriptor();
            const bool covered =
                trackTypeIsCovered(
                    instance.track->getTrackType());
            const auto drawer =
                getTrackDrawerEntry(
                    rtd, instance.track->isInverted(),
                    covered);
            const auto scheme = std::min<uint8_t>(
                instance.track->getColourScheme(),
                uint8_t(kNumRideColourSchemes - 1));

            for (uint8_t rotation = 0;
                 rotation < 4; ++rotation)
            {
                Drawing::RenderTarget target{};
                target.x = -8192;
                target.y = -8192;
                target.width = 16384;
                target.height = 16384;
                target.cullingX = target.x;
                target.cullingY = target.y;
                target.cullingWidth = target.width;
                target.cullingHeight = target.height;
                target.zoom_level = ZoomLevel{ 0 };

                auto* session =
                    PaintSessionAlloc(target, 0, rotation);
                if (session == nullptr)
                    return result;

                session->CurrentSource =
                    PaintStructSource::tile;
                session->TrackColours = ImageId(
                    0,
                    instance.ride
                        ->trackColours[scheme].main,
                    instance.ride
                        ->trackColours[scheme].additional);
                session->SupportColours = ImageId(
                    0,
                    instance.ride
                        ->trackColours[scheme].supports,
                    instance.ride
                        ->trackColours[scheme].additional);
                session->Flags =
                    PaintSessionFlags::IsTrackPiecePreview;

                std::vector<TrackElement>
                    syntheticElements(
                        ted.sequenceData.numSequences);
                for (uint8_t sequence = 0;
                     sequence
                         < ted.sequenceData.numSequences;
                     ++sequence)
                {
                    const auto& clearance =
                        ted.sequenceData
                            .sequences[sequence].clearance;
                    const CoordsXY tile{
                        kAnchor.x + clearance.x,
                        kAnchor.y + clearance.y,
                    };
                    const int32_t height =
                        kAnchor.z + clearance.z;

                    auto& synthetic =
                        syntheticElements[sequence];
                    synthetic.setType(
                        TileElementType::track);
                    synthetic.setDirection(0);
                    synthetic.setBaseZ(height);
                    synthetic.setClearanceZ(
                        height + 64);
                    synthetic.setTrackType(type);
                    synthetic.setRideType(
                        instance.track->getRideType());
                    synthetic.setSequenceIndex(sequence);
                    synthetic.setRideIndex(
                        instance.track->getRideIndex());
                    synthetic.setColourScheme(
                        static_cast<RideColourScheme>(
                            instance.track
                                ->getColourScheme()));
                    synthetic.setInverted(
                        instance.track->isInverted());

                    session->MapPosition = tile;
                    session->SpritePosition =
                        GetTileElementPaintSpritePosition(
                            tile, rotation);
                    session->CurrentlyDrawnTileElement =
                        reinterpret_cast<TileElement*>(
                            &synthetic);
                    session->LastPS = nullptr;
                    session->LastAttachedPS = nullptr;
                    session->InteractionType =
                        ViewportInteractionItem::ride;

                    paintFunction(
                        *session, *instance.ride,
                        sequence, rotation, height,
                        synthetic, drawer.supportType);
                }

                PaintSessionArrange(*session);
                const auto anchorScreen =
                    Translate3DTo2DWithZ(
                        rotation, kAnchor);
                for (auto* root = session->PaintHead;
                     root != nullptr;
                     root = root->NextQuadrantEntry)
                {
                    CollectFirstPersonSyntheticTrackPaintTree(
                        result.artwork, rotation,
                        *root, anchorScreen,
                        result.fingerprint);
                }
                PaintSessionFree(session);
            }

            result.valid =
                FirstPersonTrackObservationHasCompleteChannel(
                    result.artwork);
            return result;
        }

        void RejectFirstPersonTrackProfileCalibration(
            const FirstPersonTrackCalibrationInstance& instance,
            FirstPersonTrackProfileCalibrationState& state)
        {
            state.rejected = true;
            state.phase =
                FirstPersonTrackProfileCalibrationState::Phase::complete;
            state.sourceArtwork = {};
            state.sourceTrajectory = {};
            state.search = {};
            WithdrawFirstPersonVerifiedTrackProfile(
                instance.style);
        }

        [[nodiscard]] bool
            FirstPersonTrackGroupNeedsCalibrationViews(
                const ReconstructionGroupInfo& group)
        {
            const auto instance =
                ResolveFirstPersonTrackCalibrationInstance(
                    group);
            if (!instance.has_value())
                return false;

            auto& state =
                _trackProfileCalibrations[
                    static_cast<uint8_t>(
                        instance->style)];
            return state.phase
                != FirstPersonTrackProfileCalibrationState::Phase::complete;
        }

        void UpdateFirstPersonTrackProfileCalibration(
            const ReconstructionGroupInfo& group)
        {
            const auto instance =
                ResolveFirstPersonTrackCalibrationInstance(
                    group);
            if (!instance.has_value())
                return;

            auto& state =
                _trackProfileCalibrations[
                    static_cast<uint8_t>(
                        instance->style)];
            using Phase =
                FirstPersonTrackProfileCalibrationState::Phase;
            if (state.phase == Phase::complete)
                return;

            if (state.phase == Phase::verifySource)
            {
                const auto source =
                    PaintFirstPersonCanonicalTrackObservation(
                        *instance, TrackElemType::flat);
                if (source.supported && source.valid
                    && source.fingerprint
                        == state.sourceFingerprint)
                {
                    PublishFirstPersonVerifiedTrackProfile(
                        instance->style, state.profile,
                        state.sourceFingerprint,
                        state.passedKinds,
                        state.passedHoldouts);
                    state.phase = Phase::complete;
                    return;
                }

                WithdrawFirstPersonVerifiedTrackProfile(
                    instance->style);
                state = FirstPersonTrackProfileCalibrationState{};
                return;
            }

            if (state.phase == Phase::captureSource)
            {
                auto source =
                    PaintFirstPersonCanonicalTrackObservation(
                        *instance, TrackElemType::flat);
                if (!source.supported || !source.valid)
                {
                    RejectFirstPersonTrackProfileCalibration(
                        *instance, state);
                    return;
                }

                state.sourceArtwork =
                    std::move(source.artwork);
                state.sourceTrajectory =
                    std::move(source.trajectory);
                state.sourceAnchor = source.anchor;
                state.sourceFingerprint =
                    source.fingerprint;
                state.search = {};
                state.phase = Phase::fitSource;
                return;
            }

            if (state.phase == Phase::fitSource)
            {
                constexpr size_t kCandidateBudget = 4;
                if (!StepFirstPersonTrackRailProfileFromArtwork(
                        state.sourceArtwork,
                        state.sourceTrajectory,
                        state.sourceAnchor,
                        state.search,
                        kCandidateBudget))
                    return;

                if (!state.search.result.valid)
                {
                    RejectFirstPersonTrackProfileCalibration(
                        *instance, state);
                    return;
                }

                state.profile =
                    state.search.result.profile;
                state.search = {};
                state.phase = Phase::validateHoldouts;
                return;
            }

            struct HoldoutCategory
            {
                FirstPersonTrackValidationKind kind;
                std::array<TrackElemType, 4> candidates;
            };
            static constexpr std::array<
                HoldoutCategory, 3>
                kCategories{ {
                    {
                        FirstPersonTrackValidationKind::curve,
                        {
                            TrackElemType::leftQuarterTurn3Tiles,
                            TrackElemType::leftQuarterTurn5Tiles,
                            TrackElemType::sBendLeft,
                            TrackElemType::leftQuarterTurn1Tile,
                        },
                    },
                    {
                        FirstPersonTrackValidationKind::slope,
                        {
                            TrackElemType::up25,
                            TrackElemType::flatToUp25,
                            TrackElemType::up25ToFlat,
                            TrackElemType::down25,
                        },
                    },
                    {
                        FirstPersonTrackValidationKind::bank,
                        {
                            TrackElemType::leftBank,
                            TrackElemType::flatToLeftBank,
                            TrackElemType::leftBankToFlat,
                            TrackElemType::rightBank,
                        },
                    },
                } };

            if (state.holdoutCategory >= kCategories.size())
            {
                if (state.requiredKinds == 0
                    || state.passedKinds
                        != state.requiredKinds)
                {
                    RejectFirstPersonTrackProfileCalibration(
                        *instance, state);
                    return;
                }

                state.profile.verified = true;
                PublishFirstPersonVerifiedTrackProfile(
                    instance->style, state.profile,
                    state.sourceFingerprint,
                    state.passedKinds,
                    state.passedHoldouts);
                state.sourceArtwork = {};
                state.sourceTrajectory = {};
                state.search = {};
                state.phase = Phase::complete;
                return;
            }

            const auto& category =
                kCategories[state.holdoutCategory];
            if (state.holdoutCandidate
                >= category.candidates.size())
            {
                if (state.holdoutCategorySupported)
                {
                    RejectFirstPersonTrackProfileCalibration(
                        *instance, state);
                    return;
                }
                ++state.holdoutCategory;
                state.holdoutCandidate = 0;
                state.holdoutCategorySupported = false;
                return;
            }

            const auto type =
                category.candidates[
                    state.holdoutCandidate++];
            const auto holdout =
                PaintFirstPersonCanonicalTrackObservation(
                    *instance, type);
            if (!holdout.supported)
                return;

            state.holdoutCategorySupported = true;
            const uint8_t kind =
                static_cast<uint8_t>(category.kind);
            state.requiredKinds |= kind;
            if (!holdout.valid)
                return;

            const auto fit =
                ValidateFirstPersonTrackRailProfileAgainstArtwork(
                    holdout.artwork,
                    holdout.trajectory,
                    holdout.anchor,
                    state.profile);
            if (!IsFirstPersonTrackHoldoutFitReliable(fit))
                return;

            state.passedKinds |= kind;
            ++state.passedHoldouts;
            ++state.holdoutCategory;
            state.holdoutCandidate = 0;
            state.holdoutCategorySupported = false;
        }

        void ApplyFirstPersonTrackRailArtworkMask(        void ApplyFirstPersonTrackRailArtworkMask(
            FirstPersonSurface& surface, uint8_t rotation,
            TrackTrajectoryCacheEntry& trajectory)
        {
            if (rotation >= 4 || trajectory.sourceChannelMask == 0
                || trajectory.railSilhouettes[rotation].empty()
                || !surface.viewFacing || !surface.image.HasValue()
                || surface.image.IsBlended() || surface.mask.HasValue()
                || !surface.immutablePixels.empty())
                return;
            const auto* g1 = GfxGetG1Element(surface.image);
            if (g1 == nullptr)
                return;

            const int32_t left =
                int32_t(std::lround(surface.billboardLeft));
            const int32_t top =
                int32_t(std::lround(surface.billboardTop));
            uint8_t imageChannelFlags = 0;
            if (surface.image.HasPrimary())
                imageChannelFlags |= 1u << 0;
            if (surface.image.HasSecondary())
                imageChannelFlags |= 1u << 1;
            if (surface.image.HasTertiary())
                imageChannelFlags |= 1u << 2;

            uint64_t key = 1469598103934665603ull;
            const auto extendKey = [&](uint64_t value) {
                key ^= value;
                key *= 1099511628211ull;
            };
            extendKey(surface.image.GetIndex());
            extendKey(uint32_t(left));
            extendKey(uint32_t(top));
            extendKey(rotation);
            extendKey(trajectory.sourceChannelMask);
            extendKey(imageChannelFlags);

            const auto applyCached =
                [&](const TrackTrajectoryCacheEntry::MaskedArtwork& cached) {
                    if (!cached.changed)
                        return;
                    surface.immutablePixels = cached.pixels;
                    surface.immutableWidth = cached.width;
                    surface.immutableHeight = cached.height;
                    surface.immutableFingerprint = cached.fingerprint;
                };

            if (const auto found = trajectory.maskedArtwork.find(key);
                found != trajectory.maskedArtwork.end()
                    && found->second.image == surface.image.GetIndex()
                    && found->second.left == left
                    && found->second.top == top
                    && found->second.rotation == rotation
                    && found->second.channelMask
                        == trajectory.sourceChannelMask
                    && found->second.imageChannelFlags == imageChannelFlags)
            {
                applyCached(found->second);
                return;
            }

            TrackTrajectoryCacheEntry::MaskedArtwork cached{};
            cached.image = surface.image.GetIndex();
            cached.left = left;
            cached.top = top;
            cached.width = g1->width;
            cached.height = g1->height;
            cached.rotation = rotation;
            cached.channelMask = trajectory.sourceChannelMask;
            cached.imageChannelFlags = imageChannelFlags;

            auto pixels = DecodeFirstPersonTrackSprite(*g1);
            if (!pixels.has_value())
            {
                trajectory.maskedArtwork[key] = std::move(cached);
                return;
            }

            const auto& railSilhouette =
                trajectory.railSilhouettes[rotation];
            for (int32_t y = 0; y < g1->height; ++y)
            for (int32_t x = 0; x < g1->width; ++x)
            {
                auto& pixel =
                    (*pixels)[size_t(y) * size_t(g1->width) + size_t(x)];
                if (pixel == 0)
                    continue;

                uint8_t pixelMask = 0;
                for (size_t channelIndex = 0;
                     channelIndex < kFirstPersonTrackPixelChannelCount;
                     ++channelIndex)
                {
                    const auto channel =
                        static_cast<FirstPersonTrackPixelChannel>(
                            channelIndex);
                    if (FirstPersonTrackChannelEnabledForImage(
                            surface.image, channel)
                        && FirstPersonTrackPixelMatchesChannel(
                            pixel, channel))
                    {
                        pixelMask |=
                            FirstPersonTrackPixelChannelBit(channel);
                    }
                }
                if ((pixelMask & trajectory.sourceChannelMask) == 0
                    || !FirstPersonTrackSilhouetteContainsNear(
                        railSilhouette, left + x, top + y, 1))
                    continue;
                pixel = 0;
                cached.changed = true;
            }

            if (cached.changed)
            {
                cached.pixels = std::move(*pixels);
                uint64_t fingerprint = 1469598103934665603ull;
                const auto extendFingerprint = [&](uint64_t value) {
                    fingerprint ^= value;
                    fingerprint *= 1099511628211ull;
                };
                extendFingerprint(surface.image.GetIndex());
                extendFingerprint(rotation);
                for (const auto pixel : cached.pixels)
                    extendFingerprint(pixel);
                cached.fingerprint = fingerprint;
            }

            auto [inserted, ignored] =
                trajectory.maskedArtwork.insert_or_assign(
                    key, std::move(cached));
            (void)ignored;
            applyCached(inserted->second);
        }

        [[nodiscard]] uint8_t FirstPersonColourShade(
            const Drawing::ColourShadeMap& shades, uint8_t shade)
        {
            switch (std::min<uint8_t>(shade, 11))
            {
                case 0: return static_cast<uint8_t>(shades.colour0);
                case 1: return static_cast<uint8_t>(shades.colour1);
                case 2: return static_cast<uint8_t>(shades.darkest);
                case 3: return static_cast<uint8_t>(shades.darker);
                case 4: return static_cast<uint8_t>(shades.dark);
                case 5: return static_cast<uint8_t>(shades.midDark);
                case 6: return static_cast<uint8_t>(shades.midLight);
                case 7: return static_cast<uint8_t>(shades.light);
                case 8: return static_cast<uint8_t>(shades.lighter);
                case 9: return static_cast<uint8_t>(shades.lightest);
                case 10: return static_cast<uint8_t>(shades.colour10);
                default: return static_cast<uint8_t>(shades.colour11);
            }
        }

        [[nodiscard]] uint8_t FirstPersonRailColour(
            const Ride& ride, const TrackElement& track,
            const FirstPersonTrackRailProfile& profile, bool topFace)
        {
            const auto scheme = std::min<uint8_t>(
                track.getColourScheme(),
                uint8_t(kNumRideColourSchemes - 1));
            const uint8_t materialValue = topFace
                ? profile.topMaterialValue
                : profile.sideMaterialValue;

            if (profile.materialVerified)
            {
                const uint8_t railPaletteBit =
                    FirstPersonTrackPixelChannelBit(
                        FirstPersonTrackPixelChannel::trackRailPalette);
                if ((profile.sourceChannelMask & railPaletteBit) != 0
                    && materialValue >= static_cast<uint8_t>(
                        Drawing::PaletteIndex::trackRails0)
                    && materialValue <= static_cast<uint8_t>(
                        Drawing::PaletteIndex::trackRails2))
                {
                    return materialValue;
                }

                Drawing::Colour colour =
                    ride.trackColours[scheme].main;
                if ((profile.sourceChannelMask
                        & FirstPersonTrackPixelChannelBit(
                            FirstPersonTrackPixelChannel::secondaryRemap))
                    != 0)
                {
                    colour = ride.trackColours[scheme].additional;
                }
                else if ((profile.sourceChannelMask
                            & FirstPersonTrackPixelChannelBit(
                                FirstPersonTrackPixelChannel::tertiaryRemap))
                    != 0)
                {
                    colour = ride.trackColours[scheme].supports;
                }

                if (Drawing::colourIsValid(colour)
                    && materialValue <= 11)
                {
                    const uint8_t sampled =
                        FirstPersonColourShade(
                            Drawing::getColourMap(colour),
                            materialValue);
                    if (sampled != 0)
                        return sampled;
                }
            }

            auto colour = ride.trackColours[scheme].main;
            if (!Drawing::colourIsValid(colour))
                colour = Drawing::Colour::grey;
            const auto shades = Drawing::getColourMap(colour);
            uint8_t result = static_cast<uint8_t>(
                topFace ? shades.midLight : shades.midDark);
            if (result == 0)
            {
                result = static_cast<uint8_t>(
                    topFace
                        ? Drawing::PaletteIndex::trackRails2
                        : Drawing::PaletteIndex::trackRails1);
            }
            return result;
        }

        void AppendTrajectoryRailSegment(
            TrackTrajectoryCacheEntry& cached, uint64_t groupKey,
            const FirstPersonRailProxySegment& rail,
            uint8_t topColour, uint8_t sideColour)
        {
            const auto midpoint = Mul(Add(rail.a, rail.b), 0.5f);
            const int32_t tileX = int32_t(std::floor(
                midpoint.x / float(kCoordsXYStep)));
            const int32_t tileY = int32_t(std::floor(
                midpoint.y / float(kCoordsXYStep)));
            const uint64_t gpuRegion =
                FirstPersonGpuRegionKey(tileX, tileY);

            const auto acrossA =
                Mul(rail.basisA.right, rail.halfWidth);
            const auto acrossB =
                Mul(rail.basisB.right, rail.halfWidth);
            const auto upA =
                Mul(rail.basisA.up, rail.halfHeight);
            const auto upB =
                Mul(rail.basisB.up, rail.halfHeight);

            const auto emitFace =
                [&](const std::array<FirstPersonVec3, 4>& points,
                    uint8_t colour) {
                    FirstPersonSurface surface{};
                    surface.solidColour = colour;
                    surface.gpuRegion = gpuRegion;
                    surface.reconstructionGroup = groupKey;
                    EmitQuad(surface, { {
                        { points[0], 0.0f, 0.0f },
                        { points[1], 0.0f, 0.0f },
                        { points[2], 0.0f, 0.0f },
                        { points[3], 0.0f, 0.0f },
                    } });
                    cached.surfaces.emplace_back(
                        std::move(surface));
                };

            emitFace({ {
                Add(Sub(rail.a, acrossA), upA),
                Add(Add(rail.a, acrossA), upA),
                Add(Add(rail.b, acrossB), upB),
                Add(Sub(rail.b, acrossB), upB),
            } }, topColour);
            emitFace({ {
                Sub(Sub(rail.a, acrossA), upA),
                Sub(Sub(rail.b, acrossB), upB),
                Sub(Add(rail.b, acrossB), upB),
                Sub(Add(rail.a, acrossA), upA),
            } }, sideColour);
            emitFace({ {
                Sub(Add(rail.a, acrossA), upA),
                Sub(Add(rail.b, acrossB), upB),
                Add(Add(rail.b, acrossB), upB),
                Add(Add(rail.a, acrossA), upA),
            } }, sideColour);
            emitFace({ {
                Sub(Sub(rail.a, acrossA), upA),
                Add(Sub(rail.a, acrossA), upA),
                Add(Sub(rail.b, acrossB), upB),
                Sub(Sub(rail.b, acrossB), upB),
            } }, sideColour);
        }

        void UpdateTrackTrajectoryBounds(TrackTrajectoryCacheEntry& cached)
        {
            cached.hasBounds = false;
            if (cached.surfaces.empty())
                return;
            FirstPersonVec3 low = cached.surfaces.front().triangles.front().world;
            FirstPersonVec3 high = low;
            for (const auto& surface : cached.surfaces)
            for (const auto& vertex : surface.triangles)
            {
                const auto& p = vertex.world;
                low.x = std::min(low.x, p.x);
                low.y = std::min(low.y, p.y);
                low.z = std::min(low.z, p.z);
                high.x = std::max(high.x, p.x);
                high.y = std::max(high.y, p.y);
                high.z = std::max(high.z, p.z);
            }
            cached.minTileX = int32_t(std::floor(low.x / float(kCoordsXYStep)));
            cached.minTileY = int32_t(std::floor(low.y / float(kCoordsXYStep)));
            cached.maxTileX = int32_t(std::floor(high.x / float(kCoordsXYStep)));
            cached.maxTileY = int32_t(std::floor(high.y / float(kCoordsXYStep)));
            cached.hasBounds = true;
        }

        void IndexTrackTrajectoryRegions(
            TrackTrajectoryCacheEntry& cached)
        {
            cached.regionSurfaceIndices.clear();
            for (size_t i = 0;
                 i < cached.surfaces.size(); ++i)
            {
                const auto region =
                    cached.surfaces[i].gpuRegion;
                if (region != 0)
                {
                    cached.regionSurfaceIndices[
                        region].push_back(i);
                }
            }
        }

        [[nodiscard]] std::optional<FirstPersonTrackTrajectory> NextFirstPersonTrackTrajectory(
            const Ride& ride, const CoordsXYZ& sampleOrigin, TileElement* originElement)
        {
            CoordsXYE input{ sampleOrigin, originElement };
            CoordsXYE next{};
            int32_t nextZ{};
            int32_t nextDirection{};
            if (!trackBlockGetNext(&input, &next, &nextZ, &nextDirection)
                || next.element == nullptr)
                return std::nullopt;

            const auto* nextTrack = next.element->asTrack();
            if (nextTrack == nullptr
                || nextTrack->getRideIndex() != ride.id
                || !FirstPersonTrackTrajectoryTemplateSamplesContinuous(
                    nextTrack->getTrackType(),
                    uint8_t(nextDirection)))
                return std::nullopt;
            return BuildFirstPersonTrackTrajectory(
                nextTrack->getTrackType(), uint8_t(nextDirection),
                { float(next.x), float(next.y), float(nextZ) });
        }

        TrackTrajectoryCacheEntry BuildTrackTrajectoryGeometry(
            const Ride& ride, const TrackElement& track, TileElement* originElement,
            const CoordsXYZ& sampleOrigin, const FirstPersonVec3& groupAnchor,
            uint64_t groupKey, uint64_t signature,
            const FirstPersonTrackTrajectory& trajectory, uint64_t frame)
        {
            TrackTrajectoryCacheEntry result{};
            result.signature = signature;
            result.lastSeen = frame;
            const auto verifiedProfile =
                FirstPersonVerifiedTrackRailProfile(ride, track);
            if (!verifiedProfile.has_value() || !verifiedProfile->verified)
                return result;
            const auto& profile = *verifiedProfile;
            result.sourceChannelMask = profile.sourceChannelMask;
            result.railSilhouettes = BuildFirstPersonTrackRailSilhouettes(
                trajectory, groupAnchor, profile);
            const uint8_t topColour =
                FirstPersonRailColour(ride, track, profile, true);
            const uint8_t sideColour =
                FirstPersonRailColour(ride, track, profile, false);

            const auto railProxies =
                BuildFirstPersonRailProxySegments(
                    trajectory, profile);
            for (const auto& rail : railProxies)
            {
                AppendTrajectoryRailSegment(
                    result, groupKey, rail,
                    topColour, sideColour);
            }

            if (const auto next = NextFirstPersonTrackTrajectory(
                    ride, sampleOrigin, originElement);
                next.has_value())
            {
                const float gap = FirstPersonTrackTrajectoryEndpointGap(trajectory, *next);
                if (gap <= 4.0f)
                {
                    if (gap > 0.05f)
                    {
                        for (const float gaugeSide :
                            { -profile.halfGauge, profile.halfGauge })
                        {
                            const auto& a = trajectory.points.back();
                            const auto& b = next->points.front();
                            const FirstPersonRailProxySegment bridge{
                                FirstPersonRailProxyCentre(
                                    a, profile, gaugeSide),
                                FirstPersonRailProxyCentre(
                                    b, profile, gaugeSide),
                                a.basis,
                                b.basis,
                                profile.halfWidth,
                                profile.halfHeight,
                                FirstPersonPhysicalProxyProvenance::
                                    verifiedTrackArtwork,
                            };
                            AppendTrajectoryRailSegment(
                                result, groupKey, bridge,
                                topColour, sideColour);
                        }
                    }
                }
                else
                {
                    result.boundaryContinuous = false;
                }
            }

            UpdateTrackTrajectoryBounds(result);
            IndexTrackTrajectoryRegions(result);
            return result;
        }

        void CollectTrackTrajectories(FirstPersonScene& scene)
        {
            const uint64_t frame = _terrainCache.frame;
            const auto eraseCached =
                [&](uint64_t groupKey) {
                    const auto old =
                        _trackTrajectoryCache.find(groupKey);
                    if (old == _trackTrajectoryCache.end())
                        return;
                    MarkTrackTrajectoryRegionsDirty(
                        old->second);
                    UnregisterTrackTrajectoryRegionMembership(
                        groupKey, old->second);
                    _trackTrajectoryCache.erase(old);
                };

            if (!FirstPersonHasVerifiedTrackProfiles())
            {
                if (!_trackTrajectoryCache.empty())
                {
                    for (const auto& [groupKey, cached] :
                         _trackTrajectoryCache)
                    {
                        (void)groupKey;
                        MarkTrackTrajectoryRegionsDirty(
                            cached);
                    }
                    _trackTrajectoryCache.clear();
                    _trackTrajectoryGroupsByRegion.clear();
                }
                return;
            }

            std::unordered_set<uint64_t> seenGroups;
            seenGroups.reserve(
                scene.visibleTiles.size() / 2 + 1);

            for (const auto tile : scene.visibleTiles)
            {
                auto* element = MapGetFirstElementAt(tile);
                if (element == nullptr)
                    continue;
                do
                {
                    if (element->getType()
                            != TileElementType::track
                        || element->isGhost()
                        || element->isInvisible())
                        continue;
                    auto* track = element->asTrack();
                    const auto group =
                        GetReconstructionGroup(
                            tile, element);
                    if (track == nullptr
                        || !group.has_value()
                        || !seenGroups.insert(
                            group->key).second)
                        continue;

                    const auto* ride =
                        GetRide(track->getRideIndex());
                    if (ride == nullptr
                        || !RideUsesStandardFirstPersonTrajectory(
                            *ride))
                    {
                        eraseCached(group->key);
                        continue;
                    }

                    const auto sampleOrigin =
                        FirstPersonTrackSampleOrigin(
                            tile, element);
                    if (!sampleOrigin.has_value())
                    {
                        eraseCached(group->key);
                        continue;
                    }

                    const auto* trajectoryTemplate =
                        GetFirstPersonTrackTrajectoryTemplate(
                            track->getTrackType(),
                            track->getDirection());
                    if (trajectoryTemplate == nullptr
                        || !FirstPersonTrackTrajectoryTemplateSamplesContinuous(
                            track->getTrackType(),
                            track->getDirection()))
                    {
                        eraseCached(group->key);
                        continue;
                    }

                    uint64_t signature = group->key;
                    ExtendStableKey(
                        signature,
                        track->getColourScheme());
                    ExtendStableKey(
                        signature,
                        track->isInverted() ? 1 : 0);
                    const auto scheme =
                        std::min<uint8_t>(
                            track->getColourScheme(),
                            uint8_t(
                                kNumRideColourSchemes
                                - 1));
                    ExtendStableKey(
                        signature,
                        EnumValue(
                            ride->trackColours[
                                scheme].main));
                    ExtendStableKey(
                        signature,
                        FirstPersonTrackProfileSignature(
                            *ride, *track));
                    ExtendStableKey(
                        signature,
                        trajectoryTemplate->points.size());

                    auto it =
                        _trackTrajectoryCache.find(
                            group->key);
                    if (it != _trackTrajectoryCache.end()
                        && it->second.signature
                            == signature)
                    {
                        it->second.lastSeen = frame;
                        if (it->second.dirty)
                        {
                            it->second.dirty = false;
                            MarkTrackTrajectoryRegionsDirty(
                                it->second);
                        }
                        continue;
                    }

                    auto* originElement =
                        FindFirstPersonTrackOriginElement(
                            *sampleOrigin, *track);
                    if (originElement == nullptr)
                    {
                        eraseCached(group->key);
                        continue;
                    }

                    const auto trajectory =
                        BuildFirstPersonTrackTrajectory(
                            track->getTrackType(),
                            track->getDirection(),
                            {
                                float(sampleOrigin->x),
                                float(sampleOrigin->y),
                                float(sampleOrigin->z),
                            });
                    if (!trajectory.has_value())
                    {
                        eraseCached(group->key);
                        continue;
                    }

                    if (it !=
                        _trackTrajectoryCache.end())
                    {
                        MarkTrackTrajectoryRegionsDirty(
                            it->second);
                        UnregisterTrackTrajectoryRegionMembership(
                            group->key, it->second);
                    }

                    auto rebuilt =
                        BuildTrackTrajectoryGeometry(
                            *ride, *track, originElement,
                            *sampleOrigin, group->anchor,
                            group->key, signature,
                            *trajectory, frame);
                    MarkTrackTrajectoryRegionsDirty(
                        rebuilt);
                    _trackTrajectoryCache[
                        group->key] =
                        std::move(rebuilt);
                    RegisterTrackTrajectoryRegionMembership(
                        group->key,
                        _trackTrajectoryCache[
                            group->key]);
                } while (!(element++)->isLastForTile());
            }

            if (frame % 120 == 0)
            {
                for (auto it =
                         _trackTrajectoryCache.begin();
                     it != _trackTrajectoryCache.end();)
                {
                    if (frame - it->second.lastSeen
                        <= 240)
                    {
                        ++it;
                        continue;
                    }
                    MarkTrackTrajectoryRegionsDirty(
                        it->second);
                    UnregisterTrackTrajectoryRegionMembership(
                        it->first, it->second);
                    it = _trackTrajectoryCache.erase(it);
                }
            }
        }

        struct TileSemanticSnapshot
        {        struct TileSemanticSnapshot
        {
            uint64_t signature = 0;
            int32_t minZ = 0;
            int32_t maxZ = 0;
            bool populated = false;
        };

        TileSemanticSnapshot ReadTileSemanticSnapshot(CoordsXY pos)
        {
            const auto* elem = MapGetFirstElementAt(pos);
            if (elem == nullptr)
                return {};

            TileSemanticSnapshot result{};
            result.signature = 14695981039346656037ull;
            result.minZ = 4096;
            result.maxZ = -512;
            do
            {
                result.populated = true;
                result.minZ = std::min(result.minZ, elem->getBaseZ());
                result.maxZ = std::max(result.maxZ, SemanticClearanceZ(*elem));
                if (elem->getType() == TileElementType::surface)
                    result.maxZ = std::max(result.maxZ, elem->asSurface()->getWaterHeight());

                const auto* bytes = reinterpret_cast<const uint8_t*>(elem);
                for (size_t n = 0; n < sizeof(TileElement); ++n)
                {
                    result.signature ^= uint64_t(bytes[n]);
                    result.signature *= 1099511628211ull;
                }
            } while (!(elem++)->isLastForTile());
            return result;
        }

        uint64_t NativeTileSignature(CoordsXY pos)
        {
            return ReadTileSemanticSnapshot(pos).signature;
        }

        uint64_t TerrainKey(int32_t tx, int32_t ty)
        {
            return (uint64_t(uint32_t(tx)) << 32) | uint32_t(ty);
        }
        struct RegionCacheEntry
        {
            int32_t x1{}, y1{};
            int32_t minZ{}, maxZ{};
            bool populated = false;
            bool dirty = true;
        };
        static std::unordered_map<uint64_t, RegionCacheEntry> _regionBounds;

        FirstPersonRegionSphere CachedRegionBounds(int32_t x0, int32_t y0, int32_t x1, int32_t y1)
        {
            constexpr float kTile = float(kCoordsXYStep);
            float low = -512.0f;
            float high = 4096.0f;
            bool populated = true;
            if (x1-x0 <= 16 && y1-y0 <= 16)
            {
                auto& entry = _regionBounds[TerrainKey(x0,y0)];
                if (entry.dirty || entry.x1 != x1 || entry.y1 != y1)
                {
                    entry = {};
                    entry.x1=x1; entry.y1=y1;
                    entry.minZ=4096; entry.maxZ=-512;
                    for(int32_t ty=y0;ty<y1;++ty)
                    for(int32_t tx=x0;tx<x1;++tx)
                    {
                        const CoordsXY p{tx*kCoordsXYStep,ty*kCoordsXYStep};
                        if (!MapIsLocationValid(p)) continue;
                        const auto* tile = MapGetFirstElementAt(p);
                        if(tile==nullptr) continue;
                        do
                        {
                            entry.populated = true;
                            entry.minZ = std::min(entry.minZ,tile->getBaseZ());
                            entry.maxZ = std::max(entry.maxZ,SemanticClearanceZ(*tile));
                            if(tile->getType()==TileElementType::surface)
                                entry.maxZ = std::max(entry.maxZ,tile->asSurface()->getWaterHeight());
                        } while(!(tile++)->isLastForTile());
                    }
                    entry.dirty=false;
                }
                populated = entry.populated;
                low = float(entry.minZ);
                high = float(entry.maxZ);
            }
            const float dx = float(x1-x0)*0.5f*kTile;
            const float dy = float(y1-y0)*0.5f*kTile;
            const float dz = (high-low)*0.5f;
            // Occupancy/paint bounds can underestimate art overhang. Reserve
            // one conservative halo for tall sprite components and entities.
            const float halo = (x1-x0 <= 16 && y1-y0 <= 16) ? 512.0f : 0.0f;
            return {{float(x0+x1)*0.5f*kTile,float(y0+y1)*0.5f*kTile,(high+low)*0.5f},
                     std::sqrt(dx*dx+dy*dy+dz*dz)+halo,populated};
        }

        std::optional<FirstPersonSemanticSphere> CachedTileVisibilityBounds(
            CoordsXY world, uint32_t sourceGeneration)
        {
            const int32_t tx = world.x / kCoordsXYStep;
            const int32_t ty = world.y / kCoordsXYStep;
            const uint64_t key = TerrainKey(tx, ty);
            auto& cached = _staticPaintCache[key];
            constexpr uint64_t kVisibilityProbeInterval = 240;
            const bool probeDue = !cached.visibilityBoundValid || cached.visibilityDirty
                || FirstPersonRefreshDue(
                    key ^ 0xd6e8feb86659fd93ull, sourceGeneration,
                    cached.lastVisibilityGeneration, kVisibilityProbeInterval);
            if (probeDue)
            {
                const auto snapshot = ReadTileSemanticSnapshot(world);
                if (!snapshot.populated)
                {
                    cached.visibilityBoundValid = false;
                    cached.visibilityDirty = false;
                    cached.lastVisibilityGeneration = sourceGeneration;
                    return std::nullopt;
                }

                if (cached.valid && cached.signature != snapshot.signature)
                    cached.dirty = true;
                cached.signature = snapshot.signature;
                cached.visibilityMinZ = snapshot.minZ;
                cached.visibilityMaxZ = snapshot.maxZ;
                cached.visibilityBoundValid = true;
                cached.visibilityDirty = false;
                cached.lastVisibilityGeneration = sourceGeneration;
            }

            if (!cached.visibilityBoundValid)
                return std::nullopt;
            const float halfZ = 0.5f
                * float(cached.visibilityMaxZ - cached.visibilityMinZ);
            const FirstPersonVec3 center{
                float(world.x + kCoordsXYHalfTile),
                float(world.y + kCoordsXYHalfTile),
                0.5f * float(cached.visibilityMinZ + cached.visibilityMaxZ)
            };
            // Preserve the previous conservative overhang halo. Entity visual
            // admission is handled separately from this static tile bound.
            const float radius = std::sqrt(
                2.0f * float(kCoordsXYHalfTile * kCoordsXYHalfTile)
                + halfZ * halfZ) + 256.0f;
            return FirstPersonSemanticSphere{ center, radius };
        }

        // Complete-park coverage: partition the authoritative tile map, not a
        // camera-centred square. Region bounds are broad on purpose: the
        // base terrain of a tile cannot safely cull a tall ride on that tile.
        void DiscoverVisibleTiles(FirstPersonScene& scene)
        {
            PROFILED_FUNCTION();
            const auto& view = scene.resolvedView;
            const auto frustum = FirstPersonFrustum(
                view.camera, view.fieldOfViewDegrees, view.aspect,
                view.nearClip, view.farClip);
            const auto map = getGameState().mapSize;
            auto visit = [&](int32_t x0, int32_t y0, int32_t x1, int32_t y1) {
                for (int32_t ty = y0; ty < y1; ++ty)
                for (int32_t tx = x0; tx < x1; ++tx)
                {
                    const CoordsXY world{ tx * kCoordsXYStep, ty * kCoordsXYStep };
                    if (!MapIsLocationValid(world)) continue;
                    const auto semantic = CachedTileVisibilityBounds(
                        world, getGameState().currentTicks);
                    if (semantic.has_value()
                        && frustum.visible(semantic->center, semantic->radius))
                    {
                        scene.visibleTiles.emplace_back(world);
                    }
                }
            };
            auto bounds = [](int32_t x0,int32_t y0,int32_t x1,int32_t y1) {
                return CachedRegionBounds(x0,y0,x1,y1);
            };
            VisitFirstPersonRegions(frustum,0,0,map.x,map.y,visit,bounds);
        }

        void CollectTerrain(FirstPersonScene& scene)
        {
            PROFILED_FUNCTION();
            const auto& opt = scene.options;
            const auto frame = ++_terrainCache.frame;
            const auto& view = scene.resolvedView;
            const auto frustum = FirstPersonFrustum(
                view.camera, view.fieldOfViewDegrees, view.aspect,
                view.nearClip, view.farClip);
            for (const CoordsXY origin : scene.visibleTiles)
            {
                const int32_t tx = origin.x / kCoordsXYStep;
                const int32_t ty = origin.y / kCoordsXYStep;
                const int32_t x = origin.x;
                const int32_t y = origin.y;
                auto* tile = MapGetSurfaceElementAt(origin);
                if (tile == nullptr) continue;
                const auto baseZ = tile->getBaseZ();
                const auto waterZ = tile->getWaterHeight();
                const auto slope = tile->getSlope();
                const auto corners = GetSlopeCornerHeights(baseZ, slope);
                const float low = float(std::min({corners.north,corners.east,corners.south,corners.west}));
                const float high = float(std::max({corners.north,corners.east,corners.south,corners.west,waterZ}));
                // Region discovery keeps tall scenery independently. The terrain
                // itself can use its tighter real slope/water volume here.
                if (!frustum.visible({float(x+16),float(y+16),0.5f*(low+high)},
                                    48.0f+0.5f*(high-low))) continue;
                    const uint8_t sourceRotation = ChooseTerrainSourceRotation(slope);
                    const auto image = GetFirstPersonTerrainImage(*tile, origin, sourceRotation);
                    const auto* g1 = image.HasValue() ? GfxGetG1Element(image) : nullptr;
                    if (g1 == nullptr) continue;
                    const ImageId waterMask = waterZ > baseZ
                        ? GetFirstPersonWaterMaskImage(*tile, sourceRotation) : ImageId{};
                    const ImageId waterOverlay = waterZ > baseZ
                        ? GetFirstPersonWaterOverlayImage(*tile, opt.viewFlags, sourceRotation) : ImageId{};
                    const uint64_t terrainKey = TerrainKey(tx, ty);
                    auto& cache = _terrainCache.entries[terrainKey];
                    const bool wasDirty = cache.dirty;
                    if (const auto cachedTunnelHeight = CachedVerticalTunnelHeight(terrainKey);
                        cachedTunnelHeight.has_value())
                    {
                        const bool verticalOpening = FirstPersonVerticalTunnelCutsTerrain(
                            baseZ, *cachedTunnelHeight);
                        if (cache.verticalOpening != verticalOpening)
                        {
                            cache.verticalOpening = verticalOpening;
                            MarkStaticRegionDirtyForTile(tx, ty);
                        }
                    }
                    const bool terrainChanged =
                        cache.source != image || cache.waterMaskImage != waterMask ||
                        cache.waterOverlayImage != waterOverlay || cache.baseZ != baseZ || cache.slope != slope ||
                        cache.sourceRotation != sourceRotation || cache.waterZ != waterZ || cache.spriteX != g1->xOffset ||
                        cache.spriteY != g1->yOffset || cache.spriteWidth != g1->width ||
                        cache.spriteHeight != g1->height;
                    if (FirstPersonTerrainRevalidationNeedsRegionRebuild(
                            wasDirty, terrainChanged))
                    {
                        MarkStaticRegionDirtyForTile(tx, ty);
                    }
                    if (terrainChanged)
                    {
                        cache.source = image;
                        cache.baseZ = baseZ;
                        cache.slope = slope;
                        cache.sourceRotation = sourceRotation;
                        cache.waterZ = waterZ;
                        cache.spriteX = g1->xOffset;
                        cache.spriteY = g1->yOffset;
                        cache.spriteWidth = g1->width;
                        cache.spriteHeight = g1->height;
                        cache.waterMaskImage = waterMask;
                        cache.waterOverlayImage = waterOverlay;
                        const std::array<FirstPersonVec3, 4> world = { {
                            { float(x), float(y), float(corners.south) },
                            { float(x + kCoordsXYStep), float(y), float(corners.east) },
                            { float(x + kCoordsXYStep), float(y + kCoordsXYStep), float(corners.north) },
                            { float(x), float(y + kCoordsXYStep), float(corners.west) },
                        } };
                        // Use the matching native sprite origin for the selected
                        // quarter-turn; source view is world-stable, not camera-driven.
                        const auto sourceSpritePosition =
                            GetTileElementPaintSpritePosition(origin, sourceRotation);
                        const auto isoOrigin = Translate3DTo2DWithZ(
                            sourceRotation, { sourceSpritePosition, baseZ });
                        std::array<FirstPersonVertex, 4> v{};
                        for (size_t i = 0; i < world.size(); ++i)
                        {
                            const CoordsXYZ point{ int32_t(world[i].x), int32_t(world[i].y), int32_t(world[i].z) };
                            const auto src = Translate3DTo2DWithZ(sourceRotation, point);
                            v[i] = { world[i], float(src.x - isoOrigin.x - g1->xOffset),
                                               float(src.y - isoOrigin.y - g1->yOffset) };
                        }
                        cache.ground = {};
                        cache.ground.image = image;
                        cache.ground.edgeCoverage = true;
                        cache.ground.gpuRegion = FirstPersonGpuRegionKey(tx,ty);
                        EmitQuad(cache.ground, v, UsesOppositeTerrainDiagonal(slope));
                        cache.water.reset();
                        cache.waterOverlay.reset();
                        if (waterMask.HasValue())
                        {
                            const auto* wg1 = GfxGetG1Element(waterMask);
                            if (wg1 != nullptr)
                            {
                                FirstPersonSurface waterSurface{};
                                waterSurface.image = waterMask;
                                waterSurface.gpuRegion = FirstPersonGpuRegionKey(tx,ty);
                                std::array<FirstPersonVertex, 4> w{};
                                const auto waterIso = Translate3DTo2DWithZ(
                                    sourceRotation, { sourceSpritePosition, waterZ });
                                for (size_t i = 0; i < world.size(); ++i)
                                {
                                    const auto p = CoordsXYZ{ int32_t(world[i].x), int32_t(world[i].y), waterZ };
                                    const auto src = Translate3DTo2DWithZ(sourceRotation, p);
                                    w[i] = { { world[i].x, world[i].y, float(waterZ) },
                                             float(src.x-waterIso.x-wg1->xOffset),
                                             float(src.y-waterIso.y-wg1->yOffset) };
                                }
                                EmitQuad(waterSurface,w);
                                cache.water = std::move(waterSurface);
                            }
                            const auto* og1 = waterOverlay.HasValue() ? GfxGetG1Element(waterOverlay) : nullptr;
                            if (og1 != nullptr)
                            {
                                // The original game paints a separate water overlay.
                                // Leave it on top of the blended mask, but let its
                                // encoded holes expose the tint and world underneath.
                                FirstPersonSurface overlay{};
                                overlay.image = waterOverlay;
                                overlay.gpuRegion = FirstPersonGpuRegionKey(tx,ty);
                                std::array<FirstPersonVertex, 4> w{};
                                const auto waterIso = Translate3DTo2DWithZ(
                                    sourceRotation, { sourceSpritePosition, waterZ });
                                for (size_t i = 0; i < world.size(); ++i)
                                {
                                    const auto p = CoordsXYZ{ int32_t(world[i].x), int32_t(world[i].y), waterZ };
                                    const auto src = Translate3DTo2DWithZ(sourceRotation, p);
                                    w[i] = { { world[i].x, world[i].y, float(waterZ)+0.25f },
                                             float(src.x-waterIso.x-og1->xOffset),
                                             float(src.y-waterIso.y-og1->yOffset) };
                                }
                                EmitQuad(overlay,w);
                                cache.waterOverlay = std::move(overlay);
                            }
                        }
                    }
                    cache.lastSeen = frame;
                    cache.dirty = false;
                    if (!cache.verticalOpening && !IsResidentStaticSurface(cache.ground))
                        scene.surfaces.emplace_back(cache.ground);
                    if (cache.water.has_value() && !IsResidentStaticSurface(*cache.water))
                        scene.surfaces.emplace_back(*cache.water);
                    if (cache.waterOverlay.has_value() && !IsResidentStaticSurface(*cache.waterOverlay))
                        scene.surfaces.emplace_back(*cache.waterOverlay);
            }
            // Bound memory after travelling across multiple distant park regions.
            // Never retain a permanently growing copy of an explored park.
            if ((frame % 120) == 0)
            {
                std::erase_if(_terrainCache.entries, [frame](const auto& kv) {
                    const bool expired = frame - kv.second.lastSeen > 240;
                    if (expired)
                        MarkStaticRegionDirtyForTile(
                            int32_t(kv.first >> 32), int32_t(kv.first & 0xffffffffu));
                    return expired;
                });
            }
        }
        void RebuildStaticRegionPacket(uint64_t regionKey, uint64_t frame)
        {
            auto& packet = _staticRegionPackets[regionKey];
            packet.surfaces.clear();
            packet.textureDependencies.clear();
            packet.reconstructionGroups.clear();
            packet.reconstructionSelectionStamp = 0;

            const int32_t regionX = int32_t(uint32_t(regionKey >> 32)) - 1;
            const int32_t regionY = int32_t(uint32_t(regionKey & 0xffffffffu)) - 1;
            const int32_t x0 = regionX * 32;
            const int32_t y0 = regionY * 32;
            const int32_t x1 = x0 + 32;
            const int32_t y1 = y0 + 32;

            bool haveBounds = false;
            FirstPersonVec3 low{}, high{};
            std::unordered_set<uint32_t> dependencies;
            std::unordered_set<uint64_t> reconstructionDependencies;
            auto addSurface = [&](const FirstPersonSurface& surface) {
                if (!IsResidentStaticSurface(surface) || surface.gpuRegion != regionKey)
                    return;
                packet.surfaces.push_back(surface);
                if (surface.image.HasValue())
                    dependencies.insert(surface.image.GetIndex());
                if (surface.mask.HasValue())
                    dependencies.insert(surface.mask.GetIndex());

                FirstPersonVec3 surfaceLow{}, surfaceHigh{};
                if (surface.hasSemanticBounds)
                {
                    const auto& center = surface.semanticCenter;
                    const float r = surface.semanticRadius;
                    surfaceLow = { center.x - r, center.y - r, center.z - r };
                    surfaceHigh = { center.x + r, center.y + r, center.z + r };
                }
                else
                {
                    surfaceLow = surfaceHigh = surface.triangles[0].world;
                    for (const auto& vertex : surface.triangles)
                    {
                        const auto& p = vertex.world;
                        surfaceLow.x = std::min(surfaceLow.x, p.x);
                        surfaceLow.y = std::min(surfaceLow.y, p.y);
                        surfaceLow.z = std::min(surfaceLow.z, p.z);
                        surfaceHigh.x = std::max(surfaceHigh.x, p.x);
                        surfaceHigh.y = std::max(surfaceHigh.y, p.y);
                        surfaceHigh.z = std::max(surfaceHigh.z, p.z);
                    }
                }
                if (!haveBounds)
                {
                    low = surfaceLow;
                    high = surfaceHigh;
                    haveBounds = true;
                }
                else
                {
                    low.x = std::min(low.x, surfaceLow.x);
                    low.y = std::min(low.y, surfaceLow.y);
                    low.z = std::min(low.z, surfaceLow.z);
                    high.x = std::max(high.x, surfaceHigh.x);
                    high.y = std::max(high.y, surfaceHigh.y);
                    high.z = std::max(high.z, surfaceHigh.z);
                }
            };

            for (int32_t ty = y0; ty < y1; ++ty)
            for (int32_t tx = x0; tx < x1; ++tx)
            {
                const auto tileKey = TerrainKey(tx, ty);
                if (const auto terrainIt = _terrainCache.entries.find(tileKey);
                    terrainIt != _terrainCache.entries.end() && !terrainIt->second.dirty)
                {
                    const auto& terrain = terrainIt->second;
                    if (!terrain.verticalOpening)
                        addSurface(terrain.ground);
                    if (terrain.water.has_value()) addSurface(*terrain.water);
                    if (terrain.waterOverlay.has_value()) addSurface(*terrain.waterOverlay);
                }

                const auto cacheIt = _staticPaintCache.find(tileKey);
                if (cacheIt == _staticPaintCache.end() || !cacheIt->second.valid
                    || cacheIt->second.dirty || cacheIt->second.animated)
                    continue;
                const auto& cached = cacheIt->second;
                for (uint8_t rotation = 0; rotation < 4; ++rotation)
                {
                    const auto& variant = cached.rotations[rotation];
                    if (!variant.valid)
                        continue;
                    for (const auto& surface :
                         variant.residentSurfaces)
                    {
                        uint8_t selected =
                            cached.selectedRotation;
                        if (surface.reconstructionGroup != 0)
                        {
                            reconstructionDependencies.insert(
                                surface.reconstructionGroup);
                            const auto group =
                                _reconstructionRotations.find(
                                    surface.reconstructionGroup);
                            if (group
                                    == _reconstructionRotations.end()
                                || !group->second.hasSelectedRotation)
                                continue;
                            selected =
                                group->second.selectedRotation;
                        }
                        if (selected == rotation)
                            addSurface(surface);
                    }
                }
            }

            if (const auto membership =
                    _trackTrajectoryGroupsByRegion.find(
                        regionKey);
                membership
                    != _trackTrajectoryGroupsByRegion.end())
            {
                for (const auto groupKey :
                     membership->second)
                {
                    const auto trajectory =
                        _trackTrajectoryCache.find(
                            groupKey);
                    if (trajectory
                            == _trackTrajectoryCache.end()
                        || trajectory->second.dirty)
                        continue;
                    const auto indices =
                        trajectory->second
                            .regionSurfaceIndices.find(
                                regionKey);
                    if (indices
                        == trajectory->second
                            .regionSurfaceIndices.end())
                        continue;
                    for (const auto index :
                         indices->second)
                    {
                        if (index
                            < trajectory->second
                                .surfaces.size())
                        {
                            addSurface(
                                trajectory->second
                                    .surfaces[index]);
                        }
                    }
                }
            }
            if (_largeSceneryGeometryEnabled)
            {
                if (const auto membership =
                        _largeSceneryGroupsByRegion.find(regionKey);
                    membership != _largeSceneryGroupsByRegion.end())
                {
                    for (const auto groupKey : membership->second)
                    {
                        const auto geometry =
                            _largeSceneryGeometryCache.find(groupKey);
                        if (geometry == _largeSceneryGeometryCache.end()
                            || geometry->second.dirty)
                            continue;
                        for (const auto& surface : geometry->second.surfaces)
                            addSurface(surface);
                    }
                }
            }

            packet.textureDependencies.assign(
                dependencies.begin(), dependencies.end());
            std::sort(
                packet.textureDependencies.begin(),
                packet.textureDependencies.end());
            packet.reconstructionGroups.assign(
                reconstructionDependencies.begin(),
                reconstructionDependencies.end());
            std::sort(
                packet.reconstructionGroups.begin(),
                packet.reconstructionGroups.end());
            packet.reconstructionSelectionStamp =
                CurrentReconstructionSelectionStamp(
                    packet.reconstructionGroups);
            if (haveBounds)
            {
                packet.center = {
                    0.5f * (low.x + high.x),
                    0.5f * (low.y + high.y),
                    0.5f * (low.z + high.z),
                };
                const float dx = 0.5f * (high.x - low.x);
                const float dy = 0.5f * (high.y - low.y);
                const float dz = 0.5f * (high.z - low.z);
                packet.radius = std::sqrt(dx * dx + dy * dy + dz * dz) + 2.0f;
            }
            else
            {
                packet.center = {
                    float((x0 + x1) * kCoordsXYStep) * 0.5f,
                    float((y0 + y1) * kCoordsXYStep) * 0.5f,
                    0.0f,
                };
                packet.radius = 0.0f;
            }
            ++packet.generation;
            packet.lastSeen = frame;
            packet.dirty = false;
        }

        void SubmitVisibleStaticRegions(
            FirstPersonScene& scene, const FirstPersonFrustum& frustum, uint64_t frame)
        {
            std::unordered_set<uint64_t> visibleRegions;
            visibleRegions.reserve(scene.visibleTiles.size() / 64 + 1);
            for (const auto tile : scene.visibleTiles)
                visibleRegions.insert(FirstPersonGpuRegionKey(
                    tile.x / kCoordsXYStep, tile.y / kCoordsXYStep));
            // A single physical track piece can cross a 32x32 GPU-region edge.
            // If any sequence made the piece visible this frame, submit every
            // region occupied by its authoritative sampled trajectory.
            for (const auto& [groupKey, trajectory] :
                 _trackTrajectoryCache)
            {
                (void)groupKey;
                if (trajectory.dirty
                    || trajectory.lastSeen != frame)
                    continue;
                for (const auto& [regionKey, indices] :
                     trajectory.regionSurfaceIndices)
                {
                    (void)indices;
                    visibleRegions.insert(regionKey);
                }
            }
            if (_largeSceneryGeometryEnabled)
            {
                visibleRegions.insert(
                    _activeLargeSceneryRegions.begin(),
                    _activeLargeSceneryRegions.end());
            }

            scene.staticRegions.reserve(visibleRegions.size());
            for (const auto key : visibleRegions)
            {
                auto& packet = _staticRegionPackets[key];
                if (!packet.dirty
                    && packet.reconstructionSelectionStamp
                        != CurrentReconstructionSelectionStamp(
                            packet.reconstructionGroups))
                {
                    packet.dirty = true;
                }
                if (packet.dirty)
                    RebuildStaticRegionPacket(key, frame);
                packet.lastSeen = frame;
                if (packet.surfaces.empty() || !frustum.visible(packet.center, packet.radius))
                    continue;
                scene.staticRegions.push_back({
                    key, _sceneEpoch, packet.generation,
                    packet.center, packet.radius,
                    &packet.surfaces,
                    &packet.textureDependencies
                });
            }
        }

        // Reconstructing original scenery sprites is relatively expensive: the
        // native painter handles track, supports, multi-tile scenery, animation
        // frames, glass and object remapping. Run it ONLY for tiles whose
        // authoritative source changed (or whose dynamic art needs an update).
        // EntityPaintSetup still runs EVERY frame so guests and cars keep moving.
        void CollectPaintSprites(FirstPersonScene& scene, Drawing::RenderTarget& rt)
        {
            PROFILED_FUNCTION();
            const auto& opt = scene.options;
            const auto basis = GetFirstPersonBasis(opt.camera);
            const auto frame = _terrainCache.frame;
            const uint32_t sourceGeneration = getGameState().currentTicks;
            constexpr uint64_t kMaxStaticAge = 240;
            std::array<std::unordered_set<uint64_t>, 4> missesByRotation;
            for (auto& misses : missesByRotation)
                misses.reserve(scene.visibleTiles.size() / 16 + 1);
            size_t trackCalibrationPaintBudget = 1;
            const auto& view = scene.resolvedView;
            const FirstPersonFrustum worldFrustum(
                view.camera, view.fieldOfViewDegrees, view.aspect,
                view.nearClip, view.farClip);
            for (const auto tile : scene.visibleTiles)
            {
                const int32_t tx = tile.x / kCoordsXYStep;
                const int32_t ty = tile.y / kCoordsXYStep;
                const auto key = TerrainKey(tx, ty);
                auto& cached = _staticPaintCache[key];

                const bool authoritativeInvalidation =
                    !cached.valid || cached.dirty || cached.viewFlags != opt.viewFlags;
                const bool semanticProbeDue = FirstPersonRefreshDue(
                    key ^ 0x9e3779b97f4a7c15ull, sourceGeneration,
                    cached.lastSemanticGeneration, kMaxStaticAge);
                uint64_t probedSignature = cached.signature;
                bool probedAnimated = cached.animated;
                if (authoritativeInvalidation || semanticProbeDue)
                {
                    // Expensive packed-element hashing and group discovery belong
                    // on invalidation or staggered fallback probes, never the
                    // ordinary frame path.
                    probedSignature = NativeTileSignature(tile);
                    probedAnimated = MapAnimations::IsTileAnimatedForFirstPerson(
                        TileCoordsXY(tx, ty));
                    cached.lastSemanticGeneration = sourceGeneration;
                }
                const bool semanticChanged = authoritativeInvalidation
                    || (semanticProbeDue && probedSignature != cached.signature)
                    || (cached.valid && probedAnimated != cached.animated);
                if (semanticChanged)
                {
                    cached.signature = probedSignature;
                    cached.viewFlags = opt.viewFlags;
                    cached.animated = probedAnimated;
                    cached.valid = true;
                    cached.dirty = false;
                    cached.reconstructionGroups.clear();
                    cached.hasUngroupedResident = false;

                    auto* element = MapGetFirstElementAt(tile);
                    if (element != nullptr)
                    {
                        do
                        {
                            const auto group = GetReconstructionGroup(tile, element);
                            if (!group.has_value())
                                continue;
                            const bool duplicate = std::any_of(
                                cached.reconstructionGroups.begin(), cached.reconstructionGroups.end(),
                                [&](const ReconstructionGroupInfo& existing) {
                                    return existing.key == group->key;
                                });
                            if (!duplicate)
                                cached.reconstructionGroups.push_back(*group);
                        } while (!(element++)->isLastForTile());
                    }

                    for (auto& variant : cached.rotations)
                    {
                        variant.valid = false;
                        variant.lastPainted = 0;
                        variant.lastAnimationGeneration = 0;
                        variant.lastSourceProbeGeneration = 0;
                        variant.residentFingerprint = 0;
                        variant.verticalTunnelHeight = 0xFF;
                        variant.leftTunnels.clear();
                        variant.rightTunnels.clear();
                        variant.residentSurfaces.clear();
                        variant.streamedSurfaces.clear();
                    }
                    MarkStaticRegionDirtyForTile(tx, ty);
                }

                const auto previousRotation = cached.hasSelectedRotation
                    ? std::optional<uint8_t>{ cached.selectedRotation }
                    : std::nullopt;
                const auto tileRotation = PaintRotationForTile(opt.camera, tile, previousRotation);
                if ((!cached.hasSelectedRotation || cached.selectedRotation != tileRotation)
                    && cached.hasUngroupedResident)
                    MarkStaticRegionDirtyForTile(tx, ty);
                cached.selectedRotation = tileRotation;
                cached.hasSelectedRotation = true;
                cached.lastSeen = frame;

                uint8_t rotationMask = uint8_t(1u << tileRotation);
                for (const auto& group : cached.reconstructionGroups)
                {
                    auto& state = _reconstructionRotations[group.key];
                    const auto previous = state.hasSelectedRotation
                        ? std::optional<uint8_t>{ state.selectedRotation }
                        : std::nullopt;
                    const auto selected = PaintRotationForPoint(
                        opt.camera, group.anchor, previous);
                    state.selectedRotation = selected;
                    state.hasSelectedRotation = true;
                    state.lastSeen = frame;
                    rotationMask |= uint8_t(1u << selected);

                    if (trackCalibrationPaintBudget > 0
                        && FirstPersonTrackGroupNeedsCalibrationViews(group))
                    {
                        UpdateFirstPersonTrackProfileCalibration(group);
                        --trackCalibrationPaintBudget;
                    }
                }

                for (uint8_t rotation = 0; rotation < 4; ++rotation)
                {
                    if ((rotationMask & uint8_t(1u << rotation)) == 0)
                        continue;
                    auto& variant = cached.rotations[rotation];
                    const uint64_t refreshKey = key ^ (uint64_t(rotation + 1) << 60);
                    const uint32_t animationGeneration = sourceGeneration;
                    const bool stale = !variant.valid ||
                        (cached.animated && variant.lastAnimationGeneration != animationGeneration) ||
                        FirstPersonRefreshDue(
                            refreshKey, sourceGeneration,
                            variant.lastSourceProbeGeneration, kMaxStaticAge);
                    if (stale)
                    {
                        variant.valid = true;
                        variant.lastPainted = frame;
                        variant.lastAnimationGeneration = animationGeneration;
                        variant.lastSourceProbeGeneration = sourceGeneration;
                        variant.residentSurfaces.clear();
                        variant.streamedSurfaces.clear();
                        missesByRotation[rotation].insert(key);
                        ++scene.staticTilePaints;
                    }
                    else
                    {
                        ++scene.staticTileCacheHits;
                    }
                }
            }

            UpdateLargeSceneryReconstructions(scene, frame);

            struct PaintWorkItem
            {
                CoordsXY position{};
                uint64_t key{};
                bool staticMiss = false;
                EntityId entity = EntityId::GetNull();
            };
            std::array<std::vector<PaintWorkItem>, 4> workByRotation;
            for (const auto tile : scene.visibleTiles)
            {
                const auto key = TerrainKey(tile.x / kCoordsXYStep, tile.y / kCoordsXYStep);
                for (uint8_t rotation = 0; rotation < 4; ++rotation)
                {
                    if (missesByRotation[rotation].contains(key))
                        workByRotation[rotation].push_back({ tile, key, true, EntityId::GetNull() });
                }

            }

            // Separate dynamic hierarchy: rebuilt at simulation cadence, then
            // culled at presentation cadence using current tweened entity bounds.
            RebuildDynamicEntitySpatialCache(sourceGeneration);
            for (const auto& region : _dynamicEntitySpatialCache.regions)
            {
                if (!region.hasBounds || !worldFrustum.visible(region.center, region.radius))
                    continue;
                for (const auto entityId : region.entities)
                {
                    ++scene.dynamicTileQueries;
                    auto* entity = getGameState().entities.tryGetEntity<EntityBase>(entityId);
                    if (entity == nullptr)
                        continue;
                    if (!opt.hiddenEntity.IsNull()
                        && entity->id == opt.hiddenEntity)
                    {
                        const auto* vehicle = entity->as<Vehicle>();
                        if (vehicle == nullptr
                            || GetUsableAttachedVehicleHull(
                                *vehicle, opt.hiddenSeatIndex) == nullptr)
                            continue;
                    }

                    const auto location = entity->getLocation();
                    if (location.x == kLocationNull)
                        continue;
                    const auto visualBounds = EntityVisualBounds(*entity);
                    if (!worldFrustum.visible(visualBounds.center, visualBounds.radius))
                        continue;
                    ++scene.dynamicTilesPainted;

                    auto& state = _entityRotations[entityId.ToUnderlying()];
                    const auto previous = state.hasSelectedRotation
                        ? std::optional<uint8_t>{ state.selectedRotation }
                        : std::nullopt;
                    const uint8_t rotation = PaintRotationForPoint(
                        opt.camera, visualBounds.center, previous);
                    state.selectedRotation = rotation;
                    state.hasSelectedRotation = true;
                    state.lastSeen = frame;
                    workByRotation[rotation].push_back({
                        CoordsXY{ location.x, location.y }.toTileStart(), 0, false, entityId
                    });
                }
            }

            const auto map = getGameState().mapSize;
            auto makeCollectionTarget = [&](uint8_t rotation) {
                int32_t minX = std::numeric_limits<int32_t>::max();
                int32_t minY = std::numeric_limits<int32_t>::max();
                int32_t maxX = std::numeric_limits<int32_t>::min();
                int32_t maxY = std::numeric_limits<int32_t>::min();
                for (const int32_t x : { 0, map.x * kCoordsXYStep })
                for (const int32_t y : { 0, map.y * kCoordsXYStep })
                for (const int32_t z : { -512, 4096 })
                {
                    const auto screen = Translate3DTo2DWithZ(rotation, { x, y, z });
                    minX = std::min(minX, screen.x);
                    maxX = std::max(maxX, screen.x);
                    minY = std::min(minY, screen.y);
                    maxY = std::max(maxY, screen.y);
                }
                Drawing::RenderTarget collection{};
                collection.x = minX - 512;
                collection.y = minY - 512;
                collection.width = maxX - minX + 1024;
                collection.height = maxY - minY + 1024;
                collection.cullingX = collection.x;
                collection.cullingY = collection.y;
                collection.cullingWidth = collection.width;
                collection.cullingHeight = collection.height;
                collection.zoom_level = ZoomLevel{ 0 };
                collection.DrawingEngine = rt.DrawingEngine;
                return collection;
            };

            constexpr size_t kTilesPerPaintSession = 256;
            for (uint8_t rotation = 0; rotation < 4; ++rotation)
            {
                auto& rotationWork = workByRotation[rotation];
                if (rotationWork.empty())
                    continue;
                auto collection = makeCollectionTarget(rotation);

                for (size_t offset = 0; offset < rotationWork.size(); offset += kTilesPerPaintSession)
                {
                    const size_t end = std::min(offset + kTilesPerPaintSession, rotationWork.size());
                    auto* session = PaintSessionAlloc(collection, opt.viewFlags, rotation);
                    if (session == nullptr)
                    {
                        for (const auto key : missesByRotation[rotation])
                            _staticPaintCache[key].rotations[rotation].valid = false;
                        return;
                    }
                    Drawing::ScrollingText::BeginFirstPersonSnapshotCapture();

                    for (size_t i = offset; i < end; ++i)
                    {
                        const auto& item = rotationWork[i];
                        session->CurrentlyDrawnEntity = nullptr;
                        session->CurrentlyDrawnTileElement = nullptr;
                        if (item.staticMiss)
                        {
                            session->CurrentSource = PaintStructSource::tile;
                            TileElementPaintSetup(*session, item.position);

                            // Tunnel data is transient session state and is reset
                            // by the next tile. Preserve it while it is authoritative.
                            auto cacheIt = _staticPaintCache.find(item.key);
                            if (cacheIt != _staticPaintCache.end())
                            {
                                auto& variant = cacheIt->second.rotations[rotation];
                                variant.verticalTunnelHeight = session->VerticalTunnelHeight;
                                variant.leftTunnels.assign(
                                    session->LeftTunnels.begin(), session->LeftTunnels.end());
                                variant.rightTunnels.assign(
                                    session->RightTunnels.begin(), session->RightTunnels.end());

                                if (auto terrainIt = _terrainCache.entries.find(item.key);
                                    terrainIt != _terrainCache.entries.end())
                                {
                                    const bool verticalOpening = FirstPersonVerticalTunnelCutsTerrain(
                                        terrainIt->second.baseZ, session->VerticalTunnelHeight);
                                    if (terrainIt->second.verticalOpening != verticalOpening)
                                    {
                                        terrainIt->second.verticalOpening = verticalOpening;
                                        MarkStaticRegionDirtyForTile(
                                            item.position.x / kCoordsXYStep,
                                            item.position.y / kCoordsXYStep);
                                    }
                                }
                            }
                        }
                        else if (!item.entity.IsNull())
                        {
                            if (auto* entity = getGameState().entities.tryGetEntity<EntityBase>(item.entity);
                                entity != nullptr)
                            {
                                session->CurrentSource = PaintStructSource::entity;
                                session->MapPosition = item.position;
                                EntityPaintSetupEntity(*session, *entity);
                            }
                        }
                    }

                    PaintSessionArrange(*session);
                    std::unordered_set<const TileElement*> emittedPathDecks;
                    for (auto* root = session->PaintHead; root; root = root->NextQuadrantEntry)
                    {
                        const bool dynamic = IsFirstPersonEntityPaintRoot(*root);
                        const uint64_t key = TerrainKey(
                            root->MapPos.x / kCoordsXYStep, root->MapPos.y / kCoordsXYStep);
                        if (!dynamic && !missesByRotation[rotation].contains(key))
                            continue;

                        if (root->Element != nullptr && root->Element->getType() == TileElementType::surface)
                        {
                            const auto sx = std::abs(root->Bounds.x_end - root->Bounds.x);
                            const auto sy = std::abs(root->Bounds.y_end - root->Bounds.y);
                            const auto sz = std::abs(root->Bounds.z_end - root->Bounds.z);
                            if (sz < 8 || std::min(sx, sy) > 4 || std::max(sx, sy) < 16)
                                continue;
                        }

                        const auto fallbackAnchor = Anchor(*root);
                        const auto reconstruction = GetSpriteReconstructionFrame(
                            *root, fallbackAnchor);
                        const auto anchor = reconstruction.anchor;
                        const CoordsXYZ point{
                            int32_t(std::lround(anchor.x)),
                            int32_t(std::lround(anchor.y)),
                            int32_t(std::lround(anchor.z))
                        };
                        const auto isoAnchor = Translate3DTo2DWithZ(rotation, point);
                        const auto startSurface = scene.surfaces.size();
                        const bool emitPathDeck = root->Element != nullptr &&
                            root->Element->getType() == TileElementType::path &&
                            IsPathDeckCarrier(*root) &&
                            emittedPathDecks.insert(root->Element).second;
                        AppendRoot(
                            scene, *root, anchor, basis, isoAnchor,
                            opt.viewFlags, opt.hiddenEntity, opt.hiddenSeatIndex,
                            rotation, emitPathDeck);
                        if (const auto semantic = LargeScenerySemanticBounds(*root); semantic.has_value())
                        {
                            for (size_t i = startSurface; i < scene.surfaces.size(); ++i)
                            {
                                scene.surfaces[i].hasSemanticBounds = true;
                                scene.surfaces[i].semanticCenter = semantic->center;
                                scene.surfaces[i].semanticRadius = semantic->radius;
                            }
                        }

                        if (!dynamic)
                        {
                            const int32_t tx = root->MapPos.x / kCoordsXYStep;
                            const int32_t ty = root->MapPos.y / kCoordsXYStep;
                            const auto region = FirstPersonGpuRegionKey(tx, ty);
                            auto cacheIt = _staticPaintCache.find(key);
                            // Flat-ride tile painters may attach a vehicle for
                            // interaction ownership while producing the structure.
                            // Cache those roots, but refresh them once per sim tick.
                            if (root->Entity != nullptr && cacheIt != _staticPaintCache.end()
                                && !cacheIt->second.animated)
                            {
                                cacheIt->second.animated = true;
                                MarkStaticRegionDirtyForTile(tx, ty);
                            }
                            for (size_t i = startSurface; i < scene.surfaces.size(); ++i)
                            {
                                scene.surfaces[i].reconstructionGroup = reconstruction.groupKey;
                                if (!scene.surfaces[i].immutablePixels.empty())
                                {
                                    scene.surfaces[i].gpuRegion = 0;
                                    if (cacheIt != _staticPaintCache.end() && !cacheIt->second.animated)
                                    {
                                        cacheIt->second.animated = true;
                                        MarkStaticRegionDirtyForTile(tx, ty);
                                    }
                                }
                                else if (!scene.surfaces[i].viewFacing)
                                {
                                    scene.surfaces[i].gpuRegion = region;
                                }
                            }

                            if (cacheIt != _staticPaintCache.end())
                            {
                                auto& variant = cacheIt->second.rotations[rotation];
                                for (size_t i = startSurface; i < scene.surfaces.size(); ++i)
                                {
                                    auto& surface = scene.surfaces[i];
                                    if (IsResidentStaticSurface(surface))
                                    {
                                        variant.residentSurfaces.push_back(surface);
                                        if (surface.reconstructionGroup == 0)
                                            cacheIt->second.hasUngroupedResident = true;
                                    }
                                    else
                                    {
                                        variant.streamedSurfaces.push_back(surface);
                                    }
                                }
                            }
                            scene.surfaces.erase(
                                scene.surfaces.begin() + startSurface, scene.surfaces.end());
                        }
                        else
                        {
                            const auto first = scene.surfaces.begin() + startSurface;
                            scene.surfaces.erase(
                                std::remove_if(first, scene.surfaces.end(),
                                    [&](const FirstPersonSurface& surface) {
                                        return !SurfaceMayBeVisible(surface, worldFrustum);
                                    }),
                                scene.surfaces.end());
                        }
                    }
                    Drawing::ScrollingText::EndFirstPersonSnapshotCapture();
                    PaintSessionFree(session);
                }
            }

            // Repainting a streamed/animated tile is not a resident-geometry
            // change. For non-animated fallback probes, compare the completed
            // resident output and dirty its packet only if that output changed.
            for (uint8_t rotation = 0;
                 rotation < 4; ++rotation)
            {
                for (const auto key :
                     missesByRotation[rotation])
                {
                    const auto cacheIt =
                        _staticPaintCache.find(key);
                    if (cacheIt
                        == _staticPaintCache.end())
                        continue;
                    auto& cached = cacheIt->second;
                    auto& variant =
                        cached.rotations[rotation];
                    const uint64_t fingerprint =
                        ResidentStaticSurfaceFingerprint(
                            variant.residentSurfaces);
                    if (!cached.animated
                        && variant.residentFingerprint
                            != fingerprint)
                    {
                        MarkStaticRegionDirtyForTile(
                            int32_t(key >> 32),
                            int32_t(
                                key & 0xffffffffu));
                    }
                    variant.residentFingerprint =
                        fingerprint;
                }
            }

            // Only geometry that cannot live in a persistent opaque region is
            // reconstructed at surface granularity on an ordinary frame.
            for (const auto tile : scene.visibleTiles)
            {
                const auto key = TerrainKey(tile.x / kCoordsXYStep, tile.y / kCoordsXYStep);
                const auto cacheIt = _staticPaintCache.find(key);
                if (cacheIt == _staticPaintCache.end())
                    continue;
                const auto& cached = cacheIt->second;
                for (uint8_t rotation = 0; rotation < 4; ++rotation)
                {
                    const auto& variant = cached.rotations[rotation];
                    if (!variant.valid)
                        continue;
                    const auto appendSelected = [&](const std::vector<FirstPersonSurface>& surfaces) {
                        for (const auto& staticSurface : surfaces)
                        {
                            if (IsReconstructedLargeSceneryBody(staticSurface, frame))
                                continue;
                            uint8_t selected = cached.selectedRotation;
                            if (staticSurface.reconstructionGroup != 0)
                            {
                                const auto group = _reconstructionRotations.find(staticSurface.reconstructionGroup);
                                if (group == _reconstructionRotations.end() || !group->second.hasSelectedRotation)
                                    continue;
                                selected = group->second.selectedRotation;
                            }
                            if (selected != rotation)
                                continue;

                            auto surface = staticSurface;
                            if (surface.reconstructionGroup != 0)
                            {
                                const auto trajectory =
                                    _trackTrajectoryCache.find(
                                        surface.reconstructionGroup);
                                if (trajectory != _trackTrajectoryCache.end()
                                    && !trajectory->second.dirty
                                    && trajectory->second.lastSeen == frame)
                                {
                                    ApplyFirstPersonTrackRailArtworkMask(
                                        surface, rotation, trajectory->second);
                                }
                            }
                            ReorientBillboard(surface, opt.camera.position, basis.right);
                            if (SurfaceMayBeVisible(surface, worldFrustum))
                                scene.surfaces.emplace_back(std::move(surface));
                        }
                    };
                    appendSelected(variant.streamedSurfaces);
                    if (cached.animated)
                        appendSelected(variant.residentSurfaces);
                }
            }

            // Cache entries can have been painted in different presentation
            // epochs. Rebase blended surfaces to one unique logical order after
            // deterministic scene assembly so equal-depth peeling always has a
            // complete tie-break. Native-arranged roots preserve their relative
            // order within the assembled stream.
            uint32_t transparentOrdinal = 1;
            for (auto& surface : scene.surfaces)
            {
                if (surface.image.HasValue() && surface.image.IsBlended())
                    surface.nativePaintOrdinal = transparentOrdinal++;
            }

            if (frame % 120 == 0)
            {
                std::erase_if(_staticPaintCache, [frame](const auto& kv) {
                    const bool expired = frame - kv.second.lastSeen > 240;
                    if (expired)
                        MarkStaticRegionDirtyForTile(
                            int32_t(kv.first >> 32), int32_t(kv.first & 0xffffffffu));
                    return expired;
                });
                std::erase_if(_reconstructionRotations, [frame](const auto& kv) {
                    return frame - kv.second.lastSeen > 240;
                });
                std::erase_if(_staticRegionPackets, [frame](const auto& kv) {
                    return frame - kv.second.lastSeen > 240;
                });
                std::erase_if(_entityRotations, [frame](const auto& kv) {
                    return frame - kv.second.lastSeen > 240;
                });
            }

            SubmitVisibleStaticRegions(scene, worldFrustum, frame);
        }

    } // namespace

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
            GetLargeSceneryAssetModel(*entry, true);
        if (model == nullptr || !model->reliable)
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
        const float x = float(tileOrigin.x);
        const float y = float(tileOrigin.y);
        FirstPersonVec3 a{}, b{};
        // Clockwise edge order follows the native wall-slope convention:
        // south->west, west->north, north->east, east->south.
        switch (direction & 3)
        {
            case 0: a = { x, y, float(baseZ) }; b = { x, y + kCoordsXYStep, float(baseZ) }; break;
            case 1: a = { x, y + kCoordsXYStep, float(baseZ) }; b = { x + kCoordsXYStep, y + kCoordsXYStep, float(baseZ) }; break;
            case 2: a = { x + kCoordsXYStep, y + kCoordsXYStep, float(baseZ) }; b = { x + kCoordsXYStep, y, float(baseZ) }; break;
            default: a = { x + kCoordsXYStep, y, float(baseZ) }; b = { x, y, float(baseZ) }; break;
        }
        if ((slope & EDGE_SLOPE_UPWARDS) != 0)
            b.z += 2 * kCoordsZStep;
        else if ((slope & EDGE_SLOPE_DOWNWARDS) != 0)
            a.z += 2 * kCoordsZStep;

        const float wallHeight = float(std::max(0, height));
        FirstPersonWallPlane plane{};
        plane.corners = {
            a,
            b,
            FirstPersonVec3{ b.x, b.y, b.z + wallHeight },
            FirstPersonVec3{ a.x, a.y, a.z + wallHeight },
        };
        return plane;
    }

    std::optional<FirstPersonProjection> ProjectFirstPersonPoint(
        const FirstPersonCamera& c, const FirstPersonVec3& p, const ScreenSize& size,
        float fov, float nearClip)
    {
        return ProjectFirstPersonMath(c, p, size.width, size.height, fov, nearClip);
    }
    FirstPersonScene CollectFirstPersonScene(
        const FirstPersonRenderOptions& opt, const ScreenSize& dimensions)
    {
        FirstPersonScene scene{};
        scene.options = opt;
        scene.dimensions = dimensions;
        const auto map = getGameState().mapSize;
        scene.resolvedView = ResolveFirstPersonView(
            opt.camera, dimensions.width, dimensions.height, map.x, map.y,
            opt.fieldOfViewDegrees, opt.nearClip, opt.farClip);
        scene.options.farClip = scene.resolvedView.farClip;
        // Independent of the overhead paint collector: geometry is derived from live map state.
        const auto visibilityStart=std::chrono::steady_clock::now();
        DiscoverVisibleTiles(scene);
        const auto terrainStart=std::chrono::steady_clock::now();
        scene.visibilityCpuMs=std::chrono::duration<float,std::milli>(terrainStart-visibilityStart).count();
        CollectTerrain(scene);
        CollectTrackTrajectories(scene);
        scene.terrainCpuMs=std::chrono::duration<float,std::milli>(
            std::chrono::steady_clock::now()-terrainStart).count();
        // This remains an explicitly identified compatibility bridge for complex sprite selection.
        return scene;
    }
    void ClearFirstPersonSceneCache()
    {
        ++_sceneEpoch;
        if (_sceneEpoch == 0)
            _sceneEpoch = 1;
        _terrainCache.entries.clear();
        _terrainCache.frame = 0;
        _regionBounds.clear();
        _staticPaintCache.clear();
        _reconstructionRotations.clear();
        _entityRotations.clear();
        _dynamicEntitySpatialCache = {};
        _trackTrajectoryCache.clear();
        _trackTrajectoryGroupsByRegion.clear();
        std::erase_if(
            _trackProfileCalibrations,
            [](const auto& item) {
                return !item.second.profile.verified;
            });
        for (auto& [style, state] :
             _trackProfileCalibrations)
        {
            (void)style;
            state.phase =
                FirstPersonTrackProfileCalibrationState::
                    Phase::verifySource;
            state.sourceArtwork = {};
            state.sourceTrajectory = {};
            state.search = {};
        }
        ClearFirstPersonVerifiedTrackProfiles();
        _largeSceneryAssetModels.clear();
        _largeSceneryGeometryCache.clear();
        ClearFirstPersonLargeSceneryPhysicalProxies();
        _largeSceneryGroupsByRegion.clear();
        _activeLargeSceneryRegions.clear();
        _largeSceneryGeometryEnabled = false;
        _staticRegionPackets.clear();
    }
    void InvalidateFirstPersonSceneRegion(CoordsXY low, CoordsXY high)
    {
        ClearFirstPersonLargeSceneryPhysicalProxies();
        if (_regionBounds.empty() && _terrainCache.entries.empty()
            && _staticPaintCache.empty() && _trackTrajectoryCache.empty()
            && _largeSceneryGeometryCache.empty() && _staticRegionPackets.empty()) return;
        const auto floorTile = [](int32_t x) {return int32_t(std::floor(float(x)/kCoordsXYStep));};
        const auto x0 = floorTile(std::min(low.x,high.x));
        const auto y0 = floorTile(std::min(low.y,high.y));
        const auto x1 = floorTile(std::max(low.x,high.x));
        const auto y1 = floorTile(std::max(low.y,high.y));
        for (int32_t regionY = y0 / 32; regionY <= y1 / 32; ++regionY)
        for (int32_t regionX = x0 / 32; regionX <= x1 / 32; ++regionX)
            _staticRegionPackets[
                FirstPersonGpuRegionKey(regionX * 32, regionY * 32)].dirty = true;
        for (auto& [key, entry] : _regionBounds)
        {
            const int32_t originX = int32_t(key >> 32);
            const int32_t originY = int32_t(key & 0xffffffffu);
            if (x0 < entry.x1 && x1 >= originX && y0 < entry.y1 && y1 >= originY)
                entry.dirty = true;
        }
        for (auto& [key, terrain] : _terrainCache.entries)
        {
            const int32_t tx = int32_t(key >> 32);
            const int32_t ty = int32_t(key & 0xffffffffu);
            if (tx >= x0 && tx <= x1 && ty >= y0 && ty <= y1)
                terrain.dirty = true;
        }
        for (auto& [key,cached] : _staticPaintCache)
        {
            const int32_t tx=int32_t(key>>32);
            const int32_t ty=int32_t(key&0xffffffffu);
            if (tx>=x0 && tx<=x1 && ty>=y0 && ty<=y1)
            {
                cached.dirty = true;
                cached.visibilityDirty = true;
            }
        }
        for (auto& [groupKey, trajectory] : _trackTrajectoryCache)
        {
            (void)groupKey;
            if (!trajectory.hasBounds)
                continue;
            if (x0 <= trajectory.maxTileX && x1 >= trajectory.minTileX
                && y0 <= trajectory.maxTileY && y1 >= trajectory.minTileY)
            {
                MarkTrackTrajectoryRegionsDirty(trajectory);
                trajectory.dirty = true;
            }
        }
        for (auto& [groupKey, geometry] : _largeSceneryGeometryCache)
        {
            (void)groupKey;
            if (!geometry.hasBounds)
                continue;
            if (x0 <= geometry.maxTileX && x1 >= geometry.minTileX
                && y0 <= geometry.maxTileY && y1 >= geometry.minTileY)
            {
                MarkLargeSceneryGeometryRegionsDirty(geometry);
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
    void RenderFirstPerson(Drawing::RenderTarget& rt, const FirstPersonRenderOptions& opt)
    {
        PROFILED_FUNCTION();
        const ScreenSize dimensions{ rt.width, rt.height };
        if (dimensions.width <= 0 || dimensions.height <= 0 || rt.DrawingEngine == nullptr) return;
        const auto start = std::chrono::steady_clock::now();
        auto scene = CollectFirstPersonScene(opt, dimensions);
        const auto paintStart=std::chrono::steady_clock::now();
        CollectPaintSprites(scene, rt);
        const auto submitStart=std::chrono::steady_clock::now();
        scene.paintCpuMs=std::chrono::duration<float,std::milli>(submitStart-paintStart).count();
        scene.prepareCpuMs=std::chrono::duration<float,std::milli>(submitStart-start).count();
        auto* context = rt.DrawingEngine->GetDrawingContext();
        if (context != nullptr)
        {
            context->DrawFirstPersonScene(rt, scene);
        }
    }
} // namespace OpenRCT2::Paint

