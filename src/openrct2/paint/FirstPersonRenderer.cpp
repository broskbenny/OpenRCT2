/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#include "FirstPersonRenderer.h"
#include "Paint.h"
#include "tile_element/Paint.Surface.h"
#include "tile_element/Paint.Path.h"
#include "tile_element/Paint.TileElement.h"
#include "Paint.Entity.h"

#include "../Context.h"
#include "../GameState.h"
#include "../drawing/Drawing.Sprite.h"
#include "../drawing/IDrawingContext.h"
#include "../drawing/IDrawingEngine.h"
#include "../drawing/RenderTarget.h"
#include "../drawing/ScrollingText.h"
#include "../entity/EntityBase.h"
#include "../ride/CarEntry.h"
#include "../ride/Vehicle.h"
#include "../interface/Viewport.h"
#include "../profiling/Profiling.h"
#include "../world/Footpath.h"
#include "../world/Map.h"
#include "../world/MapAnimation.h"
#include "../world/Wall.h"
#include "../ride/Track.h"
#include "../world/tile_element/SurfaceElement.h"
#include "../world/tile_element/PathElement.h"
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
                return ReconstructionGroupInfo{ key, anchor };
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
                return ReconstructionGroupInfo{ key, anchor };
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

        // OpenRCT2 paints walls and exposed land edges using narrow, tall world-space
        // bounding boxes. Those are actual geometric constraints, unlike the bounds of
        // a tree or house. Project the original sprite onto that plane rather than
        // rotating the whole image to face the passenger. Doors with tiny split
        // bounds intentionally retain billboards: their two parts are not a full wall.
        bool AppendPhysicalPlane(
            FirstPersonScene& scene, const PaintStruct& ps, ImageId image,
            const ScreenCoordsXY& spritePos, uint8_t rotation, ImageId mask = {})
        {
            const auto layout = GetSpriteCompositeLayout(image, mask);
            if (!layout.has_value()) return false;
            const float x0 = float(std::min(ps.Bounds.x, ps.Bounds.x_end));
            const float x1 = float(std::max(ps.Bounds.x, ps.Bounds.x_end));
            const float y0 = float(std::min(ps.Bounds.y, ps.Bounds.y_end));
            const float y1 = float(std::max(ps.Bounds.y, ps.Bounds.y_end));
            const float z0 = float(std::min(ps.Bounds.z, ps.Bounds.z_end));
            const float z1 = float(std::max(ps.Bounds.z, ps.Bounds.z_end));
            const float sx = x1 - x0, sy = y1 - y0;
            // Never infer a solid slab from a general paint/occlusion box.
            if (!(z1 - z0 >= 8.0f &&
                  ((sx >= 16.0f && sy <= 4.0f) || (sy >= 16.0f && sx <= 4.0f))))
                return false;
            const bool alongX = sx >= sy;
            const float fixed = alongX ? 0.5f*(y0+y1) : 0.5f*(x0+x1);
            const std::array<FirstPersonVec3, 4> world = alongX
                ? std::array<FirstPersonVec3,4>{{ {x0,fixed,z0},{x1,fixed,z0},
                                                 {x1,fixed,z1},{x0,fixed,z1} }}
                : std::array<FirstPersonVec3,4>{{ {fixed,y0,z0},{fixed,y1,z0},
                                                 {fixed,y1,z1},{fixed,y0,z1} }};
            FirstPersonSurface surface{};
            surface.image = image;
            surface.mask = mask;
            std::array<FirstPersonVertex,4> v{};
            for (size_t i = 0; i < v.size(); ++i)
            {
                const auto& p = world[i];
                const CoordsXYZ loc{ int32_t(std::lround(p.x)),
                                     int32_t(std::lround(p.y)),
                                     int32_t(std::lround(p.z)) };
                const auto iso = Translate3DTo2DWithZ(rotation, loc);
                v[i] = { p, float(iso.x - spritePos.x - layout->xOffset),
                            float(iso.y - spritePos.y - layout->yOffset) };
            }
            EmitQuad(surface,v);
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
        void AppendRoot(
            FirstPersonScene& scene, const PaintStruct& ps, const FirstPersonVec3& anchor,
            const FirstPersonBasis& basis, const ScreenCoordsXY& isoAnchor,
            uint32_t viewFlags, EntityId hidden, uint8_t rotation, bool emitPathDeck)
        {
            if (ps.Entity != nullptr && !hidden.IsNull() && ps.Entity->id == hidden)
                return;
            const auto visibility = GetPaintStructVisibility(&ps, viewFlags);
            if (visibility == VisibilityKind::hidden)
                return;
            const auto colourify = [&](ImageId id) {
                return visibility == VisibilityKind::partial
                    ? id.WithTransparency(Drawing::FilterPaletteID::paletteDarken1)
                    : id;
            };
            const bool physicallyPlanar = ps.Element != nullptr &&
                (ps.Element->getType() == TileElementType::wall ||
                 ps.Element->getType() == TileElementType::surface);
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

            // Native path surface sprites are represented by the semantic deck
            // above. Bridge/support sprites remain artwork and are never
            // flattened into the walking plane.
            if (!groundPathArtwork)
            {
                const auto surfaceStart = scene.surfaces.size();
                if (!AppendSemanticWallPlane(scene, ps, colourify(ps.image_id), ps.ScreenPos, rotation) &&
                    (!physicallyPlanar ||
                     !AppendPhysicalPlane(scene, ps, colourify(ps.image_id), ps.ScreenPos, rotation)))
                {
                    AppendLayer(
                        scene, anchor, basis, isoAnchor, colourify(ps.image_id), ps.ScreenPos);
                }
                if (scene.surfaces.size() > surfaceStart)
                    ApplyImmutablePaintSnapshot(scene.surfaces.back(), ps.FirstPersonSnapshot);
            }
            if (ps.Children != nullptr)
            {
                AppendRoot(scene, *ps.Children, anchor, basis, isoAnchor, viewFlags, hidden, rotation, false);
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
        // Persistent NATIVE PAINT results, separated from dynamic entity sprites.
        // A cached surface never retains a PaintStruct/TileElement pointer: all
        // native session pointers expire immediately after PaintSessionFree.
        struct StaticPaintRotationCache
        {
            uint64_t lastPainted{};
            uint32_t lastAnimationGeneration{};
            uint32_t lastSourceProbeGeneration{};
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
            float halfWidth = std::max(1.0f, float(entity.spriteData.width));
            float verticalExtent = std::max(
                float(entity.spriteData.heightMin), float(entity.spriteData.heightMax));
            if (const auto* vehicle = entity.as<Vehicle>(); vehicle != nullptr)
            {
                if (const auto* entry = vehicle->Entry(); entry != nullptr)
                {
                    halfWidth = std::max(halfWidth, float(entry->spriteWidth));
                    verticalExtent = std::max(
                        verticalExtent,
                        float(std::max(entry->spriteHeightNegative, entry->spriteHeightPositive)));
                }
            }
            return {
                { float(worldLoc.x), float(worldLoc.y), float(worldLoc.z) },
                std::max(32.0f, std::hypot(halfWidth, verticalExtent) + 16.0f)
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
        };
        static std::unordered_map<uint64_t, StaticRegionPacketCache> _staticRegionPackets;

        [[nodiscard]] bool IsResidentStaticSurface(const FirstPersonSurface& surface)
        {
            return surface.gpuRegion != 0 && !surface.viewFacing
                && surface.image.HasValue() && !surface.image.IsBlended()
                && surface.immutablePixels.empty();
        }

        void MarkStaticRegionDirtyForTile(int32_t tileX, int32_t tileY)
        {
            _staticRegionPackets[FirstPersonGpuRegionKey(tileX, tileY)].dirty = true;
        }

        struct TileSemanticSnapshot
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
            const auto& opt = scene.options;
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
                    if (terrainChanged)
                    {
                        MarkStaticRegionDirtyForTile(tx, ty);
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

            const int32_t regionX = int32_t(uint32_t(regionKey >> 32)) - 1;
            const int32_t regionY = int32_t(uint32_t(regionKey & 0xffffffffu)) - 1;
            const int32_t x0 = regionX * 32;
            const int32_t y0 = regionY * 32;
            const int32_t x1 = x0 + 32;
            const int32_t y1 = y0 + 32;

            bool haveBounds = false;
            FirstPersonVec3 low{}, high{};
            std::unordered_set<uint32_t> dependencies;
            auto addSurface = [&](const FirstPersonSurface& surface) {
                if (!IsResidentStaticSurface(surface) || surface.gpuRegion != regionKey)
                    return;
                packet.surfaces.push_back(surface);
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
                    for (const auto& surface : variant.residentSurfaces)
                    {
                        uint8_t selected = cached.selectedRotation;
                        if (surface.reconstructionGroup != 0)
                        {
                            const auto group = _reconstructionRotations.find(surface.reconstructionGroup);
                            if (group == _reconstructionRotations.end() || !group->second.hasSelectedRotation)
                                continue;
                            selected = group->second.selectedRotation;
                        }
                        if (selected == rotation)
                            addSurface(surface);
                    }
                }
            }

            packet.textureDependencies.assign(dependencies.begin(), dependencies.end());
            std::sort(packet.textureDependencies.begin(), packet.textureDependencies.end());
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

            scene.staticRegions.reserve(visibleRegions.size());
            for (const auto key : visibleRegions)
            {
                auto& packet = _staticRegionPackets[key];
                if (packet.dirty)
                    RebuildStaticRegionPacket(key, frame);
                packet.lastSeen = frame;
                if (packet.surfaces.empty() || !frustum.visible(packet.center, packet.radius))
                    continue;
                scene.staticRegions.push_back({
                    key, packet.generation, packet.center, packet.radius,
                    &packet.surfaces, &packet.textureDependencies
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
                        MarkStaticRegionDirtyForTile(tx, ty);
                        ++scene.staticTilePaints;
                    }
                    else
                    {
                        ++scene.staticTileCacheHits;
                    }
                }
            }

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
                    if (!opt.hiddenEntity.IsNull() && entity->id == opt.hiddenEntity)
                        continue;

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
                            opt.viewFlags, opt.hiddenEntity, rotation, emitPathDeck);
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
        scene.terrainCpuMs=std::chrono::duration<float,std::milli>(
            std::chrono::steady_clock::now()-terrainStart).count();
        // This remains an explicitly identified compatibility bridge for complex sprite selection.
        return scene;
    }
    void ClearFirstPersonSceneCache()
    {
        _terrainCache.entries.clear();
        _terrainCache.frame = 0;
        _regionBounds.clear();
        _staticPaintCache.clear();
        _reconstructionRotations.clear();
        _entityRotations.clear();
        _dynamicEntitySpatialCache = {};
        _staticRegionPackets.clear();
    }
    void InvalidateFirstPersonSceneRegion(CoordsXY low, CoordsXY high)
    {
        if (_regionBounds.empty() && _terrainCache.entries.empty()
            && _staticPaintCache.empty() && _staticRegionPackets.empty()) return;
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

