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
#include "../entity/EntityBase.h"
#include "../interface/Viewport.h"
#include "../profiling/Profiling.h"
#include "../world/Footpath.h"
#include "../world/Map.h"
#include "../world/MapAnimation.h"
#include "../world/Wall.h"
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
        constexpr float kPi = 3.14159265358979323846f;
        constexpr float kDegToRad = kPi / 180.0f;
        static FirstPersonQualityController _quality;

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
        [[nodiscard]] uint8_t PaintRotationForTile(const FirstPersonCamera& camera, CoordsXY tile)
        {
            // Native sprite direction is a property of VIEWPOINT, not head gaze.
            // Use the passenger's position relative to this tile, so turning the
            // head in place cannot repaint the park or abruptly swap sprite sides.
            float dx = float(tile.x + kCoordsXYStep / 2) - camera.position.x;
            float dy = float(tile.y + kCoordsXYStep / 2) - camera.position.y;
            if (std::hypot(dx, dy) <= 0.1f)
            {
                const auto forward = GetFirstPersonBasis(camera).forward;
                dx = forward.x;
                dy = forward.y;
            }
            float yaw = std::atan2(dy, dx);
            if (yaw < 0.0f) yaw += 2.0f * kPi;
            return static_cast<uint8_t>(std::lround(yaw / (0.5f * kPi))) & 3;
        }
        [[nodiscard]] FirstPersonVec3 FixedSpriteRight(uint8_t rotation)
        {
            // Match the horizontal right axis of the corresponding quarter-turn
            // view. Connected static artwork therefore shares a world-fixed axis
            // instead of each component independently facing the passenger.
            switch (rotation & 3)
            {
                case 0: return { 0.0f, 1.0f, 0.0f };
                case 1: return { -1.0f, 0.0f, 0.0f };
                case 2: return { 0.0f, -1.0f, 0.0f };
                default: return { 1.0f, 0.0f, 0.0f };
            }
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
        [[nodiscard]] bool UsesViewFacingImpostor(const PaintStruct& ps)
        {
            if (ps.Entity != nullptr)
                return true;
            if (ps.Element == nullptr)
                return true;
            if (ps.Element->getType() == TileElementType::smallScenery)
            {
                const auto* small = ps.Element->asSmallScenery();
                const auto* entry = small != nullptr ? small->getEntry() : nullptr;
                // Trees already read well as upright impostors. Architecture,
                // rides, supports, signs and other connected static pieces need
                // a shared world-fixed orientation to keep their corners joined.
                return entry == nullptr || entry->flags.has(SmallSceneryFlag::isTree);
            }
            return false;
        }
        void AppendLayer(
            FirstPersonScene& scene, const FirstPersonVec3& anchor, const FirstPersonBasis& basis,
            const ScreenCoordsXY& isoAnchor, ImageId image, const ScreenCoordsXY& spritePos,
            uint8_t rotation, bool viewFacing, ImageId mask = {})
        {
            const auto layout = GetSpriteCompositeLayout(image, mask);
            if (!layout.has_value())
                return;
            const float left = float(spritePos.x + layout->xOffset - isoAnchor.x);
            const float top = float(spritePos.y + layout->yOffset - isoAnchor.y);
            const auto right = viewFacing
                ? UprightBillboardRight(anchor, scene.options.camera.position, basis.right)
                : FixedSpriteRight(rotation);
            const auto up = FirstPersonVec3{ 0.0f, 0.0f, 1.0f };
            const auto p0 = Add(Add(anchor, Mul(right, left)), Mul(up, -top));
            const auto p1 = Add(p0, Mul(right, float(layout->width)));
            const auto p2 = Add(p1, Mul(up, -float(layout->height)));
            const auto p3 = Add(p0, Mul(up, -float(layout->height)));
            FirstPersonSurface surface{};
            surface.image = image;
            surface.mask = mask;
            surface.viewFacing = viewFacing;
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
            const auto spritePos = Translate3DTo2DWithZ(rotation, { origin, baseZ });
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
                const bool viewFacing = UsesViewFacingImpostor(ps);
                if (!AppendSemanticWallPlane(scene, ps, colourify(ps.image_id), ps.ScreenPos, rotation) &&
                    (!physicallyPlanar ||
                     !AppendPhysicalPlane(scene, ps, colourify(ps.image_id), ps.ScreenPos, rotation)))
                {
                    AppendLayer(
                        scene, anchor, basis, isoAnchor, colourify(ps.image_id), ps.ScreenPos,
                        rotation, viewFacing);
                }
            }
            if (ps.Children != nullptr)
            {
                AppendRoot(scene, *ps.Children, anchor, basis, isoAnchor, viewFlags, hidden, rotation, false);
            }
            else
            {
                const bool viewFacing = UsesViewFacingImpostor(ps);
                for (auto* a = ps.Attached; a != nullptr; a = a->NextEntry)
                {
                    const auto colourImage = colourify(a->IsMasked ? a->ColourImageId : a->image_id);
                    const auto maskImage = a->IsMasked ? a->image_id : ImageId{};
                    const auto position = ps.ScreenPos + a->RelativePos;
                    if (!AppendSemanticWallPlane(scene, ps, colourImage, position, rotation, maskImage) &&
                        (!physicallyPlanar ||
                         !AppendPhysicalPlane(scene, ps, colourImage, position, rotation, maskImage)))
                    {
                        AppendLayer(
                            scene, anchor, basis, isoAnchor, colourImage, position,
                            rotation, viewFacing, maskImage);
                    }
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
            int32_t spriteX{}, spriteY{}, spriteWidth{}, spriteHeight{};
            FirstPersonSurface ground{};
            std::optional<FirstPersonSurface> water;
            std::optional<FirstPersonSurface> waterOverlay;
            ImageId waterMaskImage{}, waterOverlayImage{};
            uint64_t lastSeen{};
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
            bool valid = false;
            std::vector<FirstPersonSurface> surfaces;
        };
        struct StaticPaintCacheEntry
        {
            uint64_t signature{};
            uint64_t lastSeen{};
            uint32_t viewFlags{};
            bool valid = false;
            bool dirty = true;
            bool animated = false;
            // Keep all four native quarter-turn variants. Crossing a viewpoint
            // boundary can paint a variant once without destroying the previous
            // one, so moving back and forth does not thrash the whole park.
            std::array<StaticPaintRotationCache, 4> rotations;
        };
        static std::unordered_map<uint64_t, StaticPaintCacheEntry> _staticPaintCache;

        // Hash the actual packed native tile elements, not only terrain height.
        // This detects direct map mutations even if they bypass the ordinary
        // viewport invalidation path. Ride-wide changes are covered by the
        // explicit native redraw hooks and a bounded refresh interval below.
        uint64_t NativeTileSignature(CoordsXY pos)
        {
            const auto* elem = MapGetFirstElementAt(pos);
            if (elem == nullptr) return 0;
            uint64_t hash = 14695981039346656037ull;
            do
            {
                const auto* bytes = reinterpret_cast<const uint8_t*>(elem);
                for (size_t n=0;n<sizeof(TileElement);++n)
                {
                    hash ^= uint64_t(bytes[n]);
                    hash *= 1099511628211ull;
                }
            } while (!(elem++)->isLastForTile());
            return hash;
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
        static std::unordered_map<uint64_t, uint64_t> _coarseLastUsed;

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

        // Complete-park coverage: partition the authoritative tile map, not a
        // camera-centred square. Region bounds are broad on purpose: the
        // base terrain of a tile cannot safely cull a tall ride on that tile.
        void DiscoverVisibleTiles(FirstPersonScene& scene)
        {
            PROFILED_FUNCTION();
            const auto& opt = scene.options;
            const auto frustum = FirstPersonFrustum(
                opt.camera, opt.fieldOfViewDegrees,
                float(scene.dimensions.width) / float(std::max(1, scene.dimensions.height)),
                opt.nearClip, opt.farClip);
            const auto map = getGameState().mapSize;
            auto visit = [&](int32_t x0, int32_t y0, int32_t x1, int32_t y1) {
                for (int32_t ty = y0; ty < y1; ++ty)
                for (int32_t tx = x0; tx < x1; ++tx)
                {
                    const CoordsXY world{ tx * kCoordsXYStep, ty * kCoordsXYStep };
                    if (!MapIsLocationValid(world)) continue;
                    const auto* element = MapGetFirstElementAt(world);
                    if (element == nullptr) continue;
                    int32_t minZ = 4096;
                    int32_t maxZ = -512;
                    do
                    {
                        minZ = std::min(minZ, element->getBaseZ());
                        maxZ = std::max(maxZ, SemanticClearanceZ(*element));
                        if (element->getType() == TileElementType::surface)
                        {
                            maxZ = std::max(maxZ, element->asSurface()->getWaterHeight());
                        }
                    } while (!(element++)->isLastForTile());
                    if (maxZ < minZ) continue;
                    // Every tile's visible art may overhang its occupancy box.
                    // A conservative margin is preferable to skyline popping.
                    const float halfZ = 0.5f * float(maxZ - minZ);
                    const FirstPersonVec3 center{ float(world.x + 16), float(world.y + 16),
                                                  0.5f * float(minZ + maxZ) };
                    const float radius = std::sqrt(2.0f * 16.0f * 16.0f + halfZ * halfZ) + 256.0f;
                    if (frustum.visible(center, radius))
                        scene.visibleTiles.emplace_back(world);
                }
            };
            auto bounds = [](int32_t x0,int32_t y0,int32_t x1,int32_t y1) {
                return CachedRegionBounds(x0,y0,x1,y1);
            };
            VisitFirstPersonRegions(frustum,0,0,map.x,map.y,visit,bounds);
        }

        // Opaque terrain LOD merges only genuinely level, dry, identical
        // source-art squares. Small cliffs, water, mixed textures and sloped
        // corners cannot silently disappear into a coarse patch. The source
        // image is stretched only when the entire patch is sufficiently small
        // on screen that its repeated native tiles are not distinguishable.
        bool AppendDistantFlatPatch(
            FirstPersonScene& scene, CoordsXY origin, int32_t factor,
            const std::unordered_set<uint64_t>& visible, std::unordered_set<uint64_t>& consumed,
            const FirstPersonFrustum& frustum)
        {
            const auto tx = origin.x / kCoordsXYStep;
            const auto ty = origin.y / kCoordsXYStep;
            if (factor < 2 || tx % factor != 0 || ty % factor != 0)
                return false;
            const float dx = float(factor * kCoordsXYStep);
            const FirstPersonVec3 center{ float(origin.x) + dx * 0.5f,
                                          float(origin.y) + dx * 0.5f, 0.0f };
            const FirstPersonVec3 delta = Sub(center, frustum.eye);
            const float depth = Dot(delta, frustum.basis.forward);
            if (depth <= dx) return false;
            const float focal = float(scene.dimensions.width) * 0.5f /
                frustum.tanHalfHorizontal;
            const uint64_t lodKey = (TerrainKey(tx,ty) << 2) | (factor == 4 ? 2ull : 1ull);
            const auto previous = _coarseLastUsed.find(lodKey);
            const bool recentlySelected = previous != _coarseLastUsed.end() &&
                previous->second + 1 >= _terrainCache.frame;
            const float allowed = scene.activePixelTolerance * (recentlySelected ? 1.20f : 0.80f);
            if (focal * dx / depth > allowed)
                return false;
            ImageId image{};
            int32_t height = 0;
            const G1Element* sprite = nullptr;
            for (int32_t y = 0; y < factor; ++y)
            for (int32_t x = 0; x < factor; ++x)
            {
                const CoordsXY pos{ origin.x + x * kCoordsXYStep, origin.y + y * kCoordsXYStep };
                const auto key = TerrainKey(tx + x, ty + y);
                if (visible.count(key) == 0 || consumed.count(key) != 0)
                    return false;
                auto* surface = MapGetSurfaceElementAt(pos);
                if (surface == nullptr || surface->getSlope() != kTileSlopeFlat || surface->getWaterHeight() > 0)
                    return false;
                const auto selected = GetFirstPersonTerrainImage(*surface, pos);
                if (!selected.HasValue()) return false;
                if (x == 0 && y == 0)
                {
                    image = selected;
                    height = surface->getBaseZ();
                    sprite = GfxGetG1Element(image);
                    if (sprite == nullptr) return false;
                }
                else if (selected != image || surface->getBaseZ() != height)
                    return false;
            }
            FirstPersonSurface patch{};
            patch.image = image;
            patch.edgeCoverage = true;
            patch.gpuRegion = FirstPersonGpuRegionKey(tx,ty);
            const auto native = std::array<CoordsXYZ,4>{{
                { origin.x, origin.y, height },
                { origin.x + kCoordsXYStep, origin.y, height },
                { origin.x + kCoordsXYStep, origin.y + kCoordsXYStep, height },
                { origin.x, origin.y + kCoordsXYStep, height },
            }};
            const auto isoOrigin = Translate3DTo2DWithZ(0, native[0]);
            std::array<FirstPersonVertex,4> quad{};
            for (size_t i = 0; i < quad.size(); ++i)
            {
                const auto projected = Translate3DTo2DWithZ(0, native[i]);
                quad[i] = {{ float(origin.x + (i == 1 || i == 2 ? factor*kCoordsXYStep : 0)),
                             float(origin.y + (i >= 2 ? factor*kCoordsXYStep : 0)), float(height) },
                           float(projected.x - isoOrigin.x - sprite->xOffset),
                           float(projected.y - isoOrigin.y - sprite->yOffset) };
            }
            EmitQuad(patch,quad);
            scene.surfaces.emplace_back(std::move(patch));
            _coarseLastUsed[lodKey] = _terrainCache.frame;
            for (int32_t y = 0; y < factor; ++y)
            for (int32_t x = 0; x < factor; ++x)
                consumed.emplace(TerrainKey(tx+x,ty+y));
            return true;
        }

        void CollectTerrain(FirstPersonScene& scene)
        {
            PROFILED_FUNCTION();
            const auto& opt = scene.options;
            const auto frame = ++_terrainCache.frame;
            const auto frustum = FirstPersonFrustum(
                opt.camera, opt.fieldOfViewDegrees,
                float(scene.dimensions.width) / float(std::max(1, scene.dimensions.height)),
                opt.nearClip, opt.farClip);
            std::unordered_set<uint64_t> visible;
            visible.reserve(scene.visibleTiles.size());
            for (const auto tile : scene.visibleTiles)
                visible.emplace(TerrainKey(tile.x / kCoordsXYStep, tile.y / kCoordsXYStep));
            std::unordered_set<uint64_t> consumed;
            consumed.reserve(visible.size());
            for (const CoordsXY origin : scene.visibleTiles)
            {
                const int32_t tx = origin.x / kCoordsXYStep;
                const int32_t ty = origin.y / kCoordsXYStep;
                const uint64_t key = TerrainKey(tx, ty);
                if (consumed.count(key) != 0) continue;
                if (AppendDistantFlatPatch(scene, origin, 4, visible, consumed, frustum) ||
                    AppendDistantFlatPatch(scene, origin, 2, visible, consumed, frustum))
                    continue;
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
                    const auto image = GetFirstPersonTerrainImage(*tile, origin);
                    const auto* g1 = image.HasValue() ? GfxGetG1Element(image) : nullptr;
                    if (g1 == nullptr) continue;
                    const ImageId waterMask = waterZ > baseZ ? GetFirstPersonWaterMaskImage(*tile) : ImageId{};
                    const ImageId waterOverlay = waterZ > baseZ
                        ? GetFirstPersonWaterOverlayImage(*tile, opt.viewFlags) : ImageId{};
                    auto& cache = _terrainCache.entries[TerrainKey(tx,ty)];
                    if (cache.source != image || cache.waterMaskImage != waterMask ||
                        cache.waterOverlayImage != waterOverlay || cache.baseZ != baseZ || cache.slope != slope ||
                        cache.waterZ != waterZ || cache.spriteX != g1->xOffset ||
                        cache.spriteY != g1->yOffset || cache.spriteWidth != g1->width ||
                        cache.spriteHeight != g1->height)
                    {
                        cache.source = image;
                        cache.baseZ = baseZ;
                        cache.slope = slope;
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
                        // Native image ID and four world corners determine the cached
                        // mapping. A camera move does NOT reproject the source image.
                        const auto isoOrigin = Translate3DTo2DWithZ(0, { origin, baseZ });
                        std::array<FirstPersonVertex, 4> v{};
                        for (size_t i = 0; i < world.size(); ++i)
                        {
                            const CoordsXYZ point{ int32_t(world[i].x), int32_t(world[i].y), int32_t(world[i].z) };
                            const auto src = Translate3DTo2DWithZ(0, point);
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
                                const auto waterIso = Translate3DTo2DWithZ(0, { origin, waterZ });
                                for (size_t i = 0; i < world.size(); ++i)
                                {
                                    const auto p = CoordsXYZ{ int32_t(world[i].x), int32_t(world[i].y), waterZ };
                                    const auto src = Translate3DTo2DWithZ(0, p);
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
                                const auto waterIso = Translate3DTo2DWithZ(0, { origin, waterZ });
                                for (size_t i = 0; i < world.size(); ++i)
                                {
                                    const auto p = CoordsXYZ{ int32_t(world[i].x), int32_t(world[i].y), waterZ };
                                    const auto src = Translate3DTo2DWithZ(0, p);
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
                    scene.surfaces.emplace_back(cache.ground);
                    if (cache.water.has_value()) scene.surfaces.emplace_back(*cache.water);
                    if (cache.waterOverlay.has_value()) scene.surfaces.emplace_back(*cache.waterOverlay);
            }
            // Bound memory after travelling across multiple distant park regions.
            // Never retain a permanently growing copy of an explored park.
            if ((frame % 120) == 0)
            {
                std::erase_if(_terrainCache.entries, [frame](const auto& kv) {
                    return frame - kv.second.lastSeen > 240;
                });
                std::erase_if(_coarseLastUsed, [frame](const auto& kv) {
                    return frame - kv.second > 240;
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
            constexpr uint64_t kMaxStaticAge = 240;
            std::unordered_set<uint64_t> misses;
            misses.reserve(scene.visibleTiles.size() / 8 + 1);
            std::unordered_map<uint64_t, uint8_t> tileRotations;
            tileRotations.reserve(scene.visibleTiles.size());
            const FirstPersonFrustum worldFrustum(
                opt.camera, opt.fieldOfViewDegrees,
                float(scene.dimensions.width) / float(std::max(scene.dimensions.height, 1)),
                opt.nearClip, opt.farClip);

            for (const auto tile : scene.visibleTiles)
            {
                const auto key = TerrainKey(tile.x / kCoordsXYStep, tile.y / kCoordsXYStep);
                const auto rotation = PaintRotationForTile(opt.camera, tile);
                tileRotations.emplace(key, rotation);
                const auto sig = NativeTileSignature(tile);
                auto& cached = _staticPaintCache[key];
                const bool semanticChanged = !cached.valid || cached.dirty ||
                    cached.signature != sig || cached.viewFlags != opt.viewFlags;
                if (semanticChanged)
                {
                    cached.signature = sig;
                    cached.viewFlags = opt.viewFlags;
                    cached.animated = MapAnimations::IsTileAnimatedForFirstPerson(
                        TileCoordsXY(tile.x / kCoordsXYStep, tile.y / kCoordsXYStep));
                    cached.valid = true;
                    cached.dirty = false;
                    for (auto& variant : cached.rotations)
                    {
                        variant.valid = false;
                        variant.lastPainted = 0;
                        variant.surfaces.clear();
                    }
                }
                cached.lastSeen = frame;
                auto& variant = cached.rotations[rotation];
                const uint64_t refreshKey = key ^ (uint64_t(rotation + 1) << 60);
                const bool stale = !variant.valid ||
                    (cached.animated && variant.lastPainted != frame) ||
                    FirstPersonRefreshDue(refreshKey, frame, variant.lastPainted, kMaxStaticAge);
                if (stale)
                {
                    variant.valid = true;
                    variant.lastPainted = frame;
                    variant.surfaces.clear();
                    misses.insert(key);
                    ++scene.staticTilePaints;
                }
                else
                {
                    ++scene.staticTileCacheHits;
                }
            }

            struct WorkTile
            {
                CoordsXY position{};
                uint64_t key{};
                bool staticMiss{};
                bool hasEntities{};
            };
            std::array<std::vector<WorkTile>, 4> workByRotation;
            for (const auto tile : scene.visibleTiles)
            {
                const auto key = TerrainKey(tile.x / kCoordsXYStep, tile.y / kCoordsXYStep);
                const bool hasEntities = !getGameState().entities.getEntityTileList(tile).empty();
                ++scene.dynamicTileQueries;
                const bool staticMiss = misses.contains(key);
                if (staticMiss || hasEntities)
                    workByRotation[tileRotations.at(key)].push_back({ tile, key, staticMiss, hasEntities });
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
                        for (const auto key : misses)
                        {
                            const auto rotationIt = tileRotations.find(key);
                            if (rotationIt != tileRotations.end())
                                _staticPaintCache[key].rotations[rotationIt->second].valid = false;
                        }
                        return;
                    }

                    for (size_t i = offset; i < end; ++i)
                    {
                        const auto& item = rotationWork[i];
                        if (item.staticMiss)
                        {
                            session->CurrentlyDrawnEntity = nullptr;
                            session->CurrentlyDrawnTileElement = nullptr;
                            TileElementPaintSetup(*session, item.position);
                        }
                        if (item.hasEntities)
                        {
                            session->CurrentlyDrawnEntity = nullptr;
                            session->CurrentlyDrawnTileElement = nullptr;
                            EntityPaintSetup(*session, item.position);
                            ++scene.dynamicTilesPainted;
                        }
                    }

                    PaintSessionArrange(*session);
                    std::unordered_set<const TileElement*> emittedPathDecks;
                    for (auto* root = session->PaintHead; root; root = root->NextQuadrantEntry)
                    {
                        const bool dynamic = root->Entity != nullptr;
                        const uint64_t key = TerrainKey(
                            root->MapPos.x / kCoordsXYStep, root->MapPos.y / kCoordsXYStep);
                        if (!dynamic)
                        {
                            const auto expectedRotation = tileRotations.find(key);
                            if (!misses.contains(key) || expectedRotation == tileRotations.end() ||
                                expectedRotation->second != rotation)
                                continue;
                        }
                        if (root->Element != nullptr && root->Element->getType() == TileElementType::surface)
                        {
                            const auto sx = std::abs(root->Bounds.x_end - root->Bounds.x);
                            const auto sy = std::abs(root->Bounds.y_end - root->Bounds.y);
                            const auto sz = std::abs(root->Bounds.z_end - root->Bounds.z);
                            if (sz < 8 || std::min(sx, sy) > 4 || std::max(sx, sy) < 16)
                                continue;
                        }

                        const auto anchor = Anchor(*root);
                        const CoordsXYZ point{
                            int32_t(anchor.x), int32_t(anchor.y), int32_t(anchor.z)
                        };
                        const auto isoAnchor = Translate3DTo2DWithZ(rotation, point);
                        const auto start = scene.surfaces.size();
                        const bool emitPathDeck = root->Element != nullptr &&
                            root->Element->getType() == TileElementType::path &&
                            emittedPathDecks.insert(root->Element).second;
                        AppendRoot(
                            scene, *root, anchor, basis, isoAnchor,
                            opt.viewFlags, opt.hiddenEntity, rotation, emitPathDeck);

                        if (const auto semantic = LargeScenerySemanticBounds(*root); semantic.has_value())
                        {
                            for (size_t i = start; i < scene.surfaces.size(); ++i)
                            {
                                scene.surfaces[i].hasSemanticBounds = true;
                                scene.surfaces[i].semanticCenter = semantic->center;
                                scene.surfaces[i].semanticRadius = semantic->radius;
                            }
                        }

                        if (!dynamic)
                        {
                            const auto region = FirstPersonGpuRegionKey(
                                root->MapPos.x / kCoordsXYStep, root->MapPos.y / kCoordsXYStep);
                            for (size_t i = start; i < scene.surfaces.size(); ++i)
                                if (!scene.surfaces[i].viewFacing)
                                    scene.surfaces[i].gpuRegion = region;

                            auto cacheIt = _staticPaintCache.find(key);
                            if (cacheIt != _staticPaintCache.end())
                            {
                                auto& surfaces = cacheIt->second.rotations[rotation].surfaces;
                                surfaces.insert(
                                    surfaces.end(), scene.surfaces.begin() + start, scene.surfaces.end());
                            }
                            // Static display assembly happens once, in stable
                            // visible-tile order after all misses are captured.
                            scene.surfaces.erase(scene.surfaces.begin() + start, scene.surfaces.end());
                        }
                        else
                        {
                            const auto first = scene.surfaces.begin() + start;
                            scene.surfaces.erase(
                                std::remove_if(first, scene.surfaces.end(),
                                    [&](const FirstPersonSurface& surface) {
                                        return !SurfaceMayBeVisible(surface, worldFrustum);
                                    }),
                                scene.surfaces.end());
                        }
                    }
                    PaintSessionFree(session);
                }
            }

            // Cache hits and freshly painted misses take the SAME deterministic
            // path. This prevents [A,B] -> [B,A] reorder churn from invalidating
            // otherwise identical resident GPU-region fingerprints.
            for (const auto tile : scene.visibleTiles)
            {
                const auto key = TerrainKey(tile.x / kCoordsXYStep, tile.y / kCoordsXYStep);
                const auto cacheIt = _staticPaintCache.find(key);
                if (cacheIt == _staticPaintCache.end())
                    continue;
                const auto rotationIt = tileRotations.find(key);
                if (rotationIt == tileRotations.end())
                    continue;
                const auto& variant = cacheIt->second.rotations[rotationIt->second];
                if (!variant.valid)
                    continue;
                for (const auto& staticSurface : variant.surfaces)
                {
                    auto surface = staticSurface;
                    ReorientBillboard(surface, opt.camera.position, basis.right);
                    if (SurfaceMayBeVisible(surface, worldFrustum))
                        scene.surfaces.emplace_back(std::move(surface));
                }
            }

            if (frame % 120 == 0)
            {
                std::erase_if(_staticPaintCache, [frame](const auto& kv) {
                    return frame - kv.second.lastSeen > 240;
                });
            }
        }

    } // namespace

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
        const auto map = getGameState().mapSize;
        scene.options.farClip=CompleteParkFarClip(opt.camera.position,map.x,map.y,opt.farClip);
        scene.dimensions = dimensions;
        // Independent of the overhead paint collector: geometry is derived from live map state.
        scene.activePixelTolerance = opt.fixedPixelTolerance > 0.0f ? opt.fixedPixelTolerance : _quality.pixelTolerance;
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
        _coarseLastUsed.clear();
        _quality = {};
    }
    void InvalidateFirstPersonSceneRegion(CoordsXY low, CoordsXY high)
    {
        if (_regionBounds.empty() && _terrainCache.entries.empty() && _staticPaintCache.empty()) return;
        const auto floorTile = [](int32_t x) {return int32_t(std::floor(float(x)/kCoordsXYStep));};
        const auto x0 = floorTile(std::min(low.x,high.x));
        const auto y0 = floorTile(std::min(low.y,high.y));
        const auto x1 = floorTile(std::max(low.x,high.x));
        const auto y1 = floorTile(std::max(low.y,high.y));
        for (auto& [key, entry] : _regionBounds)
        {
            const int32_t originX = int32_t(key >> 32);
            const int32_t originY = int32_t(key & 0xffffffffu);
            if (x0 < entry.x1 && x1 >= originX && y0 < entry.y1 && y1 >= originY)
                entry.dirty = true;
        }
        for (auto& [key,cached] : _staticPaintCache)
        {
            const int32_t tx=int32_t(key>>32);
            const int32_t ty=int32_t(key&0xffffffffu);
            if (tx>=x0 && tx<=x1 && ty>=y0 && ty<=y1)
                cached.dirty=true;
        }
        // Do not erase cached terrain on generic isometric invalidation. Native
        // invalidation also happens for shadows, animations and moving objects.
        // CollectTerrain validates actual terrain style, slope, ground height,
        // water state and G1 sprite dimensions against live map data BEFORE
        // reusing the surface. Clearing every time would defeat static caching
        // in an animated park. Likewise, retain LOD hysteresis: an edited tile
        // is revalidated for eligibility on its next admission, rather than
        // oscillating because unrelated entities moved nearby.
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
            const float cpuMs = std::chrono::duration<float, std::milli>(
                std::chrono::steady_clock::now() - start).count();
            if (opt.fixedPixelTolerance <= 0.0f)
                _quality.observe(cpuMs, context->GetFirstPersonGpuTimeMs(), opt.targetFrameMs);
        }
    }
} // namespace OpenRCT2::Paint

