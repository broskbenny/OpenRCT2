/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#include "FirstPersonRenderer.h"
#include "FirstPersonAssetReconstruction.h"
#include "FirstPersonLargeSceneryReconstruction.h"
#include "FirstPersonPhysicalProxy.h"
#include "FirstPersonTrackTrajectory.h"
#include "FirstPersonTunnelGeometry.h"
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
        static std::unordered_map<uint16_t, CoordsXY>
            _passengerAnchorSourceTiles;


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
                if (entry == nullptr
                    || sequence >= entry->tiles.size())
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
            if (ps.Element != nullptr)
            {
                // PaintStruct bounds are sorting metadata, not world geometry.
                // Even a genuine fallback billboard is anchored to the
                // authoritative tile element, never to the sorting box.
                return {
                    float(ps.MapPos.x + kCoordsXYHalfTile),
                    float(ps.MapPos.y + kCoordsXYHalfTile),
                    float(ps.Element->getBaseZ()),
                };
            }
            // Only non-world/native paint records without an element or entity
            // lack a stronger anchor.
            return {
                float(ps.MapPos.x + kCoordsXYHalfTile),
                float(ps.MapPos.y + kCoordsXYHalfTile),
                0.0f,
            };
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

            // Grouped fragments share one canonical native source frame.
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
        // into a conservative sphere for view admission; reconstruction itself
        // uses occupancy-defined faces with native sprites only as artwork evidence.
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

        // Cull the final physical surface, not painter sorting bounds.
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
            GetUsableVehicleHull(const Vehicle& vehicle)
        {
            const auto visual =
                ResolveVehicleVisualState(vehicle);
            const auto* entry = visual.carEntry;
            if (entry == nullptr)
                return nullptr;
            return GetFirstPersonVehicleBodyHull(*entry);
        }

        [[nodiscard]] const FirstPersonVehicleBodyHull*
            GetUsableAttachedVehicleHull(
                const Vehicle& vehicle, uint8_t seatIndex)
        {
            if (seatIndex == 0xFF)
                return nullptr;
            const auto visual =
                ResolveVehicleVisualState(vehicle);
            const auto* entry = visual.carEntry;
            const auto* hull =
                GetUsableVehicleHull(vehicle);
            if (entry == nullptr || hull == nullptr)
                return nullptr;
            const auto* seat =
                GetFirstPersonPassengerAssetSeat(*entry, seatIndex);
            // The body model can be perfectly usable for rendering while still
            // being unsuitable for a camera attachment whose calibrated eye is
            // embedded in the reconstructed shell.
            if (seat == nullptr
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

        void AppendVehicleHull(
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
            (void)spritePos;
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
                + uint32_t(
                    FirstPersonSmallSceneryWitherStage(
                        *entry, *small))
                    * 4u;
            if (image.GetIndex() != expectedBodyImage)
                return false;

            const auto* hull =
                GetFirstPersonSmallSceneryVisualHull(
                    *entry, *small);
            if (hull == nullptr)
                return false;
            const auto faces =
                BuildFirstPersonVisualHullBoundaryFaces(*hull);
            if (faces.empty())
                return false;

            const auto faceVisible =
                [](FirstPersonVec3 normal,
                    uint8_t sourceRotation) {
                    if (normal.z > 0.5f)
                        return true;
                    if (normal.z < -0.5f)
                        return false;
                    return FirstPersonFaceVisibleFromNativeView(
                        {
                            int32_t(std::lround(normal.x)),
                            int32_t(std::lround(normal.y)),
                        },
                        sourceRotation);
                };

            // Ownership is computed independently of texture fit. A projective
            // source is trustworthy only when this physical face is actually
            // frontmost in that native isometric view.
            std::array<FirstPersonDepthOwnerMap, 4>
                depthOwners{};
            for (uint8_t sourceRotation = 0;
                 sourceRotation < 4; ++sourceRotation)
            {
                for (size_t faceIndex = 0;
                     faceIndex < faces.size(); ++faceIndex)
                {
                    const auto& face = faces[faceIndex];
                    if (!faceVisible(
                            face.normal, sourceRotation))
                        continue;
                    std::array<ScreenCoordsXY, 4> screen{};
                    std::array<float, 4> depth{};
                    for (size_t i = 0;
                         i < face.corners.size(); ++i)
                    {
                        const CoordsXYZ point{
                            int32_t(std::lround(
                                face.corners[i].x)),
                            int32_t(std::lround(
                                face.corners[i].y)),
                            int32_t(std::lround(
                                face.corners[i].z)),
                        };
                        screen[i] =
                            Translate3DTo2DWithZ(
                                sourceRotation, point);
                        depth[i] =
                            FirstPersonIsoDepth(
                                sourceRotation, point);
                    }
                    AddFirstPersonDepthTriangle(
                        depthOwners[sourceRotation],
                        uint32_t(faceIndex),
                        { screen[0], screen[1], screen[2] },
                        { depth[0], depth[1], depth[2] });
                    AddFirstPersonDepthTriangle(
                        depthOwners[sourceRotation],
                        uint32_t(faceIndex),
                        { screen[0], screen[2], screen[3] },
                        { depth[0], depth[2], depth[3] });
                }
            }

            struct SmallSceneryFaceView
            {
                ImageId image{};
                const G1Element* g1 = nullptr;
                uint8_t rotation = 0;
                std::vector<uint8_t> pixels;
                float ownership = 0.0f;
                float sourceCoverage = 0.0f;
                float score = 0.0f;
            };

            const auto edgeLength =
                [](FirstPersonVec3 a, FirstPersonVec3 b) {
                    const float dx = b.x - a.x;
                    const float dy = b.y - a.y;
                    const float dz = b.z - a.z;
                    return std::sqrt(
                        dx * dx + dy * dy + dz * dz);
                };
            const auto facePoint =
                [](const std::array<FirstPersonVec3, 4>& face,
                   float s, float t) {
                    const auto lerp =
                        [](FirstPersonVec3 a, FirstPersonVec3 b,
                           float alpha) {
                            return FirstPersonVec3{
                                a.x + (b.x - a.x) * alpha,
                                a.y + (b.y - a.y) * alpha,
                                a.z + (b.z - a.z) * alpha,
                            };
                        };
                    return lerp(
                        lerp(face[0], face[1], s),
                        lerp(face[3], face[2], s), t);
                };
            const auto ownedNear =
                [&](uint8_t sourceRotation,
                    uint32_t owner, int32_t x, int32_t y) {
                    for (int32_t dy = -1; dy <= 1; ++dy)
                    for (int32_t dx = -1; dx <= 1; ++dx)
                    {
                        const auto found =
                            depthOwners[sourceRotation].find(
                                FirstPersonSilhouettePixelKey(
                                    x + dx, y + dy));
                        if (found
                                != depthOwners[sourceRotation].end()
                            && found->second.owner == owner)
                            return true;
                    }
                    return false;
                };

            const float tileX = float(ps.MapPos.x);
            const float tileY = float(ps.MapPos.y);
            const float baseZ = float(small->getBaseZ());
            for (size_t faceIndex = 0;
                 faceIndex < faces.size(); ++faceIndex)
            {
                const auto& face = faces[faceIndex];
                std::vector<SmallSceneryFaceView> candidates;
                for (const auto& view : hull->textureViews)
                {
                    const auto sourceImage =
                        image.WithIndex(view.image);
                    const auto* g1 =
                        GfxGetG1Element(sourceImage);
                    if (g1 == nullptr
                        || g1->width <= 0
                        || g1->height <= 0
                        || !faceVisible(
                            face.normal,
                            view.imageDirection))
                        continue;

                    const auto decoded =
                        DecodeFirstPersonSpritePixels(*g1);
                    if (!decoded.has_value())
                        continue;

                    std::array<ScreenCoordsXY, 4> projected{};
                    for (size_t i = 0;
                         i < face.corners.size(); ++i)
                    {
                        projected[i] =
                            Translate3DTo2DWithZ(
                                view.imageDirection,
                                {
                                    int32_t(std::lround(
                                        face.corners[i].x)),
                                    int32_t(std::lround(
                                        face.corners[i].y)),
                                    int32_t(std::lround(
                                        face.corners[i].z)),
                                });
                    }
                    FirstPersonSilhouette silhouette{};
                    AddFirstPersonSilhouetteQuad(
                        silhouette, projected);
                    const float ownership =
                        FirstPersonDepthOwnerCoverage(
                            depthOwners[
                                view.imageDirection],
                            uint32_t(faceIndex),
                            silhouette);
                    if (ownership < 0.20f)
                        continue;

                    const auto paintOffset =
                        FirstPersonSmallSceneryPaintOffset(
                            *entry, *small,
                            view.imageDirection);
                    const auto sourceOrigin =
                        Translate3DTo2DWithZ(
                            view.imageDirection,
                            { paintOffset, 0 });
                    const float sourceCoverage =
                        FirstPersonSilhouettePredicateCoverage(
                            silhouette,
                            [&](int32_t x, int32_t y) {
                                return FirstPersonVisualHullPixelOpaque(
                                    *g1,
                                    x - sourceOrigin.x
                                        - g1->xOffset,
                                    y - sourceOrigin.y
                                        - g1->yOffset);
                            });
                    if (sourceCoverage <= 0.0f)
                        continue;

                    const bool reliable =
                        FirstPersonTextureReprojectionIsReliable(
                            ownership, sourceCoverage);
                    candidates.push_back({
                        sourceImage,
                        g1,
                        view.imageDirection,
                        *decoded,
                        ownership,
                        sourceCoverage,
                        (reliable ? 1000.0f : 0.0f)
                            + ownership * 4.0f
                            + sourceCoverage
                            + float(silhouette.size()) * 0.001f,
                    });
                }

                std::sort(
                    candidates.begin(), candidates.end(),
                    [](const SmallSceneryFaceView& a,
                       const SmallSceneryFaceView& b) {
                        if (a.score != b.score)
                            return a.score > b.score;
                        return a.rotation < b.rotation;
                    });

                FirstPersonSurface surface{};
                surface.physicalCoverage = true;
                surface.cameraIndependent = true;

                if (!candidates.empty())
                {
                    surface.image = candidates.front().image;
                    const int32_t width =
                        std::clamp(
                            int32_t(std::ceil(std::max(
                                edgeLength(
                                    face.corners[0],
                                    face.corners[1]),
                                edgeLength(
                                    face.corners[3],
                                    face.corners[2])))),
                            1, 256);
                    const int32_t height =
                        std::clamp(
                            int32_t(std::ceil(std::max(
                                edgeLength(
                                    face.corners[0],
                                    face.corners[3]),
                                edgeLength(
                                    face.corners[1],
                                    face.corners[2])))),
                            1, 256);

                    std::array<uint32_t, 256> fallbackCounts{};
                    for (const auto pixel :
                         candidates.front().pixels)
                    {
                        if (pixel != 0)
                            ++fallbackCounts[pixel];
                    }
                    uint8_t fallbackPixel = 0;
                    uint32_t fallbackCount = 0;
                    for (size_t i = 1;
                         i < fallbackCounts.size(); ++i)
                    {
                        if (fallbackCounts[i] > fallbackCount)
                        {
                            fallbackPixel = uint8_t(i);
                            fallbackCount = fallbackCounts[i];
                        }
                    }

                    std::vector<uint8_t> pixels(
                        size_t(width) * size_t(height), 0);
                    for (int32_t y = 0; y < height; ++y)
                    for (int32_t x = 0; x < width; ++x)
                    {
                        const auto point =
                            facePoint(
                                face.corners,
                                (float(x) + 0.5f)
                                    / float(width),
                                (float(y) + 0.5f)
                                    / float(height));

                        uint8_t selectedPixel = 0;
                        for (const auto& candidate :
                             candidates)
                        {
                            if (!SameFirstPersonMaterialTemplate(
                                    surface.image,
                                    candidate.image))
                                continue;

                            const auto projected =
                                Translate3DTo2DWithZ(
                                    candidate.rotation,
                                    {
                                        int32_t(std::lround(
                                            point.x)),
                                        int32_t(std::lround(
                                            point.y)),
                                        int32_t(std::lround(
                                            point.z)),
                                    });
                            if (!ownedNear(
                                    candidate.rotation,
                                    uint32_t(faceIndex),
                                    projected.x,
                                    projected.y))
                                continue;

                            const auto paintOffset =
                                FirstPersonSmallSceneryPaintOffset(
                                    *entry, *small,
                                    candidate.rotation);
                            const auto sourceOrigin =
                                Translate3DTo2DWithZ(
                                    candidate.rotation,
                                    { paintOffset, 0 });
                            const int32_t u =
                                projected.x - sourceOrigin.x
                                - candidate.g1->xOffset;
                            const int32_t v =
                                projected.y - sourceOrigin.y
                                - candidate.g1->yOffset;
                            if (u < 0 || v < 0
                                || u >= candidate.g1->width
                                || v >= candidate.g1->height)
                                continue;

                            const uint8_t pixel =
                                candidate.pixels[
                                    size_t(v)
                                        * size_t(
                                            candidate.g1->width)
                                    + size_t(u)];
                            if (pixel == 0)
                                continue;
                            selectedPixel = pixel;
                            break;
                        }

                        if (selectedPixel == 0)
                            selectedPixel = fallbackPixel;
                        pixels[
                            size_t(y) * size_t(width)
                            + size_t(x)] =
                            selectedPixel;
                    }

                    surface.immutablePixels =
                        std::move(pixels);
                    surface.immutableWidth =
                        int16_t(width);
                    surface.immutableHeight =
                        int16_t(height);
                    surface.persistentBitmap = true;

                    uint64_t fingerprint =
                        14695981039346656037ull;
                    ExtendStableKey(
                        fingerprint,
                        surface.image.GetRemap());
                    ExtendStableKey(
                        fingerprint,
                        surface.image.HasPrimary()
                            ? EnumValue(
                                surface.image.GetPrimary())
                            : 0xffu);
                    ExtendStableKey(
                        fingerprint,
                        surface.image.HasSecondary()
                            ? EnumValue(
                                surface.image.GetSecondary())
                            : 0xffu);
                    ExtendStableKey(
                        fingerprint,
                        surface.image.HasTertiary()
                            ? EnumValue(
                                surface.image.GetTertiary())
                            : 0xffu);
                    ExtendStableKey(fingerprint, width);
                    ExtendStableKey(fingerprint, height);
                    for (const auto pixel :
                         surface.immutablePixels)
                    {
                        fingerprint ^= pixel;
                        fingerprint *= 1099511628211ull;
                    }
                    surface.immutableFingerprint =
                        fingerprint;

                    const std::array<
                        FirstPersonVertex, 4> vertices{ {
                        {
                            { tileX + face.corners[0].x,
                              tileY + face.corners[0].y,
                              baseZ + face.corners[0].z },
                            0.0f, 0.0f,
                        },
                        {
                            { tileX + face.corners[1].x,
                              tileY + face.corners[1].y,
                              baseZ + face.corners[1].z },
                            float(width), 0.0f,
                        },
                        {
                            { tileX + face.corners[2].x,
                              tileY + face.corners[2].y,
                              baseZ + face.corners[2].z },
                            float(width), float(height),
                        },
                        {
                            { tileX + face.corners[3].x,
                              tileY + face.corners[3].y,
                              baseZ + face.corners[3].z },
                            0.0f, float(height),
                        },
                    } };
                    EmitQuad(surface, vertices);
                    scene.surfaces.emplace_back(
                        std::move(surface));
                    continue;
                }

                // Native sprites normally provide no underside observation.
                // Keep the authoritative hull face and use one native material
                // only as a palette/remap fallback; geometry is still exact.
                for (const auto& view : hull->textureViews)
                {
                    const auto sourceImage =
                        image.WithIndex(view.image);
                    const auto* g1 =
                        GfxGetG1Element(sourceImage);
                    if (g1 == nullptr
                        || g1->width <= 0
                        || g1->height <= 0)
                        continue;
                    surface.image = sourceImage;
                    surface.textureFallbackOnly = true;
                    break;
                }
                if (!surface.image.HasValue())
                    return false;

                std::array<FirstPersonVertex, 4> vertices{};
                for (size_t i = 0;
                     i < vertices.size(); ++i)
                {
                    const auto& point = face.corners[i];
                    vertices[i].world = {
                        tileX + point.x,
                        tileY + point.y,
                        baseZ + point.z,
                    };
                }
                EmitQuad(surface, vertices);
                scene.surfaces.emplace_back(
                    std::move(surface));
            }
            return true;
        }

        [[nodiscard]] bool IsHiddenPassengerTileComponent(
            const PaintStruct& ps, EntityId hiddenEntity, uint8_t hiddenSeatIndex)
        {
            return ps.Source == PaintStructSource::tile
                && ps.Entity != nullptr
                && !hiddenEntity.IsNull()
                && ps.Entity->id == hiddenEntity
                && hiddenSeatIndex < 32
                && (ps.FirstPersonPassengerSeatMask
                    & (uint32_t{ 1 }
                        << hiddenSeatIndex))
                    != 0;
        }

        void AppendRoot(
            FirstPersonScene& scene, const PaintStruct& ps, const FirstPersonVec3& anchor,
            const FirstPersonBasis& basis, const ScreenCoordsXY& isoAnchor,
            uint32_t viewFlags, EntityId hidden, uint8_t hiddenSeatIndex,
            uint8_t rotation)
        {
            const bool matchesHiddenEntity =
                ps.Entity != nullptr && !hidden.IsNull()
                && ps.Entity->id == hidden;
            const auto* entityVehicle =
                ps.Source == PaintStructSource::entity
                    && ps.Entity != nullptr
                ? ps.Entity->as<Vehicle>() : nullptr;
            const auto* vehicleHull =
                entityVehicle != nullptr
                ? GetUsableVehicleHull(*entityVehicle)
                : nullptr;
            const auto* attachedVehicle =
                matchesHiddenEntity ? entityVehicle : nullptr;
            const auto* attachedHull =
                attachedVehicle != nullptr
                ? GetUsableAttachedVehicleHull(
                    *attachedVehicle, hiddenSeatIndex)
                : nullptr;
            const bool reconstructAttachedVehicle =
                attachedVehicle != nullptr && attachedHull != nullptr;
            const bool reconstructVehicleBody =
                entityVehicle != nullptr && vehicleHull != nullptr;
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
                // Shared rider sprites can own a pair of seats. Remove only
                // the selected rider's remap pixels and preserve the adjacent
                // passenger, then continue the native paint chain.
                const auto riderVisibility =
                    GetPaintStructVisibility(&ps, viewFlags);
                if (riderVisibility != VisibilityKind::hidden
                    && ps.image_id.HasValue())
                {
                    auto riderImage = ps.image_id;
                    if (riderVisibility
                        == VisibilityKind::partial)
                    {
                        riderImage =
                            riderImage.WithTransparency(
                                Drawing::FilterPaletteID::
                                    paletteDarken1);
                    }
                    const auto surfaceStart =
                        scene.surfaces.size();
                    AppendLayer(
                        scene, anchor, basis, isoAnchor,
                        riderImage, ps.ScreenPos);
                    if (scene.surfaces.size() > surfaceStart
                        && !ApplyAdjacentRiderMask(
                            scene.surfaces.back(),
                            ps.image_id,
                            (hiddenSeatIndex & 1u) != 0))
                    {
                        scene.surfaces.pop_back();
                    }
                }
                if (ps.Children != nullptr)
                {
                    AppendRoot(
                        scene, *ps.Children, anchor, basis, isoAnchor,
                        viewFlags, hidden, hiddenSeatIndex, rotation);
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
            bool suppressCurrentImage =
                ps.FirstPersonSemanticRole
                    != FirstPersonPaintSemanticRole::none;
            if (reconstructVehicleBody)
            {
                const auto component =
                    GetAttachedVehicleComponent(
                        *entityVehicle, ps.image_id);
                if (component.role
                    == AttachedVehicleComponentRole::body)
                {
                    AppendVehicleHull(
                        scene, *entityVehicle, *vehicleHull,
                        colourify(ps.image_id));
                    suppressCurrentImage = true;
                }
                else if (matchesHiddenEntity
                    && reconstructAttachedVehicle
                    && component.role
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
                        if (entityVehicle->num_peeps > adjacentSeat)
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
                else if (matchesHiddenEntity
                    && reconstructAttachedVehicle
                    && component.role
                        == AttachedVehicleComponentRole::other)
                {
                    // Unknown close-up components of the attached vehicle can
                    // still intersect the passenger eye. Other vehicles retain
                    // those native components around their reconstructed body.
                    suppressCurrentImage = true;
                }
            }

            if (!suppressCurrentImage)
            {
                const auto surfaceStart = scene.surfaces.size();
                const bool smallPhysical =
                    AppendCalibratedSmallSceneryGeometry(
                        scene, ps, colourify(ps.image_id),
                        ps.ScreenPos, rotation);
                if (!smallPhysical)
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
                    viewFlags, hidden, hiddenSeatIndex, rotation);
            }
            else
            {
                for (auto* a = ps.Attached; a != nullptr; a = a->NextEntry)
                {
                    const auto colourImage = colourify(a->IsMasked ? a->ColourImageId : a->image_id);
                    const auto maskImage = a->IsMasked ? a->image_id : ImageId{};
                    const auto position = ps.ScreenPos + a->RelativePos;
                    const auto surfaceStart = scene.surfaces.size();
                    AppendLayer(
                        scene, anchor, basis, isoAnchor,
                        colourImage, position, maskImage);
                    if (scene.surfaces.size() > surfaceStart)
                        ApplyImmutablePaintSnapshot(scene.surfaces.back(), a->FirstPersonSnapshot);
                }
            }
        }

        // Terrain cache keys include authoritative visible state.
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

        struct FirstPersonPreparedFrameCache
        {
            bool active = false;
            bool valid = false;
            uint64_t serial = 0;
            FirstPersonRenderOptions options{};
            ScreenSize dimensions{};
            ScreenCoordsXY screenOrigin{};
            Drawing::IDrawingEngine* drawingEngine = nullptr;
            FirstPersonScene scene{};
        };
        static FirstPersonPreparedFrameCache _preparedFrame;

        [[nodiscard]] bool SameFirstPersonVec3(
            const FirstPersonVec3& a, const FirstPersonVec3& b)
        {
            return a.x == b.x && a.y == b.y && a.z == b.z;
        }

        [[nodiscard]] bool SameFirstPersonBasis(
            const FirstPersonBasis& a, const FirstPersonBasis& b)
        {
            return SameFirstPersonVec3(a.forward, b.forward)
                && SameFirstPersonVec3(a.right, b.right)
                && SameFirstPersonVec3(a.up, b.up);
        }

        [[nodiscard]] bool SameFirstPersonCamera(
            const FirstPersonCamera& a, const FirstPersonCamera& b)
        {
            return SameFirstPersonVec3(a.position, b.position)
                && a.yaw == b.yaw
                && a.pitch == b.pitch
                && a.roll == b.roll
                && a.hasExplicitBasis == b.hasExplicitBasis
                && (!a.hasExplicitBasis
                    || SameFirstPersonBasis(
                        a.explicitBasis, b.explicitBasis));
        }

        [[nodiscard]] bool SameFirstPersonRenderOptions(
            const FirstPersonRenderOptions& a,
            const FirstPersonRenderOptions& b)
        {
            return SameFirstPersonCamera(a.camera, b.camera)
                && a.hiddenEntity == b.hiddenEntity
                && a.hiddenSeatIndex == b.hiddenSeatIndex
                && a.viewFlags == b.viewFlags
                && a.radiusTiles == b.radiusTiles
                && a.fieldOfViewDegrees == b.fieldOfViewDegrees
                && a.nearClip == b.nearClip
                && a.farClip == b.farClip;
        }
        // Persistent native paint results never retain session pointers.
        struct StaticPaintRotationCache
        {
            uint64_t lastPainted{};
            uint32_t lastAnimationGeneration{};
            uint64_t residentFingerprint{};
            uint64_t semanticFingerprint{};
            bool valid = false;
            bool semanticValid = false;
            uint8_t verticalTunnelHeight = 0xFF;
            std::vector<FirstPersonSurface> residentSurfaces;
            std::vector<FirstPersonSurface> streamedSurfaces;
            std::vector<FirstPersonPaintSemanticComponent>
                semanticComponents;
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
            uint64_t cameraIndependentFingerprint = 0;
            uint64_t semanticSurfaceFingerprint = 0;
            std::vector<ReconstructionGroupInfo> reconstructionGroups;
            std::vector<FirstPersonSurface>
                cameraIndependentResidentSurfaces;
            std::vector<FirstPersonSurface>
                cameraIndependentStreamedSurfaces;
            std::vector<FirstPersonSurface>
                semanticResidentSurfaces;
            std::vector<FirstPersonSurface>
                semanticStreamedSurfaces;
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
            struct ArtworkProjection
            {
                ImageId image{};
                ImageId mask{};
                FirstPersonVec3 anchor{};
                float left = 0.0f;
                float top = 0.0f;
                uint64_t immutableFingerprint = 0;
                std::vector<uint8_t> immutablePixels;
                int16_t immutableWidth = 0;
                int16_t immutableHeight = 0;
                uint8_t rotation = 0;
            };

            uint64_t signature{};
            uint64_t lastSeen{};
            bool dirty = false;
            bool boundaryContinuous = true;
            bool hasBounds = false;
            int32_t minTileX{}, minTileY{}, maxTileX{}, maxTileY{};
            std::array<bool, 4> artworkCaptureAttempted{};
            std::array<FirstPersonSilhouette, 4> railSilhouettes{};
            std::array<std::vector<ArtworkProjection>, 4> artworkProjections{};
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
            result.reserve(model.faces.size());
            constexpr float kCollisionSkin = 0.5f;

            for (const auto& face : model.faces)
            {
                if (face.sequence >= entry.tiles.size())
                    continue;
                const auto& tile = entry.tiles[face.sequence];

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
                        group.anchor.x + float(localXY.x),
                        group.anchor.y + float(localXY.y),
                        group.anchor.z + float(corner.z),
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

                if (high.x - low.x < 0.01f)
                {
                    low.x -= kCollisionSkin;
                    high.x += kCollisionSkin;
                }
                else if (high.y - low.y < 0.01f)
                {
                    low.y -= kCollisionSkin;
                    high.y += kCollisionSkin;
                }
                else
                {
                    low.z -= kCollisionSkin;
                    high.z += kCollisionSkin;
                }
                result.push_back({
                    low, high,
                    FirstPersonPhysicalProxyProvenance::
                        authoritativeLargeSceneryOccupancy,
                    static_cast<uint8_t>(
                        FirstPersonPhysicalProxyCapability::collide),
                    group.key,
                });
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

            const auto faceVisible =
                [](LargeSceneryAssetFaceKind kind,
                   uint8_t nativeDirection) {
                    if (kind == LargeSceneryAssetFaceKind::top)
                        return true;
                    if (kind == LargeSceneryAssetFaceKind::bottom)
                        return false;
                    CoordsXY normal{};
                    switch (kind)
                    {
                        case LargeSceneryAssetFaceKind::minX:
                            normal = { -1, 0 };
                            break;
                        case LargeSceneryAssetFaceKind::maxX:
                            normal = { 1, 0 };
                            break;
                        case LargeSceneryAssetFaceKind::minY:
                            normal = { 0, -1 };
                            break;
                        case LargeSceneryAssetFaceKind::maxY:
                            normal = { 0, 1 };
                            break;
                        default:
                            return false;
                    }
                    return FirstPersonFaceVisibleFromNativeView(
                        normal, nativeDirection);
                };

            std::array<FirstPersonDepthOwnerMap, 4>
                depthOwners{};
            for (uint8_t direction = 0;
                 direction < 4; ++direction)
            {
                for (size_t faceIndex = 0;
                     faceIndex < model.faces.size();
                     ++faceIndex)
                {
                    const auto& face =
                        model.faces[faceIndex];
                    if (!faceVisible(face.kind, direction))
                        continue;
                    std::array<ScreenCoordsXY, 4> screen{};
                    std::array<float, 4> depth{};
                    for (size_t i = 0;
                         i < face.corners.size(); ++i)
                    {
                        const auto& p = face.corners[i];
                        const CoordsXYZ point{
                            int32_t(std::lround(p.x)),
                            int32_t(std::lround(p.y)),
                            int32_t(std::lround(p.z)),
                        };
                        screen[i] =
                            Translate3DTo2DWithZ(
                                direction, point);
                        depth[i] =
                            FirstPersonIsoDepth(
                                direction, point);
                    }
                    AddFirstPersonDepthTriangle(
                        depthOwners[direction],
                        uint32_t(faceIndex),
                        { screen[0], screen[1], screen[2] },
                        { depth[0], depth[1], depth[2] });
                    AddFirstPersonDepthTriangle(
                        depthOwners[direction],
                        uint32_t(faceIndex),
                        { screen[0], screen[2], screen[3] },
                        { depth[0], depth[2], depth[3] });
                }
            }

            const auto edgeLength =
                [](FirstPersonVec3 a, FirstPersonVec3 b) {
                    const float dx = b.x - a.x;
                    const float dy = b.y - a.y;
                    const float dz = b.z - a.z;
                    return std::sqrt(
                        dx * dx + dy * dy + dz * dz);
                };
            const auto facePoint =
                [](const std::array<FirstPersonVec3, 4>& face,
                   float s, float t) {
                    const auto lerp =
                        [](FirstPersonVec3 a, FirstPersonVec3 b,
                           float alpha) {
                            return FirstPersonVec3{
                                a.x + (b.x - a.x) * alpha,
                                a.y + (b.y - a.y) * alpha,
                                a.z + (b.z - a.z) * alpha,
                            };
                        };
                    return lerp(
                        lerp(face[0], face[1], s),
                        lerp(face[3], face[2], s), t);
                };
            const auto ownedNear =
                [&](uint8_t direction, uint32_t owner,
                    int32_t x, int32_t y) {
                    for (int32_t dy = -1; dy <= 1; ++dy)
                    for (int32_t dx = -1; dx <= 1; ++dx)
                    {
                        const auto found =
                            depthOwners[direction].find(
                                FirstPersonSilhouettePixelKey(
                                    x + dx, y + dy));
                        if (found != depthOwners[direction].end()
                            && found->second.owner == owner)
                            return true;
                    }
                    return false;
                };

            bool haveBounds = false;
            FirstPersonVec3 low{}, high{};
            for (size_t faceIndex = 0;
                 faceIndex < model.faces.size();
                 ++faceIndex)
            {
                const auto& face = model.faces[faceIndex];
                if (face.sequence >= entry.tiles.size())
                    continue;
                const auto& tile = entry.tiles[face.sequence];
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

                std::array<FirstPersonVec3, 4>
                    worldFace{};
                FirstPersonVec3 faceCenter{};
                for (size_t i = 0;
                     i < face.corners.size(); ++i)
                {
                    const auto localXY =
                        FirstPersonLargeSceneryPlacedPoint(
                            {
                                tile.offset.x,
                                tile.offset.y,
                            },
                            {
                                face.corners[i].x,
                                face.corners[i].y,
                            },
                            objectDirection);
                    worldFace[i] = {
                        group.anchor.x + float(localXY.x),
                        group.anchor.y + float(localXY.y),
                        group.anchor.z
                            + float(face.corners[i].z),
                    };
                    faceCenter =
                        Add(faceCenter, worldFace[i]);

                    if (!haveBounds)
                    {
                        low = high = worldFace[i];
                        haveBounds = true;
                    }
                    else
                    {
                        low.x = std::min(
                            low.x, worldFace[i].x);
                        low.y = std::min(
                            low.y, worldFace[i].y);
                        low.z = std::min(
                            low.z, worldFace[i].z);
                        high.x = std::max(
                            high.x, worldFace[i].x);
                        high.y = std::max(
                            high.y, worldFace[i].y);
                        high.z = std::max(
                            high.z, worldFace[i].z);
                    }
                }
                faceCenter = Mul(faceCenter, 0.25f);

                struct LargeSceneryFaceView
                {
                    ImageId image{};
                    const G1Element* g1 = nullptr;
                    uint8_t direction = 0;
                    uint8_t sourceRotation = 0;
                    ScreenCoordsXY spritePos{};
                    std::vector<uint8_t> pixels;
                    float score = 0.0f;
                };
                std::vector<LargeSceneryFaceView>
                    candidates;
                for (uint8_t direction = 0;
                     direction < 4; ++direction)
                {
                    if (!faceVisible(face.kind, direction))
                        continue;
                    const ImageIndex imageIndex =
                        entry.image + 4
                        + (ImageIndex(face.sequence) << 2)
                        + direction;
                    const auto sourceImage =
                        imageTemplate.WithIndex(imageIndex);
                    const auto* g1 =
                        GfxGetG1Element(sourceImage);
                    if (g1 == nullptr
                        || g1->width <= 0
                        || g1->height <= 0)
                        continue;
                    const auto decoded =
                        DecodeFirstPersonSpritePixels(*g1);
                    if (!decoded.has_value())
                        continue;

                    std::array<ScreenCoordsXY, 4>
                        projected{};
                    FirstPersonSilhouette silhouette{};
                    for (size_t i = 0;
                         i < face.corners.size(); ++i)
                    {
                        const auto& p = face.corners[i];
                        projected[i] =
                            Translate3DTo2DWithZ(
                                direction,
                                {
                                    int32_t(std::lround(p.x)),
                                    int32_t(std::lround(p.y)),
                                    int32_t(std::lround(p.z)),
                                });
                    }
                    AddFirstPersonSilhouetteQuad(
                        silhouette, projected);
                    const float ownership =
                        FirstPersonDepthOwnerCoverage(
                            depthOwners[direction],
                            uint32_t(faceIndex),
                            silhouette);
                    if (ownership < 0.20f)
                        continue;

                    const uint8_t sourceRotation =
                        FirstPersonViewportRotationForNativeView(
                            objectDirection, direction);
                    const auto spriteOrigin =
                        GetTileElementPaintSpritePosition(
                            tileWorld, sourceRotation);
                    const auto spritePos =
                        Translate3DTo2DWithZ(
                            sourceRotation,
                            { spriteOrigin, tileBaseZ });

                    candidates.push_back({
                        sourceImage,
                        g1,
                        direction,
                        sourceRotation,
                        spritePos,
                        *decoded,
                        ownership
                            * float(
                                std::max<size_t>(
                                    1, silhouette.size())),
                    });
                }
                std::sort(
                    candidates.begin(), candidates.end(),
                    [](const LargeSceneryFaceView& a,
                       const LargeSceneryFaceView& b) {
                        if (a.score != b.score)
                            return a.score > b.score;
                        return a.direction < b.direction;
                    });

                FirstPersonSurface surface{};
                surface.physicalCoverage = true;
                surface.cameraIndependent = true;
                surface.reconstructionGroup = group.key;
                surface.gpuRegion =
                    FirstPersonGpuRegionKey(
                        int32_t(std::floor(
                            faceCenter.x
                            / float(kCoordsXYStep))),
                        int32_t(std::floor(
                            faceCenter.y
                            / float(kCoordsXYStep))));

                if (!candidates.empty())
                {
                    surface.image =
                        candidates.front().image;
                    const int32_t width =
                        std::clamp(
                            int32_t(std::ceil(std::max(
                                edgeLength(
                                    worldFace[0],
                                    worldFace[1]),
                                edgeLength(
                                    worldFace[3],
                                    worldFace[2])))),
                            1, 256);
                    const int32_t height =
                        std::clamp(
                            int32_t(std::ceil(std::max(
                                edgeLength(
                                    worldFace[0],
                                    worldFace[3]),
                                edgeLength(
                                    worldFace[1],
                                    worldFace[2])))),
                            1, 256);

                    std::array<uint32_t, 256>
                        fallbackCounts{};
                    for (const auto pixel :
                         candidates.front().pixels)
                    {
                        if (pixel != 0)
                            ++fallbackCounts[pixel];
                    }
                    uint8_t fallbackPixel = 0;
                    uint32_t fallbackCount = 0;
                    for (size_t i = 1;
                         i < fallbackCounts.size(); ++i)
                    {
                        if (fallbackCounts[i]
                            > fallbackCount)
                        {
                            fallbackPixel =
                                uint8_t(i);
                            fallbackCount =
                                fallbackCounts[i];
                        }
                    }

                    std::vector<uint8_t> pixels(
                        size_t(width)
                            * size_t(height), 0);
                    for (int32_t y = 0;
                         y < height; ++y)
                    for (int32_t x = 0;
                         x < width; ++x)
                    {
                        const auto worldPoint =
                            facePoint(
                                worldFace,
                                (float(x) + 0.5f)
                                    / float(width),
                                (float(y) + 0.5f)
                                    / float(height));
                        const auto localPoint =
                            facePoint(
                                face.corners,
                                (float(x) + 0.5f)
                                    / float(width),
                                (float(y) + 0.5f)
                                    / float(height));

                        uint8_t selectedPixel = 0;
                        for (const auto& candidate :
                             candidates)
                        {
                            if (!SameFirstPersonMaterialTemplate(
                                    surface.image,
                                    candidate.image))
                                continue;

                            const auto localProjected =
                                Translate3DTo2DWithZ(
                                    candidate.direction,
                                    {
                                        int32_t(std::lround(
                                            localPoint.x)),
                                        int32_t(std::lround(
                                            localPoint.y)),
                                        int32_t(std::lround(
                                            localPoint.z)),
                                    });
                            if (!ownedNear(
                                    candidate.direction,
                                    uint32_t(faceIndex),
                                    localProjected.x,
                                    localProjected.y))
                                continue;

                            const auto source =
                                Translate3DTo2DWithZ(
                                    candidate.sourceRotation,
                                    {
                                        int32_t(std::lround(
                                            worldPoint.x)),
                                        int32_t(std::lround(
                                            worldPoint.y)),
                                        int32_t(std::lround(
                                            worldPoint.z)),
                                    });
                            const int32_t u =
                                source.x
                                - candidate.spritePos.x
                                - candidate.g1->xOffset;
                            const int32_t v =
                                source.y
                                - candidate.spritePos.y
                                - candidate.g1->yOffset;
                            if (u < 0 || v < 0
                                || u >= candidate.g1->width
                                || v >= candidate.g1->height)
                                continue;
                            const uint8_t pixel =
                                candidate.pixels[
                                    size_t(v)
                                        * size_t(
                                            candidate.g1->width)
                                    + size_t(u)];
                            if (pixel == 0)
                                continue;
                            selectedPixel = pixel;
                            break;
                        }
                        if (selectedPixel == 0)
                            selectedPixel = fallbackPixel;
                        pixels[
                            size_t(y) * size_t(width)
                            + size_t(x)] =
                            selectedPixel;
                    }

                    surface.immutablePixels =
                        std::move(pixels);
                    surface.immutableWidth =
                        int16_t(width);
                    surface.immutableHeight =
                        int16_t(height);
                    surface.persistentBitmap = true;

                    uint64_t fingerprint =
                        14695981039346656037ull;
                    ExtendStableKey(
                        fingerprint,
                        surface.image.GetRemap());
                    ExtendStableKey(
                        fingerprint,
                        surface.image.HasPrimary()
                            ? EnumValue(
                                surface.image.GetPrimary())
                            : 0xffu);
                    ExtendStableKey(
                        fingerprint,
                        surface.image.HasSecondary()
                            ? EnumValue(
                                surface.image.GetSecondary())
                            : 0xffu);
                    ExtendStableKey(
                        fingerprint,
                        surface.image.HasTertiary()
                            ? EnumValue(
                                surface.image.GetTertiary())
                            : 0xffu);
                    ExtendStableKey(
                        fingerprint, width);
                    ExtendStableKey(
                        fingerprint, height);
                    for (const auto pixel :
                         surface.immutablePixels)
                    {
                        fingerprint ^= pixel;
                        fingerprint *= 1099511628211ull;
                    }
                    surface.immutableFingerprint =
                        fingerprint;

                    const std::array<
                        FirstPersonVertex, 4> vertices{ {
                        { worldFace[0], 0.0f, 0.0f },
                        { worldFace[1],
                          float(width), 0.0f },
                        { worldFace[2],
                          float(width), float(height) },
                        { worldFace[3],
                          0.0f, float(height) },
                    } };
                    EmitQuad(surface, vertices);
                }
                else
                {
                    // Native artwork has no underside view. Preserve the
                    // occupancy/hull face and use a native material only for
                    // stable palette/remap fallback.
                    for (uint8_t direction = 0;
                         direction < 4; ++direction)
                    {
                        const ImageIndex imageIndex =
                            entry.image + 4
                            + (ImageIndex(face.sequence)
                                << 2)
                            + direction;
                        const auto sourceImage =
                            imageTemplate.WithIndex(
                                imageIndex);
                        const auto* g1 =
                            GfxGetG1Element(sourceImage);
                        if (g1 != nullptr
                            && g1->width > 0
                            && g1->height > 0)
                        {
                            surface.image =
                                sourceImage;
                            surface.textureFallbackOnly =
                                true;
                            break;
                        }
                    }
                    if (!surface.image.HasValue())
                        continue;
                    std::array<
                        FirstPersonVertex, 4> vertices{};
                    for (size_t i = 0;
                         i < vertices.size(); ++i)
                        vertices[i].world =
                            worldFace[i];
                    EmitQuad(surface, vertices);
                }

                result.surfaces.emplace_back(
                    std::move(surface));
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

                    const auto* model =
                        GetLargeSceneryAssetModel(
                            *entry);
                    if (model == nullptr
                        || !model->usable)
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
                    ExtendStableKey(
                        signature, model->faces.size());

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

            // World-fixed large-scenery geometry is retained until native
            // invalidation or park/POV teardown. Camera direction is not an
            // invalidation source.
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
            if (surface.persistentBitmap)
                return !surface.image.IsBlended();
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
                    fingerprint, surface.coplanarOwner ? 1 : 0);
                ExtendStableKey(
                    fingerprint, surface.physicalCoverage ? 1 : 0);
                ExtendStableKey(
                    fingerprint, surface.artworkCarrier ? 1 : 0);
                ExtendStableKey(
                    fingerprint, surface.persistentBitmap ? 1 : 0);
                ExtendStableKey(
                    fingerprint, surface.cameraIndependent ? 1 : 0);
                ExtendStableKey(
                    fingerprint, surface.immutableFingerprint);
                ExtendStableKey(
                    fingerprint, surface.textureFallbackOnly ? 1 : 0);
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

        [[nodiscard]] uint64_t SemanticComponentFingerprint(
            const std::vector<FirstPersonPaintSemanticComponent>& components)
        {
            uint64_t fingerprint = 14695981039346656037ull;
            const auto extendFloat = [&](float value) {
                ExtendStableKey(
                    fingerprint,
                    uint64_t(int64_t(std::llround(
                        double(value) * 1024.0))));
            };
            const auto extendVec =
                [&](FirstPersonPaintSemanticVec3 value) {
                    extendFloat(value.x);
                    extendFloat(value.y);
                    extendFloat(value.z);
                };
            const auto extendImage = [&](ImageId image) {
                ExtendStableKey(fingerprint, image.GetIndex());
                ExtendStableKey(fingerprint, image.GetRemap());
                ExtendStableKey(
                    fingerprint, EnumValue(image.GetPrimary()));
                ExtendStableKey(
                    fingerprint, EnumValue(image.GetSecondary()));
                ExtendStableKey(
                    fingerprint, EnumValue(image.GetTertiary()));
                ExtendStableKey(
                    fingerprint, image.HasPrimary() ? 1 : 0);
                ExtendStableKey(
                    fingerprint, image.HasSecondary() ? 1 : 0);
                ExtendStableKey(
                    fingerprint, image.HasTertiary() ? 1 : 0);
                ExtendStableKey(
                    fingerprint, image.IsBlended() ? 1 : 0);
            };

            ExtendStableKey(
                fingerprint, components.size());
            for (const auto& component : components)
            {
                ExtendStableKey(
                    fingerprint, EnumValue(component.role));
                ExtendStableKey(
                    fingerprint,
                    uint32_t(component.mapPosition.x));
                ExtendStableKey(
                    fingerprint,
                    uint32_t(component.mapPosition.y));
                ExtendStableKey(
                    fingerprint,
                    EnumValue(component.geometry.kind));
                ExtendStableKey(
                    fingerprint,
                    component.geometry.pointCount);
                for (size_t i = 0;
                     i < component.geometry.pointCount
                        && i < component.geometry.points.size();
                     ++i)
                {
                    extendVec(component.geometry.points[i]);
                }
                extendFloat(
                    component.geometry.halfWidth);
                extendFloat(
                    component.geometry.halfHeight);
                ExtendStableKey(
                    fingerprint,
                    component.geometry.localHullKey);
                extendVec(component.transform.origin);
                extendVec(component.transform.axisX);
                extendVec(component.transform.axisY);
                extendVec(component.transform.axisZ);
                extendImage(component.artwork.image);
                extendImage(component.artwork.mask);
                ExtendStableKey(
                    fingerprint,
                    uint32_t(component.artwork.screenPos.x));
                ExtendStableKey(
                    fingerprint,
                    uint32_t(component.artwork.screenPos.y));
                // Artwork group/snapshot handles are paint-session
                // bookkeeping and can be renumbered when a different subset
                // of tiles is repainted. They are not semantic world state.
                ExtendStableKey(
                    fingerprint,
                    component.artwork.immutableWidth);
                ExtendStableKey(
                    fingerprint,
                    component.artwork.immutableHeight);
                if (!component.artwork.immutablePixels.empty())
                {
                    uint64_t pixelHash =
                        14695981039346656037ull;
                    for (const auto pixel :
                         component.artwork.immutablePixels)
                    {
                        pixelHash ^= pixel;
                        pixelHash *=
                            1099511628211ull;
                    }
                    ExtendStableKey(
                        fingerprint, pixelHash);
                }
                ExtendStableKey(
                    fingerprint,
                    component.artwork.sourceRotation);
                ExtendStableKey(
                    fingerprint,
                    component.artwork.decal ? 1 : 0);
                ExtendStableKey(
                    fingerprint,
                    component.repetitionIndex);
                ExtendStableKey(
                    fingerprint,
                    component.collidable ? 1 : 0);
            }
            return fingerprint;
        }

        [[nodiscard]] uint64_t CameraIndependentSurfaceKey(
            const FirstPersonSurface& surface)
        {
            uint64_t key = 14695981039346656037ull;
            ExtendStableKey(key, surface.immutableFingerprint);
            ExtendStableKey(key, surface.solidColour);
            ExtendStableKey(key, surface.nativePaintOrdinal);
            ExtendStableKey(key, surface.image.GetRemap());
            ExtendStableKey(
                key,
                surface.image.HasPrimary()
                    ? EnumValue(surface.image.GetPrimary())
                    : 0xffu);
            ExtendStableKey(
                key,
                surface.image.HasSecondary()
                    ? EnumValue(surface.image.GetSecondary())
                    : 0xffu);
            ExtendStableKey(
                key,
                surface.image.HasTertiary()
                    ? EnumValue(surface.image.GetTertiary())
                    : 0xffu);
            for (const auto& vertex : surface.triangles)
            {
                ExtendStableKey(
                    key,
                    uint64_t(int64_t(std::llround(
                        double(vertex.world.x) * 256.0))));
                ExtendStableKey(
                    key,
                    uint64_t(int64_t(std::llround(
                        double(vertex.world.y) * 256.0))));
                ExtendStableKey(
                    key,
                    uint64_t(int64_t(std::llround(
                        double(vertex.world.z) * 256.0))));
            }
            return key;
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

        [[nodiscard]] uint64_t FirstPersonTrackProfileSignature(
            const Ride& ride, const TrackElement& track)
        {
            const auto profile =
                FirstPersonTrackRailProfileFor(ride, track);
            if (!profile.has_value())
                return 0;
            uint64_t result = 14695981039346656037ull;
            ExtendStableKey(result, profile->railCount);
            ExtendStableKey(
                result, uint32_t(std::lround(profile->halfGauge * 100.0f)));
            ExtendStableKey(
                result, uint32_t(std::lround(profile->halfWidth * 100.0f)));
            ExtendStableKey(
                result, uint32_t(std::lround(profile->halfHeight * 100.0f)));
            ExtendStableKey(
                result, uint32_t(int32_t(std::lround(
                    profile->verticalOffset * 100.0f))));
            return result;
        }

        [[nodiscard]] std::optional<std::vector<uint8_t>>
            DecodeFirstPersonSpritePixels(const G1Element& g1)
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

        [[nodiscard]] bool SameFirstPersonMaterialTemplate(
            ImageId a, ImageId b)
        {
            if (!a.HasValue() || !b.HasValue())
                return false;
            if (a.IsBlended() != b.IsBlended()
                || a.IsRemap() != b.IsRemap()
                || a.HasPrimary() != b.HasPrimary()
                || a.HasSecondary() != b.HasSecondary()
                || a.HasTertiary() != b.HasTertiary())
                return false;
            if (a.GetRemap() != b.GetRemap())
                return false;
            if (a.HasPrimary()
                && a.GetPrimary() != b.GetPrimary())
                return false;
            if (a.HasSecondary()
                && a.GetSecondary() != b.GetSecondary())
                return false;
            if (a.HasTertiary()
                && a.GetTertiary() != b.GetTertiary())
                return false;
            return true;
        }

        void CaptureFirstPersonTrackArtworkProjection(
            const FirstPersonSurface& surface, uint8_t rotation)
        {
            if (surface.reconstructionGroup == 0
                || rotation >= 4
                || !surface.viewFacing
                || !surface.image.HasValue()
                || surface.image.IsBlended())
                return;

            const auto found = _trackTrajectoryCache.find(
                surface.reconstructionGroup);
            if (found == _trackTrajectoryCache.end()
                || found->second.dirty
                || found->second.surfaces.empty())
                return;

            auto& trajectory = found->second;
            const auto* g1 =
                GfxGetG1Element(surface.image);
            if (g1 == nullptr
                || g1->width <= 0
                || g1->height <= 0)
                return;

            // Track material must actually overlap the authoritative rail
            // projection. This rejects distant support/decorative layers while
            // keeping ties, cross-members and rail artwork that belong to the
            // guideway itself.
            const auto& railSilhouette =
                trajectory.railSilhouettes[rotation];
            if (!railSilhouette.empty())
            {
                const int32_t left =
                    int32_t(std::lround(
                        surface.billboardLeft));
                const int32_t top =
                    int32_t(std::lround(
                        surface.billboardTop));
                const int32_t right =
                    left + g1->width;
                const int32_t bottom =
                    top + g1->height;
                constexpr int32_t kArtworkOverlapHalo = 2;
                if (right
                        <= railSilhouette.minX
                            - kArtworkOverlapHalo
                    || left
                        >= railSilhouette.maxX
                            + kArtworkOverlapHalo
                    || bottom
                        <= railSilhouette.minY
                            - kArtworkOverlapHalo
                    || top
                        >= railSilhouette.maxY
                            + kArtworkOverlapHalo)
                    return;
            }

            auto& projections =
                trajectory.artworkProjections[rotation];
            const auto duplicate = std::find_if(
                projections.begin(), projections.end(),
                [&](const TrackTrajectoryCacheEntry::ArtworkProjection& p) {
                    return p.image == surface.image
                        && p.mask == surface.mask
                        && std::abs(
                               p.left
                               - surface.billboardLeft)
                            < 0.01f
                        && std::abs(
                               p.top
                               - surface.billboardTop)
                            < 0.01f;
                });
            if (duplicate != projections.end())
            {
                if (duplicate->immutableFingerprint
                    == surface.immutableFingerprint)
                    return;
                duplicate->immutableFingerprint =
                    surface.immutableFingerprint;
                duplicate->immutablePixels =
                    surface.immutablePixels;
                duplicate->immutableWidth =
                    surface.immutableWidth;
                duplicate->immutableHeight =
                    surface.immutableHeight;
                MarkTrackTrajectoryRegionsDirty(
                    trajectory);
                return;
            }

            constexpr size_t kMaximumProjectionLayers = 16;
            if (projections.size() >= kMaximumProjectionLayers)
                return;

            TrackTrajectoryCacheEntry::ArtworkProjection
                projection{};
            projection.image = surface.image;
            projection.mask = surface.mask;
            projection.anchor =
                surface.billboardAnchor;
            projection.left =
                surface.billboardLeft;
            projection.top =
                surface.billboardTop;
            projection.immutableFingerprint =
                surface.immutableFingerprint;
            projection.immutablePixels =
                surface.immutablePixels;
            projection.immutableWidth =
                surface.immutableWidth;
            projection.immutableHeight =
                surface.immutableHeight;
            projection.rotation = rotation;
            projections.emplace_back(
                std::move(projection));
            // Geometry stays unchanged; only the resident rail material packet
            // needs to pick up the newly captured native artwork.
            MarkTrackTrajectoryRegionsDirty(trajectory);
        }

        void AppendFirstPersonTrackArtworkProjections(
            const FirstPersonSurface& carrier,
            const TrackTrajectoryCacheEntry& trajectory,
            std::vector<FirstPersonSurface>& output)
        {
            if (!carrier.artworkCarrier)
                return;

            const std::array<FirstPersonVec3, 4> face{ {
                carrier.triangles[0].world,
                carrier.triangles[1].world,
                carrier.triangles[2].world,
                carrier.triangles[5].world,
            } };
            const auto edgeLength =
                [](FirstPersonVec3 a, FirstPersonVec3 b) {
                    const float dx = b.x - a.x;
                    const float dy = b.y - a.y;
                    const float dz = b.z - a.z;
                    return std::sqrt(
                        dx * dx + dy * dy + dz * dz);
                };
            const auto facePoint =
                [&](float s, float t) {
                    const auto lerp =
                        [](FirstPersonVec3 a,
                           FirstPersonVec3 b,
                           float alpha) {
                            return FirstPersonVec3{
                                a.x + (b.x - a.x) * alpha,
                                a.y + (b.y - a.y) * alpha,
                                a.z + (b.z - a.z) * alpha,
                            };
                        };
                    return lerp(
                        lerp(face[0], face[1], s),
                        lerp(face[3], face[2], s), t);
                };

            const int32_t width =
                std::clamp(
                    int32_t(std::ceil(std::max(
                        edgeLength(face[0], face[1]),
                        edgeLength(face[3], face[2])))),
                    1, 256);
            const int32_t height =
                std::clamp(
                    int32_t(std::ceil(std::max(
                        edgeLength(face[0], face[3]),
                        edgeLength(face[1], face[2])))),
                    1, 256);

            std::array<float, 4> rotationArea{};
            for (uint8_t rotation = 0;
                 rotation < 4; ++rotation)
            {
                if (trajectory
                        .artworkProjections[rotation]
                        .empty())
                    continue;
                std::array<ScreenCoordsXY, 4>
                    projected{};
                for (size_t i = 0;
                     i < face.size(); ++i)
                {
                    const auto& p = face[i];
                    projected[i] =
                        Translate3DTo2DWithZ(
                            rotation,
                            {
                                int32_t(std::lround(p.x)),
                                int32_t(std::lround(p.y)),
                                int32_t(std::lround(p.z)),
                            });
                }
                float twiceArea = 0.0f;
                for (size_t i = 0;
                     i < projected.size(); ++i)
                {
                    const auto& a = projected[i];
                    const auto& b =
                        projected[(i + 1) & 3u];
                    twiceArea +=
                        float(a.x * b.y - b.x * a.y);
                }
                rotationArea[rotation] =
                    std::abs(twiceArea) * 0.5f;
            }

            struct DecodedTrackProjection
            {
                const TrackTrajectoryCacheEntry::
                    ArtworkProjection* projection = nullptr;
                const G1Element* g1 = nullptr;
                std::vector<uint8_t> pixels;
                const G1Element* maskG1 = nullptr;
                std::vector<uint8_t> maskPixels;
                size_t paintOrder = 0;
            };
            std::array<
                std::vector<DecodedTrackProjection>, 4>
                decoded{};
            for (uint8_t rotation = 0;
                 rotation < 4; ++rotation)
            {
                const auto& projections =
                    trajectory
                        .artworkProjections[rotation];
                decoded[rotation].reserve(
                    projections.size());
                for (size_t index = 0;
                     index < projections.size();
                     ++index)
                {
                    const auto& projection =
                        projections[index];
                    const auto* g1 =
                        GfxGetG1Element(
                            projection.image);
                    if (g1 == nullptr
                        || g1->width <= 0
                        || g1->height <= 0)
                        continue;
                    std::vector<uint8_t> pixels;
                    if (!projection.immutablePixels.empty()
                        && projection.immutableWidth
                            == g1->width
                        && projection.immutableHeight
                            == g1->height)
                    {
                        pixels =
                            projection.immutablePixels;
                    }
                    else
                    {
                        const auto decodedPixels =
                            DecodeFirstPersonSpritePixels(
                                *g1);
                        if (!decodedPixels.has_value())
                            continue;
                        pixels = *decodedPixels;
                    }

                    DecodedTrackProjection item{};
                    item.projection = &projection;
                    item.g1 = g1;
                    item.pixels = std::move(pixels);
                    item.paintOrder = index;
                    if (projection.mask.HasValue())
                    {
                        item.maskG1 =
                            GfxGetG1Element(
                                projection.mask);
                        if (item.maskG1 != nullptr)
                        {
                            const auto maskPixels =
                                DecodeFirstPersonSpritePixels(
                                    *item.maskG1);
                            if (maskPixels.has_value())
                                item.maskPixels =
                                    *maskPixels;
                        }
                    }
                    decoded[rotation]
                        .emplace_back(std::move(item));
                }
            }

            struct TrackMaterialClass
            {
                ImageId material{};
                size_t order =
                    std::numeric_limits<size_t>::max();
            };
            std::vector<TrackMaterialClass>
                materials;
            for (uint8_t rotation = 0;
                 rotation < 4; ++rotation)
            {
                for (const auto& item :
                     decoded[rotation])
                {
                    auto found = std::find_if(
                        materials.begin(),
                        materials.end(),
                        [&](const TrackMaterialClass& material) {
                            return SameFirstPersonMaterialTemplate(
                                material.material,
                                item.projection->image);
                        });
                    if (found == materials.end())
                    {
                        materials.push_back({
                            item.projection->image,
                            item.paintOrder,
                        });
                    }
                    else
                    {
                        found->order =
                            std::min(
                                found->order,
                                item.paintOrder);
                    }
                }
            }
            std::sort(
                materials.begin(), materials.end(),
                [](const TrackMaterialClass& a,
                   const TrackMaterialClass& b) {
                    if (a.order != b.order)
                        return a.order < b.order;
                    return a.material.GetIndex()
                        < b.material.GetIndex();
                });

            const auto sampleProjection =
                [](const DecodedTrackProjection& item,
                   const FirstPersonVec3& worldPoint)
                    -> std::optional<uint8_t> {
                    const auto& projection =
                        *item.projection;
                    const CoordsXYZ anchorPoint{
                        int32_t(std::lround(
                            projection.anchor.x)),
                        int32_t(std::lround(
                            projection.anchor.y)),
                        int32_t(std::lround(
                            projection.anchor.z)),
                    };
                    const CoordsXYZ world{
                        int32_t(std::lround(
                            worldPoint.x)),
                        int32_t(std::lround(
                            worldPoint.y)),
                        int32_t(std::lround(
                            worldPoint.z)),
                    };
                    const auto isoAnchor =
                        Translate3DTo2DWithZ(
                            projection.rotation,
                            anchorPoint);
                    const auto iso =
                        Translate3DTo2DWithZ(
                            projection.rotation,
                            world);
                    const int32_t u =
                        int32_t(std::lround(
                            float(
                                iso.x
                                - isoAnchor.x)
                            - projection.left));
                    const int32_t v =
                        int32_t(std::lround(
                            float(
                                iso.y
                                - isoAnchor.y)
                            - projection.top));
                    if (u < 0 || v < 0
                        || u >= item.g1->width
                        || v >= item.g1->height)
                        return std::nullopt;

                    if (item.maskG1 != nullptr
                        && !item.maskPixels.empty())
                    {
                        if (u >= item.maskG1->width
                            || v >= item.maskG1->height)
                            return std::nullopt;
                        const uint8_t mask =
                            item.maskPixels[
                                size_t(v)
                                    * size_t(
                                        item.maskG1->width)
                                + size_t(u)];
                        if (mask == 0)
                            return std::nullopt;
                    }

                    const uint8_t pixel =
                        item.pixels[
                            size_t(v)
                                * size_t(item.g1->width)
                            + size_t(u)];
                    if (pixel == 0)
                        return std::nullopt;
                    return pixel;
                };

            for (size_t materialIndex = 0;
                 materialIndex < materials.size();
                 ++materialIndex)
            {
                const auto& materialClass =
                    materials[materialIndex];
                std::vector<uint8_t> rankedRotations;
                for (uint8_t rotation = 0;
                     rotation < 4; ++rotation)
                {
                    const bool hasCompatible =
                        std::any_of(
                            decoded[rotation].begin(),
                            decoded[rotation].end(),
                            [&](const DecodedTrackProjection& item) {
                                return SameFirstPersonMaterialTemplate(
                                    materialClass.material,
                                    item.projection->image);
                            });
                    if (hasCompatible
                        && rotationArea[rotation] > 0.0f)
                    {
                        rankedRotations.push_back(
                            rotation);
                    }
                }
                std::sort(
                    rankedRotations.begin(),
                    rankedRotations.end(),
                    [&](uint8_t a, uint8_t b) {
                        if (rotationArea[a]
                            != rotationArea[b])
                            return rotationArea[a]
                                > rotationArea[b];
                        return a < b;
                    });
                if (rankedRotations.empty())
                    continue;

                std::vector<uint8_t> pixels(
                    size_t(width)
                        * size_t(height),
                    0);
                bool hasPixel = false;
                for (int32_t y = 0;
                     y < height; ++y)
                for (int32_t x = 0;
                     x < width; ++x)
                {
                    const auto worldPoint =
                        facePoint(
                            (float(x) + 0.5f)
                                / float(width),
                            (float(y) + 0.5f)
                                / float(height));
                    uint8_t selectedPixel = 0;

                    for (const auto rotation :
                         rankedRotations)
                    {
                        const auto& layers =
                            decoded[rotation];
                        // Native paint order is preserved inside each
                        // rotation. The topmost compatible pixel wins before
                        // another native view is consulted.
                        for (auto it = layers.rbegin();
                             it != layers.rend(); ++it)
                        {
                            if (!SameFirstPersonMaterialTemplate(
                                    materialClass.material,
                                    it->projection->image))
                                continue;
                            const auto sample =
                                sampleProjection(
                                    *it, worldPoint);
                            if (!sample.has_value())
                                continue;
                            selectedPixel =
                                *sample;
                            break;
                        }
                        if (selectedPixel != 0)
                            break;
                    }

                    pixels[
                        size_t(y) * size_t(width)
                        + size_t(x)] =
                        selectedPixel;
                    hasPixel =
                        hasPixel || selectedPixel != 0;
                }
                if (!hasPixel)
                    continue;

                FirstPersonSurface surface = carrier;
                surface.image =
                    materialClass.material;
                surface.mask = {};
                surface.solidColour = 0;
                surface.physicalCoverage = false;
                surface.persistentBitmap = true;
                surface.immutablePixels =
                    std::move(pixels);
                surface.immutableWidth =
                    int16_t(width);
                surface.immutableHeight =
                    int16_t(height);

                uint64_t fingerprint =
                    14695981039346656037ull;
                ExtendStableKey(
                    fingerprint,
                    materialClass.material.GetRemap());
                ExtendStableKey(
                    fingerprint,
                    materialClass.material.HasPrimary()
                        ? EnumValue(
                            materialClass.material.GetPrimary())
                        : 0xffu);
                ExtendStableKey(
                    fingerprint,
                    materialClass.material.HasSecondary()
                        ? EnumValue(
                            materialClass.material.GetSecondary())
                        : 0xffu);
                ExtendStableKey(
                    fingerprint,
                    materialClass.material.HasTertiary()
                        ? EnumValue(
                            materialClass.material.GetTertiary())
                        : 0xffu);
                ExtendStableKey(
                    fingerprint, materialIndex);
                ExtendStableKey(
                    fingerprint, width);
                ExtendStableKey(
                    fingerprint, height);
                for (const auto pixel :
                     surface.immutablePixels)
                {
                    fingerprint ^= pixel;
                    fingerprint *= 1099511628211ull;
                }
                surface.immutableFingerprint =
                    fingerprint;

                const std::array<
                    FirstPersonVertex, 4> vertices{ {
                    { face[0], 0.0f, 0.0f },
                    { face[1], float(width), 0.0f },
                    { face[2], float(width),
                      float(height) },
                    { face[3], 0.0f,
                      float(height) },
                } };
                EmitQuad(surface, vertices);
                output.emplace_back(
                    std::move(surface));
            }
        }

        [[nodiscard]] std::array<FirstPersonSilhouette, 4>
            BuildFirstPersonAuthoritativeRailSilhouettes(
                const FirstPersonTrackTrajectory& trajectory,
                const FirstPersonVec3& anchor,
                const FirstPersonTrackRailProfile& profile)
        {
            std::array<FirstPersonSilhouette, 4> result{};
            const auto rails =
                BuildFirstPersonRailProxySegments(
                    trajectory, profile);
            if (rails.empty())
                return result;

            for (const auto& rail : rails)
            {
                const auto acrossA =
                    Mul(rail.basisA.right, rail.halfWidth);
                const auto acrossB =
                    Mul(rail.basisB.right, rail.halfWidth);
                const auto upA =
                    Mul(rail.basisA.up, rail.halfHeight);
                const auto upB =
                    Mul(rail.basisB.up, rail.halfHeight);

                const std::array<FirstPersonVec3, 8> p{ {
                    Sub(Sub(rail.a, acrossA), upA),
                    Add(Sub(rail.a, upA), acrossA),
                    Add(Add(rail.a, acrossA), upA),
                    Add(Sub(rail.a, acrossA), upA),
                    Sub(Sub(rail.b, acrossB), upB),
                    Add(Sub(rail.b, upB), acrossB),
                    Add(Add(rail.b, acrossB), upB),
                    Add(Sub(rail.b, acrossB), upB),
                } };
                const std::array<std::array<uint8_t, 4>, 4>
                    quads{ {
                        { 0, 1, 5, 4 },
                        { 1, 2, 6, 5 },
                        { 2, 3, 7, 6 },
                        { 3, 0, 4, 7 },
                    } };

                for (uint8_t rotation = 0;
                     rotation < 4; ++rotation)
                {
                    const CoordsXYZ anchorPoint{
                        int32_t(std::lround(anchor.x)),
                        int32_t(std::lround(anchor.y)),
                        int32_t(std::lround(anchor.z)),
                    };
                    const auto anchorScreen =
                        Translate3DTo2DWithZ(
                            rotation, anchorPoint);
                    for (const auto& quad : quads)
                    {
                        std::array<ScreenCoordsXY, 4>
                            screen{};
                        for (size_t i = 0;
                             i < screen.size(); ++i)
                        {
                            const auto& world =
                                p[quad[i]];
                            const auto projected =
                                Translate3DTo2DWithZ(
                                    rotation,
                                    {
                                        int32_t(std::lround(
                                            world.x)),
                                        int32_t(std::lround(
                                            world.y)),
                                        int32_t(std::lround(
                                            world.z)),
                                    });
                            screen[i] = {
                                projected.x
                                    - anchorScreen.x,
                                projected.y
                                    - anchorScreen.y,
                            };
                        }
                        AddFirstPersonSilhouetteQuad(
                            result[rotation], screen);
                    }
                }
            }
            return result;
        }

        [[nodiscard]] uint8_t FirstPersonRailColour(
            const Ride& ride, const TrackElement& track,
            bool topFace)
        {
            const auto scheme = std::min<uint8_t>(
                track.getColourScheme(),
                uint8_t(kNumRideColourSchemes - 1));
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
                    surface.physicalCoverage = true;
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

        void AppendTrajectoryArtworkCarrierSegment(
            TrackTrajectoryCacheEntry& cached, uint64_t groupKey,
            const FirstPersonTrackTrajectoryPoint& a,
            const FirstPersonTrackTrajectoryPoint& b,
            const FirstPersonTrackRailProfile& profile)
        {
            const auto centreA = Add(
                a.position,
                Mul(a.basis.up, profile.verticalOffset));
            const auto centreB = Add(
                b.position,
                Mul(b.basis.up, profile.verticalOffset));
            const auto midpoint = Mul(Add(centreA, centreB), 0.5f);
            const int32_t tileX = int32_t(std::floor(
                midpoint.x / float(kCoordsXYStep)));
            const int32_t tileY = int32_t(std::floor(
                midpoint.y / float(kCoordsXYStep)));
            const uint64_t gpuRegion =
                FirstPersonGpuRegionKey(tileX, tileY);

            const float halfAcross =
                profile.railCount > 1
                ? std::max(
                    4.0f,
                    profile.halfGauge
                        + profile.halfWidth + 2.0f)
                : std::max(
                    4.0f,
                    profile.halfWidth + 3.0f);
            const float halfVertical =
                std::max(
                    3.0f,
                    profile.halfHeight + 2.0f);
            const auto acrossA =
                Mul(a.basis.right, halfAcross);
            const auto acrossB =
                Mul(b.basis.right, halfAcross);
            const auto upA =
                Mul(a.basis.up, halfVertical);
            const auto upB =
                Mul(b.basis.up, halfVertical);

            const auto emitCarrier =
                [&](const std::array<FirstPersonVec3, 4>& points) {
                    FirstPersonSurface surface{};
                    surface.artworkCarrier = true;
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

            // This is deliberately an appearance shell, not collision geometry.
            // Transparent sprite pixels cut holes; solid trajectory rails remain
            // underneath as the physical fallback.
            emitCarrier({ {
                Add(Sub(centreA, acrossA), upA),
                Add(Add(centreA, acrossA), upA),
                Add(Add(centreB, acrossB), upB),
                Add(Sub(centreB, acrossB), upB),
            } });
            emitCarrier({ {
                Sub(Add(centreA, acrossA), upA),
                Sub(Add(centreB, acrossB), upB),
                Add(Add(centreB, acrossB), upB),
                Add(Add(centreA, acrossA), upA),
            } });
            emitCarrier({ {
                Sub(Sub(centreA, acrossA), upA),
                Add(Sub(centreA, acrossA), upA),
                Add(Sub(centreB, acrossB), upB),
                Sub(Sub(centreB, acrossB), upB),
            } });
            emitCarrier({ {
                Sub(Sub(centreA, acrossA), upA),
                Sub(Sub(centreB, acrossB), upB),
                Sub(Add(centreB, acrossB), upB),
                Sub(Add(centreA, acrossA), upA),
            } });
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
            const auto resolvedProfile =
                FirstPersonTrackRailProfileFor(ride, track);
            if (!resolvedProfile.has_value())
                return result;
            const auto& profile = *resolvedProfile;
            result.railSilhouettes =
                BuildFirstPersonAuthoritativeRailSilhouettes(
                    trajectory, groupAnchor, profile);
            const uint8_t topColour =
                FirstPersonRailColour(ride, track, true);
            const uint8_t sideColour =
                FirstPersonRailColour(ride, track, false);

            const auto railProxies =
                BuildFirstPersonRailProxySegments(
                    trajectory, profile);
            for (const auto& rail : railProxies)
            {
                AppendTrajectoryRailSegment(
                    result, groupKey, rail,
                    topColour, sideColour);
            }

            size_t previousCarrierPoint = 0;
            for (size_t i = 1;
                 i < trajectory.points.size(); ++i)
            {
                const auto& a =
                    trajectory.points[previousCarrierPoint];
                const auto& b =
                    trajectory.points[i];
                const float distance =
                    FirstPersonTrackTrajectoryPointDistance(
                        a, b);
                const float forwardDot =
                    Dot(a.basis.forward, b.basis.forward);
                const float upDot =
                    Dot(a.basis.up, b.basis.up);
                const bool turns =
                    forwardDot < 0.9914449f
                    || upDot < 0.9914449f;
                const bool last =
                    i + 1 == trajectory.points.size();
                if (!last
                    && distance < 3.0f
                    && !turns)
                    continue;
                if (distance > 0.05f)
                {
                    AppendTrajectoryArtworkCarrierSegment(
                        result, groupKey, a, b, profile);
                }
                previousCarrierPoint = i;
            }

            const auto& tunnelDescriptor =
                GetTunnelDescriptor(
                    track.isInverted()
                    ? TunnelType::invertedFlat
                    : TunnelType::standardFlat);
            const float tunnelFloor =
                profile.verticalOffset
                - profile.halfHeight - 4.0f;
            const float tunnelCeiling =
                tunnelFloor
                + float(tunnelDescriptor.height
                    * kCoordsZPerTinyZ);
            const float tunnelHalfWidth =
                std::max(
                    8.0f,
                    profile.halfGauge
                        + profile.halfWidth + 4.0f);
            uint8_t tunnelColour =
                static_cast<uint8_t>(
                    Drawing::getColourMap(
                        Drawing::Colour::darkBrown).midDark);
            if (tunnelColour == 0)
                tunnelColour = static_cast<uint8_t>(
                    Drawing::PaletteIndex::trackRails1);
            for (const auto& tunnel :
                 BuildFirstPersonTrackTunnelRoute(
                     trajectory, tunnelHalfWidth,
                     tunnelFloor, tunnelCeiling))
            {
                FirstPersonSurface surface{};
                surface.solidColour = tunnelColour;
                const int32_t tileX =
                    int32_t(std::floor(
                        tunnel.midpoint.x
                        / float(kCoordsXYStep)));
                const int32_t tileY =
                    int32_t(std::floor(
                        tunnel.midpoint.y
                        / float(kCoordsXYStep)));
                surface.gpuRegion =
                    FirstPersonGpuRegionKey(tileX, tileY);
                std::array<FirstPersonVertex, 4>
                    vertices{};
                for (size_t i = 0; i < 4; ++i)
                    vertices[i].world =
                        tunnel.quad.corners[i];
                EmitQuad(surface, vertices);
                result.surfaces.push_back(
                    std::move(surface));
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
                        const auto& a = trajectory.points.back();
                        const auto& b = next->points.front();
                        const auto appendBridge = [&](float gaugeSide) {
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
                                    authoritativeTrackTrajectory,
                            };
                            AppendTrajectoryRailSegment(
                                result, groupKey, bridge,
                                topColour, sideColour);
                        };
                        if (profile.railCount == 1)
                        {
                            appendBridge(0.0f);
                        }
                        else
                        {
                            appendBridge(-profile.halfGauge);
                            appendBridge(profile.halfGauge);
                        }
                        AppendTrajectoryArtworkCarrierSegment(
                            result, groupKey, a, b, profile);
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
                        ++scene.trackGeometryCacheHits;
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

                    ++scene.trackGeometryBuilds;
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

            // Stable trajectory geometry remains cached until the track is
            // authoritatively invalidated. Looking away must not make rails
            // expensive to rediscover when the player turns back.
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
                        cache.ground.physicalCoverage = true;
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
            // Terrain is bounded by the park and changes only through native
            // invalidation. Retaining it makes a turn a visibility operation,
            // not a terrain reconstruction operation.
        }
                [[nodiscard]] std::vector<std::array<FirstPersonVec3, 4>>
            BuildFirstPersonSemanticPhysicalFaces(
                const FirstPersonPaintSemanticComponent& component)
        {
            std::vector<std::array<FirstPersonVec3, 4>> result;
            if (component.geometry.kind
                == FirstPersonPaintSemanticPrimitiveKind::opening)
                return result;

            const auto worldPoint =
                [&](FirstPersonPaintSemanticVec3 p) {
                    return FirstPersonSemanticWorldPoint(
                        component.transform, p);
                };
            const auto& geometry = component.geometry;

            if (geometry.kind
                    == FirstPersonPaintSemanticPrimitiveKind::plane
                || geometry.kind
                    == FirstPersonPaintSemanticPrimitiveKind::footprint)
            {
                if (geometry.pointCount >= 4)
                {
                    result.push_back({ {
                        worldPoint(geometry.points[0]),
                        worldPoint(geometry.points[1]),
                        worldPoint(geometry.points[2]),
                        worldPoint(geometry.points[3]),
                    } });
                }
                return result;
            }

            if (geometry.kind
                == FirstPersonPaintSemanticPrimitiveKind::beam)
            {
                if (geometry.pointCount < 2)
                    return result;
                const auto a = worldPoint(geometry.points[0]);
                const auto b = worldPoint(geometry.points[1]);
                FirstPersonVec3 d{
                    b.x - a.x, b.y - a.y, b.z - a.z
                };
                const float length = std::sqrt(
                    d.x * d.x + d.y * d.y + d.z * d.z);
                if (!(length > 1e-5f))
                    return result;
                d = {
                    d.x / length, d.y / length, d.z / length
                };
                FirstPersonVec3 side{ d.y, -d.x, 0.0f };
                float sideLength = std::sqrt(
                    side.x * side.x + side.y * side.y);
                if (!(sideLength > 1e-5f))
                {
                    side = { 1.0f, 0.0f, 0.0f };
                    sideLength = 1.0f;
                }
                side = {
                    side.x / sideLength,
                    side.y / sideLength,
                    side.z / sideLength,
                };
                const FirstPersonVec3 up{
                    side.y * d.z - side.z * d.y,
                    side.z * d.x - side.x * d.z,
                    side.x * d.y - side.y * d.x,
                };
                const float halfWidth =
                    std::max(0.5f, geometry.halfWidth);
                const float halfHeight =
                    std::max(0.5f, geometry.halfHeight);
                const auto corner =
                    [&](FirstPersonVec3 p,
                        float sideSign, float upSign) {
                        return FirstPersonVec3{
                            p.x + side.x * halfWidth * sideSign
                                + up.x * halfHeight * upSign,
                            p.y + side.y * halfWidth * sideSign
                                + up.y * halfHeight * upSign,
                            p.z + side.z * halfWidth * sideSign
                                + up.z * halfHeight * upSign,
                        };
                    };
                const std::array<FirstPersonVec3, 8> p{ {
                    corner(a, -1.0f, -1.0f),
                    corner(a,  1.0f, -1.0f),
                    corner(a,  1.0f,  1.0f),
                    corner(a, -1.0f,  1.0f),
                    corner(b, -1.0f, -1.0f),
                    corner(b,  1.0f, -1.0f),
                    corner(b,  1.0f,  1.0f),
                    corner(b, -1.0f,  1.0f),
                } };
                result.push_back({ { p[0], p[1], p[2], p[3] } });
                result.push_back({ { p[4], p[7], p[6], p[5] } });
                result.push_back({ { p[0], p[4], p[5], p[1] } });
                result.push_back({ { p[1], p[5], p[6], p[2] } });
                result.push_back({ { p[2], p[6], p[7], p[3] } });
                result.push_back({ { p[3], p[7], p[4], p[0] } });
                return result;
            }

            if ((geometry.kind
                    == FirstPersonPaintSemanticPrimitiveKind::box
                 || geometry.kind
                    == FirstPersonPaintSemanticPrimitiveKind::localHull)
                && geometry.pointCount >= 2)
            {
                const auto low = geometry.points[0];
                const auto high = geometry.points[1];
                const std::array<
                    FirstPersonPaintSemanticVec3, 8> local{ {
                    { low.x, low.y, low.z },
                    { high.x, low.y, low.z },
                    { high.x, high.y, low.z },
                    { low.x, high.y, low.z },
                    { low.x, low.y, high.z },
                    { high.x, low.y, high.z },
                    { high.x, high.y, high.z },
                    { low.x, high.y, high.z },
                } };
                std::array<FirstPersonVec3, 8> p{};
                for (size_t i = 0; i < p.size(); ++i)
                    p[i] = worldPoint(local[i]);
                result.push_back({ { p[0], p[1], p[5], p[4] } });
                result.push_back({ { p[1], p[2], p[6], p[5] } });
                result.push_back({ { p[2], p[3], p[7], p[6] } });
                result.push_back({ { p[3], p[0], p[4], p[7] } });
                result.push_back({ { p[4], p[5], p[6], p[7] } });
                result.push_back({ { p[3], p[2], p[1], p[0] } });
            }
            return result;
        }

        [[nodiscard]] bool SameFirstPersonSemanticFace(
            const std::array<FirstPersonVec3, 4>& a,
            const std::array<FirstPersonVec3, 4>& b)
        {
            const auto samePoint =
                [](FirstPersonVec3 p, FirstPersonVec3 q) {
                    constexpr float kEpsilon = 0.01f;
                    return std::abs(p.x - q.x) <= kEpsilon
                        && std::abs(p.y - q.y) <= kEpsilon
                        && std::abs(p.z - q.z) <= kEpsilon;
                };

            // Faces can use a different starting corner or winding.
            for (size_t start = 0; start < 4; ++start)
            {
                bool forward = true;
                bool reverse = true;
                for (size_t i = 0; i < 4; ++i)
                {
                    forward = forward
                        && samePoint(
                            a[i], b[(start + i) & 3u]);
                    reverse = reverse
                        && samePoint(
                            a[i],
                            b[(start + 4u - i) & 3u]);
                }
                if (forward || reverse)
                    return true;
            }
            return false;
        }

        [[nodiscard]] uint64_t FirstPersonSemanticPhysicalFingerprint(
            const FirstPersonPaintSemanticComponent& component)
        {
            uint64_t fingerprint = 14695981039346656037ull;
            ExtendStableKey(fingerprint, EnumValue(component.role));
            ExtendStableKey(
                fingerprint, EnumValue(component.geometry.kind));
            ExtendStableKey(
                fingerprint, component.geometry.localHullKey);
            ExtendStableKey(
                fingerprint, component.repetitionIndex);
            ExtendStableKey(
                fingerprint, component.collidable ? 1 : 0);
            const auto faces =
                BuildFirstPersonSemanticPhysicalFaces(component);
            ExtendStableKey(fingerprint, faces.size());

            std::vector<std::array<int64_t, 3>> points;
            points.reserve(faces.size() * 4);
            for (const auto& face : faces)
            for (const auto& p : face)
            {
                points.push_back({
                    int64_t(std::llround(double(p.x) * 256.0)),
                    int64_t(std::llround(double(p.y) * 256.0)),
                    int64_t(std::llround(double(p.z) * 256.0)),
                });
            }
            std::sort(points.begin(), points.end());
            for (const auto& p : points)
            {
                ExtendStableKey(fingerprint, uint64_t(p[0]));
                ExtendStableKey(fingerprint, uint64_t(p[1]));
                ExtendStableKey(fingerprint, uint64_t(p[2]));
            }
            ExtendStableKey(
                fingerprint,
                uint64_t(int64_t(std::llround(
                    double(component.geometry.halfWidth) * 256.0))));
            ExtendStableKey(
                fingerprint,
                uint64_t(int64_t(std::llround(
                    double(component.geometry.halfHeight) * 256.0))));
            return fingerprint;
        }

        struct FirstPersonSemanticArtworkView
        {
            const FirstPersonPaintSemanticComponent* component = nullptr;
            const G1Element* g1 = nullptr;
            std::vector<uint8_t> pixels;
            const G1Element* maskG1 = nullptr;
            std::vector<uint8_t> maskPixels;
        };

        [[nodiscard]] FirstPersonSemanticArtworkView
            DecodeFirstPersonSemanticArtworkView(
                const FirstPersonPaintSemanticComponent* component)
        {
            FirstPersonSemanticArtworkView result{};
            if (component == nullptr
                || !component->artwork.image.HasValue())
                return result;

            const auto* g1 =
                GfxGetG1Element(component->artwork.image);
            if (g1 == nullptr || g1->width <= 0
                || g1->height <= 0)
                return result;

            result.component = component;
            result.g1 = g1;
            if (!component->artwork.immutablePixels.empty()
                && component->artwork.immutableWidth
                    == g1->width
                && component->artwork.immutableHeight
                    == g1->height)
            {
                result.pixels =
                    component->artwork.immutablePixels;
            }
            if (result.pixels.empty())
            {
                const auto decoded =
                    DecodeFirstPersonSpritePixels(*g1);
                if (!decoded.has_value())
                    return {};
                result.pixels = *decoded;
            }

            if (component->artwork.mask.HasValue())
            {
                result.maskG1 =
                    GfxGetG1Element(component->artwork.mask);
                if (result.maskG1 != nullptr)
                {
                    const auto decodedMask =
                        DecodeFirstPersonSpritePixels(
                            *result.maskG1);
                    if (decodedMask.has_value())
                        result.maskPixels = *decodedMask;
                }
            }
            return result;
        }

        [[nodiscard]] bool FirstPersonSemanticFaceOwnedNear(
            const FirstPersonDepthOwnerMap& map,
            uint32_t owner, int32_t x, int32_t y)
        {
            for (int32_t dy = -1; dy <= 1; ++dy)
            for (int32_t dx = -1; dx <= 1; ++dx)
            {
                const auto found = map.find(
                    FirstPersonSilhouettePixelKey(
                        x + dx, y + dy));
                if (found != map.end()
                    && found->second.owner == owner)
                    return true;
            }
            return false;
        }

        [[nodiscard]] FirstPersonVec3
            FirstPersonSemanticFacePoint(
                const std::array<FirstPersonVec3, 4>& face,
                float s, float t)
        {
            const auto lerp =
                [](FirstPersonVec3 a, FirstPersonVec3 b, float alpha) {
                    return FirstPersonVec3{
                        a.x + (b.x - a.x) * alpha,
                        a.y + (b.y - a.y) * alpha,
                        a.z + (b.z - a.z) * alpha,
                    };
                };
            return lerp(
                lerp(face[0], face[1], s),
                lerp(face[3], face[2], s), t);
        }

        [[nodiscard]] float FirstPersonSemanticEdgeLength(
            FirstPersonVec3 a, FirstPersonVec3 b)
        {
            const float dx = b.x - a.x;
            const float dy = b.y - a.y;
            const float dz = b.z - a.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        [[nodiscard]] uint8_t FirstPersonSemanticFallbackPixel(
            const FirstPersonSemanticArtworkView& view)
        {
            std::array<uint32_t, 256> counts{};
            for (const auto pixel : view.pixels)
                if (pixel != 0)
                    ++counts[pixel];
            uint8_t best = 0;
            uint32_t bestCount = 0;
            for (size_t i = 1; i < counts.size(); ++i)
            {
                if (counts[i] > bestCount)
                {
                    best = uint8_t(i);
                    bestCount = counts[i];
                }
            }
            return best;
        }

        [[nodiscard]] std::optional<uint8_t>
            SampleFirstPersonSemanticArtwork(
                const FirstPersonSemanticArtworkView& view,
                const FirstPersonVec3& worldPoint,
                uint8_t sourceRotation)
        {
            if (view.component == nullptr || view.g1 == nullptr
                || view.pixels.empty())
                return std::nullopt;

            const CoordsXYZ point{
                int32_t(std::lround(worldPoint.x)),
                int32_t(std::lround(worldPoint.y)),
                int32_t(std::lround(worldPoint.z)),
            };
            const auto iso =
                Translate3DTo2DWithZ(sourceRotation, point);
            const int32_t u =
                iso.x - view.component->artwork.screenPos.x
                - view.g1->xOffset;
            const int32_t v =
                iso.y - view.component->artwork.screenPos.y
                - view.g1->yOffset;
            if (u < 0 || v < 0
                || u >= view.g1->width
                || v >= view.g1->height)
                return std::nullopt;

            if (view.maskG1 != nullptr
                && !view.maskPixels.empty())
            {
                const int32_t mu =
                    iso.x - view.component->artwork.screenPos.x
                    - view.maskG1->xOffset;
                const int32_t mv =
                    iso.y - view.component->artwork.screenPos.y
                    - view.maskG1->yOffset;
                if (mu < 0 || mv < 0
                    || mu >= view.maskG1->width
                    || mv >= view.maskG1->height)
                    return std::nullopt;
                const uint8_t mask =
                    view.maskPixels[
                        size_t(mv) * size_t(view.maskG1->width)
                        + size_t(mu)];
                if (mask == 0)
                    return std::nullopt;
            }

            const uint8_t pixel =
                view.pixels[
                    size_t(v) * size_t(view.g1->width)
                    + size_t(u)];
            return pixel != 0
                ? std::optional<uint8_t>{ pixel }
                : std::nullopt;
        }

        [[nodiscard]] std::vector<FirstPersonSurface>
            BuildFirstPersonSemanticComponentSurfaces(
                const FirstPersonPaintSemanticComponent& component,
                uint64_t gpuRegion,
                const std::array<
                    const FirstPersonPaintSemanticComponent*, 4>&
                    artworkViews,
                const std::vector<
                    const FirstPersonPaintSemanticComponent*>*
                    depthPeers = nullptr)
        {
            std::vector<FirstPersonSurface> result;
            const auto faces =
                BuildFirstPersonSemanticPhysicalFaces(component);
            if (faces.empty())
                return result;

            std::vector<std::array<FirstPersonVec3, 4>>
                depthFaces = faces;
            if (depthPeers != nullptr)
            {
                for (const auto* peer : *depthPeers)
                {
                    if (peer == nullptr
                        || peer == &component)
                        continue;
                    const auto peerFaces =
                        BuildFirstPersonSemanticPhysicalFaces(
                            *peer);
                    for (const auto& peerFace : peerFaces)
                    {
                        const bool coincident =
                            std::any_of(
                                faces.begin(), faces.end(),
                                [&](const auto& targetFace) {
                                    return SameFirstPersonSemanticFace(
                                        targetFace, peerFace);
                                });
                        // Coplanar decals/host planes describe the same
                        // physical surface and are allowed to share it. Only
                        // genuinely distinct geometry participates as an
                        // occluder.
                        if (!coincident)
                            depthFaces.push_back(peerFace);
                    }
                }
            }

            std::array<
                const FirstPersonPaintSemanticComponent*, 4>
                observations = artworkViews;
            if (component.artwork.sourceRotation < 4
                && observations[
                       component.artwork.sourceRotation]
                    == nullptr)
            {
                observations[
                    component.artwork.sourceRotation] =
                    &component;
            }

            std::array<FirstPersonSemanticArtworkView, 4>
                decodedViews{};
            for (uint8_t rotation = 0; rotation < 4; ++rotation)
            {
                decodedViews[rotation] =
                    DecodeFirstPersonSemanticArtworkView(
                        observations[rotation]);
            }

            std::array<FirstPersonDepthOwnerMap, 4>
                depthOwners{};
            std::array<std::vector<float>, 4>
                projectedAreas{};
            for (uint8_t rotation = 0; rotation < 4; ++rotation)
            {
                projectedAreas[rotation].resize(
                    faces.size(), 0.0f);
                if (decodedViews[rotation].component == nullptr)
                    continue;
                for (size_t faceIndex = 0;
                     faceIndex < depthFaces.size(); ++faceIndex)
                {
                    std::array<ScreenCoordsXY, 4> screen{};
                    std::array<float, 4> depth{};
                    for (size_t i = 0; i < 4; ++i)
                    {
                        const auto& p = depthFaces[faceIndex][i];
                        const CoordsXYZ point{
                            int32_t(std::lround(p.x)),
                            int32_t(std::lround(p.y)),
                            int32_t(std::lround(p.z)),
                        };
                        screen[i] =
                            Translate3DTo2DWithZ(
                                rotation, point);
                        depth[i] =
                            FirstPersonIsoDepth(
                                rotation, point);
                    }
                    AddFirstPersonDepthTriangle(
                        depthOwners[rotation],
                        uint32_t(faceIndex),
                        { screen[0], screen[1], screen[2] },
                        { depth[0], depth[1], depth[2] });
                    AddFirstPersonDepthTriangle(
                        depthOwners[rotation],
                        uint32_t(faceIndex),
                        { screen[0], screen[2], screen[3] },
                        { depth[0], depth[2], depth[3] });
                    float twiceArea = 0.0f;
                    for (size_t i = 0; i < 4; ++i)
                    {
                        const auto& a = screen[i];
                        const auto& b = screen[(i + 1) & 3u];
                        twiceArea +=
                            float(a.x * b.y - b.x * a.y);
                    }
                    if (faceIndex < faces.size())
                    {
                        projectedAreas[rotation][faceIndex] =
                            std::abs(twiceArea) * 0.5f;
                    }
                }
            }

            uint8_t fallbackColour = static_cast<uint8_t>(
                Drawing::PaletteIndex::trackRails1);
            if (component.artwork.image.HasPrimary())
            {
                const auto colour =
                    component.artwork.image.GetPrimary();
                if (Drawing::colourIsValid(colour))
                    fallbackColour = static_cast<uint8_t>(
                        Drawing::getColourMap(colour).midDark);
            }

            for (size_t faceIndex = 0;
                 faceIndex < faces.size(); ++faceIndex)
            {
                const auto& face = faces[faceIndex];
                struct Candidate
                {
                    uint8_t rotation = 0;
                    float score = 0.0f;
                };
                std::vector<Candidate> candidates;
                for (uint8_t rotation = 0;
                     rotation < 4; ++rotation)
                {
                    const auto& view =
                        decodedViews[rotation];
                    if (view.component == nullptr
                        || view.g1 == nullptr
                        || view.pixels.empty())
                        continue;

                    std::array<ScreenCoordsXY, 4> projected{};
                    FirstPersonSilhouette silhouette{};
                    for (size_t i = 0; i < 4; ++i)
                    {
                        const auto& p = face[i];
                        projected[i] =
                            Translate3DTo2DWithZ(
                                rotation,
                                {
                                    int32_t(std::lround(p.x)),
                                    int32_t(std::lround(p.y)),
                                    int32_t(std::lround(p.z)),
                                });
                    }
                    AddFirstPersonSilhouetteQuad(
                        silhouette, projected);
                    const float ownership =
                        FirstPersonDepthOwnerCoverage(
                            depthOwners[rotation],
                            uint32_t(faceIndex),
                            silhouette);
                    if (ownership < 0.20f)
                        continue;
                    const float score =
                        projectedAreas[rotation][faceIndex]
                        * ownership;
                    if (score > 0.0f)
                        candidates.push_back({
                            rotation, score
                        });
                }
                std::sort(
                    candidates.begin(), candidates.end(),
                    [](const Candidate& a, const Candidate& b) {
                        if (a.score != b.score)
                            return a.score > b.score;
                        return a.rotation < b.rotation;
                    });

                FirstPersonSurface surface{};
                surface.gpuRegion = gpuRegion;
                surface.physicalCoverage =
                    !component.artwork.decal;
                surface.coplanarOwner =
                    FirstPersonSemanticRoleOwnsCoplanarSurface(
                        component.role);

                if (!candidates.empty())
                {
                    const auto& anchor =
                        decodedViews[
                            candidates.front().rotation];
                    surface.image =
                        anchor.component->artwork.image;
                    const int32_t width =
                        std::clamp(
                            int32_t(std::ceil(std::max(
                                FirstPersonSemanticEdgeLength(
                                    face[0], face[1]),
                                FirstPersonSemanticEdgeLength(
                                    face[3], face[2])))),
                            1, 256);
                    const int32_t height =
                        std::clamp(
                            int32_t(std::ceil(std::max(
                                FirstPersonSemanticEdgeLength(
                                    face[0], face[3]),
                                FirstPersonSemanticEdgeLength(
                                    face[1], face[2])))),
                            1, 256);

                    std::vector<uint8_t> pixels(
                        size_t(width) * size_t(height), 0);
                    const uint8_t materialFallback =
                        FirstPersonSemanticFallbackPixel(anchor);
                    for (int32_t y = 0; y < height; ++y)
                    for (int32_t x = 0; x < width; ++x)
                    {
                        const float s =
                            (float(x) + 0.5f)
                            / float(width);
                        const float t =
                            (float(y) + 0.5f)
                            / float(height);
                        const auto worldPoint =
                            FirstPersonSemanticFacePoint(
                                face, s, t);

                        uint8_t selectedPixel = 0;
                        for (const auto& candidate :
                             candidates)
                        {
                            const auto& view =
                                decodedViews[
                                    candidate.rotation];
                            if (!SameFirstPersonMaterialTemplate(
                                    surface.image,
                                    view.component->artwork.image))
                                continue;

                            const CoordsXYZ point{
                                int32_t(std::lround(
                                    worldPoint.x)),
                                int32_t(std::lround(
                                    worldPoint.y)),
                                int32_t(std::lround(
                                    worldPoint.z)),
                            };
                            const auto iso =
                                Translate3DTo2DWithZ(
                                    candidate.rotation,
                                    point);
                            if (!FirstPersonSemanticFaceOwnedNear(
                                    depthOwners[
                                        candidate.rotation],
                                    uint32_t(faceIndex),
                                    iso.x, iso.y))
                                continue;

                            const auto sample =
                                SampleFirstPersonSemanticArtwork(
                                    view, worldPoint,
                                    candidate.rotation);
                            if (!sample.has_value())
                                continue;
                            selectedPixel = *sample;
                            break;
                        }

                        if (selectedPixel == 0
                            && surface.physicalCoverage)
                        {
                            selectedPixel =
                                materialFallback != 0
                                ? materialFallback
                                : fallbackColour;
                        }
                        pixels[
                            size_t(y) * size_t(width)
                            + size_t(x)] =
                            selectedPixel;
                    }

                    bool hasPixel = false;
                    for (const auto pixel : pixels)
                    {
                        if (pixel != 0)
                        {
                            hasPixel = true;
                            break;
                        }
                    }

                    if (hasPixel)
                    {
                        surface.immutablePixels =
                            std::move(pixels);
                        surface.immutableWidth =
                            int16_t(width);
                        surface.immutableHeight =
                            int16_t(height);
                        surface.persistentBitmap = true;

                        uint64_t fingerprint =
                            14695981039346656037ull;
                        ExtendStableKey(
                            fingerprint,
                            surface.image.GetRemap());
                        ExtendStableKey(
                            fingerprint,
                            surface.image.HasPrimary()
                                ? EnumValue(
                                    surface.image.GetPrimary())
                                : 0xffu);
                        ExtendStableKey(
                            fingerprint,
                            surface.image.HasSecondary()
                                ? EnumValue(
                                    surface.image.GetSecondary())
                                : 0xffu);
                        ExtendStableKey(
                            fingerprint,
                            surface.image.HasTertiary()
                                ? EnumValue(
                                    surface.image.GetTertiary())
                                : 0xffu);
                        ExtendStableKey(
                            fingerprint, width);
                        ExtendStableKey(
                            fingerprint, height);
                        for (const auto pixel :
                             surface.immutablePixels)
                        {
                            fingerprint ^= pixel;
                            fingerprint *= 1099511628211ull;
                        }
                        surface.immutableFingerprint =
                            fingerprint;

                        const std::array<
                            FirstPersonVertex, 4> vertices{ {
                            { face[0], 0.0f, 0.0f },
                            { face[1], float(width), 0.0f },
                            { face[2], float(width), float(height) },
                            { face[3], 0.0f, float(height) },
                        } };
                        EmitQuad(surface, vertices);
                        result.emplace_back(
                            std::move(surface));
                        continue;
                    }
                }

                surface.solidColour =
                    fallbackColour != 0
                    ? fallbackColour : 1;
                std::array<FirstPersonVertex, 4> vertices{};
                for (size_t i = 0; i < face.size(); ++i)
                    vertices[i].world = face[i];
                EmitQuad(surface, vertices);
                result.emplace_back(std::move(surface));
            }
            return result;
        }

        struct FirstPersonSemanticViewGroup
        {
            uint64_t key = 0;
            const FirstPersonPaintSemanticComponent*
                canonical = nullptr;
            std::array<
                const FirstPersonPaintSemanticComponent*, 4>
                views{};
        };

        [[nodiscard]] std::vector<FirstPersonSemanticViewGroup>
            BuildFirstPersonSemanticViewGroups(
                const StaticPaintCacheEntry& cached)
        {
            std::vector<FirstPersonSemanticViewGroup> result;
            uint8_t canonicalRotation = 0xff;
            uint32_t newestGeneration = 0;
            for (uint8_t rotation = 0;
                 rotation < 4; ++rotation)
            {
                const auto& variant =
                    cached.rotations[rotation];
                if (!variant.semanticValid
                    || variant.semanticComponents.empty())
                    continue;
                if (canonicalRotation >= 4
                    || variant.lastAnimationGeneration
                        > newestGeneration)
                {
                    canonicalRotation = rotation;
                    newestGeneration =
                        variant.lastAnimationGeneration;
                }
            }
            if (canonicalRotation >= 4)
                return result;

            std::unordered_map<uint64_t, size_t>
                groupIndex;
            const auto& canonicalVariant =
                cached.rotations[canonicalRotation];
            const uint32_t canonicalGeneration =
                canonicalVariant.lastAnimationGeneration;
            std::unordered_map<uint64_t, uint32_t>
                canonicalOccurrences;
            for (const auto& component :
                 canonicalVariant.semanticComponents)
            {
                const uint64_t physical =
                    FirstPersonSemanticPhysicalFingerprint(
                        component);
                const uint32_t occurrence =
                    canonicalOccurrences[physical]++;
                uint64_t groupKey = physical;
                ExtendStableKey(groupKey, occurrence);

                const size_t index = result.size();
                groupIndex.emplace(groupKey, index);
                result.push_back({});
                auto& group = result.back();
                group.key = groupKey;
                group.canonical = &component;
                group.views[
                    component.artwork.sourceRotation & 3u] =
                    &component;
            }

            for (uint8_t rotation = 0;
                 rotation < 4; ++rotation)
            {
                if (rotation == canonicalRotation)
                    continue;
                const auto& variant =
                    cached.rotations[rotation];
                if (!variant.semanticValid
                    || variant.lastAnimationGeneration
                        != canonicalGeneration)
                    continue;
                std::unordered_map<uint64_t, uint32_t>
                    occurrences;
                for (const auto& component :
                     variant.semanticComponents)
                {
                    const uint64_t physical =
                        FirstPersonSemanticPhysicalFingerprint(
                            component);
                    const uint32_t occurrence =
                        occurrences[physical]++;
                    uint64_t groupKey = physical;
                    ExtendStableKey(groupKey, occurrence);
                    const auto found =
                        groupIndex.find(groupKey);
                    if (found == groupIndex.end())
                        continue;
                    result[found->second].views[
                        component.artwork.sourceRotation & 3u] =
                        &component;
                }
            }

            std::sort(
                result.begin(), result.end(),
                [](const FirstPersonSemanticViewGroup& a,
                   const FirstPersonSemanticViewGroup& b) {
                    return a.key < b.key;
                });
            return result;
        }

        [[nodiscard]] uint64_t
            FirstPersonSemanticViewSetFingerprint(
                const StaticPaintCacheEntry& cached)
        {
            uint64_t fingerprint =
                14695981039346656037ull;
            for (uint8_t rotation = 0;
                 rotation < 4; ++rotation)
            {
                const auto& variant =
                    cached.rotations[rotation];
                ExtendStableKey(
                    fingerprint,
                    variant.semanticValid ? 1 : 0);
                if (!variant.semanticValid)
                    continue;
                ExtendStableKey(
                    fingerprint,
                    variant.lastAnimationGeneration);
                ExtendStableKey(
                    fingerprint,
                    variant.semanticFingerprint);
            }
            return fingerprint;
        }

        void EnsureFirstPersonSemanticSurfaceCache(
            StaticPaintCacheEntry& cached)
        {
            const uint64_t fingerprint =
                FirstPersonSemanticViewSetFingerprint(
                    cached);
            if (cached.semanticSurfaceFingerprint
                == fingerprint)
                return;

            cached.semanticResidentSurfaces.clear();
            cached.semanticStreamedSurfaces.clear();

            const auto semanticGroups =
                BuildFirstPersonSemanticViewGroups(cached);
            for (const auto& group : semanticGroups)
            {
                const auto* canonical =
                    group.canonical;
                if (canonical == nullptr)
                    continue;
                const bool moving =
                    canonical->role
                        == FirstPersonPaintSemanticRole::
                            movingMachinery
                    || canonical->role
                        == FirstPersonPaintSemanticRole::seat;
                if (moving)
                    continue;

                const int32_t componentTileX =
                    canonical->mapPosition.x
                    / kCoordsXYStep;
                const int32_t componentTileY =
                    canonical->mapPosition.y
                    / kCoordsXYStep;
                const uint64_t componentRegion =
                    FirstPersonGpuRegionKey(
                        componentTileX,
                        componentTileY);

                std::vector<
                    const FirstPersonPaintSemanticComponent*>
                    depthPeers;
                if (canonical->artwork.group != 0)
                {
                    for (const auto& peerGroup :
                         semanticGroups)
                    {
                        const auto* peer =
                            peerGroup.canonical;
                        if (peer != nullptr
                            && peer->artwork.group
                                == canonical->artwork.group)
                        {
                            depthPeers.push_back(peer);
                        }
                    }
                }

                auto surfaces =
                    BuildFirstPersonSemanticComponentSurfaces(
                        *canonical,
                        componentRegion,
                        group.views,
                        depthPeers.empty()
                            ? nullptr : &depthPeers);
                for (auto& surface : surfaces)
                {
                    if (IsResidentStaticSurface(surface))
                    {
                        cached.semanticResidentSurfaces
                            .emplace_back(
                                std::move(surface));
                    }
                    else
                    {
                        cached.semanticStreamedSurfaces
                            .emplace_back(
                                std::move(surface));
                    }
                }
            }
            cached.semanticSurfaceFingerprint =
                fingerprint;
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

                if (auto supportIt =
                        _staticPaintCache.find(tileKey);
                    supportIt != _staticPaintCache.end()
                    && supportIt->second.valid
                    && !supportIt->second.dirty)
                {
                    EnsureFirstPersonSemanticSurfaceCache(
                        supportIt->second);
                    for (const auto& surface :
                         supportIt->second
                             .semanticResidentSurfaces)
                    {
                        if (surface.gpuRegion == regionKey)
                            addSurface(surface);
                    }
                }

                const auto cacheIt = _staticPaintCache.find(tileKey);
                if (cacheIt == _staticPaintCache.end() || !cacheIt->second.valid
                    || cacheIt->second.dirty || cacheIt->second.animated)
                    continue;
                const auto& cached = cacheIt->second;
                for (const auto& surface :
                     cached.cameraIndependentResidentSurfaces)
                    addSurface(surface);

                EnsureFirstPersonSemanticSurfaceCache(cached);
                for (const auto& surface :
                     cached.semanticStreamedSurfaces)
                {
                    if (SurfaceMayBeVisible(
                            surface, worldFrustum))
                    {
                        scene.surfaces.push_back(surface);
                    }
                }

                for (uint8_t rotation = 0; rotation < 4; ++rotation)
                {
                    const auto& variant = cached.rotations[rotation];
                    if (!variant.valid)
                        continue;
                    for (const auto& surface :
                         variant.residentSurfaces)
                    {
                        if (surface.reconstructionGroup != 0)
                        {
                            const auto trajectory =
                                _trackTrajectoryCache.find(
                                    surface.reconstructionGroup);
                            if (trajectory != _trackTrajectoryCache.end()
                                && !trajectory->second.dirty
                                && !trajectory->second.surfaces.empty())
                            {
                                // Trajectory-backed track is fully
                                // camera-independent here. Its native
                                // PaintStruct survives only as artwork evidence
                                // and must not enter the fallback source-view
                                // dependency set.
                                continue;
                            }
                        }

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
                        if (selected != rotation)
                            continue;
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
                    // Track geometry and its face-selected native
                    // material are both camera-independent.
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
                            const auto& sourceSurface =
                                trajectory->second
                                    .surfaces[index];
                            if (sourceSurface.artworkCarrier)
                            {
                                std::vector<FirstPersonSurface>
                                    projected;
                                AppendFirstPersonTrackArtworkProjections(
                                    sourceSurface,
                                    trajectory->second,
                                    projected);
                                for (const auto& surface :
                                     projected)
                                    addSurface(surface);
                            }
                            else
                            {
                                addSurface(sourceSurface);
                            }
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

        [[nodiscard]] bool
            FirstPersonTerrainEdgeEvidenceNeeded(CoordsXY tile)
        {
            const auto* surface =
                MapGetSurfaceElementAt(tile);
            if (surface == nullptr)
                return false;
            const auto self =
                GetSlopeCornerHeights(
                    surface->getBaseZ(),
                    surface->getSlope());

            const uint16_t waterHeight =
                surface->getWaterHeight();
            if (waterHeight > 0)
            {
                for (const CoordsXY delta : {
                         CoordsXY{ kCoordsXYStep, 0 },
                         CoordsXY{ -kCoordsXYStep, 0 },
                         CoordsXY{ 0, kCoordsXYStep },
                         CoordsXY{ 0, -kCoordsXYStep } })
                {
                    const CoordsXY neighbourTile =
                        tile + delta;
                    const auto* neighbour =
                        MapIsLocationValid(neighbourTile)
                        ? MapGetSurfaceElementAt(
                            neighbourTile)
                        : nullptr;
                    if (neighbour == nullptr
                        || neighbour->getWaterHeight()
                            != waterHeight)
                        return true;
                }
            }

            const auto exposedAgainst =
                [&](CoordsXY delta,
                    int32_t selfA, int32_t selfB,
                    auto neighbourA, auto neighbourB) {
                    const CoordsXY neighbourTile =
                        tile + delta;
                    if (!MapIsLocationValid(neighbourTile))
                        return true;
                    const auto* neighbour =
                        MapGetSurfaceElementAt(
                            neighbourTile);
                    if (neighbour == nullptr)
                        return true;
                    const auto other =
                        GetSlopeCornerHeights(
                            neighbour->getBaseZ(),
                            neighbour->getSlope());
                    return selfA > neighbourA(other)
                        || selfB > neighbourB(other);
                };

            return exposedAgainst(
                       { kCoordsXYStep, 0 },
                       self.east, self.north,
                       [](const auto& h) { return h.south; },
                       [](const auto& h) { return h.west; })
                || exposedAgainst(
                       { -kCoordsXYStep, 0 },
                       self.west, self.south,
                       [](const auto& h) { return h.north; },
                       [](const auto& h) { return h.east; })
                || exposedAgainst(
                       { 0, kCoordsXYStep },
                       self.north, self.west,
                       [](const auto& h) { return h.east; },
                       [](const auto& h) { return h.south; })
                || exposedAgainst(
                       { 0, -kCoordsXYStep },
                       self.south, self.east,
                       [](const auto& h) { return h.west; },
                       [](const auto& h) { return h.north; });
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
            constexpr uint64_t kSemanticProbeInterval = 240;
            std::array<std::unordered_set<uint64_t>, 4> missesByRotation;
            for (auto& misses : missesByRotation)
                misses.reserve(scene.visibleTiles.size() / 16 + 1);
            std::unordered_set<uint64_t>
                cameraIndependentRefreshKeys;
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
                    cached.lastSemanticGeneration, kSemanticProbeInterval);
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
                    cached.cameraIndependentResidentSurfaces.clear();
                    cached.cameraIndependentStreamedSurfaces.clear();
                    cached.cameraIndependentFingerprint = 0;
                    cached.semanticSurfaceFingerprint = 0;
                    cached.semanticResidentSurfaces.clear();
                    cached.semanticStreamedSurfaces.clear();
                    WithdrawFirstPersonSemanticComponents(tile);
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
                        variant.residentFingerprint = 0;
                        variant.semanticFingerprint = 0;
                        variant.semanticValid = false;
                        variant.verticalTunnelHeight = 0xFF;
                        variant.residentSurfaces.clear();
                        variant.streamedSurfaces.clear();
                        variant.semanticComponents.clear();
                    }
                    MarkStaticRegionDirtyForTile(tx, ty);
                }

                const auto previousRotation = cached.hasSelectedRotation
                    ? std::optional<uint8_t>{ cached.selectedRotation }
                    : std::nullopt;
                const auto tileRotation = PaintRotationForTile(opt.camera, tile, previousRotation);
                if ((!cached.hasSelectedRotation
                        || cached.selectedRotation != tileRotation)
                    && cached.hasUngroupedResident)
                    MarkStaticRegionDirtyForTile(tx, ty);
                cached.selectedRotation = tileRotation;
                cached.hasSelectedRotation = true;
                cached.lastSeen = frame;

                uint8_t rotationMask = uint8_t(1u << tileRotation);
                if (FirstPersonTerrainEdgeEvidenceNeeded(tile)
                    || (opt.viewFlags
                        & VIEWPORT_FLAG_CLIP_VIEW) != 0)
                {
                    // Every world edge is a native "bottom" edge in at least
                    // one quarter-turn. Capture all four once so cliff geometry
                    // and face artwork exist before passenger movement can
                    // influence source-view selection.
                    rotationMask |= 0x0Fu;
                }
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

                    if (group.type == TileElementType::track)
                    {
                        const auto trajectory =
                            _trackTrajectoryCache.find(group.key);
                        if (trajectory
                            != _trackTrajectoryCache.end()
                            && !trajectory->second.dirty)
                        {
                            for (uint8_t sourceRotation = 0;
                                 sourceRotation < 4;
                                 ++sourceRotation)
                            {
                                if (!trajectory->second
                                         .artworkCaptureAttempted[
                                             sourceRotation])
                                {
                                    rotationMask |= uint8_t(
                                        1u << sourceRotation);
                                }
                            }
                        }
                    }

                }

                for (uint8_t rotation = 0; rotation < 4; ++rotation)
                {
                    if ((rotationMask & uint8_t(1u << rotation)) == 0)
                        continue;
                    auto& variant = cached.rotations[rotation];
                    const uint32_t animationGeneration =
                        sourceGeneration;
                    // Static variants are invalidated by authoritative map
                    // invalidation or the staggered semantic signature probe
                    // above. Do not repaint unchanged static artwork merely
                    // because time passed.
                    const bool stale =
                        !variant.valid
                        || (cached.animated
                            && variant.lastAnimationGeneration
                                != animationGeneration);
                    if (stale)
                    {
                        if (cameraIndependentRefreshKeys
                                .insert(key).second)
                        {
                            cached.cameraIndependentResidentSurfaces.clear();
                            cached.cameraIndependentStreamedSurfaces.clear();
                        }
                        variant.valid = true;
                        variant.lastPainted = frame;
                        variant.lastAnimationGeneration = animationGeneration;
                        variant.residentSurfaces.clear();
                        variant.streamedSurfaces.clear();
                        variant.semanticComponents.clear();
                        variant.semanticValid = false;
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
            std::unordered_map<uint64_t, CoordsXY>
                semanticCompletionWork;
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
                        for (const auto key : cameraIndependentRefreshKeys)
                        {
                            MarkStaticRegionDirtyForTile(
                                int32_t(key >> 32),
                                int32_t(key & 0xffffffffu));
                        }
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
                            std::vector<FirstPersonPaintSemanticComponent>
                                semanticComponents;
                            session->CurrentSource = PaintStructSource::tile;
                            session->FirstPersonSemanticComponentSink =
                                &semanticComponents;
                            TileElementPaintSetup(*session, item.position);
                            session->FirstPersonSemanticComponentSink =
                                nullptr;

                            // Native painters publish semantic geometry while
                            // exact placement and animation transforms are known.
                            auto cacheIt = _staticPaintCache.find(item.key);
                            if (cacheIt != _staticPaintCache.end())
                            {
                                auto& variant =
                                    cacheIt->second.rotations[rotation];
                                const uint64_t previousSemanticFingerprint =
                                    variant.semanticFingerprint;
                                variant.semanticComponents =
                                    std::move(semanticComponents);
                                variant.semanticValid = true;
                                variant.semanticFingerprint =
                                    SemanticComponentFingerprint(
                                        variant.semanticComponents);
                                if (!variant.semanticComponents.empty())
                                {
                                    semanticCompletionWork.try_emplace(
                                        item.key, item.position);
                                }
                                if (previousSemanticFingerprint
                                    != variant.semanticFingerprint)
                                {
                                    MarkStaticRegionDirtyForTile(
                                        item.position.x / kCoordsXYStep,
                                        item.position.y / kCoordsXYStep);
                                }
                                // Walking collision needs only stable physical
                                // geometry; any native rotation is equivalent
                                // for that purpose.
                                PublishFirstPersonSemanticComponents(
                                    item.position,
                                    variant.semanticComponents);
                                variant.verticalTunnelHeight = session->VerticalTunnelHeight;
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
                    for (auto* root = session->PaintHead; root; root = root->NextQuadrantEntry)
                    {
                        const bool dynamic = IsFirstPersonEntityPaintRoot(*root);
                        const uint64_t key = TerrainKey(
                            root->MapPos.x / kCoordsXYStep, root->MapPos.y / kCoordsXYStep);
                        if (!dynamic && !missesByRotation[rotation].contains(key))
                            continue;

                        if (root->Element != nullptr
                            && root->Element->getType()
                                == TileElementType::surface
                            && root->FirstPersonSemanticRole
                                == FirstPersonPaintSemanticRole::none)
                        {
                            // Terrain/water tops are reconstructed directly
                            // from map geometry. Surface-painter artwork enters
                            // first person only when the native painter has
                            // explicitly attached a semantic physical meaning.
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
                        AppendRoot(
                            scene, *root, anchor, basis, isoAnchor,
                            opt.viewFlags, opt.hiddenEntity, opt.hiddenSeatIndex,
                            rotation);
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
                            const bool trackRoot =
                                root->Element != nullptr
                                && root->Element->getType()
                                    == TileElementType::track;
                            for (size_t i = startSurface; i < scene.surfaces.size(); ++i)
                            {
                                scene.surfaces[i].reconstructionGroup = reconstruction.groupKey;
                                if (trackRoot)
                                {
                                    CaptureFirstPersonTrackArtworkProjection(
                                        scene.surfaces[i], rotation);
                                }
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

                            if (trackRoot
                                && reconstruction.groupKey != 0)
                            {
                                const auto trajectory =
                                    _trackTrajectoryCache.find(
                                        reconstruction.groupKey);
                                if (trajectory
                                    != _trackTrajectoryCache.end())
                                {
                                    trajectory->second
                                        .artworkCaptureAttempted[
                                            rotation] = true;
                                }
                            }

                            if (cacheIt != _staticPaintCache.end())
                            {
                                auto& variant = cacheIt->second.rotations[rotation];
                                for (size_t i = startSurface; i < scene.surfaces.size(); ++i)
                                {
                                    auto& surface = scene.surfaces[i];
                                    if (surface.cameraIndependent)
                                    {
                                        auto& target =
                                            IsResidentStaticSurface(surface)
                                            ? cacheIt->second
                                                .cameraIndependentResidentSurfaces
                                            : cacheIt->second
                                                .cameraIndependentStreamedSurfaces;
                                        const uint64_t independentKey =
                                            CameraIndependentSurfaceKey(
                                                surface);
                                        const bool duplicate =
                                            std::any_of(
                                                target.begin(),
                                                target.end(),
                                                [&](const FirstPersonSurface& existing) {
                                                    return CameraIndependentSurfaceKey(
                                                        existing)
                                                        == independentKey;
                                                });
                                        if (!duplicate)
                                            target.push_back(surface);
                                        continue;
                                    }

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

            // Once a native painter proves that a tile has semantic geometry,
            // capture the remaining source rotations as ARTWORK EVIDENCE only.
            // Batch these captures by rotation: opening a fresh PaintSession for
            // every tile/view was a major cold-entry cost and provided no extra
            // correctness.
            std::array<std::vector<PaintWorkItem>, 4>
                semanticEvidenceByRotation;
            for (const auto& [key, position] :
                 semanticCompletionWork)
            {
                auto cacheIt = _staticPaintCache.find(key);
                if (cacheIt == _staticPaintCache.end()
                    || !cacheIt->second.valid
                    || cacheIt->second.dirty)
                    continue;

                for (uint8_t rotation = 0;
                     rotation < 4; ++rotation)
                {
                    const auto& variant =
                        cacheIt->second.rotations[rotation];
                    if (variant.semanticValid
                        && (!cacheIt->second.animated
                            || variant.lastAnimationGeneration
                                == sourceGeneration))
                        continue;
                    semanticEvidenceByRotation[rotation]
                        .push_back({
                            position, key, true,
                            EntityId::GetNull()
                        });
                }
            }

            for (uint8_t rotation = 0;
                 rotation < 4; ++rotation)
            {
                auto& evidence =
                    semanticEvidenceByRotation[rotation];
                if (evidence.empty())
                    continue;
                auto collection =
                    makeCollectionTarget(rotation);

                for (size_t offset = 0;
                     offset < evidence.size();
                     offset += kTilesPerPaintSession)
                {
                    const size_t end =
                        std::min(
                            offset + kTilesPerPaintSession,
                            evidence.size());
                    auto* semanticSession =
                        PaintSessionAlloc(
                            collection, opt.viewFlags,
                            rotation);
                    if (semanticSession == nullptr)
                        break;

                    Drawing::ScrollingText::
                        BeginFirstPersonSnapshotCapture();
                    for (size_t i = offset;
                         i < end; ++i)
                    {
                        const auto& item = evidence[i];
                        auto cacheIt =
                            _staticPaintCache.find(item.key);
                        if (cacheIt
                                == _staticPaintCache.end()
                            || !cacheIt->second.valid
                            || cacheIt->second.dirty)
                            continue;

                        std::vector<
                            FirstPersonPaintSemanticComponent>
                            semanticComponents;
                        semanticSession
                            ->CurrentlyDrawnEntity = nullptr;
                        semanticSession
                            ->CurrentlyDrawnTileElement = nullptr;
                        semanticSession->CurrentSource =
                            PaintStructSource::tile;
                        semanticSession
                            ->FirstPersonSemanticComponentSink =
                            &semanticComponents;
                        TileElementPaintSetup(
                            *semanticSession,
                            item.position);
                        semanticSession
                            ->FirstPersonSemanticComponentSink =
                            nullptr;

                        auto& variant =
                            cacheIt->second
                                .rotations[rotation];
                        const uint64_t
                            previousFingerprint =
                                variant
                                    .semanticFingerprint;
                        variant.semanticComponents =
                            std::move(
                                semanticComponents);
                        variant.semanticValid = true;
                        variant.lastAnimationGeneration =
                            sourceGeneration;
                        variant.semanticFingerprint =
                            SemanticComponentFingerprint(
                                variant
                                    .semanticComponents);
                        if (previousFingerprint
                            != variant
                                   .semanticFingerprint)
                        {
                            MarkStaticRegionDirtyForTile(
                                item.position.x
                                    / kCoordsXYStep,
                                item.position.y
                                    / kCoordsXYStep);
                        }
                    }
                    Drawing::ScrollingText::
                        EndFirstPersonSnapshotCapture();
                    PaintSessionFree(semanticSession);
                }
            }

            for (const auto key :
                 cameraIndependentRefreshKeys)
            {
                const auto cacheIt =
                    _staticPaintCache.find(key);
                if (cacheIt == _staticPaintCache.end())
                    continue;
                auto& cached = cacheIt->second;
                const uint64_t fingerprint =
                    cached.cameraIndependentResidentSurfaces.empty()
                    ? 0
                    : ResidentStaticSurfaceFingerprint(
                        cached.cameraIndependentResidentSurfaces);
                if (cached.cameraIndependentFingerprint
                    != fingerprint)
                {
                    MarkStaticRegionDirtyForTile(
                        int32_t(key >> 32),
                        int32_t(key & 0xffffffffu));
                }
                cached.cameraIndependentFingerprint =
                    fingerprint;
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
                for (const auto& staticSurface :
                     cached.cameraIndependentStreamedSurfaces)
                {
                    if (SurfaceMayBeVisible(
                            staticSurface, worldFrustum))
                    {
                        scene.surfaces.emplace_back(
                            staticSurface);
                    }
                }
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
                            if (staticSurface.reconstructionGroup != 0)
                            {
                                const auto trajectory =
                                    _trackTrajectoryCache.find(
                                        staticSurface.reconstructionGroup);
                                if (trajectory != _trackTrajectoryCache.end()
                                    && !trajectory->second.dirty
                                    && !trajectory->second.surfaces.empty())
                                {
                                    // Native track sprites are retained only
                                    // as texture evidence. Rendering them as
                                    // view-facing geometry would reintroduce
                                    // the morphing impostor track.
                                    continue;
                                }
                            }
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

            // Moving semantic components use the newest semantic transform as
            // their canonical physical model. Native rotations contribute only
            // artwork evidence, so head/camera direction cannot choose geometry.
            for (const auto tile : scene.visibleTiles)
            {
                const auto key = TerrainKey(
                    tile.x / kCoordsXYStep,
                    tile.y / kCoordsXYStep);
                const auto cacheIt =
                    _staticPaintCache.find(key);
                if (cacheIt == _staticPaintCache.end()
                    || !cacheIt->second.valid
                    || cacheIt->second.dirty)
                    continue;

                const auto semanticGroups =
                    BuildFirstPersonSemanticViewGroups(
                        cacheIt->second);
                for (const auto& group :
                     semanticGroups)
                {
                    const auto* canonical =
                        group.canonical;
                    if (canonical == nullptr
                        || (canonical->role
                                != FirstPersonPaintSemanticRole::movingMachinery
                            && canonical->role
                                != FirstPersonPaintSemanticRole::seat))
                        continue;

                    std::vector<
                        const FirstPersonPaintSemanticComponent*>
                        depthPeers;
                    if (canonical->artwork.group != 0)
                    {
                        for (const auto& peerGroup :
                             semanticGroups)
                        {
                            const auto* peer =
                                peerGroup.canonical;
                            if (peer != nullptr
                                && peer->artwork.group
                                    == canonical->artwork.group)
                            {
                                depthPeers.push_back(peer);
                            }
                        }
                    }
                    auto surfaces =
                        BuildFirstPersonSemanticComponentSurfaces(
                            *canonical, 0,
                            group.views,
                            depthPeers.empty()
                                ? nullptr : &depthPeers);
                    for (auto& surface : surfaces)
                    {
                        if (SurfaceMayBeVisible(
                                surface, worldFrustum))
                            scene.surfaces.emplace_back(
                                std::move(surface));
                    }
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

            // Static tile/region/reconstruction caches are park-bounded and
            // survive camera motion. Dynamic entity rotation state may expire.
            if (frame % 120 == 0)
            {
                std::erase_if(_entityRotations, [frame](const auto& kv) {
                    return frame - kv.second.lastSeen > 240;
                });
            }

            SubmitVisibleStaticRegions(scene, worldFrustum, frame);
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
    FirstPersonScene CollectFirstPersonScene(
        const FirstPersonRenderOptions& opt, const ScreenSize& dimensions)
    {
        FirstPersonScene scene{};
        scene.sceneEpoch = _sceneEpoch;
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
        const auto trackStart=std::chrono::steady_clock::now();
        scene.terrainCpuMs=std::chrono::duration<float,std::milli>(
            trackStart-terrainStart).count();
        CollectTrackTrajectories(scene);
        scene.trackCpuMs=std::chrono::duration<float,std::milli>(
            std::chrono::steady_clock::now()-trackStart).count();
        // This remains an explicitly identified compatibility bridge for complex sprite selection.
        return scene;
    }
    void ResetFirstPersonPresentationCache()
    {
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
        ClearLargeSceneryAssetModelCache();
        ClearFirstPersonSmallSceneryReconstructionCache();
        _largeSceneryGeometryCache.clear();
        ClearFirstPersonLargeSceneryPhysicalProxies();
        _largeSceneryGroupsByRegion.clear();
        _activeLargeSceneryRegions.clear();
        _largeSceneryGeometryEnabled = false;
        ClearFirstPersonSemanticComponents();
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
        for (int32_t ty = y0; ty <= y1; ++ty)
        for (int32_t tx = x0; tx <= x1; ++tx)
        {
            WithdrawFirstPersonSemanticComponents(
                { tx * kCoordsXYStep,
                  ty * kCoordsXYStep });
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
    void BeginFirstPersonPresentationFrame()
    {
        _preparedFrame.active = true;
        _preparedFrame.valid = false;
        _preparedFrame.scene = {};
        ++_preparedFrame.serial;
        if (_preparedFrame.serial == 0)
            ++_preparedFrame.serial;
    }

    void EndFirstPersonPresentationFrame()
    {
        _preparedFrame.active = false;
        _preparedFrame.valid = false;
        _preparedFrame.drawingEngine = nullptr;
        _preparedFrame.scene = {};
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

            const auto start =
                std::chrono::steady_clock::now();
            localScene =
                CollectFirstPersonScene(opt, dimensions);
            localScene.screenOrigin = screenOrigin;
            localScene.presentationFrameSerial =
                _preparedFrame.active
                    ? _preparedFrame.serial : 0;
            const auto paintStart =
                std::chrono::steady_clock::now();
            CollectPaintSprites(localScene, rt);
            const auto submitStart =
                std::chrono::steady_clock::now();
            localScene.paintCpuMs =
                std::chrono::duration<float, std::milli>(
                    submitStart - paintStart).count();
            localScene.prepareCpuMs =
                std::chrono::duration<float, std::milli>(
                    submitStart - start).count();

            if (_preparedFrame.active)
            {
                _preparedFrame.options = opt;
                _preparedFrame.dimensions = dimensions;
                _preparedFrame.screenOrigin = screenOrigin;
                _preparedFrame.drawingEngine =
                    rt.DrawingEngine;
                _preparedFrame.scene =
                    std::move(localScene);
                _preparedFrame.valid = true;
                scene = &_preparedFrame.scene;
            }
            else
            {
                scene = &localScene;
            }
        }

        auto* context =
            rt.DrawingEngine->GetDrawingContext();
        if (context != nullptr)
            context->DrawFirstPersonScene(rt, *scene);
    }
} // namespace OpenRCT2::Paint

