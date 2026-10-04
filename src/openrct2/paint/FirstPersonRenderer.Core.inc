        static FirstPersonWorkQueue _reconstructionWorker;
        static FirstPersonFrameBudget _snapshotBudget{0, std::chrono::microseconds(0)};
        static uint64_t _nextReconstructionRequest = 0;
        static FirstPersonFrameBudget _discoveryBudget{0, std::chrono::microseconds(0)};
        uint64_t NativeTileSignature(CoordsXY pos);

        static std::unordered_map<uint16_t, CoordsXY>
            _passengerAnchorSourceTiles;

        [[nodiscard]] std::optional<std::vector<uint8_t>>
            DecodeFirstPersonSpritePixels(const G1Element& g1);

        [[nodiscard]] bool SameFirstPersonMaterialTemplate(
            ImageId a, ImageId b);

        [[nodiscard]] constexpr uint8_t
            FirstPersonMissingMaterialPixel(int32_t x, int32_t y)
        {
            // Deliberately unmistakable indexed checkerboard for genuinely
            // unknown material. Transparent source pixels never use this.
            constexpr uint8_t dark =
                static_cast<uint8_t>(Drawing::PaletteIndex::pi10);
            constexpr uint8_t marker =
                static_cast<uint8_t>(Drawing::PaletteIndex::pi44);
            return (((x >> 2) ^ (y >> 2)) & 1) != 0
                ? marker : dark;
        }

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

            if (element->getType() == TileElementType::entrance)
            {
                auto* entrance = element->asEntrance();
                if (entrance == nullptr
                    || entrance->getEntranceType()
                        != EntranceType::parkEntrance)
                    return std::nullopt;

                const uint8_t direction =
                    static_cast<uint8_t>(
                        entrance->getDirection()) & 3u;
                const CoordsXY side =
                    CoordsDirectionDelta[
                        (direction - 1u) & 3u];
                CoordsXY centre = tile;
                switch (entrance->getSequenceIndex())
                {
                    case ParkEntranceSequence::left:
                        centre.x -= side.x;
                        centre.y -= side.y;
                        break;
                    case ParkEntranceSequence::right:
                        centre.x += side.x;
                        centre.y += side.y;
                        break;
                    default:
                        break;
                }

                const FirstPersonVec3 anchor{
                    float(centre.x),
                    float(centre.y),
                    float(entrance->getBaseZ()),
                };
                uint64_t key = 14695981039346656037ull;
                ExtendStableKey(key, 3);
                ExtendStableKey(key, uint32_t(centre.x));
                ExtendStableKey(key, uint32_t(centre.y));
                ExtendStableKey(
                    key, uint32_t(entrance->getBaseZ()));
                ExtendStableKey(key, direction);
                ExtendStableKey(
                    key,
                    entrance->getEntryIndex());
                if (key == 0)
                    key = 1;
                return ReconstructionGroupInfo{
                    key, anchor, tile,
                    TileElementType::entrance
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

        struct VehicleHullJob
        {
            CarEntry entry{};
            FirstPersonSpriteSnapshot sprites;
            FirstPersonVehicleBodyHull hull;
            bool submitted = false;
            bool ready = false;
        };
        static std::unordered_map<uint32_t, std::shared_ptr<VehicleHullJob>> _vehicleHullJobs;

        const FirstPersonVehicleBodyHull* RequestVehicleHull(const CarEntry& entry)
        {
            if (!entry.groupEnabled(SpriteGroupType::slopeFlat)) return nullptr;
            auto found = _vehicleHullJobs.find(entry.baseImageId);
            if (found == _vehicleHullJobs.end())
            {
                if (!_snapshotBudget.available() || !_reconstructionWorker.hasCapacity()) return nullptr;
                auto job = std::make_shared<VehicleHullJob>();
                job->entry = entry;
                found = _vehicleHullJobs.emplace(entry.baseImageId, std::move(job)).first;
            }
            const auto job = found->second;
            if (job->ready) return job->hull.valid ? &job->hull : nullptr;
            if (!job->submitted)
            {
                for (uint8_t direction = 0; direction < 32; ++direction)
                    if (!job->sprites.capture(entry.getSpriteOffset(SpriteGroupType::slopeFlat, direction, 0),
                            _snapshotBudget)) return nullptr;
                job->submitted = _reconstructionWorker.submit([job] {
                    auto hull = BuildFirstPersonVehicleBodyHullFromSnapshot(job->entry,
                        [&](ImageIndex id) { return job->sprites.get(id); });
                    return [job, hull = std::move(hull)]() mutable {
                        job->hull = std::move(hull);
                        job->ready = true;
                    };
                }, [job] { job->ready = true; });
            }
            return nullptr;
        }

        [[nodiscard]] const FirstPersonVehicleBodyHull*
            GetUsableVehicleHull(const Vehicle& vehicle)
        {
            const auto visual =
                ResolveVehicleVisualState(vehicle);
            const auto* entry = visual.carEntry;
            if (entry == nullptr)
                return nullptr;
            return RequestVehicleHull(*entry);
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

        bool BuildSmallSceneryGeometry(FirstPersonScene& scene,
            const SmallSceneryElement& element, const SmallSceneryEntry& metadata,
            ImageId image, const FirstPersonSpriteSnapshot& sprites)
        {
            const auto* small = &element;
            const auto* entry = &metadata;
            const auto ownedHull = BuildFirstPersonSmallSceneryVisualHullFromSnapshot(
                metadata, element, [&](ImageIndex id) { return sprites.get(id); });
            const auto* hull = &ownedHull;
            if (!hull->valid) return false;
            const float tileX = 0, tileY = 0, baseZ = 0;
            // Face reprojection is an asset/state operation, not an instance
            // operation. Identical scenery instances can share the expensive
            // local-space bake and apply only their world translation and
            // recolour material at submission time.
            struct BakedSmallSceneryEntry
            {
                bool resolved = false;
                bool usable = false;
                std::vector<FirstPersonSurface> localSurfaces;
            };
            BakedSmallSceneryEntry baked;

            const auto faces =
                BuildFirstPersonVisualHullBoundaryFaces(*hull);
            if (faces.empty())
            {
                baked.resolved = true;
                baked.usable = false;
                return false;
            }

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

            struct SmallSceneryDecodedView
            {
                ImageId image{};
                const G1Element* g1 = nullptr;
                uint8_t rotation = 0;
                std::vector<uint8_t> pixels;
            };
            std::vector<SmallSceneryDecodedView>
                decodedViews;
            decodedViews.reserve(
                hull->textureViews.size());
            for (const auto& view : hull->textureViews)
            {
                const auto sourceImage =
                    image.WithIndex(view.image);
                const auto* g1 =
                    sprites.get(sourceImage);
                if (g1 == nullptr
                    || g1->width <= 0
                    || g1->height <= 0)
                    continue;
                auto decoded =
                    DecodeFirstPersonSpritePixels(*g1);
                if (!decoded.has_value())
                    continue;
                decodedViews.push_back({
                    sourceImage,
                    g1,
                    view.imageDirection,
                    std::move(*decoded),
                });
            }

            struct SmallSceneryFaceView
            {
                const SmallSceneryDecodedView* source = nullptr;
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

            const size_t surfaceStart = scene.surfaces.size();
            size_t emittedFaces = 0;
            for (size_t faceIndex = 0;
                 faceIndex < faces.size(); ++faceIndex)
            {
                const auto& face = faces[faceIndex];
                std::vector<SmallSceneryFaceView> candidates;
                for (const auto& source : decodedViews)
                {
                    if (!faceVisible(
                            face.normal,
                            source.rotation))
                        continue;

                    std::array<ScreenCoordsXY, 4> projected{};
                    for (size_t i = 0;
                         i < face.corners.size(); ++i)
                    {
                        projected[i] =
                            Translate3DTo2DWithZ(
                                source.rotation,
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
                            depthOwners[source.rotation],
                            uint32_t(faceIndex),
                            silhouette);
                    if (ownership < 0.20f)
                        continue;

                    const auto paintOffset =
                        FirstPersonSmallSceneryPaintOffset(
                            *entry, *small,
                            source.rotation);
                    const auto sourceOrigin =
                        Translate3DTo2DWithZ(
                            source.rotation,
                            { paintOffset, 0 });
                    const float sourceCoverage =
                        FirstPersonSilhouettePredicateCoverage(
                            silhouette,
                            [&](int32_t x, int32_t y) {
                                const int32_t u =
                                    x - sourceOrigin.x
                                    - source.g1->xOffset;
                                const int32_t v =
                                    y - sourceOrigin.y
                                    - source.g1->yOffset;
                                if (u < 0 || v < 0
                                    || u >= source.g1->width
                                    || v >= source.g1->height)
                                    return false;
                                if (!source.g1->flags.has(
                                        G1Flag::hasTransparency))
                                    return true;
                                return source.pixels[
                                    size_t(v)
                                        * size_t(
                                            source.g1->width)
                                    + size_t(u)] != 0;
                            });
                    if (sourceCoverage <= 0.0f)
                        continue;

                    const bool reliable =
                        FirstPersonTextureReprojectionIsReliable(
                            ownership, sourceCoverage);
                    candidates.push_back({
                        &source,
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
                        return a.source->rotation
                            < b.source->rotation;
                    });

                FirstPersonSurface surface{};
                // The hull is physical/collision evidence; its sprite material
                // is still alpha-tested artwork. Do not turn transparent sprite
                // pixels into visible matter merely because a hull face exists.
                surface.cameraIndependent = true;
                surface.exteriorOnly = true;
                surface.outwardNormal = face.normal;

                if (!candidates.empty())
                {
                    surface.image =
                        candidates.front().source->image;
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

                    std::vector<uint8_t> pixels(
                        size_t(width) * size_t(height), 0);
                    size_t visiblePixelCount = 0;
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
                            const auto& sourceView =
                                *candidate.source;
                            if (!SameFirstPersonMaterialTemplate(
                                    surface.image,
                                    sourceView.image))
                                continue;

                            const auto projected =
                                Translate3DTo2DWithZ(
                                    sourceView.rotation,
                                    {
                                        int32_t(std::lround(
                                            point.x)),
                                        int32_t(std::lround(
                                            point.y)),
                                        int32_t(std::lround(
                                            point.z)),
                                    });
                            if (!ownedNear(
                                    sourceView.rotation,
                                    uint32_t(faceIndex),
                                    projected.x,
                                    projected.y))
                                continue;

                            const auto paintOffset =
                                FirstPersonSmallSceneryPaintOffset(
                                    *entry, *small,
                                    sourceView.rotation);
                            const auto sourceOrigin =
                                Translate3DTo2DWithZ(
                                    sourceView.rotation,
                                    { paintOffset, 0 });
                            const int32_t u =
                                projected.x - sourceOrigin.x
                                - sourceView.g1->xOffset;
                            const int32_t v =
                                projected.y - sourceOrigin.y
                                - sourceView.g1->yOffset;
                            if (u < 0 || v < 0
                                || u >= sourceView.g1->width
                                || v >= sourceView.g1->height)
                                continue;

                            const uint8_t pixel =
                                sourceView.pixels[
                                    size_t(v)
                                        * size_t(
                                            sourceView.g1->width)
                                    + size_t(u)];
                            if (pixel == 0)
                                continue;
                            selectedPixel = pixel;
                            break;
                        }

                        // Zero is meaningful visual evidence: the native
                        // sprite has no material at this projected point.
                        // Preserve it instead of painting the hull/collision
                        // volume with a guessed dominant colour.
                        pixels[
                            size_t(y) * size_t(width)
                            + size_t(x)] =
                            selectedPixel;
                        if (selectedPixel != 0)
                            ++visiblePixelCount;
                    }

                    if (visiblePixelCount == 0)
                        continue;

                    uint64_t fingerprint =
                        14695981039346656037ull;
                    ExtendStableKey(fingerprint, width);
                    ExtendStableKey(fingerprint, height);
                    for (const auto pixel : pixels)
                    {
                        fingerprint ^= pixel;
                        fingerprint *= 1099511628211ull;
                    }
                    surface.persistentPixels =
                        std::make_shared<
                            std::vector<uint8_t>>(
                                std::move(pixels));
                    surface.immutableWidth =
                        int16_t(width);
                    surface.immutableHeight =
                        int16_t(height);
                    surface.persistentBitmap = true;
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
                    ++emittedFaces;
                    continue;
                }

                // Native sprites normally provide no underside observation.
                // An unobserved bottom face may exist physically for collision,
                // but inventing a flat material for it is not visual evidence.
                if (face.kind
                    == FirstPersonVisualHullFaceKind::bottom)
                    continue;

                // A normally visible face with no native-view material means
                // reconstruction is incomplete. Roll back this object's partial
                // mesh and preserve the ordinary native sprite fallback rather
                // than showing guessed solid-colour geometry.
                scene.surfaces.resize(surfaceStart);
                baked.localSurfaces.clear();
                baked.resolved = true;
                baked.usable = false;
                return false;
            }

            baked.localSurfaces.clear();
            if (emittedFaces != 0)
            {
                baked.localSurfaces.reserve(
                    scene.surfaces.size()
                    - surfaceStart);
                for (size_t i = surfaceStart;
                     i < scene.surfaces.size(); ++i)
                {
                    auto local =
                        scene.surfaces[i];
                    for (auto& vertex :
                         local.triangles)
                    {
                        vertex.world.x -= tileX;
                        vertex.world.y -= tileY;
                        vertex.world.z -= baseZ;
                    }
                    baked.localSurfaces.emplace_back(
                        std::move(local));
                }
            }
            baked.resolved = true;
            baked.usable = emittedFaces != 0;
            return baked.usable;
        }

        struct SmallSceneryJob
        {
            SmallSceneryEntry entry{};
            SmallSceneryElement element{};
            FirstPersonSpriteSnapshot sprites;
            std::vector<FirstPersonSurface> surfaces;
            std::unordered_set<uint64_t> waitingTiles;
            bool submitted = false;
            bool ready = false;
        };
        static std::unordered_map<uint64_t, std::shared_ptr<SmallSceneryJob>> _smallSceneryJobs;

        bool AppendCalibratedSmallSceneryGeometry(
            FirstPersonScene& scene, const PaintStruct& ps, ImageId image,
            const ScreenCoordsXY&, uint8_t rotation, ImageId mask = {})
        {
            const auto* small = ps.Element ? ps.Element->asSmallScenery() : nullptr;
            if (small == nullptr || !image.HasValue() || mask.HasValue()) return false;
            const auto* entry = small->getEntry();
            if (entry == nullptr) return false;
            const auto stage = FirstPersonSmallSceneryWitherStage(*entry, *small);
            if (image.GetIndex() != entry->image + (small->getDirectionWithOffset(rotation) & 3u)
                    + uint32_t(stage) * 4u) return false;
            uint64_t key = entry->image;
            ExtendStableKey(key, small->getDirection());
            ExtendStableKey(key, small->getSceneryQuadrant());
            ExtendStableKey(key, small->getOccupiedQuadrants());
            ExtendStableKey(key, stage);
            auto it = _smallSceneryJobs.find(key);
            if (it == _smallSceneryJobs.end())
            {
                if (!_snapshotBudget.available() || !_reconstructionWorker.hasCapacity())
                {
                    // Revisit this native view next frame; never fall through
                    // to a synchronous silhouette reconstruction.
                    InvalidateFirstPersonSceneTile(ps.MapPos);
                    return true;
                }
                auto job = std::make_shared<SmallSceneryJob>();
                job->entry = *entry;
                job->entry.frame_offsets = nullptr;
                job->entry.FrameOffsetCount = 0;
                job->element = *small;
                it = _smallSceneryJobs.emplace(key, std::move(job)).first;
            }
            const auto job = it->second;
            if (job->ready)
            {
                if (job->surfaces.empty()) return false;
                for (const auto& local : job->surfaces)
                {
                    auto surface = local;
                    surface.image = image.WithIndex(local.image.GetIndex());
                    for (auto& vertex : surface.triangles)
                    {
                        vertex.world.x += float(ps.MapPos.x);
                        vertex.world.y += float(ps.MapPos.y);
                        vertex.world.z += float(small->getBaseZ());
                    }
                    scene.surfaces.emplace_back(std::move(surface));
                }
                return true;
            }
            const auto tileKey = (uint64_t(uint32_t(ps.MapPos.x)) << 32) | uint32_t(ps.MapPos.y);
            job->waitingTiles.insert(tileKey);
            if (!job->submitted)
            {
                for (uint8_t view = 0; view < 4; ++view)
                    if (!job->sprites.capture(entry->image + view + uint32_t(stage) * 4u, _snapshotBudget))
                    {
                        InvalidateFirstPersonSceneTile(ps.MapPos);
                        return true;
                    }
                job->submitted = _reconstructionWorker.submit([job] {
                    FirstPersonScene result;
                    BuildSmallSceneryGeometry(result, job->element, job->entry,
                        ImageId(job->entry.image), job->sprites);
                    return [job, surfaces = std::move(result.surfaces)]() mutable {
                        job->surfaces = std::move(surfaces);
                        job->ready = true;
                        for (const auto tile : job->waitingTiles)
                            InvalidateFirstPersonSceneTile({int32_t(tile >> 32), int32_t(tile)});
                        job->waitingTiles.clear();
                    };
                }, [job] {
                    job->ready = true;
                    for (const auto tile : job->waitingTiles)
                        InvalidateFirstPersonSceneTile({int32_t(tile >> 32), int32_t(tile)});
                    job->waitingTiles.clear();
                });
                if (!job->submitted) InvalidateFirstPersonSceneTile(ps.MapPos);
            }
            return true; // pending geometry deliberately absent
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
            const auto nodeSurfaceStart =
                scene.surfaces.size();
            const auto applyNativePaintOrdinal = [&]() {
                if (ps.FirstPersonNativePaintOrdinal == 0)
                    return;
                for (size_t i = nodeSurfaceStart;
                     i < scene.surfaces.size(); ++i)
                {
                    if (scene.surfaces[i].nativePaintOrdinal == 0)
                    {
                        scene.surfaces[i].nativePaintOrdinal =
                            ps.FirstPersonNativePaintOrdinal;
                    }
                }
            };

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
                applyNativePaintOrdinal();
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
            applyNativePaintOrdinal();
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
                // Attached entries do not carry independent semantic/physical
                // ownership; they are pixel layers of this PaintStruct.
                applyNativePaintOrdinal();
            }
        }

        // Terrain cache keys include authoritative visible state.
        struct TerrainCacheEntry
        {
            ImageId source{};
            int32_t baseZ{}, waterZ{};
            uint8_t slope{};
            uint8_t sourceRotation{};
            uint8_t surfaceStyle{};
            uint8_t grassVisualStage{};
            bool verticalOpening = false;
            int32_t spriteX{}, spriteY{}, spriteWidth{}, spriteHeight{};
            FirstPersonSurface ground{};
            // A shallow shoreline can clip each of the terrain quad's two
            // authoritative triangles independently, so water is not always
            // representable by one full-tile quad.
            std::vector<FirstPersonSurface> water;
            std::vector<FirstPersonSurface> waterOverlay;
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
        struct MovingSemanticMesh
        {
            uint32_t componentId{};
            FirstPersonPaintSemanticRole role{};
            uint16_t repetition{};
            std::vector<FirstPersonSurface> localSurfaces;
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
            bool hasCameraDependentFallback = false;
            bool visibilityBoundValid = false;
            bool visibilityKnown = false;
            bool visibilityDirty = true;
            // Discovery may observe a newer authoritative tile before the
            // native paint/reconstruction cache consumes it. Keep that
            // observation separate from the source signature so change
            // detection never destroys its own previous-state evidence.
            uint64_t visibilitySignature = 0;
            int32_t visibilityMinZ = 0;
            int32_t visibilityMaxZ = 0;
            uint32_t lastVisibilityGeneration = 0;
            uint64_t cameraIndependentFingerprint = 0;
            uint64_t semanticSurfaceFingerprint = 0;
            uint64_t semanticRequest = 0;
            uint64_t sourceRevision = 0;
            std::vector<MovingSemanticMesh> semanticMovingMeshes;
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
        // Advances only when at least one reconstruction group changes its
        // selected native source view. Static regions can skip hashing every
        // dependency while this remains unchanged.
        static uint64_t _reconstructionSelectionSerial = 1;

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
            // Keep slots and their entity-vector capacities across simulation
            // generations. Only regions active in the previous generation need
            // clearing before the next authoritative rebuild.
            std::vector<DynamicEntityRegion> regions;
            std::unordered_map<uint64_t, size_t> regionSlots;
            std::vector<size_t> activeSlots;
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
            for (const auto slot :
                 _dynamicEntitySpatialCache.activeSlots)
            {
                auto& region =
                    _dynamicEntitySpatialCache.regions[slot];
                region.hasBounds = false;
                region.low = {};
                region.high = {};
                region.center = {};
                region.radius = 0.0f;
                region.entities.clear();
            }
            _dynamicEntitySpatialCache.activeSlots.clear();
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
                    auto [slotIt, inserted] =
                        _dynamicEntitySpatialCache.regionSlots.emplace(
                            key,
                            _dynamicEntitySpatialCache
                                .regions.size());
                    if (inserted)
                    {
                        _dynamicEntitySpatialCache
                            .regions.emplace_back();
                    }
                    auto& region =
                        _dynamicEntitySpatialCache
                            .regions[slotIt->second];
                    if (region.entities.empty())
                    {
                        _dynamicEntitySpatialCache
                            .activeSlots.push_back(
                                slotIt->second);
                    }
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

            for (const auto slot :
                 _dynamicEntitySpatialCache.activeSlots)
            {
                auto& region =
                    _dynamicEntitySpatialCache.regions[slot];
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
            uint64_t sourceRevision{};
            uint64_t publishedSourceRevision{};
            size_t vertexCount{};
            std::shared_ptr<const std::vector<FirstPersonSurface>> surfaceStorage;
            uint64_t lastSeen{};
            bool dirty = true;
            FirstPersonVec3 center{};
            float radius{};
            std::vector<FirstPersonSurface> surfaces;
            std::vector<ImageIndex> textureDependencies;
            std::vector<uint64_t> reconstructionGroups;
            uint64_t reconstructionSelectionStamp{};
            uint64_t reconstructionSelectionSerial{};
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
            uint64_t materialVersion = 1;
            uint64_t bakedMaterialVersion = 0;
            uint64_t materialRequest = 0;
            std::unordered_map<size_t, std::vector<FirstPersonSurface>> projectedSurfaces;
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
        // Spatial candidate index for invalidation. Unlike the surface-region
        // map above, this covers every 32x32 region touched by the trajectory's
        // exact cached tile bounds.
        static std::unordered_map<uint64_t, std::unordered_set<uint64_t>>
            _trackTrajectoryGroupsByBoundsRegion;

        uint64_t TerrainKey(int32_t tx, int32_t ty);

        struct LargeSceneryGeometryCacheEntry
        {
            uint64_t signature{};
            uint64_t lastSeen{};
            bool dirty = false;
            bool visualHullRefined = false;
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
        static std::unordered_map<uint64_t, std::unordered_set<uint64_t>>
            _largeSceneryGroupsByBoundsRegion;
        static std::unordered_set<uint64_t> _activeLargeSceneryRegions;
        static bool _largeSceneryGeometryEnabled = false;

        struct ParkEntranceGeometryCacheEntry
        {
            uint64_t signature{};
            uint64_t lastSeen{};
            bool dirty = false;
            bool hasBounds = false;
            int32_t minTileX{}, minTileY{}, maxTileX{}, maxTileY{};
            std::array<
                std::pair<CoordsXY, uint64_t>, 3>
                tileSignatures{};
            std::vector<FirstPersonSurface> surfaces;
        };
        static std::unordered_map<
            uint64_t, ParkEntranceGeometryCacheEntry>
            _parkEntranceGeometryCache;
        static std::unordered_map<
            uint64_t, std::unordered_set<uint64_t>>
            _parkEntranceGroupsByRegion;
        static std::unordered_map<
            uint64_t, std::unordered_set<uint64_t>>
            _parkEntranceGroupsByBoundsRegion;
        static bool _parkEntranceGeometryEnabled = false;

        template<typename Visit>
        void VisitFirstPersonGpuRegionsInTileBounds(
            int32_t minTileX, int32_t minTileY,
            int32_t maxTileX, int32_t maxTileY,
            Visit&& visit)
        {
            if (maxTileX < minTileX || maxTileY < minTileY)
                return;
            const int32_t minRegionX = minTileX / 32;
            const int32_t minRegionY = minTileY / 32;
            const int32_t maxRegionX = maxTileX / 32;
            const int32_t maxRegionY = maxTileY / 32;
            for (int32_t regionY = minRegionY;
                 regionY <= maxRegionY; ++regionY)
            for (int32_t regionX = minRegionX;
                 regionX <= maxRegionX; ++regionX)
            {
                visit(FirstPersonGpuRegionKey(
                    regionX * 32, regionY * 32));
            }
        }

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
            if (cached.hasBounds)
            {
                VisitFirstPersonGpuRegionsInTileBounds(
                    cached.minTileX, cached.minTileY,
                    cached.maxTileX, cached.maxTileY,
                    [&](uint64_t regionKey) {
                        const auto found =
                            _largeSceneryGroupsByBoundsRegion.find(
                                regionKey);
                        if (found
                            == _largeSceneryGroupsByBoundsRegion.end())
                            return;
                        found->second.erase(groupKey);
                        if (found->second.empty())
                        {
                            _largeSceneryGroupsByBoundsRegion.erase(
                                found);
                        }
                    });
            }
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
            if (cached.hasBounds)
            {
                VisitFirstPersonGpuRegionsInTileBounds(
                    cached.minTileX, cached.minTileY,
                    cached.maxTileX, cached.maxTileY,
                    [&](uint64_t regionKey) {
                        _largeSceneryGroupsByBoundsRegion[
                            regionKey].insert(groupKey);
                    });
            }
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

        [[nodiscard]] bool
            ParkEntranceGeometryAllowedForView(
                uint32_t viewFlags)
        {
            constexpr uint32_t kIncompatible =
                VIEWPORT_FLAG_CLIP_VIEW
                | VIEWPORT_FLAG_CLIP_VIEW_SEE_THROUGH
                | VIEWPORT_FLAG_HIGHLIGHT_PATH_ISSUES;
            if ((viewFlags & kIncompatible) != 0
                || gTrackDesignSaveMode)
                return false;

            const auto* selected =
                TileInspector::GetSelectedElement();
            return selected == nullptr
                || selected->getType()
                    != TileElementType::entrance;
        }

        void MarkParkEntranceGeometryRegionsDirty(
            const ParkEntranceGeometryCacheEntry& cached)
        {
            if (cached.hasBounds)
            {
                VisitFirstPersonGpuRegionsInTileBounds(
                    cached.minTileX, cached.minTileY,
                    cached.maxTileX, cached.maxTileY,
                    [&](uint64_t regionKey) {
                        _staticRegionPackets[
                            regionKey].dirty = true;
                    });
            }
            for (const auto& surface : cached.surfaces)
            {
                if (surface.gpuRegion != 0)
                    _staticRegionPackets[
                        surface.gpuRegion].dirty = true;
            }
        }

        void UnregisterParkEntranceRegionMembership(
            uint64_t groupKey,
            const ParkEntranceGeometryCacheEntry& cached)
        {
            if (cached.hasBounds)
            {
                VisitFirstPersonGpuRegionsInTileBounds(
                    cached.minTileX, cached.minTileY,
                    cached.maxTileX, cached.maxTileY,
                    [&](uint64_t regionKey) {
                        const auto found =
                            _parkEntranceGroupsByBoundsRegion.find(
                                regionKey);
                        if (found
                            == _parkEntranceGroupsByBoundsRegion.end())
                            return;
                        found->second.erase(groupKey);
                        if (found->second.empty())
                        {
                            _parkEntranceGroupsByBoundsRegion.erase(
                                found);
                        }
                    });
            }

            std::unordered_set<uint64_t> regions;
            for (const auto& surface : cached.surfaces)
            {
                if (surface.gpuRegion != 0)
                    regions.insert(surface.gpuRegion);
            }
            for (const auto region : regions)
            {
                const auto found =
                    _parkEntranceGroupsByRegion.find(region);
                if (found == _parkEntranceGroupsByRegion.end())
                    continue;
                found->second.erase(groupKey);
                if (found->second.empty())
                    _parkEntranceGroupsByRegion.erase(found);
            }
        }

        void RegisterParkEntranceRegionMembership(
            uint64_t groupKey,
            const ParkEntranceGeometryCacheEntry& cached)
        {
            if (cached.hasBounds)
            {
                VisitFirstPersonGpuRegionsInTileBounds(
                    cached.minTileX, cached.minTileY,
                    cached.maxTileX, cached.maxTileY,
                    [&](uint64_t regionKey) {
                        _parkEntranceGroupsByBoundsRegion[
                            regionKey].insert(groupKey);
                    });
            }

            for (const auto& surface : cached.surfaces)
            {
                if (surface.gpuRegion != 0)
                {
                    _parkEntranceGroupsByRegion[
                        surface.gpuRegion].insert(groupKey);
                }
            }
        }

        [[nodiscard]] CoordsXY
            ParkEntranceSequenceTile(
                const ReconstructionGroupInfo& group,
                uint8_t direction,
                ParkEntranceSequence sequence)
        {
            CoordsXY result{
                int32_t(std::lround(group.anchor.x)),
                int32_t(std::lround(group.anchor.y)),
            };
            const CoordsXY side =
                CoordsDirectionDelta[
                    (direction - 1u) & 3u];
            if (sequence == ParkEntranceSequence::left)
            {
                result.x += side.x;
                result.y += side.y;
            }
            else if (sequence
                == ParkEntranceSequence::right)
            {
                result.x -= side.x;
                result.y -= side.y;
            }
            return result;
        }

        [[nodiscard]] EntranceElement*
            FindParkEntranceSequence(
                const ReconstructionGroupInfo& group,
                uint8_t direction,
                ObjectEntryIndex entryIndex,
                ParkEntranceSequence sequence)
        {
            const CoordsXY tile =
                ParkEntranceSequenceTile(
                    group, direction, sequence);
            auto* element =
                MapGetFirstElementAt(tile);
            if (element == nullptr)
                return nullptr;

            do
            {
                if (element->getType()
                        != TileElementType::entrance
                    || element->isGhost()
                    || element->isInvisible())
                    continue;
                auto* entrance =
                    element->asEntrance();
                if (entrance == nullptr
                    || entrance->getEntranceType()
                        != EntranceType::parkEntrance
                    || entrance->getSequenceIndex()
                        != sequence
                    || entrance->getEntryIndex()
                        != entryIndex
                    || (static_cast<uint8_t>(
                            entrance->getDirection()) & 3u)
                        != (direction & 3u)
                    || entrance->getBaseZ()
                        != int32_t(std::lround(
                            group.anchor.z)))
                    continue;
                return entrance;
            } while (!(element++)->isLastForTile());
            return nullptr;
        }

        [[nodiscard]] bool ParkEntranceInstanceComplete(
            const ReconstructionGroupInfo& group,
            uint8_t direction,
            ObjectEntryIndex entryIndex)
        {
            for (const auto sequence : {
                     ParkEntranceSequence::centre,
                     ParkEntranceSequence::left,
                     ParkEntranceSequence::right })
            {
                if (FindParkEntranceSequence(
                        group, direction, entryIndex,
                        sequence)
                    == nullptr)
                    return false;
            }
            return true;
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
            uint64_t signature, uint64_t frame,
            const FirstPersonSpriteSnapshot& sprites, uint64_t assetGeneration)
        {
            LargeSceneryGeometryCacheEntry result{};
            result.signature = signature;
            result.lastSeen = frame;
            result.bodyImageFirst = model.bodyImageFirst;
            result.bodyImageLast = model.bodyImageLast;
            const uint8_t objectDirection =
                static_cast<uint8_t>(large.getDirection()) & 3;
            const auto imageTemplate =
                LargeSceneryImageTemplate(large, entry);
            result.collisionProxies =
                BuildFirstPersonLargeSceneryCollisionProxies(
                    entry, model, group, objectDirection);
            if (!result.collisionProxies.empty())
            {
                FirstPersonVec3 proxyLow =
                    result.collisionProxies.front().low;
                FirstPersonVec3 proxyHigh =
                    result.collisionProxies.front().high;
                for (const auto& proxy :
                     result.collisionProxies)
                {
                    proxyLow.x =
                        std::min(proxyLow.x, proxy.low.x);
                    proxyLow.y =
                        std::min(proxyLow.y, proxy.low.y);
                    proxyLow.z =
                        std::min(proxyLow.z, proxy.low.z);
                    proxyHigh.x =
                        std::max(proxyHigh.x, proxy.high.x);
                    proxyHigh.y =
                        std::max(proxyHigh.y, proxy.high.y);
                    proxyHigh.z =
                        std::max(proxyHigh.z, proxy.high.z);
                }
                result.hasBounds = true;
                result.minTileX = int32_t(std::floor(
                    proxyLow.x / float(kCoordsXYStep)));
                result.minTileY = int32_t(std::floor(
                    proxyLow.y / float(kCoordsXYStep)));
                result.maxTileX = int32_t(std::floor(
                    proxyHigh.x / float(kCoordsXYStep)));
                result.maxTileY = int32_t(std::floor(
                    proxyHigh.y / float(kCoordsXYStep)));
            }

            struct BakedLargeSceneryKey
            {
                uint32_t image{};
                uint8_t objectDirection{};
                bool operator==(const BakedLargeSceneryKey&) const = default;
            };
            struct BakedLargeSceneryKeyHash
            {
                size_t operator()(const BakedLargeSceneryKey& key) const
                {
                    return size_t(key.image)
                        ^ (size_t(key.objectDirection) << 1);
                }
            };
            struct BakedLargeSceneryEntry
            {
                uint64_t generation{};
                bool resolved = false;
                bool usable = false;
                bool visualHullRefined = false;
                FirstPersonVec3 localLow{};
                FirstPersonVec3 localHigh{};
                std::vector<FirstPersonSurface> localSurfaces;
            };
            static thread_local std::unordered_map<
                BakedLargeSceneryKey,
                BakedLargeSceneryEntry,
                BakedLargeSceneryKeyHash>
                bakedCache;

            static thread_local uint64_t cacheEpoch = 0;
            if (cacheEpoch != assetGeneration)
            {
                bakedCache.clear();
                cacheEpoch = assetGeneration;
            }
            const BakedLargeSceneryKey bakedKey{
                entry.image, objectDirection
            };
            const uint64_t bakedGeneration =
                assetGeneration;
            if (const auto bakedIt = bakedCache.find(bakedKey);
                bakedIt != bakedCache.end()
                && bakedIt->second.generation
                    == bakedGeneration
                && bakedIt->second.resolved)
            {
                const auto& baked = bakedIt->second;
                if (!baked.usable)
                    return result;

                result.visualHullRefined =
                    baked.visualHullRefined;
                result.surfaces.reserve(
                    baked.localSurfaces.size());
                for (const auto& localSurface :
                     baked.localSurfaces)
                {
                    auto surface = localSurface;
                    surface.image =
                        imageTemplate.WithIndex(
                            localSurface.image.GetIndex());
                    surface.reconstructionGroup = group.key;
                    surface.diagnosticReconstructionKind = 1;
                    surface.diagnosticReconstructionAnchor =
                        group.anchor;
                    for (auto& vertex : surface.triangles)
                    {
                        vertex.world.x += group.anchor.x;
                        vertex.world.y += group.anchor.y;
                        vertex.world.z += group.anchor.z;
                    }
                    FirstPersonVec3 faceLow =
                        surface.triangles[0].world;
                    FirstPersonVec3 faceHigh = faceLow;
                    for (const auto& vertex :
                         surface.triangles)
                    {
                        faceLow.x = std::min(
                            faceLow.x, vertex.world.x);
                        faceLow.y = std::min(
                            faceLow.y, vertex.world.y);
                        faceLow.z = std::min(
                            faceLow.z, vertex.world.z);
                        faceHigh.x = std::max(
                            faceHigh.x, vertex.world.x);
                        faceHigh.y = std::max(
                            faceHigh.y, vertex.world.y);
                        faceHigh.z = std::max(
                            faceHigh.z, vertex.world.z);
                    }
                    const FirstPersonVec3 faceCenter{
                        0.5f * (faceLow.x + faceHigh.x),
                        0.5f * (faceLow.y + faceHigh.y),
                        0.5f * (faceLow.z + faceHigh.z),
                    };
                    surface.gpuRegion =
                        FirstPersonGpuRegionKey(
                            int32_t(std::floor(
                                faceCenter.x
                                / float(kCoordsXYStep))),
                            int32_t(std::floor(
                                faceCenter.y
                                / float(kCoordsXYStep))));
                    result.surfaces.emplace_back(
                        std::move(surface));
                }

                const FirstPersonVec3 low{
                    group.anchor.x + baked.localLow.x,
                    group.anchor.y + baked.localLow.y,
                    group.anchor.z + baked.localLow.z,
                };
                const FirstPersonVec3 high{
                    group.anchor.x + baked.localHigh.x,
                    group.anchor.y + baked.localHigh.y,
                    group.anchor.z + baked.localHigh.z,
                };
                result.hasBounds = true;
                result.minTileX = int32_t(
                    std::floor(low.x / float(kCoordsXYStep)));
                result.minTileY = int32_t(
                    std::floor(low.y / float(kCoordsXYStep)));
                result.maxTileX = int32_t(
                    std::floor(high.x / float(kCoordsXYStep)));
                result.maxTileY = int32_t(
                    std::floor(high.y / float(kCoordsXYStep)));
                const FirstPersonVec3 center{
                    0.5f * (low.x + high.x),
                    0.5f * (low.y + high.y),
                    0.5f * (low.z + high.z),
                };
                const float hx =
                    0.5f * (high.x - low.x);
                const float hy =
                    0.5f * (high.y - low.y);
                const float hz =
                    0.5f * (high.z - low.z);
                const float radius =
                    std::sqrt(
                        hx * hx + hy * hy + hz * hz)
                    + 2.0f;
                for (auto& surface : result.surfaces)
                {
                    surface.hasSemanticBounds = true;
                    surface.semanticCenter = center;
                    surface.semanticRadius = radius;
                }
                return result;
            }

            struct LargeSceneryDecodedView
            {
                bool attempted = false;
                ImageId image{};
                const G1Element* g1 = nullptr;
                uint8_t nativeDirection = 0;
                uint8_t sourceRotation = 0;
                ScreenCoordsXY spritePos{};
                float nativeDepth = 0.0f;
                std::vector<uint8_t> pixels;
            };
            std::vector<LargeSceneryDecodedView>
                decodedViews(entry.tiles.size() * 4u);
            const auto decodedView =
                [&](size_t sequence, uint8_t nativeDirection)
                    -> const LargeSceneryDecodedView* {
                    if (sequence >= entry.tiles.size()
                        || nativeDirection >= 4)
                        return nullptr;
                    auto& decoded =
                        decodedViews[
                            sequence * 4u + nativeDirection];
                    if (decoded.attempted)
                        return decoded.g1 != nullptr
                            ? &decoded : nullptr;
                    decoded.attempted = true;

                    const ImageIndex imageIndex =
                        entry.image + 4
                        + (ImageIndex(sequence) << 2)
                        + nativeDirection;
                    decoded.image =
                        imageTemplate.WithIndex(imageIndex);
                    decoded.g1 =
                        sprites.get(decoded.image);
                    if (decoded.g1 == nullptr
                        || decoded.g1->width <= 0
                        || decoded.g1->height <= 0)
                    {
                        decoded.g1 = nullptr;
                        return nullptr;
                    }
                    auto pixels =
                        DecodeFirstPersonSpritePixels(
                            *decoded.g1);
                    if (!pixels.has_value())
                    {
                        decoded.g1 = nullptr;
                        return nullptr;
                    }
                    decoded.pixels = std::move(*pixels);
                    decoded.nativeDirection =
                        nativeDirection;
                    decoded.sourceRotation =
                        FirstPersonViewportRotationForNativeView(
                            objectDirection,
                            nativeDirection);

                    const auto& tile =
                        entry.tiles[sequence];
                    const CoordsXY tileOffset =
                        CoordsXY{
                            tile.offset.x,
                            tile.offset.y
                        }.rotate(objectDirection);
                    const CoordsXY tileWorld{
                        int32_t(std::lround(
                            group.anchor.x))
                            + tileOffset.x,
                        int32_t(std::lround(
                            group.anchor.y))
                            + tileOffset.y,
                    };
                    const int32_t tileBaseZ =
                        int32_t(std::lround(
                            group.anchor.z))
                        + tile.offset.z;
                    const auto spriteOrigin =
                        GetTileElementPaintSpritePosition(
                            tileWorld,
                            decoded.sourceRotation);
                    decoded.spritePos =
                        Translate3DTo2DWithZ(
                            decoded.sourceRotation,
                            { spriteOrigin, tileBaseZ });
                    decoded.nativeDepth =
                        FirstPersonIsoDepth(
                            decoded.sourceRotation,
                            {
                                tileWorld.x + kCoordsXYHalfTile,
                                tileWorld.y + kCoordsXYHalfTile,
                                tileBaseZ,
                            });
                    return &decoded;
                };

            struct LargeSceneryWorldCell
            {
                FirstPersonVec3 low{};
                FirstPersonVec3 high{};
                size_t sequence{};
            };
            std::vector<LargeSceneryWorldCell>
                occupancyCells;
            occupancyCells.reserve(entry.tiles.size() * 4u);
            static constexpr std::array<CoordsXY, 4>
                kQuarterCellOffsets{ {
                    { 1, 1 }, // SW
                    { 1, 0 }, // NW
                    { 0, 0 }, // NE
                    { 0, 1 }, // SE
                } };

            bool haveEnvelope = false;
            FirstPersonVec3 envelopeLow{}, envelopeHigh{};
            for (size_t sequence = 0;
                 sequence < entry.tiles.size(); ++sequence)
            {
                const auto& tile = entry.tiles[sequence];
                for (uint8_t quarter = 0;
                     quarter < 4; ++quarter)
                {
                    if ((tile.corners
                            & (1u << quarter))
                        == 0)
                        continue;
                    const int32_t assetX0 =
                        tile.offset.x
                        + kQuarterCellOffsets[quarter].x * 16;
                    const int32_t assetY0 =
                        tile.offset.y
                        + kQuarterCellOffsets[quarter].y * 16;
                    const int32_t assetX1 = assetX0 + 16;
                    const int32_t assetY1 = assetY0 + 16;

                    std::array<CoordsXY, 4> placed{ {
                        FirstPersonLargeSceneryPlacedPoint(
                            { tile.offset.x, tile.offset.y },
                            { assetX0, assetY0 },
                            objectDirection),
                        FirstPersonLargeSceneryPlacedPoint(
                            { tile.offset.x, tile.offset.y },
                            { assetX1, assetY0 },
                            objectDirection),
                        FirstPersonLargeSceneryPlacedPoint(
                            { tile.offset.x, tile.offset.y },
                            { assetX1, assetY1 },
                            objectDirection),
                        FirstPersonLargeSceneryPlacedPoint(
                            { tile.offset.x, tile.offset.y },
                            { assetX0, assetY1 },
                            objectDirection),
                    } };
                    int32_t minX = placed[0].x;
                    int32_t maxX = placed[0].x;
                    int32_t minY = placed[0].y;
                    int32_t maxY = placed[0].y;
                    for (const auto p : placed)
                    {
                        minX = std::min(minX, p.x);
                        maxX = std::max(maxX, p.x);
                        minY = std::min(minY, p.y);
                        maxY = std::max(maxY, p.y);
                    }

                    LargeSceneryWorldCell cell{
                        {
                            group.anchor.x + float(minX),
                            group.anchor.y + float(minY),
                            group.anchor.z
                                + float(tile.offset.z),
                        },
                        {
                            group.anchor.x + float(maxX),
                            group.anchor.y + float(maxY),
                            group.anchor.z
                                + float(
                                    tile.offset.z
                                    + tile.zClearance),
                        },
                        sequence,
                    };
                    if (cell.high.x <= cell.low.x
                        || cell.high.y <= cell.low.y
                        || cell.high.z <= cell.low.z)
                        continue;
                    occupancyCells.push_back(cell);
                    if (!haveEnvelope)
                    {
                        envelopeLow = cell.low;
                        envelopeHigh = cell.high;
                        haveEnvelope = true;
                    }
                    else
                    {
                        envelopeLow.x =
                            std::min(
                                envelopeLow.x, cell.low.x);
                        envelopeLow.y =
                            std::min(
                                envelopeLow.y, cell.low.y);
                        envelopeLow.z =
                            std::min(
                                envelopeLow.z, cell.low.z);
                        envelopeHigh.x =
                            std::max(
                                envelopeHigh.x, cell.high.x);
                        envelopeHigh.y =
                            std::max(
                                envelopeHigh.y, cell.high.y);
                        envelopeHigh.z =
                            std::max(
                                envelopeHigh.z, cell.high.z);
                    }
                }
            }

            const auto cacheFailure = [&]() {
                BakedLargeSceneryEntry baked{};
                baked.generation = bakedGeneration;
                baked.resolved = true;
                baked.usable = false;
                bakedCache[bakedKey] = std::move(baked);
            };
            if (!haveEnvelope || occupancyCells.empty())
            {
                cacheFailure();
                return result;
            }

            std::array<uint8_t, 4>
                nativeDirectionForRotation{};
            for (uint8_t nativeDirection = 0;
                 nativeDirection < 4; ++nativeDirection)
            {
                const uint8_t rotation =
                    FirstPersonViewportRotationForNativeView(
                        objectDirection,
                        nativeDirection);
                nativeDirectionForRotation[rotation] =
                    nativeDirection;
            }

            std::vector<FirstPersonVisualHullView>
                hullViews;
            hullViews.reserve(4);
            for (uint8_t rotation = 0;
                 rotation < 4; ++rotation)
            {
                const uint8_t nativeDirection =
                    nativeDirectionForRotation[rotation];
                FirstPersonVisualHullView view{};
                view.imageDirection = rotation;
                bool haveImage = false;
                for (size_t sequence = 0;
                     sequence < entry.tiles.size();
                     ++sequence)
                {
                    const auto* source =
                        decodedView(
                            sequence, nativeDirection);
                    if (source == nullptr)
                        continue;
                    if (!haveImage)
                    {
                        view.image =
                            source->image.GetIndex();
                        haveImage = true;
                    }
                    for (int32_t y = 0;
                         y < source->g1->height; ++y)
                    for (int32_t x = 0;
                         x < source->g1->width; ++x)
                    {
                        const uint8_t pixel =
                            source->pixels[
                                size_t(y)
                                    * size_t(
                                        source->g1->width)
                                + size_t(x)];
                        if (pixel == 0)
                            continue;
                        view.observed.add(
                            source->spritePos.x
                                + source->g1->xOffset
                                + x,
                            source->spritePos.y
                                + source->g1->yOffset
                                + y);
                    }
                }
                if (!haveImage || view.observed.empty())
                {
                    cacheFailure();
                    return result;
                }
                hullViews.emplace_back(std::move(view));
            }

            float hullStep = 4.0f;
            while (true)
            {
                const int32_t nx = int32_t(std::ceil(
                    (envelopeHigh.x - envelopeLow.x)
                    / hullStep));
                const int32_t ny = int32_t(std::ceil(
                    (envelopeHigh.y - envelopeLow.y)
                    / hullStep));
                const int32_t nz = int32_t(std::ceil(
                    (envelopeHigh.z - envelopeLow.z)
                    / hullStep));
                const size_t count =
                    nx > 0 && ny > 0 && nz > 0
                    ? size_t(nx) * size_t(ny)
                        * size_t(nz)
                    : 0;
                if (nx > 0 && ny > 0 && nz > 0
                    && nx <= 96 && ny <= 96
                    && nz <= 96
                    && count <= 131072)
                    break;
                hullStep *= 2.0f;
                if (hullStep > 32.0f)
                {
                    cacheFailure();
                    return result;
                }
            }

            FirstPersonVisualHullConfig hullConfig{};
            hullConfig.minimumViews = 4;
            hullConfig.minimumOccupiedCells = 1;
            hullConfig.maximumOccupiedCells = 131072;
            hullConfig.maximumGridCells = 131072;
            hullConfig.maximumAxisCells = 96;
            // Multi-view support itself performs the carving. Do not reject a
            // valid carved hull merely because decorative sprite pixels extend
            // beyond collision/occupancy metadata.
            hullConfig.minimumCandidateCoverage = 0.0f;
            hullConfig.minimumObservedCoverage = 0.0f;
            hullConfig.maximumEdgeError =
                std::numeric_limits<int32_t>::max();

            const auto pointInCell =
                [](FirstPersonVec3 point,
                   const LargeSceneryWorldCell& cell,
                   float epsilon = 0.001f) {
                    return point.x
                            >= cell.low.x - epsilon
                        && point.x
                            < cell.high.x + epsilon
                        && point.y
                            >= cell.low.y - epsilon
                        && point.y
                            < cell.high.y + epsilon
                        && point.z
                            >= cell.low.z - epsilon
                        && point.z
                            < cell.high.z + epsilon;
                };
            const auto occupancyPredicate =
                [&](FirstPersonVec3 point) {
                    return std::any_of(
                        occupancyCells.begin(),
                        occupancyCells.end(),
                        [&](const auto& cell) {
                            return pointInCell(
                                point, cell, 0.001f);
                        });
                };
            const auto projectPoint =
                [](uint8_t rotation,
                   FirstPersonVec3 point) {
                    const auto projected =
                        Translate3DTo2DWithZ(
                            rotation,
                            {
                                int32_t(std::lround(
                                    point.x)),
                                int32_t(std::lround(
                                    point.y)),
                                int32_t(std::lround(
                                    point.z)),
                            });
                    return std::array<float, 2>{
                        float(projected.x),
                        float(projected.y)
                    };
                };
            const auto pointSupported =
                [&](const FirstPersonVisualHullView& view,
                    FirstPersonVec3 point) {
                    const auto projected =
                        Translate3DTo2DWithZ(
                            view.imageDirection,
                            {
                                int32_t(std::lround(point.x)),
                                int32_t(std::lround(point.y)),
                                int32_t(std::lround(point.z)),
                            });
                    // Carving asks whether the WHOLE multi-tile object is
                    // visually present in this native view. A particular
                    // sequence sprite being transparent does not prove empty
                    // space: another tile sprite may own that visible pixel.
                    for (int32_t dy = -1; dy <= 1; ++dy)
                    for (int32_t dx = -1; dx <= 1; ++dx)
                    {
                        if (view.observed.contains(
                                projected.x + dx,
                                projected.y + dy))
                            return true;
                    }
                    return false;
                };

            const FirstPersonVisualHull hull =
                BuildFirstPersonVisualHull(
                    hullViews,
                    {
                        envelopeLow.x,
                        envelopeHigh.x,
                        envelopeLow.y,
                        envelopeHigh.y,
                        envelopeLow.z,
                        envelopeHigh.z,
                        hullStep,
                    },
                    hullConfig,
                    projectPoint,
                    occupancyPredicate,
                    pointSupported);

            std::vector<FirstPersonVisualHullFace> faces;
            if (hull.valid)
            {
                faces =
                    BuildFirstPersonVisualHullBoundaryFaces(
                        hull);
                result.visualHullRefined =
                    !faces.empty();
            }

            if (faces.empty())
            {
                // Carving is refinement, not permission to throw away known
                // structure. Preserve the old authoritative occupancy shell
                // as a non-billboard fallback when an asset does not provide
                // enough silhouette evidence for a stable carved hull.
                faces.reserve(model.faces.size());
                for (const auto& sourceFace :
                     model.faces)
                {
                    if (sourceFace.sequence
                        >= entry.tiles.size())
                        continue;
                    const auto& tile =
                        entry.tiles[
                            sourceFace.sequence];
                    FirstPersonVisualHullFace face{};
                    for (size_t i = 0;
                         i < face.corners.size(); ++i)
                    {
                        const auto& corner =
                            sourceFace.corners[i];
                        const auto localXY =
                            FirstPersonLargeSceneryPlacedPoint(
                                {
                                    tile.offset.x,
                                    tile.offset.y,
                                },
                                {
                                    corner.x,
                                    corner.y,
                                },
                                objectDirection);
                        face.corners[i] = {
                            group.anchor.x
                                + float(localXY.x),
                            group.anchor.y
                                + float(localXY.y),
                            group.anchor.z
                                + float(corner.z),
                        };
                    }
                    if (sourceFace.kind
                        == LargeSceneryAssetFaceKind::top)
                    {
                        face.normal = { 0.0f, 0.0f, 1.0f };
                    }
                    else if (sourceFace.kind
                        == LargeSceneryAssetFaceKind::bottom)
                    {
                        face.normal = { 0.0f, 0.0f, -1.0f };
                    }
                    else
                    {
                        CoordsXY localNormal{};
                        switch (sourceFace.kind)
                        {
                            case LargeSceneryAssetFaceKind::minX:
                                localNormal = { -1, 0 };
                                break;
                            case LargeSceneryAssetFaceKind::maxX:
                                localNormal = { 1, 0 };
                                break;
                            case LargeSceneryAssetFaceKind::minY:
                                localNormal = { 0, -1 };
                                break;
                            case LargeSceneryAssetFaceKind::maxY:
                                localNormal = { 0, 1 };
                                break;
                            default:
                                break;
                        }
                        const auto worldNormal =
                            localNormal.rotate(
                                objectDirection);
                        face.normal = {
                            float(worldNormal.x),
                            float(worldNormal.y),
                            0.0f,
                        };
                    }
                    faces.push_back(face);
                }
            }
            if (faces.empty())
            {
                cacheFailure();
                return result;
            }

            const auto faceVisible =
                [](const FirstPersonVisualHullFace& face,
                   uint8_t rotation) {
                    if (face.normal.z > 0.5f)
                        return true;
                    if (face.normal.z < -0.5f)
                        return false;
                    const CoordsXY normal{
                        int32_t(std::lround(
                            face.normal.x)),
                        int32_t(std::lround(
                            face.normal.y)),
                    };
                    return FirstPersonFaceVisibleFromNativeView(
                        normal, rotation);
                };

            std::array<FirstPersonDepthOwnerMap, 4>
                depthOwners{};
            for (uint8_t rotation = 0;
                 rotation < 4; ++rotation)
            {
                for (size_t faceIndex = 0;
                     faceIndex < faces.size();
                     ++faceIndex)
                {
                    const auto& face =
                        faces[faceIndex];
                    if (!faceVisible(face, rotation))
                        continue;
                    std::array<ScreenCoordsXY, 4> screen{};
                    std::array<float, 4> depth{};
                    for (size_t i = 0; i < 4; ++i)
                    {
                        const auto& p =
                            face.corners[i];
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
                }
            }

            const auto edgeLength =
                [](FirstPersonVec3 a,
                   FirstPersonVec3 b) {
                    const float dx = b.x - a.x;
                    const float dy = b.y - a.y;
                    const float dz = b.z - a.z;
                    return std::sqrt(
                        dx * dx + dy * dy + dz * dz);
                };
            const auto facePoint =
                [](const auto& face,
                   float s, float t) {
                    const auto lerp =
                        [](FirstPersonVec3 a,
                           FirstPersonVec3 b,
                           float alpha) {
                            return FirstPersonVec3{
                                a.x
                                    + (b.x - a.x)
                                        * alpha,
                                a.y
                                    + (b.y - a.y)
                                        * alpha,
                                a.z
                                    + (b.z - a.z)
                                        * alpha,
                            };
                        };
                    return lerp(
                        lerp(face[0], face[1], s),
                        lerp(face[3], face[2], s),
                        t);
                };
            const auto ownedNear =
                [&](uint8_t rotation,
                    uint32_t owner,
                    int32_t x, int32_t y) {
                    for (int32_t dy = -1;
                         dy <= 1; ++dy)
                    for (int32_t dx = -1;
                         dx <= 1; ++dx)
                    {
                        const auto found =
                            depthOwners[rotation].find(
                                FirstPersonSilhouettePixelKey(
                                    x + dx,
                                    y + dy));
                        if (found
                                != depthOwners[
                                       rotation]
                                       .end()
                            && found->second.owner
                                == owner)
                            return true;
                    }
                    return false;
                };
            enum class LargeSceneryCompositeSampleKind : uint8_t
            {
                unknown,
                transparent,
                colour,
            };
            struct LargeSceneryCompositeSample
            {
                LargeSceneryCompositeSampleKind kind =
                    LargeSceneryCompositeSampleKind::unknown;
                uint8_t pixel = 0;
            };
            const auto sampleComposite =
                [&](uint8_t nativeDirection,
                    ScreenCoordsXY projected)
                    -> LargeSceneryCompositeSample {
                    bool coveredByAnySprite = false;
                    const LargeSceneryDecodedView*
                        bestSource = nullptr;
                    uint8_t bestPixel = 0;
                    for (size_t sequence = 0;
                         sequence < entry.tiles.size();
                         ++sequence)
                    {
                        const auto* source =
                            decodedView(
                                sequence, nativeDirection);
                        if (source == nullptr)
                            continue;
                        const int32_t u =
                            projected.x
                            - source->spritePos.x
                            - source->g1->xOffset;
                        const int32_t v =
                            projected.y
                            - source->spritePos.y
                            - source->g1->yOffset;
                        if (u < 0 || v < 0
                            || u >= source->g1->width
                            || v >= source->g1->height)
                            continue;
                        coveredByAnySprite = true;
                        const uint8_t pixel =
                            source->pixels[
                                size_t(v)
                                    * size_t(source->g1->width)
                                + size_t(u)];
                        if (pixel == 0)
                            continue;
                        if (bestSource == nullptr
                            || source->nativeDepth
                                > bestSource->nativeDepth)
                        {
                            bestSource = source;
                            bestPixel = pixel;
                        }
                    }
                    if (bestSource != nullptr)
                    {
                        return {
                            LargeSceneryCompositeSampleKind::colour,
                            bestPixel,
                        };
                    }
                    if (coveredByAnySprite)
                    {
                        return {
                            LargeSceneryCompositeSampleKind::transparent,
                            0,
                        };
                    }
                    return {
                        LargeSceneryCompositeSampleKind::unknown,
                        0,
                    };
                };

            bool haveBounds = false;
            FirstPersonVec3 low{}, high{};
            for (size_t faceIndex = 0;
                 faceIndex < faces.size();
                 ++faceIndex)
            {
                const auto& face = faces[faceIndex];
                if (face.normal.z < -0.5f)
                    continue;

                FirstPersonVec3 faceCenter{};
                for (const auto& point :
                     face.corners)
                {
                    faceCenter =
                        Add(faceCenter, point);
                    if (!haveBounds)
                    {
                        low = high = point;
                        haveBounds = true;
                    }
                    else
                    {
                        low.x =
                            std::min(low.x, point.x);
                        low.y =
                            std::min(low.y, point.y);
                        low.z =
                            std::min(low.z, point.z);
                        high.x =
                            std::max(high.x, point.x);
                        high.y =
                            std::max(high.y, point.y);
                        high.z =
                            std::max(high.z, point.z);
                    }
                }
                faceCenter = Mul(
                    faceCenter, 0.25f);

                struct LargeSceneryFaceView
                {
                    uint8_t rotation = 0;
                    uint8_t nativeDirection = 0;
                    float score = 0.0f;
                };
                std::vector<LargeSceneryFaceView>
                    candidates;
                for (uint8_t rotation = 0;
                     rotation < 4; ++rotation)
                {
                    if (!faceVisible(
                            face, rotation))
                        continue;
                    std::array<ScreenCoordsXY, 4>
                        projected{};
                    FirstPersonSilhouette silhouette{};
                    for (size_t i = 0;
                         i < face.corners.size();
                         ++i)
                    {
                        const auto& p =
                            face.corners[i];
                        projected[i] =
                            Translate3DTo2DWithZ(
                                rotation,
                                {
                                    int32_t(
                                        std::lround(
                                            p.x)),
                                    int32_t(
                                        std::lround(
                                            p.y)),
                                    int32_t(
                                        std::lround(
                                            p.z)),
                                });
                    }
                    AddFirstPersonSilhouetteQuad(
                        silhouette,
                        projected);
                    const float ownership =
                        FirstPersonDepthOwnerCoverage(
                            depthOwners[rotation],
                            uint32_t(faceIndex),
                            silhouette);
                    if (ownership < 0.20f)
                        continue;
                    candidates.push_back({
                        rotation,
                        nativeDirectionForRotation[
                            rotation],
                        ownership
                            * float(std::max<size_t>(
                                1,
                                silhouette.size())),
                    });
                }
                std::sort(
                    candidates.begin(),
                    candidates.end(),
                    [](const LargeSceneryFaceView& a,
                       const LargeSceneryFaceView& b) {
                        if (a.score != b.score)
                            return a.score > b.score;
                        return a.rotation
                            < b.rotation;
                    });

                ImageId fallbackImage{};
                for (const auto& candidate : candidates)
                {
                    for (size_t sequence = 0;
                         sequence < entry.tiles.size();
                         ++sequence)
                    {
                        const auto* source =
                            decodedView(
                                sequence,
                                candidate.nativeDirection);
                        if (source == nullptr)
                            continue;
                        fallbackImage = source->image;
                        break;
                    }
                    if (fallbackImage.HasValue())
                        break;
                }
                if (!fallbackImage.HasValue())
                    continue;

                FirstPersonSurface surface{};
                surface.image = fallbackImage;
                surface.cameraIndependent = true;
                // Large scenery is reconstructed as a closed physical hull
                // (carved visual hull when available, authoritative occupancy
                // shell otherwise). Its boundary face already carries the
                // world-space outward normal, so use the same physical
                // half-space visibility rule as reconstructed small scenery.
                surface.exteriorOnly = true;
                surface.outwardNormal = face.normal;
                surface.reconstructionGroup =
                    group.key;
                surface.diagnosticReconstructionKind = 1;
                surface.diagnosticReconstructionAnchor =
                    group.anchor;
                surface.gpuRegion =
                    FirstPersonGpuRegionKey(
                        int32_t(std::floor(
                            faceCenter.x
                            / float(kCoordsXYStep))),
                        int32_t(std::floor(
                            faceCenter.y
                            / float(kCoordsXYStep))));

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
                std::vector<uint8_t> pixels(
                    size_t(width)
                        * size_t(height), 0);
                bool haveKnownMaterial = false;
                bool haveUnknownMaterial = false;
                size_t visiblePixelCount = 0;

                for (int32_t y = 0;
                     y < height; ++y)
                for (int32_t x = 0;
                     x < width; ++x)
                {
                    const auto worldPoint =
                        facePoint(
                            face.corners,
                            (float(x) + 0.5f)
                                / float(width),
                            (float(y) + 0.5f)
                                / float(height));
                    bool texelKnown = false;
                    bool texelUnknown = candidates.empty();
                    uint8_t selectedPixel = 0;
                    for (const auto& candidate :
                         candidates)
                    {
                        const auto projected =
                            Translate3DTo2DWithZ(
                                candidate.rotation,
                                {
                                    int32_t(std::lround(
                                        worldPoint.x)),
                                    int32_t(std::lround(
                                        worldPoint.y)),
                                    int32_t(std::lround(
                                        worldPoint.z)),
                                });
                        if (!ownedNear(
                                candidate.rotation,
                                uint32_t(faceIndex),
                                projected.x,
                                projected.y))
                            continue;

                        const auto sample =
                            sampleComposite(
                                candidate.nativeDirection,
                                projected);
                        if (sample.kind
                            == LargeSceneryCompositeSampleKind::unknown)
                        {
                            texelUnknown = true;
                            continue;
                        }
                        texelKnown = true;
                        selectedPixel =
                            sample.kind
                                == LargeSceneryCompositeSampleKind::colour
                            ? sample.pixel : 0;
                        // Whole-object transparency is trustworthy evidence.
                        // Do not borrow a colour from another view merely to
                        // fill a carved physical face.
                        break;
                    }

                    if (texelKnown)
                    {
                        haveKnownMaterial = true;
                    }
                    else if (texelUnknown)
                    {
                        haveUnknownMaterial = true;
                        selectedPixel =
                            FirstPersonMissingMaterialPixel(
                                x, y);
                    }
                    pixels[
                        size_t(y) * size_t(width)
                        + size_t(x)] =
                        selectedPixel;
                    if (selectedPixel != 0)
                        ++visiblePixelCount;
                }

                if (visiblePixelCount == 0
                    && haveKnownMaterial
                    && !haveUnknownMaterial)
                    continue;
                if (visiblePixelCount == 0)
                    continue;

                uint64_t fingerprint =
                    14695981039346656037ull;
                ExtendStableKey(
                    fingerprint, width);
                ExtendStableKey(
                    fingerprint, height);
                for (const auto pixel : pixels)
                {
                    fingerprint ^= pixel;
                    fingerprint *= 1099511628211ull;
                }
                surface.persistentPixels =
                    std::make_shared<
                        std::vector<uint8_t>>(
                            std::move(pixels));
                surface.immutableWidth =
                    int16_t(width);
                surface.immutableHeight =
                    int16_t(height);
                surface.persistentBitmap = true;
                surface.immutableFingerprint =
                    fingerprint;

                const std::array<
                    FirstPersonVertex, 4> vertices{ {
                    { face.corners[0],
                      0.0f, 0.0f },
                    { face.corners[1],
                      float(width), 0.0f },
                    { face.corners[2],
                      float(width),
                      float(height) },
                    { face.corners[3],
                      0.0f,
                      float(height) },
                } };
                EmitQuad(surface, vertices);
                result.surfaces.emplace_back(
                    std::move(surface));
            }

            if (haveBounds
                && !result.surfaces.empty())
            {
                result.hasBounds = true;
                result.minTileX = int32_t(
                    std::floor(
                        low.x
                        / float(kCoordsXYStep)));
                result.minTileY = int32_t(
                    std::floor(
                        low.y
                        / float(kCoordsXYStep)));
                result.maxTileX = int32_t(
                    std::floor(
                        high.x
                        / float(kCoordsXYStep)));
                result.maxTileY = int32_t(
                    std::floor(
                        high.y
                        / float(kCoordsXYStep)));
                const FirstPersonVec3 center{
                    0.5f * (low.x + high.x),
                    0.5f * (low.y + high.y),
                    0.5f * (low.z + high.z),
                };
                const float hx =
                    0.5f * (high.x - low.x);
                const float hy =
                    0.5f * (high.y - low.y);
                const float hz =
                    0.5f * (high.z - low.z);
                const float radius =
                    std::sqrt(
                        hx * hx + hy * hy + hz * hz)
                    + 2.0f;
                for (auto& surface :
                     result.surfaces)
                {
                    surface.hasSemanticBounds = true;
                    surface.semanticCenter = center;
                    surface.semanticRadius = radius;
                }

                BakedLargeSceneryEntry baked{};
                baked.generation =
                    bakedGeneration;
                baked.resolved = true;
                baked.usable = true;
                baked.visualHullRefined =
                    result.visualHullRefined;
                baked.localLow = {
                    low.x - group.anchor.x,
                    low.y - group.anchor.y,
                    low.z - group.anchor.z,
                };
                baked.localHigh = {
                    high.x - group.anchor.x,
                    high.y - group.anchor.y,
                    high.z - group.anchor.z,
                };
                baked.localSurfaces.reserve(
                    result.surfaces.size());
                for (const auto& sourceSurface :
                     result.surfaces)
                {
                    auto local =
                        sourceSurface;
                    local.reconstructionGroup = 0;
                    local.diagnosticReconstructionKind = 0;
                    local.diagnosticReconstructionAnchor = {};
                    local.gpuRegion = 0;
                    local.hasSemanticBounds = false;
                    local.semanticCenter = {};
                    local.semanticRadius = 0.0f;
                    for (auto& vertex :
                         local.triangles)
                    {
                        vertex.world.x -=
                            group.anchor.x;
                        vertex.world.y -=
                            group.anchor.y;
                        vertex.world.z -=
                            group.anchor.z;
                    }
                    baked.localSurfaces.emplace_back(
                        std::move(local));
                }
                bakedCache[bakedKey] =
                    std::move(baked);
            }
            else
            {
                cacheFailure();
            }
            return result;
        }

        [[nodiscard]] std::vector<FirstPersonSurface>
            BuildFirstPersonSemanticComponentSurfaces(
                const FirstPersonPaintSemanticComponent& component,
                uint64_t gpuRegion,
                uint64_t diagnosticSemanticGroup,
                const std::array<
                    const FirstPersonPaintSemanticComponent*, 4>&
                    artworkViews,
                const std::vector<
                    const FirstPersonPaintSemanticComponent*>*
                    depthPeers,
                const FirstPersonSpriteSnapshot& sprites,
                bool* visualHullRefinedOut = nullptr,
                bool allowVisualHull = true);

        [[nodiscard]] FirstPersonPaintSemanticTransform
            MakeGroupedParkEntranceTransform(
                CoordsXY tile, uint8_t direction)
        {
            FirstPersonPaintSemanticTransform transform{};
            transform.origin = {
                float(tile.x + kCoordsXYHalfTile),
                float(tile.y + kCoordsXYHalfTile),
                0.0f,
            };
            switch (direction & 3u)
            {
                case 1:
                    transform.axisX = { 0.0f, 1.0f, 0.0f };
                    transform.axisY = { -1.0f, 0.0f, 0.0f };
                    break;
                case 2:
                    transform.axisX = { -1.0f, 0.0f, 0.0f };
                    transform.axisY = { 0.0f, -1.0f, 0.0f };
                    break;
                case 3:
                    transform.axisX = { 0.0f, -1.0f, 0.0f };
                    transform.axisY = { 1.0f, 0.0f, 0.0f };
                    break;
                default:
                    break;
            }
            return transform;
        }

        [[nodiscard]] ScreenCoordsXY
            GroupedParkEntranceArtworkScreenPos(
                CoordsXY tile, int32_t baseZ,
                uint8_t sourceRotation)
        {
            const auto spriteOrigin =
                GetTileElementPaintSpritePosition(
                    tile, sourceRotation);
            return Translate3DTo2DWithZ(
                sourceRotation,
                { spriteOrigin, baseZ });
        }

        ParkEntranceGeometryCacheEntry
            BuildParkEntranceInstanceGeometry(
                const std::array<ImageIndex, 12>& bodyImages,
                uint8_t direction,
                int32_t clearanceHeight,
                const ReconstructionGroupInfo& group,
                uint64_t signature, uint64_t frame,
                const FirstPersonSpriteSnapshot& sprites,
                const std::optional<
                    FirstPersonWalkabilityConstraint>&
                    passage)
        {
            (void)sprites;

            ParkEntranceGeometryCacheEntry result{};
            result.signature = signature;
            result.lastSeen = frame;

            result.hasBounds = true;
            result.minTileX =
                result.maxTileX =
                    int32_t(std::lround(group.anchor.x))
                    / kCoordsXYStep;
            result.minTileY =
                result.maxTileY =
                    int32_t(std::lround(group.anchor.y))
                    / kCoordsXYStep;
            for (const auto sequence : {
                     ParkEntranceSequence::left,
                     ParkEntranceSequence::right })
            {
                const auto tile =
                    ParkEntranceSequenceTile(
                        group, direction, sequence);
                const int32_t tx =
                    tile.x / kCoordsXYStep;
                const int32_t ty =
                    tile.y / kCoordsXYStep;
                result.minTileX =
                    std::min(result.minTileX, tx);
                result.minTileY =
                    std::min(result.minTileY, ty);
                result.maxTileX =
                    std::max(result.maxTileX, tx);
                result.maxTileY =
                    std::max(result.maxTileY, ty);
            }

            if (clearanceHeight <= 0)
                return result;

            // Appearance reconstruction remains deliberately conservative:
            // the park entrance is the collision-sized three-tile envelope
            // with explicit unknown material. Walkability contributes only a
            // negative geometric constraint. If the authoritative map says
            // the centre entrance is traversable in two opposite directions,
            // that guaranteed corridor is cut through this fallback shell.
            const int32_t baseZ =
                int32_t(std::lround(group.anchor.z));
            const FirstPersonVec3 low{
                float(result.minTileX * kCoordsXYStep),
                float(result.minTileY * kCoordsXYStep),
                float(baseZ),
            };
            const FirstPersonVec3 high{
                float((result.maxTileX + 1)
                    * kCoordsXYStep),
                float((result.maxTileY + 1)
                    * kCoordsXYStep),
                float(baseZ + clearanceHeight),
            };
            const auto centreTile =
                ParkEntranceSequenceTile(
                    group, direction,
                    ParkEntranceSequence::centre);
            const std::array<CoordsXY, 3>
                sequenceTiles{ {
                    centreTile,
                    ParkEntranceSequenceTile(
                        group, direction,
                        ParkEntranceSequence::left),
                    ParkEntranceSequenceTile(
                        group, direction,
                        ParkEntranceSequence::right),
                } };
            const auto sourceTileForPoint =
                [&](FirstPersonVec3 point) {
                    CoordsXY best = sequenceTiles[0];
                    float bestDistance =
                        std::numeric_limits<float>::max();
                    for (const auto& tile :
                         sequenceTiles)
                    {
                        const float dx =
                            point.x
                            - float(
                                tile.x
                                + kCoordsXYHalfTile);
                        const float dy =
                            point.y
                            - float(
                                tile.y
                                + kCoordsXYHalfTile);
                        const float distance =
                            dx * dx + dy * dy;
                        if (distance < bestDistance)
                        {
                            best = tile;
                            bestDistance = distance;
                        }
                    }
                    return best;
                };
            const auto passageAxis =
                passage.has_value()
                ? FirstPersonPassageAxisForConnections(
                    passage->connectedSides)
                : FirstPersonPassageAxis::none;
            const float passageTop =
                std::min(
                    high.z,
                    low.z + float(kPathClearance));

            struct EntranceFallbackFace
            {
                std::array<FirstPersonVec3, 4> corners{};
                FirstPersonVec3 normal{};
            };
            std::vector<EntranceFallbackFace> faces;
            faces.reserve(16);

            const auto addFace =
                [&](FirstPersonVec3 a,
                    FirstPersonVec3 b,
                    FirstPersonVec3 c0,
                    FirstPersonVec3 d,
                    FirstPersonVec3 normal) {
                    const auto ab = Sub(b, a);
                    const auto ad = Sub(d, a);
                    if (std::sqrt(FpDot(ab, ab)) < 0.01f
                        || std::sqrt(FpDot(ad, ad)) < 0.01f)
                        return;
                    faces.push_back({
                        { a, b, c0, d }, normal
                    });
                };

            if (passageAxis
                    == FirstPersonPassageAxis::x
                && passageTop > low.z + 0.01f)
            {
                const float corridorLow =
                    float(centreTile.y)
                    + kFirstPersonPathDeckInset;
                const float corridorHigh =
                    float(centreTile.y)
                    + kFirstPersonPathDeckMax;

                addFace(
                    { low.x, low.y, low.z },
                    { high.x, low.y, low.z },
                    { high.x, low.y, high.z },
                    { low.x, low.y, high.z },
                    { 0.0f, -1.0f, 0.0f });
                addFace(
                    { high.x, high.y, low.z },
                    { low.x, high.y, low.z },
                    { low.x, high.y, high.z },
                    { high.x, high.y, high.z },
                    { 0.0f, 1.0f, 0.0f });

                for (const auto endX :
                     { low.x, high.x })
                {
                    const float nx =
                        endX == low.x ? -1.0f : 1.0f;
                    addFace(
                        { endX, low.y, low.z },
                        { endX, corridorLow, low.z },
                        { endX, corridorLow, high.z },
                        { endX, low.y, high.z },
                        { nx, 0.0f, 0.0f });
                    addFace(
                        { endX, corridorHigh, low.z },
                        { endX, high.y, low.z },
                        { endX, high.y, high.z },
                        { endX, corridorHigh, high.z },
                        { nx, 0.0f, 0.0f });
                    if (passageTop < high.z - 0.01f)
                    {
                        addFace(
                            { endX, corridorLow, passageTop },
                            { endX, corridorHigh, passageTop },
                            { endX, corridorHigh, high.z },
                            { endX, corridorLow, high.z },
                            { nx, 0.0f, 0.0f });
                    }
                }

                addFace(
                    { low.x, corridorLow, low.z },
                    { high.x, corridorLow, low.z },
                    { high.x, corridorLow, passageTop },
                    { low.x, corridorLow, passageTop },
                    { 0.0f, 1.0f, 0.0f });
                addFace(
                    { high.x, corridorHigh, low.z },
                    { low.x, corridorHigh, low.z },
                    { low.x, corridorHigh, passageTop },
                    { high.x, corridorHigh, passageTop },
                    { 0.0f, -1.0f, 0.0f });
                if (passageTop < high.z - 0.01f)
                {
                    addFace(
                        { low.x, corridorLow, passageTop },
                        { high.x, corridorLow, passageTop },
                        { high.x, corridorHigh, passageTop },
                        { low.x, corridorHigh, passageTop },
                        { 0.0f, 0.0f, -1.0f });
                    addFace(
                        { low.x, low.y, high.z },
                        { high.x, low.y, high.z },
                        { high.x, high.y, high.z },
                        { low.x, high.y, high.z },
                        { 0.0f, 0.0f, 1.0f });
                }
                else
                {
                    addFace(
                        { low.x, low.y, high.z },
                        { high.x, low.y, high.z },
                        { high.x, corridorLow, high.z },
                        { low.x, corridorLow, high.z },
                        { 0.0f, 0.0f, 1.0f });
                    addFace(
                        { low.x, corridorHigh, high.z },
                        { high.x, corridorHigh, high.z },
                        { high.x, high.y, high.z },
                        { low.x, high.y, high.z },
                        { 0.0f, 0.0f, 1.0f });
                }
                addFace(
                    { low.x, corridorLow, low.z },
                    { high.x, corridorLow, low.z },
                    { high.x, low.y, low.z },
                    { low.x, low.y, low.z },
                    { 0.0f, 0.0f, -1.0f });
                addFace(
                    { low.x, high.y, low.z },
                    { high.x, high.y, low.z },
                    { high.x, corridorHigh, low.z },
                    { low.x, corridorHigh, low.z },
                    { 0.0f, 0.0f, -1.0f });
            }
            else if (passageAxis
                    == FirstPersonPassageAxis::y
                && passageTop > low.z + 0.01f)
            {
                const float corridorLow =
                    float(centreTile.x)
                    + kFirstPersonPathDeckInset;
                const float corridorHigh =
                    float(centreTile.x)
                    + kFirstPersonPathDeckMax;

                addFace(
                    { low.x, high.y, low.z },
                    { low.x, low.y, low.z },
                    { low.x, low.y, high.z },
                    { low.x, high.y, high.z },
                    { -1.0f, 0.0f, 0.0f });
                addFace(
                    { high.x, low.y, low.z },
                    { high.x, high.y, low.z },
                    { high.x, high.y, high.z },
                    { high.x, low.y, high.z },
                    { 1.0f, 0.0f, 0.0f });

                for (const auto endY :
                     { low.y, high.y })
                {
                    const float ny =
                        endY == low.y ? -1.0f : 1.0f;
                    addFace(
                        { low.x, endY, low.z },
                        { corridorLow, endY, low.z },
                        { corridorLow, endY, high.z },
                        { low.x, endY, high.z },
                        { 0.0f, ny, 0.0f });
                    addFace(
                        { corridorHigh, endY, low.z },
                        { high.x, endY, low.z },
                        { high.x, endY, high.z },
                        { corridorHigh, endY, high.z },
                        { 0.0f, ny, 0.0f });
                    if (passageTop < high.z - 0.01f)
                    {
                        addFace(
                            { corridorLow, endY, passageTop },
                            { corridorHigh, endY, passageTop },
                            { corridorHigh, endY, high.z },
                            { corridorLow, endY, high.z },
                            { 0.0f, ny, 0.0f });
                    }
                }

                addFace(
                    { corridorLow, high.y, low.z },
                    { corridorLow, low.y, low.z },
                    { corridorLow, low.y, passageTop },
                    { corridorLow, high.y, passageTop },
                    { 1.0f, 0.0f, 0.0f });
                addFace(
                    { corridorHigh, low.y, low.z },
                    { corridorHigh, high.y, low.z },
                    { corridorHigh, high.y, passageTop },
                    { corridorHigh, low.y, passageTop },
                    { -1.0f, 0.0f, 0.0f });
                if (passageTop < high.z - 0.01f)
                {
                    addFace(
                        { corridorLow, low.y, passageTop },
                        { corridorHigh, low.y, passageTop },
                        { corridorHigh, high.y, passageTop },
                        { corridorLow, high.y, passageTop },
                        { 0.0f, 0.0f, -1.0f });
                    addFace(
                        { low.x, low.y, high.z },
                        { high.x, low.y, high.z },
                        { high.x, high.y, high.z },
                        { low.x, high.y, high.z },
                        { 0.0f, 0.0f, 1.0f });
                }
                else
                {
                    addFace(
                        { low.x, low.y, high.z },
                        { corridorLow, low.y, high.z },
                        { corridorLow, high.y, high.z },
                        { low.x, high.y, high.z },
                        { 0.0f, 0.0f, 1.0f });
                    addFace(
                        { corridorHigh, low.y, high.z },
                        { high.x, low.y, high.z },
                        { high.x, high.y, high.z },
                        { corridorHigh, high.y, high.z },
                        { 0.0f, 0.0f, 1.0f });
                }
                addFace(
                    { low.x, low.y, low.z },
                    { corridorLow, low.y, low.z },
                    { corridorLow, high.y, low.z },
                    { low.x, high.y, low.z },
                    { 0.0f, 0.0f, -1.0f });
                addFace(
                    { corridorHigh, low.y, low.z },
                    { high.x, low.y, low.z },
                    { high.x, high.y, low.z },
                    { corridorHigh, high.y, low.z },
                    { 0.0f, 0.0f, -1.0f });
            }
            else
            {
                addFace(
                    { low.x, low.y, low.z },
                    { high.x, low.y, low.z },
                    { high.x, low.y, high.z },
                    { low.x, low.y, high.z },
                    { 0.0f, -1.0f, 0.0f });
                addFace(
                    { high.x, low.y, low.z },
                    { high.x, high.y, low.z },
                    { high.x, high.y, high.z },
                    { high.x, low.y, high.z },
                    { 1.0f, 0.0f, 0.0f });
                addFace(
                    { high.x, high.y, low.z },
                    { low.x, high.y, low.z },
                    { low.x, high.y, high.z },
                    { high.x, high.y, high.z },
                    { 0.0f, 1.0f, 0.0f });
                addFace(
                    { low.x, high.y, low.z },
                    { low.x, low.y, low.z },
                    { low.x, low.y, high.z },
                    { low.x, high.y, high.z },
                    { -1.0f, 0.0f, 0.0f });
                addFace(
                    { low.x, low.y, high.z },
                    { high.x, low.y, high.z },
                    { high.x, high.y, high.z },
                    { low.x, high.y, high.z },
                    { 0.0f, 0.0f, 1.0f });
                addFace(
                    { low.x, high.y, low.z },
                    { high.x, high.y, low.z },
                    { high.x, low.y, low.z },
                    { low.x, low.y, low.z },
                    { 0.0f, 0.0f, -1.0f });
            }

            const auto edgeLength =
                [](FirstPersonVec3 a,
                    FirstPersonVec3 b) {
                    const auto d = Sub(b, a);
                    return std::sqrt(FpDot(d, d));
                };
            const ImageId diagnosticImage{
                bodyImages[0]
            };

            for (const auto& face : faces)
            {
                FirstPersonSurface surface{};
                surface.image = diagnosticImage;
                surface.cameraIndependent = true;
                surface.textureFallbackOnly = true;
                surface.reconstructionGroup = group.key;
                surface.diagnosticReconstructionKind = 3;
                surface.diagnosticReconstructionAnchor =
                    group.anchor;
                surface.diagnosticSemanticGroup = group.key;
                surface.diagnosticSemanticRole =
                    static_cast<uint8_t>(
                        FirstPersonPaintSemanticRole::
                            structureBody);
                surface.exteriorOnly = true;
                surface.outwardNormal = face.normal;

                const FirstPersonVec3 faceCenter{
                    0.25f * (
                        face.corners[0].x
                        + face.corners[1].x
                        + face.corners[2].x
                        + face.corners[3].x),
                    0.25f * (
                        face.corners[0].y
                        + face.corners[1].y
                        + face.corners[2].y
                        + face.corners[3].y),
                    0.25f * (
                        face.corners[0].z
                        + face.corners[1].z
                        + face.corners[2].z
                        + face.corners[3].z),
                };
                surface.diagnosticSourceTile =
                    sourceTileForPoint(faceCenter);
                surface.gpuRegion =
                    FirstPersonGpuRegionKey(
                        int32_t(std::floor(
                            faceCenter.x
                            / float(kCoordsXYStep))),
                        int32_t(std::floor(
                            faceCenter.y
                            / float(kCoordsXYStep))));

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
                std::vector<uint8_t> pixels(
                    size_t(width) * size_t(height));
                uint64_t fingerprint =
                    14695981039346656037ull;
                ExtendStableKey(fingerprint, width);
                ExtendStableKey(fingerprint, height);
                for (int32_t y = 0; y < height; ++y)
                for (int32_t x = 0; x < width; ++x)
                {
                    const uint8_t pixel =
                        FirstPersonMissingMaterialPixel(
                            x, y);
                    pixels[
                        size_t(y) * size_t(width)
                        + size_t(x)] = pixel;
                    fingerprint ^= pixel;
                    fingerprint *= 1099511628211ull;
                }

                surface.persistentPixels =
                    std::make_shared<
                        std::vector<uint8_t>>(
                            std::move(pixels));
                surface.immutableWidth =
                    int16_t(width);
                surface.immutableHeight =
                    int16_t(height);
                surface.persistentBitmap = true;
                surface.immutableFingerprint =
                    fingerprint;
                EmitQuad(
                    surface,
                    { {
                        { face.corners[0],
                          0.0f, 0.0f },
                        { face.corners[1],
                          float(width), 0.0f },
                        { face.corners[2],
                          float(width),
                          float(height) },
                        { face.corners[3],
                          0.0f,
                          float(height) },
                    } });
                result.surfaces.emplace_back(
                    std::move(surface));
            }

            const FirstPersonVec3 center{
                0.5f * (low.x + high.x),
                0.5f * (low.y + high.y),
                0.5f * (low.z + high.z),
            };
            const float hx =
                0.5f * (high.x - low.x);
            const float hy =
                0.5f * (high.y - low.y);
            const float hz =
                0.5f * (high.z - low.z);
            const float radius =
                std::sqrt(
                    hx * hx + hy * hy + hz * hz)
                + 2.0f;
            for (auto& surface : result.surfaces)
            {
                surface.hasSemanticBounds = true;
                surface.semanticCenter = center;
                surface.semanticRadius = radius;
            }

            return result;
        }

        struct LargeSceneryJob
        {
            LargeSceneryEntry entry{};
            LargeSceneryElement element{};
            std::vector<LargeSceneryTile> tiles;
            std::vector<std::pair<CoordsXY, uint64_t>> tileSignatures;
            ReconstructionGroupInfo group{};
            FirstPersonSpriteSnapshot sprites;
            std::optional<
                FirstPersonWalkabilityConstraint>
                passage;
            size_t nextImage = 0;
            bool submitted = false;
            int32_t minX{}, minY{}, maxX{}, maxY{};
        };
        static std::unordered_map<uint64_t, std::shared_ptr<LargeSceneryJob>> _largeSceneryJobs;

        void QueueLargeSceneryGeometry(const LargeSceneryElement& large,
            const LargeSceneryEntry& entry, const ReconstructionGroupInfo& group, uint64_t frame)
        {
            if (!_snapshotBudget.available() || !_reconstructionWorker.hasCapacity()) return;
            if (!_largeSceneryJobs.contains(group.key) && _largeSceneryJobs.size() >= 8) return;
            auto& slot = _largeSceneryJobs[group.key];
            if (!slot)
            {
                slot = std::make_shared<LargeSceneryJob>();
                slot->entry = entry;
                slot->entry.tiles = {}; // never retain the object's span or text pointer
                slot->entry.text = nullptr;
                slot->tiles.assign(entry.tiles.begin(), entry.tiles.end());
                slot->element = large;
                slot->group = group;
                slot->minX = slot->maxX = int32_t(group.anchor.x) / kCoordsXYStep;
                slot->minY = slot->maxY = int32_t(group.anchor.y) / kCoordsXYStep;
                for (const auto& tile : slot->tiles)
                {
                    const auto offset = CoordsXY{tile.offset.x, tile.offset.y}.rotate(large.getDirection());
                    const auto x = int32_t(group.anchor.x + offset.x) / kCoordsXYStep;
                    const auto y = int32_t(group.anchor.y + offset.y) / kCoordsXYStep;
                    const CoordsXY location{x * kCoordsXYStep, y * kCoordsXYStep};
                    slot->tileSignatures.emplace_back(location, NativeTileSignature(location));
                    slot->minX = std::min(slot->minX, x); slot->maxX = std::max(slot->maxX, x);
                    slot->minY = std::min(slot->minY, y); slot->maxY = std::max(slot->maxY, y);
                }
            }
            const auto job = slot;
            if (job->submitted) return;
            while (job->nextImage < job->tiles.size() * 4)
            {
                if (!job->sprites.capture(entry.image + 4 + uint32_t(job->nextImage), _snapshotBudget)) return;
                ++job->nextImage;
            }
            if (!LargeSceneryInstanceComplete(entry, group, large.getDirection() & 3u))
            {
                _largeSceneryJobs.erase(group.key);
                return;
            }
            const auto epoch = _reconstructionWorker.epoch();
            job->submitted = _reconstructionWorker.submit([job, frame, epoch]() {
                auto metadata = job->entry;
                metadata.tiles = job->tiles;
                const auto model = BuildLargeSceneryAssetModel(metadata);
                auto result = BuildLargeSceneryInstanceGeometry(job->element, metadata,
                    model, job->group, job->group.key, frame, job->sprites, epoch);
                return [job, result = std::move(result)]() mutable {
                    const auto pending = _largeSceneryJobs.find(job->group.key);
                    if (pending == _largeSceneryJobs.end() || pending->second != job) return;
                    for (const auto& [tile, signature] : job->tileSignatures)
                        if (NativeTileSignature(tile) != signature)
                        {
                            _largeSceneryJobs.erase(pending);
                            return;
                        }
                    auto existing = _largeSceneryGeometryCache.find(job->group.key);
                    if (existing != _largeSceneryGeometryCache.end())
                    {
                        MarkLargeSceneryGeometryRegionsDirty(existing->second);
                        UnregisterLargeSceneryRegionMembership(job->group.key, existing->second);
                    }
                    MarkLargeSceneryGeometryRegionsDirty(result);
                    RegisterLargeSceneryRegionMembership(job->group.key, result);
                    _largeSceneryGeometryCache[job->group.key] = std::move(result);
                    _largeSceneryJobs.erase(pending);
                };
            }, [job] {
                const auto it = _largeSceneryJobs.find(job->group.key);
                if (it != _largeSceneryJobs.end() && it->second == job)
                    _largeSceneryJobs.erase(it);
            });
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
            // the static-tile semantic cache. Asset/direction carving decisions
            // and per-instance results are cached, including failures, so an
            // unsupported asset does not repeat identical cold reconstruction
            // work on every frame.
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

                    QueueLargeSceneryGeometry(*large, *entry, group, frame);
                }
            }

            // World-fixed large-scenery geometry is retained until native
            // invalidation or park/POV teardown. Camera direction is not an
            // invalidation source.
        }

        struct ParkEntranceJob
        {
            ReconstructionGroupInfo group{};
            ObjectEntryIndex entryIndex{};
            std::array<ImageIndex, 12> bodyImages{};
            uint8_t direction = 0;
            int32_t clearanceHeight = 0;
            uint64_t signature = 0;
            std::array<
                std::pair<CoordsXY, uint64_t>, 3>
                tileSignatures{};
            FirstPersonSpriteSnapshot sprites;
            size_t nextImage = 0;
            bool submitted = false;
        };
        static std::unordered_map<
            uint64_t, std::shared_ptr<ParkEntranceJob>>
            _parkEntranceJobs;

        void QueueParkEntranceGeometry(
            const EntranceElement& centre,
            const EntranceObject& object,
            const ReconstructionGroupInfo& group,
            uint64_t frame)
        {
            if (!_snapshotBudget.available()
                || !_reconstructionWorker.hasCapacity())
                return;
            if (!_parkEntranceJobs.contains(group.key)
                && _parkEntranceJobs.size() >= 8)
                return;

            auto& slot =
                _parkEntranceJobs[group.key];
            if (!slot)
            {
                slot =
                    std::make_shared<ParkEntranceJob>();
                slot->group = group;
                slot->entryIndex =
                    centre.getEntryIndex();
                slot->direction =
                    static_cast<uint8_t>(
                        centre.getDirection()) & 3u;
                slot->clearanceHeight =
                    centre.getClearanceZ()
                    - centre.getBaseZ();
                const auto centreTile =
                    ParkEntranceSequenceTile(
                        group, slot->direction,
                        ParkEntranceSequence::centre);
                slot->passage =
                    FindFirstPersonGuaranteedPassage(
                        centreTile,
                        centre.getBaseZ());
                uint64_t signature =
                    14695981039346656037ull;
                ExtendStableKey(
                    signature, group.key);
                ExtendStableKey(
                    signature,
                    uint32_t(slot->clearanceHeight));
                ExtendStableKey(
                    signature,
                    slot->passage.has_value() ? 1u : 0u);
                if (slot->passage.has_value())
                {
                    ExtendStableKey(
                        signature,
                        uint32_t(slot->passage->baseZ));
                    ExtendStableKey(
                        signature,
                        uint32_t(
                            slot->passage->connectedSides));
                    ExtendStableKey(
                        signature,
                        slot->passage
                                ->guaranteedThroughPassage
                            ? 1u : 0u);
                }

                size_t imageSlot = 0;
                for (const auto sequence : {
                         ParkEntranceSequence::centre,
                         ParkEntranceSequence::left,
                         ParkEntranceSequence::right })
                {
                    for (uint8_t nativeDirection = 0;
                         nativeDirection < 4;
                         ++nativeDirection)
                    {
                        const auto image =
                            object.GetImage(
                                sequence,
                                nativeDirection);
                        slot->bodyImages[
                            imageSlot++] = image;
                        ExtendStableKey(
                            signature, image);
                    }
                }

                size_t index = 0;
                for (const auto sequence : {
                         ParkEntranceSequence::centre,
                         ParkEntranceSequence::left,
                         ParkEntranceSequence::right })
                {
                    const auto tile =
                        ParkEntranceSequenceTile(
                            group, slot->direction,
                            sequence);
                    const uint64_t tileSignature =
                        NativeTileSignature(tile);
                    slot->tileSignatures[index++] = {
                        tile, tileSignature
                    };
                    ExtendStableKey(
                        signature, tileSignature);
                }
                slot->signature = signature;
            }

            const auto job = slot;
            if (job->submitted)
                return;

            while (job->nextImage
                < job->bodyImages.size())
            {
                const ImageIndex image =
                    job->bodyImages[job->nextImage];
                if (!job->sprites.capture(
                        image, _snapshotBudget))
                    return;
                ++job->nextImage;
            }

            if (!ParkEntranceInstanceComplete(
                    group, job->direction,
                    job->entryIndex))
            {
                _parkEntranceJobs.erase(group.key);
                return;
            }

            job->submitted =
                _reconstructionWorker.submit(
                    [job, frame]() {
                        auto result =
                            BuildParkEntranceInstanceGeometry(
                                job->bodyImages,
                                job->direction,
                                job->clearanceHeight,
                                job->group,
                                job->signature,
                                frame,
                                job->sprites,
                                job->passage);
                        return [
                            job,
                            result =
                                std::move(result)
                        ]() mutable {
                            const auto pending =
                                _parkEntranceJobs.find(
                                    job->group.key);
                            if (pending
                                    == _parkEntranceJobs.end()
                                || pending->second != job)
                                return;

                            for (const auto& [
                                     tile, signature] :
                                 job->tileSignatures)
                            {
                                if (NativeTileSignature(tile)
                                    != signature)
                                {
                                    _parkEntranceJobs.erase(
                                        pending);
                                    return;
                                }
                            }

                            auto existing =
                                _parkEntranceGeometryCache.find(
                                    job->group.key);
                            if (existing
                                != _parkEntranceGeometryCache.end())
                            {
                                MarkParkEntranceGeometryRegionsDirty(
                                    existing->second);
                                UnregisterParkEntranceRegionMembership(
                                    job->group.key,
                                    existing->second);
                            }

                            result.tileSignatures =
                                job->tileSignatures;
                            MarkParkEntranceGeometryRegionsDirty(
                                result);
                            RegisterParkEntranceRegionMembership(
                                job->group.key, result);
                            _parkEntranceGeometryCache[
                                job->group.key] =
                                std::move(result);
                            _parkEntranceJobs.erase(pending);
                        };
                    },
                    [job] {
                        const auto found =
                            _parkEntranceJobs.find(
                                job->group.key);
                        if (found
                                != _parkEntranceJobs.end()
                            && found->second == job)
                        {
                            _parkEntranceJobs.erase(found);
                        }
                    });
        }

        void UpdateParkEntranceReconstructions(
            FirstPersonScene& scene, uint64_t frame)
        {
            const bool enabled =
                ParkEntranceGeometryAllowedForView(
                    scene.options.viewFlags);
            if (_parkEntranceGeometryEnabled != enabled)
            {
                for (const auto& [groupKey, cached] :
                     _parkEntranceGeometryCache)
                {
                    (void)groupKey;
                    MarkParkEntranceGeometryRegionsDirty(
                        cached);
                }
                _parkEntranceGeometryEnabled = enabled;
            }
            if (!enabled)
                return;

            std::unordered_set<uint64_t> seenGroups;
            seenGroups.reserve(
                scene.visibleTiles.size() / 4 + 1);

            for (const auto tile : scene.visibleTiles)
            {
                const auto tileKey =
                    TerrainKey(
                        tile.x / kCoordsXYStep,
                        tile.y / kCoordsXYStep);
                const auto paintCache =
                    _staticPaintCache.find(tileKey);
                if (paintCache
                        == _staticPaintCache.end()
                    || !paintCache->second.valid
                    || paintCache->second.dirty)
                    continue;

                for (const auto& group :
                     paintCache->second.reconstructionGroups)
                {
                    if (group.type
                            != TileElementType::entrance
                        || !seenGroups.insert(
                                group.key).second)
                        continue;

                    auto cached =
                        _parkEntranceGeometryCache.find(
                            group.key);
                    if (cached
                            != _parkEntranceGeometryCache.end()
                        && !cached->second.dirty)
                    {
                        cached->second.lastSeen = frame;
                        continue;
                    }

                    EntranceElement* source = nullptr;
                    auto* element =
                        MapGetFirstElementAt(
                            group.sourceTile);
                    if (element != nullptr)
                    {
                        do
                        {
                            if (element->getType()
                                    != TileElementType::entrance
                                || element->isGhost()
                                || element->isInvisible())
                                continue;
                            const auto candidateGroup =
                                GetReconstructionGroup(
                                    group.sourceTile,
                                    element);
                            if (candidateGroup.has_value()
                                && candidateGroup->key
                                    == group.key)
                            {
                                source =
                                    element->asEntrance();
                                break;
                            }
                        } while (!(element++)->isLastForTile());
                    }

                    if (source == nullptr)
                    {
                        if (cached
                            != _parkEntranceGeometryCache.end())
                        {
                            MarkParkEntranceGeometryRegionsDirty(
                                cached->second);
                            UnregisterParkEntranceRegionMembership(
                                group.key,
                                cached->second);
                            _parkEntranceGeometryCache.erase(
                                cached);
                        }
                        continue;
                    }

                    const uint8_t direction =
                        static_cast<uint8_t>(
                            source->getDirection()) & 3u;
                    const auto entryIndex =
                        source->getEntryIndex();
                    auto* centre =
                        FindParkEntranceSequence(
                            group, direction,
                            entryIndex,
                            ParkEntranceSequence::centre);
                    auto& objectManager =
                        GetContext()->GetObjectManager();
                    const auto* object =
                        objectManager.GetLoadedObject<
                            EntranceObject>(
                                entryIndex);
                    if (centre == nullptr
                        || object == nullptr
                        || !ParkEntranceInstanceComplete(
                            group, direction,
                            entryIndex))
                    {
                        if (cached
                            != _parkEntranceGeometryCache.end())
                        {
                            MarkParkEntranceGeometryRegionsDirty(
                                cached->second);
                            UnregisterParkEntranceRegionMembership(
                                group.key,
                                cached->second);
                            _parkEntranceGeometryCache.erase(
                                cached);
                        }
                        continue;
                    }

                    QueueParkEntranceGeometry(
                        *centre, *object, group,
                        frame);
                }
            }
        }

        [[nodiscard]] bool
            IsReconstructedParkEntranceBody(
                const FirstPersonSurface& surface,
                uint64_t frame)
        {
            // The grouped hull replaces only the entrance object's semantic
            // body. The same tile painter can also emit supports, the path
            // deck and scrolling-sign artwork; those are separate native
            // structures and must survive the body swap.
            if (!_parkEntranceGeometryEnabled
                || surface.reconstructionGroup == 0
                || surface.diagnosticSemanticRole
                    != static_cast<uint8_t>(
                        FirstPersonPaintSemanticRole::structureBody))
                return false;
            const auto found =
                _parkEntranceGeometryCache.find(
                    surface.reconstructionGroup);
            return found
                    != _parkEntranceGeometryCache.end()
                && !found->second.dirty
                && found->second.lastSeen == frame
                && !found->second.surfaces.empty();
        }

        [[nodiscard]] bool
            AlignParkEntranceSignToGroupedBody(
                FirstPersonSurface& surface,
                uint64_t frame)
        {
            if (surface.reconstructionGroup == 0
                || surface.diagnosticSemanticRole
                    != static_cast<uint8_t>(
                        FirstPersonPaintSemanticRole::sign)
                || !surface.coplanarOwner)
                return false;

            const auto geometry =
                _parkEntranceGeometryCache.find(
                    surface.reconstructionGroup);
            if (geometry
                    == _parkEntranceGeometryCache.end()
                || geometry->second.dirty
                || geometry->second.lastSeen != frame
                || geometry->second.surfaces.empty())
                return false;

            const auto& pixels =
                surface.immutablePixelData();
            if (pixels.empty()
                || surface.immutableWidth <= 0
                || surface.immutableHeight <= 0)
                return false;

            const std::array<FirstPersonVec3, 4>
                sourceFace{ {
                    surface.triangles[0].world,
                    surface.triangles[1].world,
                    surface.triangles[2].world,
                    surface.triangles[5].world,
                } };
            const auto sourceHorizontal =
                Sub(sourceFace[1], sourceFace[0]);
            const auto sourceVertical =
                Sub(sourceFace[3], sourceFace[0]);
            auto signNormal =
                FpCross(
                    sourceHorizontal, sourceVertical);
            const float signNormalLength =
                std::sqrt(
                    FpDot(signNormal, signNormal));
            if (!(signNormalLength > 0.001f))
                return false;
            signNormal =
                Mul(
                    signNormal,
                    1.0f / signNormalLength);

            const FirstPersonVec3 sourceCenter{
                0.25f * (
                    sourceFace[0].x
                    + sourceFace[1].x
                    + sourceFace[2].x
                    + sourceFace[3].x),
                0.25f * (
                    sourceFace[0].y
                    + sourceFace[1].y
                    + sourceFace[2].y
                    + sourceFace[3].y),
                0.25f * (
                    sourceFace[0].z
                    + sourceFace[1].z
                    + sourceFace[2].z
                    + sourceFace[3].z),
            };

            const FirstPersonSurface* best = nullptr;
            FirstPersonVec3 bestNormal{};
            float bestPlaneDelta = 0.0f;
            float bestDistance =
                std::numeric_limits<float>::max();

            for (const auto& candidate :
                 geometry->second.surfaces)
            {
                if (!candidate.exteriorOnly)
                    continue;
                // Scrolling text is part of the centre entrance sequence.
                // Side structures in the same grouped replacement are never
                // eligible carriers.
                if (candidate.diagnosticSourceTile.x
                        != surface.diagnosticSourceTile.x
                    || candidate.diagnosticSourceTile.y
                        != surface.diagnosticSourceTile.y)
                    continue;

                auto normal =
                    candidate.outwardNormal;
                const float normalLength =
                    std::sqrt(
                        FpDot(normal, normal));
                if (!(normalLength > 0.001f))
                    continue;
                normal =
                    Mul(
                        normal,
                        1.0f / normalLength);

                // The native sign plane already establishes which physical
                // side of the entrance owns the text. Preserve that sidedness
                // rather than choosing a merely parallel opposite face.
                if (std::abs(normal.z) > 0.25f
                    || FpDot(normal, signNormal) < 0.90f)
                    continue;

                const float plane =
                    FpDot(
                        normal,
                        candidate.triangles[0].world);
                const float planeDelta =
                    plane
                    - FpDot(
                        normal,
                        sourceCenter);
                const float distance =
                    std::abs(planeDelta);
                if (distance >= bestDistance)
                    continue;

                best = &candidate;
                bestNormal = normal;
                bestPlaneDelta = planeDelta;
                bestDistance = distance;
            }

            if (best == nullptr)
                return false;

            // The semantic sign surface has already inverse-projected the
            // exact scrolling-text raster rectangle onto the authoritative
            // centre gate plane. That gives us the correct entrance-local
            // horizontal and vertical placement. Moving that rectangle to a
            // different depth and then preserving its old isometric screen
            // coordinates would necessarily alter X/Z and make the text slide
            // relative to the architecture. Keep its physical placement and
            // translate only along the owning face normal.
            std::array<FirstPersonVec3, 4> mapped{};
            for (size_t i = 0; i < mapped.size(); ++i)
            {
                mapped[i] =
                    Add(
                        sourceFace[i],
                        Mul(
                            bestNormal,
                            bestPlaneDelta));
            }

            surface.triangles[0].world = mapped[0];
            surface.triangles[1].world = mapped[1];
            surface.triangles[2].world = mapped[2];
            surface.triangles[3].world = mapped[0];
            surface.triangles[4].world = mapped[2];
            surface.triangles[5].world = mapped[3];
            surface.exteriorOnly = true;
            surface.outwardNormal = bestNormal;
            return true;
        }

        [[nodiscard]] bool IsReconstructedLargeSceneryBody(
            const FirstPersonSurface& surface, uint64_t frame)
        {
            if (!_largeSceneryGeometryEnabled || surface.reconstructionGroup == 0
                || !surface.image.HasValue())
                return false;
            const auto found = _largeSceneryGeometryCache.find(surface.reconstructionGroup);
            if (found == _largeSceneryGeometryCache.end() || found->second.dirty
                || found->second.lastSeen != frame
                || found->second.surfaces.empty())
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
                && !surface.hasImmutablePixelData();
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
                    fingerprint, surface.exteriorOnly ? 1 : 0);
                if (surface.exteriorOnly)
                {
                    extendFloat(surface.outwardNormal.x);
                    extendFloat(surface.outwardNormal.y);
                    extendFloat(surface.outwardNormal.z);
                }
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
                ExtendStableKey(
                    fingerprint,
                    component.visualDeckEdgeMask);
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
            ExtendStableKey(key, surface.exteriorOnly ? 1 : 0);
            if (surface.exteriorOnly)
            {
                ExtendStableKey(
                    key,
                    uint64_t(int64_t(std::llround(
                        double(surface.outwardNormal.x) * 1024.0))));
                ExtendStableKey(
                    key,
                    uint64_t(int64_t(std::llround(
                        double(surface.outwardNormal.y) * 1024.0))));
                ExtendStableKey(
                    key,
                    uint64_t(int64_t(std::llround(
                        double(surface.outwardNormal.z) * 1024.0))));
            }
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
            if (cached.hasBounds)
            {
                VisitFirstPersonGpuRegionsInTileBounds(
                    cached.minTileX, cached.minTileY,
                    cached.maxTileX, cached.maxTileY,
                    [&](uint64_t regionKey) {
                        const auto found =
                            _trackTrajectoryGroupsByBoundsRegion.find(
                                regionKey);
                        if (found
                            == _trackTrajectoryGroupsByBoundsRegion.end())
                            return;
                        found->second.erase(groupKey);
                        if (found->second.empty())
                        {
                            _trackTrajectoryGroupsByBoundsRegion.erase(
                                found);
                        }
                    });
            }
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
            if (cached.hasBounds)
            {
                VisitFirstPersonGpuRegionsInTileBounds(
                    cached.minTileX, cached.minTileY,
                    cached.maxTileX, cached.maxTileY,
                    [&](uint64_t regionKey) {
                        _trackTrajectoryGroupsByBoundsRegion[
                            regionKey].insert(groupKey);
                    });
            }
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
                ++trajectory.materialVersion;
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
            ++trajectory.materialVersion;
            MarkTrackTrajectoryRegionsDirty(trajectory);
        }

        void AppendFirstPersonTrackArtworkProjections(
            const FirstPersonSurface& carrier,
            const TrackTrajectoryCacheEntry& trajectory,
            std::vector<FirstPersonSurface>& output,
            const FirstPersonSpriteSnapshot& sprites)
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
                        sprites.get(
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
                            sprites.get(
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
                uint64_t fingerprint =
                    14695981039346656037ull;
                ExtendStableKey(
                    fingerprint, width);
                ExtendStableKey(
                    fingerprint, height);
                for (const auto pixel : pixels)
                {
                    fingerprint ^= pixel;
                    fingerprint *= 1099511628211ull;
                }
                surface.persistentBitmap = true;
                surface.persistentPixels =
                    std::make_shared<
                        std::vector<uint8_t>>(
                            std::move(pixels));
                surface.immutableWidth =
                    int16_t(width);
                surface.immutableHeight =
                    int16_t(height);
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


        struct TrackMaterialJob
        {
            TrackTrajectoryCacheEntry snapshot;
            FirstPersonSpriteSnapshot sprites;
            bool submitted = false;
        };
        static std::unordered_map<uint64_t, std::shared_ptr<TrackMaterialJob>> _trackMaterialJobs;

        void EnsureTrackMaterials(uint64_t key, TrackTrajectoryCacheEntry& trajectory)
        {
            if (trajectory.dirty || trajectory.bakedMaterialVersion == trajectory.materialVersion) return;
            if (!_snapshotBudget.available() || !_reconstructionWorker.hasCapacity()) return;
            if (!_trackMaterialJobs.contains(key) && _trackMaterialJobs.size() >= 16) return;
            auto& slot = _trackMaterialJobs[key];
            if (slot && (slot->snapshot.materialVersion != trajectory.materialVersion
                || slot->snapshot.signature != trajectory.signature)) slot.reset();
            if (!slot)
            {
                slot = std::make_shared<TrackMaterialJob>();
                trajectory.materialRequest = ++_nextReconstructionRequest;
                slot->snapshot = trajectory;
                slot->snapshot.projectedSurfaces.clear();
            }
            const auto job = slot;
            if (job->submitted) return;
            for (const auto& views : job->snapshot.artworkProjections)
            for (const auto& view : views)
                if (!job->sprites.capture(view.image, _snapshotBudget)
                    || !job->sprites.capture(view.mask, _snapshotBudget)) return;
            job->submitted = _reconstructionWorker.submit([job, key] {
                std::unordered_map<size_t, std::vector<FirstPersonSurface>> result;
                for (size_t i = 0; i < job->snapshot.surfaces.size(); ++i)
                {
                    const auto& surface = job->snapshot.surfaces[i];
                    if (surface.artworkCarrier)
                        AppendFirstPersonTrackArtworkProjections(surface, job->snapshot, result[i], job->sprites);
                }
                return [job, key, result = std::move(result)]() mutable {
                    const auto pending = _trackMaterialJobs.find(key);
                    if (pending == _trackMaterialJobs.end() || pending->second != job) return;
                    _trackMaterialJobs.erase(pending);
                    const auto found = _trackTrajectoryCache.find(key);
                    if (found == _trackTrajectoryCache.end()) return;
                    auto& target = found->second;
                    if (target.dirty || target.materialRequest != job->snapshot.materialRequest
                        || target.materialVersion != job->snapshot.materialVersion) return;
                    target.projectedSurfaces = std::move(result);
                    target.bakedMaterialVersion = job->snapshot.materialVersion;
                    MarkTrackTrajectoryRegionsDirty(target);
                };
            }, [job, key] {
                const auto found = _trackMaterialJobs.find(key);
                if (found != _trackMaterialJobs.end() && found->second == job) _trackMaterialJobs.erase(found);
            });
        }
