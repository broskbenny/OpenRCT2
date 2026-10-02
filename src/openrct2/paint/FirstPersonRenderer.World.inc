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

        [[nodiscard]] TileElement*
            FindFirstPersonTrackOriginElement(
                const CoordsXYZ& sampleOrigin,
                const TrackElement& reference)
        {
            auto* element =
                MapGetFirstElementAt(sampleOrigin);
            if (element == nullptr)
                return nullptr;

            do
            {
                auto* candidate = element->asTrack();
                if (candidate == nullptr)
                    continue;
                if (candidate->getBaseZ()
                    != sampleOrigin.z)
                    continue;
                if (candidate->getDirection()
                    != reference.getDirection())
                    continue;
                if (candidate->getTrackType()
                    != reference.getTrackType())
                    continue;
                if (candidate->getSequenceIndex() != 0)
                    continue;
                if (candidate->getRideIndex()
                    != reference.getRideIndex())
                    continue;

                return element;
            } while (!(element++)->isLastForTile());

            return nullptr;
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
            scene.activeTrackRegions.reserve(
                scene.visibleTiles.size() / 8 + 1);
            const auto activateTrajectoryRegions =
                [&](const TrackTrajectoryCacheEntry& trajectory) {
                    for (const auto& [regionKey, indices] :
                         trajectory.regionSurfaceIndices)
                    {
                        (void)indices;
                        scene.activeTrackRegions.push_back(regionKey);
                    }
                };

            for (const auto tile : scene.visibleTiles)
            {
                // Stable tiles already carry the authoritative reconstruction
                // groups discovered when their semantic cache was built. Reuse
                // those groups instead of walking every tile element again on
                // every rendered frame. Cold/dirty/missing trajectories fall
                // through to the live authoritative scan below.
                const auto tileKey = TerrainKey(
                    tile.x / kCoordsXYStep,
                    tile.y / kCoordsXYStep);
                const auto paintCache =
                    _staticPaintCache.find(tileKey);
                bool needsLiveTrackScan =
                    paintCache == _staticPaintCache.end()
                    || !paintCache->second.valid
                    || paintCache->second.dirty;
                if (!needsLiveTrackScan)
                {
                    for (const auto& group :
                         paintCache->second.reconstructionGroups)
                    {
                        if (group.type
                            != TileElementType::track
                            || !seenGroups.insert(
                                group.key).second)
                            continue;

                        const auto trajectory =
                            _trackTrajectoryCache.find(
                                group.key);
                        if (trajectory
                                == _trackTrajectoryCache.end()
                            || trajectory->second.dirty)
                        {
                            // Let the live scan resolve unsupported, newly
                            // changed, or not-yet-built track authoritatively.
                            seenGroups.erase(group.key);
                            needsLiveTrackScan = true;
                            break;
                        }

                        trajectory->second.lastSeen = frame;
                        activateTrajectoryRegions(
                            trajectory->second);
                    }
                    if (!needsLiveTrackScan)
                        continue;
                }

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
                        activateTrajectoryRegions(it->second);
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
                    activateTrajectoryRegions(
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

        void InvalidateFirstPersonRegionBounds(
            int32_t dirtyX0, int32_t dirtyY0,
            int32_t dirtyX1, int32_t dirtyY1)
        {
            const auto map = getGameState().mapSize;
            const auto invalidateNode =
                [&](auto&& self,
                    int32_t x0, int32_t y0,
                    int32_t x1, int32_t y1) -> void {
                    if (x0 >= x1 || y0 >= y1
                        || dirtyX1 < x0 || dirtyX0 >= x1
                        || dirtyY1 < y0 || dirtyY0 >= y1)
                        return;

                    if (x1 - x0 <= 16
                        && y1 - y0 <= 16)
                    {
                        const auto found =
                            _regionBounds.find(
                                TerrainKey(x0, y0));
                        if (found != _regionBounds.end()
                            && found->second.x1 == x1
                            && found->second.y1 == y1)
                        {
                            found->second.dirty = true;
                        }
                        return;
                    }

                    // Mirror VisitFirstPersonRegions exactly. The leaf
                    // partition depends only on map dimensions, so a local
                    // edit can descend directly to the cached leaves it
                    // intersects instead of scanning every cached leaf.
                    if (y1 - y0 >= x1 - x0)
                    {
                        const int32_t mid =
                            y0 + (y1 - y0) / 2;
                        self(
                            self, x0, y0, x1, mid);
                        self(
                            self, x0, mid, x1, y1);
                    }
                    else
                    {
                        const int32_t mid =
                            x0 + (x1 - x0) / 2;
                        self(
                            self, x0, y0, mid, y1);
                        self(
                            self, mid, y0, x1, y1);
                    }
                };
            invalidateNode(
                invalidateNode,
                0, 0, map.x, map.y);
        }

        FirstPersonRegionBounds CachedRegionBounds(
            int32_t x0, int32_t y0,
            int32_t x1, int32_t y1)
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
                    const uint32_t sourceGeneration =
                        getGameState().currentTicks;
                    for(int32_t ty=y0;ty<y1;++ty)
                    for(int32_t tx=x0;tx<x1;++tx)
                    {
                        const CoordsXY p{tx*kCoordsXYStep,ty*kCoordsXYStep};
                        if (!MapIsLocationValid(p)) continue;

                        // The leaf bound and per-tile visibility cache require
                        // the same authoritative tile walk. Publish both from
                        // this one snapshot so cold first-person entry does not
                        // immediately rescan every element in the visible leaf.
                        const auto snapshot =
                            ReadTileSemanticSnapshot(p);
                        auto& tileCache =
                            _staticPaintCache[TerrainKey(tx, ty)];
                        if (!snapshot.populated)
                        {
                            tileCache.visibilityBoundValid = false;
                            tileCache.visibilityDirty = false;
                            tileCache.lastVisibilityGeneration =
                                sourceGeneration;
                            continue;
                        }

                        if (tileCache.valid
                            && tileCache.signature
                                != snapshot.signature)
                        {
                            tileCache.dirty = true;
                        }
                        tileCache.signature = snapshot.signature;
                        tileCache.visibilityMinZ = snapshot.minZ;
                        tileCache.visibilityMaxZ = snapshot.maxZ;
                        tileCache.visibilityBoundValid = true;
                        tileCache.visibilityDirty = false;
                        tileCache.lastVisibilityGeneration =
                            sourceGeneration;

                        entry.populated = true;
                        entry.minZ = std::min(
                            entry.minZ, snapshot.minZ);
                        entry.maxZ = std::max(
                            entry.maxZ, snapshot.maxZ);
                    }
                    entry.dirty=false;
                }
                populated = entry.populated;
                low = float(entry.minZ);
                high = float(entry.maxZ);
            }
            const float dx =
                float(x1-x0)*0.5f*kTile;
            const float dy =
                float(y1-y0)*0.5f*kTile;
            const float dz =
                std::max(0.0f, (high-low)*0.5f);
            // Occupancy/paint bounds can underestimate source-art overhang.
            // Keep the existing conservative 512-unit allowance, but express
            // the region as its actual world box instead of an enclosing sphere.
            // Applying the halo at every hierarchy level also prevents a coarse
            // parent from culling artwork that extends beyond a child tile.
            constexpr float kArtworkHalo = 512.0f;
            return {
                {
                    float(x0+x1)*0.5f*kTile,
                    float(y0+y1)*0.5f*kTile,
                    (high+low)*0.5f
                },
                {
                    dx + kArtworkHalo,
                    dy + kArtworkHalo,
                    dz + kArtworkHalo
                },
                populated
            };
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

                    // Terrain artwork is deterministic from these surface fields.
                    // Avoid resolving object artwork/G1 metadata every rendered
                    // frame when neither native invalidation nor the staggered
                    // semantic fallback probe reported a change.
                    const uint8_t surfaceStyle = static_cast<uint8_t>(tile->getSurfaceObjectIndex());
                    const uint8_t grassVisualStage =
                        tile->getGrassLength() & 0x7u;
                    const ImageId waterOverlay = waterZ > baseZ
                        ? GetFirstPersonWaterOverlayImage(
                            *tile, opt.viewFlags, sourceRotation)
                        : ImageId{};
                    const auto semanticCache =
                        _staticPaintCache.find(terrainKey);
                    const bool semanticDirty =
                        semanticCache != _staticPaintCache.end()
                        && semanticCache->second.dirty;
                    const bool sourceStateMatches =
                        !wasDirty && !semanticDirty
                        && cache.baseZ == baseZ
                        && cache.waterZ == waterZ
                        && cache.slope == slope
                        && cache.sourceRotation == sourceRotation
                        && cache.surfaceStyle == surfaceStyle
                        && cache.grassVisualStage == grassVisualStage
                        && cache.waterOverlayImage == waterOverlay;
                    if (sourceStateMatches)
                    {
                        cache.lastSeen = frame;
                        if (!cache.verticalOpening
                            && !IsResidentStaticSurface(cache.ground))
                            scene.surfaces.emplace_back(cache.ground);
                        if (cache.water.has_value()
                            && !IsResidentStaticSurface(*cache.water))
                            scene.surfaces.emplace_back(*cache.water);
                        if (cache.waterOverlay.has_value()
                            && !IsResidentStaticSurface(*cache.waterOverlay))
                            scene.surfaces.emplace_back(*cache.waterOverlay);
                        continue;
                    }

                    const auto image =
                        GetFirstPersonTerrainImage(
                            *tile, origin, sourceRotation);
                    const auto* g1 =
                        image.HasValue()
                        ? GfxGetG1Element(image)
                        : nullptr;
                    if (g1 == nullptr)
                        continue;
                    const ImageId waterMask = waterZ > baseZ
                        ? GetFirstPersonWaterMaskImage(
                            *tile, sourceRotation)
                        : ImageId{};
                    const bool terrainChanged =
                        cache.source != image
                        || cache.waterMaskImage != waterMask
                        || cache.waterOverlayImage != waterOverlay
                        || cache.baseZ != baseZ
                        || cache.slope != slope
                        || cache.sourceRotation != sourceRotation
                        || cache.waterZ != waterZ
                        || cache.surfaceStyle != surfaceStyle
                        || cache.grassVisualStage != grassVisualStage
                        || cache.spriteX != g1->xOffset
                        || cache.spriteY != g1->yOffset
                        || cache.spriteWidth != g1->width
                        || cache.spriteHeight != g1->height;
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
                        cache.surfaceStyle = surfaceStyle;
                        cache.grassVisualStage = grassVisualStage;
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

        enum class FirstPersonSemanticArtworkSampleKind : uint8_t
        {
            unknown,
            transparent,
            colour,
        };

        struct FirstPersonSemanticArtworkSample
        {
            FirstPersonSemanticArtworkSampleKind kind =
                FirstPersonSemanticArtworkSampleKind::unknown;
            uint8_t pixel = 0;
        };

        [[nodiscard]] FirstPersonSemanticArtworkSample
            SampleFirstPersonSemanticArtwork(
                const FirstPersonSemanticArtworkView& view,
                const FirstPersonVec3& worldPoint,
                uint8_t sourceRotation)
        {
            if (view.component == nullptr || view.g1 == nullptr
                || view.pixels.empty())
                return {};

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
                return {};

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
                    return {};
                const uint8_t mask =
                    view.maskPixels[
                        size_t(mv) * size_t(view.maskG1->width)
                        + size_t(mu)];
                if (mask == 0)
                {
                    return {
                        FirstPersonSemanticArtworkSampleKind::transparent,
                        0
                    };
                }
            }

            const uint8_t pixel =
                view.pixels[
                    size_t(v) * size_t(view.g1->width)
                    + size_t(u)];
            if (pixel == 0)
            {
                return {
                    FirstPersonSemanticArtworkSampleKind::transparent,
                    0
                };
            }
            return {
                FirstPersonSemanticArtworkSampleKind::colour,
                pixel
            };
        }

        struct FirstPersonSemanticVisualHullTemplate
        {
            bool usable = false;
            std::vector<std::array<
                FirstPersonPaintSemanticVec3, 4>> localFaces;
        };

        [[nodiscard]] bool FirstPersonSemanticTransformIsOrthonormal(
            const FirstPersonPaintSemanticTransform& transform)
        {
            const auto dot =
                [](FirstPersonPaintSemanticVec3 a,
                   FirstPersonPaintSemanticVec3 b) {
                    return a.x * b.x + a.y * b.y + a.z * b.z;
                };
            constexpr float kTolerance = 0.01f;
            const float xx = dot(transform.axisX, transform.axisX);
            const float yy = dot(transform.axisY, transform.axisY);
            const float zz = dot(transform.axisZ, transform.axisZ);
            return std::abs(xx - 1.0f) <= kTolerance
                && std::abs(yy - 1.0f) <= kTolerance
                && std::abs(zz - 1.0f) <= kTolerance
                && std::abs(dot(transform.axisX, transform.axisY))
                    <= kTolerance
                && std::abs(dot(transform.axisX, transform.axisZ))
                    <= kTolerance
                && std::abs(dot(transform.axisY, transform.axisZ))
                    <= kTolerance;
        }

        [[nodiscard]] FirstPersonPaintSemanticVec3
            FirstPersonSemanticLocalPoint(
                const FirstPersonPaintSemanticTransform& transform,
                FirstPersonVec3 world)
        {
            const FirstPersonVec3 delta{
                world.x - transform.origin.x,
                world.y - transform.origin.y,
                world.z - transform.origin.z,
            };
            return {
                delta.x * transform.axisX.x
                    + delta.y * transform.axisX.y
                    + delta.z * transform.axisX.z,
                delta.x * transform.axisY.x
                    + delta.y * transform.axisY.y
                    + delta.z * transform.axisY.z,
                delta.x * transform.axisZ.x
                    + delta.y * transform.axisZ.y
                    + delta.z * transform.axisZ.z,
            };
        }

        [[nodiscard]] uint64_t
            FirstPersonSemanticVisualHullKey(
                const FirstPersonPaintSemanticComponent& component,
                const std::array<
                    FirstPersonSemanticArtworkView, 4>& views)
        {
            uint64_t key = 14695981039346656037ull;
            ExtendStableKey(
                key, EnumValue(component.geometry.kind));
            ExtendStableKey(key, component.geometry.pointCount);
            ExtendStableKey(key, component.geometry.localHullKey);
            for (size_t i = 0;
                 i < component.geometry.pointCount
                    && i < component.geometry.points.size();
                 ++i)
            {
                const auto& point =
                    component.geometry.points[i];
                ExtendStableKey(
                    key, uint64_t(int64_t(std::llround(
                        double(point.x) * 256.0))));
                ExtendStableKey(
                    key, uint64_t(int64_t(std::llround(
                        double(point.y) * 256.0))));
                ExtendStableKey(
                    key, uint64_t(int64_t(std::llround(
                        double(point.z) * 256.0))));
            }
            ExtendStableKey(
                key, uint64_t(int64_t(std::llround(
                    double(component.geometry.halfWidth)
                    * 256.0))));
            ExtendStableKey(
                key, uint64_t(int64_t(std::llround(
                    double(component.geometry.halfHeight)
                    * 256.0))));
            for (const auto axis : {
                     component.transform.axisX,
                     component.transform.axisY,
                     component.transform.axisZ })
            {
                ExtendStableKey(
                    key, uint64_t(int64_t(std::llround(
                        double(axis.x) * 256.0))));
                ExtendStableKey(
                    key, uint64_t(int64_t(std::llround(
                        double(axis.y) * 256.0))));
                ExtendStableKey(
                    key, uint64_t(int64_t(std::llround(
                        double(axis.z) * 256.0))));
            }
            for (uint8_t rotation = 0;
                 rotation < 4; ++rotation)
            {
                const auto& view = views[rotation];
                ExtendStableKey(
                    key,
                    view.component != nullptr
                        ? view.component->artwork.image.GetIndex()
                        : 0u);
                ExtendStableKey(
                    key,
                    view.component != nullptr
                            && view.component->artwork.mask.HasValue()
                        ? view.component->artwork.mask.GetIndex()
                        : 0u);
                ExtendStableKey(
                    key,
                    reinterpret_cast<uintptr_t>(view.g1));
                ExtendStableKey(
                    key,
                    reinterpret_cast<uintptr_t>(view.maskG1));
            }
            return key;
        }

        [[nodiscard]] std::optional<
            std::vector<std::array<FirstPersonVec3, 4>>>
            BuildFirstPersonSemanticVisualHullFaces(
                const FirstPersonPaintSemanticComponent& component,
                const std::array<
                    FirstPersonSemanticArtworkView, 4>& views)
        {
            if (component.artwork.decal
                || (component.geometry.kind
                        != FirstPersonPaintSemanticPrimitiveKind::box
                    && component.geometry.kind
                        != FirstPersonPaintSemanticPrimitiveKind::localHull)
                || component.geometry.pointCount < 2
                || !FirstPersonSemanticTransformIsOrthonormal(
                    component.transform))
                return std::nullopt;

            for (const auto& view : views)
            {
                if (view.component == nullptr
                    || view.g1 == nullptr
                    || view.pixels.empty())
                    return std::nullopt;
            }

            static std::unordered_map<
                uint64_t, FirstPersonSemanticVisualHullTemplate>
                cache;
            if (cache.size() > 4096)
                cache.clear();

            const uint64_t key =
                FirstPersonSemanticVisualHullKey(
                    component, views);
            const auto instantiate =
                [&](const FirstPersonSemanticVisualHullTemplate&
                        cached)
                    -> std::optional<
                        std::vector<std::array<
                            FirstPersonVec3, 4>>> {
                    if (!cached.usable)
                        return std::vector<
                            std::array<FirstPersonVec3, 4>>{};
                    std::vector<std::array<
                        FirstPersonVec3, 4>> result;
                    result.reserve(cached.localFaces.size());
                    for (const auto& localFace :
                         cached.localFaces)
                    {
                        std::array<FirstPersonVec3, 4>
                            face{};
                        for (size_t i = 0;
                             i < face.size(); ++i)
                        {
                            face[i] =
                                FirstPersonSemanticWorldPoint(
                                    component.transform,
                                    localFace[i]);
                        }
                        result.push_back(face);
                    }
                    return result;
                };

            if (const auto found = cache.find(key);
                found != cache.end())
                return instantiate(found->second);

            FirstPersonSemanticVisualHullTemplate
                cached{};
            const auto physicalFaces =
                BuildFirstPersonSemanticPhysicalFaces(
                    component);
            if (physicalFaces.empty())
            {
                cache.emplace(key, std::move(cached));
                return std::vector<
                    std::array<FirstPersonVec3, 4>>{};
            }

            FirstPersonVec3 low =
                physicalFaces.front().front();
            FirstPersonVec3 high = low;
            for (const auto& face : physicalFaces)
            for (const auto& point : face)
            {
                low.x = std::min(low.x, point.x);
                low.y = std::min(low.y, point.y);
                low.z = std::min(low.z, point.z);
                high.x = std::max(high.x, point.x);
                high.y = std::max(high.y, point.y);
                high.z = std::max(high.z, point.z);
            }

            float step = 1.0f;
            while (true)
            {
                const int32_t nx = int32_t(std::ceil(
                    (high.x - low.x) / step));
                const int32_t ny = int32_t(std::ceil(
                    (high.y - low.y) / step));
                const int32_t nz = int32_t(std::ceil(
                    (high.z - low.z) / step));
                const size_t count =
                    nx > 0 && ny > 0 && nz > 0
                    ? size_t(nx) * size_t(ny)
                        * size_t(nz)
                    : 0;
                if (nx > 0 && ny > 0 && nz > 0
                    && nx <= 96 && ny <= 96
                    && nz <= 96 && count <= 65536)
                    break;
                step *= 2.0f;
                if (step > 16.0f)
                {
                    cache.emplace(key, std::move(cached));
                    return std::vector<
                        std::array<FirstPersonVec3, 4>>{};
                }
            }

            std::vector<FirstPersonVisualHullView>
                hullViews;
            hullViews.reserve(4);
            for (uint8_t rotation = 0;
                 rotation < 4; ++rotation)
            {
                const auto& source = views[rotation];
                FirstPersonVisualHullView hullView{};
                hullView.imageDirection = rotation;
                hullView.image =
                    source.component->artwork.image.GetIndex();
                hullView.g1 = source.g1;
                for (int32_t y = 0;
                     y < source.g1->height; ++y)
                for (int32_t x = 0;
                     x < source.g1->width; ++x)
                {
                    if (source.pixels[
                            size_t(y)
                                * size_t(source.g1->width)
                            + size_t(x)]
                        == 0)
                        continue;
                    const int32_t screenX =
                        source.component->artwork.screenPos.x
                        + source.g1->xOffset + x;
                    const int32_t screenY =
                        source.component->artwork.screenPos.y
                        + source.g1->yOffset + y;
                    if (source.maskG1 != nullptr
                        && !source.maskPixels.empty())
                    {
                        const int32_t mx =
                            screenX
                            - source.component
                                  ->artwork.screenPos.x
                            - source.maskG1->xOffset;
                        const int32_t my =
                            screenY
                            - source.component
                                  ->artwork.screenPos.y
                            - source.maskG1->yOffset;
                        if (mx < 0 || my < 0
                            || mx >= source.maskG1->width
                            || my >= source.maskG1->height
                            || source.maskPixels[
                                size_t(my)
                                    * size_t(
                                        source.maskG1->width)
                                + size_t(mx)]
                                == 0)
                            continue;
                    }
                    hullView.observed.add(
                        screenX, screenY);
                }
                if (hullView.observed.empty())
                {
                    cache.emplace(key, std::move(cached));
                    return std::vector<
                        std::array<FirstPersonVec3, 4>>{};
                }
                hullViews.push_back(std::move(hullView));
            }

            const auto localLow =
                component.geometry.points[0];
            const auto localHigh =
                component.geometry.points[1];
            const auto occupancy =
                [&](FirstPersonVec3 world) {
                    const auto local =
                        FirstPersonSemanticLocalPoint(
                            component.transform, world);
                    constexpr float epsilon = 0.001f;
                    return local.x
                            >= std::min(
                                localLow.x, localHigh.x)
                                - epsilon
                        && local.x
                            < std::max(
                                localLow.x, localHigh.x)
                                + epsilon
                        && local.y
                            >= std::min(
                                localLow.y, localHigh.y)
                                - epsilon
                        && local.y
                            < std::max(
                                localLow.y, localHigh.y)
                                + epsilon
                        && local.z
                            >= std::min(
                                localLow.z, localHigh.z)
                                - epsilon
                        && local.z
                            < std::max(
                                localLow.z, localHigh.z)
                                + epsilon;
                };
            const auto projectPoint =
                [](uint8_t rotation,
                   FirstPersonVec3 point) {
                    const auto projected =
                        Translate3DTo2DWithZ(
                            rotation,
                            {
                                int32_t(std::lround(point.x)),
                                int32_t(std::lround(point.y)),
                                int32_t(std::lround(point.z)),
                            });
                    return std::array<float, 2>{
                        float(projected.x),
                        float(projected.y)
                    };
                };
            const auto pointSupported =
                [&](const FirstPersonVisualHullView& hullView,
                    FirstPersonVec3 point) {
                    const auto& source =
                        views[hullView.imageDirection];
                    const auto projected =
                        Translate3DTo2DWithZ(
                            hullView.imageDirection,
                            {
                                int32_t(std::lround(point.x)),
                                int32_t(std::lround(point.y)),
                                int32_t(std::lround(point.z)),
                            });
                    for (int32_t dy = -1; dy <= 1; ++dy)
                    for (int32_t dx = -1; dx <= 1; ++dx)
                    {
                        const int32_t screenX =
                            projected.x + dx;
                        const int32_t screenY =
                            projected.y + dy;
                        const int32_t u =
                            screenX
                            - source.component
                                  ->artwork.screenPos.x
                            - source.g1->xOffset;
                        const int32_t v =
                            screenY
                            - source.component
                                  ->artwork.screenPos.y
                            - source.g1->yOffset;
                        if (u < 0 || v < 0
                            || u >= source.g1->width
                            || v >= source.g1->height)
                            continue;
                        if (source.pixels[
                                size_t(v)
                                    * size_t(source.g1->width)
                                + size_t(u)]
                            == 0)
                            continue;
                        if (source.maskG1 != nullptr
                            && !source.maskPixels.empty())
                        {
                            const int32_t mu =
                                screenX
                                - source.component
                                      ->artwork.screenPos.x
                                - source.maskG1->xOffset;
                            const int32_t mv =
                                screenY
                                - source.component
                                      ->artwork.screenPos.y
                                - source.maskG1->yOffset;
                            if (mu < 0 || mv < 0
                                || mu >= source.maskG1->width
                                || mv >= source.maskG1->height
                                || source.maskPixels[
                                    size_t(mv)
                                        * size_t(
                                            source.maskG1->width)
                                    + size_t(mu)]
                                    == 0)
                                continue;
                        }
                        return true;
                    }
                    return false;
                };

            FirstPersonVisualHullConfig config{};
            config.minimumViews = 4;
            config.minimumOccupiedCells = 1;
            config.maximumOccupiedCells = 65536;
            config.maximumGridCells = 65536;
            config.maximumAxisCells = 96;
            config.minimumCandidateCoverage = 0.0f;
            config.minimumObservedCoverage = 0.0f;
            config.maximumEdgeError =
                std::numeric_limits<int32_t>::max();

            const auto hull =
                BuildFirstPersonVisualHull(
                    hullViews,
                    {
                        low.x, high.x,
                        low.y, high.y,
                        low.z, high.z,
                        step,
                    },
                    config,
                    projectPoint,
                    occupancy,
                    pointSupported);
            if (!hull.valid)
            {
                cache.emplace(key, std::move(cached));
                return std::vector<
                    std::array<FirstPersonVec3, 4>>{};
            }

            const auto carved =
                BuildFirstPersonVisualHullBoundaryFaces(
                    hull);
            if (carved.empty())
            {
                cache.emplace(key, std::move(cached));
                return std::vector<
                    std::array<FirstPersonVec3, 4>>{};
            }

            cached.usable = true;
            cached.localFaces.reserve(carved.size());
            for (const auto& face : carved)
            {
                std::array<
                    FirstPersonPaintSemanticVec3, 4>
                    local{};
                for (size_t i = 0;
                     i < local.size(); ++i)
                {
                    local[i] =
                        FirstPersonSemanticLocalPoint(
                            component.transform,
                            face.corners[i]);
                }
                cached.localFaces.push_back(local);
            }
            auto [inserted, ok] =
                cache.emplace(key, std::move(cached));
            (void)ok;
            return instantiate(inserted->second);
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
            auto faces =
                BuildFirstPersonSemanticPhysicalFaces(component);
            if (faces.empty())
                return result;

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

            if (const auto carved =
                    BuildFirstPersonSemanticVisualHullFaces(
                        component, decodedViews);
                carved.has_value() && !carved->empty())
            {
                faces = *carved;
            }

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
                        // Peers remain authoritative physical occluders. The
                        // visual hull only refines where this component shows
                        // material; it never changes collision/placement.
                        if (!coincident)
                            depthFaces.push_back(peerFace);
                    }
                }
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
                // Geometry/collision and visible material are independent.
                // Palette index zero is explicit native transparency, not a
                // hole to fill merely because this component has a 3D shape.
                surface.physicalCoverage = false;
                // A decal is explicitly appearance layered onto an existing
                // physical plane. Give it the same equal-depth paint-order
                // ownership used by floors/decks instead of making coincident
                // artwork compete in the depth buffer.
                surface.coplanarOwner =
                    component.artwork.decal
                    || component.artwork.nativePaintOrdinal != 0;
                surface.nativePaintOrdinal =
                    component.artwork.nativePaintOrdinal;

                ImageId fallbackImage{};
                for (const auto& view : decodedViews)
                {
                    if (view.component != nullptr
                        && view.component->artwork.image.HasValue())
                    {
                        fallbackImage =
                            view.component->artwork.image;
                        break;
                    }
                }
                if (!candidates.empty())
                {
                    fallbackImage =
                        decodedViews[
                            candidates.front().rotation]
                            .component->artwork.image;
                }
                if (!fallbackImage.HasValue())
                    continue;

                surface.image = fallbackImage;
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
                bool haveKnownMaterial = false;
                bool haveUnknownMaterial = false;
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

                    bool texelKnown = false;
                    bool texelUnknown = candidates.empty();
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
                        if (sample.kind
                            == FirstPersonSemanticArtworkSampleKind::unknown)
                        {
                            texelUnknown = true;
                            continue;
                        }

                        // A trustworthy transparent texel is real evidence.
                        // Do not search another view for a colour just to make
                        // the physical face look filled.
                        texelKnown = true;
                        selectedPixel =
                            sample.kind
                                == FirstPersonSemanticArtworkSampleKind::colour
                            ? sample.pixel : 0;
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
                            FirstPersonMissingMaterialPixel(x, y);
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

                // A fully known-transparent face has no visible material.
                // Its physical/collision representation remains independent.
                if (!hasPixel && haveKnownMaterial
                    && !haveUnknownMaterial)
                    continue;

                if (!hasPixel)
                    continue;

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
                    || (cached.animated
                        && variant.lastAnimationGeneration
                            != canonicalGeneration))
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
                // Persistent baked materials sample their own immutable
                // bitmap; the original ImageId remains only as palette/remap
                // metadata. Object reloads clear the whole first-person scene,
                // so keeping that unused source atlas entry as a dependency
                // would only trigger needless preloads/revision checks.
                if (surface.image.HasValue()
                    && !surface.hasImmutablePixelData())
                {
                    dependencies.insert(
                        surface.image.GetIndex());
                }
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
                auto& cached = cacheIt->second;
                for (const auto& surface :
                     cached.cameraIndependentResidentSurfaces)
                    addSurface(surface);

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
            packet.reconstructionSelectionSerial =
                _reconstructionSelectionSerial;
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
            // CollectTrackTrajectories already recorded every region belonging
            // to groups encountered by visible tiles. Reuse that exact set
            // instead of scanning the park-wide persistent trajectory cache.
            visibleRegions.insert(
                scene.activeTrackRegions.begin(),
                scene.activeTrackRegions.end());
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
                    && packet.reconstructionSelectionSerial
                        != _reconstructionSelectionSerial)
                {
                    const auto currentSelectionStamp =
                        CurrentReconstructionSelectionStamp(
                            packet.reconstructionGroups);
                    if (packet.reconstructionSelectionStamp
                        != currentSelectionStamp)
                    {
                        packet.dirty = true;
                    }
                    else
                    {
                        packet.reconstructionSelectionSerial =
                            _reconstructionSelectionSerial;
                    }
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

