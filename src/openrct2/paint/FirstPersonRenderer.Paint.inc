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
                    cached.hasCameraDependentFallback = false;

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

                bool hasTerrainEdgeSemantic = false;
                for (const auto& variant :
                     cached.rotations)
                {
                    if (!variant.semanticValid)
                        continue;
                    if (std::any_of(
                            variant.semanticComponents.begin(),
                            variant.semanticComponents.end(),
                            [](const auto& component) {
                                return component.role
                                    == FirstPersonPaintSemanticRole::
                                        terrainEdge;
                            }))
                    {
                        hasTerrainEdgeSemantic = true;
                        break;
                    }
                }
                const bool hasAnyValidRotation =
                    std::any_of(
                        cached.rotations.begin(),
                        cached.rotations.end(),
                        [](const StaticPaintRotationCache& variant) {
                            return variant.valid;
                        });
                const bool selectedTerrainEvidenceMissing =
                    hasTerrainEdgeSemantic
                    && !cached.rotations[
                            tileRotation].semanticValid;
                const bool needsTileSourceView =
                    !hasAnyValidRotation
                    || cached.animated
                    || cached.hasUngroupedResident
                    || cached.hasCameraDependentFallback
                    || selectedTerrainEvidenceMissing;
                uint8_t rotationMask =
                    needsTileSourceView
                    ? uint8_t(1u << tileRotation)
                    : 0;

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

                    bool needsGroupSourceView = true;
                    if (group.type == TileElementType::track)
                    {
                        const auto trajectory =
                            _trackTrajectoryCache.find(group.key);
                        if (trajectory != _trackTrajectoryCache.end()
                            && !trajectory->second.dirty
                            && !trajectory->second.surfaces.empty())
                        {
                            const bool hasArtwork =
                                std::any_of(
                                    trajectory->second
                                        .artworkProjections.begin(),
                                    trajectory->second
                                        .artworkProjections.end(),
                                    [](const auto& views) {
                                        return !views.empty();
                                    });
                            const bool exhausted =
                                std::all_of(
                                    trajectory->second
                                        .artworkCaptureAttempted.begin(),
                                    trajectory->second
                                        .artworkCaptureAttempted.end(),
                                    [](bool attempted) {
                                        return attempted;
                                    });
                            // Trajectory geometry is world-fixed. A missing
                            // native observation is texture evidence only, so
                            // acquire at most the currently selected view and
                            // never prewarm rotations during a camera turn.
                            needsGroupSourceView =
                                !hasArtwork
                                && !exhausted
                                && rotationMask == 0;
                        }
                    }
                    else if (
                        group.type
                            == TileElementType::largeScenery)
                    {
                        const auto geometry =
                            _largeSceneryGeometryCache.find(
                                group.key);
                        if (geometry
                                != _largeSceneryGeometryCache.end()
                            && !geometry->second.dirty
                            && !geometry->second.surfaces.empty())
                        {
                            needsGroupSourceView = false;
                        }
                    }

                    if (needsGroupSourceView)
                    {
                        rotationMask |=
                            uint8_t(1u << selected);
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
                    uint32_t nativePaintOrdinal = 1;
                    for (auto* root = session->PaintHead; root; root = root->NextQuadrantEntry)
                    {
                        const uint32_t rootPaintOrdinal =
                            nativePaintOrdinal++;
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

                        if (!dynamic
                            && root->FirstPersonSemanticRole
                                != FirstPersonPaintSemanticRole::none)
                        {
                            const auto cacheIt =
                                _staticPaintCache.find(key);
                            if (cacheIt
                                    != _staticPaintCache.end()
                                && rotation < 4)
                            {
                                auto& components =
                                    cacheIt->second
                                        .rotations[rotation]
                                        .semanticComponents;
                                for (auto& component :
                                     components)
                                {
                                    if (component.artwork
                                            .nativePaintOrdinal
                                            != 0
                                        || component.role
                                            != root
                                                ->FirstPersonSemanticRole
                                        || component.mapPosition.x
                                            != root->MapPos.x
                                        || component.mapPosition.y
                                            != root->MapPos.y
                                        || component.artwork.image
                                            != root->image_id)
                                        continue;
                                    component.artwork
                                        .nativePaintOrdinal =
                                        rootPaintOrdinal;
                                }
                            }
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
                        for (size_t i = startSurface;
                             i < scene.surfaces.size(); ++i)
                        {
                            if (scene.surfaces[i]
                                    .nativePaintOrdinal == 0)
                            {
                                scene.surfaces[i]
                                    .nativePaintOrdinal =
                                    rootPaintOrdinal;
                            }
                        }
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
                                        if (surface.viewFacing
                                            && surface.reconstructionGroup
                                                == 0)
                                        {
                                            cacheIt->second
                                                .hasCameraDependentFallback =
                                                true;
                                        }
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

            // Semantic geometry is usable as soon as the selected native view
            // publishes it. Do not synchronously repaint the same tile from the
            // other three quarter-turns before presenting the frame. Those
            // rotations are artwork evidence only and are acquired naturally
            // when camera/source-view selection actually needs them. This keeps
            // first-person entry proportional to what the player is seeing,
            // while the physical model remains camera-independent.

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
                auto& cached = cacheIt->second;

                EnsureFirstPersonSemanticSurfaceCache(cached);

                for (const auto& semanticSurface :
                     cached.semanticStreamedSurfaces)
                {
                    if (SurfaceMayBeVisible(
                            semanticSurface, worldFrustum))
                    {
                        scene.surfaces.emplace_back(
                            semanticSurface);
                    }
                }

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

