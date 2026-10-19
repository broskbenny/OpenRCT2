        // Only source-tile geometry without cross-object ownership or
        // occupancy-interface obligations may leave the coherent 32x32
        // packet. Grouped rail/building geometry and reconstructed boundaries
        // remain in that packet and retain its exact relationship resolver.
        [[nodiscard]] bool FirstPersonIndependentLocalScenery(
            const FirstPersonSurface& surface)
        {
            const auto semanticRole =
                static_cast<FirstPersonPaintSemanticRole>(surface.diagnosticSemanticRole);
            // Missing semantic classification is NOT proof of independent
            // ownership: native walls, roofs and buildings can be unclassified.
            // Only explicitly local fixtures and certified reconstructed
            // small-scenery instances leave the coherent interface packet.
            const bool locallyOwned =
                semanticRole == FirstPersonPaintSemanticRole::pathFixture
                || semanticRole == FirstPersonPaintSemanticRole::smallSceneryBody;
            return IsResidentStaticSurface(surface)
                && locallyOwned
                && surface.reconstructionGroup == 0
                && !surface.reconstructedOccupancyBoundary
                && !surface.coplanarOwner
                && !surface.artworkCarrier
                && !surface.exteriorOnly;
        }

        [[nodiscard]] bool FirstPersonResidentContainsCoherentGeometry(
            const std::vector<FirstPersonSurface>& surfaces)
        {
            return std::any_of(surfaces.begin(), surfaces.end(),
                [](const auto& surface) {
                    return IsResidentStaticSurface(surface)
                        && !FirstPersonIndependentLocalScenery(surface);
                });
        }

        // A native source-view change only affects surfaces selected from
        // cached rotations. Keep a 32x32 relationship rebuild when any
        // rotated resident needs coherent ownership; otherwise the local
        // 8x8 cell is the only changed GPU residency unit.
        [[nodiscard]] bool FirstPersonRotationHasCoherentTileGeometry(
            const StaticPaintCacheEntry& cached)
        {
            for (const auto& variant : cached.rotations)
            {
                if (!variant.valid)
                    continue;
                for (const auto& surface : variant.residentSurfaces)
                {
                    if (IsResidentStaticSurface(surface)
                        && surface.reconstructionGroup == 0
                        && !FirstPersonIndependentLocalScenery(surface))
                        return true;
                }
            }
            return false;
        }

        struct FirstPersonLocalSceneryPacket
        {
            std::shared_ptr<const std::vector<FirstPersonSurface>> surfaces;
            std::vector<ImageIndex> textureDependencies;
            FirstPersonVec3 center{};
            float radius = 0.0f;
            uint64_t generation = 0;
            uint64_t fingerprint = 0;
            size_t vertexCount = 0;
            bool containsNativeSource = false;
            uint32_t viewFlags = 0;
            bool trackDesignSaveMode = false;
            // Inspector selection changes appearance for one map placement.
            uint64_t selectionGeneration = 0;
            bool containsInspectedElement = false;
            bool dirty = true;
        };
        static std::unordered_map<uint64_t, FirstPersonLocalSceneryPacket>
            _localSceneryPackets;
        static uint64_t _localSceneryEpoch = 0;

        void MarkLocalSceneryDirtyForTile(int32_t tx, int32_t ty)
        {
            _localSceneryPackets[FirstPersonLocalSceneryRegionKey(tx, ty)].dirty = true;
        }

        // A placement key also includes authoritative object identity and
        // elevation. Reusing the same tile-element slot for another asset
        // must never inherit the previous object's published mesh.
        [[nodiscard]] uint64_t FirstPersonLocalSmallSceneryInstanceKey(
            int32_t tx, int32_t ty, uint32_t index,
            const SmallSceneryElement& small)
        {
            uint64_t key = 14695981039346656037ull;
            ExtendStableKey(key, uint32_t(tx));
            ExtendStableKey(key, uint32_t(ty));
            ExtendStableKey(key, index);
            ExtendStableKey(key, uint32_t(small.getEntryIndex()));
            ExtendStableKey(key, uint32_t(small.getBaseZ()));
            return key == 0 ? 1 : key;
        }

        void SubmitVisibleLocalSceneryRegions(
            FirstPersonScene& scene, const FirstPersonFrustum& frustum)
        {
            if (_localSceneryEpoch != _sceneEpoch)
            {
                _localSceneryPackets.clear();
                _localSceneryEpoch = _sceneEpoch;
            }
            // Selection is not part of TileElement bytes or viewport flags.
            // Visit each cell only once per selection generation to find the
            // affected object; don't rebuild the entire visible world on click.
            static const TileElement* lastInspected = nullptr;
            static uint64_t selectionGeneration = 1;
            const TileElement* inspected = TileInspector::GetSelectedElement();
            if (inspected != lastInspected)
            {
                lastInspected = inspected;
                ++selectionGeneration;
                if (selectionGeneration == 0)
                    ++selectionGeneration;
            }
            std::unordered_set<uint64_t> visible;
            visible.reserve(scene.visibleTiles.size() / 32 + 1);
            for (const auto tile : scene.visibleTiles)
                visible.insert(FirstPersonLocalSceneryRegionKey(
                    tile.x / kCoordsXYStep, tile.y / kCoordsXYStep));

            // A completed cell survives a camera turn even when source-tile
            // admission omitted its original seed tile on this frame.
            for (const auto& [key, packet] : _localSceneryPackets)
            {
                if (packet.surfaces && !packet.surfaces->empty()
                    && frustum.visible(packet.center, packet.radius))
                    visible.insert(key);
            }

            std::vector<std::pair<float, uint64_t>> rebuild;
            rebuild.reserve(visible.size());
            for (const auto key : visible)
            {
                auto& packet = _localSceneryPackets[key];
                if (packet.selectionGeneration != selectionGeneration)
                {
                    bool contains = false;
                    if (inspected != nullptr)
                    {
                        const int32_t rx =
                            int32_t(uint32_t((key >> 32) & 0x3fffffffu)) - 1;
                        const int32_t ry = int32_t(uint32_t(key)) - 1;
                        for (int32_t ty = ry * 8; ty < (ry + 1) * 8 && !contains; ++ty)
                        for (int32_t tx = rx * 8; tx < (rx + 1) * 8 && !contains; ++tx)
                        {
                            const CoordsXY tile{tx * kCoordsXYStep, ty * kCoordsXYStep};
                            if (!MapIsLocationValid(tile))
                                continue;
                            auto* element = MapGetFirstElementAt(tile);
                            if (element == nullptr)
                                continue;
                            do
                            {
                                if (element == inspected)
                                {
                                    contains = true;
                                    break;
                                }
                            } while (!(element++)->isLastForTile());
                        }
                    }
                    if (packet.containsInspectedElement != contains)
                        packet.dirty = true;
                    packet.containsInspectedElement = contains;
                    packet.selectionGeneration = selectionGeneration;
                }
                // Placement ownership depends on presentation flags as well
                // as map identity. Existing GPU cells must not outlive a
                // transition to ghost/highlight/track-design native artwork.
                if (packet.surfaces
                    && (packet.viewFlags != scene.options.viewFlags
                        || packet.trackDesignSaveMode != gTrackDesignSaveMode))
                    packet.dirty = true;
                if (!packet.dirty)
                    continue;
                const int32_t rx = int32_t(uint32_t((key >> 32) & 0x3fffffffu)) - 1;
                const int32_t ry = int32_t(uint32_t(key)) - 1;
                const float dx = float(rx * 8 * kCoordsXYStep + 4 * kCoordsXYStep) - frustum.eye.x;
                const float dy = float(ry * 8 * kCoordsXYStep + 4 * kCoordsXYStep) - frustum.eye.y;
                rebuild.emplace_back(dx * dx + dy * dy, key);
            }
            std::sort(rebuild.begin(), rebuild.end());
            FirstPersonFrameBudget budget(4, std::chrono::microseconds(1200));
            for (const auto& [distance, key] : rebuild)
            {
                (void)distance;
                if (!budget.take())
                    break;
                const int32_t rx = int32_t(uint32_t((key >> 32) & 0x3fffffffu)) - 1;
                const int32_t ry = int32_t(uint32_t(key)) - 1;
                std::vector<FirstPersonSurface> surfaces;
                surfaces.reserve(64);
                std::vector<ImageIndex> dependencies;
                FirstPersonVec3 low{}, high{};
                bool haveBounds = false;
                bool waitingForSourceRevalidation = false;
                bool containsNativeSource = false;

                const auto add = [&](const FirstPersonSurface& source,
                                     uint64_t owningRegion, bool mapOwned = false) {
                    if (!IsResidentStaticSurface(source)
                        || source.gpuRegion != owningRegion
                        || (!mapOwned && !FirstPersonIndependentLocalScenery(source)))
                        return;
                    if (!mapOwned)
                        containsNativeSource = true;
                    auto surface = source;
                    surface.gpuRegion = key;
                    if (surface.image.HasValue()
                        && (!surface.hasImmutablePixelData()
                            || surface.physicalCoverage))
                        dependencies.push_back(surface.image.GetIndex());
                    if (surface.mask.HasValue())
                        dependencies.push_back(surface.mask.GetIndex());
                    FirstPersonVec3 a{}, b{};
                    if (surface.hasSemanticBounds)
                    {
                        const auto& centre = surface.semanticCenter;
                        const float radius = surface.semanticRadius;
                        a = {centre.x - radius, centre.y - radius, centre.z - radius};
                        b = {centre.x + radius, centre.y + radius, centre.z + radius};
                    }
                    else
                    {
                        a = b = surface.triangles[0].world;
                        for (const auto& vertex : surface.triangles)
                        {
                            const auto& p = vertex.world;
                            a.x = std::min(a.x, p.x); a.y = std::min(a.y, p.y);
                            a.z = std::min(a.z, p.z);
                            b.x = std::max(b.x, p.x); b.y = std::max(b.y, p.y);
                            b.z = std::max(b.z, p.z);
                        }
                    }
                    if (!haveBounds)
                    {
                        low = a; high = b; haveBounds = true;
                    }
                    else
                    {
                        low.x = std::min(low.x, a.x); low.y = std::min(low.y, a.y);
                        low.z = std::min(low.z, a.z);
                        high.x = std::max(high.x, b.x); high.y = std::max(high.y, b.y);
                        high.z = std::max(high.z, b.z);
                    }
                    surfaces.emplace_back(std::move(surface));
                };

                std::vector<FirstPersonSurface> mapSurfaces;
                const auto previousCell = _localSceneryPackets.find(key);
                for (int32_t ty = ry * 8; ty < (ry + 1) * 8; ++ty)
                for (int32_t tx = rx * 8; tx < (rx + 1) * 8; ++tx)
                {
                    // Instantiate ready assets from the authoritative map.
                    const CoordsXY mapTile{tx * kCoordsXYStep, ty * kCoordsXYStep};
                    const uint64_t owningRegion = FirstPersonGpuRegionKey(tx, ty);
                    if (MapIsLocationValid(mapTile))
                    {
                        auto* element = MapGetFirstElementAt(mapTile);
                        uint32_t elementIndex = 0;
                        if (element != nullptr)
                        do
                        {
                            if (element->getType() == TileElementType::smallScenery)
                            {
                                const auto* small = element->asSmallScenery();
                                if (small != nullptr)
                                {
                                    mapSurfaces.clear();
                                    const uint64_t instance =
                                        FirstPersonLocalSmallSceneryInstanceKey(
                                            tx, ty, elementIndex, *small);
                                    if (AppendFirstPersonMapSmallScenery(
                                            mapSurfaces, mapTile, *small,
                                            scene.options.viewFlags))
                                    {
                                        const auto target = BuildFirstPersonTileInteractionTarget(
                                            ViewportInteractionItem::scenery, mapTile, element);
                                        for (auto& surface : mapSurfaces)
                                        {
                                            surface.interactionTarget = target;
                                            surface.sourceInstanceKey = instance;
                                            add(surface, owningRegion, true);
                                        }
                                    }
                                    else if (FirstPersonMapSmallSceneryPending(
                                            *small, scene.options.viewFlags)
                                        && previousCell != _localSceneryPackets.end()
                                        && previousCell->second.surfaces)
                                    {
                                        // Carry only this exact object's prior
                                        // appearance while its new asset is
                                        // awaiting reconstruction. An edited
                                        // object ID, deleted placement or
                                        // altered elevation never matches.
                                        for (const auto& old :
                                             *previousCell->second.surfaces)
                                        {
                                            if (old.sourceInstanceKey != instance
                                                || old.diagnosticSemanticRole
                                                    != static_cast<uint8_t>(
                                                        FirstPersonPaintSemanticRole::smallSceneryBody))
                                                continue;
                                            auto carried = old;
                                            carried.gpuRegion = owningRegion;
                                            add(carried, owningRegion, true);
                                        }
                                    }
                                }
                            }
                            ++elementIndex;
                        } while (!(element++)->isLastForTile());
                    }
                    const auto it = _staticPaintCache.find(TerrainKey(tx, ty));
                    if (it == _staticPaintCache.end())
                        continue;
                    if (it->second.dirty)
                    {
                        waitingForSourceRevalidation = true;
                        continue;
                    }
                    if (!it->second.valid)
                        continue;
                    const auto& cached = it->second;
                    for (const auto& source : cached.semanticResidentSurfaces)
                        add(source, owningRegion);
                    if (cached.animated)
                        continue;
                    for (const auto& source : cached.cameraIndependentResidentSurfaces)
                        add(source, owningRegion);
                    const uint8_t rotation = cached.selectedRotation;
                    if (rotation < cached.rotations.size()
                        && cached.rotations[rotation].valid)
                        for (const auto& source :
                             cached.rotations[rotation].residentSurfaces)
                            add(source, owningRegion);
                }
                // Preserve warm native-owned faces during source revalidation.
                const auto current = _localSceneryPackets.find(key);
                if (waitingForSourceRevalidation
                    && current != _localSceneryPackets.end()
                    && current->second.surfaces
                    && !current->second.surfaces->empty()
                    && current->second.containsNativeSource)
                    continue;
                std::sort(dependencies.begin(), dependencies.end());
                dependencies.erase(
                    std::unique(dependencies.begin(), dependencies.end()),
                    dependencies.end());
                auto& packet = _localSceneryPackets[key];
                packet.viewFlags = scene.options.viewFlags;
                packet.trackDesignSaveMode = gTrackDesignSaveMode;
                const uint64_t fingerprint =
                    ResidentStaticSurfaceFingerprint(surfaces);
                if (packet.surfaces
                    && packet.fingerprint == fingerprint
                    && packet.textureDependencies == dependencies)
                {
                    // Identical GPU triangles do not imply identical picking
                    // provenance. Refresh immutable CPU metadata for selection
                    // and tile-edit validation without uploading the VBO again.
                    packet.surfaces =
                        std::make_shared<const std::vector<FirstPersonSurface>>(
                            std::move(surfaces));
                    packet.containsNativeSource = containsNativeSource;
                    packet.dirty = false;
                    continue;
                }
                packet.fingerprint = fingerprint;
                packet.containsNativeSource = containsNativeSource;
                packet.vertexCount = surfaces.size() * 6;
                packet.surfaces =
                    std::make_shared<const std::vector<FirstPersonSurface>>(
                        std::move(surfaces));
                packet.textureDependencies = std::move(dependencies);
                if (haveBounds)
                {
                    packet.center = {
                        (low.x + high.x) * 0.5f,
                        (low.y + high.y) * 0.5f,
                        (low.z + high.z) * 0.5f
                    };
                    const float dx = (high.x - low.x) * 0.5f;
                    const float dy = (high.y - low.y) * 0.5f;
                    const float dz = (high.z - low.z) * 0.5f;
                    packet.radius = std::sqrt(dx * dx + dy * dy + dz * dz) + 2.0f;
                }
                else
                {
                    packet.radius = 0.0f;
                }
                ++packet.generation;
                packet.dirty = false;
            }

            for (const auto key : visible)
            {
                const auto it = _localSceneryPackets.find(key);
                if (it == _localSceneryPackets.end())
                    continue;
                const auto& packet = it->second;
                if (!packet.surfaces || packet.surfaces->empty()
                    || !frustum.visible(packet.center, packet.radius))
                    continue;
                scene.staticRegions.push_back({
                    key, _sceneEpoch, packet.generation,
                    packet.center, packet.radius,
                    packet.surfaces.get(), &packet.textureDependencies,
                    packet.surfaces, packet.generation, packet.vertexCount
                });
            }
        }

