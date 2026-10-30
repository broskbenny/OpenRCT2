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

        // Defined by the authoritative region builder after this include.
        // Ownership handoff must invalidate both packet and in-flight worker.
        void MarkFirstPersonAuthoritativeRegionEdit(uint64_t regionKey);

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
            // Last published authoritative tile bytes. Prevent a stale
            // component from surviving deletion or tile-slot reuse.
            std::array<uint64_t, 64> sourceSignatures{};
            std::unordered_set<uint64_t> mapOwnedInstanceKeys;
            std::unordered_set<uint64_t> mappedPathTiles;
            std::unordered_set<uint64_t> mappedWallTiles;
            uint32_t viewFlags = 0;
            bool trackDesignSaveMode = false;
            bool paintBlockedMode = false;
            bool paintWideGhostMode = false;
            // Inspector selection changes appearance for one map placement.
            uint64_t selectionGeneration = 0;
            bool containsInspectedElement = false;
            bool dirty = true;
            uint64_t invalidationSerial = 0;
        };
        static std::unordered_map<uint64_t, FirstPersonLocalSceneryPacket>
            _localSceneryPackets;
        struct FirstPersonLocalSceneryStaging
        {
            bool initialized = false;
            uint64_t epoch = 0;
            uint64_t selectionGeneration = 0;
            uint64_t sourceInvalidationSerial = 0;
            uint64_t lastVisibleFrame = 0;
            uint32_t viewFlags = 0;
            bool trackDesignSaveMode = false;
            bool paintBlockedMode = false;
            bool paintWideGhostMode = false;
            size_t nextTile = 0;
            FirstPersonPacketTileSurfaceCursor tileCursor{};
            std::array<uint64_t, 64> sourceSignatures{};
            std::vector<FirstPersonSurface> surfaces;
            std::vector<ImageIndex> dependencies;
            FirstPersonVec3 low{}, high{};
            bool haveBounds = false;
            bool waitingForSourceRevalidation = false;
            bool containsNativeSource = false;
            std::unordered_set<uint64_t> mapOwnedNativeKeys;
            std::unordered_set<uint64_t> mapOwnedInstanceKeys;
            std::unordered_set<uint64_t> mappedPathTiles;
            std::unordered_set<uint64_t> mappedWallTiles;
        };
        static std::unordered_map<uint64_t, FirstPersonLocalSceneryStaging>
            _localSceneryBuilds;
        static uint64_t _localSceneryEpoch = 0;

        [[nodiscard]] bool FirstPersonLocalFlatPathReady(CoordsXY tile)
        {
            const auto key = FirstPersonLocalSceneryRegionKey(
                tile.x / kCoordsXYStep, tile.y / kCoordsXYStep);
            const auto it = _localSceneryPackets.find(key);
            if (it == _localSceneryPackets.end() || it->second.dirty
                || !it->second.surfaces)
                return false;
            return it->second.mappedPathTiles.contains(
                TerrainKey(tile.x / kCoordsXYStep,
                    tile.y / kCoordsXYStep));
        }

        [[nodiscard]] bool FirstPersonLocalPlainWallReady(CoordsXY tile)
        {
            const auto key = FirstPersonLocalSceneryRegionKey(
                tile.x / kCoordsXYStep, tile.y / kCoordsXYStep);
            const auto it = _localSceneryPackets.find(key);
            return it != _localSceneryPackets.end()
                && !it->second.dirty && it->second.surfaces
                && it->second.mappedWallTiles.contains(TerrainKey(
                    tile.x / kCoordsXYStep, tile.y / kCoordsXYStep));
        }


        [[nodiscard]] bool FirstPersonTileObjectsMapOwnedAndReady(
            CoordsXY tile)
        {
            auto* element = MapGetFirstElementAt(tile);
            if (element == nullptr)
                return false;
            bool hasObject = false;
            uint32_t elementIndex = 0;
            const auto packet = _localSceneryPackets.find(
                FirstPersonLocalSceneryRegionKey(
                    tile.x / kCoordsXYStep, tile.y / kCoordsXYStep));
            do
            {
                switch (element->getType())
                {
                    case TileElementType::surface:
                        break;
                    case TileElementType::path:
                    {
                        const auto* path = element->asPath();
                        // The map-owned deck is NOT its bins, benches,
                        // lamps or other path additions. These still need
                        // native source acquisition at cold-object priority.
                        if (path == nullptr || path->hasAddition()
                            || !FirstPersonLocalFlatPathReady(tile))
                            return false;
                        hasObject = true;
                        break;
                    }
                    case TileElementType::wall:
                        if (!FirstPersonLocalPlainWallReady(tile))
                            return false;
                        hasObject = true;
                        break;
                    case TileElementType::track:
                    case TileElementType::largeScenery:
                    case TileElementType::entrance:
                    {
                        // A completed physical group already supplies 3D
                        // occupancy independent of native paint. Do not
                        // spend scarce cold-source slots on its ready tiles
                        // while native-only objects remain unseen. Ordinary
                        // source sweeps still acquire missing artwork.
                        const auto group = GetReconstructionGroup(tile, element);
                        if (!group.has_value())
                            return false;
                        if (element->getType() == TileElementType::track)
                        {
                            const auto found = _trackTrajectoryCache.find(group->key);
                            if (found == _trackTrajectoryCache.end()
                                || found->second.dirty
                                || found->second.surfaces.empty())
                                return false;
                        }
                        else if (element->getType() == TileElementType::largeScenery)
                        {
                            const auto found = _largeSceneryGeometryCache.find(group->key);
                            if (found == _largeSceneryGeometryCache.end()
                                || found->second.dirty
                                || found->second.surfaces.empty())
                                return false;
                        }
                        else
                        {
                            const auto found = _parkEntranceGeometryCache.find(group->key);
                            if (found == _parkEntranceGeometryCache.end()
                                || found->second.dirty
                                || found->second.surfaces.empty())
                                return false;
                        }
                        hasObject = true;
                        break;
                    }
                    case TileElementType::smallScenery:
                    {
                        const auto* small = element->asSmallScenery();
                        const auto* entry = small != nullptr
                            ? small->getEntry() : nullptr;
                        // The native truss remains necessary even when the
                        // static scenery body already has a map-owned mesh.
                        if (small == nullptr
                            || (entry != nullptr && small->needsSupports()
                                && !entry->flags.has(SmallSceneryFlag::hasNoSupports))
                            || packet == _localSceneryPackets.end()
                            || packet->second.dirty
                            || !packet->second.surfaces
                            || !packet->second.mapOwnedInstanceKeys.contains(
                                FirstPersonLocalSmallSceneryInstanceKey(
                                    tile.x / kCoordsXYStep,
                                    tile.y / kCoordsXYStep,
                                    elementIndex, *small)))
                            return false;
                        hasObject = true;
                        break;
                    }
                    default:
                        return false;
                }
                ++elementIndex;
            } while (!(element++)->isLastForTile());
            // Native captures are still admitted through the independent
            // periodic sweep for terrain edges and presentation changes.
            return hasObject;
        }

        void MarkLocalSceneryDirtyForTile(int32_t tx, int32_t ty)
        {
            auto& packet = _localSceneryPackets[FirstPersonLocalSceneryRegionKey(tx, ty)];
            packet.dirty = true;
            ++packet.invalidationSerial;
        }

        // Map ownership of a wall depends on the opposite edge in the
        // neighbouring tile. Only edits crossing an 8x8-cell boundary need
        // additional local packet invalidation: edits inside one cell are
        // already covered by rebuilding its whole authoritative tile map.
        void MarkLocalSceneryRelationshipDirtyForTile(int32_t tx, int32_t ty)
        {
            MarkLocalSceneryDirtyForTile(tx, ty);
            constexpr int32_t kCellMask = 7;
            if ((tx & kCellMask) == 0 && tx > 0)
                MarkLocalSceneryDirtyForTile(tx - 1, ty);
            if ((tx & kCellMask) == kCellMask)
                MarkLocalSceneryDirtyForTile(tx + 1, ty);
            if ((ty & kCellMask) == 0 && ty > 0)
                MarkLocalSceneryDirtyForTile(tx, ty - 1);
            if ((ty & kCellMask) == kCellMask)
                MarkLocalSceneryDirtyForTile(tx, ty + 1);
        }

        // A placement key also includes authoritative object identity and
        // elevation. Reusing the same tile-element slot for another asset
        // must never inherit the previous object's published mesh.
        // An asset worker becoming ready is not a GPU publication.
        // Native fallback is retired only after the exact authoritative
        // instance has entered an immutable local-cell packet. A changed
        // placement cannot inherit another instance's completed geometry.
        [[nodiscard]] bool FirstPersonLocalSceneryOwnsSmallScenery(
            CoordsXY tile, const SmallSceneryElement& small)
        {
            const auto cell = _localSceneryPackets.find(
                FirstPersonLocalSceneryRegionKey(
                    tile.x / kCoordsXYStep, tile.y / kCoordsXYStep));
            if (cell == _localSceneryPackets.end() || !cell->second.surfaces)
                return false;
            auto* element = MapGetFirstElementAt(tile);
            if (element == nullptr)
                return false;
            const auto* selected =
                reinterpret_cast<const TileElement*>(&small);
            uint32_t index = 0;
            do
            {
                if (element == selected)
                    return cell->second.mapOwnedInstanceKeys.contains(
                        FirstPersonLocalSmallSceneryInstanceKey(
                            tile.x / kCoordsXYStep, tile.y / kCoordsXYStep,
                            index, small));
                ++index;
            } while (!(element++)->isLastForTile());
            return false;
        }

        // Native instance keys identify the tile, element index and type.
        // Only its body may be retired after exact map-owned GPU publication;
        // elevated wooden supports are still painted by the native path.
        [[nodiscard]] bool FirstPersonNativeInstanceMapOwned(
            CoordsXY tile, FirstPersonNativeInstanceRecord record)
        {
            const auto role = static_cast<FirstPersonPaintSemanticRole>(record.role);
            if (role == FirstPersonPaintSemanticRole::pathDeck)
                return FirstPersonLocalFlatPathReady(tile);
            if (role == FirstPersonPaintSemanticRole::wall)
                return FirstPersonLocalPlainWallReady(tile);
            if (role != FirstPersonPaintSemanticRole::smallSceneryBody)
                return false;
            auto* element = MapGetFirstElementAt(tile);
            uint32_t index = 0;
            while (element != nullptr)
            {
                if (element->getType() == TileElementType::smallScenery)
                {
                    uint64_t nativeKey = 14695981039346656037ull;
                    ExtendStableKey(nativeKey, uint32_t(tile.x / kCoordsXYStep));
                    ExtendStableKey(nativeKey, uint32_t(tile.y / kCoordsXYStep));
                    ExtendStableKey(nativeKey, index);
                    ExtendStableKey(nativeKey, uint8_t(TileElementType::smallScenery));
                    if ((nativeKey == 0 ? 1 : nativeKey) == record.instance)
                    {
                        const auto* small = element->asSmallScenery();
                        return small != nullptr
                            && FirstPersonLocalSceneryOwnsSmallScenery(tile, *small);
                    }
                }
                if (element->isLastForTile())
                    break;
                ++element;
                ++index;
            }
            return false;
        }

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
            // Last-known-good ownership is legal only for the exact
            // authoritative pose and appearance. Reusing a tile-element
            // slot after rotation, recolouring or withering cannot inherit
            // the preceding mesh while another asset job is pending.
            ExtendStableKey(key, uint32_t(small.getDirection()));
            ExtendStableKey(key, uint32_t(small.getSceneryQuadrant()));
            ExtendStableKey(key, uint32_t(small.getOccupiedQuadrants()));
            if (const auto* entry = small.getEntry(); entry != nullptr)
                ExtendStableKey(key,
                    FirstPersonSmallSceneryWitherStage(*entry, small));
            ExtendStableKey(key, uint32_t(small.getPrimaryColour()));
            ExtendStableKey(key, uint32_t(small.getSecondaryColour()));
            ExtendStableKey(key, uint32_t(small.getTertiaryColour()));
            return key == 0 ? 1 : key;
        }

        void SubmitVisibleLocalSceneryRegions(
            FirstPersonScene& scene, const FirstPersonFrustum& frustum)
        {
            if (_localSceneryEpoch != _sceneEpoch)
            {
                _localSceneryPackets.clear();
                _localSceneryBuilds.clear();
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
                        || packet.trackDesignSaveMode != gTrackDesignSaveMode
                        || packet.paintBlockedMode != gPaintBlockedTiles
                        || packet.paintWideGhostMode != gPaintWidePathsAsGhost))
                    packet.dirty = true;
                if (!packet.dirty)
                    continue;
                const int32_t rx = int32_t(uint32_t((key >> 32) & 0x3fffffffu)) - 1;
                const int32_t ry = int32_t(uint32_t(key)) - 1;
                const float dx = float(rx * 8 * kCoordsXYStep + 4 * kCoordsXYStep) - frustum.eye.x;
                const float dy = float(ry * 8 * kCoordsXYStep + 4 * kCoordsXYStep) - frustum.eye.y;
                rebuild.emplace_back(dx * dx + dy * dy, key);
            }
            // A camera turn must not keep an unbounded number of partial
            // 8x8 snapshots alive. Retain a few recent ones for return turns.
            constexpr uint64_t kLocalBuildGrace = 120;
            constexpr size_t kPausedLocalBuildLimit = 8;
            std::vector<std::pair<uint64_t, uint64_t>> pausedLocalBuilds;
            for (auto it = _localSceneryBuilds.begin();
                 it != _localSceneryBuilds.end();)
            {
                if (visible.contains(it->first))
                {
                    it->second.lastVisibleFrame = _terrainCache.frame;
                    ++it;
                }
                else if (_terrainCache.frame - it->second.lastVisibleFrame
                         > kLocalBuildGrace)
                    it = _localSceneryBuilds.erase(it);
                else
                {
                    pausedLocalBuilds.emplace_back(
                        it->second.lastVisibleFrame, it->first);
                    ++it;
                }
            }
            std::sort(pausedLocalBuilds.begin(), pausedLocalBuilds.end(),
                [](const auto& x, const auto& y) {
                    return x.first > y.first
                        || (x.first == y.first && x.second < y.second);
                });
            for (size_t i = kPausedLocalBuildLimit;
                 i < pausedLocalBuilds.size(); ++i)
                _localSceneryBuilds.erase(pausedLocalBuilds[i].second);
            std::sort(rebuild.begin(), rebuild.end(),
                [&](const auto& a, const auto& b) {
                    const bool aStaged = _localSceneryBuilds.contains(a.second);
                    const bool bStaged = _localSceneryBuilds.contains(b.second);
                    if (aStaged != bStaged)
                        return aStaged;
                    return a < b;
                });
            FirstPersonFrameBudget budget(4, std::chrono::microseconds(1200));
            FirstPersonFrameBudget tileBudget(8192, std::chrono::microseconds(1200));
            for (const auto& [distance, key] : rebuild)
            {
                (void)distance;
                if (!budget.take())
                    break;
                const int32_t rx = int32_t(uint32_t((key >> 32) & 0x3fffffffu)) - 1;
                const int32_t ry = int32_t(uint32_t(key)) - 1;
                auto& oldPacket = _localSceneryPackets[key];
                auto& staging = _localSceneryBuilds[key];
                if (!staging.initialized || staging.epoch != _sceneEpoch
                    || staging.selectionGeneration != oldPacket.selectionGeneration
                    || staging.viewFlags != scene.options.viewFlags
                    || staging.trackDesignSaveMode != gTrackDesignSaveMode
                    || staging.paintBlockedMode != gPaintBlockedTiles
                    || staging.paintWideGhostMode != gPaintWidePathsAsGhost)
                {
                    staging = {};
                    staging.initialized = true;
                    staging.epoch = _sceneEpoch;
                    staging.selectionGeneration = oldPacket.selectionGeneration;
                    staging.sourceInvalidationSerial = oldPacket.invalidationSerial;
                    staging.viewFlags = scene.options.viewFlags;
                    staging.trackDesignSaveMode = gTrackDesignSaveMode;
                    staging.paintBlockedMode = gPaintBlockedTiles;
                    staging.paintWideGhostMode = gPaintWidePathsAsGhost;
                    staging.surfaces.reserve(64);
                }
                staging.lastVisibleFrame = _terrainCache.frame;
                auto& surfaces = staging.surfaces;
                auto& dependencies = staging.dependencies;
                auto& low = staging.low;
                auto& high = staging.high;
                auto& haveBounds = staging.haveBounds;
                auto& waitingForSourceRevalidation = staging.waitingForSourceRevalidation;
                auto& containsNativeSource = staging.containsNativeSource;
                auto& mapOwnedNativeKeys = staging.mapOwnedNativeKeys;
                auto& mapOwnedInstanceKeys = staging.mapOwnedInstanceKeys;
                auto& mappedPathTiles = staging.mappedPathTiles;
                auto& mappedWallTiles = staging.mappedWallTiles;

                const auto add = [&](const FirstPersonSurface& source,
                                     uint64_t owningRegion, bool mapOwned = false) {
                    if (!IsResidentStaticSurface(source)
                        || source.gpuRegion != owningRegion
                        || (!mapOwned && !FirstPersonIndependentLocalScenery(source))
                        || (!mapOwned && mapOwnedNativeKeys.contains(source.sourceInstanceKey)
                            && source.diagnosticSemanticRole == static_cast<uint8_t>(
                                FirstPersonPaintSemanticRole::smallSceneryBody)))
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
                while (staging.nextTile < 64 && tileBudget.take())
                {
                    const size_t cellIndex = staging.nextTile;
                    const int32_t tx = rx * 8 + int32_t(cellIndex % 8);
                    const int32_t ty = ry * 8 + int32_t(cellIndex / 8);
                    // Capture exact map provenance: source edits during
                    // staging must never publish a hybrid 8x8 tile snapshot.
                    const CoordsXY mapTile{tx * kCoordsXYStep, ty * kCoordsXYStep};
                    const bool validMapTile = MapIsLocationValid(mapTile);
                    const uint64_t owningRegion = FirstPersonGpuRegionKey(tx, ty);
                    if (staging.tileCursor.phase == 0)
                    {
                        staging.sourceSignatures[cellIndex] =
                            validMapTile ? NativeTileSignature(mapTile) : 0;
                    if (validMapTile)
                    {
                        auto* element = MapGetFirstElementAt(mapTile);
                        uint32_t elementIndex = 0;
                        if (element != nullptr)
                        {
                            mapSurfaces.clear();
                            if (AppendFirstPersonMapPlainWallTile(
                                    mapSurfaces, mapTile,
                                    scene.options.viewFlags))
                            {
                                mappedWallTiles.insert(TerrainKey(tx, ty));
                                for (auto& surface : mapSurfaces)
                                    add(surface, owningRegion, true);
                            }
                        }
                        if (element != nullptr)
                        do
                        {
                            if (element->getType() == TileElementType::path
                                && FirstPersonTileHasMapOwnedFlatPathDeck(
                                    mapTile, scene.options.viewFlags))
                            {
                                const auto* path = element->asPath();
                                mapSurfaces.clear();
                                if (path != nullptr
                                    && AppendFirstPersonMapFlatPathDeck(
                                        mapSurfaces, mapTile, *path,
                                        scene.options.viewFlags))
                                {
                                    mappedPathTiles.insert(TerrainKey(tx, ty));
                                    for (auto& surface : mapSurfaces)
                                        add(surface, owningRegion, true);
                                }
                            }
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
                                        mapOwnedInstanceKeys.insert(instance);
                                        // A previously native-owned cache view
                                        // may survive an inspector selection
                                        // transition. A completed map instance
                                        // supersedes that exact native element,
                                        // otherwise it would render twice.
                                        uint64_t nativeKey = 14695981039346656037ull;
                                        ExtendStableKey(nativeKey, uint32_t(tx));
                                        ExtendStableKey(nativeKey, uint32_t(ty));
                                        ExtendStableKey(nativeKey, elementIndex);
                                        ExtendStableKey(nativeKey,
                                            uint8_t(TileElementType::smallScenery));
                                        mapOwnedNativeKeys.insert(nativeKey == 0 ? 1 : nativeKey);
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
                        staging.tileCursor.nextPhase();
                    }
                    const auto it = _staticPaintCache.find(TerrainKey(tx, ty));
                    if (it == _staticPaintCache.end()
                        || !it->second.valid || it->second.dirty)
                    {
                        if (it != _staticPaintCache.end() && it->second.dirty)
                            waitingForSourceRevalidation = true;
                        staging.tileCursor.nextTile();
                        ++staging.nextTile;
                        continue;
                    }
                    const auto& cached = it->second;
                    // Native source rotation is selected per exact tile
                    // element, not once for every object in this cell.
                    std::unordered_map<FirstPersonNativeInstanceRecord, uint8_t,
                        FirstPersonNativeInstanceRecordHash> selectedNativeInstances;
                    while (staging.tileCursor.phase <= 6)
                    {
                        const uint8_t phase = staging.tileCursor.phase;
                        const std::vector<FirstPersonSurface>* source = nullptr;
                        if (phase == 1)
                            source = &cached.semanticResidentSurfaces;
                        else if (phase == 2 && !cached.animated)
                            source = &cached.cameraIndependentResidentSurfaces;
                        else if (phase >= 3 && phase <= 6 && !cached.animated)
                        {
                            const uint8_t rotation = phase - 3;
                            if (cached.rotations[rotation].valid)
                                source = &cached.rotations[rotation].residentSurfaces;
                        }
                        if (source == nullptr)
                        {
                            staging.tileCursor.nextPhase();
                            continue;
                        }
                        if (!ResumeFirstPersonPacketSurfaces(
                                *source, staging.tileCursor, tileBudget,
                                [&](const FirstPersonSurface& surface) {
                                    if (phase >= 3)
                                    {
                                        if (surface.reconstructionGroup != 0)
                                            return;
                                        uint8_t selected = 0xffu;
                                        const auto record = FirstPersonNativeRecordOf(surface);
                                        if (record.instance != 0)
                                        {
                                            const auto found = selectedNativeInstances.find(record);
                                            if (found != selectedNativeInstances.end())
                                                selected = found->second;
                                            else
                                            {
                                                selected = FirstPersonReadyInstanceRotation(
                                                    cached, surface, false);
                                                selectedNativeInstances.emplace(record, selected);
                                            }
                                        }
                                        else
                                            selected = FirstPersonReadyInstanceRotation(
                                                cached, surface, false);
                                        if (selected != phase - 3)
                                            return;
                                    }
                                    add(surface, owningRegion);
                                }))
                            break;
                    }
                    if (staging.tileCursor.phase <= 6)
                        break;
                    staging.tileCursor.nextTile();
                    ++staging.nextTile;
                }
                if (staging.nextTile != 64)
                    continue;
                bool editedDuringBuild = false;
                for (size_t index = 0; index < 64; ++index)
                {
                    const CoordsXY tile{
                        (rx * 8 + int32_t(index % 8)) * kCoordsXYStep,
                        (ry * 8 + int32_t(index / 8)) * kCoordsXYStep
                    };
                    const uint64_t liveSignature = MapIsLocationValid(tile)
                        ? NativeTileSignature(tile) : 0;
                    if (liveSignature != staging.sourceSignatures[index])
                    {
                        editedDuringBuild = true;
                        break;
                    }
                }
                if (editedDuringBuild)
                {
                    // Retain the previous immutable cell. A true edit requires
                    // a fresh coherent snapshot, not a mixed-generation cell.
                    _localSceneryBuilds.erase(key);
                    continue;
                }
                // Preserve warm native-owned faces during source revalidation.
                const auto current = _localSceneryPackets.find(key);
                if (waitingForSourceRevalidation
                    && current != _localSceneryPackets.end()
                    && current->second.surfaces
                    && !current->second.surfaces->empty()
                    && current->second.containsNativeSource)
                {
                    _localSceneryBuilds.erase(key);
                    continue;
                }
                // A valid new local-cell snapshot may be incomplete even
                // after native capture: one object can be missing while
                // neighbours publish. Carry only independently owned source
                // components that were ACTUALLY in the last published GPU
                // packet and whose complete authoritative tile bytes match.
                // Never use an asset-ready flag in place of publication.
                if (current != _localSceneryPackets.end()
                    && current->second.surfaces
                    && current->second.containsNativeSource
                    && current->second.viewFlags == staging.viewFlags
                    && current->second.trackDesignSaveMode == staging.trackDesignSaveMode
                    && current->second.paintBlockedMode == staging.paintBlockedMode
                    && current->second.paintWideGhostMode == staging.paintWideGhostMode
                    && current->second.selectionGeneration == staging.selectionGeneration
                    && TileInspector::GetSelectedElement() == nullptr)
                {
                    std::unordered_set<FirstPersonNativeInstanceRecord,
                        FirstPersonNativeInstanceRecordHash> present;
                    for (const auto& face : surfaces)
                        if (face.sourceInstanceKey != 0)
                            present.insert(FirstPersonNativeRecordOf(face));
                    std::unordered_set<FirstPersonNativeInstanceRecord,
                        FirstPersonNativeInstanceRecordHash> retained;
                    for (const auto& old : *current->second.surfaces)
                    {
                        if (!FirstPersonIndependentLocalScenery(old)
                            || old.sourceInstanceKey == 0)
                            continue;
                        const auto record = FirstPersonNativeRecordOf(old);
                        if (present.contains(record))
                            continue;
                        const int32_t tx = old.diagnosticSourceTile.x / kCoordsXYStep;
                        const int32_t ty = old.diagnosticSourceTile.y / kCoordsXYStep;
                        if (tx < rx * 8 || tx >= (rx + 1) * 8
                            || ty < ry * 8 || ty >= (ry + 1) * 8)
                            continue;
                        const size_t index = size_t(ty - ry * 8) * 8
                            + size_t(tx - rx * 8);
                        if (staging.sourceSignatures[index] == 0
                            || staging.sourceSignatures[index]
                                != current->second.sourceSignatures[index])
                            continue;
                        if (!retained.contains(record))
                        {
                            const auto role =
                                static_cast<FirstPersonPaintSemanticRole>(record.role);
                            const uint64_t tileKey = TerrainKey(tx, ty);
                            if ((role == FirstPersonPaintSemanticRole::smallSceneryBody
                                    && mapOwnedNativeKeys.contains(record.instance))
                                || (role == FirstPersonPaintSemanticRole::pathDeck
                                    && mappedPathTiles.contains(tileKey))
                                || (role == FirstPersonPaintSemanticRole::wall
                                    && mappedWallTiles.contains(tileKey)))
                            {
                                present.insert(record);
                                continue;
                            }
                            retained.insert(record);
                        }
                        // Published local faces have a local-cell GPU key.
                        // Reapply the authoritative source-region key only
                        // for the normal bounded add()/bounds/material path.
                        auto carried = old;
                        const uint64_t sourceRegion = FirstPersonGpuRegionKey(tx, ty);
                        carried.gpuRegion = sourceRegion;
                        add(carried, sourceRegion);
                        if (current->second.mapOwnedInstanceKeys.contains(record.instance))
                            mapOwnedInstanceKeys.insert(record.instance);
                    }
                }
                std::sort(dependencies.begin(), dependencies.end());
                dependencies.erase(
                    std::unique(dependencies.begin(), dependencies.end()),
                    dependencies.end());
                auto& packet = _localSceneryPackets[key];
                if ((packet.mappedPathTiles != mappedPathTiles
                    || packet.mappedWallTiles != mappedWallTiles))
                {
                    // First appearance, removal or fallback of a map-owned
                    // deck changes which presentation must win in the
                    // coherent 32x32 packet. A general source revision alone
                    // does not restart an in-progress geometry worker.
                    const uint64_t coherentKey = FirstPersonGpuRegionKey(
                        rx * 8, ry * 8);
                    MarkFirstPersonAuthoritativeRegionEdit(coherentKey);
                    MarkStaticRegionPacketDirty(coherentKey);
                }
                packet.sourceSignatures = staging.sourceSignatures;
                packet.viewFlags = scene.options.viewFlags;
                packet.trackDesignSaveMode = gTrackDesignSaveMode;
                packet.paintBlockedMode = gPaintBlockedTiles;
                packet.paintWideGhostMode = gPaintWidePathsAsGhost;
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
                    packet.mapOwnedInstanceKeys = std::move(mapOwnedInstanceKeys);
                    packet.mappedPathTiles = std::move(mappedPathTiles);
                    packet.mappedWallTiles = std::move(mappedWallTiles);
                    packet.dirty = packet.invalidationSerial != staging.sourceInvalidationSerial;
                    _localSceneryBuilds.erase(key);
                    continue;
                }
                packet.fingerprint = fingerprint;
                packet.containsNativeSource = containsNativeSource;
                packet.mapOwnedInstanceKeys = std::move(mapOwnedInstanceKeys);
                packet.mappedPathTiles = std::move(mappedPathTiles);
                packet.mappedWallTiles = std::move(mappedWallTiles);
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
                packet.dirty = packet.invalidationSerial != staging.sourceInvalidationSerial;
                _localSceneryBuilds.erase(key);
            }

            // Visible cells are discovered in an unordered set. Their GPU
            // owner-replay order must not change when nearby cells enter or
            // leave view: equal-depth path edges and local fixtures otherwise
            // change their winner as the camera turns.
            std::vector<uint64_t> orderedVisible(
                visible.begin(), visible.end());
            std::sort(orderedVisible.begin(), orderedVisible.end());
            for (const auto key : orderedVisible)
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

