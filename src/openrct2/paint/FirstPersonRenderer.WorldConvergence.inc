        // Authoritative park inventory. Native paint sessions observe *views*,
        // not objects: one valid tile rotation does not certify that every
        // placed object on that tile has a presentation source.
        struct FirstPersonWorldObjectRecord
        {
            uint32_t elementIndex{};
            TileElementType type{};
            uint64_t identity{};
            uint64_t nativeSourceKey{};
        };

        struct FirstPersonWorldTileRecord
        {
            uint64_t sourceSignature{};
            std::vector<FirstPersonWorldObjectRecord> objects;
        };

        static uint64_t _worldInventoryEpoch = 0;
        static size_t _worldInventoryObjectCount = 0;
        static std::unordered_map<uint64_t, FirstPersonWorldTileRecord>
            _worldInventory;

        void ClearFirstPersonWorldInventory()
        {
            _worldInventory.clear();
            _worldInventoryObjectCount = 0;
            _worldInventoryEpoch = _sceneEpoch;
        }

        void EnsureFirstPersonWorldInventoryEpoch()
        {
            if (_worldInventoryEpoch != _sceneEpoch)
                ClearFirstPersonWorldInventory();
        }

        void InvalidateFirstPersonWorldInventoryTile(int32_t tx, int32_t ty)
        {
            EnsureFirstPersonWorldInventoryEpoch();
            const auto it = _worldInventory.find(TerrainKey(tx, ty));
            if (it == _worldInventory.end())
                return;
            _worldInventoryObjectCount -= it->second.objects.size();
            _worldInventory.erase(it);
        }

        [[nodiscard]] uint64_t FirstPersonWorldNativeSourceKey(
            int32_t tx, int32_t ty, uint32_t elementIndex,
            TileElementType type)
        {
            uint64_t key = 14695981039346656037ull;
            ExtendStableKey(key, uint32_t(tx));
            ExtendStableKey(key, uint32_t(ty));
            ExtendStableKey(key, elementIndex);
            ExtendStableKey(key, uint8_t(type));
            return key == 0 ? 1 : key;
        }

        // Called by the existing finite full-park traversal and its edit
        // queue, not by a new perpetual park scan. Empty tiles allocate no
        // record. Source signatures prevent a replaced element at the same
        // index from inheriting the previous object's identity.
        void ObserveFirstPersonWorldInventoryTile(CoordsXY tile)
        {
            EnsureFirstPersonWorldInventoryEpoch();
            const int32_t tx = tile.x / kCoordsXYStep;
            const int32_t ty = tile.y / kCoordsXYStep;
            const uint64_t tileKey = TerrainKey(tx, ty);
            auto* element = MapGetFirstElementAt(tile);
            if (element == nullptr)
            {
                InvalidateFirstPersonWorldInventoryTile(tx, ty);
                return;
            }

            FirstPersonWorldTileRecord next{};
            next.sourceSignature = NativeTileSignature(tile);
            const auto prior = _worldInventory.find(tileKey);
            if (prior != _worldInventory.end()
                && prior->second.sourceSignature == next.sourceSignature)
                return;

            uint32_t index = 0;
            do
            {
                if (element->getType() != TileElementType::surface
                    && !element->isGhost() && !element->isInvisible())
                {
                    FirstPersonWorldObjectRecord object{};
                    object.elementIndex = index;
                    object.type = element->getType();
                    object.nativeSourceKey = FirstPersonWorldNativeSourceKey(
                        tx, ty, index, object.type);
                    if (object.type == TileElementType::smallScenery
                        && element->asSmallScenery() != nullptr)
                    {
                        object.identity = FirstPersonLocalSmallSceneryInstanceKey(
                            tx, ty, index, *element->asSmallScenery());
                    }
                    else
                    {
                        // Source identity is exact to this tile revision.
                        // Grouped objects may span several tile elements;
                        // those remain separately inventoried placements.
                        uint64_t identity = object.nativeSourceKey;
                        ExtendStableKey(identity, next.sourceSignature);
                        object.identity = identity == 0 ? 1 : identity;
                    }
                    next.objects.push_back(object);
                }
                ++index;
            } while (!(element++)->isLastForTile());

            if (prior != _worldInventory.end())
            {
                _worldInventoryObjectCount -= prior->second.objects.size();
                _worldInventory.erase(prior);
            }
            if (!next.objects.empty())
            {
                _worldInventoryObjectCount += next.objects.size();
                _worldInventory.emplace(tileKey, std::move(next));
            }
        }

        [[nodiscard]] bool FirstPersonNativeSourceContainsInstance(
            const StaticPaintCacheEntry& cached, uint64_t nativeSourceKey)
        {
            const auto contains = [nativeSourceKey](
                const std::vector<FirstPersonSurface>& surfaces) {
                return std::any_of(surfaces.begin(), surfaces.end(),
                    [nativeSourceKey](const FirstPersonSurface& source) {
                        return source.sourceInstanceKey == nativeSourceKey
                            && source.diagnosticSemanticRole
                                == static_cast<uint8_t>(
                                    FirstPersonPaintSemanticRole::smallSceneryBody);
                    });
            };
            if (contains(cached.cameraIndependentResidentSurfaces)
                || contains(cached.cameraIndependentStreamedSurfaces))
                return true;
            for (const auto& view : cached.rotations)
            {
                if (view.valid
                    && (contains(view.residentSurfaces)
                        || contains(view.streamedSurfaces)))
                    return true;
            }
            return false;
        }

        // A tile-level valid rotation is not proof of coverage for every
        // small-scenery element. Where a specific placement has neither
        // exact map-owned publication nor native source evidence, request
        // an as-yet-unobserved rotation. Never repeat an already valid empty
        // observation indefinitely: that case remains diagnosable but does
        // not consume the frame's two capture slots forever.
        [[nodiscard]] std::optional<uint8_t>
        FirstPersonMissingSmallSceneryCaptureRotation(
            CoordsXY tile, const StaticPaintCacheEntry& cached,
            uint8_t preferredRotation)
        {
            if (!cached.valid || cached.animated)
                return std::nullopt;
            auto* element = MapGetFirstElementAt(tile);
            if (element == nullptr)
                return std::nullopt;
            uint32_t index = 0;
            do
            {
                if (element->getType() == TileElementType::smallScenery
                    && !element->isGhost() && !element->isInvisible())
                {
                    const auto* small = element->asSmallScenery();
                    if (small != nullptr && small->getEntry() != nullptr
                        && !FirstPersonLocalSceneryOwnsSmallScenery(tile, *small)
                        && !FirstPersonNativeSourceContainsInstance(
                            cached, FirstPersonWorldNativeSourceKey(
                                tile.x / kCoordsXYStep,
                                tile.y / kCoordsXYStep, index,
                                TileElementType::smallScenery)))
                    {
                        if (preferredRotation < 4
                            && !cached.rotations[preferredRotation].valid)
                            return preferredRotation;
                        for (uint8_t rotation = 0; rotation < 4; ++rotation)
                        {
                            if (!cached.rotations[rotation].valid)
                                return rotation;
                        }
                    }
                }
                ++index;
            } while (!(element++)->isLastForTile());
            return std::nullopt;
        }

        struct FirstPersonWorldCoverage
        {
            size_t inventoriedVisibleObjects = 0;
            size_t smallPlacements = 0;
            size_t mapPublishedSmall = 0;
            size_t nativeEvidenceSmall = 0;
            size_t missingSmall = 0;
            size_t exhaustedSmall = 0;
            size_t sampledTiles = 0;
            size_t tilesWithoutObjectRecords = 0;
        };

        // An explicit bounded-frequency audit, not a per-frame full-park
        // walk. These are presentation SOURCE certificates; native evidence
        // alone does not prove that GPU submission and visibility succeeded.
        [[nodiscard]] FirstPersonWorldCoverage
        InspectFirstPersonWorldCoverage(const FirstPersonScene& scene)
        {
            FirstPersonWorldCoverage coverage{};
            static size_t sampleCursor = 0;
            const size_t count = scene.visibleTiles.size();
            FirstPersonFrameBudget sampleBudget(
                256, std::chrono::microseconds(750));
            const size_t begin = count == 0 ? 0 : sampleCursor % count;
            while (coverage.sampledTiles < count && sampleBudget.take())
            {
                const auto tile = scene.visibleTiles[
                    (begin + coverage.sampledTiles++) % count];
                const int32_t tx = tile.x / kCoordsXYStep;
                const int32_t ty = tile.y / kCoordsXYStep;
                const auto inventory = _worldInventory.find(TerrainKey(tx, ty));
                if (inventory == _worldInventory.end())
                {
                    // Empty map tiles intentionally have no inventory record.
                    ++coverage.tilesWithoutObjectRecords;
                    continue;
                }
                const auto& objects = inventory->second.objects;
                coverage.inventoriedVisibleObjects += objects.size();
                const auto cell = _localSceneryPackets.find(
                    FirstPersonLocalSceneryRegionKey(tx, ty));
                const auto native = _staticPaintCache.find(TerrainKey(tx, ty));
                const bool nativeCurrent = native != _staticPaintCache.end()
                    && native->second.valid && !native->second.dirty
                    && native->second.signature == inventory->second.sourceSignature;
                for (const auto& object : objects)
                {
                    if (object.type != TileElementType::smallScenery)
                        continue;
                    ++coverage.smallPlacements;
                    if (cell != _localSceneryPackets.end()
                        && cell->second.surfaces
                        && cell->second.mapOwnedInstanceKeys.contains(object.identity))
                    {
                        ++coverage.mapPublishedSmall;
                    }
                    else if (nativeCurrent
                        && FirstPersonNativeSourceContainsInstance(
                            native->second, object.nativeSourceKey))
                    {
                        ++coverage.nativeEvidenceSmall;
                    }
                    else
                    {
                        ++coverage.missingSmall;
                        if (nativeCurrent
                            && std::all_of(native->second.rotations.begin(),
                                native->second.rotations.end(),
                                [](const auto& view) { return view.valid; }))
                            ++coverage.exhaustedSmall;
                    }
                }
            }
            sampleCursor += coverage.sampledTiles;
            return coverage;
        }
