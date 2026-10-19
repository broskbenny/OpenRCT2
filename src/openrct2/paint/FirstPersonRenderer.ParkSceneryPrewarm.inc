        // Offscreen preparation is a finite initial map traversal plus
        // edit-triggered tile work. Camera-visible assets retain priority in
        // PrewarmVisibleSmallSceneryAssets; there is no permanent full-park
        // polling pass after the initial traversal completes.
        static uint64_t _parkSceneryScanEpoch = 0;
        static size_t _parkSceneryScanCursor = 0;
        static std::unordered_set<uint64_t> _parkSceneryChangedTiles;

        void QueueFirstPersonParkSceneryTile(int32_t tx, int32_t ty)
        {
            if (_parkSceneryScanEpoch != _sceneEpoch)
            {
                _parkSceneryChangedTiles.clear();
                _parkSceneryScanCursor = 0;
                _parkSceneryScanEpoch = _sceneEpoch;
            }
            _parkSceneryChangedTiles.insert(TerrainKey(tx, ty));
        }

        void PrewarmParkSmallSceneryAssets()
        {
            const auto map = getGameState().mapSize;
            if (map.x <= 0 || map.y <= 0)
                return;
            if (_parkSceneryScanEpoch != _sceneEpoch)
            {
                _parkSceneryChangedTiles.clear();
                _parkSceneryScanCursor = 0;
                _parkSceneryScanEpoch = _sceneEpoch;
            }
            const size_t width = size_t(map.x);
            const size_t height = size_t(map.y);
            const size_t total = width * height;
            if (total == 0)
                return;

            FirstPersonFrameBudget budget(
                32, std::chrono::microseconds(300));
            size_t examined = 0;
            while (budget.take())
            {
                const bool moreInitial = _parkSceneryScanCursor < total;
                const bool takeDirty = !_parkSceneryChangedTiles.empty()
                    && (!moreInitial || (examined & 1u) == 0);
                size_t tileNumber = 0;
                if (takeDirty)
                {
                    auto it = _parkSceneryChangedTiles.begin();
                    const uint64_t key = *it;
                    _parkSceneryChangedTiles.erase(it);
                    const size_t tx = size_t(uint32_t(key >> 32));
                    const size_t ty = size_t(uint32_t(key));
                    if (tx >= width || ty >= height)
                    {
                        ++examined;
                        continue;
                    }
                    tileNumber = ty * width + tx;
                }
                else if (moreInitial)
                {
                    // Advance even if this tile has no objects or was edited.
                    // Half the work remains reserved for full-map progress
                    // under sustained edit traffic, preventing starvation.
                    tileNumber = _parkSceneryScanCursor++;
                }
                else
                {
                    // The initial park scan is complete and no edited tiles
                    // need reconstruction. Do no work until the next change.
                    break;
                }
                ++examined;
                const CoordsXY tile{
                    int32_t(tileNumber % width) * kCoordsXYStep,
                    int32_t(tileNumber / width) * kCoordsXYStep
                };
                if (!MapIsLocationValid(tile))
                    continue;
                auto* element = MapGetFirstElementAt(tile);
                if (element == nullptr)
                    continue;
                do
                {
                    if (element->getType() != TileElementType::smallScenery
                        || element->isGhost() || element->isInvisible())
                        continue;
                    if (const auto* small = element->asSmallScenery();
                        small != nullptr)
                    {
                        const auto* entry = small->getEntry();
                        if (entry != nullptr
                            && FirstPersonMapSmallSceneryEligible(
                                *entry, *small, 0))
                            PrewarmFirstPersonSmallSceneryAsset(
                                tile, *small);
                    }
                } while (!(element++)->isLastForTile());
            }
        }
