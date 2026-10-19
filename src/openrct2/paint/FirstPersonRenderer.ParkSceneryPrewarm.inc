        // Asset preparation is persistent and park-wide. Visibility selects
        // priority, not whether a reusable scenery asset is ever discovered.
        // Visible tiles are prepared first by World.inc; this small residual
        // slice advances across the entire authoritative map independently
        // of camera orientation, without native paint sessions.
        void PrewarmParkSmallSceneryAssets()
        {
            const auto map = getGameState().mapSize;
            if (map.x <= 0 || map.y <= 0)
                return;
            static uint64_t epoch = 0;
            static size_t cursor = 0;
            if (epoch != _sceneEpoch)
            {
                epoch = _sceneEpoch;
                cursor = 0;
            }
            const size_t width = size_t(map.x);
            const size_t height = size_t(map.y);
            const size_t total = width * height;
            if (total == 0)
                return;

            FirstPersonFrameBudget budget(
                32, std::chrono::microseconds(300));
            while (budget.take())
            {
                const size_t tileNumber = cursor++ % total;
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
                        PrewarmFirstPersonSmallSceneryAsset(tile, *small);
                } while (!(element++)->isLastForTile());
            }
        }
