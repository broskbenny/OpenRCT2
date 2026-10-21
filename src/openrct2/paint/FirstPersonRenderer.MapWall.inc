        // Only plain, static, fully opaque wall bodies can leave native
        // PaintSession discovery. Doors, glass, animation, scrolling signs,
        // slope variants and double-sided artwork retain the proven painter.
        [[nodiscard]] bool FirstPersonMapOwnsPlainWall(
            const WallElement& wall, uint32_t viewFlags)
        {
            const auto* entry = wall.getEntry();
            if (entry == nullptr || wall.isGhost() || wall.isInvisible()
                || wall.getSlope() != 0 || entry->height == 0
                || gTrackDesignSaveMode
                || (viewFlags & (
                    VIEWPORT_FLAG_HIGHLIGHT_PATH_ISSUES
                    | VIEWPORT_FLAG_HIDE_SCENERY
                    | VIEWPORT_FLAG_INVISIBLE_SCENERY
                    | VIEWPORT_FLAG_CLIP_VIEW
                    | VIEWPORT_FLAG_UNDERGROUND_INSIDE)) != 0)
                return false;
            return !entry->flags.has(WallSceneryFlag::isDoor)
                && !entry->flags.has(WallSceneryFlag::hasGlass)
                && !entry->flags.has(WallSceneryFlag::isDoubleSided)
                && !entry->flags2.has(WallSceneryFlag2::isTransparent)
                && !entry->flags2.has(WallSceneryFlag2::isAnimated)
                && entry->scrolling_mode == kScrollingModeNone;
        }

        // The coherent native packet can suppress wall artwork by tile only
        // when the tile has exactly one WallElement. More elaborate structures
        // remain native to avoid accidentally replacing a second wall face.
        [[nodiscard]] bool FirstPersonTileHasMapOwnedPlainWall(
            CoordsXY tile, uint32_t viewFlags)
        {
            const auto* element = MapGetFirstElementAt(tile);
            if (element == nullptr)
                return false;
            size_t count = 0;
            bool eligible = false;
            do
            {
                if (element->getType() == TileElementType::wall)
                {
                    ++count;
                    const auto* wall = element->asWall();
                    eligible = wall != nullptr
                        && FirstPersonMapOwnsPlainWall(*wall, viewFlags);
                }
            } while (!(element++)->isLastForTile());
            return count == 1 && eligible;
        }

        [[nodiscard]] bool AppendFirstPersonMapPlainWall(
            std::vector<FirstPersonSurface>& destination,
            CoordsXY tile, const WallElement& wall, uint32_t viewFlags)
        {
            if (!FirstPersonMapOwnsPlainWall(wall, viewFlags))
                return false;
            const auto* entry = wall.getEntry();
            const uint8_t direction = uint8_t(wall.getDirection()) & 3u;
            ImageId templateImage{};
            if (entry->flags.has(WallSceneryFlag::hasPrimaryColour))
                templateImage = templateImage.WithPrimary(wall.getPrimaryColour());
            if (entry->flags.has(WallSceneryFlag::hasSecondaryColour))
                templateImage = templateImage.WithSecondary(wall.getSecondaryColour());
            if (entry->flags.has(WallSceneryFlag::hasTertiaryColour))
                templateImage = templateImage.WithTertiary(wall.getTertiaryColour());
            if (TileInspector::GetSelectedElement()
                == reinterpret_cast<const TileElement*>(&wall))
                templateImage = ImageId().WithRemap(
                    Drawing::FilterPaletteID::paletteGhost);
            const auto image = templateImage.WithIndex(
                entry->image + GetFirstPersonWallBodyImageOffset(
                    direction, wall.getSlope(), false, false));
            const auto* sprite = GfxGetG1Element(image);
            if (sprite == nullptr || sprite->width <= 0
                || sprite->height <= 0)
                return false;

            // Exactly the same local artwork offsets used by PaintWallWall.
            constexpr std::array<std::array<int32_t, 2>, 4>
                artworkOffsets{ { { 0, 0 }, { 1, 31 },
                    { 31, 0 }, { 2, 1 } } };
            constexpr uint8_t sourceRotation = 0;
            const auto paintOrigin = GetTileElementPaintSpritePosition(
                tile, sourceRotation);
            const auto relative = artworkOffsets[direction];
            const auto imageAnchor = Translate3DTo2DWithZ(
                sourceRotation,
                { paintOrigin.x + relative[0], paintOrigin.y + relative[1],
                    wall.getBaseZ() });
            const auto plane = BuildFirstPersonWallPlane(
                tile, wall.getBaseZ(), direction, wall.getSlope(),
                int32_t(entry->height) * kCoordsZStep);
            std::array<FirstPersonVertex, 4> vertices{};
            for (size_t i = 0; i < vertices.size(); ++i)
            {
                const auto& world = plane.corners[i];
                const auto projected = Translate3DTo2DWithZ(
                    sourceRotation,
                    { int32_t(std::lround(world.x)),
                        int32_t(std::lround(world.y)),
                        int32_t(std::lround(world.z)) });
                vertices[i] = {
                    world,
                    float(projected.x - imageAnchor.x - sprite->xOffset),
                    float(projected.y - imageAnchor.y - sprite->yOffset)
                };
            }
            FirstPersonSurface surface{};
            surface.image = image;
            surface.cameraIndependent = true;
            surface.coplanarOwner = true;
            surface.gpuRegion = FirstPersonGpuRegionKey(
                tile.x / kCoordsXYStep, tile.y / kCoordsXYStep);
            surface.interactionTarget = BuildFirstPersonTileInteractionTarget(
                ViewportInteractionItem::wall, tile,
                reinterpret_cast<const TileElement*>(&wall));
            surface.diagnosticSemanticRole =
                static_cast<uint8_t>(FirstPersonPaintSemanticRole::wall);
            surface.diagnosticSourceTile = tile;
            EmitQuad(surface, vertices);
            destination.push_back(std::move(surface));
            return true;
        }
