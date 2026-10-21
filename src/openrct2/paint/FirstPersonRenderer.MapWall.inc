        // Only plain, static, fully opaque wall bodies can leave native
        // PaintSession discovery. Doors, glass, animation, scrolling signs,
        // unsupported slopes retain the proven painter. Opaque double-sided
        // bodies use the native back-facing offset from the shared table.
        // The two authored slope states use the native image table and the
        // same slanted world quad already defined by wall semantics.
        [[nodiscard]] bool FirstPersonMapOwnsPlainWall(
            const WallElement& wall, uint32_t viewFlags)
        {
            const auto* entry = wall.getEntry();
            if (entry == nullptr || wall.isGhost() || wall.isInvisible()
                || wall.getSlope() > 2 || entry->height == 0
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
                && !entry->flags2.has(WallSceneryFlag2::isTransparent)
                && !entry->flags2.has(WallSceneryFlag2::isAnimated)
                && entry->scrolling_mode == kScrollingModeNone;
        }

        // Tile-wide suppression is safe only if EVERY wall edge is mapped.
        // Reject coincident directions, neighbour-side overlaps and structural
        // interfaces rather than hiding any unmatched native wall bodies.
        [[nodiscard]] bool FirstPersonTileHasMapOwnedPlainWall(
            CoordsXY tile, uint32_t viewFlags)
        {
            const auto* element = MapGetFirstElementAt(tile);
            if (element == nullptr)
                return false;
            constexpr std::array<std::array<int32_t, 2>, 4> neighbourStep{ {
                { -kCoordsXYStep, 0 }, { 0, kCoordsXYStep },
                { kCoordsXYStep, 0 }, { 0, -kCoordsXYStep }
            } };
            std::array<bool, 4> directions{};
            size_t count = 0;
            do
            {
                if (element->getType() == TileElementType::track
                    || element->getType() == TileElementType::entrance
                    || element->getType() == TileElementType::largeScenery)
                    return false;
                if (element->getType() != TileElementType::wall)
                    continue;
                const auto* wall = element->asWall();
                if (wall == nullptr
                    || !FirstPersonMapOwnsPlainWall(*wall, viewFlags))
                    return false;
                const uint8_t direction =
                    uint8_t(wall->getDirection()) & 3u;
                if (directions[direction])
                    return false;
                directions[direction] = true;
                ++count;
                // Opposing placements on two tiles can be coplanar and need
                // the existing coherent resolver, regardless of elevation.
                const CoordsXY neighbour{
                    tile.x + neighbourStep[direction][0],
                    tile.y + neighbourStep[direction][1]
                };
                if (!MapIsLocationValid(neighbour))
                    continue;
                const auto* other = MapGetFirstElementAt(neighbour);
                if (other == nullptr)
                    continue;
                do
                {
                    if (other->getType() == TileElementType::wall
                        && other->asWall() != nullptr
                        && (uint8_t(other->asWall()->getDirection()) & 3u)
                            == ((direction + 2u) & 3u))
                        return false;
                } while (!(other++)->isLastForTile());
            } while (!(element++)->isLastForTile());
            return count > 0;
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
                    direction, wall.getSlope(),
                    entry->flags.has(WallSceneryFlag::isDoubleSided), false));
            const auto* sprite = GfxGetG1Element(image);
            // Resident static VBOs deliberately exclude blended sprites.
            // An unsupported material must never be marked map-owned or the
            // coherent native face would be suppressed with no replacement.
            if (image.IsBlended() || sprite == nullptr
                || sprite->width <= 0 || sprite->height <= 0)
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

        // Build all walls in a tile as one transaction: one missing texture
        // must not make the coherent renderer drop the other native faces.
        [[nodiscard]] bool AppendFirstPersonMapPlainWallTile(
            std::vector<FirstPersonSurface>& destination,
            CoordsXY tile, uint32_t viewFlags)
        {
            if (!FirstPersonTileHasMapOwnedPlainWall(tile, viewFlags))
                return false;
            const auto* element = MapGetFirstElementAt(tile);
            if (element == nullptr)
                return false;
            std::vector<FirstPersonSurface> completed;
            do
            {
                if (element->getType() == TileElementType::wall)
                {
                    const auto* wall = element->asWall();
                    if (wall == nullptr
                        || !AppendFirstPersonMapPlainWall(
                            completed, tile, *wall, viewFlags))
                        return false;
                }
            } while (!(element++)->isLastForTile());
            if (completed.empty())
                return false;
            for (auto& surface : completed)
                destination.push_back(std::move(surface));
            return true;
        }
