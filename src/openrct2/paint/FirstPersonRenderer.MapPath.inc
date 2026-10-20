        // Map-native path decks do not require per-placement PaintSession
        // discovery. Restrict direct ownership to flat paths sitting on flat,
        // dry ground: supported, sloped, ghost and wide-plaza presentations
        // continue through the proven native semantic painter.
        [[nodiscard]] bool FirstPersonMapOwnsFlatPathDeck(
            CoordsXY tile, const PathElement& path, uint32_t viewFlags)
        {
            if (path.isGhost() || path.isInvisible()
                || path.isSloped() || path.isWide()
                || gTrackDesignSaveMode
                || (viewFlags & VIEWPORT_FLAG_HIGHLIGHT_PATH_ISSUES) != 0
                || path.getSurfaceDescriptor() == nullptr
                || path.getRailingsDescriptor() == nullptr)
                return false;
            const auto* ground = MapGetSurfaceElementAt(tile);
            return ground != nullptr
                && ground->getSlope() == kTileSlopeFlat
                && ground->getBaseZ() == path.getBaseZ()
                && ground->getWaterHeight() <= path.getBaseZ();
        }

        [[nodiscard]] bool FirstPersonTileHasMapOwnedFlatPathDeck(
            CoordsXY tile, uint32_t viewFlags)
        {
            const auto* element = MapGetFirstElementAt(tile);
            size_t paths = 0;
            bool eligible = false;
            if (element == nullptr)
                return false;
            do
            {
                if (element->getType() == TileElementType::path)
                {
                    ++paths;
                    const auto* path = element->asPath();
                    eligible = path != nullptr
                        && FirstPersonMapOwnsFlatPathDeck(tile, *path, viewFlags);
                }
            } while (!(element++)->isLastForTile());
            return paths == 1 && eligible;
        }

        [[nodiscard]] bool AppendFirstPersonMapFlatPathDeck(
            std::vector<FirstPersonSurface>& destination,
            CoordsXY tile, const PathElement& path, uint32_t viewFlags)
        {
            if (!FirstPersonMapOwnsFlatPathDeck(tile, path, viewFlags))
                return false;
            const auto* descriptor = path.getSurfaceDescriptor();
            // The native painter already exposes the authoritative rotation
            // table, so no parallel/path-specific sprite rule is invented.
            constexpr uint8_t sourceRotation = 0;
            ImageId image(descriptor->image
                + GetPathSurfaceImageOffset(path, sourceRotation));
            if (TileInspector::GetSelectedElement()
                == reinterpret_cast<const TileElement*>(&path))
                image = image.WithRemap(
                    Drawing::FilterPaletteID::paletteGhost);
            if (gPaintBlockedTiles && path.isBlockedByVehicle())
                image = image.WithRemap(
                    Drawing::FilterPaletteID::palette46);
            const auto* sprite = GfxGetG1Element(image);
            if (sprite == nullptr || sprite->width <= 0
                || sprite->height <= 0)
                return false;

            const auto paintOrigin = GetTileElementPaintSpritePosition(
                tile, sourceRotation);
            const auto isoOrigin = Translate3DTo2DWithZ(
                sourceRotation, { paintOrigin, path.getBaseZ() });
            const auto cells = BuildFirstPersonPathFootprint(
                path.getEdges(), path.getCorners(), path.isQueue());
            const auto target = BuildFirstPersonTileInteractionTarget(
                ViewportInteractionItem::footpath, tile,
                reinterpret_cast<const TileElement*>(&path));
            for (const auto& cell : cells)
            {
                const std::array<FirstPersonVec3, 4> points{ {
                    { float(tile.x + cell.x0), float(tile.y + cell.y0),
                        float(path.getBaseZ()) },
                    { float(tile.x + cell.x1), float(tile.y + cell.y0),
                        float(path.getBaseZ()) },
                    { float(tile.x + cell.x1), float(tile.y + cell.y1),
                        float(path.getBaseZ()) },
                    { float(tile.x + cell.x0), float(tile.y + cell.y1),
                        float(path.getBaseZ()) },
                } };
                std::array<FirstPersonVertex, 4> vertices{};
                for (size_t i = 0; i < 4; ++i)
                {
                    const CoordsXYZ world{
                        int32_t(points[i].x),
                        int32_t(points[i].y),
                        int32_t(points[i].z)
                    };
                    const auto projected = Translate3DTo2DWithZ(
                        sourceRotation, world);
                    vertices[i] = {
                        points[i],
                        float(projected.x - isoOrigin.x - sprite->xOffset),
                        float(projected.y - isoOrigin.y - sprite->yOffset)
                    };
                }
                FirstPersonSurface surface{};
                surface.image = image;
                surface.cameraIndependent = true;
                // Path paint is coplanar with terrain. Use the existing exact
                // owner/depth replay; never introduce a world-Z offset.
                surface.coplanarOwner = true;
                surface.gpuRegion = FirstPersonGpuRegionKey(
                    tile.x / kCoordsXYStep, tile.y / kCoordsXYStep);
                surface.interactionTarget = target;
                surface.diagnosticSemanticRole = static_cast<uint8_t>(
                    FirstPersonPaintSemanticRole::pathDeck);
                surface.diagnosticSourceTile = tile;
                EmitQuad(surface, vertices);
                destination.push_back(std::move(surface));
            }
            return true;
        }

