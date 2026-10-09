        bool BuildSmallSceneryGeometry(FirstPersonScene& scene,
            const SmallSceneryElement& element, const SmallSceneryEntry& metadata,
            ImageId image, const FirstPersonSpriteSnapshot& sprites)
        {
            const auto* small = &element;
            const auto* entry = &metadata;
            FirstPersonVisualHull ownedHull{};
            const FirstPersonVisualHull* hull = nullptr;
            if (!FirstPersonSmallSceneryUsesCrossedTreeAppearance(metadata))
            {
                ownedHull = BuildFirstPersonSmallSceneryVisualHullFromSnapshot(
                    metadata, element,
                    [&](ImageIndex id) { return sprites.get(id); });
                if (!ownedHull.valid)
                    return false;
                hull = &ownedHull;
            }
            const auto appearance =
                BuildFirstPersonSmallSceneryAppearancePlan(
                    metadata, element, hull);
            if (!appearance.valid())
                return false;
            const float tileX = 0, tileY = 0, baseZ = 0;
            // Face reprojection is an asset/state operation, not an instance
            // operation. Identical scenery instances can share the expensive
            // local-space bake and apply only their world translation and
            // recolour material at submission time.
            struct BakedSmallSceneryEntry
            {
                bool resolved = false;
                bool usable = false;
                std::vector<FirstPersonSurface> localSurfaces;
            };
            BakedSmallSceneryEntry baked;

            auto faces = appearance.faces;
            const auto treeFootprint =
                appearance.kind
                    == FirstPersonSmallSceneryAppearanceKind::crossedTree
                ? BuildFirstPersonSmallSceneryFootprint(
                    *entry, *small)
                : FirstPersonSmallSceneryFootprint{};
            if (faces.empty())
            {
                baked.resolved = true;
                baked.usable = false;
                return false;
            }

            const auto faceVisible =
                [](FirstPersonVec3 normal,
                    uint8_t sourceRotation) {
                    return FirstPersonSurfaceVisibleFromNativeView(
                        normal, sourceRotation);
                };

            // Ownership is computed independently of texture fit. A projective
            // source is trustworthy only when this physical face is actually
            // frontmost in that native isometric view.
            std::array<FirstPersonDepthOwnerMap, 4>
                depthOwners{};
            if (appearance.usesDepthOwnership)
            for (uint8_t sourceRotation = 0;
                 sourceRotation < 4; ++sourceRotation)
            {
                for (size_t faceIndex = 0;
                     faceIndex < faces.size(); ++faceIndex)
                {
                    const auto& face = faces[faceIndex];
                    if (!faceVisible(
                            face.normal, sourceRotation))
                        continue;
                    std::array<ScreenCoordsXY, 4> screen{};
                    std::array<float, 4> depth{};
                    for (size_t i = 0;
                         i < face.corners.size(); ++i)
                    {
                        const CoordsXYZ point{
                            int32_t(std::lround(
                                face.corners[i].x)),
                            int32_t(std::lround(
                                face.corners[i].y)),
                            int32_t(std::lround(
                                face.corners[i].z)),
                        };
                        screen[i] =
                            Translate3DTo2DWithZ(
                                sourceRotation, point);
                        depth[i] =
                            FirstPersonIsoDepth(
                                sourceRotation, point);
                    }
                    AddFirstPersonDepthTriangle(
                        depthOwners[sourceRotation],
                        uint32_t(faceIndex),
                        { screen[0], screen[1], screen[2] },
                        { depth[0], depth[1], depth[2] });
                    AddFirstPersonDepthTriangle(
                        depthOwners[sourceRotation],
                        uint32_t(faceIndex),
                        { screen[0], screen[2], screen[3] },
                        { depth[0], depth[2], depth[3] });
                }
            }

            struct SmallSceneryDecodedView
            {
                ImageId image{};
                const G1Element* g1 = nullptr;
                uint8_t rotation = 0;
                int32_t groundingShiftU = 0;
                int32_t groundingShiftV = 0;
                FirstPersonTreeArtworkMapping
                    xCentredMapping{};
                FirstPersonTreeArtworkMapping
                    yCentredMapping{};
                std::vector<uint8_t> pixels;
            };
            const auto textureViews =
                BuildFirstPersonSmallSceneryTextureViews(
                    *entry, *small,
                    FirstPersonSmallSceneryWitherStage(
                        *entry, *small));
            std::vector<SmallSceneryDecodedView>
                decodedViews;
            decodedViews.reserve(
                textureViews.size());
            for (const auto& view : textureViews)
            {
                const auto sourceImage =
                    image.WithIndex(view.image);
                const auto* g1 =
                    sprites.get(sourceImage);
                if (g1 == nullptr
                    || g1->width <= 0
                    || g1->height <= 0)
                    continue;
                auto decoded =
                    DecodeFirstPersonSpritePixels(*g1);
                if (!decoded.has_value())
                    continue;
                // Native artwork registration is a presentation-space
                // correction, not a change to the inferred physical mesh.
                // Use the exact shift certified during four-view carving.
                int32_t groundingShiftU = hull != nullptr
                    ? hull->viewPixelOffsets[view.imageDirection & 3u].x
                    : 0;
                int32_t groundingShiftV = hull != nullptr
                    ? hull->viewPixelOffsets[view.imageDirection & 3u].y
                    : 0;
                if (appearance.kind
                    == FirstPersonSmallSceneryAppearanceKind::crossedTree)
                {
                    if (treeFootprint.valid)
                    {
                        const auto nominalGround =
                            FirstPersonSmallScenerySpritePixelForPoint(
                                *entry, *small,
                                view.imageDirection,
                                {
                                    int32_t(std::lround(
                                        treeFootprint.centreForward())),
                                    int32_t(std::lround(
                                        treeFootprint.centreRight())),
                                    0,
                                },
                                *g1);
                        const auto grounding =
                            FindFirstPersonTreeArtworkGrounding(
                                *decoded,
                                g1->width, g1->height,
                                nominalGround.x,
                                nominalGround.y,
                                g1->flags.has(
                                    G1Flag::hasTransparency));
                        if (grounding.valid)
                        {
                            groundingShiftU =
                                grounding.shiftU;
                            groundingShiftV =
                                grounding.shiftV;
                        }
                    }
                }
                FirstPersonTreeArtworkMapping
                    xCentredMapping{};
                FirstPersonTreeArtworkMapping
                    yCentredMapping{};
                if (appearance.kind
                        == FirstPersonSmallSceneryAppearanceKind::crossedTree
                    && treeFootprint.valid)
                {
                    constexpr int32_t kBasisStep = 16;
                    const int32_t cx =
                        int32_t(std::lround(
                            treeFootprint.centreForward()));
                    const int32_t cy =
                        int32_t(std::lround(
                            treeFootprint.centreRight()));
                    const auto effectivePixel =
                        [&](CoordsXYZ local) {
                            const auto pixel =
                                FirstPersonSmallScenerySpritePixelForPoint(
                                    *entry, *small,
                                    view.imageDirection,
                                    local, *g1);
                            return std::array<float, 2>{
                                float(
                                    pixel.x
                                    + groundingShiftU),
                                float(
                                    pixel.y
                                    + groundingShiftV),
                            };
                        };
                    const auto ground =
                        effectivePixel(
                            { cx, cy, 0 });
                    const auto up =
                        effectivePixel(
                            { cx, cy, kBasisStep });
                    const auto alongY =
                        effectivePixel(
                            {
                                cx,
                                cy + kBasisStep,
                                0,
                            });
                    const auto alongX =
                        effectivePixel(
                            {
                                cx + kBasisStep,
                                cy,
                                0,
                            });

                    // Tree sprites are authored upright artwork, not literal
                    // photographs of a vertical world plane. Native isometric
                    // projection makes horizontal world motion drift in screen
                    // V; feeding that drift back into the carrier shears a
                    // rectangular sprite into a rhombus. Keep the two artwork
                    // axes independent: world-horizontal drives source U and
                    // world-up drives source V.
                    const float groundU =
                        ground[0] + 0.5f;
                    const float groundV =
                        ground[1] + 1.0f;
                    const float upVPerUnit =
                        (up[1] - ground[1])
                        / float(kBasisStep);
                    xCentredMapping =
                        BuildFirstPersonTreeArtworkMapping(
                            groundU, groundV,
                            (alongY[0] - ground[0])
                                / float(kBasisStep),
                            upVPerUnit);
                    yCentredMapping =
                        BuildFirstPersonTreeArtworkMapping(
                            groundU, groundV,
                            (alongX[0] - ground[0])
                                / float(kBasisStep),
                            upVPerUnit);
                }
                decodedViews.push_back({
                    sourceImage,
                    g1,
                    view.imageDirection,
                    groundingShiftU,
                    groundingShiftV,
                    xCentredMapping,
                    yCentredMapping,
                    std::move(*decoded),
                });
            }

            if (appearance.kind
                == FirstPersonSmallSceneryAppearanceKind::crossedTree)
            {
                FirstPersonTreePlaneExtent
                    xCentredExtent{};
                FirstPersonTreePlaneExtent
                    yCentredExtent{};
                if (treeFootprint.valid)
                {
                    for (const auto& source :
                         decodedViews)
                    {
                        const auto bounds =
                            FindFirstPersonTreeArtworkBounds(
                                source.pixels,
                                source.g1->width,
                                source.g1->height,
                                source.g1->flags.has(
                                    G1Flag::hasTransparency));
                        if (!bounds.valid)
                            continue;
                        xCentredExtent.include(
                            source.xCentredMapping,
                            bounds);
                        yCentredExtent.include(
                            source.yCentredMapping,
                            bounds);
                    }

                    const auto artworkFaces =
                        BuildFirstPersonCrossedTreeFaces(
                            treeFootprint,
                            &xCentredExtent,
                            &yCentredExtent);
                    if (!artworkFaces.empty())
                        faces = artworkFaces;
                }
            }

            struct SmallSceneryFaceView
            {
                const SmallSceneryDecodedView* source = nullptr;
                float ownership = 0.0f;
                float sourceCoverage = 0.0f;
                float score = 0.0f;
            };

            const auto edgeLength =
                [](FirstPersonVec3 a, FirstPersonVec3 b) {
                    const float dx = b.x - a.x;
                    const float dy = b.y - a.y;
                    const float dz = b.z - a.z;
                    return std::sqrt(
                        dx * dx + dy * dy + dz * dz);
                };
            const auto ownedNear =
                [&](uint8_t sourceRotation,
                    uint32_t owner, int32_t x, int32_t y) {
                    if (!appearance.usesDepthOwnership)
                        return true;
                    for (int32_t dy = -1; dy <= 1; ++dy)
                    for (int32_t dx = -1; dx <= 1; ++dx)
                    {
                        const auto found =
                            depthOwners[sourceRotation].find(
                                FirstPersonSilhouettePixelKey(
                                    x + dx, y + dy));
                        if (found
                                != depthOwners[sourceRotation].end()
                            && found->second.owner == owner)
                            return true;
                    }
                    return false;
                };

            const size_t surfaceStart = scene.surfaces.size();
            size_t emittedFaces = 0;
            FirstPersonHullMaterialCoverage materialCoverage(faces);
            for (size_t faceIndex = 0;
                 faceIndex < faces.size(); ++faceIndex)
            {
                const auto& face = faces[faceIndex];
                std::vector<SmallSceneryFaceView> candidates;
                for (const auto& source : decodedViews)
                {
                    if (!faceVisible(
                            face.normal,
                            source.rotation))
                        continue;

                    std::array<ScreenCoordsXY, 4> projected{};
                    for (size_t i = 0;
                         i < face.corners.size(); ++i)
                    {
                        projected[i] =
                            Translate3DTo2DWithZ(
                                source.rotation,
                                {
                                    int32_t(std::lround(
                                        face.corners[i].x)),
                                    int32_t(std::lround(
                                        face.corners[i].y)),
                                    int32_t(std::lround(
                                        face.corners[i].z)),
                                });
                    }
                    FirstPersonSilhouette silhouette{};
                    AddFirstPersonSilhouetteQuad(
                        silhouette, projected);
                    float ownership = 1.0f;
                    if (appearance.usesDepthOwnership)
                    {
                        ownership =
                            FirstPersonDepthOwnerCoverage(
                                depthOwners[source.rotation],
                                uint32_t(faceIndex),
                                silhouette);
                        if (ownership < 0.20f)
                            continue;
                    }

                    const auto paintOffset =
                        FirstPersonSmallSceneryPaintOffset(
                            *entry, *small,
                            source.rotation);
                    const auto sourceOrigin =
                        Translate3DTo2DWithZ(
                            source.rotation,
                            { paintOffset, 0 });
                    const float sourceCoverage =
                        FirstPersonSilhouettePredicateCoverage(
                            silhouette,
                            [&](int32_t x, int32_t y) {
                                const int32_t u =
                                    x - sourceOrigin.x
                                    - source.g1->xOffset
                                    + source.groundingShiftU;
                                const int32_t v =
                                    y - sourceOrigin.y
                                    - source.g1->yOffset
                                    + source.groundingShiftV;
                                if (u < 0 || v < 0
                                    || u >= source.g1->width
                                    || v >= source.g1->height)
                                    return false;
                                if (!source.g1->flags.has(
                                        G1Flag::hasTransparency))
                                    return true;
                                return source.pixels[
                                    size_t(v)
                                        * size_t(
                                            source.g1->width)
                                    + size_t(u)] != 0;
                            });
                    if (sourceCoverage <= 0.0f)
                        continue;

                    const bool reliable =
                        FirstPersonTextureReprojectionIsReliable(
                            ownership, sourceCoverage);
                    candidates.push_back({
                        &source,
                        ownership,
                        sourceCoverage,
                        (reliable ? 1000.0f : 0.0f)
                            + ownership * 4.0f
                            + sourceCoverage
                            + float(silhouette.size()) * 0.001f,
                    });
                }

                std::sort(
                    candidates.begin(), candidates.end(),
                    [](const SmallSceneryFaceView& a,
                       const SmallSceneryFaceView& b) {
                        if (a.score != b.score)
                            return a.score > b.score;
                        return a.source->rotation
                            < b.source->rotation;
                    });

                FirstPersonSurface surface{};
                // Physical occupancy and visible representation are deliberately
                // separate. Both reconstructed hulls and semantic tree planes
                // carry alpha-tested native artwork rather than guessed matter.
                surface.cameraIndependent = true;
                surface.outwardNormal = face.normal;
                surface.artworkCarrier =
                    appearance.kind
                        == FirstPersonSmallSceneryAppearanceKind::
                            crossedTree;
                surface.exteriorOnly =
                    appearance.oneSidedArtwork;

                if (!candidates.empty())
                {
                    surface.image =
                        candidates.front().source->image;
                    const int32_t width =
                        std::clamp(
                            int32_t(std::ceil(std::max(
                                edgeLength(
                                    face.corners[0],
                                    face.corners[1]),
                                edgeLength(
                                    face.corners[3],
                                    face.corners[2])))),
                            1, 256);
                    const int32_t height =
                        std::clamp(
                            int32_t(std::ceil(std::max(
                                edgeLength(
                                    face.corners[0],
                                    face.corners[3]),
                                edgeLength(
                                    face.corners[1],
                                    face.corners[2])))),
                            1, 256);

                    std::vector<uint8_t> pixels(
                        size_t(width) * size_t(height), 0);
                    size_t visiblePixelCount = 0;
                    size_t drawablePixelCount = 0;
                    for (int32_t y = 0; y < height; ++y)
                    for (int32_t x = 0; x < width; ++x)
                    {
                        FirstPersonVec3 point{};
                        if (!FirstPersonVisualHullSampleFace(
                                face,
                                (float(x) + 0.5f) / float(width),
                                (float(y) + 0.5f) / float(height),
                                point))
                            continue;
                        ++drawablePixelCount;

                        uint8_t selectedPixel = 0;
                        const size_t materialViewCount =
                            appearance.mergesMaterialViews
                            ? candidates.size()
                            : std::min<size_t>(
                                  1, candidates.size());
                        for (size_t materialView = 0;
                             materialView < materialViewCount;
                             ++materialView)
                        {
                            const auto& candidate =
                                candidates[materialView];
                            const auto& sourceView =
                                *candidate.source;
                            if (!SameFirstPersonMaterialTemplate(
                                    surface.image,
                                    sourceView.image))
                                continue;

                            int32_t u = 0;
                            int32_t v = 0;
                            if (appearance.kind
                                == FirstPersonSmallSceneryAppearanceKind::
                                    crossedTree)
                            {
                                const bool xCentred =
                                    std::abs(face.normal.x) > 0.5f;
                                const auto& mapping =
                                    xCentred
                                    ? sourceView.xCentredMapping
                                    : sourceView.yCentredMapping;
                                if (!mapping.valid
                                    || !treeFootprint.valid)
                                    continue;

                                const float along =
                                    xCentred
                                    ? point.y
                                        - treeFootprint.centreRight()
                                    : point.x
                                        - treeFootprint.centreForward();
                                const float up =
                                    point.z
                                    - treeFootprint.minUp;
                                const auto sourcePixel =
                                    mapping.sourcePixel(
                                        along, up);
                                u = int32_t(std::floor(
                                    sourcePixel[0]));
                                v = int32_t(std::floor(
                                    sourcePixel[1]));
                            }
                            else
                            {
                                const auto projected =
                                    Translate3DTo2DWithZ(
                                        sourceView.rotation,
                                        {
                                            int32_t(std::lround(
                                                point.x)),
                                            int32_t(std::lround(
                                                point.y)),
                                            int32_t(std::lround(
                                                point.z)),
                                        });
                                if (!ownedNear(
                                        sourceView.rotation,
                                        uint32_t(faceIndex),
                                        projected.x,
                                        projected.y))
                                    continue;

                                const auto paintOffset =
                                    FirstPersonSmallSceneryPaintOffset(
                                        *entry, *small,
                                        sourceView.rotation);
                                const auto sourceOrigin =
                                    Translate3DTo2DWithZ(
                                        sourceView.rotation,
                                        { paintOffset, 0 });
                                u =
                                    projected.x - sourceOrigin.x
                                    - sourceView.g1->xOffset
                                    + sourceView.groundingShiftU;
                                v =
                                    projected.y - sourceOrigin.y
                                    - sourceView.g1->yOffset
                                    + sourceView.groundingShiftV;
                            }
                            if (u < 0 || v < 0
                                || u >= sourceView.g1->width
                                || v >= sourceView.g1->height)
                                continue;

                            const uint8_t pixel =
                                sourceView.pixels[
                                    size_t(v)
                                        * size_t(
                                            sourceView.g1->width)
                                    + size_t(u)];
                            if (pixel == 0)
                                continue;
                            selectedPixel = pixel;
                            break;
                        }

                        // Zero is meaningful visual evidence: the native
                        // sprite has no material at this projected point.
                        // Preserve it instead of painting the hull/collision
                        // volume with a guessed dominant colour.
                        pixels[
                            size_t(y) * size_t(width)
                            + size_t(x)] =
                            selectedPixel;
                        if (selectedPixel != 0)
                            ++visiblePixelCount;
                    }

                    if (visiblePixelCount == 0)
                        continue;

                    if (appearance.certifiesHullMaterial)
                        materialCoverage.recordFace(
                            faceIndex, drawablePixelCount,
                            visiblePixelCount);

                    uint64_t fingerprint =
                        14695981039346656037ull;
                    ExtendStableKey(fingerprint, width);
                    ExtendStableKey(fingerprint, height);
                    for (const auto pixel : pixels)
                    {
                        fingerprint ^= pixel;
                        fingerprint *= 1099511628211ull;
                    }
                    surface.persistentPixels =
                        std::make_shared<
                            std::vector<uint8_t>>(
                                std::move(pixels));
                    surface.immutableWidth =
                        int16_t(width);
                    surface.immutableHeight =
                        int16_t(height);
                    surface.persistentBitmap = true;
                    surface.immutableFingerprint =
                        fingerprint;

                    const std::array<
                        FirstPersonVertex, 4> vertices{ {
                        {
                            { tileX + face.corners[0].x,
                              tileY + face.corners[0].y,
                              baseZ + face.corners[0].z },
                            0.0f, 0.0f,
                        },
                        {
                            { tileX + face.corners[1].x,
                              tileY + face.corners[1].y,
                              baseZ + face.corners[1].z },
                            float(width), 0.0f,
                        },
                        {
                            { tileX + face.corners[2].x,
                              tileY + face.corners[2].y,
                              baseZ + face.corners[2].z },
                            float(width), float(height),
                        },
                        {
                            { tileX + face.corners[3].x,
                              tileY + face.corners[3].y,
                              baseZ + face.corners[3].z },
                            0.0f, float(height),
                        },
                    } };
                    EmitQuad(surface, vertices);
                    scene.surfaces.emplace_back(
                        std::move(surface));
                    ++emittedFaces;
                    continue;
                }

                // Native sprites normally provide no underside observation.
                // An unobserved bottom face may exist physically for collision,
                // but inventing a flat material for it is not visual evidence.
                if (face.kind
                    == FirstPersonVisualHullFaceKind::bottom)
                    continue;

                // A normally visible face with no native-view material means
                // reconstruction is incomplete. Roll back this object's partial
                // mesh and preserve the ordinary native sprite fallback rather
                // than showing guessed solid-colour geometry.
                scene.surfaces.resize(surfaceStart);
                baked.localSurfaces.clear();
                baked.resolved = true;
                baked.usable = false;
                return false;
            }

            if (appearance.certifiesHullMaterial)
            {
                materialCoverage.applyTo(
                    scene.surfaces.begin() + surfaceStart,
                    scene.surfaces.end());
            }
            if (hull != nullptr)
            {
                // Keep the source and selected geometry evidence attached to
                // the actual picked surface. A silhouette-rejected object
                // that fell back to the declared occupancy envelope has no
                // round-trip views; a carved one carries its certificate.
                for (size_t i = surfaceStart;
                     i < scene.surfaces.size(); ++i)
                {
                    auto& surface = scene.surfaces[i];
                    surface.diagnosticReconstructionKind = 4;
                    surface.diagnosticContinuousSurfaceRefined =
                        hull->continuousSurfaceRefined;
                    surface.diagnosticVoxelRoundTripPixelDisagreement =
                        uint64_t(hull->voxelRoundTripSymmetricDifference);
                    surface.diagnosticRoundTripPixelDisagreement =
                        uint64_t(hull->roundTripSymmetricDifference);
                    surface.diagnosticRoundTripViewCount =
                        uint8_t(std::min<size_t>(
                            hull->roundTripViewCount, 4));
                    const auto& attempt = hull->attempt;
                    surface.diagnosticHullAttemptStage = attempt.stage;
                    surface.diagnosticHullAttemptViews = attempt.nativeViews;
                    surface.diagnosticHullAttemptCells = attempt.occupiedCells;
                    surface.diagnosticHullAttemptFaces = attempt.voxelFaces;
                    surface.diagnosticHullAttemptCertifiedViews =
                        attempt.certificateViews;
                    surface.diagnosticHullAttemptIoU =
                        attempt.minimumIoU;
                    surface.diagnosticHullAttemptCandidateCoverage =
                        attempt.candidateCoverage;
                    surface.diagnosticHullAttemptObservedCoverage =
                        attempt.observedCoverage;
                    surface.diagnosticHullAttemptEdgeError =
                        attempt.maximumEdgeError;
                    surface.diagnosticHullAttemptDisagreement =
                        attempt.disagreementPixels;
                }
            }
            baked.localSurfaces.clear();
            if (emittedFaces != 0)
            {
                baked.localSurfaces.reserve(
                    scene.surfaces.size()
                    - surfaceStart);
                for (size_t i = surfaceStart;
                     i < scene.surfaces.size(); ++i)
                {
                    auto local =
                        scene.surfaces[i];
                    for (auto& vertex :
                         local.triangles)
                    {
                        vertex.world.x -= tileX;
                        vertex.world.y -= tileY;
                        vertex.world.z -= baseZ;
                    }
                    baked.localSurfaces.emplace_back(
                        std::move(local));
                }
            }
            baked.resolved = true;
            baked.usable = emittedFaces != 0;
            return baked.usable;
        }

        struct SmallSceneryJob
        {
            SmallSceneryEntry entry{};
            SmallSceneryElement element{};
            FirstPersonSpriteSnapshot sprites;
            std::vector<FirstPersonSurface> surfaces;
            std::unordered_set<uint64_t> waitingTiles;
            bool submitted = false;
            bool ready = false;
        };
        static std::unordered_map<uint64_t, std::shared_ptr<SmallSceneryJob>> _smallSceneryJobs;

        bool AppendCalibratedSmallSceneryGeometry(
            FirstPersonScene& scene, const PaintStruct& ps, ImageId image,
            const ScreenCoordsXY&, uint8_t rotation, ImageId mask = {})
        {
            const auto* small = ps.Element ? ps.Element->asSmallScenery() : nullptr;
            if (small == nullptr || !image.HasValue() || mask.HasValue()) return false;
            const auto* entry = small->getEntry();
            if (entry == nullptr) return false;
            const auto stage = FirstPersonSmallSceneryWitherStage(*entry, *small);
            if (image.GetIndex() != entry->image + (small->getDirectionWithOffset(rotation) & 3u)
                    + uint32_t(stage) * 4u) return false;
            uint64_t key = entry->image;
            ExtendStableKey(key, small->getDirection());
            ExtendStableKey(key, small->getSceneryQuadrant());
            ExtendStableKey(key, small->getOccupiedQuadrants());
            ExtendStableKey(key, stage);
            auto it = _smallSceneryJobs.find(key);
            if (it == _smallSceneryJobs.end())
            {
                if (!_snapshotBudget.available() || !_reconstructionWorker.hasCapacity())
                {
                    // Revisit this native view next frame; never fall through
                    // to a synchronous silhouette reconstruction.
                    InvalidateFirstPersonSceneTile(ps.MapPos);
                    return true;
                }
                auto job = std::make_shared<SmallSceneryJob>();
                job->entry = *entry;
                job->entry.frame_offsets = nullptr;
                job->entry.FrameOffsetCount = 0;
                job->element = *small;
                it = _smallSceneryJobs.emplace(key, std::move(job)).first;
            }
            const auto job = it->second;
            if (job->ready)
            {
                if (job->surfaces.empty()) return false;
                for (const auto& local : job->surfaces)
                {
                    auto surface = local;
                    surface.image = image.WithIndex(local.image.GetIndex());
                    for (auto& vertex : surface.triangles)
                    {
                        vertex.world.x += float(ps.MapPos.x);
                        vertex.world.y += float(ps.MapPos.y);
                        vertex.world.z += float(small->getBaseZ());
                    }
                    scene.surfaces.emplace_back(std::move(surface));
                }
                return true;
            }
            const auto tileKey = (uint64_t(uint32_t(ps.MapPos.x)) << 32) | uint32_t(ps.MapPos.y);
            job->waitingTiles.insert(tileKey);
            if (!job->submitted)
            {
                for (uint8_t view = 0; view < 4; ++view)
                    if (!job->sprites.capture(entry->image + view + uint32_t(stage) * 4u, _snapshotBudget))
                    {
                        InvalidateFirstPersonSceneTile(ps.MapPos);
                        return true;
                    }
                job->submitted = _reconstructionWorker.submit([job] {
                    FirstPersonScene result;
                    BuildSmallSceneryGeometry(result, job->element, job->entry,
                        ImageId(job->entry.image), job->sprites);
                    return [job, surfaces = std::move(result.surfaces)]() mutable {
                        job->surfaces = std::move(surfaces);
                        job->ready = true;
                        for (const auto tile : job->waitingTiles)
                            InvalidateFirstPersonSceneTile({int32_t(tile >> 32), int32_t(tile)});
                        job->waitingTiles.clear();
                    };
                }, [job] {
                    job->ready = true;
                    for (const auto tile : job->waitingTiles)
                        InvalidateFirstPersonSceneTile({int32_t(tile >> 32), int32_t(tile)});
                    job->waitingTiles.clear();
                });
                if (!job->submitted) InvalidateFirstPersonSceneTile(ps.MapPos);
            }
            return true; // pending geometry deliberately absent
        }

