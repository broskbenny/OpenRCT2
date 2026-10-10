        struct SmallSceneryBuildEvidence
        {
            uint32_t hullFaces = 0;
            uint32_t materialFaces = 0;
            uint32_t withoutSourceMaterial = 0;
            uint32_t withoutOpaqueSamples = 0;
            uint32_t unobservedUndersides = 0;
        };

        bool BuildSmallSceneryGeometry(FirstPersonScene& scene,
            const SmallSceneryElement& element, const SmallSceneryEntry& metadata,
            ImageId image, const FirstPersonSpriteSnapshot& sprites,
            FirstPersonVisualHullAttempt* reconstructionAttempt = nullptr,
            SmallSceneryBuildEvidence* buildEvidence = nullptr)
        {
            if (buildEvidence != nullptr)
                *buildEvidence = {};
            const auto* small = &element;
            const auto* entry = &metadata;
            FirstPersonVisualHull ownedHull{};
            const FirstPersonVisualHull* hull = nullptr;
            if (!FirstPersonSmallSceneryUsesCrossedTreeAppearance(metadata))
            {
                ownedHull = BuildFirstPersonSmallSceneryVisualHullFromSnapshot(
                    metadata, element,
                    [&](ImageIndex id) { return sprites.get(id); });
                if (reconstructionAttempt != nullptr)
                    *reconstructionAttempt = ownedHull.attempt;
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
            auto faces = appearance.faces;
            if (buildEvidence != nullptr)
                buildEvidence->hullFaces = uint32_t(faces.size());
            const auto treeFootprint =
                appearance.kind
                    == FirstPersonSmallSceneryAppearanceKind::crossedTree
                ? BuildFirstPersonSmallSceneryFootprint(
                    *entry, *small)
                : FirstPersonSmallSceneryFootprint{};
            if (faces.empty())
            {
                return false;
            }

            const auto faceVisible =
                [](FirstPersonVec3 normal,
                    uint8_t sourceRotation) {
                    return FirstPersonSurfaceVisibleFromNativeView(
                        normal, sourceRotation);
                };

            // Visibility is computed independently of texture fit. A projective
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
                            FirstPersonSmallSceneryArtworkPoint(
                                *entry, sourceRotation, point);
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
                float visibility = 0.0f;
                float sourceCoverage = 0.0f;
                float score = 0.0f;
                FirstPersonSilhouette visible;
            };

            const auto edgeLength =
                [](FirstPersonVec3 a, FirstPersonVec3 b) {
                    const float dx = b.x - a.x;
                    const float dy = b.y - a.y;
                    const float dz = b.z - a.z;
                    return std::sqrt(
                        dx * dx + dy * dy + dz * dz);
                };
            const auto visibleNear =
                [&](const FirstPersonSilhouette& visible,
                    int32_t x, int32_t y) {
                    if (!appearance.usesDepthOwnership)
                        return true;
                    for (int32_t dy = -1; dy <= 1; ++dy)
                    for (int32_t dx = -1; dx <= 1; ++dx)
                    {
                        if (visible.contains(x + dx, y + dy))
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
                    std::array<float, 4> depth{};
                    for (size_t i = 0;
                         i < face.corners.size(); ++i)
                    {
                        const CoordsXYZ point{
                            int32_t(std::lround(face.corners[i].x)),
                            int32_t(std::lround(face.corners[i].y)),
                            int32_t(std::lround(face.corners[i].z)),
                        };
                        projected[i] =
                            FirstPersonSmallSceneryArtworkPoint(
                                *entry, source.rotation, point);
                        depth[i] = FirstPersonIsoDepth(source.rotation, point);
                    }
                    FirstPersonSilhouette silhouette{};
                    FirstPersonSilhouette visible{};
                    float visibilityCoverage = 1.0f;
                    if (appearance.usesDepthOwnership)
                    {
                        auto visibility = BuildFirstPersonDepthVisibility(
                            depthOwners[source.rotation], projected, depth);
                        visibilityCoverage = visibility.coverage();
                        if (visibilityCoverage < 0.20f)
                            continue;
                        silhouette = std::move(visibility.projected);
                        visible = std::move(visibility.visible);
                    }
                    else
                        AddFirstPersonSilhouetteQuad(silhouette, projected);

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
                            visibilityCoverage, sourceCoverage);
                    candidates.push_back({
                        &source,
                        visibilityCoverage,
                        sourceCoverage,
                        (reliable ? 1000.0f : 0.0f)
                            + visibilityCoverage * 4.0f
                            + sourceCoverage
                            + float(silhouette.size()) * 0.001f,
                        std::move(visible),
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
                                    FirstPersonSmallSceneryArtworkPoint(
                                        *entry, sourceView.rotation,
                                        {
                                            int32_t(std::lround(
                                                point.x)),
                                            int32_t(std::lround(
                                                point.y)),
                                            int32_t(std::lround(
                                                point.z)),
                                        });
                                if (!visibleNear(
                                        candidate.visible,
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
                    {
                        if (buildEvidence != nullptr)
                            ++buildEvidence->withoutOpaqueSamples;
                        continue;
                    }

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
                {
                    if (buildEvidence != nullptr)
                        ++buildEvidence->unobservedUndersides;
                    continue;
                }

                // A face with no trustworthy native source material has no
                // presentable texels. It must not be invented, but it must
                // not also invalidate OTHER faces with independent material
                // evidence. Voxel hulls routinely contain tiny boundary
                // facets hidden in every native view. The previous rollback
                // silently discarded even fully certified visible patches,
                // leaving an unrelated camera-facing sprite as the only
                // presentation. Omit this unsupported face and preserve
                // evidence-backed reconstructed surfaces.
                if (buildEvidence != nullptr)
                    ++buildEvidence->withoutSourceMaterial;
                continue;
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
                    surface.diagnosticHullRegistrationAttempted =
                        attempt.registrationAttempted;
                    surface.diagnosticHullRegistrationPixels =
                        attempt.registrationPixels;
                }
            }
            if (buildEvidence != nullptr)
                buildEvidence->materialFaces = uint32_t(emittedFaces);
            return emittedFaces != 0;
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
        // A reconstruction outcome belongs to the asset, not every tile
        // instance or camera rotation. Report it at most once per native
        // image base and failure phase.
        static std::unordered_set<uint64_t> _smallSceneryReportedOutcomes;

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
                    FirstPersonVisualHullAttempt attempt{};
                    SmallSceneryBuildEvidence buildEvidence{};
                    BuildSmallSceneryGeometry(result, job->element, job->entry,
                        ImageId(job->entry.image), job->sprites,
                        &attempt, &buildEvidence);
                    // Only baked surfaces are needed after this worker finishes.
                    // Release copied source sprites here, off the game thread.
                    job->sprites = {};
                    return [job, surfaces = std::move(result.surfaces),
                            attempt, buildEvidence]() mutable {
                        job->surfaces = std::move(surfaces);
                        job->ready = true;
                        const uint32_t unsupportedFaces =
                            buildEvidence.withoutSourceMaterial
                            + buildEvidence.withoutOpaqueSamples;
                        // The old failure log reported only stage 5 and
                        // observed coverage below 20%, hiding all other
                        // failure modes, including the complete loss of a
                        // certified 3D mesh during texture assignment.
                        // Emit once per asset/reconstruction phase, not once
                        // per instance or camera repaint.
                        if (job->surfaces.empty()
                            || (unsupportedFaces > 8
                                && unsupportedFaces
                                    > buildEvidence.materialFaces / 2))
                        {
                            uint64_t reportKey = job->entry.image;
                            ExtendStableKey(
                                reportKey, uint64_t(attempt.stage));
                            ExtendStableKey(
                                reportKey, job->surfaces.empty() ? 0u : 1u);
                            if (_smallSceneryReportedOutcomes.insert(
                                    reportKey).second)
                            {
                                Console::WriteLine(
                                    "FP_SMALL_SCENERY_EVIDENCE image=%u"
                                    " output=%s stage=%u inputViews=%u"
                                    " supportedCells=%u voxelFaces=%u"
                                    " hullFaces=%u materialFaces=%u"
                                    " missingSourceFaces=%u emptyTextureFaces=%u"
                                    " unobservedBottomFaces=%u"
                                    " silhouetteIoU=%.3f"
                                    " observedCoverage=%.3f"
                                    " candidateCoverage=%.3f edge=%d"
                                    " registrationTried=%d",
                                    job->entry.image,
                                    job->surfaces.empty()
                                        ? "native-fallback"
                                        : "partial-3d",
                                    unsigned(attempt.stage),
                                    unsigned(attempt.nativeViews),
                                    attempt.occupiedCells,
                                    attempt.voxelFaces,
                                    buildEvidence.hullFaces,
                                    buildEvidence.materialFaces,
                                    buildEvidence.withoutSourceMaterial,
                                    buildEvidence.withoutOpaqueSamples,
                                    buildEvidence.unobservedUndersides,
                                    double(attempt.minimumIoU),
                                    double(attempt.observedCoverage),
                                    double(attempt.candidateCoverage),
                                    attempt.maximumEdgeError,
                                    attempt.registrationAttempted ? 1 : 0);
                            }
                        }
                        for (const auto tile : job->waitingTiles)
                            InvalidateFirstPersonSceneTile({int32_t(tile >> 32), int32_t(tile)});
                        job->waitingTiles.clear();
                    };
                }, [job] {
                    // FirstPersonWorkQueue invokes this continuation only
                    // when its worker body throws. Never label an exception
                    // as a normal four-view geometric rejection: retain one
                    // clear asset-level diagnostic instead of a silent sprite.
                    job->sprites = {};
                    uint64_t reportKey = job->entry.image;
                    ExtendStableKey(reportKey, 255u);
                    if (_smallSceneryReportedOutcomes.insert(
                            reportKey).second)
                    {
                        Console::WriteLine(
                            "FP_SMALL_SCENERY_EVIDENCE image=%u"
                            " output=native-fallback stage=worker-exception",
                            job->entry.image);
                    }
                    job->ready = true;
                    for (const auto tile : job->waitingTiles)
                        InvalidateFirstPersonSceneTile({int32_t(tile >> 32), int32_t(tile)});
                    job->waitingTiles.clear();
                });
                if (!job->submitted) InvalidateFirstPersonSceneTile(ps.MapPos);
            }
            return true; // pending geometry deliberately absent
        }

