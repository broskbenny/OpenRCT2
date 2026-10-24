// Included inside BuildFirstPersonSourceTriangulatedWireFaces after rod
// selection and face-budget calculation. The enclosing function supplies:
// result, faceLimit, views, native, projectPoint, and projectedUses.
//
// Unlike a voxel or a camera-facing card, each patch is a real narrow 3D
// surface on a source-supported plane. Find contiguous *vertical* one-unit
// spans using all four original silhouettes, and validate the ACTUAL raster
// of every proposed quad before it competes for a limited face budget.
// This supplements straight members with irregular art-supported sheets;
// no individual DAT name, geometry template, or palette index is used.
if (includeSourceSupportedPatches && result.size() + 12 < faceLimit)
{
    struct SourcePatch
    {
        FirstPersonVisualHullFace face{};
        std::array<std::vector<uint64_t>, 4> projected{};
        int32_t initialGain{};
    };
    std::vector<SourcePatch> patches;
    constexpr size_t kMaximumSourcePatchPool = 6000;
    constexpr size_t kMaximumSourcePatchCandidates = 1200;
    const float minX = hull.minForward;
    const float maxX = minX + float(hull.sizeForward) * hull.step;
    const float minY = hull.minRight;
    const float maxY = minY + float(hull.sizeRight) * hull.step;
    const int32_t minZ = int32_t(std::ceil(hull.minUp));
    const int32_t maxZ = int32_t(std::floor(
        hull.minUp + float(hull.sizeUp) * hull.step));

    const auto planePoint = [](int family, int32_t planeCoordinate,
                               float along, float up) {
        switch (family)
        {
            case 0: return FirstPersonVec3{float(planeCoordinate), along, up};
            case 1: return FirstPersonVec3{along, float(planeCoordinate), up};
            case 2: return FirstPersonVec3{along, float(planeCoordinate) - along, up};
            default: return FirstPersonVec3{along, along - float(planeCoordinate), up};
        }
    };
    const auto withinXY = [&](FirstPersonVec3 p) {
        constexpr float margin = .001f;
        return p.x >= minX - margin && p.x <= maxX + margin
            && p.y >= minY - margin && p.y <= maxY + margin;
    };
    const auto supportedPoint = [&](FirstPersonVec3 p) {
        for (uint8_t d = 0; d < 4; ++d)
        {
            const auto uv = projectPoint(d, p);
            if (!native[d]->observed.contains(
                    int32_t(std::lround(uv[0])),
                    int32_t(std::lround(uv[1]))))
                return false;
        }
        return true;
    };

    const auto considerFace = [&](FirstPersonVisualHullFace face)
    {
        if (patches.size() >= kMaximumSourcePatchPool)
            return;
        for (const auto& v : face.corners)
            if (!withinXY(v))
                return;
        face.normal = unit(FpCross(
            sub(face.corners[1], face.corners[0]),
            sub(face.corners[2], face.corners[0])));
        if (FpDot(face.normal, face.normal) < .5f)
            return;
        face.kind = face.normal.z > .7f ? FirstPersonVisualHullFaceKind::top
            : face.normal.z < -.7f ? FirstPersonVisualHullFaceKind::bottom
            : std::abs(face.normal.x) > std::abs(face.normal.y)
                ? (face.normal.x < 0 ? FirstPersonVisualHullFaceKind::minForward
                                     : FirstPersonVisualHullFaceKind::maxForward)
                : (face.normal.y < 0 ? FirstPersonVisualHullFaceKind::minRight
                                     : FirstPersonVisualHullFaceKind::maxRight);

        SourcePatch candidate{};
        candidate.face = face;
        int32_t totalGain = 0;
        size_t witnessed = 0;
        for (uint8_t d = 0; d < 4; ++d)
        {
            std::array<ScreenCoordsXY, 4> screen{};
            for (size_t k = 0; k < 4; ++k)
            {
                const auto uv = projectPoint(d, face.corners[k]);
                screen[k] = {
                    int32_t(std::lround(uv[0])),
                    int32_t(std::lround(uv[1]))
                };
            }
            FirstPersonSilhouette silhouette{};
            AddFirstPersonSilhouetteQuad(silhouette, screen);
            if (silhouette.overflowed)
                return;
            if (!silhouette.empty())
                ++witnessed;
            int32_t viewGain = 0;
            auto& pixels = candidate.projected[d];
            pixels.reserve(silhouette.size());
            for (uint64_t pixel : silhouette.pixels)
            {
                pixels.push_back(pixel);
                if (projectedUses[d].contains(pixel))
                    continue;
                viewGain += native[d]->observed.pixels.contains(pixel)
                    ? 1 : -1;
            }
            // One source view may be edge-on to the patch. A visible patch
            // is never allowed to worsen ANY source silhouette.
            if (viewGain < 0)
                return;
            totalGain += viewGain;
        }
        if (witnessed < 2 || totalGain <= 0)
            return;
        candidate.initialGain = totalGain;
        patches.push_back(std::move(candidate));
    };
    const auto considerPatch = [&](int family, int32_t planeCoordinate,
                                   int32_t along, int32_t startZ,
                                   int32_t endZ)
    {
        if (endZ - startZ < 2)
            return;
        FirstPersonVisualHullFace face{};
        face.corners = {{
            planePoint(family, planeCoordinate, float(along), float(startZ)),
            planePoint(family, planeCoordinate, float(along + 1), float(startZ)),
            planePoint(family, planeCoordinate, float(along + 1), float(endZ)),
            planePoint(family, planeCoordinate, float(along), float(endZ))
        }};
        considerFace(face);
    };

    // Four generic vertical plane families, including two diagonals.
    // Every plane is clipped to the physical tile bounds. Samples are
    // screened against all four source images before face rasterization.
    const int32_t firstAlong = int32_t(std::ceil(minX));
    const int32_t lastAlong = int32_t(std::floor(maxX));
    for (int family = 0; family < 4; ++family)
    {
        int32_t firstPlane = 0;
        int32_t lastPlane = 0;
        if (family == 0)
        {
            firstPlane = int32_t(std::ceil(minX));
            lastPlane = int32_t(std::floor(maxX));
        }
        else if (family == 1)
        {
            firstPlane = int32_t(std::ceil(minY));
            lastPlane = int32_t(std::floor(maxY));
        }
        else if (family == 2)
        {
            firstPlane = int32_t(std::ceil(minX + minY));
            lastPlane = int32_t(std::floor(maxX + maxY));
        }
        else
        {
            firstPlane = int32_t(std::ceil(minX - maxY));
            lastPlane = int32_t(std::floor(maxX - minY));
        }
        for (int32_t planeCoordinate = firstPlane; planeCoordinate <= lastPlane; ++planeCoordinate)
        {
            for (int32_t along = firstAlong; along < lastAlong; ++along)
            {
                int32_t runStart = -1;
                for (int32_t z = minZ; z <= maxZ; ++z)
                {
                    const auto sample = planePoint(
                        family, planeCoordinate, float(along) + .5f,
                        float(z) + .5f);
                    const bool supported = z < maxZ
                        && withinXY(sample) && supportedPoint(sample);
                    if (supported && runStart < 0)
                        runStart = z;
                    if (!supported && runStart >= 0)
                    {
                        considerPatch(family, planeCoordinate, along, runStart, z);
                        runStart = -1;
                    }
                }
            }
        }
    }

    // Source-observed long horizontal ornament strips: flat lintels, caps
    // and ornament ledges are not reproducible as only vertical panels or
    // straight Hough beams. Search for continuous two-unit or longer spans
    // at integer heights, with one world-unit lateral thickness. As with
    // vertical sheets, each actual quad is independently raster-checked in
    // all four native source views; ground/bottom faces are not invented.
    for (int family = 0; family < 2; ++family)
    {
        for (int32_t height = minZ + 1; height <= maxZ; ++height)
        {
            for (int32_t fixedCoordinate = firstAlong;
                 fixedCoordinate < lastAlong; ++fixedCoordinate)
            {
                int32_t runStart = -1;
                for (int32_t along = firstAlong;
                     along <= lastAlong; ++along)
                {
                    const FirstPersonVec3 sample = family == 0
                        ? FirstPersonVec3{float(along) + .5f,
                            float(fixedCoordinate) + .5f,float(height)}
                        : FirstPersonVec3{float(fixedCoordinate) + .5f,
                            float(along) + .5f,float(height)};
                    const bool supported = along < lastAlong
                        && withinXY(sample) && supportedPoint(sample);
                    if (supported && runStart < 0)
                        runStart = along;
                    if (!supported && runStart >= 0)
                    {
                        if (along - runStart >= 2)
                        {
                            FirstPersonVisualHullFace face{};
                            const float low = float(runStart);
                            const float high = float(along);
                            const float fixed = float(fixedCoordinate);
                            const float up = float(height);
                            if (family == 0)
                                face.corners = {{
                                    {low, fixed, up},
                                    {high, fixed, up},
                                    {high, fixed + 1.0f, up},
                                    {low, fixed + 1.0f, up}
                                }};
                            else
                                face.corners = {{
                                    {fixed, low, up},
                                    {fixed + 1.0f, low, up},
                                    {fixed + 1.0f, high, up},
                                    {fixed, high, up}
                                }};
                            considerFace(face);
                        }
                        runStart = -1;
                    }
                }
            }
        }
    }

    // Diagonal horizontal lintels and cross-members belong to the same
    // native source surface family as axis-aligned ledges. Restrict them to
    // two simple x+y and x-y planes, in a top-facing orientation, using
    // the identical four-view pixel support and face-budget certificate.
    // Restricting strips to length >= 2 avoids a spray of voxel-like 1x1
    // decorative tessellation.
    for (int family = 0; family < 2; ++family)
    {
        const int32_t firstPlane = family == 0
            ? int32_t(std::ceil(minX + minY))
            : int32_t(std::ceil(minX - maxY));
        const int32_t lastPlane = family == 0
            ? int32_t(std::floor(maxX + maxY))
            : int32_t(std::floor(maxX - minY));
        for (int32_t level = minZ + 1; level <= maxZ; ++level)
        {
            for (int32_t planeCoordinate = firstPlane;
                 planeCoordinate <= lastPlane; ++planeCoordinate)
            {
                int32_t runStart = -1;
                for (int32_t along = firstAlong;
                     along <= lastAlong; ++along)
                {
                    const float alongY = family == 0
                        ? float(planeCoordinate - along)
                        : float(along - planeCoordinate);
                    const FirstPersonVec3 sample{
                        float(along) + .5f, alongY + .5f,
                        float(level)
                    };
                    const bool supported = along < lastAlong
                        && withinXY(sample) && supportedPoint(sample);
                    if (supported && runStart < 0)
                        runStart = along;
                    if (!supported && runStart >= 0)
                    {
                        if (along - runStart >= 2)
                        {
                            const auto diagonalPoint = [&](
                                int32_t x, float lateral)
                            {
                                const float y = family == 0
                                    ? float(planeCoordinate - x)
                                    : float(x - planeCoordinate);
                                return FirstPersonVec3{
                                    float(x), y + lateral,
                                    float(level)
                                };
                            };
                            FirstPersonVisualHullFace face{};
                            face.corners = {{
                                diagonalPoint(runStart, 0.0f),
                                diagonalPoint(along, 0.0f),
                                diagonalPoint(along, 1.0f),
                                diagonalPoint(runStart, 1.0f)
                            }};
                            considerFace(face);
                        }
                        runStart = -1;
                    }
                }
            }
        }
    }

    if (!patches.empty())
    {
        std::stable_sort(patches.begin(), patches.end(),
            [](const SourcePatch& a, const SourcePatch& b) {
                return a.initialGain > b.initialGain;
            });
        if (patches.size() > kMaximumSourcePatchCandidates)
            patches.resize(kMaximumSourcePatchCandidates);

        const size_t patchBudget = std::min<size_t>(
            240, faceLimit - result.size());
        std::vector<uint8_t> used(patches.size(), 0);

        // Greedy total pixel count was biased towards already well-explained
        // source views. A patch that repairs the weakest authored rotation
        // matters more than the same number of pixels in a stronger view.
        // Keep the exact same per-view nonnegative source-support rule; only
        // the selection priority changes. Update projected counts alongside
        // the actual source-projection union to avoid re-rasterizing rods.
        std::array<size_t,4> predictedCounts{};
        std::array<size_t,4> observedMatches{};
        for (uint8_t d = 0; d < 4; ++d)
        {
            for (const auto& [pixel,uses] : projectedUses[d])
            {
                if (uses == 0)
                    continue;
                ++predictedCounts[d];
                if (native[d]->observed.pixels.contains(pixel))
                    ++observedMatches[d];
            }
        }

        // Exact incremental gain cache. After a source pixel becomes part
        // of the selected union, its contribution changes exactly once.
        // Rescanning all four rasters of all 1200 candidates after each
        // selection performed millions of redundant lookups. The reverse
        // pixel-to-candidate dependency index preserves the previous
        // greedy result, including candidates whose negative support is
        // subsequently covered by a different approved surface.
        std::vector<std::array<int32_t,4>> remainingGain(patches.size());
        std::array<std::unordered_map<uint64_t,std::vector<uint16_t>>,4>
            dependentCandidates{};
        for (size_t i = 0; i < patches.size(); ++i)
        for (uint8_t d = 0; d < 4; ++d)
        {
            for (uint64_t pixel : patches[i].projected[d])
            {
                if (projectedUses[d].contains(pixel))
                    continue;
                remainingGain[i][d] +=
                    native[d]->observed.pixels.contains(pixel) ? 1 : -1;
                // Candidate indices cannot exceed the 1200-patch bound.
                dependentCandidates[d][pixel].push_back(uint16_t(i));
            }
        }
        for (size_t selected = 0; selected < patchBudget; ++selected)
        {
            std::array<float,4> sourceIoU{};
            float strongestIoU = 0.0f;
            for (uint8_t d = 0; d < 4; ++d)
            {
                const size_t unionPixels = native[d]->observed.size()
                    + predictedCounts[d] - observedMatches[d];
                sourceIoU[d] = unionPixels == 0 ? 0.0f
                    : float(observedMatches[d]) / float(unionPixels);
                strongestIoU = std::max(strongestIoU,sourceIoU[d]);
            }
            // The source-only admission gate judges the WEAKEST view.
            // Small per-view weights keep all source-covered pixels useful
            // while preferentially recovering the least complete view.
            std::array<float,4> viewWeights{};
            for (uint8_t d = 0; d < 4; ++d)
                viewWeights[d] = 1.0f
                    + 8.0f * (strongestIoU - sourceIoU[d]);

            size_t best = patches.size();
            float bestGain = 0.0f;
            for (size_t i = 0; i < patches.size(); ++i)
            {
                if (used[i])
                    continue;
                float gain = 0.0f;
                bool admissible = true;
                for (uint8_t d = 0; d < 4 && admissible; ++d)
                {
                    const int32_t viewGain = remainingGain[i][d];
                    if (viewGain < 0)
                        admissible = false;
                    gain += float(viewGain) * viewWeights[d];
                }
                if (admissible && gain > bestGain)
                {
                    bestGain = gain;
                    best = i;
                }
            }
            if (best == patches.size())
                break;
            used[best] = 1;
            result.push_back(patches[best].face);
            for (uint8_t d = 0; d < 4; ++d)
                for (uint64_t pixel : patches[best].projected[d])
                {
                    if (!projectedUses[d].contains(pixel))
                    {
                        ++predictedCounts[d];
                        const bool nativePixel =
                            native[d]->observed.pixels.contains(pixel);
                        if (nativePixel)
                            ++observedMatches[d];
                        const int32_t contribution = nativePixel ? 1 : -1;
                        const auto affected = dependentCandidates[d].find(pixel);
                        if (affected != dependentCandidates[d].end())
                            for (uint16_t index : affected->second)
                                remainingGain[index][d] -= contribution;
                    }
                    ++projectedUses[d][pixel];
                }
        }
    }
}
