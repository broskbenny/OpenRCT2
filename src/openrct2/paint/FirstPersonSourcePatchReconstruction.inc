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

    const auto considerPatch = [&](int family, int32_t planeCoordinate,
                                   int32_t along, int32_t startZ,
                                   int32_t endZ)
    {
        if (endZ - startZ < 2 || patches.size() >= kMaximumSourcePatchPool)
            return;
        FirstPersonVisualHullFace face{};
        face.corners = {{
            planePoint(family, planeCoordinate, float(along), float(startZ)),
            planePoint(family, planeCoordinate, float(along + 1), float(startZ)),
            planePoint(family, planeCoordinate, float(along + 1), float(endZ)),
            planePoint(family, planeCoordinate, float(along), float(endZ))
        }};
        for (const auto& v : face.corners)
            if (!withinXY(v))
                return;
        face.normal = unit(FpCross(
            sub(face.corners[1], face.corners[0]),
            sub(face.corners[2], face.corners[0])));
        if (FpDot(face.normal, face.normal) < .5f)
            return;
        face.kind = std::abs(face.normal.x) > std::abs(face.normal.y)
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
        for (size_t selected = 0; selected < patchBudget; ++selected)
        {
            size_t best = patches.size();
            int32_t bestGain = 0;
            for (size_t i = 0; i < patches.size(); ++i)
            {
                if (used[i])
                    continue;
                int32_t gain = 0;
                bool admissible = true;
                for (uint8_t d = 0; d < 4 && admissible; ++d)
                {
                    int32_t viewGain = 0;
                    for (uint64_t pixel : patches[i].projected[d])
                    {
                        if (projectedUses[d].contains(pixel))
                            continue;
                        viewGain += native[d]->observed.pixels.contains(pixel)
                            ? 1 : -1;
                    }
                    if (viewGain < 0)
                        admissible = false;
                    gain += viewGain;
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
                    ++projectedUses[d][pixel];
        }
    }
}
