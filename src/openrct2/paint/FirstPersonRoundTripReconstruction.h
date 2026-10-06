/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonAssetReconstruction.h"
#include "FirstPersonMath.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

namespace OpenRCT2::Paint
{
    // Canonical candidate geometry for inverse reconstruction. Keeping the
    // certificate triangle-based is deliberate: voxel hulls, semantic surfaces
    // and future fitted meshes can all be judged by the same forward renderer.
    struct FirstPersonReconstructionTriangle
    {
        std::array<FirstPersonVec3, 3> corners{};
    };

    struct FirstPersonRoundTripViewFit
    {
        uint8_t imageDirection{};
        FirstPersonSilhouetteFit fit{};
        size_t symmetricDifferencePixels{};
    };

    struct FirstPersonRoundTripCertificate
    {
        bool valid = false;
        size_t viewCount = 0;
        float averageIntersectionOverUnion = 0.0f;
        float minimumIntersectionOverUnion = 0.0f;
        float minimumCandidateCoverage = 0.0f;
        float minimumObservedCoverage = 0.0f;
        int32_t maximumEdgeError = 0;
        size_t symmetricDifferencePixels = 0;
        size_t maximumViewSymmetricDifferencePixels = 0;
        std::vector<FirstPersonRoundTripViewFit> views;

        [[nodiscard]] const FirstPersonSilhouetteFit*
            fitForDirection(uint8_t direction) const
        {
            const auto found = std::find_if(
                views.begin(), views.end(),
                [direction](const auto& view) {
                    return view.imageDirection == direction;
                });
            return found == views.end()
                ? nullptr : &found->fit;
        }
    };

    [[nodiscard]] inline size_t
        FirstPersonSilhouetteSymmetricDifference(
            const FirstPersonSilhouette& a,
            const FirstPersonSilhouette& b)
    {
        size_t intersection = 0;
        const auto& smaller =
            a.size() < b.size() ? a : b;
        const auto& larger =
            a.size() < b.size() ? b : a;
        for (const auto pixel : smaller.pixels)
        {
            if (larger.pixels.contains(pixel))
                ++intersection;
        }
        return a.size() + b.size()
            - intersection * 2;
    }

    [[nodiscard]] inline bool
        FirstPersonRoundTripNonWorse(
            const FirstPersonRoundTripCertificate& candidate,
            const FirstPersonRoundTripCertificate& baseline)
    {
        if (!candidate.valid || !baseline.valid
            || candidate.viewCount != baseline.viewCount)
            return false;
        return candidate.symmetricDifferencePixels
                <= baseline.symmetricDifferencePixels
            && candidate.maximumViewSymmetricDifferencePixels
                <= baseline.maximumViewSymmetricDifferencePixels
            && candidate.maximumEdgeError
                <= baseline.maximumEdgeError;
    }

    [[nodiscard]] inline bool
        FirstPersonRoundTripStrictlyImproves(
            const FirstPersonRoundTripCertificate& candidate,
            const FirstPersonRoundTripCertificate& baseline)
    {
        // Promotion is evidence-conservative: total native-pixel disagreement
        // must fall, while neither the worst single view nor the silhouette
        // edge error may regress. This avoids inventing a subjective
        // "smoothness" score or an uncalibrated absolute threshold.
        return FirstPersonRoundTripNonWorse(
                   candidate, baseline)
            && candidate.symmetricDifferencePixels
                < baseline.symmetricDifferencePixels;
    }

    [[nodiscard]] constexpr bool
        FirstPersonSurfaceVisibleFromNativeView(
            FirstPersonVec3 outwardNormal,
            uint8_t rotation)
    {
        // FirstPersonIsoDepth() increases toward the canonical native camera.
        // Its world-space gradient is (view.x, view.y, +1), so this is the
        // exact orthographic facing test for arbitrary (including sloped)
        // reconstruction surfaces.
        const auto view =
            FirstPersonNativeViewDirection(rotation);
        return outwardNormal.x * float(view.x)
                + outwardNormal.y * float(view.y)
                + outwardNormal.z
            > 1.0e-6f;
    }

    struct FirstPersonStructuralOwnershipConfig
    {
        float sampleSpacing = 16.0f;
        uint8_t maximumSamplesPerAxis = 8;
        size_t minimumPairedSamples = 16;
        size_t minimumViews = 2;
    };

    struct FirstPersonStructuralOwnershipComparison
    {
        bool valid = false;
        bool baselineEvidenceAvailable = false;
        bool candidateEvidenceAvailable = false;
        size_t pairedSamples = 0;
        size_t viewCount = 0;
        size_t baselineEvidenceSamples = 0;
        size_t baselineEvidenceViews = 0;
        size_t candidateEvidenceSamples = 0;
        size_t candidateEvidenceViews = 0;
        size_t baselineMatches = 0;
        size_t baselineMismatches = 0;
        size_t candidateMatches = 0;
        size_t candidateMismatches = 0;
        size_t candidateBetter = 0;
        size_t candidateWorse = 0;

        [[nodiscard]] bool candidateNonWorse() const
        {
            // Do not trade one structurally supported correspondence for
            // another. With no calibrated weights, a single match->mismatch
            // regression is evidence against promotion.
            return valid
                && candidateWorse == 0
                && candidateMismatches
                    <= baselineMismatches;
        }

        [[nodiscard]] bool candidateStrictlyImproves() const
        {
            return candidateNonWorse()
                && candidateBetter != 0;
        }
    };

    [[nodiscard]] inline bool
        FirstPersonEvidenceDrivenCandidateImproves(
            const FirstPersonRoundTripCertificate& candidate,
            const FirstPersonRoundTripCertificate& baseline,
            const FirstPersonStructuralOwnershipComparison&
                structural)
    {
        if (!FirstPersonRoundTripNonWorse(
                candidate, baseline))
            return false;

        // If the baseline has enough structural evidence, a candidate may
        // not make that evidence disappear. This closes an otherwise subtle
        // escape hatch where changing visibility could turn a strong ownership
        // constraint into "unknown" and fall back to silhouette-only promotion.
        if (structural.baselineEvidenceAvailable
            && !structural.valid)
            return false;
        // When even the baseline lacks enough unambiguous source ownership,
        // ignore the structural channel rather than inventing correspondence.
        if (structural.valid
            && !structural.candidateNonWorse())
            return false;

        return candidate.symmetricDifferencePixels
                    < baseline.symmetricDifferencePixels
            || (structural.valid
                && structural.candidateStrictlyImproves());
    }

    template<
        typename FaceRange, typename DirectionRange,
        typename ProjectPoint, typename DepthPoint,
        typename FaceVisible, typename ExpectedOwner,
        typename ObservedOwner>
    [[nodiscard]] inline
        FirstPersonStructuralOwnershipComparison
        CompareFirstPersonStructuralOwnership(
            const FaceRange& referenceFaces,
            const FaceRange& baselineFaces,
            const FaceRange& candidateFaces,
            const DirectionRange& directions,
            ProjectPoint&& projectPoint,
            DepthPoint&& depthPoint,
            FaceVisible&& faceVisible,
            ExpectedOwner&& expectedOwner,
            ObservedOwner&& observedOwner,
            const FirstPersonStructuralOwnershipConfig&
                config = {})
    {
        FirstPersonStructuralOwnershipComparison result{};
        if (referenceFaces.empty()
            || referenceFaces.size()
                != baselineFaces.size()
            || referenceFaces.size()
                != candidateFaces.size()
            || directions.empty()
            || !(config.sampleSpacing > 0.0f)
            || config.maximumSamplesPerAxis == 0)
            return result;

        using DepthMaps =
            std::vector<FirstPersonDepthOwnerMap>;
        const auto buildDepthMaps =
            [&](const FaceRange& faces) {
                DepthMaps maps(directions.size());
                for (size_t viewIndex = 0;
                     viewIndex < directions.size();
                     ++viewIndex)
                {
                    const uint8_t direction =
                        directions[viewIndex];
                    for (size_t faceIndex = 0;
                         faceIndex < faces.size();
                         ++faceIndex)
                    {
                        if (faceIndex
                            > std::numeric_limits<
                                  uint32_t>::max())
                            return DepthMaps{};
                        const auto& face =
                            faces[faceIndex];
                        if (!faceVisible(
                                face, direction))
                            continue;

                        std::array<
                            ScreenCoordsXY, 4> screen{};
                        std::array<float, 4> depth{};
                        for (size_t i = 0;
                             i < face.corners.size();
                             ++i)
                        {
                            const auto projected =
                                projectPoint(
                                    direction,
                                    face.corners[i]);
                            screen[i] = {
                                int32_t(std::lround(
                                    projected[0])),
                                int32_t(std::lround(
                                    projected[1])),
                            };
                            depth[i] =
                                depthPoint(
                                    direction,
                                    face.corners[i]);
                        }
                        AddFirstPersonDepthTriangle(
                            maps[viewIndex],
                            uint32_t(faceIndex),
                            {
                                screen[0], screen[1],
                                screen[2],
                            },
                            {
                                depth[0], depth[1],
                                depth[2],
                            });
                        AddFirstPersonDepthTriangle(
                            maps[viewIndex],
                            uint32_t(faceIndex),
                            {
                                screen[0], screen[2],
                                screen[3],
                            },
                            {
                                depth[0], depth[2],
                                depth[3],
                            });
                    }
                }
                return maps;
            };

        const auto baselineDepth =
            buildDepthMaps(baselineFaces);
        const auto candidateDepth =
            buildDepthMaps(candidateFaces);
        if (baselineDepth.size()
                != directions.size()
            || candidateDepth.size()
                != directions.size())
            return result;

        const auto pointOnFace =
            [](const auto& face,
               float s, float t) {
                const auto lerp =
                    [](FirstPersonVec3 a,
                       FirstPersonVec3 b,
                       float alpha) {
                        return FirstPersonVec3{
                            a.x + (b.x - a.x) * alpha,
                            a.y + (b.y - a.y) * alpha,
                            a.z + (b.z - a.z) * alpha,
                        };
                    };
                return lerp(
                    lerp(
                        face.corners[0],
                        face.corners[1], s),
                    lerp(
                        face.corners[3],
                        face.corners[2], s),
                    t);
            };
        const auto edgeLength =
            [](FirstPersonVec3 a,
               FirstPersonVec3 b) {
                const float dx = b.x - a.x;
                const float dy = b.y - a.y;
                const float dz = b.z - a.z;
                return std::sqrt(
                    dx * dx + dy * dy + dz * dz);
            };
        const auto ownedNear =
            [](const FirstPersonDepthOwnerMap& map,
               uint32_t owner,
               ScreenCoordsXY point) {
                for (int32_t dy = -1;
                     dy <= 1; ++dy)
                for (int32_t dx = -1;
                     dx <= 1; ++dx)
                {
                    const auto found =
                        map.find(
                            FirstPersonSilhouettePixelKey(
                                point.x + dx,
                                point.y + dy));
                    if (found != map.end()
                        && found->second.owner
                            == owner)
                        return true;
                }
                return false;
            };

        std::array<bool, 256> viewsWithEvidence{};
        std::array<bool, 256> baselineViewsWithEvidence{};
        std::array<bool, 256> candidateViewsWithEvidence{};
        for (size_t faceIndex = 0;
             faceIndex < referenceFaces.size();
             ++faceIndex)
        {
            if (faceIndex
                > std::numeric_limits<
                      uint32_t>::max())
                return {};
            const auto& reference =
                referenceFaces[faceIndex];
            const float width =
                std::max(
                    edgeLength(
                        reference.corners[0],
                        reference.corners[1]),
                    edgeLength(
                        reference.corners[3],
                        reference.corners[2]));
            const float height =
                std::max(
                    edgeLength(
                        reference.corners[0],
                        reference.corners[3]),
                    edgeLength(
                        reference.corners[1],
                        reference.corners[2]));
            const int32_t sampleX =
                std::clamp(
                    int32_t(std::ceil(
                        width
                        / config.sampleSpacing)),
                    1,
                    int32_t(
                        config.maximumSamplesPerAxis));
            const int32_t sampleY =
                std::clamp(
                    int32_t(std::ceil(
                        height
                        / config.sampleSpacing)),
                    1,
                    int32_t(
                        config.maximumSamplesPerAxis));

            for (int32_t y = 0;
                 y < sampleY; ++y)
            for (int32_t x = 0;
                 x < sampleX; ++x)
            {
                const float s =
                    (float(x) + 0.5f)
                    / float(sampleX);
                const float t =
                    (float(y) + 0.5f)
                    / float(sampleY);
                const auto referencePoint =
                    pointOnFace(
                        reference, s, t);
                const auto expected =
                    expectedOwner(
                        referencePoint,
                        reference.normal);
                if (!expected.has_value())
                    continue;

                const auto baselinePoint =
                    pointOnFace(
                        baselineFaces[faceIndex],
                        s, t);
                const auto candidatePoint =
                    pointOnFace(
                        candidateFaces[faceIndex],
                        s, t);

                for (size_t viewIndex = 0;
                     viewIndex < directions.size();
                     ++viewIndex)
                {
                    const uint8_t direction =
                        directions[viewIndex];
                    const auto baselineProjectedF =
                        projectPoint(
                            direction,
                            baselinePoint);
                    const auto candidateProjectedF =
                        projectPoint(
                            direction,
                            candidatePoint);
                    const ScreenCoordsXY baselineProjected{
                        int32_t(std::lround(
                            baselineProjectedF[0])),
                        int32_t(std::lround(
                            baselineProjectedF[1])),
                    };
                    const ScreenCoordsXY candidateProjected{
                        int32_t(std::lround(
                            candidateProjectedF[0])),
                        int32_t(std::lround(
                            candidateProjectedF[1])),
                    };
                    const bool baselineOwned =
                        ownedNear(
                            baselineDepth[viewIndex],
                            uint32_t(faceIndex),
                            baselineProjected);
                    const bool candidateOwned =
                        ownedNear(
                            candidateDepth[viewIndex],
                            uint32_t(faceIndex),
                            candidateProjected);
                    const auto baselineObserved =
                        baselineOwned
                        ? observedOwner(
                              direction,
                              baselineProjected)
                        : std::nullopt;
                    const auto candidateObserved =
                        candidateOwned
                        ? observedOwner(
                              direction,
                              candidateProjected)
                        : std::nullopt;

                    if (baselineObserved.has_value())
                    {
                        ++result.baselineEvidenceSamples;
                        baselineViewsWithEvidence[direction] =
                            true;
                    }
                    if (candidateObserved.has_value())
                    {
                        ++result.candidateEvidenceSamples;
                        candidateViewsWithEvidence[direction] =
                            true;
                    }
                    if (!baselineObserved.has_value()
                        || !candidateObserved.has_value())
                        continue;

                    const bool baselineMatches =
                        *baselineObserved
                            == *expected;
                    const bool candidateMatches =
                        *candidateObserved
                            == *expected;
                    ++result.pairedSamples;
                    if (baselineMatches)
                        ++result.baselineMatches;
                    else
                        ++result.baselineMismatches;
                    if (candidateMatches)
                        ++result.candidateMatches;
                    else
                        ++result.candidateMismatches;
                    if (!baselineMatches
                        && candidateMatches)
                        ++result.candidateBetter;
                    else if (baselineMatches
                        && !candidateMatches)
                        ++result.candidateWorse;
                    viewsWithEvidence[direction] = true;
                }
            }
        }

        result.viewCount =
            size_t(std::count(
                viewsWithEvidence.begin(),
                viewsWithEvidence.end(),
                true));
        result.baselineEvidenceViews =
            size_t(std::count(
                baselineViewsWithEvidence.begin(),
                baselineViewsWithEvidence.end(),
                true));
        result.candidateEvidenceViews =
            size_t(std::count(
                candidateViewsWithEvidence.begin(),
                candidateViewsWithEvidence.end(),
                true));
        result.baselineEvidenceAvailable =
            result.baselineEvidenceSamples
                >= config.minimumPairedSamples
            && result.baselineEvidenceViews
                >= config.minimumViews;
        result.candidateEvidenceAvailable =
            result.candidateEvidenceSamples
                >= config.minimumPairedSamples
            && result.candidateEvidenceViews
                >= config.minimumViews;
        result.valid =
            result.pairedSamples
                >= config.minimumPairedSamples
            && result.viewCount
                >= config.minimumViews;
        return result;
    }

    template<typename FaceRange>
    [[nodiscard]] inline std::vector<FirstPersonReconstructionTriangle>
        TriangulateFirstPersonReconstructionFaces(
            const FaceRange& faces)
    {
        std::vector<FirstPersonReconstructionTriangle> result;
        result.reserve(faces.size() * 2);
        for (const auto& face : faces)
        {
            FirstPersonReconstructionTriangle first{};
            first.corners = {
                face.corners[0],
                face.corners[1],
                face.corners[2],
            };
            result.push_back(first);

            FirstPersonReconstructionTriangle second{};
            second.corners = {
                face.corners[0],
                face.corners[2],
                face.corners[3],
            };
            result.push_back(second);
        }
        return result;
    }

    template<typename ProjectPoint>
    [[nodiscard]] inline FirstPersonSilhouette
        RenderFirstPersonReconstructionSilhouette(
            const std::vector<FirstPersonReconstructionTriangle>& triangles,
            uint8_t imageDirection,
            ProjectPoint&& projectPoint)
    {
        FirstPersonSilhouette result{};
        for (const auto& triangle : triangles)
        {
            std::array<ScreenCoordsXY, 3> screen{};
            for (size_t i = 0; i < triangle.corners.size(); ++i)
            {
                const auto projected =
                    projectPoint(
                        imageDirection,
                        triangle.corners[i]);
                screen[i] = {
                    int32_t(std::lround(projected[0])),
                    int32_t(std::lround(projected[1])),
                };
            }
            AddFirstPersonSilhouetteTriangle(
                result, screen[0], screen[1], screen[2]);
            if (result.overflowed)
                return result;
        }
        return result;
    }

    // Forward-render a candidate through the exact same canonical image
    // directions as its source observations, then compare the resulting
    // silhouette with the native artwork. This is a certificate of what the
    // candidate actually projects to; it deliberately knows nothing about how
    // that candidate was inferred.
    template<typename ObservationRange, typename ProjectPoint>
    [[nodiscard]] inline FirstPersonRoundTripCertificate
        CertifyFirstPersonReconstruction(
            const std::vector<FirstPersonReconstructionTriangle>& triangles,
            const ObservationRange& observations,
            ProjectPoint&& projectPoint)
    {
        FirstPersonRoundTripCertificate result{};
        if (triangles.empty() || observations.empty())
            return result;

        result.minimumIntersectionOverUnion = 1.0f;
        result.minimumCandidateCoverage = 1.0f;
        result.minimumObservedCoverage = 1.0f;
        result.views.reserve(observations.size());
        std::array<bool, 256> seenDirections{};

        for (const auto& observation : observations)
        {
            const uint8_t direction =
                observation.imageDirection;
            if (seenDirections[direction]
                || observation.observed.empty()
                || observation.observed.overflowed)
                return {};

            auto predicted =
                RenderFirstPersonReconstructionSilhouette(
                    triangles, direction, projectPoint);
            if (predicted.empty() || predicted.overflowed)
                return {};

            const auto fit =
                CompareFirstPersonSilhouettes(
                    observation.observed, predicted);
            if (!fit.valid)
                return {};

            const size_t disagreement =
                FirstPersonSilhouetteSymmetricDifference(
                    observation.observed, predicted);
            seenDirections[direction] = true;
            FirstPersonRoundTripViewFit viewFit{};
            viewFit.imageDirection = direction;
            viewFit.fit = fit;
            viewFit.symmetricDifferencePixels =
                disagreement;
            result.views.push_back(viewFit);
            ++result.viewCount;
            result.symmetricDifferencePixels +=
                disagreement;
            result.maximumViewSymmetricDifferencePixels =
                std::max(
                    result.maximumViewSymmetricDifferencePixels,
                    disagreement);
            result.averageIntersectionOverUnion +=
                fit.intersectionOverUnion;
            result.minimumIntersectionOverUnion =
                std::min(
                    result.minimumIntersectionOverUnion,
                    fit.intersectionOverUnion);
            result.minimumCandidateCoverage =
                std::min(
                    result.minimumCandidateCoverage,
                    fit.candidateCoverage);
            result.minimumObservedCoverage =
                std::min(
                    result.minimumObservedCoverage,
                    fit.observedCoverage);
            result.maximumEdgeError =
                std::max(
                    result.maximumEdgeError,
                    fit.maxEdgeError);
        }

        if (result.viewCount == 0)
            return {};
        result.averageIntersectionOverUnion /=
            float(result.viewCount);
        result.valid = true;
        return result;
    }
} // namespace OpenRCT2::Paint
