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
        FirstPersonRoundTripStrictlyImproves(
            const FirstPersonRoundTripCertificate& candidate,
            const FirstPersonRoundTripCertificate& baseline)
    {
        if (!candidate.valid || !baseline.valid
            || candidate.viewCount != baseline.viewCount)
            return false;

        // Promotion is evidence-conservative: total native-pixel disagreement
        // must fall, while neither the worst single view nor the silhouette
        // edge error may regress. This avoids inventing a subjective
        // "smoothness" score or an uncalibrated absolute threshold.
        return candidate.symmetricDifferencePixels
                < baseline.symmetricDifferencePixels
            && candidate.maximumViewSymmetricDifferencePixels
                <= baseline.maximumViewSymmetricDifferencePixels
            && candidate.maximumEdgeError
                <= baseline.maximumEdgeError;
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
