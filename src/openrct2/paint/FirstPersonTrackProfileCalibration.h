/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "FirstPersonAssetReconstruction.h"
#include "FirstPersonTrackTrajectory.h"

#include "../drawing/PaletteIndex.h"
#include "../interface/Viewport.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace OpenRCT2::Paint
{
    enum class FirstPersonTrackPixelChannel : uint8_t
    {
        trackRailPalette,
        primaryRemap,
        secondaryRemap,
        tertiaryRemap,
        count,
    };

    constexpr size_t kFirstPersonTrackPixelChannelCount =
        static_cast<size_t>(FirstPersonTrackPixelChannel::count);

    [[nodiscard]] constexpr uint8_t FirstPersonTrackPixelChannelBit(
        FirstPersonTrackPixelChannel channel)
    {
        return uint8_t(1u << static_cast<uint8_t>(channel));
    }

    [[nodiscard]] constexpr bool FirstPersonTrackPixelMatchesChannel(
        uint8_t pixel, FirstPersonTrackPixelChannel channel)
    {
        using Drawing::PaletteIndex;
        switch (channel)
        {
            case FirstPersonTrackPixelChannel::trackRailPalette:
                return pixel >= static_cast<uint8_t>(PaletteIndex::trackRails0)
                    && pixel <= static_cast<uint8_t>(PaletteIndex::trackRails2);
            case FirstPersonTrackPixelChannel::primaryRemap:
                return pixel >= static_cast<uint8_t>(PaletteIndex::primaryRemap0)
                    && pixel <= static_cast<uint8_t>(PaletteIndex::primaryRemap11);
            case FirstPersonTrackPixelChannel::secondaryRemap:
                return pixel >= static_cast<uint8_t>(PaletteIndex::secondaryRemap0)
                    && pixel <= static_cast<uint8_t>(PaletteIndex::secondaryRemap11);
            case FirstPersonTrackPixelChannel::tertiaryRemap:
                return pixel >= static_cast<uint8_t>(PaletteIndex::tertiaryRemap0)
                    && pixel <= static_cast<uint8_t>(PaletteIndex::tertiaryRemap11);
            default:
                return false;
        }
    }

    struct FirstPersonTrackArtworkSample
    {
        int32_t x{};
        int32_t y{};
        uint8_t pixel{};
    };

    struct FirstPersonTrackArtworkObservation
    {
        std::array<std::array<FirstPersonSilhouette, 4>,
            kFirstPersonTrackPixelChannelCount> channelViews{};
        std::array<std::array<std::vector<FirstPersonTrackArtworkSample>, 4>,
            kFirstPersonTrackPixelChannelCount> channelSamples{};
    };

    struct FirstPersonTrackProfileFit
    {
        bool valid = false;
        float score = -std::numeric_limits<float>::infinity();
        float averageCandidateCoverage = 0.0f;
        float minimumCandidateCoverage = 0.0f;
        float averageObservedCoverage = 0.0f;
        FirstPersonTrackPixelChannel channel =
            FirstPersonTrackPixelChannel::trackRailPalette;
    };

    struct FirstPersonTrackProfileCalibrationResult
    {
        bool valid = false;
        FirstPersonTrackRailProfile profile{};
        FirstPersonTrackProfileFit fit{};
    };

    enum class FirstPersonTrackValidationKind : uint8_t
    {
        curve = 1u << 0,
        slope = 1u << 1,
        bank = 1u << 2,
    };

    [[nodiscard]] inline uint8_t FirstPersonTrackValidationKinds(
        const FirstPersonTrackTrajectory& trajectory)
    {
        if (trajectory.points.size() < 2)
            return 0;

        uint8_t result = 0;
        const auto& first = trajectory.points.front();
        float minZ = first.position.z;
        float maxZ = first.position.z;
        for (const auto& point : trajectory.points)
        {
            minZ = std::min(minZ, point.position.z);
            maxZ = std::max(maxZ, point.position.z);

            const float horizontalDot =
                first.basis.forward.x * point.basis.forward.x
                + first.basis.forward.y * point.basis.forward.y;
            const float firstHorizontal = std::hypot(
                first.basis.forward.x, first.basis.forward.y);
            const float pointHorizontal = std::hypot(
                point.basis.forward.x, point.basis.forward.y);
            if (firstHorizontal > 0.1f && pointHorizontal > 0.1f
                && horizontalDot / (firstHorizontal * pointHorizontal) < 0.985f)
            {
                result |= static_cast<uint8_t>(
                    FirstPersonTrackValidationKind::curve);
            }

            if (std::abs(point.basis.forward.z) > 0.08f)
            {
                result |= static_cast<uint8_t>(
                    FirstPersonTrackValidationKind::slope);
            }

            // Pitch tilts the up vector too, so it is not bank evidence.
            // Roll is the component that lifts the local right axis out of the
            // horizontal plane.
            if (std::abs(point.basis.right.z) > 0.08f)
            {
                result |= static_cast<uint8_t>(
                    FirstPersonTrackValidationKind::bank);
            }
        }

        if (maxZ - minZ > 3.0f)
            result |= static_cast<uint8_t>(
                FirstPersonTrackValidationKind::slope);
        return result;
    }

    [[nodiscard]] inline ScreenCoordsXY ProjectFirstPersonTrackArtworkPoint(
        uint8_t rotation, const FirstPersonVec3& anchor,
        const FirstPersonVec3& point)
    {
        const CoordsXYZ anchorPoint{
            int32_t(std::lround(anchor.x)),
            int32_t(std::lround(anchor.y)),
            int32_t(std::lround(anchor.z)),
        };
        const CoordsXYZ worldPoint{
            int32_t(std::lround(point.x)),
            int32_t(std::lround(point.y)),
            int32_t(std::lround(point.z)),
        };
        const auto a = Translate3DTo2DWithZ(rotation, anchorPoint);
        const auto p = Translate3DTo2DWithZ(rotation, worldPoint);
        return { p.x - a.x, p.y - a.y };
    }

    [[nodiscard]] inline FirstPersonVec3 AddFirstPersonTrackVector(
        const FirstPersonVec3& a, const FirstPersonVec3& b)
    {
        return { a.x + b.x, a.y + b.y, a.z + b.z };
    }

    [[nodiscard]] inline FirstPersonVec3 ScaleFirstPersonTrackVector(
        const FirstPersonVec3& a, float scale)
    {
        return { a.x * scale, a.y * scale, a.z * scale };
    }

    [[nodiscard]] inline float DotFirstPersonTrackVector(
        const FirstPersonVec3& a, const FirstPersonVec3& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    inline void AddFirstPersonTrackProfileSegmentSilhouette(
        FirstPersonSilhouette& silhouette, uint8_t rotation,
        const FirstPersonVec3& anchor,
        const FirstPersonTrackTrajectoryPoint& a,
        const FirstPersonTrackTrajectoryPoint& b,
        const FirstPersonTrackRailProfile& profile, float gaugeSide)
    {
        const auto centreFor = [&](const FirstPersonTrackTrajectoryPoint& point) {
            return AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(
                    point.position,
                    ScaleFirstPersonTrackVector(point.basis.right, gaugeSide)),
                ScaleFirstPersonTrackVector(
                    point.basis.up, profile.verticalOffset));
        };

        const auto centreA = centreFor(a);
        const auto centreB = centreFor(b);
        const auto acrossA =
            ScaleFirstPersonTrackVector(a.basis.right, profile.halfWidth);
        const auto acrossB =
            ScaleFirstPersonTrackVector(b.basis.right, profile.halfWidth);
        const auto upA =
            ScaleFirstPersonTrackVector(a.basis.up, profile.halfHeight);
        const auto upB =
            ScaleFirstPersonTrackVector(b.basis.up, profile.halfHeight);

        const auto addQuad = [&](const std::array<FirstPersonVec3, 4>& world) {
            std::array<ScreenCoordsXY, 4> screen{};
            for (size_t i = 0; i < screen.size(); ++i)
            {
                screen[i] = ProjectFirstPersonTrackArtworkPoint(
                    rotation, anchor, world[i]);
            }
            AddFirstPersonSilhouetteQuad(silhouette, screen);
        };

        addQuad({ {
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(centreA, upA),
                ScaleFirstPersonTrackVector(acrossA, -1.0f)),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(centreA, upA), acrossA),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(centreB, upB), acrossB),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(centreB, upB),
                ScaleFirstPersonTrackVector(acrossB, -1.0f)),
        } });
        addQuad({ {
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(centreA,
                    ScaleFirstPersonTrackVector(upA, -1.0f)),
                ScaleFirstPersonTrackVector(acrossA, -1.0f)),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(centreB,
                    ScaleFirstPersonTrackVector(upB, -1.0f)),
                ScaleFirstPersonTrackVector(acrossB, -1.0f)),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(centreB,
                    ScaleFirstPersonTrackVector(upB, -1.0f)),
                acrossB),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(centreA,
                    ScaleFirstPersonTrackVector(upA, -1.0f)),
                acrossA),
        } });
        addQuad({ {
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(centreA, acrossA),
                ScaleFirstPersonTrackVector(upA, -1.0f)),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(centreB, acrossB),
                ScaleFirstPersonTrackVector(upB, -1.0f)),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(centreB, acrossB), upB),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(centreA, acrossA), upA),
        } });
        addQuad({ {
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(
                    centreA, ScaleFirstPersonTrackVector(acrossA, -1.0f)),
                ScaleFirstPersonTrackVector(upA, -1.0f)),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(
                    centreA, ScaleFirstPersonTrackVector(acrossA, -1.0f)),
                upA),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(
                    centreB, ScaleFirstPersonTrackVector(acrossB, -1.0f)),
                upB),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(
                    centreB, ScaleFirstPersonTrackVector(acrossB, -1.0f)),
                ScaleFirstPersonTrackVector(upB, -1.0f)),
        } });
    }

    [[nodiscard]] inline std::array<FirstPersonSilhouette, 4>
        BuildFirstPersonTrackRailSilhouettes(
            const FirstPersonTrackTrajectory& trajectory,
            const FirstPersonVec3& anchor,
            const FirstPersonTrackRailProfile& profile)
    {
        std::array<FirstPersonSilhouette, 4> result{};
        if (trajectory.points.size() < 2 || profile.halfGauge <= 0.0f
            || profile.halfWidth <= 0.0f || profile.halfHeight <= 0.0f)
            return result;

        size_t previous = 0;
        for (size_t i = 1; i < trajectory.points.size(); ++i)
        {
            const auto& a = trajectory.points[previous];
            const auto& b = trajectory.points[i];
            const float distance =
                FirstPersonTrackTrajectoryPointDistance(a, b);
            const bool turns =
                DotFirstPersonTrackVector(
                    a.basis.forward, b.basis.forward) < 0.9914449f
                || DotFirstPersonTrackVector(
                    a.basis.up, b.basis.up) < 0.9914449f;
            const bool last = i + 1 == trajectory.points.size();
            if (!last && distance < 3.0f && !turns)
                continue;

            if (distance > 0.05f)
            {
                for (uint8_t rotation = 0; rotation < 4; ++rotation)
                {
                    AddFirstPersonTrackProfileSegmentSilhouette(
                        result[rotation], rotation, anchor, a, b, profile,
                        -profile.halfGauge);
                    AddFirstPersonTrackProfileSegmentSilhouette(
                        result[rotation], rotation, anchor, a, b, profile,
                        profile.halfGauge);
                }
            }
            previous = i;
        }
        return result;
    }

    struct FirstPersonTrackRailFaceSilhouettes
    {
        std::array<FirstPersonSilhouette, 4> top{};
        std::array<FirstPersonSilhouette, 4> side{};
    };

    inline void AddFirstPersonTrackProfileSegmentMaterialSilhouettes(
        FirstPersonSilhouette& top,
        FirstPersonSilhouette& side,
        uint8_t rotation, const FirstPersonVec3& anchor,
        const FirstPersonTrackTrajectoryPoint& a,
        const FirstPersonTrackTrajectoryPoint& b,
        const FirstPersonTrackRailProfile& profile,
        float gaugeSide)
    {
        const auto centreFor =
            [&](const FirstPersonTrackTrajectoryPoint& point) {
                return AddFirstPersonTrackVector(
                    AddFirstPersonTrackVector(
                        point.position,
                        ScaleFirstPersonTrackVector(
                            point.basis.right, gaugeSide)),
                    ScaleFirstPersonTrackVector(
                        point.basis.up,
                        profile.verticalOffset));
            };
        const auto centreA = centreFor(a);
        const auto centreB = centreFor(b);
        const auto acrossA =
            ScaleFirstPersonTrackVector(
                a.basis.right, profile.halfWidth);
        const auto acrossB =
            ScaleFirstPersonTrackVector(
                b.basis.right, profile.halfWidth);
        const auto upA =
            ScaleFirstPersonTrackVector(
                a.basis.up, profile.halfHeight);
        const auto upB =
            ScaleFirstPersonTrackVector(
                b.basis.up, profile.halfHeight);

        const auto addQuad =
            [&](FirstPersonSilhouette& target,
                const std::array<FirstPersonVec3, 4>& world) {
                std::array<ScreenCoordsXY, 4> screen{};
                for (size_t i = 0; i < screen.size(); ++i)
                {
                    screen[i] =
                        ProjectFirstPersonTrackArtworkPoint(
                            rotation, anchor, world[i]);
                }
                AddFirstPersonSilhouetteQuad(target, screen);
            };

        addQuad(top, { {
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(centreA, upA),
                ScaleFirstPersonTrackVector(acrossA, -1.0f)),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(centreA, upA),
                acrossA),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(centreB, upB),
                acrossB),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(centreB, upB),
                ScaleFirstPersonTrackVector(acrossB, -1.0f)),
        } });
        addQuad(side, { {
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(
                    centreA,
                    ScaleFirstPersonTrackVector(upA, -1.0f)),
                ScaleFirstPersonTrackVector(acrossA, -1.0f)),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(
                    centreB,
                    ScaleFirstPersonTrackVector(upB, -1.0f)),
                ScaleFirstPersonTrackVector(acrossB, -1.0f)),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(
                    centreB,
                    ScaleFirstPersonTrackVector(upB, -1.0f)),
                acrossB),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(
                    centreA,
                    ScaleFirstPersonTrackVector(upA, -1.0f)),
                acrossA),
        } });
        addQuad(side, { {
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(centreA, acrossA),
                ScaleFirstPersonTrackVector(upA, -1.0f)),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(centreB, acrossB),
                ScaleFirstPersonTrackVector(upB, -1.0f)),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(centreB, acrossB),
                upB),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(centreA, acrossA),
                upA),
        } });
        addQuad(side, { {
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(
                    centreA,
                    ScaleFirstPersonTrackVector(acrossA, -1.0f)),
                ScaleFirstPersonTrackVector(upA, -1.0f)),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(
                    centreA,
                    ScaleFirstPersonTrackVector(acrossA, -1.0f)),
                upA),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(
                    centreB,
                    ScaleFirstPersonTrackVector(acrossB, -1.0f)),
                upB),
            AddFirstPersonTrackVector(
                AddFirstPersonTrackVector(
                    centreB,
                    ScaleFirstPersonTrackVector(acrossB, -1.0f)),
                ScaleFirstPersonTrackVector(upB, -1.0f)),
        } });
    }

    [[nodiscard]] inline FirstPersonTrackRailFaceSilhouettes
        BuildFirstPersonTrackRailFaceSilhouettes(
            const FirstPersonTrackTrajectory& trajectory,
            const FirstPersonVec3& anchor,
            const FirstPersonTrackRailProfile& profile)
    {
        FirstPersonTrackRailFaceSilhouettes result{};
        if (trajectory.points.size() < 2)
            return result;

        size_t previous = 0;
        for (size_t i = 1; i < trajectory.points.size(); ++i)
        {
            const auto& a = trajectory.points[previous];
            const auto& b = trajectory.points[i];
            const float distance =
                FirstPersonTrackTrajectoryPointDistance(a, b);
            const bool turns =
                DotFirstPersonTrackVector(
                    a.basis.forward, b.basis.forward)
                    < 0.9914449f
                || DotFirstPersonTrackVector(
                    a.basis.up, b.basis.up)
                    < 0.9914449f;
            const bool last =
                i + 1 == trajectory.points.size();
            if (!last && distance < 3.0f && !turns)
                continue;
            if (distance > 0.05f)
            {
                for (uint8_t rotation = 0;
                     rotation < 4; ++rotation)
                {
                    AddFirstPersonTrackProfileSegmentMaterialSilhouettes(
                        result.top[rotation],
                        result.side[rotation],
                        rotation, anchor, a, b, profile,
                        -profile.halfGauge);
                    AddFirstPersonTrackProfileSegmentMaterialSilhouettes(
                        result.top[rotation],
                        result.side[rotation],
                        rotation, anchor, a, b, profile,
                        profile.halfGauge);
                }
            }
            previous = i;
        }
        return result;
    }

    [[nodiscard]] constexpr uint8_t
        FirstPersonTrackMaterialValue(
            uint8_t pixel, FirstPersonTrackPixelChannel channel)
    {
        using Drawing::PaletteIndex;
        switch (channel)
        {
            case FirstPersonTrackPixelChannel::primaryRemap:
                return uint8_t(pixel
                    - static_cast<uint8_t>(
                        PaletteIndex::primaryRemap0));
            case FirstPersonTrackPixelChannel::secondaryRemap:
                return uint8_t(pixel
                    - static_cast<uint8_t>(
                        PaletteIndex::secondaryRemap0));
            case FirstPersonTrackPixelChannel::tertiaryRemap:
                return uint8_t(pixel
                    - static_cast<uint8_t>(
                        PaletteIndex::tertiaryRemap0));
            case FirstPersonTrackPixelChannel::trackRailPalette:
            default:
                return pixel;
        }
    }

    inline void DeriveFirstPersonTrackRailMaterial(
        FirstPersonTrackRailProfile& profile,
        const FirstPersonTrackArtworkObservation& observation,
        const FirstPersonTrackTrajectory& trajectory,
        const FirstPersonVec3& anchor,
        FirstPersonTrackPixelChannel channel)
    {
        profile.materialVerified = false;
        const auto faces =
            BuildFirstPersonTrackRailFaceSilhouettes(
                trajectory, anchor, profile);
        std::vector<uint8_t> topValues;
        std::vector<uint8_t> sideValues;
        const auto containsNear = [](
            const FirstPersonSilhouette& silhouette,
            int32_t x, int32_t y) {
            for (int32_t dy = -1; dy <= 1; ++dy)
            for (int32_t dx = -1; dx <= 1; ++dx)
            {
                if (silhouette.contains(x + dx, y + dy))
                    return true;
            }
            return false;
        };
        for (uint8_t rotation = 0; rotation < 4; ++rotation)
        {
            const auto& samples =
                observation.channelSamples[
                    static_cast<size_t>(channel)][rotation];
            for (const auto& sample : samples)
            {
                bool inTop =
                    faces.top[rotation].contains(
                        sample.x, sample.y);
                bool inSide =
                    faces.side[rotation].contains(
                        sample.x, sample.y);
                if (!inTop && !inSide)
                {
                    inTop = containsNear(
                        faces.top[rotation],
                        sample.x, sample.y);
                    inSide = containsNear(
                        faces.side[rotation],
                        sample.x, sample.y);
                }
                // Pixels that can be explained by both faces are exactly the
                // ambiguous silhouette boundary. Do not let them determine
                // a face material.
                if (inTop == inSide)
                    continue;
                const uint8_t value =
                    FirstPersonTrackMaterialValue(
                        sample.pixel, channel);
                (inTop ? topValues : sideValues)
                    .push_back(value);
            }
        }
        constexpr size_t kMinimumFaceSamples = 8;
        if (topValues.size() < kMinimumFaceSamples
            || sideValues.size() < kMinimumFaceSamples)
            return;

        const auto median = [](std::vector<uint8_t>& values) {
            const size_t middle = values.size() / 2;
            std::nth_element(
                values.begin(),
                values.begin() + middle,
                values.end());
            return values[middle];
        };
        profile.topMaterialValue = median(topValues);
        profile.sideMaterialValue = median(sideValues);
        profile.materialVerified = true;
    }

    [[nodiscard]] inline bool FirstPersonTrackSilhouetteContainsNear(
        const FirstPersonSilhouette& silhouette, int32_t x, int32_t y,
        int32_t tolerance = 1)
    {
        for (int32_t dy = -tolerance; dy <= tolerance; ++dy)
        for (int32_t dx = -tolerance; dx <= tolerance; ++dx)
        {
            if (silhouette.contains(x + dx, y + dy))
                return true;
        }
        return false;
    }

    [[nodiscard]] inline float FirstPersonTrackNearCoverage(
        const FirstPersonSilhouette& source,
        const FirstPersonSilhouette& target, int32_t tolerance = 1)
    {
        if (source.empty() || target.empty())
            return 0.0f;
        size_t covered = 0;
        for (const auto pixel : source.pixels)
        {
            const int32_t x = int32_t(uint32_t(pixel >> 32));
            const int32_t y = int32_t(uint32_t(pixel));
            if (FirstPersonTrackSilhouetteContainsNear(
                    target, x, y, tolerance))
                ++covered;
        }
        return float(covered) / float(source.size());
    }

    [[nodiscard]] inline bool FirstPersonTrackObservationChannelComplete(
        const FirstPersonTrackArtworkObservation& observation,
        FirstPersonTrackPixelChannel channel)
    {
        const auto& views =
            observation.channelViews[static_cast<size_t>(channel)];
        for (const auto& view : views)
        {
            if (view.overflowed || view.size() < 8 || view.size() > 32768)
                return false;
        }
        return true;
    }

    [[nodiscard]] inline bool FirstPersonTrackObservationHasCompleteChannel(
        const FirstPersonTrackArtworkObservation& observation)
    {
        for (size_t i = 0; i < kFirstPersonTrackPixelChannelCount; ++i)
        {
            if (FirstPersonTrackObservationChannelComplete(
                    observation,
                    static_cast<FirstPersonTrackPixelChannel>(i)))
                return true;
        }
        return false;
    }

    [[nodiscard]] inline FirstPersonTrackProfileFit
        EvaluateFirstPersonTrackProfileFit(
            const FirstPersonTrackArtworkObservation& observation,
            const std::array<FirstPersonSilhouette, 4>& candidate,
            FirstPersonTrackPixelChannel channel)
    {
        FirstPersonTrackProfileFit result{};
        result.channel = channel;
        if (!FirstPersonTrackObservationChannelComplete(
                observation, channel))
            return result;

        const auto& observed =
            observation.channelViews[static_cast<size_t>(channel)];
        result.valid = true;
        result.minimumCandidateCoverage = 1.0f;
        for (size_t rotation = 0; rotation < 4; ++rotation)
        {
            if (candidate[rotation].overflowed
                || candidate[rotation].size() < 8)
            {
                return {};
            }
            const float candidateCoverage = FirstPersonTrackNearCoverage(
                candidate[rotation], observed[rotation], 1);
            const float observedCoverage = FirstPersonTrackNearCoverage(
                observed[rotation], candidate[rotation], 1);
            result.minimumCandidateCoverage = std::min(
                result.minimumCandidateCoverage, candidateCoverage);
            result.averageCandidateCoverage += candidateCoverage * 0.25f;
            result.averageObservedCoverage += observedCoverage * 0.25f;
        }

        const float symmetricEvidence = std::sqrt(std::max(
            0.0f,
            result.averageCandidateCoverage
                * result.averageObservedCoverage));
        result.score =
            0.85f * result.averageCandidateCoverage
            + 0.15f * symmetricEvidence;
        return result;
    }

    [[nodiscard]] inline FirstPersonTrackProfileFit
        BestFirstPersonTrackProfileFit(
            const FirstPersonTrackArtworkObservation& observation,
            const std::array<FirstPersonSilhouette, 4>& candidate)
    {
        FirstPersonTrackProfileFit best{};
        for (size_t i = 0; i < kFirstPersonTrackPixelChannelCount; ++i)
        {
            const auto channel =
                static_cast<FirstPersonTrackPixelChannel>(i);
            const auto fit = EvaluateFirstPersonTrackProfileFit(
                observation, candidate, channel);
            if (!fit.valid)
                continue;
            if (!best.valid || fit.score > best.score)
                best = fit;
        }
        return best;
    }

    [[nodiscard]] inline bool IsFirstPersonTrackCalibrationFitReliable(
        const FirstPersonTrackProfileFit& fit)
    {
        // The fitted prism is deliberately allowed to explain only the rail
        // subset of a remap channel: sleepers, spine and supports can share a
        // colour. What must be strong is candidate -> artwork agreement in all
        // four native projections.
        return fit.valid
            && fit.minimumCandidateCoverage >= 0.72f
            && fit.averageCandidateCoverage >= 0.80f
            && fit.score >= 0.70f;
    }

    [[nodiscard]] inline bool IsFirstPersonTrackHoldoutFitReliable(
        const FirstPersonTrackProfileFit& fit)
    {
        // Curves/slopes/banks have more raster aliasing than the calibration
        // straight, but still require the same fixed 3-D profile to explain
        // most projected rail pixels in every view.
        return fit.valid
            && fit.minimumCandidateCoverage >= 0.62f
            && fit.averageCandidateCoverage >= 0.72f
            && fit.score >= 0.63f;
    }

    [[nodiscard]] inline FirstPersonTrackProfileCalibrationResult
        FitFirstPersonTrackRailProfileFromArtwork(
            const FirstPersonTrackArtworkObservation& observation,
            const FirstPersonTrackTrajectory& trajectory,
            const FirstPersonVec3& anchor)
    {
        FirstPersonTrackProfileCalibrationResult result{};
        if (trajectory.points.size() < 2
            || !FirstPersonTrackObservationHasCompleteChannel(observation))
            return result;

        struct SearchResult
        {
            bool valid = false;
            FirstPersonTrackRailProfile profile{};
            FirstPersonTrackProfileFit fit{};
        };
        SearchResult best{};

        const auto tryCandidate = [&](float halfGauge, float halfWidth,
                                      float verticalOffset,
                                      SearchResult& destination) {
            if (!(halfGauge >= 1.5f && halfGauge <= 15.0f)
                || !(halfWidth >= 0.35f && halfWidth <= 4.0f)
                || halfGauge <= halfWidth + 0.5f
                || !(verticalOffset >= -20.0f
                    && verticalOffset <= 20.0f))
                return;

            FirstPersonTrackRailProfile profile{};
            profile.halfGauge = halfGauge;
            profile.halfWidth = halfWidth;
            // The fitted "rail width" is used as the square prism section.
            // This avoids inventing a second, unobserved thickness parameter.
            profile.halfHeight = halfWidth;
            profile.verticalOffset = verticalOffset;

            const auto candidate = BuildFirstPersonTrackRailSilhouettes(
                trajectory, anchor, profile);
            const auto fit =
                BestFirstPersonTrackProfileFit(observation, candidate);
            if (!fit.valid)
                return;

            const float oldComplexity =
                destination.profile.halfWidth
                + destination.profile.halfGauge * 0.01f;
            const float newComplexity =
                halfWidth + halfGauge * 0.01f;
            if (!destination.valid || fit.score > destination.fit.score + 1e-5f
                || (std::abs(fit.score - destination.fit.score) <= 1e-5f
                    && newComplexity < oldComplexity))
            {
                destination.valid = true;
                destination.profile = profile;
                destination.fit = fit;
            }
        };

        for (float halfGauge = 2.0f; halfGauge <= 14.0f; halfGauge += 2.0f)
        for (float halfWidth = 0.5f; halfWidth <= 3.5f; halfWidth += 1.0f)
        for (float verticalOffset = -16.0f;
             verticalOffset <= 16.0f; verticalOffset += 4.0f)
        {
            tryCandidate(
                halfGauge, halfWidth, verticalOffset, best);
        }
        if (!best.valid)
            return result;

        SearchResult refined = best;
        for (float halfGauge = best.profile.halfGauge - 1.5f;
             halfGauge <= best.profile.halfGauge + 1.5f;
             halfGauge += 0.5f)
        for (float halfWidth = best.profile.halfWidth - 1.0f;
             halfWidth <= best.profile.halfWidth + 1.0f;
             halfWidth += 0.25f)
        for (float verticalOffset = best.profile.verticalOffset - 3.0f;
             verticalOffset <= best.profile.verticalOffset + 3.0f;
             verticalOffset += 1.0f)
        {
            tryCandidate(
                halfGauge, halfWidth, verticalOffset, refined);
        }

        if (!refined.valid
            || !IsFirstPersonTrackCalibrationFitReliable(refined.fit))
            return result;

        refined.profile.sourceChannelMask =
            FirstPersonTrackPixelChannelBit(refined.fit.channel);
        DeriveFirstPersonTrackRailMaterial(
            refined.profile, observation, trajectory,
            anchor, refined.fit.channel);
        result.valid = true;
        result.profile = refined.profile;
        result.fit = refined.fit;
        return result;
    }

    [[nodiscard]] inline FirstPersonTrackProfileFit
        ValidateFirstPersonTrackRailProfileAgainstArtwork(
            const FirstPersonTrackArtworkObservation& observation,
            const FirstPersonTrackTrajectory& trajectory,
            const FirstPersonVec3& anchor,
            const FirstPersonTrackRailProfile& profile)
    {
        if (trajectory.points.size() < 2
            || !FirstPersonTrackObservationHasCompleteChannel(observation))
            return {};
        const auto candidate = BuildFirstPersonTrackRailSilhouettes(
            trajectory, anchor, profile);
        FirstPersonTrackProfileFit best{};
        for (size_t i = 0; i < kFirstPersonTrackPixelChannelCount; ++i)
        {
            const auto channel =
                static_cast<FirstPersonTrackPixelChannel>(i);
            if ((profile.sourceChannelMask
                    & FirstPersonTrackPixelChannelBit(channel)) == 0)
                continue;
            const auto fit = EvaluateFirstPersonTrackProfileFit(
                observation, candidate, channel);
            if (!fit.valid)
                continue;
            if (!best.valid || fit.score > best.score)
                best = fit;
        }
        return best;
    }
} // namespace OpenRCT2::Paint
