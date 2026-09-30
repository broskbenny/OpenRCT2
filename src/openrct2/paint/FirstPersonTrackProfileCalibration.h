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
        // Observed evidence is measured only in a narrow band around the
        // predicted rail section, so remote sleepers/supports neither help nor
        // hurt the fit.
        float averageLocalObservedCoverage = 0.0f;
        // Both predicted rails must be independently supported. This prevents
        // one broad same-colour structure from standing in for a twin-rail
        // section merely because the combined prism lies inside it.
        float minimumRailCoverage = 0.0f;
        // Holdouts use a property the source optimizer does not optimise:
        // support continuity along each predicted rail locus.
        float minimumContinuityCoverage = 0.0f;
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
        if (trajectory.points.size() < 2
            || profile.railCount == 0
            || (profile.railCount > 1 && profile.halfGauge <= 0.0f)
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
                    if (profile.railCount == 1)
                    {
                        AddFirstPersonTrackProfileSegmentSilhouette(
                            result[rotation], rotation, anchor, a, b, profile,
                            0.0f);
                    }
                    else
                    {
                        AddFirstPersonTrackProfileSegmentSilhouette(
                            result[rotation], rotation, anchor, a, b, profile,
                            -profile.halfGauge);
                        AddFirstPersonTrackProfileSegmentSilhouette(
                            result[rotation], rotation, anchor, a, b, profile,
                            profile.halfGauge);
                    }
                }
            }
            previous = i;
        }
        return result;
    }

    [[nodiscard]] inline std::array<FirstPersonSilhouette, 4>
        BuildFirstPersonTrackSingleRailSilhouettes(
            const FirstPersonTrackTrajectory& trajectory,
            const FirstPersonVec3& anchor,
            const FirstPersonTrackRailProfile& profile,
            float gaugeSide)
    {
        std::array<FirstPersonSilhouette, 4> result{};
        if (trajectory.points.size() < 2
            || profile.halfWidth <= 0.0f
            || profile.halfHeight <= 0.0f)
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
                    AddFirstPersonTrackProfileSegmentSilhouette(
                        result[rotation], rotation, anchor,
                        a, b, profile, gaugeSide);
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

    [[nodiscard]] inline float
        FirstPersonTrackLocalObservedCoverage(
            const FirstPersonSilhouette& observed,
            const FirstPersonSilhouette& candidate,
            int32_t bandTolerance = 3,
            int32_t matchTolerance = 1)
    {
        if (observed.empty() || candidate.empty())
            return 0.0f;
        size_t localObserved = 0;
        size_t explained = 0;
        for (const auto pixel : observed.pixels)
        {
            const int32_t x =
                int32_t(uint32_t(pixel >> 32));
            const int32_t y =
                int32_t(uint32_t(pixel));
            if (!FirstPersonTrackSilhouetteContainsNear(
                    candidate, x, y, bandTolerance))
                continue;
            ++localObserved;
            if (FirstPersonTrackSilhouetteContainsNear(
                    candidate, x, y, matchTolerance))
                ++explained;
        }
        return localObserved != 0
            ? float(explained) / float(localObserved)
            : 0.0f;
    }

    [[nodiscard]] inline FirstPersonTrackProfileFit
        EvaluateFirstPersonTrackProfileFit(
            const FirstPersonTrackArtworkObservation& observation,
            const std::array<FirstPersonSilhouette, 4>& candidate,
            const std::array<FirstPersonSilhouette, 4>& negativeRail,
            const std::array<FirstPersonSilhouette, 4>& positiveRail,
            FirstPersonTrackPixelChannel channel)
    {
        FirstPersonTrackProfileFit result{};
        result.channel = channel;
        if (!FirstPersonTrackObservationChannelComplete(
                observation, channel))
            return result;

        const auto& observed =
            observation.channelViews[
                static_cast<size_t>(channel)];
        result.valid = true;
        result.minimumCandidateCoverage = 1.0f;
        result.minimumRailCoverage = 1.0f;
        for (size_t rotation = 0;
             rotation < 4; ++rotation)
        {
            if (candidate[rotation].overflowed
                || candidate[rotation].size() < 8
                || negativeRail[rotation].overflowed
                || positiveRail[rotation].overflowed
                || negativeRail[rotation].size() < 3
                || positiveRail[rotation].size() < 3)
            {
                return {};
            }

            const float candidateCoverage =
                FirstPersonTrackNearCoverage(
                    candidate[rotation],
                    observed[rotation], 1);
            const float observedCoverage =
                FirstPersonTrackNearCoverage(
                    observed[rotation],
                    candidate[rotation], 1);
            const float localObservedCoverage =
                FirstPersonTrackLocalObservedCoverage(
                    observed[rotation],
                    candidate[rotation], 3, 1);
            const float negativeCoverage =
                FirstPersonTrackNearCoverage(
                    negativeRail[rotation],
                    observed[rotation], 1);
            const float positiveCoverage =
                FirstPersonTrackNearCoverage(
                    positiveRail[rotation],
                    observed[rotation], 1);

            result.minimumCandidateCoverage =
                std::min(
                    result.minimumCandidateCoverage,
                    candidateCoverage);
            result.minimumRailCoverage =
                std::min({
                    result.minimumRailCoverage,
                    negativeCoverage,
                    positiveCoverage,
                });
            result.averageCandidateCoverage +=
                candidateCoverage * 0.25f;
            result.averageObservedCoverage +=
                observedCoverage * 0.25f;
            result.averageLocalObservedCoverage +=
                localObservedCoverage * 0.25f;
        }

        // Local two-sided evidence dominates. Global observed coverage remains
        // diagnostic only because remote supports/sleepers may legitimately
        // share the selected palette/remap channel.
        result.score =
            0.45f * result.averageCandidateCoverage
            + 0.30f * result.averageLocalObservedCoverage
            + 0.25f * result.minimumRailCoverage;
        return result;
    }

    [[nodiscard]] inline FirstPersonTrackProfileFit
        BestFirstPersonTrackProfileFit(
            const FirstPersonTrackArtworkObservation& observation,
            const std::array<FirstPersonSilhouette, 4>& candidate,
            const std::array<FirstPersonSilhouette, 4>& negativeRail,
            const std::array<FirstPersonSilhouette, 4>& positiveRail)
    {
        FirstPersonTrackProfileFit best{};
        for (size_t i = 0;
             i < kFirstPersonTrackPixelChannelCount; ++i)
        {
            const auto channel =
                static_cast<FirstPersonTrackPixelChannel>(i);
            const auto fit =
                EvaluateFirstPersonTrackProfileFit(
                    observation, candidate,
                    negativeRail, positiveRail, channel);
            if (!fit.valid)
                continue;
            if (!best.valid || fit.score > best.score)
                best = fit;
        }
        return best;
    }

    [[nodiscard]] inline float
        FirstPersonTrackRailContinuityCoverage(
            const FirstPersonTrackArtworkObservation& observation,
            const FirstPersonTrackTrajectory& trajectory,
            const FirstPersonVec3& anchor,
            const FirstPersonTrackRailProfile& profile,
            FirstPersonTrackPixelChannel channel)
    {
        if (!FirstPersonTrackObservationChannelComplete(
                observation, channel)
            || trajectory.points.empty())
            return 0.0f;

        const auto& observed =
            observation.channelViews[
                static_cast<size_t>(channel)];
        float minimum = 1.0f;
        const size_t stride = std::max<size_t>(
            1, trajectory.points.size() / 24);
        for (const float gaugeSide :
            { -profile.halfGauge, profile.halfGauge })
        {
            for (uint8_t rotation = 0;
                 rotation < 4; ++rotation)
            {
                size_t samples = 0;
                size_t supported = 0;
                for (size_t i = 0;
                     i < trajectory.points.size();
                     i += stride)
                {
                    const auto& point =
                        trajectory.points[i];
                    const FirstPersonVec3 centre{
                        point.position.x
                            + point.basis.right.x * gaugeSide
                            + point.basis.up.x
                                * profile.verticalOffset,
                        point.position.y
                            + point.basis.right.y * gaugeSide
                            + point.basis.up.y
                                * profile.verticalOffset,
                        point.position.z
                            + point.basis.right.z * gaugeSide
                            + point.basis.up.z
                                * profile.verticalOffset,
                    };
                    const auto projected =
                        ProjectFirstPersonTrackArtworkPoint(
                            rotation, anchor, centre);
                    ++samples;
                    if (FirstPersonTrackSilhouetteContainsNear(
                            observed[rotation],
                            projected.x, projected.y, 2))
                        ++supported;
                }
                if (samples == 0)
                    return 0.0f;
                minimum = std::min(
                    minimum,
                    float(supported)
                        / float(samples));
            }
        }
        return minimum;
    }

    [[nodiscard]] inline bool
        IsFirstPersonTrackCalibrationFitReliable(
            const FirstPersonTrackProfileFit& fit)
    {
        return fit.valid
            && fit.minimumCandidateCoverage >= 0.70f
            && fit.averageCandidateCoverage >= 0.78f
            && fit.minimumRailCoverage >= 0.65f
            && fit.averageLocalObservedCoverage >= 0.45f
            && fit.score >= 0.68f;
    }

    [[nodiscard]] inline bool
        IsFirstPersonTrackHoldoutFitReliable(
            const FirstPersonTrackProfileFit& fit)
    {
        // Continuity is intentionally a holdout-only discriminator. The source
        // optimizer never receives it as an objective, so a source fit cannot
        // optimise its way into passing this independent check.
        return fit.valid
            && fit.minimumCandidateCoverage >= 0.60f
            && fit.averageCandidateCoverage >= 0.68f
            && fit.minimumRailCoverage >= 0.55f
            && fit.averageLocalObservedCoverage >= 0.38f
            && fit.minimumContinuityCoverage >= 0.70f
            && fit.score >= 0.60f;
    }

    struct FirstPersonTrackProfileSearchState
    {
        enum class Phase : uint8_t
        {
            coarse,
            refine,
            complete,
        };

        bool initialized = false;
        Phase phase = Phase::coarse;
        size_t candidateIndex = 0;
        bool bestValid = false;
        float bestSelectionScore =
            -std::numeric_limits<float>::infinity();
        FirstPersonTrackRailProfile bestProfile{};
        FirstPersonTrackProfileFit bestFit{};
        FirstPersonTrackRailProfile refinementBase{};
        FirstPersonTrackProfileCalibrationResult result{};
    };

    inline void ConsiderFirstPersonTrackProfileCandidate(
        const FirstPersonTrackArtworkObservation& observation,
        const FirstPersonTrackTrajectory& trajectory,
        const FirstPersonVec3& anchor,
        FirstPersonTrackProfileSearchState& state,
        float halfGauge, float halfWidth,
        float halfHeight, float verticalOffset)
    {
        if (!(halfGauge >= 1.5f && halfGauge <= 15.0f)
            || !(halfWidth >= 0.35f && halfWidth <= 4.0f)
            || !(halfHeight >= 0.25f && halfHeight <= 4.0f)
            || halfGauge <= halfWidth + 0.5f
            || !(verticalOffset >= -20.0f
                && verticalOffset <= 20.0f))
            return;

        FirstPersonTrackRailProfile profile{};
        profile.halfGauge = halfGauge;
        profile.halfWidth = halfWidth;
        profile.halfHeight = halfHeight;
        profile.verticalOffset = verticalOffset;

        const auto candidate =
            BuildFirstPersonTrackRailSilhouettes(
                trajectory, anchor, profile);
        const auto negativeRail =
            BuildFirstPersonTrackSingleRailSilhouettes(
                trajectory, anchor, profile,
                -profile.halfGauge);
        const auto positiveRail =
            BuildFirstPersonTrackSingleRailSilhouettes(
                trajectory, anchor, profile,
                profile.halfGauge);
        const auto fit =
            BestFirstPersonTrackProfileFit(
                observation, candidate,
                negativeRail, positiveRail);
        if (!fit.valid)
            return;

        const float anisotropyPenalty =
            0.006f * std::abs(halfHeight - halfWidth);
        const float selectionScore =
            fit.score - anisotropyPenalty;
        const float oldComplexity =
            state.bestProfile.halfWidth
            + state.bestProfile.halfHeight
            + state.bestProfile.halfGauge * 0.01f;
        const float newComplexity =
            halfWidth + halfHeight
            + halfGauge * 0.01f;
        if (!state.bestValid
            || selectionScore
                > state.bestSelectionScore + 1e-5f
            || (std::abs(
                    selectionScore
                    - state.bestSelectionScore)
                    <= 1e-5f
                && newComplexity < oldComplexity))
        {
            state.bestValid = true;
            state.bestSelectionScore = selectionScore;
            state.bestProfile = profile;
            state.bestFit = fit;
        }
    }

    [[nodiscard]] inline bool
        StepFirstPersonTrackRailProfileFromArtwork(
            const FirstPersonTrackArtworkObservation& observation,
            const FirstPersonTrackTrajectory& trajectory,
            const FirstPersonVec3& anchor,
            FirstPersonTrackProfileSearchState& state,
            size_t candidateBudget)
    {
        constexpr size_t kCoarseGaugeCount = 7;
        constexpr size_t kCoarseWidthCount = 4;
        constexpr size_t kCoarseHeightCount = 4;
        constexpr size_t kCoarseOffsetCount = 9;
        constexpr size_t kCoarseCandidates =
            kCoarseGaugeCount * kCoarseWidthCount
            * kCoarseHeightCount * kCoarseOffsetCount;
        constexpr size_t kRefineGaugeCount = 5;
        constexpr size_t kRefineWidthCount = 7;
        constexpr size_t kRefineHeightCount = 7;
        constexpr size_t kRefineOffsetCount = 5;
        constexpr size_t kRefineCandidates =
            kRefineGaugeCount * kRefineWidthCount
            * kRefineHeightCount * kRefineOffsetCount;
        static constexpr std::array<float, 4>
            kCoarseHeights{ 0.5f, 1.0f, 2.0f, 3.0f };

        if (state.phase
            == FirstPersonTrackProfileSearchState::Phase::complete)
            return true;

        if (!state.initialized)
        {
            state.initialized = true;
            if (trajectory.points.size() < 2
                || !FirstPersonTrackObservationHasCompleteChannel(
                    observation))
            {
                state.phase =
                    FirstPersonTrackProfileSearchState::Phase::complete;
                return true;
            }
        }

        const auto completeSearch = [&]() {
            if (state.bestValid
                && IsFirstPersonTrackCalibrationFitReliable(
                    state.bestFit))
            {
                auto profile = state.bestProfile;
                profile.sourceChannelMask =
                    FirstPersonTrackPixelChannelBit(
                        state.bestFit.channel);
                DeriveFirstPersonTrackRailMaterial(
                    profile, observation, trajectory,
                    anchor, state.bestFit.channel);
                state.result.valid = true;
                state.result.profile = profile;
                state.result.fit = state.bestFit;
            }
            state.phase =
                FirstPersonTrackProfileSearchState::Phase::complete;
        };

        const auto advancePhase = [&]() {
            if (state.phase
                    == FirstPersonTrackProfileSearchState::Phase::coarse
                && state.candidateIndex >= kCoarseCandidates)
            {
                if (!state.bestValid)
                {
                    completeSearch();
                    return;
                }
                state.refinementBase = state.bestProfile;
                state.phase =
                    FirstPersonTrackProfileSearchState::Phase::refine;
                state.candidateIndex = 0;
            }
            if (state.phase
                    == FirstPersonTrackProfileSearchState::Phase::refine
                && state.candidateIndex >= kRefineCandidates)
            {
                completeSearch();
            }
        };

        advancePhase();
        while (candidateBudget > 0
            && state.phase
                != FirstPersonTrackProfileSearchState::Phase::complete)
        {
            size_t index = state.candidateIndex++;
            float halfGauge = 0.0f;
            float halfWidth = 0.0f;
            float halfHeight = 0.0f;
            float verticalOffset = 0.0f;

            if (state.phase
                == FirstPersonTrackProfileSearchState::Phase::coarse)
            {
                const size_t offsetIndex =
                    index % kCoarseOffsetCount;
                index /= kCoarseOffsetCount;
                const size_t heightIndex =
                    index % kCoarseHeightCount;
                index /= kCoarseHeightCount;
                const size_t widthIndex =
                    index % kCoarseWidthCount;
                index /= kCoarseWidthCount;
                const size_t gaugeIndex =
                    index % kCoarseGaugeCount;
                halfGauge =
                    2.0f + 2.0f * float(gaugeIndex);
                halfWidth =
                    0.5f + float(widthIndex);
                halfHeight =
                    kCoarseHeights[heightIndex];
                verticalOffset =
                    -16.0f + 4.0f * float(offsetIndex);
            }
            else
            {
                const size_t offsetIndex =
                    index % kRefineOffsetCount;
                index /= kRefineOffsetCount;
                const size_t heightIndex =
                    index % kRefineHeightCount;
                index /= kRefineHeightCount;
                const size_t widthIndex =
                    index % kRefineWidthCount;
                index /= kRefineWidthCount;
                const size_t gaugeIndex =
                    index % kRefineGaugeCount;
                halfGauge =
                    state.refinementBase.halfGauge
                    - 1.0f + 0.5f * float(gaugeIndex);
                halfWidth =
                    state.refinementBase.halfWidth
                    - 0.75f + 0.25f * float(widthIndex);
                halfHeight =
                    state.refinementBase.halfHeight
                    - 0.75f + 0.25f * float(heightIndex);
                verticalOffset =
                    state.refinementBase.verticalOffset
                    - 2.0f + float(offsetIndex);
            }

            ConsiderFirstPersonTrackProfileCandidate(
                observation, trajectory, anchor, state,
                halfGauge, halfWidth,
                halfHeight, verticalOffset);
            --candidateBudget;
            advancePhase();
        }
        return state.phase
            == FirstPersonTrackProfileSearchState::Phase::complete;
    }

    [[nodiscard]] inline FirstPersonTrackProfileCalibrationResult
        FitFirstPersonTrackRailProfileFromArtwork(
            const FirstPersonTrackArtworkObservation& observation,
            const FirstPersonTrackTrajectory& trajectory,
            const FirstPersonVec3& anchor)
    {
        FirstPersonTrackProfileSearchState state{};
        while (!StepFirstPersonTrackRailProfileFromArtwork(
            observation, trajectory, anchor, state, 256))
        {
        }
        return state.result;
    }

    [[nodiscard]] inline FirstPersonTrackProfileFit
        ValidateFirstPersonTrackRailProfileAgainstArtwork(
            const FirstPersonTrackArtworkObservation& observation,
            const FirstPersonTrackTrajectory& trajectory,
            const FirstPersonVec3& anchor,
            const FirstPersonTrackRailProfile& profile)
    {
        if (trajectory.points.size() < 2
            || !FirstPersonTrackObservationHasCompleteChannel(
                observation))
            return {};

        const auto candidate =
            BuildFirstPersonTrackRailSilhouettes(
                trajectory, anchor, profile);
        const auto negativeRail =
            BuildFirstPersonTrackSingleRailSilhouettes(
                trajectory, anchor, profile,
                -profile.halfGauge);
        const auto positiveRail =
            BuildFirstPersonTrackSingleRailSilhouettes(
                trajectory, anchor, profile,
                profile.halfGauge);

        FirstPersonTrackProfileFit best{};
        for (size_t i = 0;
             i < kFirstPersonTrackPixelChannelCount; ++i)
        {
            const auto channel =
                static_cast<FirstPersonTrackPixelChannel>(i);
            if ((profile.sourceChannelMask
                    & FirstPersonTrackPixelChannelBit(channel))
                == 0)
                continue;
            auto fit =
                EvaluateFirstPersonTrackProfileFit(
                    observation, candidate,
                    negativeRail, positiveRail, channel);
            if (!fit.valid)
                continue;
            fit.minimumContinuityCoverage =
                FirstPersonTrackRailContinuityCoverage(
                    observation, trajectory, anchor,
                    profile, channel);
            if (!best.valid || fit.score > best.score)
                best = fit;
        }
        return best;
    }
} // namespace OpenRCT2::Paint
