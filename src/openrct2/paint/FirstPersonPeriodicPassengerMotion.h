/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include "../drawing/Drawing.Sprite.h"
#include "../ride/RideEntry.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace OpenRCT2::Paint
{
    constexpr uint16_t kFirstPersonFerrisWheelFrameCount = 128;
    constexpr uint16_t kFirstPersonFerrisWheelSampleStride = 4;
    constexpr size_t kFirstPersonFerrisWheelSampleCount =
        kFirstPersonFerrisWheelFrameCount / kFirstPersonFerrisWheelSampleStride;

    struct FirstPersonPeriodicScreenObservation
    {
        bool valid = false;
        float x{};
        float y{};
        float width{};
        float height{};
    };

    using FirstPersonFerrisWheelObservationSet =
        std::array<std::array<FirstPersonPeriodicScreenObservation, kFirstPersonFerrisWheelSampleCount>, 4>;

    struct FirstPersonPeriodicOrbitCalibration
    {
        bool valid = false;

        // Seat-pair orbit relative to the common native Ferris-wheel painter
        // origin for object direction zero.
        float centerX{};
        float centerY{};
        float centerZ{};
        float orbitCosX{};
        float orbitCosZ{};
        float orbitSinX{};
        float orbitSinZ{};
        float radius{};

        // Artwork-derived cabin dimensions. These are deliberately estimates,
        // not simulation truth, and retain uncertainty for diagnostics/fallback.
        float seatHalfSeparation{};
        float eyeHeight{};
        float centerUncertainty{};
        float radiusUncertainty{};
        float seatSeparationUncertainty{};
        float eyeHeightUncertainty{};
        float trainingRmse{};
        float validationRmse{};
    };

    struct FirstPersonPeriodicOrbitPoint
    {
        float x{};
        float y{};
        float z{};
    };

    [[nodiscard]] inline float FirstPersonWrapPeriodicFrame(float frame, float frameCount)
    {
        if (!(frameCount > 0.0f) || !std::isfinite(frame))
            return 0.0f;
        frame = std::fmod(frame, frameCount);
        if (frame < 0.0f)
            frame += frameCount;
        return frame;
    }

    [[nodiscard]] inline float FirstPersonFerrisWheelRiderPhase(
        float primaryFrame, uint8_t seatIndex)
    {
        // PaintFerrisWheelRiders advances one 128-frame phase by four frames
        // per seat index. The painter iterates i += 2, so both passengers in a
        // rider pair occupy the same gondola phase.
        const uint8_t pairSeat = seatIndex & 0xFEu;
        return FirstPersonWrapPeriodicFrame(
            primaryFrame + float(pairSeat) * 4.0f,
            float(kFirstPersonFerrisWheelFrameCount));
    }

    [[nodiscard]] inline FirstPersonPeriodicOrbitPoint
        FirstPersonFerrisWheelSeatBaseOffsetFromVehicle(
            FirstPersonPeriodicOrbitPoint painterLocalSeat,
            uint8_t objectDirection, float vehicleZOffset)
    {
        // Sequence 0 is the track origin for FlatTrack1x4C. Its Ferris painter
        // origin is (-16,0,+7), rotated with the ride. The integral vehicle,
        // however, is always created at the sequence-0 tile centre (+16,+16)
        // rather than by rotating that centre offset. Keep those two transforms
        // separate or directions 1-3 shift the whole recovered wheel.
        const float painterX = painterLocalSeat.x - 16.0f;
        const float painterY = painterLocalSeat.y;
        float rotatedX = painterX;
        float rotatedY = painterY;
        switch (objectDirection & 3u)
        {
            case 1:
                rotatedX = painterY;
                rotatedY = -painterX;
                break;
            case 2:
                rotatedX = -painterX;
                rotatedY = -painterY;
                break;
            case 3:
                rotatedX = -painterY;
                rotatedY = painterX;
                break;
            default:
                break;
        }
        return {
            rotatedX - 16.0f,
            rotatedY - 16.0f,
            painterLocalSeat.z + 7.0f - vehicleZOffset,
        };
    }

    [[nodiscard]] inline FirstPersonPeriodicOrbitPoint SampleFirstPersonPeriodicOrbit(
        const FirstPersonPeriodicOrbitCalibration& calibration, float phase)
    {
        constexpr float kTwoPi = 6.28318530717958647692f;
        const float angle = FirstPersonWrapPeriodicFrame(
            phase, float(kFirstPersonFerrisWheelFrameCount))
            * (kTwoPi / float(kFirstPersonFerrisWheelFrameCount));
        const float c = std::cos(angle);
        const float s = std::sin(angle);
        return {
            calibration.centerX + calibration.orbitCosX * c + calibration.orbitSinX * s,
            calibration.centerY,
            calibration.centerZ + calibration.orbitCosZ * c + calibration.orbitSinZ * s,
        };
    }

    namespace Detail
    {
        struct PeriodicHarmonicFit
        {
            bool valid = false;
            float meanX{};
            float meanY{};
            float cosX{};
            float cosY{};
            float sinX{};
            float sinY{};
            float rmse{};
        };

        [[nodiscard]] inline std::array<float, 2> ProjectFerrisLocal(
            uint8_t rotation, float x, float y, float z)
        {
            switch (rotation & 3)
            {
                default:
                case 0: return { y - x, 0.5f * (x + y) - z };
                case 1: return { -x - y, 0.5f * (y - x) - z };
                case 2: return { x - y, -0.5f * (x + y) - z };
                case 3: return { x + y, 0.5f * (x - y) - z };
            }
        }

        [[nodiscard]] inline PeriodicHarmonicFit FitPeriodicHarmonic(
            const std::array<FirstPersonPeriodicScreenObservation, kFirstPersonFerrisWheelSampleCount>& observations,
            int sampleParity = -1)
        {
            PeriodicHarmonicFit result{};
            size_t count = 0;
            for (size_t i = 0; i < observations.size(); ++i)
            {
                if (sampleParity >= 0 && int(i & 1u) != sampleParity)
                    continue;
                const auto& observation = observations[i];
                if (!observation.valid || !std::isfinite(observation.x)
                    || !std::isfinite(observation.y))
                    return result;
                result.meanX += observation.x;
                result.meanY += observation.y;
                ++count;
            }
            if (count < 8)
                return result;
            const float countF = float(count);
            result.meanX /= countF;
            result.meanY /= countF;

            constexpr float kTwoPi = 6.28318530717958647692f;
            for (size_t i = 0; i < observations.size(); ++i)
            {
                if (sampleParity >= 0 && int(i & 1u) != sampleParity)
                    continue;
                const float phase = float(i * kFirstPersonFerrisWheelSampleStride);
                const float angle = phase
                    * (kTwoPi / float(kFirstPersonFerrisWheelFrameCount));
                const float c = std::cos(angle);
                const float s = std::sin(angle);
                result.cosX += observations[i].x * c;
                result.cosY += observations[i].y * c;
                result.sinX += observations[i].x * s;
                result.sinY += observations[i].y * s;
            }
            result.cosX *= 2.0f / countF;
            result.cosY *= 2.0f / countF;
            result.sinX *= 2.0f / countF;
            result.sinY *= 2.0f / countF;

            float error2 = 0.0f;
            for (size_t i = 0; i < observations.size(); ++i)
            {
                if (sampleParity >= 0 && int(i & 1u) != sampleParity)
                    continue;
                const float phase = float(i * kFirstPersonFerrisWheelSampleStride);
                const float angle = phase
                    * (kTwoPi / float(kFirstPersonFerrisWheelFrameCount));
                const float c = std::cos(angle);
                const float s = std::sin(angle);
                const float dx = observations[i].x
                    - (result.meanX + result.cosX * c + result.sinX * s);
                const float dy = observations[i].y
                    - (result.meanY + result.cosY * c + result.sinY * s);
                error2 += dx * dx + dy * dy;
            }
            result.rmse = std::sqrt(error2 / countF);
            result.valid = std::isfinite(result.rmse);
            return result;
        }

        [[nodiscard]] inline float EvaluatePeriodicHarmonic(
            const PeriodicHarmonicFit& fit,
            const std::array<FirstPersonPeriodicScreenObservation, kFirstPersonFerrisWheelSampleCount>& observations,
            int sampleParity)
        {
            if (!fit.valid)
                return std::numeric_limits<float>::infinity();
            constexpr float kTwoPi = 6.28318530717958647692f;
            float error2 = 0.0f;
            size_t count = 0;
            for (size_t i = 0; i < observations.size(); ++i)
            {
                if (int(i & 1u) != sampleParity)
                    continue;
                if (!observations[i].valid)
                    return std::numeric_limits<float>::infinity();
                const float phase = float(i * kFirstPersonFerrisWheelSampleStride);
                const float angle = phase
                    * (kTwoPi / float(kFirstPersonFerrisWheelFrameCount));
                const float c = std::cos(angle);
                const float s = std::sin(angle);
                const float dx = observations[i].x
                    - (fit.meanX + fit.cosX * c + fit.sinX * s);
                const float dy = observations[i].y
                    - (fit.meanY + fit.cosY * c + fit.sinY * s);
                error2 += dx * dx + dy * dy;
                ++count;
            }
            return count != 0
                ? std::sqrt(error2 / float(count))
                : std::numeric_limits<float>::infinity();
        }

        struct LocalCoefficient
        {
            float x{};
            float y{};
            float z{};
        };

        [[nodiscard]] inline LocalCoefficient RecoverFromFirstTwoViews(
            float sx0, float sy0, float sx1, float sy1)
        {
            LocalCoefficient result{};
            result.x = -0.5f * (sx0 + sx1);
            result.y = 0.5f * (sx0 - sx1);
            const float z0 = 0.5f * (result.x + result.y) - sy0;
            const float z1 = 0.5f * (result.y - result.x) - sy1;
            result.z = 0.5f * (z0 + z1);
            return result;
        }

        [[nodiscard]] inline LocalCoefficient RecoverFromFourViews(
            const std::array<PeriodicHarmonicFit, 4>& fits,
            float PeriodicHarmonicFit::* screenX,
            float PeriodicHarmonicFit::* screenY)
        {
            const float sx0 = fits[0].*screenX;
            const float sx1 = fits[1].*screenX;
            const float sx2 = fits[2].*screenX;
            const float sx3 = fits[3].*screenX;
            const float xyDifference = 0.5f * (sx2 - sx0);
            const float xySum = 0.5f * (sx3 - sx1);

            LocalCoefficient result{};
            result.x = 0.5f * (xyDifference + xySum);
            result.y = 0.5f * (xySum - xyDifference);
            result.z = -0.25f * (
                fits[0].*screenY + fits[1].*screenY
                + fits[2].*screenY + fits[3].*screenY);
            return result;
        }

        [[nodiscard]] inline float Median(std::vector<float> values)
        {
            if (values.empty())
                return 0.0f;
            const size_t middle = values.size() / 2;
            std::nth_element(values.begin(), values.begin() + middle, values.end());
            float result = values[middle];
            if ((values.size() & 1u) == 0)
            {
                const auto lower = std::max_element(values.begin(), values.begin() + middle);
                result = 0.5f * (result + *lower);
            }
            return result;
        }

        [[nodiscard]] inline float MedianAbsoluteDeviation(
            const std::vector<float>& values, float median)
        {
            std::vector<float> deviations;
            deviations.reserve(values.size());
            for (const float value : values)
                deviations.push_back(std::abs(value - median));
            return Median(std::move(deviations));
        }

        [[nodiscard]] inline bool ExtractFerrisRiderObservation(
            const G1Element& g1, FirstPersonPeriodicScreenObservation& observation)
        {
            if (g1.offset == nullptr || g1.width <= 0 || g1.height <= 0
                || g1.width > 512 || g1.height > 512
                || g1.flags.has(G1Flag::isPalette))
                return false;

            int32_t minX = g1.width;
            int32_t minY = g1.height;
            int32_t maxX = -1;
            int32_t maxY = -1;
            const auto addPixel = [&](int32_t x, int32_t y) {
                minX = std::min(minX, x);
                minY = std::min(minY, y);
                maxX = std::max(maxX, x);
                maxY = std::max(maxY, y);
            };

            if (g1.flags.has(G1Flag::hasRLECompression))
            {
                for (int32_t y = 0; y < g1.height; ++y)
                {
                    const uint16_t lineOffset =
                        uint16_t(g1.offset[y * 2])
                        | (uint16_t(g1.offset[y * 2 + 1]) << 8);
                    const uint8_t* run = g1.offset + lineOffset;
                    bool endOfLine = false;
                    size_t runGuard = 0;
                    while (!endOfLine && runGuard++ < 256)
                    {
                        uint8_t length = *run++;
                        const int32_t x = *run++;
                        endOfLine = (length & 0x80u) != 0;
                        length &= 0x7Fu;
                        for (uint8_t n = 0; n < length; ++n)
                            addPixel(x + n, y);
                        run += length;
                    }
                    if (!endOfLine)
                        return false;
                }
            }
            else
            {
                const bool transparent = g1.flags.has(G1Flag::hasTransparency);
                for (int32_t y = 0; y < g1.height; ++y)
                for (int32_t x = 0; x < g1.width; ++x)
                {
                    const uint8_t pixel =
                        g1.offset[size_t(y) * size_t(g1.width) + size_t(x)];
                    if (transparent && pixel == 0)
                        continue;
                    addPixel(x, y);
                }
            }

            if (maxX < minX || maxY < minY)
                return false;
            const float width = float(maxX - minX + 1);
            const float height = float(maxY - minY + 1);

            // The phase fit uses a lower-body seat marker rather than declaring
            // the changing silhouette centroid to be the passenger eye. Eye
            // height is calibrated separately from the observed body extent.
            constexpr float kSeatMarkerFractionFromTop = 0.72f;
            observation.valid = true;
            observation.x = float(g1.xOffset) + 0.5f * float(minX + maxX + 1);
            observation.y = float(g1.yOffset) + float(minY)
                + kSeatMarkerFractionFromTop * height;
            observation.width = width;
            observation.height = height;
            return true;
        }
    } // namespace Detail

    [[nodiscard]] inline FirstPersonPeriodicOrbitCalibration FitFirstPersonFerrisWheelOrbit(
        const FirstPersonFerrisWheelObservationSet& observations)
    {
        FirstPersonPeriodicOrbitCalibration result{};
        std::array<Detail::PeriodicHarmonicFit, 4> fullFits{};
        for (size_t direction = 0; direction < fullFits.size(); ++direction)
        {
            fullFits[direction] = Detail::FitPeriodicHarmonic(observations[direction]);
            if (!fullFits[direction].valid)
                return result;
        }

        const auto train0 = Detail::FitPeriodicHarmonic(observations[0], 0);
        const auto train1 = Detail::FitPeriodicHarmonic(observations[1], 0);
        if (!train0.valid || !train1.valid)
            return result;
        const float holdout0 =
            Detail::EvaluatePeriodicHarmonic(train0, observations[0], 1);
        const float holdout1 =
            Detail::EvaluatePeriodicHarmonic(train1, observations[1], 1);
        result.trainingRmse = std::sqrt(
            0.5f * (holdout0 * holdout0 + holdout1 * holdout1));

        const auto trainCos = Detail::RecoverFromFirstTwoViews(
            train0.cosX, train0.cosY, train1.cosX, train1.cosY);
        const auto trainSin = Detail::RecoverFromFirstTwoViews(
            train0.sinX, train0.sinY, train1.sinX, train1.sinY);

        float validationError2 = 0.0f;
        size_t validationTerms = 0;
        for (uint8_t direction = 2; direction < 4; ++direction)
        {
            const auto projectedCos = Detail::ProjectFerrisLocal(
                direction, trainCos.x, trainCos.y, trainCos.z);
            const auto projectedSin = Detail::ProjectFerrisLocal(
                direction, trainSin.x, trainSin.y, trainSin.z);
            const std::array<float, 4> errors{
                projectedCos[0] - fullFits[direction].cosX,
                projectedCos[1] - fullFits[direction].cosY,
                projectedSin[0] - fullFits[direction].sinX,
                projectedSin[1] - fullFits[direction].sinY,
            };
            for (const float error : errors)
            {
                validationError2 += error * error;
                ++validationTerms;
            }
        }
        result.validationRmse = validationTerms != 0
            ? std::sqrt(validationError2 / float(validationTerms))
            : std::numeric_limits<float>::infinity();

        const auto center = Detail::RecoverFromFourViews(
            fullFits, &Detail::PeriodicHarmonicFit::meanX,
            &Detail::PeriodicHarmonicFit::meanY);
        const auto cosCoeff = Detail::RecoverFromFourViews(
            fullFits, &Detail::PeriodicHarmonicFit::cosX,
            &Detail::PeriodicHarmonicFit::cosY);
        const auto sinCoeff = Detail::RecoverFromFourViews(
            fullFits, &Detail::PeriodicHarmonicFit::sinX,
            &Detail::PeriodicHarmonicFit::sinY);

        float centerError2 = 0.0f;
        for (uint8_t direction = 0; direction < 4; ++direction)
        {
            const auto projectedCenter = Detail::ProjectFerrisLocal(
                direction, center.x, center.y, center.z);
            const float dx = projectedCenter[0] - fullFits[direction].meanX;
            const float dy = projectedCenter[1] - fullFits[direction].meanY;
            centerError2 += dx * dx + dy * dy;
        }
        result.centerUncertainty = std::sqrt(centerError2 / 8.0f);

        const float cosNorm = std::hypot(cosCoeff.x, cosCoeff.z);
        const float sinNorm = std::hypot(sinCoeff.x, sinCoeff.z);
        const float radius = 0.5f * (cosNorm + sinNorm);
        if (!(radius >= 12.0f && radius <= 96.0f)
            || cosNorm < 1e-4f || sinNorm < 1e-4f)
            return result;

        const float dot =
            cosCoeff.x * sinCoeff.x + cosCoeff.z * sinCoeff.z;
        const float circleError = std::abs(cosNorm - sinNorm)
            + std::abs(dot) / std::max(radius, 1.0f);
        const float offPlane = std::hypot(cosCoeff.y, sinCoeff.y);
        if (!std::isfinite(result.trainingRmse)
            || !std::isfinite(result.validationRmse)
            || result.trainingRmse > 5.0f
            || result.validationRmse > 6.0f
            || result.centerUncertainty > 10.0f
            || circleError > std::max(8.0f, radius * 0.25f)
            || offPlane > std::max(6.0f, radius * 0.18f))
            return result;

        result.centerX = center.x;
        result.centerY = center.y;
        result.centerZ = center.z;
        result.radius = radius;

        // Project the noisy first-harmonic ellipse onto the nearest circular
        // orbit in the known X/Z wheel plane while retaining native phase and
        // direction of travel.
        const float ux = cosCoeff.x / cosNorm;
        const float uz = cosCoeff.z / cosNorm;
        result.orbitCosX = ux * radius;
        result.orbitCosZ = uz * radius;
        const float determinant =
            cosCoeff.x * sinCoeff.z - cosCoeff.z * sinCoeff.x;
        const float handedness = determinant >= 0.0f ? 1.0f : -1.0f;
        result.orbitSinX = handedness * -uz * radius;
        result.orbitSinZ = handedness * ux * radius;

        std::vector<float> widths;
        std::vector<float> heights;
        widths.reserve(4 * kFirstPersonFerrisWheelSampleCount);
        heights.reserve(4 * kFirstPersonFerrisWheelSampleCount);
        for (const auto& view : observations)
        for (const auto& observation : view)
        {
            if (!observation.valid)
                continue;
            widths.push_back(observation.width);
            heights.push_back(observation.height);
        }
        const float medianWidth = Detail::Median(widths);
        const float medianHeight = Detail::Median(heights);
        const float widthMad =
            Detail::MedianAbsoluteDeviation(widths, medianWidth);
        const float heightMad =
            Detail::MedianAbsoluteDeviation(heights, medianHeight);

        result.seatHalfSeparation =
            std::clamp(medianWidth * 0.18f, 2.0f, 6.0f);
        result.eyeHeight =
            std::clamp(medianHeight * 0.55f, 6.0f, 18.0f);
        result.centerUncertainty = std::max(
            result.centerUncertainty,
            std::max(result.trainingRmse, result.validationRmse));
        result.radiusUncertainty = circleError + result.validationRmse;
        result.seatSeparationUncertainty =
            std::max(0.5f, widthMad * 0.18f + medianWidth * 0.08f);
        result.eyeHeightUncertainty =
            std::max(1.0f, heightMad * 0.55f + medianHeight * 0.10f);
        result.valid = true;
        return result;
    }

    [[nodiscard]] inline FirstPersonFerrisWheelObservationSet
        CollectFirstPersonFerrisWheelObservations(const RideObjectEntry& rideEntry)
    {
        FirstPersonFerrisWheelObservationSet result{};
        constexpr size_t kMaxDecodedPixels = 1048576;
        size_t decodedPixels = 0;
        const uint32_t baseImage = rideEntry.Cars[0].baseImageId;
        for (uint8_t direction = 0; direction < 4; ++direction)
        for (size_t sample = 0; sample < kFirstPersonFerrisWheelSampleCount; ++sample)
        {
            const uint32_t frame =
                uint32_t(sample * kFirstPersonFerrisWheelSampleStride);
            const uint32_t imageIndex =
                baseImage + 32u + uint32_t(direction) * 128u + frame;
            const auto* g1 = ::GfxGetG1Element(imageIndex);
            if (g1 == nullptr || g1->width <= 0 || g1->height <= 0)
                return {};
            decodedPixels += size_t(g1->width) * size_t(g1->height);
            if (decodedPixels > kMaxDecodedPixels)
                return {};
            if (!Detail::ExtractFerrisRiderObservation(
                    *g1, result[direction][sample]))
                return {};
        }
        return result;
    }

    [[nodiscard]] inline const FirstPersonPeriodicOrbitCalibration*
        GetFirstPersonFerrisWheelCalibration(const RideObjectEntry& rideEntry)
    {
        struct CacheEntry
        {
            const uint8_t* sourceIdentity = nullptr;
            FirstPersonPeriodicOrbitCalibration calibration{};
        };
        static std::unordered_map<uint32_t, CacheEntry> cache;

        const uint32_t baseImage = rideEntry.Cars[0].baseImageId;
        const auto* first = ::GfxGetG1Element(baseImage + 32u);
        if (first == nullptr || first->offset == nullptr)
            return nullptr;

        auto& cached = cache[baseImage];
        if (cached.sourceIdentity != first->offset)
        {
            cached.sourceIdentity = first->offset;
            cached.calibration = FitFirstPersonFerrisWheelOrbit(
                CollectFirstPersonFerrisWheelObservations(rideEntry));
        }
        return cached.calibration.valid ? &cached.calibration : nullptr;
    }
} // namespace OpenRCT2::Paint
