/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace OpenRCT2::Paint
{
    enum class FirstPersonNativeCompositeKind : uint8_t
    {
        unknown,
        transparent,
        colour,
    };

    struct FirstPersonNativeCompositeSample
    {
        FirstPersonNativeCompositeKind kind = FirstPersonNativeCompositeKind::unknown;
        uint8_t pixel = 0;
    };

    struct FirstPersonNativeCompositeSource
    {
        int32_t x{}, y{}, width{}, height{};
        float depth{};
        const std::vector<uint8_t>* pixels{};
    };

    // The large-scenery baker's existing native-depth composite, evaluated
    // once per view rather than searching every tile for every sub-sample.
    // Coverage is separate from colour: uncovered gaps are unknown, whereas
    // covered palette-zero pixels remain authored transparency. Equal-depth
    // overlaps retain the first sequence, exactly as the original sampler did.
    // Oversized views use the same bounded-memory direct lookup.
    class FirstPersonNativeComposite
    {
    public:
        static constexpr size_t kMaximumPixels = 262144;

        void build(
            std::vector<FirstPersonNativeCompositeSource> sources, size_t expectedSamples = std::numeric_limits<size_t>::max())
        {
            _pixels.clear();
            _covered.clear();
            _sources = std::move(sources);
            _sources.erase(
                std::remove_if(
                    _sources.begin(), _sources.end(),
                    [](const auto& source) {
                        return source.pixels == nullptr || source.width <= 0 || source.height <= 0
                            || uint64_t(source.width) * uint64_t(source.height) != source.pixels->size();
                    }),
                _sources.end());
            if (_sources.empty())
                return;

            int64_t lowX = _sources.front().x, lowY = _sources.front().y;
            int64_t highX = lowX, highY = lowY;
            uint64_t paintedPixels = 0;
            for (const auto& source : _sources)
            {
                lowX = std::min(lowX, int64_t(source.x));
                lowY = std::min(lowY, int64_t(source.y));
                highX = std::max(highX, int64_t(source.x) + source.width);
                highY = std::max(highY, int64_t(source.y) + source.height);
                paintedPixels += uint64_t(source.width) * uint64_t(source.height);
            }
            const uint64_t width = uint64_t(highX - lowX), height = uint64_t(highY - lowY);
            if (width > kMaximumPixels || height > kMaximumPixels || width * height > kMaximumPixels
                || paintedPixels > kMaximumPixels * 16)
                return;
            // A few small faces do not amortize painting a large source atlas.
            // Keep their original direct sampler instead of adding cold work.
            if ((paintedPixels + _sources.size() - 1) / _sources.size() > expectedSamples)
                return;
            std::stable_sort(_sources.begin(), _sources.end(), [](const auto& a, const auto& b) { return a.depth > b.depth; });

            _x = lowX;
            _y = lowY;
            _width = int32_t(width);
            _height = int32_t(height);
            _pixels.assign(size_t(width * height), 0);
            _covered.assign(_pixels.size(), 0);
            for (const auto& source : _sources)
            {
                for (int32_t y = 0; y < source.height; ++y)
                    for (int32_t x = 0; x < source.width; ++x)
                    {
                        const size_t destination = size_t(int64_t(source.y) + y - _y) * size_t(_width)
                            + size_t(int64_t(source.x) + x - _x);
                        _covered[destination] = 1;
                        if (_pixels[destination] == 0)
                            _pixels[destination] = (*source.pixels)[size_t(y) * size_t(source.width) + size_t(x)];
                    }
            }
        }

        [[nodiscard]] FirstPersonNativeCompositeSample sample(int32_t x, int32_t y) const
        {
            if (!_pixels.empty())
            {
                const int64_t u = int64_t(x) - _x, v = int64_t(y) - _y;
                if (u < 0 || v < 0 || u >= _width || v >= _height)
                    return {};
                const size_t index = size_t(v) * size_t(_width) + size_t(u);
                return classify(_covered[index] != 0, _pixels[index]);
            }

            bool covered = false;
            const FirstPersonNativeCompositeSource* best = nullptr;
            uint8_t bestPixel = 0;
            for (const auto& source : _sources)
            {
                const int64_t u = int64_t(x) - source.x, v = int64_t(y) - source.y;
                if (u < 0 || v < 0 || u >= source.width || v >= source.height)
                    continue;
                covered = true;
                const uint8_t pixel = (*source.pixels)[size_t(v) * size_t(source.width) + size_t(u)];
                if (pixel != 0 && (best == nullptr || source.depth > best->depth))
                {
                    best = &source;
                    bestPixel = pixel;
                }
            }
            return classify(covered, bestPixel);
        }

        [[nodiscard]] size_t cachedPixelCount() const
        {
            return _pixels.size();
        }

    private:
        [[nodiscard]] static FirstPersonNativeCompositeSample classify(bool covered, uint8_t pixel)
        {
            if (pixel != 0)
                return { FirstPersonNativeCompositeKind::colour, pixel };
            return { covered ? FirstPersonNativeCompositeKind::transparent : FirstPersonNativeCompositeKind::unknown, 0 };
        }

        int64_t _x{}, _y{};
        int32_t _width{}, _height{};
        std::vector<uint8_t> _pixels;
        std::vector<uint8_t> _covered;
        std::vector<FirstPersonNativeCompositeSource> _sources;
    };
} // namespace OpenRCT2::Paint
