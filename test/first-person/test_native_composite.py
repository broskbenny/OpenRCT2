"""Compare the production composite with the original per-sequence lookup."""
from pathlib import Path
import json
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]

SOURCE = r'''
#include "src/openrct2/paint/FirstPersonNativeComposite.h"
#include <cassert>
#include <chrono>
#include <iostream>
#include <limits>
#include <random>
using namespace OpenRCT2::Paint;

FirstPersonNativeCompositeSample reference(
    const std::vector<FirstPersonNativeCompositeSource>& sources, int32_t x, int32_t y)
{
    bool covered = false;
    const FirstPersonNativeCompositeSource* best = nullptr;
    uint8_t pixel = 0;
    for (const auto& source : sources)
    {
        const int64_t u = int64_t(x) - source.x, v = int64_t(y) - source.y;
        if (u < 0 || v < 0 || u >= source.width || v >= source.height)
            continue;
        covered = true;
        const uint8_t value = (*source.pixels)[size_t(v) * size_t(source.width) + size_t(u)];
        if (value != 0 && (best == nullptr || source.depth > best->depth))
        {
            best = &source;
            pixel = value;
        }
    }
    return {pixel != 0 ? FirstPersonNativeCompositeKind::colour
                      : covered ? FirstPersonNativeCompositeKind::transparent
                                : FirstPersonNativeCompositeKind::unknown, pixel};
}

int main()
{
    std::mt19937 random(24136);
    size_t comparisons = 0;
    std::vector<std::vector<uint8_t>> images(32);
    std::vector<FirstPersonNativeCompositeSource> sources;
    for (size_t i = 0; i < images.size(); ++i)
    {
        const int32_t width = 1 + int32_t(random() % 64), height = 1 + int32_t(random() % 64);
        images[i].resize(size_t(width) * size_t(height));
        for (auto& pixel : images[i])
            pixel = random() % 3 == 0 ? 0 : uint8_t(1 + random() % 255);
        sources.push_back({int32_t(random() % 80) - 40, int32_t(random() % 80) - 40,
                           width, height, float(i % 5), &images[i]});
    }
    FirstPersonNativeComposite composite;
    composite.build(sources);
    assert(composite.cachedPixelCount() != 0);
    for (int32_t y = -45; y < 110; ++y)
    for (int32_t x = -45; x < 110; ++x)
    {
        const auto actual = composite.sample(x, y), expected = reference(sources, x, y);
        assert(actual.kind == expected.kind && actual.pixel == expected.pixel);
        ++comparisons;
    }
    composite.build(sources, 1);
    assert(composite.cachedPixelCount() == 0);
    for (int32_t y = -45; y < 110; ++y)
    for (int32_t x = -45; x < 110; ++x)
    {
        const auto actual = composite.sample(x, y), expected = reference(sources, x, y);
        assert(actual.kind == expected.kind && actual.pixel == expected.pixel);
        ++comparisons;
    }

    // Distinct transparent rectangles must not turn the intervening gap into
    // transparent evidence. Extreme coordinates and huge spans do not allocate.
    std::vector<uint8_t> alpha(4, 0), colour{42, 0, 51, 61};
    sources = {{-10, -10, 2, 2, 0, &alpha}, {10, 10, 2, 2, 0, &colour}};
    composite.build(sources);
    assert(composite.sample(-10, -10).kind == FirstPersonNativeCompositeKind::transparent);
    assert(composite.sample(0, 0).kind == FirstPersonNativeCompositeKind::unknown);
    assert(composite.sample(10, 10).pixel == 42);
    sources.push_back({std::numeric_limits<int32_t>::max() - 1,
                       std::numeric_limits<int32_t>::min(), 2, 2, 3, &colour});
    composite.build(sources);
    assert(composite.cachedPixelCount() == 0);
    for (const auto& point : {std::pair{-10, -10}, std::pair{0, 0}, std::pair{10, 10},
                            std::pair{std::numeric_limits<int32_t>::max() - 1,
                                      std::numeric_limits<int32_t>::min()}})
    {
        const auto actual = composite.sample(point.first, point.second);
        const auto expected = reference(sources, point.first, point.second);
        assert(actual.kind == expected.kind && actual.pixel == expected.pixel);
        ++comparisons;
    }
    composite.build({});
    assert(composite.sample(0, 0).kind == FirstPersonNativeCompositeKind::unknown);

    // Include construction in a repeatable cold-bake lookup benchmark.
    // Timing is reported, never used as a correctness assertion.
    sources.clear();
    for (size_t i = 0; i < images.size(); ++i)
    {
        images[i].assign(64 * 64, 0);
        for (size_t p = i; p < images[i].size(); p += images.size())
            images[i][p] = uint8_t(1 + p % 200);
        sources.push_back({0, 0, 64, 64, float(i), &images[i]});
    }
    std::vector<std::pair<int32_t, int32_t>> lookups;
    for (size_t i = 0; i < 200000; ++i)
        lookups.push_back({int32_t(random() % 64), int32_t(random() % 64)});
    const auto start = std::chrono::steady_clock::now();
    uint64_t referenceSum = 0, cachedSum = 0;
    for (const auto& [x, y] : lookups)
        referenceSum += reference(sources, x, y).pixel;
    const auto middle = std::chrono::steady_clock::now();
    composite.build(sources);
    for (const auto& [x, y] : lookups)
        cachedSum += composite.sample(x, y).pixel;
    const auto finish = std::chrono::steady_clock::now();
    assert(referenceSum == cachedSum);
    std::cout << "{\"comparisons\":" << comparisons
              << ",\"lookups\":" << lookups.size()
              << ",\"reference_ms\":" << std::chrono::duration<double, std::milli>(middle - start).count()
              << ",\"cached_ms_including_build\":" << std::chrono::duration<double, std::milli>(finish - middle).count()
              << ",\"cache_bytes\":" << 2 * composite.cachedPixelCount() << "}\n";
}
'''


def exercise(sanitize=True):
    with tempfile.TemporaryDirectory(prefix="fp-native-composite-") as folder:
        source, binary = Path(folder) / "test.cpp", Path(folder) / "test"
        source.write_text(SOURCE)
        command = ["g++", "-std=c++20", "-O2", "-Wall", "-Wextra", "-Werror",
                   "-pedantic", "-I", str(ROOT), str(source), "-o", str(binary)]
        if sanitize:
            command += ["-fsanitize=undefined", "-fno-sanitize-recover=all"]
        subprocess.run(command, check=True)
        return json.loads(subprocess.check_output([str(binary)]))


class NativeCompositeTest(unittest.TestCase):
    @unittest.skipUnless(shutil.which("g++"), "Native compiler unavailable")
    def test_palette_alpha_depth_ties_gaps_and_memory_fallback(self):
        result = exercise()
        self.assertEqual(result["comparisons"], 48054)


if __name__ == "__main__":
    print(json.dumps(exercise(sanitize=False), indent=2))
