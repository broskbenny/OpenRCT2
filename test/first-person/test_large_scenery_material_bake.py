"""Compile exact large-scenery material blocks; check the rendered UV footprint.

Synthetic decoded sprites isolate texture sampling from geometry inference.
This exercises production visibility, projection, compositing, evidence and
alpha decisions. It does not run DAT decoding, full scene capture or the GPU.
An optional --core reads an earlier Core.inc for a reproducible comparison.
"""
from pathlib import Path
import argparse
import json
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
PAINT = ROOT / "src/openrct2/paint"


def definition(source, marker):
    start = source.index(marker)
    opening = source.index("{", start)
    level, end = 1, opening + 1
    while level:
        level += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end] + ";\n"


def between(source, start, end):
    return source[source.index(start):source.index(end, source.index(start))]


ADAPTER = r'''
#include "src/openrct2/paint/FirstPersonMaterialInference.h"
#include "src/openrct2/paint/FirstPersonMaterialContinuity.h"
#include "src/openrct2/paint/FirstPersonBoundaryMaterialPolicy.h"
#include "src/openrct2/paint/FirstPersonNativeComposite.h"
#include <bit>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <optional>
#include <unordered_map>
#include <unordered_set>
namespace OpenRCT2 {
struct ImageId { int32_t id{}; bool HasValue() const { return id != 0; } };
struct CoordsXY {
    int32_t x{}, y{};
    constexpr CoordsXY rotate(int32_t rotation) const {
        switch (rotation & 3) {
            case 1: return {y, -x};
            case 2: return {-x, -y};
            case 3: return {-y, x};
            default: return *this;
        }
    }
};
struct CoordsXYZ {
    int32_t x{}, y{}, z{};
    CoordsXYZ rotate(int32_t rotation) const {
        const auto xy = CoordsXY{x, y}.rotate(rotation);
        return {xy.x, xy.y, z};
    }
};
struct ScreenCoordsXY { int32_t x{}, y{}; };
namespace Drawing { enum class PaletteIndex : uint8_t { pi10 = 10, pi44 = 44 }; }
namespace Paint {
struct FirstPersonVec3 { float x{}, y{}, z{}; };
float FpDot(FirstPersonVec3 a, FirstPersonVec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
struct FirstPersonVisualHullFace {
    std::array<FirstPersonVec3, 4> corners{};
    FirstPersonVec3 normal{};
};
struct Sprite { int32_t width{}, height{}, xOffset{}, yOffset{}; };
struct LargeSceneryDecodedView {
    ImageId image{1};
    const Sprite* g1{};
    ScreenCoordsXY spritePos{};
    float nativeDepth{};
    std::vector<uint8_t> pixels;
};
struct Options { bool opaqueBoundaryMaterial = false; };
struct Hull { bool valid = true; };
} }
using namespace OpenRCT2;
using namespace OpenRCT2::Paint;
'''

SETUP = r'''
struct Result {
    int faceIndex{}, width{}, height{}, candidates{}, primaryDirection{};
    size_t drawable{}, visible{};
    std::vector<uint8_t> pixels;
};
double lastBakeMilliseconds = 0;
std::vector<Result> bake(const std::vector<FirstPersonVisualHullFace>& faces,
                         int width, int height, bool cutout, bool structural)
{
    struct Entry { std::vector<int> tiles; } entry;
    entry.tiles.resize(8);
    constexpr int sourceSize = 256;
    std::vector<Sprite> sprites(4 * entry.tiles.size(), {sourceSize, sourceSize, -128, -128});
    std::vector<LargeSceneryDecodedView> views(sprites.size());
    for (uint8_t direction = 0; direction < 4; ++direction)
    for (size_t sequence = 0; sequence < entry.tiles.size(); ++sequence)
    {
        auto& view = views[size_t(direction) * entry.tiles.size() + sequence];
        view.g1 = &sprites[size_t(direction) * entry.tiles.size() + sequence];
        view.nativeDepth = float(sequence);
        view.pixels.resize(sourceSize * sourceSize);
        const auto hole = Translate3DTo2DWithZ(direction, {16, 16, 16});
        for (int y = 0; y < sourceSize; ++y)
        for (int x = 0; x < sourceSize; ++x)
        {
            const bool authoredAlpha = cutout && std::abs(x - 128 - hole.x) < 5
                && std::abs(y - 128 - hole.y) < 5;
            if (!authoredAlpha && size_t((x + y * sourceSize) % 8) == sequence)
                view.pixels[size_t(y) * sourceSize + size_t(x)] = uint8_t(63 + (x / 4) % 3);
        }
    }
    const auto decodedView = [&](size_t sequence, uint8_t direction) {
        return &views[size_t(direction) * entry.tiles.size() + sequence];
    };
    const std::array<uint8_t, 4> nativeDirectionForRotation{0, 1, 2, 3};
    const Options options{structural};
    const Options* reconstructionOptions = structural ? &options : nullptr;
    const void* negativePassage = nullptr;
    const Hull hull{};
    const bool markUnknownUnderside = false;
    const auto bakeStart = std::chrono::steady_clock::now();
'''

MAIN = r'''
int main()
{
    std::vector<std::pair<const char*, std::vector<FirstPersonVisualHullFace>>> cases;
    FirstPersonVisualHullFace triangle;
    triangle.corners = {FirstPersonVec3{0,0,10}, {30,0,10}, {2,16,43}, {2,16,43}};
    triangle.normal = {0,-0.899,0.436};
    cases.push_back({"triangle_30x47", {triangle}});
    triangle.corners = {FirstPersonVec3{0,0,10}, {1,0,10}, {0,0,100}, {0,0,100}};
    triangle.normal = {0,-1,0};
    cases.push_back({"thin_triangle_1x256", {triangle}});
    FirstPersonVisualHullFace quad;
    quad.corners = {FirstPersonVec3{0.25,0.15,16.1}, {32.3,0.4,18.2}, {31.8,32.25,19.4}, {0.4,32.1,16.25}};
    quad.normal = {0,0,1};
    cases.push_back({"warped_quad", {quad}});
    quad.corners = {FirstPersonVec3{0.2,0.2,8.3}, {0.1,16.4,40.3}, {16.3,16.2,73.4}, {16.4,-0.1,40.5}};
    quad.normal = {0.666,0.666,-0.333};
    cases.push_back({"single_view_projection", {quad}});
    quad.corners = {FirstPersonVec3{0,0,16}, {32,0,16}, {32,32,16}, {0,32,16}};
    quad.normal = {0,0,1};
    cases.push_back({"corroborated_cutout", {quad}});
    cases.push_back({"structural_cutout", {quad}});
    // Shared coplanar triangle pixels have tied depth, not missing visibility.
    FirstPersonVisualHullFace a = quad, b = quad;
    a.corners[3] = a.corners[2];
    b.corners = {quad.corners[0], quad.corners[2], quad.corners[3], quad.corners[3]};
    cases.push_back({"shared_triangles", {a, b}});
    FirstPersonVisualHullFace back = quad, front = quad;
    back.corners = {FirstPersonVec3{32,32,1}, {40,32,1}, {40,40,1}, {32,40,1}};
    front.corners = {FirstPersonVec3{0,0,6}, {100,0,6}, {100,100,6}, {0,100,6}};
    cases.push_back({"occluded_face", {back, front}});
    std::cout << "{\"cases\":[";
    bool first = true;
    for (const auto& [name, faces] : cases)
    {
        const std::string label(name);
        const int width = label == "thin_triangle_1x256" ? 1 : label == "triangle_30x47" ? 30 : 32;
        const int height = label == "thin_triangle_1x256" ? 256 : label == "triangle_30x47" ? 47 : 32;
        const auto results = bake(faces, width, height, label.find("cutout") != std::string::npos,
                                   label == "structural_cutout");
        for (size_t i = 0; i < results.size(); ++i)
        {
            const size_t faceIndex = size_t(results[i].faceIndex);
            if (!first) std::cout << ',';
            first = false;
            const auto& result = results[i];
            size_t gpuTexels = 0, holes = 0, unknown = 0, projectionErrors = 0;
            for (int y = 0; y < height; ++y)
            for (int x = 0; x < width; ++x)
            {
                const auto& c = faces[faceIndex].corners[2];
                const auto& d = faces[faceIndex].corners[3];
                const bool triangle = c.x == d.x && c.y == d.y && c.z == d.z;
                // Independent integer intersection of the nearest-filtered UV
                // texel square with the GPU triangle s >= t.
                if (triangle && int64_t(x + 1) * height <= int64_t(y) * width)
                    continue;
                ++gpuTexels;
                const uint8_t pixel = result.pixels[size_t(y) * size_t(width) + size_t(x)];
                holes += pixel == 0;
                unknown += pixel == 10 || pixel == 44;
                if (label == "single_view_projection" || (label == "triangle_30x47"
                    && (x + 0.5) / width >= (y + 0.5) / height))
                {
                    // Independently interpolate the GPU's projected vertices.
                    // Constant palette bands make the chosen face-view's
                    // centre sample exact. More sub-samples from another view
                    // must not create a mosaic within this coherent face.
                    std::array<ScreenCoordsXY, 4> projected;
                    for (size_t corner = 0; corner < 4; ++corner)
                    {
                        const auto& p = faces[faceIndex].corners[corner];
                        projected[corner] = Translate3DTo2DWithZ(result.primaryDirection,
                            {int32_t(std::lround(p.x)), int32_t(std::lround(p.y)), int32_t(std::lround(p.z))});
                    }
                    const double s = (x + 0.5) / width, t = (y + 0.5) / height;
                    const std::array<double, 4> weights = s >= t
                        ? std::array<double, 4>{1-s, s-t, t, 0}
                        : std::array<double, 4>{1-t, 0, s, t-s};
                    double px = 0;
                    for (size_t corner = 0; corner < 4; ++corner)
                        px += weights[corner] * projected[corner].x;
                    const auto expected = uint8_t(63 + ((int(std::floor(px)) + 128) / 4) % 3);
                    projectionErrors += pixel != expected;
                }
            }
            std::cout << "{\"name\":\"" << name << "\",\"face\":" << result.faceIndex
                      << ",\"width\":" << width << ",\"height\":" << height
                      << ",\"gpu_texels\":" << gpuTexels << ",\"holes\":" << holes
                      << ",\"unknown\":" << unknown << ",\"candidates\":" << result.candidates
                      << ",\"primary_direction\":" << result.primaryDirection
                      << ",\"projection_errors\":" << projectionErrors
                      << ",\"bake_ms\":" << lastBakeMilliseconds
                      << ",\"drawable\":" << result.drawable << ",\"corners\":[";
            for (size_t corner = 0; corner < 4; ++corner)
            {
                if (corner) std::cout << ',';
                const auto& p = faces[faceIndex].corners[corner];
                std::cout << '[' << p.x << ',' << p.y << ',' << p.z << ']';
            }
            std::cout << "],\"pixels\":[";
            for (size_t pixel = 0; pixel < result.pixels.size(); ++pixel)
            {
                if (pixel) std::cout << ',';
                std::cout << int(result.pixels[pixel]);
            }
            std::cout << "]}";
        }
    }
    std::cout << "]}\n";
}
'''


def source_for(core):
    asset = (PAINT / "FirstPersonAssetReconstruction.h").read_text()
    hull = (PAINT / "FirstPersonVisualHull.h").read_text()
    roundtrip = (PAINT / "FirstPersonRoundTripReconstruction.h").read_text()
    viewport = (ROOT / "src/openrct2/interface/Viewport.cpp").read_text()
    out = ADAPTER
    for source, marker in [
        (viewport, "ScreenCoordsXY Translate3DTo2DWithZ("),
        (asset, "[[nodiscard]] constexpr CoordsXY FirstPersonNativeViewDirection("),
        (asset, "[[nodiscard]] constexpr uint64_t FirstPersonSilhouettePixelKey("),
        (asset, "struct FirstPersonSilhouette\n"),
        (asset, "[[nodiscard]] constexpr float FirstPersonIsoDepth("),
        (asset, "struct FirstPersonDepthOwnerPixel\n"),
    ]:
        out += definition(source, marker)
    out += "using FirstPersonDepthOwnerMap = std::unordered_map<uint64_t, FirstPersonDepthOwnerPixel>;\n"
    for source, marker in [
        (asset, "template<typename VisitPixel>"),
        (asset, "inline void AddFirstPersonDepthTriangle("),
        (asset, "struct FirstPersonDepthVisibility\n"),
        (asset, "[[nodiscard]] inline FirstPersonDepthVisibility BuildFirstPersonDepthVisibility("),
        (asset, "[[nodiscard]] inline float FirstPersonDepthOwnerCoverage("),
        (asset, "inline void AddFirstPersonSilhouetteTriangle("),
        (asset, "inline void AddFirstPersonSilhouetteQuad("),
        (hull, "[[nodiscard]] inline bool FirstPersonVisualHullSampleFace("),
        (hull, "[[nodiscard]] inline std::optional<std::array<float, 2>> FirstPersonVisualHullTexelUV("),
        (roundtrip, "[[nodiscard]] constexpr bool\n        FirstPersonSurfaceVisibleFromNativeView("),
        (core, "[[nodiscard]] constexpr uint8_t\n            FirstPersonMissingMaterialPixel("),
    ]:
        out += definition(source, marker)
    out += SETUP
    out += between(core, "            const auto faceVisible =", "\n            FirstPersonVisualHull hull =")
    begin = core.index("            std::array<FirstPersonDepthOwnerMap, 4>")
    out += core[begin:core.index("            const auto edgeLength =", begin)]
    if "expectedCompositeSamples" in core:
        out += definition(core, "            const auto edgeLength =")
    begin = core.index("            const auto visibleNear =" if "            const auto visibleNear =" in core else "            const auto ownedNear =", begin)
    out += core[begin:core.index("            bool haveBounds = false;", begin)]
    out += "std::vector<Result> results;\nfor (size_t faceIndex = 0; faceIndex < faces.size(); ++faceIndex) {\nconst auto& face = faces[faceIndex];\n"
    out += between(core, "                struct LargeSceneryFaceView", "\n                ImageId fallbackImage{};")
    out += between(core, "                ImageId fallbackImage{};", "\n                FirstPersonSurface surface{};")
    out += between(core, "                // Unobserved boundaries contain no native image detail.", "\n                std::vector<uint8_t> pixels(")
    out += between(core, "                std::vector<uint8_t> pixels(", "\n                if (hull.planarSurfaceSelected && haveUnknownMaterial")
    out += "(void)haveKnownMaterial; (void)haveUnknownMaterial;\nresults.push_back({int(faceIndex), width, height, int(candidates.size()), candidates.empty() ? -1 : int(candidates.front().nativeDirection), drawablePixelCount, visiblePixelCount, std::move(pixels)});\n}\nlastBakeMilliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - bakeStart).count();\nreturn results;\n}\n"
    out += MAIN
    return out


def exercise(core_path=None, sanitize=True):
    core = (core_path or PAINT / "FirstPersonRenderer.Core.inc").read_text()
    with tempfile.TemporaryDirectory(prefix="fp-large-bake-") as folder:
        source, binary = Path(folder) / "test.cpp", Path(folder) / "test"
        source.write_text(source_for(core))
        command = ["g++", "-std=c++20", "-O2", "-Wall", "-Wextra", "-Werror",
                   "-pedantic", "-I", str(ROOT), str(source), "-o", str(binary)]
        if sanitize:
            command += ["-fsanitize=undefined", "-fno-sanitize-recover=all"]
        subprocess.run(command, check=True)
        return json.loads(subprocess.check_output([str(binary)]))


class LargeSceneryMaterialBakeTest(unittest.TestCase):
    @unittest.skipUnless(shutil.which("g++"), "Native compiler unavailable")
    def test_exact_bake_covers_gpu_texels_without_erasing_native_cutouts(self):
        results = exercise()["cases"]
        for result in results:
            if result["name"] not in ("corroborated_cutout", "occluded_face"):
                self.assertEqual(result["holes"], 0, result["name"])
                self.assertEqual(result["drawable"], result["gpu_texels"])
        cutout = next(r for r in results if r["name"] == "corroborated_cutout")
        self.assertGreater(cutout["holes"], 0)
        self.assertEqual(cutout["pixels"][16 * 32 + 16], 0)
        hidden = next(r for r in results if r["name"] == "occluded_face" and r["face"] == 0)
        self.assertEqual(hidden["candidates"], 0)
        self.assertEqual(hidden["unknown"], hidden["gpu_texels"])
        self.assertEqual(hidden["holes"], 0)
        self.assertEqual(hidden["width"], 16)
        projection = next(r for r in results if r["name"] == "single_view_projection")
        self.assertEqual(projection["candidates"], 1)
        self.assertEqual(projection["projection_errors"], 0)
        roof = next(r for r in results if r["name"] == "triangle_30x47")
        self.assertEqual(roof["projection_errors"], 0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--core", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = exercise(args.core)
    if args.output:
        args.output.write_text(json.dumps(result) + "\n")
    print(json.dumps([{k: v for k, v in case.items() if k not in ("pixels", "corners")}
                      for case in result["cases"]], indent=2))
