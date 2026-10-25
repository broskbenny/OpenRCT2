"""Compile the production asset-worker vehicle face baker against synthetic hulls."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class VehicleBodyFaceBakeTest(unittest.TestCase):
    def test_production_bake_exposes_only_boundary_faces_and_reuses_native_material(self):
        # No mirrored Python approximation: compile the real inline C++ builder.
        program = r"""
#include "src/openrct2/paint/FirstPersonVehicleBodyHull.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
using namespace OpenRCT2;
using namespace OpenRCT2::Paint;

int main()
{
    FirstPersonVehicleBodyHull hull{};
    hull.valid = true;
    hull.step = 4.0f;
    hull.minForward = -4.0f;
    hull.minRight = 0.0f;
    hull.minUp = 8.0f;
    hull.sizeForward = 2;
    hull.sizeRight = 1;
    hull.sizeUp = 1;
    hull.occupied = { 1, 1 };
    auto empty = BuildFirstPersonVehicleBodyFaces(
        hull, [](ImageIndex) -> const G1Element* { return nullptr; });
    assert(empty.size() == 10);
    assert(std::count_if(empty.begin(), empty.end(),
        [](const auto& f) { return f.top; }) == 2);
    for (const auto& face : empty)
    {
        assert(!face.textured);
        for (const auto& uv : face.uv)
            assert(uv[0] == 0.0f && uv[1] == 0.0f);
    }
    // A fully opaque native view must texture outward surfaces, never the
    // underside; the native source is still an asset index, not an instance
    // colour remap, so the baked result is safe to reuse across trains.
    std::array<uint8_t, 128 * 128> pixels{};
    pixels.fill(1);
    G1Element sprite{};
    sprite.offset = pixels.data();
    sprite.width = 128;
    sprite.height = 128;
    sprite.xOffset = -64;
    sprite.yOffset = -64;
    hull.textureViews.push_back({ 0, ImageIndex{ 199 } });
    auto baked = BuildFirstPersonVehicleBodyFaces(
        hull, [&](ImageIndex index) -> const G1Element* {
            return index == ImageIndex{ 199 } ? &sprite : nullptr;
        });
    assert(baked.size() == empty.size());
    size_t textured = 0;
    size_t underside = 0;
    for (const auto& face : baked)
    {
        bool bottom = true;
        for (const auto& point : face.local)
            bottom = bottom && std::abs(point.z - hull.minUp) < 0.001f;
        if (bottom)
        {
            ++underside;
            assert(!face.textured);
        }
        if (face.textured)
        {
            ++textured;
            assert(face.image == ImageIndex{ 199 });
        }
    }
    assert(underside == 2 && textured >= 2);
    // Invalid hulls must never publish candidate geometry.
    hull.valid = false;
    assert(BuildFirstPersonVehicleBodyFaces(
        hull, [](ImageIndex) -> const G1Element* { return nullptr; }).empty());
}
"""
        with tempfile.TemporaryDirectory(prefix="fp-vehicle-body-") as temp:
            source = Path(temp) / "test.cpp"
            exe = Path(temp) / "test"
            source.write_text(program, encoding="utf-8")
            cmd = ["g++", "-std=c++20", "-O2", "-Wall", "-Wextra",
                   "-I" + str(ROOT), "-I" + str(ROOT / "src/thirdparty"),
                   str(source), "-o", str(exe)]
            subprocess.run(cmd, check=True, capture_output=True, text=True)
            subprocess.run([str(exe)], check=True, capture_output=True, text=True)


if __name__ == "__main__":
    unittest.main()
