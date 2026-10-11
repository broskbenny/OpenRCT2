"""Headless execution of the actual production native-material continuity header."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
SOURCE = r'''
#include "src/openrct2/paint/FirstPersonMaterialContinuity.h"
#include <cassert>
#include <iostream>
#include <vector>
using namespace OpenRCT2::Paint;
int main()
{
    auto check = [](int mode) {
        std::vector<uint8_t> pixels(49, 83);
        std::vector<uint8_t> provenance(49, 1);
        std::vector<uint8_t> eligible(49, 0);
        const size_t centre = 24;
        pixels[centre] = 0;
        provenance[centre] = 0;
        eligible[centre] = 1;
        if (mode == 1)
        {
            pixels[centre+1] = 0;
            provenance[centre+1] = 0;
            eligible[centre+1] = 1;
        }
        if (mode == 2) provenance[centre+1] = 0; // Unknown neighbour.
        if (mode == 3) eligible[centre] = 0; // Confirmed transparency.
        if (mode == 4)
            for (size_t i : {16u, 17u, 18u, 23u}) pixels[i] = 95;
        if (mode == 5)
        {
            pixels[0] = 0;
            provenance[0] = 0;
            eligible[0] = 1; // Never fill a silhouette boundary.
        }
        size_t count = RepairFirstPersonIsolatedMaterialPinhole(
            pixels, provenance, eligible, 7, 7);
        if (mode == 0 || mode == 5)
        {
            assert(count == 1 && pixels[centre] == 83);
        }
        else
        {
            assert(count == 0 && pixels[centre] == 0);
        }
        if (mode == 5) assert(pixels[0] == 0);
    };
    for (int mode = 0; mode < 6; ++mode) check(mode);
    std::vector<uint8_t> tiny{0, 7, 0, 7};
    assert(RepairFirstPersonIsolatedMaterialPinhole(
        tiny, tiny, tiny, 2, 2) == 0);
    std::cout << "production material continuity: PASS\n";
}
'''


class ProductionMaterialContinuityTests(unittest.TestCase):
    def test_actual_cpp_header_handles_isolated_raster_holes_safely(self):
        if shutil.which('g++') is None:
            self.skipTest('g++ unavailable')
        with tempfile.TemporaryDirectory(prefix='fp-material-test-') as td:
            source = Path(td) / 'material.cpp'
            executable = Path(td) / 'material'
            source.write_text(SOURCE)
            subprocess.run(
                ['g++', '-std=c++20', '-O2', '-Wall', '-Wextra', '-Werror',
                 '-pedantic', '-fsanitize=undefined',
                 '-fno-sanitize-recover=all', '-I', str(ROOT),
                 str(source), '-o', str(executable)], check=True)
            subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    unittest.main()
