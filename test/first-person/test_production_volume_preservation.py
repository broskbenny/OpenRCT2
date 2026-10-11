"""Compile and execute the production closed-volume admission certificate."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]

SOURCE = r'''
#include "src/openrct2/paint/FirstPersonVolumePreservation.h"
#include <array>
#include <vector>
#include <cassert>
#include <iostream>
using namespace OpenRCT2::Paint;
struct Vec { float x{}, y{}, z{}; };
struct Face { std::array<Vec, 4> corners; };
struct Hull {
    bool valid = true;
    float step=2, minForward=0, minRight=0, minUp=0;
    int sizeForward=6, sizeRight=6, sizeUp=4;
    std::vector<uint8_t> occupied;
    bool contains(int x, int y, int z) const {
        return occupied[(z*sizeRight+y)*sizeForward+x]!=0;
    }
};
int main() {
    Hull hull;
    hull.occupied.assign(6*6*4, 1);
    auto face=[](Vec a, Vec b, Vec c, Vec d) {
        return Face{{a,b,c,d}};
    };
    std::vector<Face> faces{
        face({0,0,8},{12,0,8},{12,12,8},{0,12,8}),
        face({0,12,0},{12,12,0},{12,0,0},{0,0,0}),
        face({0,0,0},{12,0,0},{12,0,8},{0,0,8}),
        face({12,12,0},{0,12,0},{0,12,8},{12,12,8}),
        face({12,0,0},{12,12,0},{12,12,8},{12,0,8}),
        face({0,12,0},{0,0,0},{0,0,8},{0,12,8})
    };
    auto valid=CertifyFirstPersonCandidateVolume(hull,faces);
    assert(valid.retainsAllCellCentres());
    hull.occupied[(0*6+3)*6+3]=0; // Genuine negative cell.
    auto sealed=CertifyFirstPersonCandidateVolume(hull,faces);
    assert(sealed.measured && sealed.falselyFilledEmptyCells==1
           && sealed.lostOccupiedCells==0);
    assert(!sealed.retainsAllCellCentres());
    hull.occupied.assign(6*6*4,1);
    faces.pop_back(); // Missing vertical wall.
    auto open=CertifyFirstPersonCandidateVolume(hull,faces);
    assert(!open.retainsAllCellCentres());
    assert(open.nonManifoldOrOpenEdges!=0);
    assert(!CertifyFirstPersonCandidateVolume(hull,faces,1).measured);
    std::cout << "production volume preservation: PASS\n";
}
'''

class VolumePreservationCppTests(unittest.TestCase):
    def test_real_header_compiles_and_rejects_false_geometry(self):
        if shutil.which('g++') is None:
            self.skipTest('C++ compiler unavailable')
        with tempfile.TemporaryDirectory(prefix='fp-volume-cpp-') as directory:
            source = Path(directory) / 'test.cpp'
            executable = Path(directory) / 'test'
            source.write_text(SOURCE)
            subprocess.run(
                ['g++', '-std=c++20', '-O2', '-Wall', '-Wextra',
                 '-Werror', '-pedantic', '-fsanitize=undefined',
                 '-fno-sanitize-recover=all', '-I', str(ROOT),
                 str(source), '-o', str(executable)], check=True)
            subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    unittest.main()
