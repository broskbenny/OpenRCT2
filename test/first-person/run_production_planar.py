#!/usr/bin/env python3
"""Run the exact production planar-solid C++ geometry function without the game.

Extracts BuildFirstPersonPlanarSolidCandidate from the repository header at
runtime; no copied Python reconstruction implementation. Other OpenRCT2 types
are minimal value-layout adapters. Tests GEOMETRY ONLY, not C++ large-scenery
paint capture, complete visual-hull carving, native materials, or GPU output.

Occupancy NPY arrays are indexed [forward, right, up] (XYZ).
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import numpy as np


MARKER = 'BuildFirstPersonPlanarSolidCandidate('


def exact_function(repo: Path) -> str:
    source = (repo / 'src/openrct2/paint/FirstPersonPlanarSolidReconstruction.h').read_text()
    marker = source.index(MARKER)
    begin = source.rfind('[[nodiscard]]', 0, marker)
    if begin < 0:
        raise ValueError('Missing production C++ declaration')
    opening = source.index('{', marker)
    depth = 1
    end = opening + 1
    while depth:
        if end == len(source):
            raise ValueError('Unterminated production C++ body')
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[begin:end]


ADAPTER = r'''
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

namespace OpenRCT2::Paint {
struct FirstPersonVec3 { float x{}, y{}, z{}; };
inline float FpDot(FirstPersonVec3 a, FirstPersonVec3 b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}
inline FirstPersonVec3 FpCross(FirstPersonVec3 a, FirstPersonVec3 b) {
    return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
enum class FirstPersonVisualHullFaceKind : uint8_t {
    minForward, maxForward, minRight, maxRight, bottom, top
};
struct FirstPersonVisualHullFace {
    std::array<FirstPersonVec3, 4> corners{};
    FirstPersonVec3 normal{};
    FirstPersonVisualHullFaceKind kind{};
};
struct FirstPersonVisualHull {
    bool valid = false;
    float step = 4.0f;
    float minForward{}, minRight{}, minUp{};
    uint8_t sizeForward{}, sizeRight{}, sizeUp{};
    std::vector<uint8_t> occupied;
    bool contains(int x, int y, int z) const {
        if (x<0 || y<0 || z<0 || x>=sizeForward ||
            y>=sizeRight || z>=sizeUp) return false;
        return occupied[(size_t(z)*size_t(sizeRight)+size_t(y))*
                        sizeForward+size_t(x)] != 0;
    }
};
'''

MAIN = r'''
} // namespace OpenRCT2::Paint
int main() {
    using namespace OpenRCT2::Paint;
    int nx=0, ny=0, nz=0;
    float step=2, x0=0, y0=0, z0=0;
    if (!(std::cin >> nx >> ny >> nz >> step >> x0 >> y0 >> z0)) return 2;
    std::cin.get();
    if (nx<1 || ny<1 || nz<1 || nx>255 || ny>255 || nz>255 ||
        size_t(nx)*size_t(ny)*size_t(nz)>131072) return 2;
    FirstPersonVisualHull hull;
    hull.valid=true;
    hull.sizeForward=uint8_t(nx);
    hull.sizeRight=uint8_t(ny);
    hull.sizeUp=uint8_t(nz);
    hull.minForward=x0; hull.minRight=y0; hull.minUp=z0; hull.step=step;
    hull.occupied.resize(size_t(nx)*size_t(ny)*size_t(nz));
    std::cin.read(reinterpret_cast<char*>(hull.occupied.data()), hull.occupied.size());
    if (size_t(std::cin.gcount()) != hull.occupied.size()) return 3;
    const auto faces=BuildFirstPersonPlanarSolidCandidate(hull);
    std::cout << std::fixed << std::setprecision(8) << "{\"faces\":[";
    bool first=true;
    for(const auto& face:faces) {
        if(!first) std::cout << ',';
        first=false;
        std::cout << "{\"kind\":" << int(face.kind) << ",\"corners\":[";
        for(size_t i=0;i<4;i++) {
            if(i) std::cout << ',';
            const auto& p=face.corners[i];
            std::cout << '[' << p.x << ',' << p.y << ',' << p.z << ']';
        }
        std::cout << "]}";
    }
    std::cout << "]}";
}
'''


def compile_production(repo: Path, executable: Path, compiler='g++') -> None:
    if shutil.which(compiler) is None:
        raise RuntimeError(f'C++ compiler not available: {compiler}')
    source = executable.with_suffix('.cpp')
    source.write_text(ADAPTER + exact_function(repo) + '\n' + MAIN)
    subprocess.run([compiler, '-std=c++20', '-O2', '-Wall', '-Wextra',
                    '-Werror', '-pedantic', str(source), '-o', str(executable)],
                   check=True)


def execute(executable: Path, occupancy, step=2.0, origin=(0., 0., 0.)):
    occupancy = np.asarray(occupancy)
    if occupancy.ndim != 3 or not np.isin(occupancy, [0, 1]).all():
        raise ValueError('Expected binary XYZ occupancy')
    nx, ny, nz = occupancy.shape
    header = f'{nx} {ny} {nz} {step} {origin[0]} {origin[1]} {origin[2]}\n'.encode()
    # Production index: (z*ny+y)*nx+x. Numpy input: [x, y, z].
    cells = np.ascontiguousarray(occupancy.transpose(2, 1, 0),
                                dtype=np.uint8).tobytes()
    run = subprocess.run([str(executable)], input=header + cells, check=True,
                         stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return json.loads(run.stdout)


def as_mesh(result: dict):
    import trimesh
    vertices, triangles = [], []
    for face in result['faces']:
        first = len(vertices)
        vertices.extend(face['corners'])
        triangles.extend([(first, first + 1, first + 2),
                          (first, first + 2, first + 3)])
    return trimesh.Trimesh(vertices=vertices, faces=triangles, process=True) if triangles else None


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('occupancy', type=Path)
    p.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
    p.add_argument('--step', type=float, default=2.0)
    p.add_argument('--origin', type=float, nargs=3, default=(0., 0., 0.))
    p.add_argument('--compiler', default='g++')
    p.add_argument('--mesh', type=Path)
    p.add_argument('--report', type=Path)
    a = p.parse_args(argv)
    grid = np.load(a.occupancy, allow_pickle=False)
    with tempfile.TemporaryDirectory(prefix='production-planar-') as d:
        exe = Path(d) / 'reconstruction'
        compile_production(a.repo, exe, a.compiler)
        result = execute(exe, grid, a.step, a.origin)
    mesh = as_mesh(result)
    report = {'grid_shape_xyz':list(grid.shape),
              'occupied_cells':int(np.count_nonzero(grid)),
              'production_function':'BuildFirstPersonPlanarSolidCandidate',
              'candidate_faces':len(result['faces']),
              'triangles':len(mesh.faces) if mesh is not None else 0,
              'watertight':bool(mesh.is_watertight) if mesh is not None else False,
              'limits':'Exact production C++ planar function; no production paint, texture, or GPU'}
    if a.mesh is not None and mesh is not None:
        a.mesh.parent.mkdir(parents=True, exist_ok=True)
        mesh.export(a.mesh)
    if a.report is not None:
        a.report.parent.mkdir(parents=True, exist_ok=True)
        a.report.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    return report


if __name__ == '__main__':
    main()
