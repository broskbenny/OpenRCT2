#!/usr/bin/env python3
"""Check that a simplified watertight mesh preserves source occupancy voids.

An independent GEOMETRY acceptance test, not the game renderer. XYZ occupancy
samples must use the exact sampling registration of the mesh. Never optimize
the offset against the same source and call that independent verification.
Authentic openings missing from the input occupancy cannot be certified.

  python test/first-person/check_mesh_free_space.py volume.npy candidate.glb \
    --step 2 --origin -1 -1 -1 --max-sealed-exterior-cells 0 \
    --max-lost-solid-cells 10
"""
from __future__ import annotations
import argparse
from collections import deque
import json
from pathlib import Path
import numpy as np


def vertical_crossings(mesh, xs, ys):
    xx, yy = np.meshgrid(xs, ys, indexing='ij')
    points = np.column_stack((xx.ravel(), yy.ravel()))
    hits = [[] for _ in points]
    triangles = np.asarray(mesh.vertices)[np.asarray(mesh.faces)]
    for a, b, c in triangles:
        bax, bay = b[0] - a[0], b[1] - a[1]
        cax, cay = c[0] - a[0], c[1] - a[1]
        determinant = bax * cay - cax * bay
        if abs(determinant) < 1e-10:
            continue  # Vertical wall: not crossed by a vertical probe.
        xmin, xmax = min(a[0], b[0], c[0]), max(a[0], b[0], c[0])
        ymin, ymax = min(a[1], b[1], c[1]), max(a[1], b[1], c[1])
        matching = np.flatnonzero(
            (points[:, 0] >= xmin) & (points[:, 0] <= xmax)
            & (points[:, 1] >= ymin) & (points[:, 1] <= ymax))
        if not matching.size:
            continue
        x, y = points[matching, 0] - a[0], points[matching, 1] - a[1]
        u = (x * cay - y * cax) / determinant
        v = (bax * y - bay * x) / determinant
        inside = (u >= -1e-8) & (v >= -1e-8) & (u + v <= 1 + 1e-8)
        for column, z in zip(matching[inside],
                             a[2] + u[inside] * (b[2]-a[2])
                             + v[inside] * (c[2]-a[2])):
            hits[int(column)].append(float(z))
    return hits


def classify_closed_mesh(mesh, shape, step=2.0, origin=(0., 0., 0.),
                         jitter=0.013):
    """XYZ grid via vertical ray parity; no external ray engine."""
    if not mesh.is_watertight:
        raise ValueError('Volume test requires a watertight triangle mesh')
    if not np.isfinite(step) or step <= 0 or len(shape) != 3 or min(shape) < 1:
        raise ValueError('Invalid grid registration')
    nx, ny, nz = map(int, shape)
    xs = origin[0] + (np.arange(nx) + 0.5 + jitter) * step
    ys = origin[1] + (np.arange(ny) + 0.5 - jitter) * step
    zs = origin[2] + (np.arange(nz) + 0.5) * step
    classified = np.zeros((nx * ny, nz), dtype=bool)
    ambiguous = 0
    for idx, intersections in enumerate(vertical_crossings(mesh, xs, ys)):
        if not intersections:
            continue
        unique = []
        for value in sorted(intersections):
            if not unique or abs(value - unique[-1]) > 1e-5:
                unique.append(value)
        if len(unique) % 2:
            ambiguous += 1
            continue  # Ambiguous rays cannot be silently accepted.
        classified[idx] = np.searchsorted(unique, zs, side='right') % 2 == 1
    return classified.reshape(nx, ny, nz), ambiguous


def exterior_connected_empty(source):
    """Six-neighbour flood fill from empty volume boundary cells."""
    nx, ny, nz = source.shape
    seen = np.zeros(source.shape, dtype=bool)
    queue = deque()
    for axis in range(3):
        for side in (0, source.shape[axis] - 1):
            coords = np.argwhere(np.take(~source, side, axis=axis))
            for sub in coords:
                index = list(map(int, sub))
                index.insert(axis, side)
                p = tuple(index)
                if not seen[p]:
                    seen[p] = True
                    queue.append(p)
    while queue:
        x, y, z = queue.popleft()
        for xx, yy, zz in ((x-1,y,z),(x+1,y,z),(x,y-1,z),
                           (x,y+1,z),(x,y,z-1),(x,y,z+1)):
            if (0 <= xx < nx and 0 <= yy < ny and 0 <= zz < nz
                    and not source[xx, yy, zz] and not seen[xx, yy, zz]):
                seen[xx, yy, zz] = True
                queue.append((xx, yy, zz))
    return seen


def measure(mesh, occupancy, step=2.0, origin=(0., 0., 0.)):
    source = np.asarray(occupancy, dtype=bool)
    if source.ndim != 3 or source.size > 131072:
        raise ValueError('Expected XYZ occupancy with at most 131072 cells')
    candidate, ambiguous = classify_closed_mesh(mesh, source.shape, step, origin)
    false_filled = candidate & ~source
    lost_solid = source & ~candidate
    exterior_empty = exterior_connected_empty(source)
    return dict(
        grid_shape_xyz=list(source.shape), grid_cells=int(source.size),
        source_solid_cells=int(source.sum()),
        candidate_solid_cells=int(candidate.sum()),
        false_filled_empty_cells=int(false_filled.sum()),
        sealed_exterior_connected_empty_cells=int(
            (false_filled & exterior_empty).sum()),
        lost_source_solid_cells=int(lost_solid.sum()),
        ambiguous_vertical_rays=ambiguous,
        watertight=bool(mesh.is_watertight),
        registration={'origin':list(origin), 'step':step,
                      'must_match_mesh_sample_coordinates':True},
        limits='Certifies geometry relative to supplied occupancy, not independently authored physical openings')


def main(argv=None):
    import trimesh
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('occupancy', type=Path)
    parser.add_argument('mesh', type=Path)
    parser.add_argument('--step', type=float, default=2.)
    parser.add_argument('--origin', type=float, nargs=3, default=(0., 0., 0.))
    parser.add_argument('--max-sealed-exterior-cells', type=int)
    parser.add_argument('--max-lost-solid-cells', type=int)
    parser.add_argument('--max-ambiguous-rays', type=int, default=0)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    mesh = trimesh.load(args.mesh, force='mesh')
    report = measure(mesh, np.load(args.occupancy, allow_pickle=False),
                     step=args.step, origin=args.origin)
    gates = {'ambiguous_rays':
             report['ambiguous_vertical_rays'] <= args.max_ambiguous_rays}
    if args.max_sealed_exterior_cells is not None:
        gates['preserve_exterior_free_space'] = (
            report['sealed_exterior_connected_empty_cells']
            <= args.max_sealed_exterior_cells)
    if args.max_lost_solid_cells is not None:
        gates['preserve_source_solid'] = (
            report['lost_source_solid_cells'] <= args.max_lost_solid_cells)
    report['requested_gates'] = gates
    report['accepted'] = all(gates.values())
    output = json.dumps(report, indent=2) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output)
    print(output, end='')
    if not report['accepted']:
        raise SystemExit(1)
    return report


if __name__ == '__main__':
    main()
