#!/usr/bin/env python3
"""Check whether an XYZ occupancy could enter the production planar-solid fitter.

Independent precondition diagnostic, NOT an invocation of C++ reconstruction.
Passing does not imply the final mesh, native artwork, or four-view checks pass.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np


def assess(occupied) -> dict:
    voxels = np.asarray(occupied)
    if voxels.ndim != 3 or not np.isin(voxels, [0, 1]).all():
        raise ValueError('Expected a three-dimensional XYZ occupancy of zero/one cells')
    x, y, z = map(int, voxels.shape)
    counts = np.count_nonzero(voxels, axis=2)
    base = voxels[:, :, 0] != 0
    # The production candidate requires a solid prefix from z=0.
    prefix = np.minimum.accumulate(voxels.astype(bool), axis=2)
    gapped = np.any((voxels != 0) & ~prefix, axis=2)
    empty = counts == 0
    precondition = x >= 2 and y >= 2 and z >= 1 and x * y <= 4096 \
        and max(x, y, z) <= 255 and not np.any(empty | gapped)
    return {
        'axis_order': 'XYZ',
        'grid': [x, y, z],
        'occupied_cells': int(np.count_nonzero(voxels)),
        'horizontal_columns': x * y,
        'empty_horizontal_columns': int(empty.sum()),
        'columns_not_occupied_at_base': int((~base).sum()),
        'columns_with_internal_gaps_or_floating_sections': int(gapped.sum()),
        'passes_initial_solid_occupancy_gate': bool(precondition),
        'production_candidate_executed': False,
        'explanation': 'Only full rectangular, base-contiguous occupancy can enter the current planar-solid fitter. More complex buildings need piecewise surface reconstruction.'
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('occupancy_npy', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args(argv)
    result = assess(np.load(args.occupancy_npy, allow_pickle=False))
    output = json.dumps(result, indent=2) + '\n'
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(output)
    print(output, end='')
    return result


if __name__ == '__main__':
    main()
