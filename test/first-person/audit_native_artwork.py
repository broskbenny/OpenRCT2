#!/usr/bin/env python3
"""Audit indexed-palette consistency of large-scenery tile sprites vs previews.

This is a native-artwork decoding/compositing check, NOT an independent 3D
quality certificate. All four preview registrations are fitted to those
very previews, so a perfect score establishes source consistency only.
"""
from __future__ import annotations
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np

from real_asset_eval import read_large_scenery


def rotation_xy(x, y, rotation):
    return ((x, y), (y, -x), (-x, -y), (-y, x))[rotation & 3]


def tile_composite(dat, rotation):
    """Composite native tile sprites in isometric depth order, not sequence order."""
    tiles, sprites = dat['tiles'], dat['images']
    if len(sprites) < 4 + 4 * len(tiles):
        raise ValueError('Incomplete native large-scenery image table')
    layers = []
    for sequence, tile in enumerate(tiles):
        item = sprites[4 + 4 * sequence + rotation]
        x, y = rotation_xy(tile['x'], tile['y'], rotation)
        sx = y - x + item['x']
        sy = (x + y) // 2 - tile['z'] + item['y']
        layers.append((x + y + tile['z'], sequence, sx, sy, item))
    layers.sort(key=lambda entry: (entry[0], entry[1]))
    x0 = min(row[2] for row in layers)
    y0 = min(row[3] for row in layers)
    x1 = max(row[2] + row[4]['width'] for row in layers)
    y1 = max(row[3] + row[4]['height'] for row in layers)
    if (x1 - x0) * (y1 - y0) > 2_000_000:
        raise ValueError('Tile sprite composite exceeds analysis budget')
    canvas = np.zeros((y1 - y0, x1 - x0), dtype=np.uint8)
    for _, sequence, x, y, item in layers:
        pixels = np.frombuffer(item['pixels'], dtype=np.uint8).reshape(
            (item['height'], item['width']))
        crop = canvas[y - y0:y - y0 + item['height'], x - x0:x - x0 + item['width']]
        crop[pixels != 0] = pixels[pixels != 0]
    return canvas, (x0, y0)


def audit_registration(reference, composite, seed_limit=64, shift_budget=200):
    """Fit translation from exact matching palette indices; report containment.

    Preview clipping is legitimate and is deliberately NOT counted as a
    false-positive shape. The returned result must never be interpreted as
    independent reconstruction precision/recall or IoU.
    """
    ref = np.asarray(reference, dtype=np.uint8)
    src = np.asarray(composite, dtype=np.uint8)
    ry, rx = np.nonzero(ref)
    sy, sx = np.nonzero(src)
    if not len(rx) or not len(sx):
        raise ValueError('Empty preview or tile composite')
    ref_palette = ref[ry, rx]
    src_palette = src[sy, sx]
    by_index = {int(v): np.flatnonzero(src_palette == v)
                for v in np.unique(src_palette)}
    counts = Counter(map(int, ref_palette))
    ordered = sorted(range(len(rx)), key=lambda i: (counts[int(ref_palette[i])], i))
    samples = ordered[:seed_limit]
    votes = Counter()
    for i in samples:
        js = by_index.get(int(ref_palette[i]))
        if js is None:
            continue
        # Shift maps source pixel coordinate to reference pixel coordinate.
        for j in js:
            dx, dy = int(rx[i] - sx[j]), int(ry[i] - sy[j])
            if abs(dx) <= shift_budget and abs(dy) <= shift_budget:
                votes[(dx, dy)] += 1
    if not votes:
        raise ValueError('No palette correspondences within registration budget')
    candidates = [pair for pair, _ in votes.most_common(64)]
    best = None
    for dx, dy in candidates:
        px = rx - dx
        py = ry - dy
        inside = ((px >= 0) & (px < src.shape[1])
                  & (py >= 0) & (py < src.shape[0]))
        sampled = np.zeros(len(rx), dtype=np.uint8)
        sampled[inside] = src[py[inside], px[inside]]
        exact = int(np.count_nonzero(sampled == ref_palette))
        differing = int(np.count_nonzero((sampled != 0) & (sampled != ref_palette)))
        missing = int(np.count_nonzero(sampled == 0))
        key = (exact, -differing, -missing, -abs(dx) - abs(dy))
        if best is None or key > best[0]:
            best = (key, (dx, dy, exact, differing, missing))
    dx, dy, exact, differing, missing = best[1]
    return {
        'registration_shift_to_preview': [dx, dy],
        'preview_opaque_pixels': int(len(rx)),
        'matched_palette_pixels': exact,
        'different_palette_pixels': differing,
        'missing_in_tile_composite': missing,
        'exact_reference_fraction': round(exact / len(rx), 6),
        'registration_fitted_to_reference': True,
        'independent_geometry_certificate': False,
    }


def audit_asset(dat):
    output = {'asset': dat['name'], 'sha256': dat['sha256'], 'tile_count': len(dat['tiles']),
              'image_count': len(dat['images']), 'native_preview_comparisons': []}
    for rotation in range(4):
        preview = dat['images'][rotation]
        reference = np.frombuffer(preview['pixels'], dtype=np.uint8).reshape(
            (preview['height'], preview['width']))
        composite, offset = tile_composite(dat, rotation)
        row = audit_registration(reference, composite)
        row.update(rotation=rotation,
                   composite_origin_world_projection=list(offset),
                   composite_opaque_pixels=int(np.count_nonzero(composite)))
        output['native_preview_comparisons'].append(row)
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dat', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--require-exact-preview-containment', action='store_true',
                        help='Strict source-consistency check, NOT a 3D admission gate')
    args = parser.parse_args(argv)
    report = audit_asset(read_large_scenery(args.dat))
    data = json.dumps(report, indent=2) + '\n'
    if args.output:
        args.output.parent.mkdir(exist_ok=True, parents=True)
        args.output.write_text(data)
    else:
        print(data, end='')
    if args.require_exact_preview_containment and any(
            row['exact_reference_fraction'] < 1.0
            for row in report['native_preview_comparisons']):
        raise SystemExit(1)
    return report


if __name__ == '__main__':
    main()
