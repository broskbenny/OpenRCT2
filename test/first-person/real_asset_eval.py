#!/usr/bin/env python3
"""Independent large-scenery DAT/mesh regression evaluator (NOT the game renderer).

Requires numpy, OpenCV, trimesh and Pillow for mesh comparisons; decoding a DAT
requires only the Python standard library. No proprietary artwork is bundled.

  python real_asset_eval.py inspect path/to/SMH2.DAT --output report.json
  python real_asset_eval.py compare path/to/SMH2.DAT candidate.glb \
      --baseline 72 --output compare.json

The baseline is an explicit calibrated screen-space y offset. Fitting it to
these reference images and then scoring those images is NOT independent proof.
Palette-index agreement and hidden-surface material ownership are NOT measured.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import struct


def _require(data: bytes, offset: int, length: int):
    if offset < 0 or length < 0 or offset + length > len(data):
        raise ValueError('Truncated or malformed RCT2 DAT')


def read_large_scenery(path: Path) -> dict:
    raw = path.read_bytes()
    _require(raw, 0, 21)
    flags, name_bytes, checksum = struct.unpack_from('<I8sI', raw)
    encoding, size = struct.unpack_from('<BI', raw, 16)
    if encoding != 1 or size != len(raw) - 21 or size > 16 * 1024 * 1024:
        raise ValueError('Expected a single Sawyer-RLE legacy object chunk')
    out = bytearray()
    at = 21
    while at < len(raw):
        code = raw[at]
        at += 1
        if code & 128:
            _require(raw, at, 1)
            n = 257 - code
            out.extend(bytes((raw[at],)) * n)
            at += 1
        else:
            n = code + 1
            _require(raw, at, n)
            out.extend(raw[at:at + n])
            at += n
        if len(out) > 16 * 1024 * 1024:
            raise ValueError('Oversized decoded DAT chunk')
    data = bytes(out)
    # LargeSceneryObject::ReadLegacy skips 22 bytes before the language strings.
    _require(data, 0, 22)
    at = 22
    names = []
    for _ in range(128):
        _require(data, at, 1)
        lang = data[at]
        at += 1
        if lang == 255:
            break
        end = data.find(b'\0', at)
        if end < 0:
            raise ValueError('Unterminated language string')
        names.append((lang, data[at:end].decode('cp1252', errors='replace')))
        at = end + 1
    else:
        raise ValueError('Unterminated language table')
    _require(data, at, 16)
    at += 16  # scenery group reference
    tiles = []
    for _ in range(512):
        _require(data, at, 2)
        if struct.unpack_from('<H', data, at)[0] == 0xffff:
            at += 2
            break
        _require(data, at, 9)
        x, y, z, clearance, flags_tile = struct.unpack_from('<hhhBH', data, at)
        tiles.append(dict(x=x, y=y, z=z, clearance=clearance,
                          walls=(flags_tile >> 8) & 15,
                          corners=(flags_tile >> 12) & 15))
        at += 9
    else:
        raise ValueError('Unterminated large-scenery tile list')
    _require(data, at, 8)
    count, declared_data_size = struct.unpack_from('<II', data, at)
    at += 8
    if not (4 <= count <= 4096 and declared_data_size <= 16 * 1024 * 1024):
        raise ValueError('Unexpected image table size (or unsupported 3D-text object)')
    _require(data, at, count * 16 + declared_data_size)
    image_data = data[at + count * 16: at + count * 16 + declared_data_size]
    images = []
    for i in range(count):
        off, width, height, ox, oy, image_flags, zoom = struct.unpack_from(
            '<IhhhhHH', data, at + i * 16)
        if not (0 <= width <= 2048 and 0 <= height <= 2048 and width * height <= 65536):
            raise ValueError(f'Invalid dimensions for image {i}')
        if off > len(image_data) or (width * height and off == len(image_data)):
            raise ValueError(f'Invalid data offset for image {i}')
        pixels = bytearray(width * height)
        if image_flags & 4:  # G1Flag::hasRLECompression
            _require(image_data, off, height * 2)
            for y in range(height):
                relative = struct.unpack_from('<H', image_data, off + y * 2)[0]
                cursor = off + relative
                for _ in range(256):
                    _require(image_data, cursor, 2)
                    code, x = image_data[cursor], image_data[cursor + 1]
                    cursor += 2
                    n = code & 127
                    _require(image_data, cursor, n)
                    if x + n > width:
                        raise ValueError(f'RLE run overflows image {i}, row {y}')
                    pixels[y * width + x: y * width + x + n] = image_data[cursor:cursor + n]
                    cursor += n
                    if code & 128:
                        break
                else:
                    raise ValueError(f'Unterminated image RLE row {i}:{y}')
        else:
            _require(image_data, off, width * height)
            pixels[:] = image_data[off:off + width * height]
        images.append(dict(width=width, height=height, x=ox, y=oy,
                           flags=image_flags, zoom=zoom, pixels=bytes(pixels),
                           opaque=sum(pixel != 0 for pixel in pixels)))
    return dict(name=name_bytes.decode('ascii', errors='replace').strip(),
                sha256=hashlib.sha256(raw).hexdigest(), checksum=checksum,
                entry_flags=flags, names=names, tiles=tiles, images=images)


def projection(rotation, x, y, z, center_x, center_y, baseline):
    """OpenRCT2 Translate3DTo2DWithZ coordinate rotation in local units."""
    dx, dy = x - center_x, y - center_y
    rx = (dx, dy, -dx, -dy)[rotation]
    ry = (dy, -dx, -dy, dx)[rotation]
    return ry - rx, (rx + ry) / 2.0 - z + baseline


def mask_metrics(reference, rendered):
    import numpy as np
    import cv2
    ref = np.asarray(reference, dtype=bool)
    pred = np.asarray(rendered, dtype=bool)
    if ref.shape != pred.shape:
        raise ValueError('Reference and prediction differ in dimensions')
    overlap = int(np.count_nonzero(ref & pred))
    union = int(np.count_nonzero(ref | pred))
    interior = cv2.erode(ref.astype('uint8'), np.ones((5, 5), 'uint8')).astype(bool)
    return dict(iou=round(overlap / max(1, union), 4),
                recall=round(overlap / max(1, int(ref.sum())), 4),
                precision=round(overlap / max(1, int(pred.sum())), 4),
                missing_reference_pixels=int(np.count_nonzero(ref & ~pred)),
                excess_model_pixels=int(np.count_nonzero(pred & ~ref)),
                interior_missing_pixels=int(np.count_nonzero(interior & ~pred)))


def project_mesh_mask(mesh, view, image, cx, cy, baseline, supersample=2):
    import numpy as np
    import cv2
    h, w = image['height'], image['width']
    out = np.zeros((h * supersample, w * supersample), dtype=np.uint8)
    vertices = np.asarray(mesh.vertices)
    x, y, z = vertices.T
    u, v = projection(view, x, y, z, cx, cy, baseline)
    points = np.rint(np.column_stack(((u - image['x']) * supersample,
                                     (v - image['y']) * supersample))).astype(np.int32)
    for tri in mesh.faces:
        cv2.fillConvexPoly(out, np.ascontiguousarray(points[tri]), 1)
    if supersample > 1:
        out = cv2.resize(out, (w, h), interpolation=cv2.INTER_AREA)
    return out != 0


def evaluate(dat: dict, mesh, baseline: float, cx: float, cy: float):
    import numpy as np
    if len(dat['images']) < 4:
        raise ValueError('Expected four native preview rotations')
    views = []
    for i in range(4):
        image = dat['images'][i]
        ref = np.frombuffer(image['pixels'], dtype=np.uint8).reshape(
            (image['height'], image['width'])) != 0
        if not ref.any():
            raise ValueError(f'Empty preview rotation {i}')
        pred = project_mesh_mask(mesh, i, image, cx, cy, baseline)
        views.append({'rotation':i, **mask_metrics(ref, pred)})
    return dict(asset=dat['name'], source_sha256=dat['sha256'],
                experimental=True, production_renderer=False,
                registration={'center_x':cx, 'center_y':cy, 'baseline':baseline,
                              'independent_calibration':False},
                geometry={'triangles':int(len(mesh.faces)),
                          'watertight':bool(mesh.is_watertight),
                          'euler_number':int(mesh.euler_number)},
                views=views,
                gates={'all_four_views_present':True,
                       'min_view_iou':min(row['iou'] for row in views),
                       'maximum_interior_missing_pixels':max(row['interior_missing_pixels'] for row in views)},
                limitations=['Uses four native overview images, not tile paint-sequence composites',
                             'Does not establish correctness of doorways, palette materials, or hidden geometry',
                             'Mesh watertightness may be undesirable for a structure with genuine openings',
                             'Calibration is supplied externally, not independently estimated',
                             'A prediction tested on images used to reconstruct it is in-sample evidence'])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    inspect = commands.add_parser('inspect')
    inspect.add_argument('dat', type=Path)
    inspect.add_argument('--output', type=Path)
    compare = commands.add_parser('compare')
    compare.add_argument('dat', type=Path)
    compare.add_argument('mesh', type=Path)
    compare.add_argument('--baseline', type=float, required=True,
                         help='Explicit screen y offset (do not fit and claim independent validation)')
    compare.add_argument('--center-x', type=float)
    compare.add_argument('--center-y', type=float)
    compare.add_argument('--output', type=Path)
    compare.add_argument('--min-view-iou', type=float,
                         help='Fail if any view IoU falls below this value')
    compare.add_argument('--max-interior-misses', type=int,
                         help='Fail if any reference interior has more missing pixels')
    compare.add_argument('--max-triangles', type=int,
                         help='Fail if mesh exceeds this triangle count')
    compare.add_argument('--require-watertight', action='store_true',
                         help='Only for structures known to be closed')
    args = parser.parse_args(argv)
    dat = read_large_scenery(args.dat)
    if args.command == 'inspect':
        result = {'name':dat['name'], 'sha256':dat['sha256'], 'tiles':dat['tiles'],
                  'image_count':len(dat['images']),
                  'image_metadata':[{k:v for k,v in im.items() if k != 'pixels'}
                                    for im in dat['images']]}
    else:
        import trimesh
        mesh = trimesh.load(args.mesh, force='mesh')
        if len(mesh.faces) == 0:
            raise ValueError('Empty mesh')
        center_x = args.center_x if args.center_x is not None else (
            min(tile['x'] for tile in dat['tiles']) + max(tile['x'] for tile in dat['tiles']) + 32) / 2
        center_y = args.center_y if args.center_y is not None else (
            min(tile['y'] for tile in dat['tiles']) + max(tile['y'] for tile in dat['tiles']) + 32) / 2
        result = evaluate(dat, mesh, args.baseline, center_x, center_y)
        gates = {}
        if args.min_view_iou is not None:
            gates['min_view_iou'] = result['gates']['min_view_iou'] >= args.min_view_iou
        if args.max_interior_misses is not None:
            gates['max_interior_misses'] = (
                result['gates']['maximum_interior_missing_pixels'] <= args.max_interior_misses)
        if args.max_triangles is not None:
            gates['max_triangles'] = result['geometry']['triangles'] <= args.max_triangles
        if args.require_watertight:
            gates['watertight'] = result['geometry']['watertight']
        result['requested_gates'] = gates
        result['accepted_by_requested_gates'] = bool(gates) and all(gates.values())
    serialized = json.dumps(result, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(serialized + '\n')
    else:
        print(serialized)
    if args.command == 'compare' and result['requested_gates'] and not all(result['requested_gates'].values()):
        raise SystemExit(1)
    return result


if __name__ == '__main__':
    main()
