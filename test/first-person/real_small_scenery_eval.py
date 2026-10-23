#!/usr/bin/env python3
"""Run the EXACT production small-scenery geometry/material bake on native DAT artwork.

No proprietary artwork enters the repository. Legacy Sawyer-RLE and G1 pixels
are decoded into a bounded binary interchange packet; the C++ harness then
compiles the real BuildSmallSceneryGeometry and visual-hull functions via
verify_material_bake.py. The four native views are in-sample evidence only:
they cannot prove unseen surfaces or arbitrary perspective appearance.

Examples:
  python test/first-person/real_small_scenery_eval.py MMENT1.DAT MMENT2.DAT MMENT3.DAT
  python test/first-person/real_small_scenery_eval.py \
      --archive /path/to/ObjData.zip --names MMENT1.DAT MMENT2.DAT MMENT3.DAT \
      --output /tmp/ornament-production.json
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import zipfile


MAX_DECODED = 16 * 1024 * 1024
MAX_IMAGE_PIXELS = 512 * 512


def require(data: bytes, offset: int, size: int) -> None:
    if offset < 0 or size < 0 or offset + size > len(data):
        raise ValueError("Malformed or truncated DAT stream")


def decode_sawyer_rle(raw: bytes) -> tuple[str, bytes]:
    require(raw, 0, 21)
    entry_flags, name, _checksum = struct.unpack_from("<I8sI", raw)
    if entry_flags & 15 != 1:
        raise ValueError("Not a small-scenery DAT object")
    encoding, declared_size = struct.unpack_from("<BI", raw, 16)
    if encoding != 1 or declared_size != len(raw) - 21:
        raise ValueError("Unsupported DAT compression or chunk length")
    out = bytearray()
    cursor = 21
    while cursor < len(raw):
        code = raw[cursor]
        cursor += 1
        if code & 128:
            require(raw, cursor, 1)
            out.extend(bytes((raw[cursor],)) * (257 - code))
            cursor += 1
        else:
            count = code + 1
            require(raw, cursor, count)
            out.extend(raw[cursor:cursor + count])
            cursor += count
        if len(out) > MAX_DECODED:
            raise ValueError("Decoded DAT exceeds size limit")
    return name.decode("ascii", "replace").strip(), bytes(out)


def decode_g1(data: bytes, offset: int, width: int, height: int, flags: int) -> bytes:
    if width <= 0 or height <= 0 or width * height > MAX_IMAGE_PIXELS:
        raise ValueError("Invalid or oversized G1 sprite")
    if not (flags & 4):
        require(data, offset, width * height)
        return data[offset:offset + width * height]
    require(data, offset, height * 2)
    pixels = bytearray(width * height)
    for y in range(height):
        relative = struct.unpack_from("<H", data, offset + 2 * y)[0]
        cursor = offset + relative
        for _ in range(256):
            require(data, cursor, 2)
            code, x = data[cursor:cursor + 2]
            cursor += 2
            count = code & 127
            if x + count > width:
                raise ValueError("G1 run exceeds sprite row")
            require(data, cursor, count)
            pixels[y * width + x:y * width + x + count] = data[cursor:cursor + count]
            cursor += count
            if code & 128:
                break
        else:
            raise ValueError("Unterminated G1 sprite row")
    return bytes(pixels)


def decode_small_scenery(raw: bytes) -> dict:
    identifier, data = decode_sawyer_rle(raw)
    require(data, 6, 22)
    flags, height = struct.unpack_from("<IB", data, 6)
    if not (1 <= height <= 255):
        raise ValueError("Invalid small-scenery height")
    at = 28  # Same 6+4+1+1+2+2+4+2+2+2+2 header contract as ReadLegacy.
    for _ in range(128):
        require(data, at, 1)
        language = data[at]
        at += 1
        if language == 255:
            break
        end = data.find(b"\x00", at)
        if end < 0:
            raise ValueError("Unterminated legacy language name")
        at = end + 1
    else:
        raise ValueError("Unterminated legacy language table")
    require(data, at, 16)
    at += 16  # RCTObjectEntry scenery-group reference.
    if flags & (1 << 15):  # SmallSceneryFlag::hasFrameOffsets.
        require(data, at, 1)
        at += 1  # First frame offset is stored explicitly.
        for _ in range(4096):
            require(data, at, 1)
            code = data[at]
            at += 1
            if code == 255:
                break
        else:
            raise ValueError("Frame-offset list exceeds limit")
    require(data, at, 8)
    count, data_size = struct.unpack_from("<II", data, at)
    at += 8
    if not (4 <= count <= 4096 and data_size <= MAX_DECODED):
        raise ValueError("No four native sprite rotations")
    require(data, at, count * 16 + data_size)
    image_data = data[at + count * 16:at + count * 16 + data_size]
    images = []
    for rotation in range(4):
        offset, width, sprite_height, ox, oy, image_flags, zoom = struct.unpack_from(
            "<IhhhhHH", data, at + rotation * 16
        )
        if image_flags & 1 == 0:
            raise ValueError("Unexpected nontransparent G1 small-scenery image")
        pixels = decode_g1(image_data, offset, width, sprite_height, image_flags)
        images.append({
            "width": width, "height": sprite_height, "x": ox, "y": oy,
            "pixels": pixels,
            "opaque": sum(pixel != 0 for pixel in pixels),
            "zoom_offset": zoom,
        })
    return {
        "identifier": identifier,
        "sha256": hashlib.sha256(raw).hexdigest(),
        "flags": flags,
        "height": height,
        "image_count": count,
        "images": images,
    }


def write_packet(asset: dict, path: Path) -> None:
    with path.open("wb") as packet:
        packet.write(struct.pack("<4sII", b"FPS1", asset["flags"], asset["height"]))
        for image in asset["images"]:
            packet.write(struct.pack(
                "<hhhhI", image["width"], image["height"],
                image["x"], image["y"], len(image["pixels"])
            ))
            packet.write(image["pixels"])


def assets_from_archive(path: Path, names: list[str]) -> list[tuple[str, bytes]]:
    with zipfile.ZipFile(path) as archive:
        members = {}
        for info in archive.infolist():
            if info.is_dir() or not info.filename.lower().endswith(".dat"):
                continue
            members.setdefault(Path(info.filename).name.upper(), []).append(info)
        selections = []
        for name in names:
            matches = members.get(Path(name).name.upper(), [])
            if len(matches) != 1:
                raise ValueError(f"{name}: expected one archive member, found {len(matches)}")
            item = matches[0]
            if item.file_size > MAX_DECODED:
                raise ValueError(f"{name}: archived object exceeds limit")
            selections.append((name, archive.read(item)))
        return selections


def run(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("assets", nargs="*", type=Path, help="Small-scenery DAT paths")
    parser.add_argument("--archive", type=Path, help="ObjData.zip (DATs are not extracted)")
    parser.add_argument("--names", nargs="+", help="Object filenames inside --archive")
    parser.add_argument("--compiler", default="g++")
    parser.add_argument("--export-dir", type=Path,
                        help="Save emitted production OBJ/UV meshes and FPM1 indexed material buffers")
    parser.add_argument("--output", type=Path, help="JSON report")
    args = parser.parse_args(argv)
    if bool(args.archive) != bool(args.names):
        parser.error("--archive and --names must be provided together")
    raw_assets = [(p.name, p.read_bytes()) for p in args.assets]
    if args.archive:
        raw_assets.extend(assets_from_archive(args.archive, args.names))
    if not raw_assets:
        parser.error("Provide DAT files or --archive and --names")

    decoded = [(name, decode_small_scenery(raw)) for name, raw in raw_assets]
    with tempfile.TemporaryDirectory(prefix="openrct2-native-dat-") as temp:
        folder = Path(temp)
        packets = []
        for index, (_name, asset) in enumerate(decoded):
            packet_path = folder / f"native_{index:04d}.bin"
            write_packet(asset, packet_path)
            packets.append(packet_path)
        runner = Path(__file__).with_name("verify_material_bake.py")
        fixture = Path(__file__).with_name("real_small_scenery_fixtures.inc")
        command = [
            sys.executable, str(runner), "--compiler", args.compiler,
            "--fixture", str(fixture),
        ]
        if args.export_dir:
            args.export_dir.mkdir(parents=True, exist_ok=True)
            command.append("--run-arg=export:" + str(args.export_dir.resolve()))
        for packet in packets:
            command.extend(("--run-arg", str(packet)))
        process = subprocess.run(
            command, capture_output=True, text=True, check=False
        )
        if process.returncode:
            raise RuntimeError(
                "Production C++ reconstruction runner failed "
                f"(exit {process.returncode}):\n"
                + process.stderr[-6000:] + "\n" + process.stdout[-2000:]
            )
        results = [json.loads(line) for line in process.stdout.splitlines()
                   if line.startswith('{"index":')]
        if len(results) != len(decoded):
            raise RuntimeError("Incomplete production geometry report:\n" + process.stdout[-4000:])

    report = []
    for (name, asset), result in zip(decoded, results):
        if result.get("index") != len(report):
            raise RuntimeError("Production result ordering mismatch")
        report.append({
            "asset": name,
            "exports": (
                {"emitted_obj": str(args.export_dir.resolve() / f"asset_{len(report):04d}.obj"),
                 "indexed_materials": str(args.export_dir.resolve() / f"asset_{len(report):04d}.fpmat")}
                if args.export_dir and result.get("input_valid") else None
            ),
            "identifier": asset["identifier"],
            "sha256": asset["sha256"],
            "flags": asset["flags"],
            "height": asset["height"],
            "image_count": asset["image_count"],
            "rotations": [
                {key: value for key, value in image.items() if key != "pixels"}
                for image in asset["images"]
            ],
            "production": result,
        })
    document = {
        "version": 1,
        "production_geometry_and_material_bake": True,
        "native_artwork_in_repository": False,
        "source_pixel_decoder": "Standalone legacy Sawyer-RLE / G1 decode, production C++ consumes indexed pixels",
        "limitations": [
            "Four original camera views are not independent geometry ground truth",
            "Unseen surfaces and arbitrary-angle appearance remain unverified",
            "Input assumes a stationary small-scenery element occupying all quadrants",
            "Palette membership alone does not establish per-texel surface ownership",
            "Measurements are from headless production paths, not an OpenGL frame",
        ],
        "objects": report,
    }
    serialized = json.dumps(document, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(serialized)
    else:
        print(serialized, end="")
    return document


if __name__ == "__main__":
    run()
