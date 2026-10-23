"""Legacy small-scenery DAT and exact-production-fixture regression tests.

Tests construct a small synthetic DAT in memory. No proprietary DAT artwork
or extracted native sprite pixels are stored in the repository.
"""
import json
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest

from real_small_scenery_eval import (
    decode_g1, decode_sawyer_rle, decode_small_scenery, write_packet,
)


def synthetic_dat():
    flags = (1 << 0) | (1 << 3) | (1 << 2)
    body = bytearray(28)
    struct.pack_into("<I", body, 6, flags)
    body[10] = 16
    body.extend(b"\xff")
    body.extend(bytes(16))
    width = height = 32
    source = []
    for rotation in range(4):
        pixels = bytearray(width * height)
        for row in range(10, 22):
            for col in range(10, 22):
                pixels[row * width + col] = 140 + rotation
        source.append(bytes(pixels))
    image_data = b"".join(source)
    body.extend(struct.pack("<II", 4, len(image_data)))
    for rotation in range(4):
        body.extend(struct.pack(
            "<IhhhhHH", rotation * width * height,
            width, height, -16, -24, 1, 0
        ))
    body.extend(image_data)
    # Sawyer encoding 1 is a sequence of signed length runs. Encode the
    # synthetic body as literal blocks of at most 128 bytes.
    packed = bytearray()
    for index in range(0, len(body), 128):
        chunk = body[index:index + 128]
        packed.append(len(chunk) - 1)
        packed.extend(chunk)
    raw = struct.pack("<I8sIBI", 1, b"SYNTEST1", 0, 1, len(packed))
    return raw + packed, source


class RealSmallSceneryDatTest(unittest.TestCase):
    def test_legacy_decoder_retains_native_palette_and_registration(self):
        raw, source = synthetic_dat()
        result = decode_small_scenery(raw)
        self.assertEqual(result["identifier"], "SYNTEST1")
        self.assertEqual(result["image_count"], 4)
        self.assertEqual(result["height"], 16)
        self.assertEqual(result["flags"], 13)
        self.assertEqual([im["pixels"] for im in result["images"]], source)
        self.assertEqual([(im["x"], im["y"]) for im in result["images"]],
                         [(-16, -24)] * 4)
        self.assertEqual([im["opaque"] for im in result["images"]], [144] * 4)

    def test_run_length_g1_preserves_sparse_openings(self):
        # Two G1 rows, each with one terminating run; other pixels stay zero.
        data = struct.pack("<HH", 4, 10) + bytes(
            [0x84, 0, 11, 12, 13, 14, 0x82, 1, 15, 16]
        )
        pixels = decode_g1(data, 0, 4, 2, 5)
        self.assertEqual(pixels, bytes([11, 12, 13, 14, 0, 15, 16, 0]))

    def test_other_object_types_are_not_misclassified(self):
        raw, _ = synthetic_dat()
        replaced = bytearray(raw)
        struct.pack_into("<I", replaced, 0, 2)
        with self.assertRaisesRegex(ValueError, "Not a small-scenery"):
            decode_sawyer_rle(bytes(replaced))

    @unittest.skipUnless(shutil.which("g++"), "C++20 compiler unavailable")
    def test_compiled_production_fixture_accepts_real_dat_packet_format(self):
        raw, _ = synthetic_dat()
        asset = decode_small_scenery(raw)
        with tempfile.TemporaryDirectory() as tmp:
            packet = Path(tmp) / "synthetic.bin"
            write_packet(asset, packet)
            verify = Path(__file__).with_name("verify_material_bake.py")
            fixture = Path(__file__).with_name("real_small_scenery_fixtures.inc")
            process = subprocess.run([
                sys.executable, str(verify),
                "--fixture", str(fixture), "--run-arg", str(packet)
            ], capture_output=True, text=True, timeout=180)
            self.assertEqual(process.returncode, 0, process.stderr[-5000:])
            records = [json.loads(line) for line in process.stdout.splitlines()
                       if line.startswith('{"index":')]
            self.assertEqual(len(records), 1, process.stdout[-2000:])
            record = records[0]
            self.assertTrue(record["input_valid"])
            self.assertEqual(record["index"], 0)
            self.assertEqual(record["height"], 16)
            self.assertEqual(record["native_opaque_pixels"], 576)
            self.assertEqual(record["native_palette_entries"], 4)
            self.assertIn("production_elapsed_ms", record)
            self.assertEqual(len(record["registered_offsets"]), 4)


if __name__ == "__main__":
    unittest.main()
