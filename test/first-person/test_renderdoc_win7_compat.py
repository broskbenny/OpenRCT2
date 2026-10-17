"""Integrity and no-regression checks for Windows 7 RenderDoc candidates.

These check the actual committed PE files and the zero-build isolated sweep.
We deliberately do not install/load RenderDoc in the test host.
"""
from __future__ import annotations

import hashlib
from pathlib import Path
import struct
import unittest

ROOT = Path(__file__).resolve().parents[2]
BIN = ROOT / "third_party/renderdoc/compatibility/windows-x64"
EXPECTED = {
    "v1.45": ("313f450bfa1f7f7d5f50cc7ee80d9f99b68d3fe873e25eb865ff0433e359f7b3", 27160728),
    "v1.32": ("04c52cf6363f1379162a91ff81a8ac7fa98e649b84e470ee197e4608da4fe745", 21535496),
    "v1.20": ("fd1c55418f327029583d2fb6890a7ff616f3112203c8cf929308360c5a19baa1", 19449632),
}


class RenderDocWin7Compatibility(unittest.TestCase):
    def test_exact_official_candidate_binaries(self):
        for version, (sha256, size) in EXPECTED.items():
            with self.subTest(version=version):
                path = BIN / version / "renderdoc.dll"
                self.assertTrue(path.is_file(), str(path))
                data = path.read_bytes()
                self.assertEqual(len(data), size)
                self.assertEqual(hashlib.sha256(data).hexdigest(), sha256)
                self.assertEqual(data[:2], b"MZ")
                offset = struct.unpack_from("<I", data, 0x3C)[0]
                self.assertEqual(data[offset:offset+4], b"PE\\0\\0".replace(b"\\0", b"\0"))
                self.assertEqual(struct.unpack_from("<H", data, offset+4)[0], 0x8664)
                self.assertTrue((BIN / version / "renderdoc-LICENSE.md").is_file())

    def test_runner_is_isolated_and_legacy_compatible(self):
        runner = (ROOT / "test/first-person/run_renderdoc_win7_matrix.ps1").read_text()
        probe = (ROOT / "test/first-person/probe_renderdoc_windows.ps1").read_text()
        for version, (sha256, _) in EXPECTED.items():
            self.assertIn(version, runner)
            self.assertIn(sha256, runner)
        self.assertIn("WaitForExit(12000)", runner)
        self.assertIn("$child.Kill()", runner)
        self.assertIn("renderdoc-win7-matrix.log", runner)
        self.assertNotIn("$PSScriptRoot", runner)
        self.assertNotIn("Invoke-WebRequest", runner)
        self.assertNotIn("openrct2.exe", runner.lower())
        self.assertIn("[string]$ExpectedSha256", probe)
        self.assertIn("[Environment]::Exit(7)", probe)
        self.assertNotIn("$sha.Dispose()", probe)


if __name__ == "__main__":
    unittest.main()
