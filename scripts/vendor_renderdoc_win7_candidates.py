#!/usr/bin/env python3
"""Vendor official RenderDoc x64 candidates for a Win7 loader compatibility sweep.

Runs only on GitHub Actions (modern HTTPS/ZIP). Windows 7 fetches binary files
by git and only runs the already-committed isolated probe; no extra install,
downloads from PowerShell, or OpenRCT2 application rebuild.
"""
from __future__ import annotations
import hashlib
import io
from pathlib import Path
import struct
from urllib.request import urlopen
from zipfile import ZipFile

ROOT = Path(__file__).resolve().parents[1]
VERSIONS = ("1.45", "1.32", "1.20")
BASE = ROOT / "third_party" / "renderdoc" / "compatibility" / "windows-x64"


def machine(data: bytes) -> int:
    if len(data) < 64 or data[:2] != b"MZ":
        raise RuntimeError("DLL lacks DOS header")
    offset = struct.unpack_from("<I", data, 60)[0]
    if offset + 6 > len(data) or data[offset:offset+4] != b"PE\0\0":
        raise RuntimeError("DLL lacks PE header")
    return struct.unpack_from("<H", data, offset + 4)[0]


def main():
    for version in VERSIONS:
        out = BASE / ("v" + version)
        dll = out / "renderdoc.dll"
        license_path = out / "renderdoc-LICENSE.md"
        if dll.exists() and license_path.exists():
            payload = dll.read_bytes()
            if machine(payload) != 0x8664:
                raise RuntimeError(f"{version} stored DLL is not AMD64")
        else:
            url = f"https://renderdoc.org/stable/{version}/RenderDoc_{version}_64.zip"
            print(f"DOWNLOAD official {url}", flush=True)
            with urlopen(url, timeout=120) as response:
                zip_data = response.read()
            print(f"ARCHIVE version={version} bytes={len(zip_data)} sha256={hashlib.sha256(zip_data).hexdigest()}", flush=True)
            with ZipFile(io.BytesIO(zip_data)) as archive:
                x64 = []
                for info in archive.infolist():
                    if info.filename.replace("\\", "/").split("/")[-1].lower() == "renderdoc.dll":
                        data = archive.read(info)
                        arch = machine(data)
                        print(f"CHECK {version} {info.filename} machine={arch:#06x}", flush=True)
                        if arch == 0x8664:
                            x64.append((info, data))
                if len(x64) != 1:
                    raise RuntimeError(f"{version}: found {len(x64)} AMD64 DLLs; expected one")
                info, payload = x64[0]
                root = info.filename.replace("\\", "/").rsplit("/", 1)[0]
                licenses = [entry for entry in archive.infolist()
                            if entry.filename.replace("\\", "/").rsplit("/", 1)[0] == root
                            and entry.filename.replace("\\", "/").split("/")[-1].lower() == "license.md"]
                if len(licenses) != 1:
                    raise RuntimeError(f"{version}: expected one LICENSE.md alongside x64 DLL")
                license_bytes = archive.read(licenses[0])
                if b"Permission is hereby granted" not in license_bytes:
                    raise RuntimeError(f"{version}: unexpected license content")
            out.mkdir(parents=True, exist_ok=True)
            dll.write_bytes(payload)
            license_path.write_bytes(license_bytes)
        print(f"CANDIDATE version={version} size={len(payload)} sha256={hashlib.sha256(payload).hexdigest()}", flush=True)


if __name__ == "__main__":
    main()
