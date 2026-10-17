#!/usr/bin/env python3
"""CI-only vendoring of RenderDoc v1.46 AMD64 hook and MIT license.

The DLL is committed so Windows 7 users build without network, ZIP, or
PowerShell/.NET runtime version dependencies.
"""
from __future__ import annotations

import hashlib
import io
from pathlib import Path
import struct
from urllib.request import urlopen
from zipfile import ZipFile

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "third_party/renderdoc/windows-x64"
DLL = OUT / "renderdoc.dll"
LICENSE = OUT / "renderdoc-LICENSE.md"
URL = "https://renderdoc.org/stable/1.46/RenderDoc_1.46_64.zip"


def pe_machine(data: bytes) -> int:
    if len(data) < 0x40 or data[:2] != b"MZ":
        raise ValueError("Missing MZ signature")
    offset = struct.unpack_from("<I", data, 0x3C)[0]
    if offset <= 0 or offset + 6 > len(data) or data[offset:offset + 4] != b"PE\0\0":
        raise ValueError("Invalid PE signature")
    return struct.unpack_from("<H", data, offset + 4)[0]


def main() -> None:
    if DLL.is_file() and LICENSE.is_file():
        data = DLL.read_bytes()
        if pe_machine(data) != 0x8664:
            raise ValueError("Bundled RenderDoc DLL is not AMD64")
        print(f"Bundled AMD64 RenderDoc: {DLL} sha256={hashlib.sha256(data).hexdigest()}")
        return

    print(f"Downloading official pinned RenderDoc portable archive: {URL}")
    with urlopen(URL, timeout=180) as response:
        archive_bytes = response.read()
    print(f"Archive bytes={len(archive_bytes)} sha256={hashlib.sha256(archive_bytes).hexdigest()}")
    with ZipFile(io.BytesIO(archive_bytes)) as archive:
        dlls = []
        for item in archive.infolist():
            if Path(item.filename).name.lower() == "renderdoc.dll":
                data = archive.read(item)
                machine = pe_machine(data)
                print(f"DLL {item.filename}: machine={machine:#06x}")
                if machine == 0x8664:
                    dlls.append((item, data))
        if len(dlls) != 1:
            raise RuntimeError(f"Expected one AMD64 DLL, found {len(dlls)}")
        dll_info, payload = dlls[0]
        parent = Path(dll_info.filename).parent
        licenses = [
            item for item in archive.infolist()
            if Path(item.filename).parent == parent and Path(item.filename).name.lower() == "license.md"
        ]
        if len(licenses) != 1:
            raise RuntimeError(f"Expected LICENSE.md next to AMD64 DLL, found {len(licenses)}")
        legal = archive.read(licenses[0])
        if b"Permission is hereby granted" not in legal and b"MIT License" not in legal:
            raise RuntimeError("Unexpected RenderDoc license contents")
    OUT.mkdir(parents=True, exist_ok=True)
    DLL.write_bytes(payload)
    LICENSE.write_bytes(legal)
    print(f"Vendored AMD64 DLL sha256={hashlib.sha256(payload).hexdigest()}")


if __name__ == "__main__":
    main()
