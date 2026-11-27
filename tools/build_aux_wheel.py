"""Pack one of the SPIKE's extra Pyodide wheels (option B, adapy-plugins#177 step 2d).

  ada_occt  ada_occt.libs/libocct.so            OCCT 8.0.0 as a shared side module
  gmsh      gmsh.py + gmsh.libs/libgmsh.so      gmsh 4.15.2 over that OCCT

Same envelope as tools/build_wheel.py (pyodide_2025_0_wasm32, cp313), but the metadata comes from
the command line: neither is adacpp, so pyproject.toml does not describe them.

Usage:
  python tools/build_aux_wheel.py <staged_dir> <out_dir> --name N --version V --summary S
         --license L [--license-file F ...] [--requires R ...]
"""

from __future__ import annotations

import argparse
import sys
import zipfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from build_wheel import ABI_TAG, PLATFORM_TAG, PYTHON_TAG, _record_line


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("staged", type=Path)
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--name", required=True)
    ap.add_argument("--version", required=True)
    ap.add_argument("--summary", default="")
    ap.add_argument("--license", default="")
    ap.add_argument("--license-file", type=Path, action="append", default=[])
    ap.add_argument("--requires", action="append", default=[])
    args = ap.parse_args()

    dist = args.name.replace("-", "_")
    dist_info = f"{dist}-{args.version}.dist-info"
    args.out_dir.mkdir(parents=True, exist_ok=True)
    wheel_path = args.out_dir / f"{dist}-{args.version}-{PYTHON_TAG}-{ABI_TAG}-{PLATFORM_TAG}.whl"

    metadata = f"Metadata-Version: 2.1\nName: {args.name}\nVersion: {args.version}\nSummary: {args.summary}\n"
    if args.license:
        metadata += f"License: {args.license}\n"
    metadata += "".join(f"Requires-Dist: {r}\n" for r in args.requires)
    wheel_meta = (
        "Wheel-Version: 1.0\nGenerator: adacpp build_aux_wheel.py\nRoot-Is-Purelib: false\n"
        f"Tag: {PYTHON_TAG}-{ABI_TAG}-{PLATFORM_TAG}\n"
    )

    records: list[str] = []
    with zipfile.ZipFile(wheel_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:

        def put(arcname: str, data: bytes) -> None:
            zf.writestr(arcname, data)
            records.append(_record_line(arcname, data))

        for path in sorted(args.staged.rglob("*")):
            if path.is_file():
                put(str(path.relative_to(args.staged)).replace("\\", "/"), path.read_bytes())
        for lic in args.license_file:
            put(f"{dist_info}/licenses/{lic.name}", lic.read_bytes())
        put(f"{dist_info}/METADATA", metadata.encode())
        put(f"{dist_info}/WHEEL", wheel_meta.encode())
        records.append(f"{dist_info}/RECORD,,")
        zf.writestr(f"{dist_info}/RECORD", "\n".join(records) + "\n")

    print(wheel_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
