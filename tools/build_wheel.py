"""Pack a pyodide-targeted wheel from a cmake-staged adacpp tree.

Layout consumed:
    <staged>/adacpp/_ada_cpp_ext_impl.so
    <staged>/adacpp/__init__.py
    <staged>/adacpp/cad/__init__.py
    ...

Layout produced:
    dist/ada_cpp-<version>-cp312-cp312-emscripten_3_1_58_wasm32.whl

Usage:
    python tools/build_wheel.py <staged_dir> <out_dir>

Why hand-rolled: scikit-build-core was removed and pyodide-build only drives
projects that hand it a build-system to invoke. We already have the .so from a
plain `cmake --build && cmake --install`; the only thing left is to wrap it in
the wheel envelope.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import sys
import tomllib
import zipfile
from pathlib import Path

# Pyodide 0.27.x ships emscripten 3.1.58. The wheel platform tag must match for
# pyodide's micropip to accept it. Pyodide 0.28+ standardises on
# pyodide_2025_0_wasm32 — bump this when we bump pyodide.
PLATFORM_TAG = "emscripten_3_1_58_wasm32"
PYTHON_TAG = "cp312"
ABI_TAG = "cp312"


def _record_line(arcname: str, data: bytes) -> str:
    digest = hashlib.sha256(data).digest()
    b64 = base64.urlsafe_b64encode(digest).rstrip(b"=").decode("ascii")
    return f"{arcname},sha256={b64},{len(data)}"


def _read_metadata(pyproject: Path) -> tuple[str, str, str, str]:
    with pyproject.open("rb") as f:
        pp = tomllib.load(f)
    project = pp["project"]
    name = project["name"]
    version = project["version"]
    summary = project.get("description", "")
    requires_python = project.get("requires-python", "")
    return name, version, summary, requires_python


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("staged", type=Path, help="path to the cmake-staged tree containing adacpp/")
    ap.add_argument("out_dir", type=Path, help="directory to write the .whl into")
    ap.add_argument("--pyproject", type=Path, default=Path(__file__).parent.parent / "pyproject.toml")
    args = ap.parse_args()

    name, version, summary, requires_python = _read_metadata(args.pyproject)
    dist_name = name.replace("-", "_")  # PEP 427 normalised name

    pkg_root = args.staged / "adacpp"
    if not pkg_root.is_dir():
        print(f"error: {pkg_root} not found — did cmake --install run?", file=sys.stderr)
        return 1

    args.out_dir.mkdir(parents=True, exist_ok=True)
    wheel_name = f"{dist_name}-{version}-{PYTHON_TAG}-{ABI_TAG}-{PLATFORM_TAG}.whl"
    wheel_path = args.out_dir / wheel_name

    dist_info = f"{dist_name}-{version}.dist-info"
    metadata = (
        "Metadata-Version: 2.1\n"
        f"Name: {name}\n"
        f"Version: {version}\n"
        f"Summary: {summary}\n"
        f"Requires-Python: {requires_python}\n"
    )
    wheel_meta = (
        "Wheel-Version: 1.0\n"
        "Generator: adacpp build_wheel.py\n"
        "Root-Is-Purelib: false\n"
        f"Tag: {PYTHON_TAG}-{ABI_TAG}-{PLATFORM_TAG}\n"
    )

    records: list[str] = []
    with zipfile.ZipFile(wheel_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        # Package payload (adacpp/...).
        for path in sorted(pkg_root.rglob("*")):
            if not path.is_file():
                continue
            arcname = str(path.relative_to(args.staged)).replace("\\", "/")
            data = path.read_bytes()
            zf.writestr(arcname, data)
            records.append(_record_line(arcname, data))

        # Dist-info metadata.
        for fname, content in (("METADATA", metadata), ("WHEEL", wheel_meta)):
            arcname = f"{dist_info}/{fname}"
            data = content.encode("utf-8")
            zf.writestr(arcname, data)
            records.append(_record_line(arcname, data))

        # RECORD itself; its own line has empty hash/size by convention.
        record_arcname = f"{dist_info}/RECORD"
        records.append(f"{record_arcname},,")
        zf.writestr(record_arcname, "\n".join(records) + "\n")

    print(wheel_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
