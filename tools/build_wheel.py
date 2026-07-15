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

# The wheel platform tag must match the target pyodide for micropip to accept
# it. Pyodide 0.28+/0.29.x (Python 3.13, emscripten 4.0.9) standardise on the
# pyodide_2025_0_wasm32 ABI tag (same as the ifcopenshell wasm wheels) — not the
# raw emscripten_X_Y_Z tag the 0.27.x line used.
PLATFORM_TAG = "pyodide_2025_0_wasm32"
PYTHON_TAG = "cp313"
ABI_TAG = "cp313"


def _record_line(arcname: str, data: bytes) -> str:
    digest = hashlib.sha256(data).digest()
    b64 = base64.urlsafe_b64encode(digest).rstrip(b"=").decode("ascii")
    return f"{arcname},sha256={b64},{len(data)}"


def _read_metadata(pyproject: Path) -> dict:
    with pyproject.open("rb") as f:
        pp = tomllib.load(f)
    project = pp["project"]
    lic = project.get("license", "")
    if isinstance(lic, dict):  # PEP 621 table form: {text = "..."} or {file = "..."}
        lic = lic.get("text", "")
    return {
        "name": project["name"],
        "version": project["version"],
        "summary": project.get("description", ""),
        "requires_python": project.get("requires-python", ""),
        "deps": list(project.get("dependencies", [])),
        "license": lic,
        "classifiers": list(project.get("classifiers", [])),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("staged", type=Path, help="path to the cmake-staged tree containing adacpp/")
    ap.add_argument("out_dir", type=Path, help="directory to write the .whl into")
    ap.add_argument("--pyproject", type=Path, default=Path(__file__).parent.parent / "pyproject.toml")
    args = ap.parse_args()

    meta = _read_metadata(args.pyproject)
    name, version = meta["name"], meta["version"]
    summary, requires_python, deps = meta["summary"], meta["requires_python"], meta["deps"]
    dist_name = name.replace("-", "_")  # PEP 427 normalised name

    pkg_root = args.staged / "adacpp"
    if not pkg_root.is_dir():
        print(f"error: {pkg_root} not found — did cmake --install run?", file=sys.stderr)
        return 1

    args.out_dir.mkdir(parents=True, exist_ok=True)
    wheel_name = f"{dist_name}-{version}-{PYTHON_TAG}-{ABI_TAG}-{PLATFORM_TAG}.whl"
    wheel_path = args.out_dir / wheel_name

    dist_info = f"{dist_name}-{version}.dist-info"
    # Every field here is sourced from pyproject so the wheel cannot drift from the project's own
    # declarations — this file hand-writes METADATA, and anything it forgets is simply absent from
    # the artifact with nothing to catch it.
    metadata = "Metadata-Version: 2.1\n" f"Name: {name}\n" f"Version: {version}\n" f"Summary: {summary}\n"
    # The wheel shipped GPL-3.0 code with no licence field, no classifier and no licence text —
    # nothing in it said what it was. The text itself is bundled further down, because GPL's own
    # notice promises the recipient "should have received a copy... along with this program".
    if meta["license"]:
        metadata += f"License: {meta['license']}\n"
    metadata += "".join(f"Classifier: {c}\n" for c in meta["classifiers"])
    metadata += f"Requires-Python: {requires_python}\n"
    # Requires-Dist is what makes micropip install numpy alongside the wheel. Omitting it (as this
    # did) ships a package whose mesh API kills the interpreter: Mesh.positions/.indices are
    # zero-copy nb::ndarray<nb::numpy> views, and with numpy absent nanobind recurses until the
    # stack blows rather than raising ImportError. Natively nothing noticed: conda always had numpy.
    metadata += "".join(f"Requires-Dist: {d}\n" for d in deps)
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

        # The licence text itself, so the artifact carries its own terms.
        lic_path = args.pyproject.parent / "LICENSE"
        if lic_path.is_file():
            arcname = f"{dist_info}/licenses/LICENSE"
            data = lic_path.read_bytes()
            zf.writestr(arcname, data)
            records.append(_record_line(arcname, data))
        else:
            print(f"warning: no LICENSE next to {args.pyproject} — wheel will ship without one", file=sys.stderr)

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
