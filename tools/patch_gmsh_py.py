"""Copy upstream gmsh's api/gmsh.py with the one change Pyodide needs: where the library is.

SPIKE (option B, adapy-plugins#177 step 2d). Upstream picks libgmsh.so.4.15 for every non-Windows,
non-Darwin system and then searches next to itself. The gmsh Pyodide wheel ships the side module as
gmsh.libs/libgmsh.so (the auditwheel layout Pyodide's loader searches), so on emscripten point there.
Everything else -- the whole ctypes API -- is upstream, byte for byte.

Usage: python tools/patch_gmsh_py.py <upstream gmsh.py> <out gmsh.py>
"""

from __future__ import annotations

import sys
from pathlib import Path

ANCHOR = 'else:\n    libname = "libgmsh.so.4.15"\n'
PATCH = (
    'elif __import__("sys").platform == "emscripten":\n'
    "    # Pyodide: the gmsh wheel ships the side module as gmsh.libs/libgmsh.so (adacpp spike)\n"
    '    libname = os.path.join("gmsh.libs", "libgmsh.so")\n'
)


def main() -> int:
    src, dst = Path(sys.argv[1]), Path(sys.argv[2])
    text = src.read_text(encoding="utf-8")
    if text.count(ANCHOR) != 1:
        print(f"error: anchor not found exactly once in {src}", file=sys.stderr)
        return 1
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(text.replace(ANCHOR, PATCH + ANCHOR), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
