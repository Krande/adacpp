#!/usr/bin/env bash
# SPIKE (option B, adapy-plugins#177 step 2d): build + pack the three Pyodide wheels.
#
#   ada_occt  libocct.so         (OCCT V8_0_0, shared side module)
#   ada_cpp   _ada_cpp_ext_impl  (NEEDS libocct.so)
#   gmsh      gmsh.py + libgmsh  (4.15.2, NEEDS libocct.so)
#
# Run inside the wasm env after `pixi run -e wasm xbuildenv-install`:
#   pixi run -e wasm bash tools/pack_gmsh_spike.sh        then   node tools/test_gmsh_pyodide.js
set -euo pipefail
cd "$(dirname "$0")/.."
B=build/wasm-pyodide-shared
INC=$(pyodide config get python_include_dir | tail -1)
emcmake cmake --preset wasm-pyodide -B "$B" -DPython_INCLUDE_DIR="$INC" -DTINY_INCLUDE_DIR="$CONDA_PREFIX/include" \
    -DADACPP_WASM_SHARED_OCCT=ON -DADACPP_WASM_GMSH=ON
cmake --build "$B" -j "${JOBS:-$(nproc)}"
rm -rf "$B/install" dist
cmake --install "$B" --prefix "$B/install"
python tools/build_wheel.py "$B/install/wheel" dist
OCCT_SRC="$B/_deps/occt-src"
python tools/build_aux_wheel.py "$B/install/wheel-occt" dist --name ada-occt --version 8.0.0 \
    --summary "OpenCASCADE Technology V8_0_0 as a shared Pyodide side module (adacpp spike)" \
    --license "LGPL-2.1-only WITH OCCT-exception-1.0" \
    --license-file "$OCCT_SRC/LICENSE_LGPL_21.txt" --license-file "$OCCT_SRC/OCCT_LGPL_EXCEPTION.txt"
python tools/build_aux_wheel.py "$B/install/wheel-gmsh" dist --name gmsh --version 4.15.2 \
    --summary "gmsh 4.15.2 for Pyodide over the shared OCCT (adacpp spike)" \
    --license "GPL-2.0-or-later" \
    --license-file "$B/install/wheel-gmsh-licenses/LICENSE.txt" \
    --license-file "$B/install/wheel-gmsh-licenses/CREDITS.txt"
ls -la dist
