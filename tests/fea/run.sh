#!/usr/bin/env bash
# Build + run the native FEA kernel tests (no OCC / nanobind / Python needed).
#
#   bash tests/fea/run.sh              # $CXX, else g++
#   bash tests/fea/run.sh --wasm       # the same tests compiled to wasm (+SIMD) with em++, run under node
#
# --wasm, not CXX=em++: inside `pixi run -e wasm` the env's compiler activation sets CXX to the conda
# g++, so `CXX=em++ pixi run -e wasm bash tests/fea/run.sh` quietly ran the NATIVE build.
#
# The flags are the kernels' numerics contract (see src/fea/fea_arrays.h): no FMA contraction, no
# fast-math. The tests themselves compare against longhand references compiled with the same flags.
set -euo pipefail
cd "$(dirname "$0")/../.."

CXX="${CXX:-g++}"
[[ "${1:-}" == "--wasm" ]] && CXX=em++
SRC=(src/fea/fea_arrays.cpp src/fea/superpose.cpp src/fea/derive.cpp src/fea/artefact_io.cpp src/fea/envelope.cpp
    src/fea/field_ops.cpp tests/fea/test_fea.cpp)
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT

if [[ "$(basename "$CXX")" == em++* ]]; then
    # Compile at -O3 and link at -O1, like the modules (cmake/wasm_toolchain.cmake): a link at -O2 or
    # above runs wasm-opt, and the binaryen conda-forge pairs with emscripten 4.0.9 cannot run it.
    objs=()
    for src in "${SRC[@]}"; do
        obj="$out/$(basename "$src" .cpp).o"
        "$CXX" -std=c++20 -O3 -msimd128 -ffp-contract=off -fwasm-exceptions -c "$src" -o "$obj"
        objs+=("$obj")
    done
    # NODERAWFS: the test's temp files go to the host file system.
    "$CXX" -O1 -fwasm-exceptions -sNODERAWFS=1 -sALLOW_MEMORY_GROWTH=1 "${objs[@]}" -o "$out/test_fea.js"
    node "$out/test_fea.js"
else
    "$CXX" -std=c++20 -O3 -ffp-contract=off -fno-fast-math -Wall "${SRC[@]}" -o "$out/test_fea"
    "$out/test_fea"
fi
