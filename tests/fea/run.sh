#!/usr/bin/env bash
# Build + run the native FEA kernel tests (no OCC / nanobind / Python needed).
#
#   bash tests/fea/run.sh              # $CXX, else g++
#   CXX=em++ bash tests/fea/run.sh     # the same tests compiled to wasm (+SIMD) and run under node
#
# The flags are the kernels' numerics contract (see src/fea/fea_arrays.h): no FMA contraction, no
# fast-math. The tests themselves compare against longhand references compiled with the same flags.
set -euo pipefail
cd "$(dirname "$0")/../.."

CXX="${CXX:-g++}"
SRC=(src/fea/fea_arrays.cpp src/fea/superpose.cpp src/fea/derive.cpp src/fea/artefact_io.cpp src/fea/envelope.cpp
    src/fea/field_ops.cpp tests/fea/test_fea.cpp)
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT

if [[ "$(basename "$CXX")" == em++* ]]; then
    # NODERAWFS: the test's temp files go to the host file system.
    "$CXX" -std=c++20 -O3 -msimd128 -ffp-contract=off -fwasm-exceptions -sNODERAWFS=1 -sALLOW_MEMORY_GROWTH=1 \
        -sASSERTIONS=0 "${SRC[@]}" -o "$out/test_fea.js"
    node "$out/test_fea.js"
else
    "$CXX" -std=c++20 -O3 -ffp-contract=off -fno-fast-math -Wall "${SRC[@]}" -o "$out/test_fea"
    "$out/test_fea"
fi
