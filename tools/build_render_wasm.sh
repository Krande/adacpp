#!/usr/bin/env bash
# Build the browser WebGPU renderer: adacpp_render.js + adacpp_render.wasm.
#
# Needs emscripten >= 4.0.10 for the emdawnwebgpu port; conda-forge stops at 4.0.9 (pyodide's), so this
# runs inside the emscripten/emsdk image unless emcc is already new enough on PATH (as it is inside that
# image, e.g. adapy's deploy/Dockerfile.viewer build stage).
#
#   tools/build_render_wasm.sh [out_dir]      default out_dir: build-render-wasm/dist
set -euo pipefail

EMSDK_IMAGE="${EMSDK_IMAGE:-emscripten/emsdk:6.0.11}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$ROOT/build-render-wasm/dist}"

if ! command -v emcc >/dev/null 2>&1 || [ "${ADACPP_RENDER_USE_DOCKER:-0}" = "1" ]; then
    mkdir -p "$OUT"
    OUT_ABS="$(cd "$OUT" && pwd)"
    exec docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp \
        -v "$ROOT":/src -v "$OUT_ABS":/out -w /src "$EMSDK_IMAGE" \
        bash -c 'export EM_CACHE=/tmp/emcache && bash tools/build_render_wasm.sh /out'
fi

BUILD="${ADACPP_RENDER_BUILD_DIR:-/tmp/adacpp-render-wasm-build}"
# Unpack the emdawnwebgpu port into the cache once, up front: left to the parallel build, every emcc
# process races to unpack it and the cache lock can deadlock ("Accessing the Emscripten cache ... is
# taking a long time"), which hung a viewer image build for over ten minutes.
embuilder build emdawnwebgpu
emcmake cmake -S "$ROOT/src/render" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD" -j"$(nproc)"
mkdir -p "$OUT"
cp "$BUILD/adacpp_render.js" "$BUILD/adacpp_render.wasm" "$OUT/"
ls -la "$OUT"
