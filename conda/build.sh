#!/bin/bash

set -ex

# Build with CMake
cmake -B build -S . -G Ninja ${CMAKE_ARGS} \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_PYTHON=ON \
    -DBUILD_STP2GLB=ON \
    -DSTP2GLB_BIN_DIR="${PREFIX}/bin" \
    -DBUILD_WASM=OFF \
    -DPYTHON_EXECUTABLE="${PYTHON}" \
    -DPYTHON_INCLUDE="${PREFIX}/include" \
    -DCMAKE_PREFIX_PATH=$PREFIX \
    -DCMAKE_SYSTEM_PREFIX_PATH=$PREFIX \
    -DPYTHON_SITE_PACKAGES="${SP_DIR}"

cmake --build build
cmake --install build

# adacpp._render, the headless WebGPU renderer: its own CMake project (src/render), installed into the package.
cmake -B build-render -S src/render -G Ninja ${CMAKE_ARGS} \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH=$PREFIX \
    -DADACPP_RENDER_PYTHON=ON \
    -DPython_EXECUTABLE="${PYTHON}" \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}" \
    -DADACPP_RENDER_PY_INSTALL_DIR="${SP_DIR}/adacpp"
cmake --build build-render --target _render
cmake --install build-render

# Install Python package files
${PYTHON} -m pip install . --no-deps --no-build-isolation -vv
