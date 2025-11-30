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

# Install Python package files
${PYTHON} -m pip install . --no-deps --no-build-isolation -vv
