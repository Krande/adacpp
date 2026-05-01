# cmake/wasm_toolchain.cmake
set(CMAKE_SYSTEM_NAME WebAssembly)
set(CMAKE_SYSTEM_VERSION 1)

# Don't override CMAKE_C_COMPILER / CMAKE_CXX_COMPILER here — emcmake's
# Emscripten.cmake toolchain (imported via -DCMAKE_TOOLCHAIN_FILE=) has
# already set them to full paths. Overriding with the bare names "emcc" /
# "em++" looks fine for the top-level project(ada-cpp) (compiler already
# resolved when this file runs), but breaks any nested project() — e.g.
# OCCT's FetchContent_MakeAvailable triggers project(OCCT) which re-runs
# the compiler check and can't find "em++" without a full path.

# Clear architecture-specific compiler flags that may come from conda environment
# These flags (-march=, -mtune=) are not compatible with WebAssembly target
set(CMAKE_C_FLAGS "" CACHE STRING "C flags" FORCE)
set(CMAKE_CXX_FLAGS "" CACHE STRING "CXX flags" FORCE)
set(CMAKE_C_FLAGS_RELEASE "" CACHE STRING "C flags for release" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "" CACHE STRING "CXX flags for release" FORCE)
set(CMAKE_C_FLAGS_DEBUG "" CACHE STRING "C flags for debug" FORCE)
set(CMAKE_CXX_FLAGS_DEBUG "" CACHE STRING "CXX flags for debug" FORCE)

# Conda's compiler activation injects GNU-ld flags via LDFLAGS (--sort-common,
# --as-needed, -z relro/now, --disable-new-dtags, -rpath) that wasm-ld doesn't
# recognise. Clear the cmake linker flag variables so they aren't passed to em++.
set(CMAKE_EXE_LINKER_FLAGS    "" CACHE STRING "exe linker flags"    FORCE)
set(CMAKE_SHARED_LINKER_FLAGS "" CACHE STRING "shared linker flags" FORCE)
set(CMAKE_MODULE_LINKER_FLAGS "" CACHE STRING "module linker flags" FORCE)

# Set output directory for WebAssembly build
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/wasm_output")
