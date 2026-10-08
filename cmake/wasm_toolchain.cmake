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
set(CMAKE_C_FLAGS_DEBUG "" CACHE STRING "C flags for debug" FORCE)
set(CMAKE_CXX_FLAGS_DEBUG "" CACHE STRING "CXX flags for debug" FORCE)

# Conda's compiler activation injects GNU-ld flags via LDFLAGS (--sort-common,
# --as-needed, -z relro/now, --disable-new-dtags, -rpath) that wasm-ld doesn't
# recognise. Clear the cmake linker flag variables so they aren't passed to em++.
set(CMAKE_EXE_LINKER_FLAGS "" CACHE STRING "exe linker flags" FORCE)
set(CMAKE_SHARED_LINKER_FLAGS "" CACHE STRING "shared linker flags" FORCE)
set(CMAKE_MODULE_LINKER_FLAGS "" CACHE STRING "module linker flags" FORCE)

# Optimisation lives in the Release flags, and Release is the default. This file used to clear the
# Release flags along with the conda ones above, and the wbuild-* tasks set no CMAKE_BUILD_TYPE, so
# every wasm module -- and the manifold / meshoptimizer / libtess2 code inside it -- compiled at -O0
# and linked with no wasm-opt and ASSERTIONS=1 (the debug libc / wasmfs variants). NDEBUG is already
# global (deps_detria.cmake), so -O3 changes code generation, not which asserts run. No -ffast-math
# here or anywhere: the FEA kernels must match the native build bit for bit.
#
# Link at -O1, which is the highest level that does not run binaryen's wasm-opt: -O1 drops the
# assertions and the debug system libraries, -O2 and up add the wasm-opt pass. That pass cannot run
# here. conda-forge's emscripten 4.0.9 requires binaryen 117 (emscripten itself expects 123, and
# conda-forge has no binaryen newer than 121). Binaryen 117 rejects the feature flags emscripten passes
# it (--enable-bulk-memory-opt, --enable-call-indirect-overlong) and, at -O3/-Os/-Oz, the
# --no-stack-ir from metadce. When a conda-forge emscripten pairs with a current binaryen, raise this
# to -O2 or -O3. (CMake repeats the -O3 compile flags on the link line; emcc honours the last -O,
# which is this -O1.)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE
        Release
        CACHE STRING
        "Build type (wasm defaults to Release)"
        FORCE
    )
endif()
set(CMAKE_C_FLAGS_RELEASE "-O3" CACHE STRING "C flags for release" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "-O3" CACHE STRING "CXX flags for release" FORCE)
set(CMAKE_EXE_LINKER_FLAGS_RELEASE
    "-O1"
    CACHE STRING
    "exe linker flags for release"
    FORCE
)
set(CMAKE_SHARED_LINKER_FLAGS_RELEASE
    "-O1"
    CACHE STRING
    "shared linker flags for release"
    FORCE
)
set(CMAKE_MODULE_LINKER_FLAGS_RELEASE
    "-O1"
    CACHE STRING
    "module linker flags for release"
    FORCE
)

# Set output directory for WebAssembly build
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/wasm_output")
