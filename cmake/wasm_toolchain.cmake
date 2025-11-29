# cmake/wasm_toolchain.cmake
set(CMAKE_SYSTEM_NAME WebAssembly)
set(CMAKE_SYSTEM_VERSION 1)

#set(CMAKE_C_COMPILER "em++" CACHE STRING "Path to emscripten C compiler")
#set(CMAKE_CXX_COMPILER "em++" CACHE STRING "Path to emscripten C++ compiler")

set(CMAKE_C_COMPILER "emcc")
set(CMAKE_CXX_COMPILER "em++")

# Clear architecture-specific compiler flags that may come from conda environment
# These flags (-march=, -mtune=) are not compatible with WebAssembly target
set(CMAKE_C_FLAGS "" CACHE STRING "C flags" FORCE)
set(CMAKE_CXX_FLAGS "" CACHE STRING "CXX flags" FORCE)
set(CMAKE_C_FLAGS_RELEASE "" CACHE STRING "C flags for release" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "" CACHE STRING "CXX flags for release" FORCE)
set(CMAKE_C_FLAGS_DEBUG "" CACHE STRING "C flags for debug" FORCE)
set(CMAKE_CXX_FLAGS_DEBUG "" CACHE STRING "CXX flags for debug" FORCE)

# Required to link properly for WASM
set(CMAKE_EXE_LINKER_FLAGS "--bind")

# Set output directory for WebAssembly build
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/wasm_output")
