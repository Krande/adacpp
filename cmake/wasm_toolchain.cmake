# cmake/wasm_toolchain.cmake
set(CMAKE_SYSTEM_NAME WebAssembly)
set(CMAKE_SYSTEM_VERSION 1)

#set(CMAKE_C_COMPILER "em++" CACHE STRING "Path to emscripten C compiler")
#set(CMAKE_CXX_COMPILER "em++" CACHE STRING "Path to emscripten C++ compiler")

set(CMAKE_C_COMPILER "emcc")
set(CMAKE_CXX_COMPILER "em++")

# Required to link properly for WASM
set(CMAKE_EXE_LINKER_FLAGS "--bind")

# Set output directory for WebAssembly build
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/wasm_output")
