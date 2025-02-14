set(WASM_SOURCES src/wasm_utils.cpp)
set(WASM_HEADERS src/wasm_utils.h)

add_library(WASM_UTILS ${WASM_SOURCES} ${WASM_HEADERS})
# Custom output for WebAssembly
set_target_properties(WASM_UTILS PROPERTIES
        OUTPUT_NAME "adacpp_utils"
        SUFFIX ".wasm")

# Export the `multiply` function for use in JS
# Properly escape EXPORTED_FUNCTIONS flag
target_link_options(WASM_UTILS PRIVATE
        "-sEXPORTED_FUNCTIONS=[\"_multiply\"]"
        "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap']")