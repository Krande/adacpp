# The OCC-free native STEP->GLB pipeline as a STANDALONE embind wasm module — no OCCT, no pyodide,
# no nanobind, no Python. Single-threaded (no -pthread, so no SharedArrayBuffer / COOP-COEP needed);
# streams the STEP from OPFS via WASMFS so multi-GB files never have to fit in the wasm heap.
if (BUILD_STEP_GLB_WASM)
    add_executable(adacpp_step_glb
            ${CMAKE_SOURCE_DIR}/src/cad/cad_wasm.cpp
            ${CMAKE_SOURCE_DIR}/src/geom/neutral/ngeom_tessellate.cpp
            ${CMAKE_SOURCE_DIR}/src/geom/neutral/ngeom_boolean.cpp
            ${CMAKE_SOURCE_DIR}/src/geom/neutral/ngeom_meshopt.cpp
            ${LIBTESS2_SOURCES}
            ${MESHOPT_SOURCES})
    target_link_libraries(adacpp_step_glb PRIVATE manifold)
    set_target_properties(adacpp_step_glb PROPERTIES OUTPUT_NAME "adacpp_step_glb" SUFFIX ".js")
    target_link_options(adacpp_step_glb PRIVATE
            "-lembind"
            "-sWASMFS=1"              # WASMFS + OPFS backend (file-backed pread, not heap)
            "-sFORCE_FILESYSTEM=1"
            "-sEXPORTED_RUNTIME_METHODS=['FS']" # JS-side FS for OPFS mount + node smoke test
            "-sALLOW_MEMORY_GROWTH=1"
            "-sMODULARIZE=1"
            "-sEXPORT_ES6=1"
            "-sEXPORT_NAME=createAdacppStepGlb"
            "-sENVIRONMENT=web,worker,node"
            "-sSTACK_SIZE=1048576")
    return() # standalone target; skip the legacy WASM_UTILS stub below
endif ()

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