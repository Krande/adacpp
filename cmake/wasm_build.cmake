# The OCC-free GLB diff (summarise + match + removed overlay) as a STANDALONE embind wasm module —
# no OCCT, no pyodide, no tinygltf. Reuses the portable diff core (glb_diff_native.h's
# summarize_glb_buf, RAM-decode path) + meshoptimizer + nlohmann json. Takes two GLB buffers, returns
# the viewer-ops + a red overlay GLB, entirely in-browser (no worker job / server memory).
if (BUILD_GLB_DIFF_WASM)
    add_executable(adacpp_glb_diff
            ${CMAKE_SOURCE_DIR}/src/cad/glb_diff_wasm.cpp
            ${CMAKE_SOURCE_DIR}/src/geom/neutral/ngeom_meshopt.cpp
            ${MESHOPT_SOURCES})
    # nlohmann json.hpp lives in the (wasm) conda env include — emscripten doesn't search it by default.
    target_include_directories(adacpp_glb_diff PRIVATE $ENV{CONDA_PREFIX}/include)
    # try/catch (nlohmann parse) needs real EH; match the wasm EH model used elsewhere (manifold).
    target_compile_options(adacpp_glb_diff PRIVATE -fwasm-exceptions)
    set_target_properties(adacpp_glb_diff PROPERTIES OUTPUT_NAME "adacpp_glb_diff" SUFFIX ".js")
    target_link_options(adacpp_glb_diff PRIVATE
            "-lembind"
            "-fwasm-exceptions"
            "-sALLOW_MEMORY_GROWTH=1"
            "-sMAXIMUM_MEMORY=4294967296"
            "-sMODULARIZE=1"
            "-sEXPORT_ES6=1"
            "-sEXPORT_NAME=createAdacppGlbDiff"
            "-sENVIRONMENT=web,worker,node"
            "--emit-tsd" "adacpp_glb_diff.d.ts"
            "-sSTACK_SIZE=1048576")
    return() # standalone target
endif ()

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
            "--emit-tsd" "adacpp_step_glb.d.ts"
            "-sSTACK_SIZE=1048576")
    return() # standalone target; skip the legacy WASM_UTILS stub below
endif ()

# The OCC-free native IFC->GLB pipeline as a STANDALONE embind wasm module — no OCCT, no pyodide, no
# ifcopenshell, no nanobind, no Python. The IFC counterpart of adacpp_step_glb: same neutral-geometry
# tessellation + GLB stack (ngeom + libtess2 + meshopt + manifold), IfcResolver front-end. Streams
# the IFC from OPFS via WASMFS so large files never have to fit the wasm heap.
if (BUILD_IFC_GLB_WASM)
    add_executable(adacpp_ifc_glb
            ${CMAKE_SOURCE_DIR}/src/cad/ifc_glb_wasm.cpp
            ${CMAKE_SOURCE_DIR}/src/geom/neutral/ngeom_tessellate.cpp
            ${CMAKE_SOURCE_DIR}/src/geom/neutral/ngeom_boolean.cpp
            ${CMAKE_SOURCE_DIR}/src/geom/neutral/ngeom_meshopt.cpp
            ${LIBTESS2_SOURCES}
            ${MESHOPT_SOURCES})
    target_link_libraries(adacpp_ifc_glb PRIVATE manifold)
    set_target_properties(adacpp_ifc_glb PROPERTIES OUTPUT_NAME "adacpp_ifc_glb" SUFFIX ".js")
    target_link_options(adacpp_ifc_glb PRIVATE
            "-lembind"
            "-sWASMFS=1"              # WASMFS + OPFS backend (file-backed pread, not heap)
            "-sFORCE_FILESYSTEM=1"
            "-sEXPORTED_RUNTIME_METHODS=['FS']" # JS-side FS for OPFS mount + node smoke test
            "-sALLOW_MEMORY_GROWTH=1"
            "-sMODULARIZE=1"
            "-sEXPORT_ES6=1"
            "-sEXPORT_NAME=createAdacppIfcGlb"
            "-sENVIRONMENT=web,worker,node"
            "--emit-tsd" "adacpp_ifc_glb.d.ts"
            "-sSTACK_SIZE=1048576")
    return() # standalone target; skip the legacy WASM_UTILS stub below
endif ()

# The OCC-free native B-rep WRITER (STEP→IFC + IFC→STEP) as a STANDALONE embind wasm module — no
# OCCT, no pyodide, no ifcopenshell, no nanobind, no Python. No tessellation (writers operate on the
# analytic NgeomRoot), so unlike the →GLB modules it needs NO libtess2 / meshopt / manifold. Shares one
# implementation with the nanobind module via brep_file_convert.h. Streams source from OPFS via WASMFS.
if (BUILD_BREP_WRITER_WASM)
    add_executable(adacpp_brep_writer
            ${CMAKE_SOURCE_DIR}/src/cad/brep_writer_wasm.cpp)
    set_target_properties(adacpp_brep_writer PROPERTIES OUTPUT_NAME "adacpp_brep_writer" SUFFIX ".js")
    target_link_options(adacpp_brep_writer PRIVATE
            "-lembind"
            "-sWASMFS=1"              # WASMFS + OPFS backend (file-backed pread, not heap)
            "-sFORCE_FILESYSTEM=1"
            "-sEXPORTED_RUNTIME_METHODS=['FS']" # JS-side FS for OPFS mount + node smoke test
            "-sALLOW_MEMORY_GROWTH=1"
            "-sMODULARIZE=1"
            "-sEXPORT_ES6=1"
            "-sEXPORT_NAME=createAdacppBrepWriter"
            "-sENVIRONMENT=web,worker,node"
            "--emit-tsd" "adacpp_brep_writer.d.ts"
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