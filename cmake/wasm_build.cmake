# File I/O for the single-threaded embind modules: WASMFS (in-heap by default, which is what node
# runs) plus an OPFS mount whose files are FileSystemSyncAccessHandles (src/wasmio). emscripten's own
# OPFS backend needs -pthread (SharedArrayBuffer, so a cross-origin-isolated page) or JSPI; without
# either its mount reports success and every file operation on it traps. This backend does its I/O
# synchronously from the worker instead. The JS API (opfsMount / opfsOpen / ...) is exported next to
# FS, so it also lands in the generated .d.ts.
function(adacpp_wasm_fs target)
    target_sources(
        ${target}
        PRIVATE ${CMAKE_SOURCE_DIR}/src/wasmio/opfs_sync_backend.cpp
    )
    # WASMFS's backend classes are internal headers of the emscripten tree (pinned: 4.0.9).
    target_include_directories(
        ${target}
        PRIVATE ${EMSCRIPTEN_ROOT_PATH}/system/lib/wasmfs
    )
    set(_js ${CMAKE_SOURCE_DIR}/src/wasmio/opfs_sync.js)
    target_link_options(
        ${target}
        PRIVATE
            "-sWASMFS=1"
            "-sFORCE_FILESYSTEM=1"
            "--js-library=${_js}"
            "-sEXPORTED_RUNTIME_METHODS=['FS','opfsMount','opfsOpen','opfsDetach','opfsReserve','opfsSettle','mountOpfs']"
            # WASMFS's FS.writeFile appends to an existing file; this one replaces it.
            "--post-js=${CMAKE_SOURCE_DIR}/src/wasmio/fs_writefile.js"
    )
    set_property(
        TARGET ${target}
        APPEND
        PROPERTY
            LINK_DEPENDS ${_js} ${CMAKE_SOURCE_DIR}/src/wasmio/fs_writefile.js
    )
endfunction()

# The format-neutral FEA kernels (load-combination superposition, derived components, envelopes,
# AFBL/AFEL writer) as a STANDALONE embind wasm module. No OCCT, no tessellator, no manifold -- just
# src/fea, which the nanobind module compiles too, so browser and server write the same bytes.
#
# The numerics flags are part of the contract, not tuning: -ffp-contract=off forbids fusing a*b+c into
# one FMA (a different rounding), and there is deliberately no -ffast-math and no -mrelaxed-simd, so
# every op is a single IEEE-754 op and the results are bit-identical to the native build. -msimd128 is
# safe on that score: f32x4 mul/add round exactly like their scalar forms. The optimisation level is
# not set here: it comes from the Release flags in cmake/wasm_toolchain.cmake, like every other module.
if(BUILD_FEA_WASM)
    add_executable(
        adacpp_fea
        ${CMAKE_SOURCE_DIR}/src/fea/fea_wasm.cpp
        ${CMAKE_SOURCE_DIR}/src/fea/fea_arrays.cpp
        ${CMAKE_SOURCE_DIR}/src/fea/superpose.cpp
        ${CMAKE_SOURCE_DIR}/src/fea/derive.cpp
        ${CMAKE_SOURCE_DIR}/src/fea/artefact_io.cpp
        ${CMAKE_SOURCE_DIR}/src/fea/envelope.cpp
        ${CMAKE_SOURCE_DIR}/src/fea/field_ops.cpp
    )
    # nlohmann json.hpp (the JSON arguments) lives in the (wasm) conda env include.
    target_include_directories(adacpp_fea PRIVATE $ENV{CONDA_PREFIX}/include)
    # try/catch (JSON parse + kernel errors -> {"ok":false}) needs real EH; same model as glb_diff.
    target_compile_options(
        adacpp_fea
        PRIVATE -msimd128 -ffp-contract=off -fwasm-exceptions
    )
    set_target_properties(
        adacpp_fea
        PROPERTIES OUTPUT_NAME "adacpp_fea" SUFFIX ".js"
    )
    target_link_options(
        adacpp_fea
        PRIVATE
            "-lembind"
            "-fwasm-exceptions"
            "-sALLOW_MEMORY_GROWTH=1"
            "-sMAXIMUM_MEMORY=4294967296"
            "-sMODULARIZE=1"
            "-sEXPORT_ES6=1"
            "-sEXPORT_NAME=createAdacppFea"
            "-sENVIRONMENT=web,worker,node"
            "--emit-tsd"
            "adacpp_fea.d.ts"
            "-sSTACK_SIZE=1048576"
    )
    adacpp_wasm_fs(adacpp_fea)
    return() # standalone target
endif()

# Prismatic extrusion expansion as a STANDALONE embind wasm module. The leanest of the set: the
# expander in ngeom_extrude.h is header-only, so this links no libtess2, no meshoptimizer, no
# manifold and no OCCT — just the arithmetic that turns a section table plus per-instance frames
# into vertex and index buffers. Section tables are prepared by whatever wrote the artefact; the
# browser only replays them.
if(BUILD_EXTRUDE_WASM)
    add_executable(
        adacpp_extrude
        ${CMAKE_SOURCE_DIR}/src/geom/neutral/extrude_wasm.cpp
    )
    set_target_properties(
        adacpp_extrude
        PROPERTIES OUTPUT_NAME "adacpp_extrude" SUFFIX ".js"
    )
    target_link_options(
        adacpp_extrude
        PRIVATE
            "-lembind"
            "-sALLOW_MEMORY_GROWTH=1"
            "-sMAXIMUM_MEMORY=4294967296"
            "-sMODULARIZE=1"
            "-sEXPORT_ES6=1"
            "-sEXPORT_NAME=createAdacppExtrude"
            "-sENVIRONMENT=web,worker,node"
            "--emit-tsd"
            "adacpp_extrude.d.ts"
            "-sSTACK_SIZE=1048576"
    )
    return() # standalone target
endif()

# The OCC-free GLB diff (summarise + match + removed overlay) as a STANDALONE embind wasm module —
# no OCCT, no pyodide, no tinygltf. Reuses the portable diff core (glb_diff_native.h's
# summarize_glb_buf, RAM-decode path) + meshoptimizer + nlohmann json. Takes two GLB buffers, returns
# the viewer-ops + a red overlay GLB, entirely in-browser (no worker job / server memory).
if(BUILD_GLB_DIFF_WASM)
    add_executable(
        adacpp_glb_diff
        ${CMAKE_SOURCE_DIR}/src/cad/glb_diff_wasm.cpp
        ${CMAKE_SOURCE_DIR}/src/geom/neutral/ngeom_meshopt.cpp
        ${MESHOPT_SOURCES}
    )
    # nlohmann json.hpp lives in the (wasm) conda env include — emscripten doesn't search it by default.
    target_include_directories(
        adacpp_glb_diff
        PRIVATE $ENV{CONDA_PREFIX}/include
    )
    # try/catch (nlohmann parse) needs real EH; match the wasm EH model used elsewhere (manifold).
    target_compile_options(adacpp_glb_diff PRIVATE -fwasm-exceptions)
    set_target_properties(
        adacpp_glb_diff
        PROPERTIES OUTPUT_NAME "adacpp_glb_diff" SUFFIX ".js"
    )
    target_link_options(
        adacpp_glb_diff
        PRIVATE
            "-lembind"
            "-fwasm-exceptions"
            "-sALLOW_MEMORY_GROWTH=1"
            "-sMAXIMUM_MEMORY=4294967296"
            "-sMODULARIZE=1"
            "-sEXPORT_ES6=1"
            "-sEXPORT_NAME=createAdacppGlbDiff"
            "-sENVIRONMENT=web,worker,node"
            "--emit-tsd"
            "adacpp_glb_diff.d.ts"
            "-sSTACK_SIZE=1048576"
    )
    return() # standalone target
endif()

# The OCC-free native STEP->GLB pipeline as a STANDALONE embind wasm module — no OCCT, no pyodide,
# no nanobind, no Python. Single-threaded (no -pthread, so no SharedArrayBuffer / COOP-COEP needed);
# streams the STEP from OPFS via WASMFS so multi-GB files never have to fit in the wasm heap.
if(BUILD_STEP_GLB_WASM)
    add_executable(
        adacpp_step_glb
        ${CMAKE_SOURCE_DIR}/src/cad/cad_wasm.cpp
        ${CMAKE_SOURCE_DIR}/src/geom/neutral/ngeom_tessellate.cpp
        ${CMAKE_SOURCE_DIR}/src/geom/neutral/ngeom_boolean.cpp
        ${CMAKE_SOURCE_DIR}/src/geom/neutral/ngeom_meshopt.cpp
        ${LIBTESS2_SOURCES}
        ${MESHOPT_SOURCES}
    )
    target_link_libraries(adacpp_step_glb PRIVATE manifold)
    set_target_properties(
        adacpp_step_glb
        PROPERTIES OUTPUT_NAME "adacpp_step_glb" SUFFIX ".js"
    )
    target_link_options(
        adacpp_step_glb
        PRIVATE
            "-lembind"
            "-sALLOW_MEMORY_GROWTH=1"
            "-sMODULARIZE=1"
            "-sEXPORT_ES6=1"
            "-sEXPORT_NAME=createAdacppStepGlb"
            "-sENVIRONMENT=web,worker,node"
            "--emit-tsd"
            "adacpp_step_glb.d.ts"
            "-sSTACK_SIZE=1048576"
    )
    adacpp_wasm_fs(adacpp_step_glb)
    return() # standalone target; skip the legacy WASM_UTILS stub below
endif()

# The OCC-free native IFC->GLB pipeline as a STANDALONE embind wasm module — no OCCT, no pyodide, no
# ifcopenshell, no nanobind, no Python. The IFC counterpart of adacpp_step_glb: same neutral-geometry
# tessellation + GLB stack (ngeom + libtess2 + meshopt + manifold), IfcResolver front-end. Streams
# the IFC from OPFS via WASMFS so large files never have to fit the wasm heap.
if(BUILD_IFC_GLB_WASM)
    add_executable(
        adacpp_ifc_glb
        ${CMAKE_SOURCE_DIR}/src/cad/ifc_glb_wasm.cpp
        ${CMAKE_SOURCE_DIR}/src/geom/neutral/ngeom_tessellate.cpp
        ${CMAKE_SOURCE_DIR}/src/geom/neutral/ngeom_boolean.cpp
        ${CMAKE_SOURCE_DIR}/src/geom/neutral/ngeom_meshopt.cpp
        ${LIBTESS2_SOURCES}
        ${MESHOPT_SOURCES}
    )
    target_link_libraries(adacpp_ifc_glb PRIVATE manifold)
    set_target_properties(
        adacpp_ifc_glb
        PROPERTIES OUTPUT_NAME "adacpp_ifc_glb" SUFFIX ".js"
    )
    target_link_options(
        adacpp_ifc_glb
        PRIVATE
            "-lembind"
            "-sALLOW_MEMORY_GROWTH=1"
            "-sMODULARIZE=1"
            "-sEXPORT_ES6=1"
            "-sEXPORT_NAME=createAdacppIfcGlb"
            "-sENVIRONMENT=web,worker,node"
            "--emit-tsd"
            "adacpp_ifc_glb.d.ts"
            "-sSTACK_SIZE=1048576"
    )
    adacpp_wasm_fs(adacpp_ifc_glb)
    return() # standalone target; skip the legacy WASM_UTILS stub below
endif()

# The OCC-free native B-rep WRITER (STEP→IFC + IFC→STEP) as a STANDALONE embind wasm module — no
# OCCT, no pyodide, no ifcopenshell, no nanobind, no Python. No tessellation (writers operate on the
# analytic NgeomRoot), so unlike the →GLB modules it needs NO libtess2 / meshopt / manifold. Shares one
# implementation with the nanobind module via brep_file_convert.h. Streams source from OPFS via WASMFS.
if(BUILD_BREP_WRITER_WASM)
    add_executable(
        adacpp_brep_writer
        ${CMAKE_SOURCE_DIR}/src/cad/brep_writer_wasm.cpp
    )
    set_target_properties(
        adacpp_brep_writer
        PROPERTIES OUTPUT_NAME "adacpp_brep_writer" SUFFIX ".js"
    )
    target_link_options(
        adacpp_brep_writer
        PRIVATE
            "-lembind"
            "-sALLOW_MEMORY_GROWTH=1"
            "-sMODULARIZE=1"
            "-sEXPORT_ES6=1"
            "-sEXPORT_NAME=createAdacppBrepWriter"
            "-sENVIRONMENT=web,worker,node"
            "--emit-tsd"
            "adacpp_brep_writer.d.ts"
            "-sSTACK_SIZE=1048576"
    )
    adacpp_wasm_fs(adacpp_brep_writer)
    return() # standalone target; skip the legacy WASM_UTILS stub below
endif()

set(WASM_SOURCES src/wasm_utils.cpp)
set(WASM_HEADERS src/wasm_utils.h)

add_library(WASM_UTILS ${WASM_SOURCES} ${WASM_HEADERS})
# Custom output for WebAssembly
set_target_properties(
    WASM_UTILS
    PROPERTIES OUTPUT_NAME "adacpp_utils" SUFFIX ".wasm"
)

# Export the `multiply` function for use in JS
# Properly escape EXPORTED_FUNCTIONS flag
target_link_options(
    WASM_UTILS
    PRIVATE
        "-sEXPORTED_FUNCTIONS=[\"_multiply\"]"
        "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap']"
)
