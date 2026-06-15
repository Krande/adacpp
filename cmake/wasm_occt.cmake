# cmake/wasm_occt.cmake
#
# Cross-compile OpenCASCADE Technology to WebAssembly via emscripten and expose
# its module targets (TKernel, TKMath, ...) for static linking into the adacpp
# nanobind module. Pulled in only when BUILD_WASM=ON; native builds use the
# conda-forge-provided OCCT via cmake/deps_occ.cmake.
#
# The OCCT install (lib/lib*.a + include/opencascade) is an expensive (~30-min)
# but stable artifact — it depends only on OCCT_GIT_TAG, the toolkit list, the
# rapidjson pin and the emscripten/pyodide ABI, none of which change with adacpp
# source. So it is published as a reusable base image and consumed via
# WASM_OCCT_PREBUILT_DIR: when that is set (env var or -D), this file SKIPS the
# ExternalProject build entirely and just points the IMPORTED targets at the
# prebuilt tree. Unset → build OCCT from source as before.
#
# We use ExternalProject_Add (not FetchContent_MakeAvailable / add_subdirectory)
# because OCCT's CMakeLists.txt resolves its helper files against
# ${CMAKE_SOURCE_DIR}, not its own source dir — when add_subdirectory'd into
# a parent project, CMAKE_SOURCE_DIR points at the parent and OCCT can't
# find adm/cmake/vardescr.cmake / occt_macros.cmake. Running OCCT as its own
# top-level project via ExternalProject sidesteps the bug entirely.

include(ExternalProject)
include(FetchContent)

# Reusable prebuilt OCCT-wasm install (the ghcr base image, docker-cp'd onto the
# runner). Default from the environment so the pixi-driven CI build can opt in
# without touching CMakePresets.
set(WASM_OCCT_PREBUILT_DIR "$ENV{WASM_OCCT_PREBUILT_DIR}"
    CACHE PATH "Prebuilt OCCT-wasm install dir; skip the OCCT ExternalProject build when set")

set(OCCT_GIT_TAG "V7_9_0" CACHE STRING "OCCT git tag to fetch for wasm build")

if (WASM_OCCT_PREBUILT_DIR)
    set(OCCT_INSTALL_DIR "${WASM_OCCT_PREBUILT_DIR}")
    message(STATUS "Using prebuilt OCCT-wasm install: ${OCCT_INSTALL_DIR}")
else ()
    set(OCCT_INSTALL_DIR "${CMAKE_BINARY_DIR}/_deps/occt-install")
    set(OCCT_SOURCE_DIR  "${CMAKE_BINARY_DIR}/_deps/occt-src")
    set(OCCT_BUILD_DIR   "${CMAKE_BINARY_DIR}/_deps/occt-build")
endif ()

# OCCT install layout (on Linux/wasm hosts): include/opencascade/*.hxx, lib/lib*.a
set(OCCT_INCLUDE_DIR "${OCCT_INSTALL_DIR}/include/opencascade")
set(OCCT_LIB_DIR     "${OCCT_INSTALL_DIR}/lib")

# OCCT toolkit list — order matters when statically linking. Listed roughly
# by dependency layer (callers first, dependencies last) so `--start-group`
# isn't strictly required, but emscripten/wasm-ld is forgiving here.
# Any toolkit added here must (a) actually be built by OCCT given the
# BUILD_MODULE_* flags below, and (b) be wanted by the wasm-side bindings —
# linking unused archives bloats the wheel. Shared by the build-from-source and
# prebuilt paths (the prebuilt install must carry exactly these archives).
set(_WASM_OCCT_TOOLKITS
    # DataExchange layer — STEP/glTF readers/writers
    TKDEGLTF      # RWGltf_CafWriter / Reader
    TKDESTEP      # STEPControl_Reader / Writer + STEPCAFControl_*
    TKRWMesh      # RWMesh_* base classes used by TKDEGLTF + others
    TKXSBase      # XS framework shared by STEP/IGES/...
    # ApplicationFramework layer — TDocStd_Document used by RWGltf_CafWriter
    TKBinXCAF     # binary persistence for XCAF docs
    TKXCAF        # eXtended CAF (colors, layers, names on shapes)
    TKVCAF        # visualization CAF
    TKBin         # binary persistence base
    TKBinL        # binary lite persistence
    TKCAF         # CAF main
    TKLCAF        # CAF light (TDocStd_Document, TDF_Label, TDataStd_Name)
    TKCDF         # Component Data Framework — CDM_Document, base of TDocStd
    # ModelingAlgorithms layer — high-level builders / mesh / boolean
    TKMesh
    TKShHealing
    TKBool
    TKBO
    TKOffset
    TKFillet
    TKHLR
    TKPrim
    TKTopAlgo
    TKGeomAlgo
    # ModelingData layer — TopoDS shapes and parametric geometry
    TKBRep
    TKGeomBase
    TKG3d
    TKG2d
    # FoundationClasses layer — must come last in static-link order
    TKMath
    TKernel
)

if (NOT WASM_OCCT_PREBUILT_DIR)
    # ---- Build OCCT from source (the ~30-min cross-compile) ----------------

    # RapidJSON is required by OCCT's TKDEGLTF (RWGltf_CafWriter / Reader). It's
    # header-only, so we just need the include dir on disk before OCCT's nested
    # CMake configure runs. Fetched at configure-time (small repo, fast clone).
    FetchContent_Declare(
        rapidjson
        GIT_REPOSITORY https://github.com/Tencent/rapidjson.git
        # NOT v1.1.0 — that 2016 release has rapidjson/document.h:319 doing
        # `length = rhs.length` against a `const SizeType length` member, which
        # newer clang (incl. emscripten 3.1.58's clang 19) rejects outright.
        # Pinned to the post-fix master snapshot OCCT's own CI uses.
        GIT_TAG        24b5e7a8b27f42fa16b96fc70aade9106cf7102f
        GIT_SHALLOW    TRUE
    )
    FetchContent_GetProperties(rapidjson)
    if (NOT rapidjson_POPULATED)
        FetchContent_Populate(rapidjson)
    endif ()
    set(RAPIDJSON_INCLUDE_DIR "${rapidjson_SOURCE_DIR}/include")
    message(STATUS "RapidJSON include dir for OCCT-wasm: ${RAPIDJSON_INCLUDE_DIR}")

    # Forward emscripten + build settings into OCCT's nested CMake configure.
    # CMAKE_TOOLCHAIN_FILE comes from emcmake; we pass it through explicitly so
    # the OCCT sub-build uses the same Emscripten.cmake (otherwise it'd try to
    # build with the host compiler).
    set(_OCCT_CMAKE_ARGS
        -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
        -DCMAKE_INSTALL_PREFIX=${OCCT_INSTALL_DIR}
        -DCMAKE_BUILD_TYPE=Release

        # Static archives — linked into the .so wheel module.
        -DBUILD_LIBRARY_TYPE=Static
        -DBUILD_USE_PCH=OFF
        -DBUILD_DOC_Overview=OFF
        -DBUILD_RESOURCES=OFF
        -DBUILD_SAMPLES_MFC=OFF
        -DBUILD_SAMPLES_QT=OFF
        -DBUILD_Inspector=OFF
        -DBUILD_RELEASE_DISABLE_EXCEPTIONS=OFF
        -DBUILD_SOVERSION_NUMBERS=0

        # M3 sub-stage: full read/write pipeline.
        # FoundationClasses     → TKernel, TKMath
        # ModelingData          → TKG2d, TKG3d, TKGeomBase, TKBRep
        # ModelingAlgorithms    → TKGeomAlgo, TKTopAlgo, TKPrim, TKBO, TKBool,
        #                         TKHLR, TKFillet, TKOffset, TKShHealing, TKMesh
        # DataExchange          → TKDESTEP (STEP r/w), TKDEGLTF (glTF r/w),
        #                         TKXSBase, TKDEIGES, TKDESTL, TKDEPLY, ...
        # ApplicationFramework  → TKLCAF, TKCAF, TKVCAF, TKBin*, TKXCAF — needed
        #                         by RWGltf_CafWriter (TDocStd_Document).
        -DBUILD_MODULE_FoundationClasses=ON
        -DBUILD_MODULE_ModelingData=ON
        -DBUILD_MODULE_ModelingAlgorithms=ON
        -DBUILD_MODULE_DataExchange=ON
        -DBUILD_MODULE_ApplicationFramework=ON
        -DBUILD_MODULE_Visualization=OFF
        -DBUILD_MODULE_Draw=OFF

        # Third-party deps off. RapidJSON comes back at M3 (glTF write), the rest
        # likely never come back — we'll never need VTK/TBB/freetype/draco for
        # adacpp's headless conversion path.
        -DUSE_FREETYPE=OFF
        -DUSE_FFMPEG=OFF
        -DUSE_FREEIMAGE=OFF
        -DUSE_GLES2=OFF
        -DUSE_OPENVR=OFF
        # RapidJSON ON for glTF writer; pointed at FetchContent'd source above.
        -DUSE_RAPIDJSON=ON
        -D3RDPARTY_RAPIDJSON_DIR=${rapidjson_SOURCE_DIR}
        -D3RDPARTY_RAPIDJSON_INCLUDE_DIR=${RAPIDJSON_INCLUDE_DIR}
        -DUSE_DRACO=OFF
        -DUSE_TBB=OFF
        -DUSE_VTK=OFF
        -DUSE_TK=OFF
        -DUSE_XLIB=OFF

        # OCCT throws Standard_Failure & friends extensively, and uses
        # setjmp/longjmp for OSD signal handling. Pyodide 0.28+/emscripten 4.0.9
        # use native WebAssembly exception handling: -fwasm-exceptions (replaces
        # the -fexceptions JS-trampoline model that fatally trapped STEP write)
        # plus -sSUPPORT_LONGJMP=wasm so setjmp/longjmp also use wasm EH. Must
        # match adacpp's own flag (one EH model across the whole link).
        -DCMAKE_CXX_FLAGS=-fwasm-exceptions\ -sSUPPORT_LONGJMP=wasm
        -DCMAKE_C_FLAGS=-fwasm-exceptions\ -sSUPPORT_LONGJMP=wasm
    )

    set(_WASM_OCCT_BYPRODUCTS)
    foreach (_tk IN LISTS _WASM_OCCT_TOOLKITS)
        list(APPEND _WASM_OCCT_BYPRODUCTS ${OCCT_LIB_DIR}/lib${_tk}.a)
    endforeach()

    ExternalProject_Add(
        occt_external
        GIT_REPOSITORY  https://github.com/Open-Cascade-SAS/OCCT.git
        GIT_TAG         ${OCCT_GIT_TAG}
        GIT_SHALLOW     TRUE
        GIT_PROGRESS    TRUE
        SOURCE_DIR      ${OCCT_SOURCE_DIR}
        BINARY_DIR      ${OCCT_BUILD_DIR}
        INSTALL_DIR     ${OCCT_INSTALL_DIR}
        # Drop OCCT's OCC_CONVERT_SIGNALS define on wasm. It converts OS signals
        # to C++ exceptions via setjmp INSIDE catch blocks — meaningless under
        # wasm (no real signals) and it violates the wasm-EH + wasm-SjLj rule
        # ("no setjmp within a catch"), which makes emscripten emit invalid wasm
        # (CompileError: "br_table: label arity inconsistent") that fails to
        # instantiate. Removing it lets the STEP write path compile to valid wasm.
        PATCH_COMMAND   sed -i "/add_definitions(-DOCC_CONVERT_SIGNALS)/d" ${OCCT_SOURCE_DIR}/adm/cmake/occt_defs_flags.cmake
        CMAKE_ARGS      ${_OCCT_CMAKE_ARGS}
        BUILD_BYPRODUCTS ${_WASM_OCCT_BYPRODUCTS}
        USES_TERMINAL_DOWNLOAD  TRUE
        USES_TERMINAL_CONFIGURE TRUE
        USES_TERMINAL_BUILD     TRUE
        USES_TERMINAL_INSTALL   TRUE
    )

    # Make sure the install include dir exists at configure time, even before
    # the build runs — otherwise IMPORTED targets that reference it via
    # INTERFACE_INCLUDE_DIRECTORIES emit a configure-time warning. CMake checks
    # include dirs at configure, archives only at link.
    file(MAKE_DIRECTORY ${OCCT_INCLUDE_DIR})
endif ()

# IMPORTED targets: present them as if they were a normal library so callers
# can `target_link_libraries(_ada_cpp_ext_impl PRIVATE TKBRep TKMath ...)`.
# When building from source, the add_dependencies hook ensures occt_external is
# built before anything that links against these archives; with a prebuilt
# install the archives already exist, so no dependency is added.
set(WASM_OCCT_TARGETS)
foreach(_tk IN LISTS _WASM_OCCT_TOOLKITS)
    if (NOT TARGET ${_tk})
        add_library(${_tk} STATIC IMPORTED GLOBAL)
        set_target_properties(${_tk} PROPERTIES
            IMPORTED_LOCATION             ${OCCT_LIB_DIR}/lib${_tk}.a
            INTERFACE_INCLUDE_DIRECTORIES ${OCCT_INCLUDE_DIR}
        )
        if (NOT WASM_OCCT_PREBUILT_DIR)
            add_dependencies(${_tk} occt_external)
        endif ()
        list(APPEND WASM_OCCT_TARGETS ${_tk})
    endif ()
endforeach()

if (WASM_OCCT_PREBUILT_DIR)
    message(STATUS "OCCT-wasm targets from prebuilt install: ${WASM_OCCT_TARGETS}")
else ()
    message(STATUS "OCCT-wasm targets staged (built at compile time): ${WASM_OCCT_TARGETS}")
endif ()
message(STATUS "OCCT-wasm install dir: ${OCCT_INSTALL_DIR}")
