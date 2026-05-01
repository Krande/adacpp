# cmake/wasm_occt.cmake
#
# Cross-compile OpenCASCADE Technology to WebAssembly via emscripten and expose
# its module targets (TKernel, TKMath, ...) for static linking into the adacpp
# nanobind module. Pulled in only when BUILD_WASM=ON; native builds use the
# conda-forge-provided OCCT via cmake/deps_occ.cmake.
#
# This is M1 of OCCT-wasm bringup — first sub-stage builds *only* the
# FoundationClasses module (TKernel, TKMath) as a proof-of-life. Subsequent
# stages enable ModelingData / ModelingAlgorithms / DataExchange / etc.
#
# We use ExternalProject_Add (not FetchContent_MakeAvailable / add_subdirectory)
# because OCCT's CMakeLists.txt resolves its helper files against
# ${CMAKE_SOURCE_DIR}, not its own source dir — when add_subdirectory'd into
# a parent project, CMAKE_SOURCE_DIR points at the parent and OCCT can't
# find adm/cmake/vardescr.cmake / occt_macros.cmake. Running OCCT as its own
# top-level project via ExternalProject sidesteps the bug entirely.

include(ExternalProject)
include(FetchContent)

set(OCCT_GIT_TAG       "V7_9_0" CACHE STRING "OCCT git tag to fetch for wasm build")
set(OCCT_INSTALL_DIR   "${CMAKE_BINARY_DIR}/_deps/occt-install")
set(OCCT_SOURCE_DIR    "${CMAKE_BINARY_DIR}/_deps/occt-src")
set(OCCT_BUILD_DIR     "${CMAKE_BINARY_DIR}/_deps/occt-build")

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

# OCCT install layout (on Linux/wasm hosts): include/opencascade/*.hxx, lib/lib*.a
set(OCCT_INCLUDE_DIR   "${OCCT_INSTALL_DIR}/include/opencascade")
set(OCCT_LIB_DIR       "${OCCT_INSTALL_DIR}/lib")

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

    # OCCT throws Standard_Failure & friends extensively. Match adacpp's wasm
    # build flag (commit 0566168 "wasm exceptions") so OCCT's catches actually
    # fire. -fexceptions is the JS-trampoline model pyodide 0.27.x ships;
    # switch to -fwasm-exceptions when we move to pyodide 0.28+.
    -DCMAKE_CXX_FLAGS=-fexceptions
    -DCMAKE_C_FLAGS=-fexceptions
)

# OCCT toolkit list — order matters when statically linking. Listed roughly
# by dependency layer (callers first, dependencies last) so `--start-group`
# isn't strictly required, but emscripten/wasm-ld is forgiving here.
# Any toolkit added here must (a) actually be built by OCCT given the
# BUILD_MODULE_* flags above, and (b) be wanted by the wasm-side bindings —
# linking unused archives bloats the wheel.
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

# IMPORTED targets: present them as if they were a normal library so callers
# can `target_link_libraries(_ada_cpp_ext_impl PRIVATE TKBRep TKMath ...)`.
# The add_dependencies hook ensures occt_external is built before anything
# that links against these archives.
set(WASM_OCCT_TARGETS)
foreach(_tk IN LISTS _WASM_OCCT_TOOLKITS)
    if (NOT TARGET ${_tk})
        add_library(${_tk} STATIC IMPORTED GLOBAL)
        set_target_properties(${_tk} PROPERTIES
            IMPORTED_LOCATION             ${OCCT_LIB_DIR}/lib${_tk}.a
            INTERFACE_INCLUDE_DIRECTORIES ${OCCT_INCLUDE_DIR}
        )
        add_dependencies(${_tk} occt_external)
        list(APPEND WASM_OCCT_TARGETS ${_tk})
    endif ()
endforeach()

message(STATUS "OCCT-wasm targets staged (built at compile time): ${WASM_OCCT_TARGETS}")
message(STATUS "OCCT-wasm install dir: ${OCCT_INSTALL_DIR}")
