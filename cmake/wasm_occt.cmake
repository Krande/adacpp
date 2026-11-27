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
set(WASM_OCCT_PREBUILT_DIR
    "$ENV{WASM_OCCT_PREBUILT_DIR}"
    CACHE PATH
    "Prebuilt OCCT-wasm install dir; skip the OCCT ExternalProject build when set"
)

# V8_0_0, matching the conda-forge occt the NATIVE build pins in pixi.toml. The
# two are independent builds — native takes occt from conda-forge, wasm
# cross-compiles it from source here — so this tag has to be moved by hand and
# was missed when the native side went to 8.0.0. V8_0_0 exactly, not V8_0_1:
# ifcopenshell 0.8.5 constrains `occt >=8.0.0,<8.0.1.0a0` natively, and there is
# no reason for the wasm OCCT to drift off the version the rest of the project
# is built and tested against.
#
# Changing this invalidates the ghcr base image: publish-occt-wasm-base.yaml
# derives its tag by sed-ing this very line, so the next publish produces
# V8_0_0-emsdk-<ver> and consumers stop matching the old V7_9_0 image.
set(OCCT_GIT_TAG "V8_0_0" CACHE STRING "OCCT git tag to fetch for wasm build")

# SPIKE (option B, Aibel-Engineering/adapy-plugins#177 step 2d): OCCT as ONE shared Pyodide side
# module (libocct.so, its own wheel) that adacpp's extension and gmsh's libgmsh.so both link against,
# so the process holds exactly one OCCT and a TopoDS_Shape* can cross from adacpp to gmsh as it does
# natively (conda: libTK*.so shared by ada-cpp and libgmsh). OFF = today's static, hidden OCCT
# baked into _ada_cpp_ext_impl.so.
#
# ON changes the OCCT build itself, so it needs its own install tree: the static archives are
# compiled with DEFAULT visibility (a shared library must export them) and -fPIC, and the prebuilt
# ghcr base image (hidden visibility) cannot be reused.
option(
    ADACPP_WASM_SHARED_OCCT
    "SPIKE: build OCCT as a shared side module (libocct.so) instead of linking it statically"
    OFF
)
if(ADACPP_WASM_SHARED_OCCT AND WASM_OCCT_PREBUILT_DIR)
    message(
        STATUS
        "ADACPP_WASM_SHARED_OCCT: ignoring WASM_OCCT_PREBUILT_DIR (hidden-visibility archives)"
    )
    set(WASM_OCCT_PREBUILT_DIR "" CACHE PATH "" FORCE)
endif()

if(WASM_OCCT_PREBUILT_DIR)
    set(OCCT_INSTALL_DIR "${WASM_OCCT_PREBUILT_DIR}")
    message(STATUS "Using prebuilt OCCT-wasm install: ${OCCT_INSTALL_DIR}")
else()
    set(OCCT_INSTALL_DIR "${CMAKE_BINARY_DIR}/_deps/occt-install")
    set(OCCT_SOURCE_DIR "${CMAKE_BINARY_DIR}/_deps/occt-src")
    set(OCCT_BUILD_DIR "${CMAKE_BINARY_DIR}/_deps/occt-build")
endif()

# OCCT install layout (on Linux/wasm hosts): include/opencascade/*.hxx, lib/lib*.a
set(OCCT_INCLUDE_DIR "${OCCT_INSTALL_DIR}/include/opencascade")
set(OCCT_LIB_DIR "${OCCT_INSTALL_DIR}/lib")

# OCCT toolkit list — order matters when statically linking. Listed roughly
# by dependency layer (callers first, dependencies last) so `--start-group`
# isn't strictly required, but emscripten/wasm-ld is forgiving here.
# Any toolkit added here must (a) actually be built by OCCT given the
# BUILD_MODULE_* flags below, and (b) be wanted by the wasm-side bindings —
# linking unused archives bloats the wheel. Shared by the build-from-source and
# prebuilt paths (the prebuilt install must carry exactly these archives).
set(_WASM_OCCT_TOOLKITS
    # DataExchange layer — STEP/glTF readers/writers
    TKDEGLTF # RWGltf_CafWriter / Reader
    TKDESTEP # STEPControl_Reader / Writer + STEPCAFControl_*
    TKRWMesh # RWMesh_* base classes used by TKDEGLTF + others
    TKXSBase # XS framework shared by STEP/IGES/...
    # ApplicationFramework layer — TDocStd_Document used by RWGltf_CafWriter
    TKBinXCAF # binary persistence for XCAF docs
    TKXCAF # eXtended CAF (colors, layers, names on shapes)
    TKVCAF # visualization CAF
    TKBin # binary persistence base
    TKBinL # binary lite persistence
    TKCAF # CAF main
    TKLCAF # CAF light (TDocStd_Document, TDF_Label, TDataStd_Name)
    TKCDF # Component Data Framework — CDM_Document, base of TDocStd
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
if(ADACPP_WASM_SHARED_OCCT)
    # gmsh's OCC_LIBS_REQUIRED for OCCT >= 7.8 also names TKDEIGES (IGES import, and
    # IGESCAFControl_Reader under OCC_CAF) and TKFeat; TKDE is the DE_* provider base both
    # TKDESTEP and TKDEIGES reference. In the static build those references simply stayed unresolved
    # imports; a shared library that two modules link against has to carry them.
    #
    # TKV3d + TKService (built by OCCT 8 as dependencies of the CAF toolkits even with the
    # Visualization module off): TKXCAF/TKVCAF objects reference Graphic3d_* vtables. A side module's
    # unresolved FUNCTION becomes a trapping stub, but an unresolved DATA symbol fails the load
    # ("bad export type for '_ZTV20Graphic3d_TextureSet'"), and whole-archive pulls every object in.
    list(
        PREPEND _WASM_OCCT_TOOLKITS
        TKDEIGES
        TKDE
        TKFeat
        TKV3d
        TKService
    )
endif()

if(NOT WASM_OCCT_PREBUILT_DIR)
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
        GIT_TAG 24b5e7a8b27f42fa16b96fc70aade9106cf7102f
        GIT_SHALLOW TRUE
    )
    FetchContent_GetProperties(rapidjson)
    if(NOT rapidjson_POPULATED)
        FetchContent_Populate(rapidjson)
    endif()
    set(RAPIDJSON_INCLUDE_DIR "${rapidjson_SOURCE_DIR}/include")
    message(
        STATUS
        "RapidJSON include dir for OCCT-wasm: ${RAPIDJSON_INCLUDE_DIR}"
    )

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
        # -fvisibility=hidden: make OCCT's symbols hidden so, when statically
        # linked into adacpp's side module, wasm-ld binds adacpp's OCCT references
        # DIRECTLY (no GOT import) instead of through the interposable global
        # table. Without this, adacpp's own OCCT calls resolve against the global
        # symbol table at load and pick up the upstream ifcopenshell wheel's
        # separate static OCCT — corrupting OCCT's RTTI registry. Hidden OCCT
        # stays private to adacpp; ifcopenshell keeps its own; no collision.
    )
    if(ADACPP_WASM_SHARED_OCCT)
        # Shared: default visibility (libocct.so must export OCCT) and PIC for the side module.
        list(
            APPEND _OCCT_CMAKE_ARGS
            -DCMAKE_CXX_FLAGS=-fwasm-exceptions\ -sSUPPORT_LONGJMP=wasm\ -fPIC
            -DCMAKE_C_FLAGS=-fwasm-exceptions\ -sSUPPORT_LONGJMP=wasm\ -fPIC
        )
    else()
        list(
            APPEND _OCCT_CMAKE_ARGS
            -DCMAKE_CXX_FLAGS=-fwasm-exceptions\ -sSUPPORT_LONGJMP=wasm\ -fvisibility=hidden\ -fvisibility-inlines-hidden
            -DCMAKE_C_FLAGS=-fwasm-exceptions\ -sSUPPORT_LONGJMP=wasm\ -fvisibility=hidden
        )
    endif()

    set(_OCCT_EXTRA_PATCH)
    if(ADACPP_WASM_SHARED_OCCT)
        set(_OCCT_EXTRA_PATCH
            COMMAND
            bash
            ${CMAKE_CURRENT_LIST_DIR}/occt_wasm_shared_patch.sh
            ${OCCT_SOURCE_DIR}
        )
    endif()

    set(_WASM_OCCT_BYPRODUCTS)
    foreach(_tk IN LISTS _WASM_OCCT_TOOLKITS)
        list(APPEND _WASM_OCCT_BYPRODUCTS ${OCCT_LIB_DIR}/lib${_tk}.a)
    endforeach()

    ExternalProject_Add(
        occt_external
        GIT_REPOSITORY https://github.com/Open-Cascade-SAS/OCCT.git
        GIT_TAG ${OCCT_GIT_TAG}
        GIT_SHALLOW TRUE
        GIT_PROGRESS TRUE
        SOURCE_DIR ${OCCT_SOURCE_DIR}
        BINARY_DIR ${OCCT_BUILD_DIR}
        INSTALL_DIR ${OCCT_INSTALL_DIR}
        # Drop OCCT's OCC_CONVERT_SIGNALS define on wasm. It converts OS signals
        # to C++ exceptions via setjmp INSIDE catch blocks — meaningless under
        # wasm (no real signals) and it violates the wasm-EH + wasm-SjLj rule
        # ("no setjmp within a catch"), which makes emscripten emit invalid wasm
        # (CompileError: "br_table: label arity inconsistent") that fails to
        # instantiate. Removing it lets the STEP write path compile to valid wasm.
        PATCH_COMMAND
            sed -i "/add_definitions(-DOCC_CONVERT_SIGNALS)/d"
            ${OCCT_SOURCE_DIR}/adm/cmake/occt_defs_flags.cmake
            ${_OCCT_EXTRA_PATCH}
        CMAKE_ARGS ${_OCCT_CMAKE_ARGS}
        BUILD_BYPRODUCTS ${_WASM_OCCT_BYPRODUCTS}
        USES_TERMINAL_DOWNLOAD TRUE
        USES_TERMINAL_CONFIGURE TRUE
        USES_TERMINAL_BUILD TRUE
        USES_TERMINAL_INSTALL TRUE
    )

    # Make sure the install include dir exists at configure time, even before
    # the build runs — otherwise IMPORTED targets that reference it via
    # INTERFACE_INCLUDE_DIRECTORIES emit a configure-time warning. CMake checks
    # include dirs at configure, archives only at link.
    file(MAKE_DIRECTORY ${OCCT_INCLUDE_DIR})
endif()

# IMPORTED targets: present them as if they were a normal library so callers
# can `target_link_libraries(_ada_cpp_ext_impl PRIVATE TKBRep TKMath ...)`.
# When building from source, the add_dependencies hook ensures occt_external is
# built before anything that links against these archives; with a prebuilt
# install the archives already exist, so no dependency is added.
set(WASM_OCCT_TARGETS)
foreach(_tk IN LISTS _WASM_OCCT_TOOLKITS)
    if(NOT TARGET ${_tk})
        add_library(${_tk} STATIC IMPORTED GLOBAL)
        set_target_properties(
            ${_tk}
            PROPERTIES
                IMPORTED_LOCATION ${OCCT_LIB_DIR}/lib${_tk}.a
                INTERFACE_INCLUDE_DIRECTORIES ${OCCT_INCLUDE_DIR}
        )
        if(NOT WASM_OCCT_PREBUILT_DIR)
            add_dependencies(${_tk} occt_external)
        endif()
        list(APPEND WASM_OCCT_TARGETS ${_tk})
    endif()
endforeach()

if(WASM_OCCT_PREBUILT_DIR)
    message(
        STATUS
        "OCCT-wasm targets from prebuilt install: ${WASM_OCCT_TARGETS}"
    )
else()
    message(
        STATUS
        "OCCT-wasm targets staged (built at compile time): ${WASM_OCCT_TARGETS}"
    )
endif()
message(STATUS "OCCT-wasm install dir: ${OCCT_INSTALL_DIR}")

if(ADACPP_WASM_SHARED_OCCT)
    # libocct.so: every toolkit above, whole-archive, in one side module. -sSIDE_MODULE=1 exports
    # every default-visibility symbol, which is the point: adacpp and gmsh import OCCT from here.
    # One .so rather than one per toolkit: Pyodide resolves a module's NEEDED entries by file name,
    # one file means one load and one place a second copy could come from.
    set_property(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS TRUE)
    set(_occt_stub "${CMAKE_BINARY_DIR}/occt_shared_stub.c")
    file(
        WRITE "${_occt_stub}"
        "int adacpp_occt_shared_marker(void) { return 1; }\n"
    )
    add_library(occt_shared SHARED "${_occt_stub}")
    set_target_properties(
        occt_shared
        PROPERTIES OUTPUT_NAME occt PREFIX "lib" SUFFIX ".so"
    )
    set(_occt_archives)
    foreach(_tk IN LISTS _WASM_OCCT_TOOLKITS)
        list(APPEND _occt_archives "${OCCT_LIB_DIR}/lib${_tk}.a")
    endforeach()
    target_link_options(
        occt_shared
        PRIVATE
            "-sSIDE_MODULE=1"
            "-sWASM_BIGINT"
            "-fwasm-exceptions"
            "-sSUPPORT_LONGJMP=wasm"
            "-Wl,--no-gc-sections"
            "-Wl,--whole-archive"
            ${_occt_archives}
            "-Wl,--no-whole-archive"
    )
    set_property(
        TARGET occt_shared
        APPEND
        PROPERTY LINK_DEPENDS ${_occt_archives}
    )
    target_include_directories(occt_shared INTERFACE ${OCCT_INCLUDE_DIR})
    if(NOT WASM_OCCT_PREBUILT_DIR)
        add_dependencies(occt_shared occt_external)
    endif()
    install(TARGETS occt_shared LIBRARY DESTINATION wheel-occt/ada_occt.libs)

    # Where a NEEDED libocct.so resolves. Pyodide resolves a NEEDED name to a path (LD_LIBRARY_PATH
    # = /usr/lib:<site-packages>, then the module's rpath) and only THEN checks whether that path is
    # already loaded -- and it does not normalise it. micropip preloads the ada_occt wheel's library
    # as <site-packages>/ada_occt.libs/libocct.so, so the rpath has to spell exactly that, or the
    # loader finds the file under another name and instantiates a SECOND OCCT. "$ORIGIN/.." would
    # do just that ("adacpp/../ada_occt.libs"). Pyodide 0.29's site-packages is fixed.
    set(WASM_OCCT_SHARED_RPATH
        "/lib/python3.13/site-packages/ada_occt.libs"
        CACHE STRING
        "Absolute in-Pyodide dir of libocct.so (rpath of the modules that NEED it)"
    )
    target_link_options(
        occt_shared
        INTERFACE "-Wl,-rpath,${WASM_OCCT_SHARED_RPATH}"
    )
endif()
