# cmake/wasm_gmsh.cmake
#
# SPIKE (option A, Aibel-Engineering/adapy-plugins#177 step 2d): cross-compile gmsh to a STATIC
# wasm library and link it INTO _ada_cpp_ext_impl.so, against the same OCCT-wasm archives
# (wasm_occt.cmake) adacpp already links. One OCCT copy in one side module means the
# TopoDS_Shape* that cad.to_topods_pointer() hands out is directly usable by gmsh's
# occ.importShapesNativePointer, exactly as natively.
#
# Python reaches gmsh through gmsh's own upstream api/gmsh.py (ctypes), shipped as a top-level
# `gmsh` module in the wheel. The only patch is where it loads the library from: under
# sys.platform == "emscripten" it CDLLs adacpp's extension module, which exports gmsh's C API
# (every GMSH_API function in api/gmshc.h, see ADACPP_GMSH_EXPORTS below).
#
# Only included when BUILD_WASM AND BUILD_PYTHON AND ADACPP_WITH_GMSH (default OFF). Native builds
# keep linking conda-forge's libgmsh (deps_gmsh.cmake) and never read this file.
#
# gmsh options: everything off (-DDEFAULT=0) and then exactly what the mesh step uses switched on:
#   MESH, POST (size fields), PARSER, SOLVER, OCC + OCC_CAF (STEP import with labels) on adacpp's
#   OCCT, EIGEN (bundled; so no BLAS/LAPACK, no Fortran), ANN + MATHEX + ONELAB (small contribs the
#   core expects). Single-threaded (no OpenMP/MPI). No FLTK/GRAPHICS, HXT, NETGEN, MMG, METIS,
#   ALGLIB, GMP, MED, CGNS, PETSc.
#
# BLOSSOM (ADACPP_GMSH_BLOSSOM, default ON) is gmsh's default quad recombination (algorithm 1), as
# in the native conda gmsh. contrib/blossom (Concorde/Blossom IV) is not GPL: Bill Cook's
# permission covers use "within the Gmsh system" (contrib/blossom/README.txt). That is fine for
# the internal-only viewer this wheel serves, but it is a restriction a GPL-3 wheel handed to third
# parties could not pass on (Fedora/Debian strip it as nonfree), so it stays switchable. With it
# OFF, gmsh falls back to its greedy pairing (meshGFaceOptimize.cpp, `#if defined(HAVE_BLOSSOM)`):
# four-sided transfinite plates are unaffected, other outlines come out quad-dominant, not all-quad.

include(ExternalProject)
include(FetchContent)

option(
    ADACPP_GMSH_BLOSSOM
    "Build gmsh's Blossom quad recombination (non-GPL contrib; internal use only)"
    ON
)
if(ADACPP_GMSH_BLOSSOM)
    set(_GMSH_BLOSSOM ON)
else()
    set(_GMSH_BLOSSOM OFF)
endif()

set(ADACPP_GMSH_VERSION
    "4.15.2"
    CACHE STRING
    "gmsh release cross-built into the pyodide extension (keep equal to the native conda gmsh)"
)
set(ADACPP_GMSH_SHA256
    "be3f66f225d27ba9fa014f07e83169285da8a051b0e8ab7103d88066b39bdd3e"
    CACHE STRING
    "sha256 of gmsh-<ver>-source.tgz"
)

# Fetch the source at CONFIGURE time (not inside the ExternalProject): the export list is parsed
# out of api/gmshc.h and api/gmsh.py is patched and staged for install, both before any build.
#
# OCCT 8 source compatibility comes from conda-forge's gmsh-feedstock (recipe/patches, the same
# patches the native conda gmsh 4.15.2 is built with), vendored in cmake/patches/gmsh/:
#   cxx17_for_occt.patch      CMAKE_CXX_STANDARD 14 -> 17 (OCCT 8 headers use std::optional ...)
#   occt800_api_compat.patch  removed Handle_* typedefs, iterator-typedef headers, Geom2dLProp
# Plus one adacpp-only edit: gmsh enables OCC_CAF only when find_package(Freetype) succeeds, and
# nothing on the STEP/XCAF import path uses freetype (it is for the GUI's text rendering), so the
# probe is replaced instead of cross-building freetype.
set(_GMSH_PATCH_DIR "${CMAKE_CURRENT_LIST_DIR}/patches/gmsh")
FetchContent_Declare(
    gmsh_src
    URL https://gmsh.info/src/gmsh-${ADACPP_GMSH_VERSION}-source.tgz
    URL_HASH SHA256=${ADACPP_GMSH_SHA256}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_GetProperties(gmsh_src)
if(NOT gmsh_src_POPULATED)
    FetchContent_Populate(gmsh_src)
endif()
set(GMSH_SOURCE_DIR "${gmsh_src_SOURCE_DIR}")

set(_gmsh_patch_stamp "${GMSH_SOURCE_DIR}/.adacpp-wasm-patched")
if(NOT EXISTS "${_gmsh_patch_stamp}")
    # `patch`, not `git apply`: the build tree usually sits inside adacpp's own checkout, and git
    # apply run there resolves the patch paths against adacpp's repo root instead of gmsh's tree.
    find_program(ADACPP_PATCH_EXECUTABLE patch REQUIRED)
    foreach(_p cxx17_for_occt.patch occt800_api_compat.patch)
        execute_process(
            COMMAND
                "${ADACPP_PATCH_EXECUTABLE}" -p1 --forward --input
                "${_GMSH_PATCH_DIR}/${_p}"
            WORKING_DIRECTORY "${GMSH_SOURCE_DIR}"
            RESULT_VARIABLE _rc
            ERROR_VARIABLE _err
        )
        if(NOT _rc EQUAL 0)
            message(FATAL_ERROR "gmsh-wasm: ${_p} does not apply: ${_err}")
        endif()
    endforeach()
    file(READ "${GMSH_SOURCE_DIR}/CMakeLists.txt" _gmsh_cml)
    string(
        REPLACE "find_package(Freetype)"
        "set(FREETYPE_FOUND TRUE) # adacpp-wasm: no freetype"
        _gmsh_cml
        "${_gmsh_cml}"
    )
    file(WRITE "${GMSH_SOURCE_DIR}/CMakeLists.txt" "${_gmsh_cml}")
    file(TOUCH "${_gmsh_patch_stamp}")
endif()

set(GMSH_BUILD_DIR "${CMAKE_BINARY_DIR}/_deps/gmsh-build")
set(GMSH_WASM_LIB "${GMSH_BUILD_DIR}/libgmsh.a")

# Same EH/SjLj model as OCCT and adacpp (one model across the whole link), PIC for the side module,
# and hidden visibility so gmsh's C++ internals bind directly inside adacpp's module and never
# interpose with anything else loaded into pyodide. The C API is exported explicitly below.
set(_GMSH_FLAGS
    "-fwasm-exceptions -sSUPPORT_LONGJMP=wasm -fPIC -fvisibility=hidden"
)

set(_GMSH_CMAKE_ARGS
    -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
    -DCMAKE_BUILD_TYPE=Release
    "-DCMAKE_C_FLAGS=${_GMSH_FLAGS}"
    "-DCMAKE_CXX_FLAGS=${_GMSH_FLAGS} -fvisibility-inlines-hidden"
    # Find OCCT's archives in the OCCT-wasm install: add it as a find ROOT (the emscripten toolchain
    # appends its sysroot), so find_library stays confined to wasm trees and never sees host libs.
    -DCMAKE_FIND_ROOT_PATH=${OCCT_INSTALL_DIR}
    -DOCC_INC=${OCCT_INCLUDE_DIR}
    -DDEFAULT=0
    -DENABLE_BUILD_LIB=ON
    -DENABLE_BUILD_SHARED=OFF
    -DENABLE_BUILD_DYNAMIC=OFF
    -DENABLE_OS_SPECIFIC_INSTALL=OFF
    -DENABLE_PRIVATE_API=OFF
    -DENABLE_WRAP_PYTHON=OFF
    -DENABLE_TESTS=OFF
    -DENABLE_FLTK=OFF
    -DENABLE_GRAPHICS=OFF
    -DENABLE_OSMESA=OFF
    -DENABLE_MESH=ON
    -DENABLE_POST=ON
    -DENABLE_PARSER=ON
    -DENABLE_SOLVER=ON
    -DENABLE_OCC=ON
    -DENABLE_OCC_CAF=ON
    -DENABLE_OCC_STATIC=ON
    -DENABLE_OCC_TBB=OFF
    -DENABLE_EIGEN=ON
    -DENABLE_BLAS_LAPACK=OFF
    -DENABLE_ANN=ON
    -DENABLE_MATHEX=ON
    -DENABLE_ONELAB=ON
    -DENABLE_BLOSSOM=${_GMSH_BLOSSOM}
    -DENABLE_OPENMP=OFF
    -DENABLE_MPI=OFF
    -DENABLE_HXT=OFF
    -DENABLE_NETGEN=OFF
    -DENABLE_MMG=OFF
    -DENABLE_METIS=OFF
    -DENABLE_ALGLIB=OFF
    -DENABLE_GMP=OFF
    -DENABLE_KBIPACK=OFF
    -DENABLE_MED=OFF
    -DENABLE_CGNS=OFF
    -DENABLE_PETSC=OFF
    -DENABLE_SLEPC=OFF
    -DENABLE_MUMPS=OFF
    -DENABLE_ZIPPER=OFF
    # GModelIO_OBJ.cpp includes tiny_obj_loader.h unconditionally (small, bundled).
    -DENABLE_TINYOBJLOADER=ON
    -DENABLE_CAIRO=OFF
    -DENABLE_TOUCHBAR=OFF
)

set(_GMSH_DEPENDS)
if(TARGET occt_external)
    list(APPEND _GMSH_DEPENDS occt_external)
endif()

ExternalProject_Add(
    gmsh_external
    SOURCE_DIR ${GMSH_SOURCE_DIR}
    BINARY_DIR ${GMSH_BUILD_DIR}
    DOWNLOAD_COMMAND ""
    UPDATE_COMMAND ""
    # gmsh turns $ENV{CMAKE_PREFIX_PATH} into -I flags (CMakeLists.txt "ENV_CMAKE_PREFIX_PATH").
    # Under pixi that is the conda env and its x86_64 glibc sysroot, whose headers then shadow
    # emscripten's libc ("gnu/stubs-32.h not found"). Configure gmsh without it. Same for conda's
    # compiler activation flags: LDFLAGS carries GNU-ld options wasm-ld rejects, which fails every
    # try_compile and silently flips gmsh's platform checks (size_t, vsnprintf, socklen_t).
    # (wasm_toolchain.cmake clears the same flags for adacpp's own configure.)
    CONFIGURE_COMMAND
        ${CMAKE_COMMAND} -E env --unset=CMAKE_PREFIX_PATH
        --unset=CMAKE_INCLUDE_PATH --unset=LDFLAGS --unset=CFLAGS
        --unset=CXXFLAGS --unset=CPPFLAGS ${CMAKE_COMMAND} -G ${CMAKE_GENERATOR} -S
        <SOURCE_DIR> -B <BINARY_DIR> ${_GMSH_CMAKE_ARGS}
    # Only the static library: the default target also links the gmsh executable.
    BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --target lib
    INSTALL_COMMAND ""
    BUILD_BYPRODUCTS ${GMSH_WASM_LIB}
    DEPENDS ${_GMSH_DEPENDS}
    USES_TERMINAL_CONFIGURE TRUE
    USES_TERMINAL_BUILD TRUE
)

add_library(gmsh_wasm STATIC IMPORTED GLOBAL)
set_target_properties(gmsh_wasm PROPERTIES IMPORTED_LOCATION ${GMSH_WASM_LIB})
add_dependencies(gmsh_wasm gmsh_external)

# The C API gmsh.py calls through ctypes: every GMSH_API function declared in api/gmshc.h.
# SIDE_MODULE=2 exports only what is named, so each one gets an explicit --export.
file(STRINGS "${GMSH_SOURCE_DIR}/api/gmshc.h" _gmshc_lines REGEX "^GMSH_API ")
set(ADACPP_GMSH_EXPORTS)
foreach(_line IN LISTS _gmshc_lines)
    if(_line MATCHES "[ *](gmsh[A-Za-z0-9_]+)[ ]*\\(")
        list(APPEND ADACPP_GMSH_EXPORTS ${CMAKE_MATCH_1})
    endif()
endforeach()
list(REMOVE_DUPLICATES ADACPP_GMSH_EXPORTS)
list(LENGTH ADACPP_GMSH_EXPORTS _n_exports)
message(STATUS "gmsh-wasm: exporting ${_n_exports} gmsh C API functions")

# Upstream gmsh.py, patched only in where it finds the library. Staged for the wheel install.
file(READ "${GMSH_SOURCE_DIR}/api/gmsh.py" _gmsh_py)
set(_gmsh_py_anchor "\nlibpath = None\n")
string(FIND "${_gmsh_py}" "${_gmsh_py_anchor}" _anchor_at)
if(_anchor_at EQUAL -1)
    message(FATAL_ERROR "gmsh-wasm: cannot patch api/gmsh.py (anchor 'libpath = None' not found)")
endif()
string(
    CONCAT _gmsh_py_patch
    "\nlibpath = None\n"
    "# adacpp-wasm: in the adacpp pyodide wheel gmsh is statically linked into adacpp's own\n"
    "# extension module (one OCCT for adacpp and gmsh), which exports gmsh's C API.\n"
    "import sys as _sys\n"
    "if _sys.platform == \"emscripten\":\n"
    "    import adacpp._ada_cpp_ext_impl as _adacpp_ext\n"
    "    libpath = _adacpp_ext.__file__\n"
)
string(REPLACE "${_gmsh_py_anchor}" "${_gmsh_py_patch}" _gmsh_py "${_gmsh_py}")
set(ADACPP_GMSH_PY "${CMAKE_BINARY_DIR}/gmsh_py/gmsh.py")
file(WRITE "${ADACPP_GMSH_PY}" "${_gmsh_py}")

# gmsh's licence travels with the wheel next to gmsh.py.
set(ADACPP_GMSH_LICENSE "${GMSH_SOURCE_DIR}/LICENSE.txt")
