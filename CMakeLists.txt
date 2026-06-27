cmake_minimum_required(VERSION 3.20)

project(ada-cpp LANGUAGES C CXX)  # C for the vendored libtess2 sources
set(CMAKE_CXX_STANDARD 20)

# Set policy CMP0167 to OLD to suppress warning about FindBoost module deprecation
# CGAL still uses the old FindBoost module, so we need to allow it
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 OLD)
endif()

OPTION(BUILD_WASM "Build the project for WebAssembly" OFF)
OPTION(BUILD_STEP_GLB_WASM "Build ONLY the OCC-free STEP->GLB wasm module (embind, no OCCT/pyodide/Python)" OFF)
OPTION(BUILD_STP2GLB_STANDALONE "Build ONLY the OCC-free STP2GLB CLI — skip OCCT/CGAL/gmsh/IFC/Python" OFF)
OPTION(BUILD_PYTHON "Build the project for Python" ON)
OPTION(BUILD_STP2GLB "Build the STP2GLB executable" ON)
OPTION(BUILD_STP2GLB_TESTING "Build the testing tree." OFF)
OPTION(BUILD_DEBUG_TESTS "Build the testing tree used primarily for debugging." OFF)

# Standalone OCC-free STP2GLB CLI: never compile the Python module (mirrors how BUILD_STEP_GLB_WASM
# narrows the build). Forced here so the option is self-sufficient — the heavy native deps and the
# python module are both skipped below regardless of how the caller set BUILD_PYTHON.
if (BUILD_STP2GLB_STANDALONE)
    set(BUILD_PYTHON OFF CACHE BOOL "Disabled by BUILD_STP2GLB_STANDALONE" FORCE)
endif ()

if (DEFINED TINY_INCLUDE_DIR)
    include_directories(${TINY_INCLUDE_DIR})
    message(STATUS "TINY_INCLUDE_DIR set to ${TINY_INCLUDE_DIR}")
else()
    message(STATUS "TINY_INCLUDE_DIR not set")
endif ()

# Vendored libtess2 (native + wasm) — boundary-driven triangulator for the step2glb-style
# tessellation path. Header-path only here; ${LIBTESS2_SOURCES} are appended to
# ADA_CPP_SOURCES alongside the tessellator once src/visit/tessellate_libtess2.cpp lands.
include(cmake/deps_libtess2.cmake)
include_directories(${LIBTESS2_INCLUDE_DIR})

# Vendored meshoptimizer (native + wasm) — meshopt_simplify cleanup (step2glb merge parity) +
# EXT_meshopt_compression encoders. ${MESHOPT_SOURCES} are appended to ADA_CPP_SOURCES below.
include(cmake/deps_meshopt.cmake)
include_directories(${MESHOPT_INCLUDE_DIR})

# Header-only NGEOM neutral types + the native STEP (Part-21) reader. On the include path so a
# consumer (e.g. src/cad/cad_py_wrap.cpp) can include step_reader.h, whose own ngeom includes are
# unqualified. OCC-free, so this is valid for the wasm build too.
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/geom/neutral ${CMAKE_CURRENT_SOURCE_DIR}/src/cadit/step)

# Create a list called ADA_CPP_SOURCES of all cpp files inside the src dir
if (BUILD_WASM)
    # Wasm builds now link the cross-built OCCT toolkits (see wasm_occt.cmake), so
    # the OCCT-backed CAD sources that the wasm bindings call must be compiled in
    # too. Anything still missing here is emitted as an `env` import by the side
    # module and traps with `unreachable` when first called (the function table
    # slot is a stub). step_writer.cpp + helpers.cpp back adacpp's STEP write
    # (backend.write_step); without them write_shapes_to_step is an unresolved
    # import. CGAL/gmsh/IfcOpenShell are still wasm-unavailable, so their sources
    # stay in the #else.
    set(ADA_CPP_SOURCES
            src/geom/Color.cpp
            src/geom/Models.cpp
            src/cadit/occt/step_writer.cpp
            src/helpers/helpers.cpp
            # NGEOM OCC-free path (neutral layer is header-only; libtess2 tessellator + the
            # vendored libtess2 C compile for wasm too). Taxonomy path is native-only (ifc).
            src/geom/neutral/ngeom_tessellate.cpp
            src/geom/neutral/ngeom_meshopt.cpp
            src/geom/neutral/ngeom_boolean.cpp
            ${LIBTESS2_SOURCES}
            ${MESHOPT_SOURCES}
    )
else ()
    set(ADA_CPP_SOURCES
            src/cadit/ifc/ifcop.cpp
            src/cadit/occt/step_to_glb.cpp
            src/cadit/occt/step_writer.cpp
            src/cadit/occt/colors.cpp
            src/cadit/tinygltf/tinygltf.cpp
            src/cadit/tinygltf/tiny_helpers.cpp
            src/helpers/helpers.cpp
            src/geom/Color.cpp
            src/geom/Models.cpp
            src/geom/geometries.cpp
            src/fem/simple_mesh.cpp
            src/visit/ShapeTesselator.cpp
            src/visit/tess_helpers.cpp
            src/visit/manual/solids/boxes.cpp
            src/visit/TessellateFactory.cpp
            # NGEOM: OCC-free libtess2 path + the taxonomy (ifcopenshell) path
            src/geom/neutral/ngeom_tessellate.cpp
            src/cadit/ifc/ngeom_taxonomy_build.cpp
            src/cadit/ifc/ngeom_taxonomy_kernel.cpp
            src/geom/neutral/ngeom_meshopt.cpp
            src/geom/neutral/ngeom_boolean.cpp
            ${LIBTESS2_SOURCES}
            ${MESHOPT_SOURCES}
    )
endif ()
set(ADA_CPP_HEADERS
        src/cadit/ifc/ifcop.h
        src/cadit/occt/step_to_glb.h
        src/cadit/occt/step_writer.h
        src/cadit/occt/colors.h
        src/cadit/tinygltf/tiny_helpers.h
        src/fem/simple_mesh.h
        src/geom/Color.h
        src/geom/geometries.h
        src/helpers/helpers.h
        src/visit/ShapeTesselator.h
        src/visit/tess_helpers.h
        src/visit/TessellateFactory.h
)

if (BUILD_WASM)
    set(ADA_CPP_PY_SOURCES
            src/adacpp_py_wrap.cpp
            src/cad/cad_py_wrap.cpp
    )
else ()
    set(ADA_CPP_PY_SOURCES
            src/adacpp_py_wrap.cpp
            src/cad/cad_py_wrap.cpp
            src/cadit/ifc/ifc_py_wrap.cpp
            src/cadit/occt/occt_py_wrap.cpp
            src/cadit/tinygltf/tiny_py_wrap.cpp
            src/geom/geom_py_wrap.cpp
            src/fem/fem_py_wrap.cpp
            src/visit/visit_py_wrap.cpp
    )
endif ()
set(ADA_CPP_PY_HEADERS
        src/cadit/ifc/ifc_py_wrap.h
        src/cadit/occt/occt_py_wrap.h
        src/cadit/tinygltf/tiny_py_wrap.h
        src/geom/geom_py_wrap.h
        src/fem/fem_py_wrap.h
        src/visit/visit_py_wrap.h
)

# Create a empty list to hold all the linked libs
set(ADA_CPP_LINK_LIBS)

# If building for WebAssembly, use the custom WASM toolchain
if (BUILD_WASM)
    message(STATUS "Building for WebAssembly using Emscripten")
    include(cmake/wasm_toolchain.cmake)
    # Cross-compile a minimal OCCT to wasm via FetchContent. Sets WASM_OCCT_TARGETS
    # for the nanobind module / executables to link against. First spike is
    # FoundationClasses only; module set widens as features land.
    # Skipped for the OCC-free STEP->GLB module — it needs no OCCT, so we avoid the heavy cross-build.
    if (NOT BUILD_STEP_GLB_WASM)
        include(cmake/wasm_occt.cmake)
    endif ()
endif ()

if (BUILD_PYTHON)
    if (BUILD_WASM)
        # Cross-compile against pyodide's Python headers; pyodide modules don't link
        # libpython (it's loaded dynamically by the pyodide runtime), so Development.Module
        # is unavailable. We point CMake at the xbuildenv Python include dir manually
        # via the preset and only request the Interpreter component (host python, for
        # running nanobind's helper scripts at configure time).
        find_package(Python 3.12 REQUIRED COMPONENTS Interpreter)
        if (NOT DEFINED Python_INCLUDE_DIR OR Python_INCLUDE_DIR STREQUAL "")
            message(FATAL_ERROR "Python_INCLUDE_DIR must be set to the pyodide xbuildenv Python.h location for WASM+PYTHON builds.")
        endif ()
        set(Python_INCLUDE_DIRS "${Python_INCLUDE_DIR}")
    else ()
        find_package(Python REQUIRED COMPONENTS Interpreter Development.Module
                OPTIONAL_COMPONENTS Development.SABIModule
        )
    endif ()
endif ()

if (NOT BUILD_WASM)
    if (BUILD_STP2GLB_STANDALONE)
        # OCC-free CLI only: skip the heavy native deps (OCCT/CGAL/gmsh/IFC) and pre_checks entirely,
        # the same way BUILD_STEP_GLB_WASM short-circuits the OCCT cross-build. Only Threads is needed
        # to link the CLI (manifold + libtess2 + meshopt are pulled in unconditionally above/below).
        message(STATUS "BUILD_STP2GLB_STANDALONE: skipping OCCT/CGAL/gmsh/IFC/Python — building the OCC-free CLI only")
        find_package(Threads REQUIRED)
    else ()
        include(cmake/pre_checks.cmake)
        # Add dependencies
        include(cmake/deps_cgal.cmake)
        include(cmake/deps_occ.cmake)
        include(cmake/deps_gmsh.cmake)
        include(cmake/deps_ifc.cmake)

        # Find Threads package for pthread support on Linux
        find_package(Threads REQUIRED)
    endif ()
endif ()

# Manifold mesh-boolean (CSG) for the OCC-free libtess2 path (ngeom_boolean.cpp) — native AND
# wasm (it is self-contained + cross-compiles under emscripten). Included after the toolchain /
# OCCT setup above so EMSCRIPTEN is defined for the wasm EH flags in deps_manifold.cmake.
include(cmake/deps_manifold.cmake)
list(APPEND ADA_CPP_LINK_LIBS manifold)
add_compile_definitions(ADACPP_HAVE_MANIFOLD)

if (BUILD_STP2GLB)
    message(STATUS "Building the STP2GLB executable")
    include(cmake/build_stp2glb.cmake)
endif ()

if (BUILD_PYTHON)
    message(STATUS "Building for Python")
    include(cmake/build_python.cmake)
endif ()

# WebAssembly-specific settings
if (BUILD_WASM)
    include(cmake/wasm_build.cmake)
endif ()

if (BUILD_DEBUG_TESTING)
    message(STATUS "Building the testing tree.")
    include(tests/cpp/test1.cmake)
endif (BUILD_DEBUG_TESTING)
