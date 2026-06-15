cmake_minimum_required(VERSION 3.20)

project(ada-cpp LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

# Set policy CMP0167 to OLD to suppress warning about FindBoost module deprecation
# CGAL still uses the old FindBoost module, so we need to allow it
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 OLD)
endif()

OPTION(BUILD_WASM "Build the project for WebAssembly" OFF)
OPTION(BUILD_PYTHON "Build the project for Python" ON)
OPTION(BUILD_STP2GLB "Build the STP2GLB executable" ON)
OPTION(BUILD_STP2GLB_TESTING "Build the testing tree." OFF)
OPTION(BUILD_DEBUG_TESTS "Build the testing tree used primarily for debugging." OFF)

if (DEFINED TINY_INCLUDE_DIR)
    include_directories(${TINY_INCLUDE_DIR})
    message(STATUS "TINY_INCLUDE_DIR set to ${TINY_INCLUDE_DIR}")
else()
    message(STATUS "TINY_INCLUDE_DIR not set")
endif ()

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
    include(cmake/wasm_occt.cmake)
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
    include(cmake/pre_checks.cmake)
    # Add dependencies
    include(cmake/deps_cgal.cmake)
    include(cmake/deps_occ.cmake)
    include(cmake/deps_gmsh.cmake)
    include(cmake/deps_ifc.cmake)

    # Find Threads package for pthread support on Linux
    find_package(Threads REQUIRED)
endif ()

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
