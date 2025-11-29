cmake_minimum_required(VERSION 3.20)

project(ada-cpp LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

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

set(ADA_CPP_PY_SOURCES
        src/adacpp_py_wrap.cpp
        src/cadit/ifc/ifc_py_wrap.cpp
        src/cadit/occt/occt_py_wrap.cpp
        src/cadit/tinygltf/tiny_py_wrap.cpp
        src/geom/geom_py_wrap.cpp
        src/fem/fem_py_wrap.cpp
        src/visit/visit_py_wrap.cpp
)
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
else ()
    include(cmake/pre_checks.cmake)

    find_package(Python REQUIRED COMPONENTS Interpreter Development.Module
            OPTIONAL_COMPONENTS Development.SABIModule
    )
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
