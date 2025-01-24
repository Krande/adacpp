cmake_minimum_required(VERSION 3.20)

project(ada-cpp LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

OPTION(BUILD_WASM "Build the project for WebAssembly" OFF)
OPTION(BUILD_PYTHON "Build the project for Python" ON)
OPTION(BUILD_STP2GLB "Build the STP2GLB executable" ON)
OPTION(BUILD_TESTING "Build the testing tree." OFF)


# Create a list called ADA_CPP_SOURCES of all cpp files inside the src dir
set(ADA_CPP_SOURCES
        src/adacpp.cpp
        src/cadit/ifc/ifcop.cpp
        src/helpers/helpers.cpp
        src/geom/Models.cpp
        src/geom/geometries.cpp
        src/cadit/occt/step_to_glb.cpp
        src/cadit/occt/occt_convert.cpp
        src/cadit/occt/step_writer.cpp
        src/cadit/occt/gltf_writer.cpp
        src/cadit/occt/colors.cpp
        src/cadit/tinygltf/tinygltf.cpp
        src/cadit/tinygltf/tiny_helpers.cpp
        src/cadit/tinygltf/tiny.cpp
        src/fem/simple_mesh.cpp
        src/visit/ShapeTesselator.cpp
        src/visit/tess_helpers.cpp
        src/visit/tess_py_wrap.cpp
        src/visit/manual/solids/boxes.cpp
        src/visit/TessellateFactory.cpp
)
set(ADA_CPP_HEADERS
        src/cadit/ifc/ifcop.h
        src/helpers/helpers.h
        src/geom/geometries.h
        src/cadit/occt/occt_convert.h
        src/cadit/occt/step_to_glb.h
        src/cadit/occt/step_writer.h
        src/cadit/occt/gltf_writer.h
        src/cadit/occt/colors.h
        src/cadit/tinygltf/tiny_helpers.h
        src/cadit/tinygltf/tiny.h
        src/fem/simple_mesh.h
        src/visit/ShapeTesselator.h
        src/visit/tess_helpers.h
        src/visit/tess_py_wrap.h
        src/visit/TessellateFactory.h
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

if (BUILD_TESTING)
    message(STATUS "Building the testing tree.")
    #include(tests/cpp/test1.cmake)
endif ()
