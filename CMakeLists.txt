cmake_minimum_required(VERSION 3.20)

project(ada-cpp LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)

OPTION(CONDA_LOCAL_DEV "Install into activate local development conda env." OFF)
OPTION(BUILD_WASM "Build the project for WebAssembly" OFF)

if (CONDA_LOCAL_DEV)
    message(STATUS "Building for local development conda env.")
    set(EXE_BIN_DIR ${CMAKE_INSTALL_PREFIX}/bin)
endif ()

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
        src/cadit/tinygltf/tiny.cpp
        src/fem/simple_mesh.cpp
        src/visit/ShapeTesselator.cpp
        src/visit/tess_helpers.cpp
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
        src/cadit/tinygltf/tiny.h
        src/cadit/tinygltf/tinyload.h
        src/fem/simple_mesh.h
        src/visit/ShapeTesselator.h
        src/visit/tess_helpers.h
        src/visit/TessellateFactory.h
)
# Create a empty list to hold all the linked libs
set(ADA_CPP_LINK_LIBS)

# If building for WebAssembly, use the custom WASM toolchain
if (BUILD_WASM)
    message(STATUS "Building for WebAssembly using Emscripten")
    #set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/wasm_toolchain.cmake")
    include(cmake/wasm_toolchain.cmake)
else ()
    include(cmake/pre_checks.cmake)

    find_package(Python 3
            REQUIRED COMPONENTS Interpreter Development.Module
            OPTIONAL_COMPONENTS Development.SABIModule
    )
    # Add dependencies
    include(cmake/deps_cgal.cmake)
    include(cmake/deps_occ.cmake)
    include(cmake/deps_gmsh.cmake)
    include(cmake/deps_ifc.cmake)
endif ()

# Add Nanobind
if (NOT BUILD_WASM)
    message(STATUS "Not a WASM build")
    include(cmake/nanobind.cmake)

    # Link libraries to the module
    target_link_libraries(_ada_cpp_ext_impl PRIVATE ${ADA_CPP_LINK_LIBS})

    # Install the module
    install(TARGETS _ada_cpp_ext_impl LIBRARY DESTINATION .)

    # Install the C++ executable
    set(SOURCES src/main.cpp src/cadit/occt/step_to_glb.cpp)
    set(HEADERS src/cadit/occt/step_to_glb.h)

    add_executable(STP2GLB ${SOURCES} ${HEADERS})

    set_target_properties(STP2GLB PROPERTIES
            INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/bin"
    )
    target_link_libraries(STP2GLB ${ADA_CPP_LINK_LIBS})

    # install to executable into the bin dir
    message(STATUS "Installing executable to ${EXE_BIN_DIR}")
    install(TARGETS STP2GLB DESTINATION ${EXE_BIN_DIR})
endif ()

# WebAssembly-specific settings
if (BUILD_WASM)
    include(cmake/wasm_build.cmake)
endif ()



# Include the tests directory
OPTION(BUILD_TESTING "Build the testing tree." OFF)
if (BUILD_TESTING)
    message(STATUS "Building the testing tree.")
    set(CMAKE_INSTALL_RPATH "${EXE_BIN_DIR}")
    set(CMAKE_BUILD_RPATH "${EXE_BIN_DIR}")
    include(tests/cpp/tests.cmake)
endif (BUILD_TESTING)