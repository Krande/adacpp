cmake_minimum_required(VERSION 3.20)

project(ada-cpp LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

include(cmake/pre_checks.cmake)

find_package(Python 3
        REQUIRED COMPONENTS Interpreter Development.Module
        OPTIONAL_COMPONENTS Development.SABIModule)

# Create a list called ADA_CPP_SOURCES of all cpp files inside the src dir
set(ADA_CPP_SOURCES
        src/adacpp.cpp
        src/helpers/helpers.cpp
        src/geom/Models.cpp
        src/geom/geometries.cpp
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

# Create a empty list to hold all the linked libs
set(ADA_CPP_LINK_LIBS)

# Add dependencies
include(cmake/deps_cgal.cmake)
include(cmake/deps_occ.cmake)
include(cmake/deps_gmsh.cmake)

# Add Nanobind
include(cmake/nanobind.cmake)

# Link libraries to the module
target_link_libraries(_ada_cpp_ext_impl PRIVATE ${ADA_CPP_LINK_LIBS})

# Include the tests directory
OPTION(BUILD_TESTING "Build the testing tree." OFF)
if (BUILD_TESTING)
    add_subdirectory(tests/cpp)
endif (BUILD_TESTING)

# Install the module
install(TARGETS _ada_cpp_ext_impl LIBRARY DESTINATION .)