cmake_minimum_required(VERSION 3.20)

project(ada-cpp LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

include(cmake/pre_checks.cmake)

find_package(Python 3 COMPONENTS Interpreter Development.Module REQUIRED)

# Create a list called ADA_CPP_SOURCES of all cpp files inside the src dir
set(ADA_CPP_SOURCES
        src/adacpp.cpp
        src/helpers/helpers.cpp
        src/models/Models.cpp
        src/exchange/occt/step_writer.cpp
        src/exchange/occt/gltf_writer.cpp
        src/exchange/tinygltf/tiny.cpp
        src/fem/simple_mesh.cpp
        src/tessellation/ShapeTesselator.cpp
        src/tessellation/tess_helpers.cpp
        src/tessellation/manual/solids/boxes.cpp
        )

# Create a empty list to hold all the linked libs
set(ADA_CPP_LINK_LIBS)

# Add dependencies
include(cmake/deps_cgal.cmake)
include(cmake/deps_occ.cmake)
include(cmake/deps_gmsh.cmake)

# Create a library from your sources
add_library(ada_cpp_lib STATIC ${ADA_CPP_SOURCES})
target_link_libraries(ada_cpp_lib PRIVATE ${ADA_CPP_LINK_LIBS})

# Add Nanobind
include(cmake/nanobind.cmake)

# Link the module to your library instead of the sources directly
target_link_libraries(_ada_cpp_ext_impl PRIVATE ada_cpp_lib)

# Include the tests directory
add_subdirectory(tests/cpp)

# Install the module
install(TARGETS _ada_cpp_ext_impl LIBRARY DESTINATION .)