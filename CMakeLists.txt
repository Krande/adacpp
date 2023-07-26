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
        src/exchange/step_writer.cpp
        src/exchange/gltf_writer.cpp
        src/exchange/tiny.cpp
        src/geom/ShapeTesselator.cpp
        src/geom/tess_helpers.cpp
        src/tessellation/solids/boxes.cpp
        )

# Create a empty list to hold all the linked libs
set(ADA_CPP_LINK_LIBS)

# Add dependencies
include(cmake/deps_cgal.cmake)
include(cmake/deps_occ.cmake)

# Add Nanobind
include(cmake/nanobind.cmake)

# Link libraries to the module
target_link_libraries(_ada_cpp_ext_impl PRIVATE ${ADA_CPP_LINK_LIBS})

# Link the module to OpenCASCADE
install(TARGETS _ada_cpp_ext_impl LIBRARY DESTINATION .)