cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmacosx-version-min=10.14")
project(ada-cpp LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

if (CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(STATUS "Building for x64 architecture")
else ()
    message(FATAL_ERROR "This project requires a 64-bit toolchain. Please update your toolchain arch to 'x86_amd64'")
endif ()

if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "Build type not set, defaulting to Release")
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif ()
message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

find_package(Python 3 COMPONENTS Interpreter Development.Module REQUIRED)

# Create a list called ADA_CPP_SOURCES of all cpp files inside the src dir
set(ADA_CPP_SOURCES
        src/adacpp.cpp
        src/helpers/helpers.cpp
        src/models/Models.cpp
        src/exchange/step_writer.cpp
        src/exchange/gltf_writer.cpp
        src/geom/ShapeTesselator.cpp
        src/geom/tess_helpers.cpp
        )

# Find OpenCASCADE
find_package(OpenCASCADE REQUIRED)
if (OpenCASCADE_FOUND)
    message(STATUS "OpenCASCADE version found: " ${OpenCASCADE_MAJOR_VERSION} ".." ${OpenCASCADE_MINOR_VERSION} ".." ${OpenCASCADE_MAINTENANCE_VERSION})
    message(STATUS "OpenCASCADE include directory: " ${OpenCASCADE_INCLUDE_DIR})
    message(STATUS "OpenCASCADE binary directory: " ${OpenCASCADE_BINARY_DIR})
    message(STATUS "OpenCASCADE library directory: " ${OpenCASCADE_LIBRARY_DIR})

    include_directories(${OpenCASCADE_INCLUDE_DIR})
    link_directories(${OpenCASCADE_LIBRARY_DIR})
endif (OpenCASCADE_FOUND)


# Detect the installed nanobind package and import it into CMake
execute_process(
        COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
        OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE NB_DIR)

message(STATUS "NanoBind Cmake directory: " ${NB_DIR})
list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")

# Import nanobind through CMake's find_package mechanism
find_package(nanobind CONFIG REQUIRED)

# Print the list of cpp files separated by spaces without altering it
string(REPLACE ";" " " ADA_CPP_SOURCES_STR "${ADA_CPP_SOURCES}")
message(STATUS "AdaCpp sources: " ${ADA_CPP_SOURCES_STR})

# Create a Python module
nanobind_add_module(_ada_cpp_ext_impl ${ADA_CPP_SOURCES})

target_link_libraries(_ada_cpp_ext_impl
        PRIVATE
        TKernel
        TKMath
        TKBRep
        TKPrim
        TKTopAlgo
        TKSTEP
        TKXDESTEP
        TKRWMesh
        )

# Link the module to OpenCASCADE
install(TARGETS _ada_cpp_ext_impl LIBRARY DESTINATION .)