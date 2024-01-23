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
        src/cadit/ifc/ifcop.cpp
        src/helpers/helpers.cpp
        src/geom/Models.cpp
        src/geom/geometries.cpp
        src/cadit/occt/step_to_glb.cpp
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

set(TINYGLTF_HEADER_ONLY ON CACHE INTERNAL "" FORCE)
set(TINYGLTF_INSTALL OFF CACHE INTERNAL "" FORCE)

# set the path to the tinygltf source which is in the conda env $PREFIX/include
#set(TINYGLTF_SOURCE_DIR ${CMAKE_PREFIX_PATH}/include)
#add_subdirectory(TINYGLTF_SOURCE_DIR)

# Add dependencies
include(cmake/deps_cgal.cmake)
include(cmake/deps_occ.cmake)
include(cmake/deps_gmsh.cmake)
include(cmake/deps_ifc.cmake)

# Add Nanobind
include(cmake/nanobind.cmake)

# Link libraries to the module
target_link_libraries(_ada_cpp_ext_impl PRIVATE ${ADA_CPP_LINK_LIBS})

# Install the module
install(TARGETS _ada_cpp_ext_impl LIBRARY DESTINATION .)

# Include the tests directory
OPTION(BUILD_TESTING "Build the testing tree." OFF)
if (BUILD_TESTING)
    # create a new list called SOURCES that appends src/main.cpp to the ADA_CPP_SOURCES list
    #set(SOURCES src/main.cpp ${ADA_CPP_SOURCES})
    set(SOURCES src/main.cpp src/cadit/occt/step_to_glb.cpp)
    set(HEADERS src/cadit/tinygltf/tinyload.h src/cadit/occt/step_to_glb.h)
    add_executable(STP2GLB ${SOURCES} ${HEADERS})
    target_link_libraries(STP2GLB ${ADA_CPP_LINK_LIBS})
    install(TARGETS STP2GLB DESTINATION .)
    include(tests/cpp/tests.cmake)
endif (BUILD_TESTING)