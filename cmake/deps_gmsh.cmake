message(STATUS "Finding Gmsh library and include path")
if(WIN32)
    find_library(GMSH_LIB NAMES gmsh.dll.lib gmsh.lib gmsh)
else()
    find_library(GMSH_LIB gmsh)
endif()

if(NOT GMSH_LIB)
    message(FATAL_ERROR "Could not find libgmsh")
endif()

message(STATUS "Found Gmsh library: ${GMSH_LIB}")

find_path(GMSH_INC gmsh.h)
if(NOT GMSH_INC)
    message(FATAL_ERROR "Could not find gmsh.h")
endif()

include_directories(${GMSH_INC})

list(APPEND
        ADA_CPP_LINK_LIBS
        ${GMSH_LIB})