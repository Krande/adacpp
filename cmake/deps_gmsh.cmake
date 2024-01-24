find_library(GMSH_LIB gmsh)
if(NOT GMSH_LIB)
    message(FATAL_ERROR "Could not find libgmsh")
endif()

find_path(GMSH_INC gmsh.h)
if(NOT GMSH_INC)
    message(FATAL_ERROR "Could not find gmsh.h")
endif()

include_directories(${GMSH_INC})

list(APPEND
        ADA_CPP_LINK_LIBS
        gmsh)