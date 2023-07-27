cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

# TODO: Gmsh should create a cmake file when installing the lib, so all the
# external dependencies can be obtained automatically

# if CMAKE_BUILD_TYPE is specified use it; otherwise set the default
# build type to "RelWithDebInfo" ("-O2 -g" with gcc) prior to calling
# project()
if(DEFINED CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose build type")
else()
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose build type")
endif()

project(api_examples CXX C)

set(CMAKE_CXX_STANDARD 11)

find_library(GMSH_LIB gmsh)
if(NOT GMSH_LIB)
    message(FATAL_ERROR "Could not find libgmsh")
endif()

find_path(GMSH_INC gmsh.h)
if(NOT GMSH_INC)
    message(FATAL_ERROR "Could not find gmsh.h")
endif()

if(GMSH_LIB MATCHES ".a") # FIXME - generalize this
    find_library(BLAS_LIB blas)
    if(BLAS_LIB)
        list(APPEND EXTRA_LIBS ${BLAS_LIB})
    endif()
    find_library(LAPACK_LIB lapack)
    if(LAPACK_LIB)
        list(APPEND EXTRA_LIBS ${LAPACK_LIB})
    endif()
endif()

if(WIN32 OR CYGWIN)
    list(APPEND EXTRA_LIBS winmm wsock32 ws2_32 psapi)
endif()

include_directories(${GMSH_INC})

include(CTest)

file(GLOB EXAMPLES *.c)
foreach(EXAMPLE ${EXAMPLES})
    get_filename_component(EXAMPLENAME ${EXAMPLE} NAME_WE)
    add_executable(${EXAMPLENAME}c ${EXAMPLE})
    target_link_libraries(${EXAMPLENAME}c ${GMSH_LIB} ${EXTRA_LIBS})
    add_test(${EXAMPLENAME}_c ${EXAMPLENAME}c -nopopup)
endforeach()
