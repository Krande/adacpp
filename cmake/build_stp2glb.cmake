# Get .h files from src/stp2glb/core
file(GLOB_RECURSE CORE_HEADERS src/stp2glb/core/*.h)

# Get .cpp files from src/stp2glb/core
file(GLOB_RECURSE CORE_SOURCES src/stp2glb/core/*.cpp)

# Install the C++ executable
set(SOURCES
        src/geom/Color.cpp
        src/stp2glb/main.cpp
        src/stp2glb/config_utils.cpp
        ${CORE_SOURCES}

)
set(HEADERS
        src/geom/Color.h
        src/stp2glb/config_structs.h
        src/stp2glb/config_utils.h
        ${CORE_HEADERS}
)

add_executable(STP2GLB ${SOURCES} ${HEADERS})

set_target_properties(STP2GLB PROPERTIES
        INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/bin"
)
target_link_libraries(STP2GLB ${ADA_CPP_LINK_LIBS})

# install to executable into the bin dir
# If EXE_BIN_DIR is not set, use root bin dir
if (NOT DEFINED STP2GLB_BIN_DIR)
    message(STATUS "STP2GLB_BIN_DIR not set, using default")
    set(CONDA_PREFIX $ENV{CONDA_PREFIX})
    message(STATUS "CONDA_PREFIX: ${CONDA_PREFIX}")
    if (DEFINED CONDA_PREFIX)
        if (WIN)
        set(STP2GLB_BIN_DIR "${CONDA_PREFIX}/Library/bin")
        else()
            set(STP2GLB_BIN_DIR "${CONDA_PREFIX}/bin")
        endif ()
    else ()
        set(STP2GLB_BIN_DIR ${CMAKE_INSTALL_PREFIX}/bin)
    endif ()
endif ()

message(STATUS "Installing executable to ${STP2GLB_BIN_DIR}")
install(TARGETS STP2GLB DESTINATION ${STP2GLB_BIN_DIR})

if (BUILD_TESTING)
    message(STATUS "Building the testing tree.")
    set(CMAKE_INSTALL_RPATH "${STP2GLB_BIN_DIR}")
    set(CMAKE_BUILD_RPATH "${STP2GLB_BIN_DIR}")
    include(tests/cpp/stp2glb_tests.cmake)
endif (BUILD_TESTING)