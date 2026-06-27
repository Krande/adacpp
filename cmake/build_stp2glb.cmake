# Standalone OCC-free STP2GLB executable. Builds from the native threaded streaming STEP->GLB
# pipeline (header-only reader + neutral geom layer) — NO OCCT, NO nanobind, NO Python. The same
# OCC-free source set as the wasm step-glb target (ngeom tessellate/boolean/meshopt + vendored
# libtess2/meshopt), linking manifold + Threads + the C++ stdlib only.
set(STP2GLB_SOURCES
        src/stp2glb/main.cpp
        src/geom/neutral/ngeom_tessellate.cpp
        src/geom/neutral/ngeom_boolean.cpp
        src/geom/neutral/ngeom_meshopt.cpp
        ${LIBTESS2_SOURCES}
        ${MESHOPT_SOURCES}
)
set(STP2GLB_HEADERS
        src/cad/step_to_glb_stream.h
)

add_executable(STP2GLB ${STP2GLB_SOURCES} ${STP2GLB_HEADERS})

set_target_properties(STP2GLB PROPERTIES
        INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/bin"
)
# OCC-free: link only manifold (mesh-boolean for the libtess2 path), pthreads, and the C++ stdlib.
# Deliberately NOT ${ADA_CPP_LINK_LIBS} (which pulls in OCCT) and NOT the src/stp2glb/core/* OCCT path.
target_link_libraries(STP2GLB PRIVATE manifold Threads::Threads)
# libtess2 / meshopt include dirs are already on the global include path (top-level CMakeLists);
# CLI11 (CLI/CLI.hpp) comes from the conda/pixi env include dir.

# install to executable into the bin dir
# If EXE_BIN_DIR is not set, use root bin dir
if (NOT DEFINED STP2GLB_BIN_DIR)
    message(STATUS "STP2GLB_BIN_DIR not set, using default")
    set(CONDA_PREFIX $ENV{CONDA_PREFIX})
    message(STATUS "CONDA_PREFIX: ${CONDA_PREFIX}")
    if (DEFINED CONDA_PREFIX)
        if (WIN32)
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

if (BUILD_STP2GLB_TESTING)
    message(STATUS "Building the testing tree.")
    set(CMAKE_INSTALL_RPATH "${STP2GLB_BIN_DIR}")
    set(CMAKE_BUILD_RPATH "${STP2GLB_BIN_DIR}")
    include(tests/cpp/stp2glb_tests.cmake)
endif (BUILD_STP2GLB_TESTING)
