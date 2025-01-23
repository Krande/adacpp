# Install the C++ executable
set(SOURCES src/main.cpp src/cadit/occt/step_to_glb.cpp)
set(HEADERS src/cadit/occt/step_to_glb.h)

add_executable(STP2GLB ${SOURCES} ${HEADERS})

set_target_properties(STP2GLB PROPERTIES
        INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/bin"
)
target_link_libraries(STP2GLB ${ADA_CPP_LINK_LIBS})

# install to executable into the bin dir
# If part of EXE_BIN_DIR contains project name, use different install dir
message(STATUS "Installing executable to ${EXE_BIN_DIR}")
install(TARGETS STP2GLB DESTINATION ${EXE_BIN_DIR})

if (BUILD_TESTING)
    message(STATUS "Building the testing tree.")
    set(CMAKE_INSTALL_RPATH "${EXE_BIN_DIR}")
    set(CMAKE_BUILD_RPATH "${EXE_BIN_DIR}")
    include(tests/cpp/tests.cmake)
endif (BUILD_TESTING)