# Find OpenCASCADE
find_package(OpenCASCADE REQUIRED)
if (OpenCASCADE_FOUND)
    message(STATUS "OpenCASCADE version found: " ${OpenCASCADE_MAJOR_VERSION} ".." ${OpenCASCADE_MINOR_VERSION} ".." ${OpenCASCADE_MAINTENANCE_VERSION})
    message(STATUS "OpenCASCADE include directory: " ${OpenCASCADE_INCLUDE_DIR})
    message(STATUS "OpenCASCADE binary directory: " ${OpenCASCADE_BINARY_DIR})
    message(STATUS "OpenCASCADE library directory: " ${OpenCASCADE_LIBRARY_DIR})

    include_directories(${OpenCASCADE_INCLUDE_DIR})
    link_directories(${OpenCASCADE_LIBRARY_DIR})

    # Remove TKIVtk from the list of OpenCASCADE libraries (if it exists) because it is causing issues with VTK
    list(REMOVE_ITEM OpenCASCADE_LIBRARIES TKIVtk)

    list(APPEND
            ADA_CPP_LINK_LIBS
            ${OpenCASCADE_LIBRARIES}
    )
endif (OpenCASCADE_FOUND)