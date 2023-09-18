find_library(IfcOpenShell_LIB ifcopenshell)
if(NOT IfcOpenShell_LIB)
    message(FATAL_ERROR "Could not find libgmsh")
endif()

find_path(IfcOpenShell_INC ifcopenshell.h)
if(NOT IfcOpenShell_INC)
    message(FATAL_ERROR "Could not find ifcopenshell.h")
endif()

include_directories(${IfcOpenShell_INC})

list(APPEND
        ADA_CPP_LINK_LIBS
        ifcopenshell)