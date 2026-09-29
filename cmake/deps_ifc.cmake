# ifcopenshell >= 0.9 ships shared libraries plus a CMake package (lib/cmake/IfcOpenShell).
# The imported targets carry the include dir, the compile definitions that change class layouts
# (IFOPSH_WITH_ROCKSDB on IfcParse, IFOPSH_WITH_CGAL / IFOPSH_WITH_OPENCASCADE on IfcGeom) and
# the transitive deps (Boost, Eigen, RocksDB, CGAL, OCCT), so none of that is spelled out here.
#
# Before 0.9 conda-forge shipped static libIfcParse.a / libIfcGeom.a with rocksdb embedded, and
# this file hand-assembled a --start-group link, a static rocksdb and the IFOPSH_WITH_ROCKSDB
# define to match the archive's IfcFile layout. Linking the shared libraries also means adacpp and
# the ifcopenshell Python module share ONE copy of IfcParse/IfcGeom in the process instead of
# each carrying its own.
#
# adacpp calls the OpenCASCADE and CGAL kernels directly (src/cadit/ifc/ngeom_taxonomy_kernel.cpp).
# In 0.9 they are plugin libraries that IfcGeom normally loads at runtime; they are still ordinary
# shared libraries, so link them like any other.
# No version argument: the package's version file only accepts an exact match.
find_package(IfcOpenShell CONFIG REQUIRED)
if(IfcOpenShell_VERSION VERSION_LESS 0.9)
    message(
        FATAL_ERROR
        "adacpp needs ifcopenshell >= 0.9, found ${IfcOpenShell_VERSION}"
    )
endif()
list(
    APPEND ADA_CPP_LINK_LIBS
    IfcOpenShell::IfcGeom
    IfcOpenShell::geometry_kernel_opencascade
    IfcOpenShell::geometry_kernel_cgal
)

# ifcopenshell 0.9.0's install rule copies ifcgeom/kernels/<kernel>/*.h but not the
# ifcgeom/kernels/ifc_geomlibrary_api.h every one of them includes, so any consumer of a kernel
# header fails with "ifc_geomlibrary_api.h: No such file or directory". Until an ifcopenshell build
# ships it, write the (upstream, verbatim-logic) header into the build tree. The kernel headers
# reach it as "../../../ifcgeom/kernels/ifc_geomlibrary_api.h" and "../ifc_geomlibrary_api.h";
# a quoted include falls back to the -I dirs, so an -I dir one level below shim/ifcgeom/kernels
# resolves both spellings to the one file. Skipped automatically once the real header is present.
get_target_property(
    _ifc_includes
    IfcOpenShell::IfcGeom
    INTERFACE_INCLUDE_DIRECTORIES
)
list(GET _ifc_includes 0 _ifc_include)
if(NOT EXISTS "${_ifc_include}/ifcgeom/kernels/ifc_geomlibrary_api.h")
    set(_ifc_shim
        "${CMAKE_BINARY_DIR}/ifcopenshell_shim/include/ifcgeom/kernels"
    )
    file(MAKE_DIRECTORY "${_ifc_shim}/_")
    file(
        WRITE "${_ifc_shim}/ifc_geomlibrary_api.h"
        "// Written by adacpp's cmake/deps_ifc.cmake: ifcopenshell 0.9.0 does not install this header.\n"
        "#ifndef IFC_GEOMLIBRARY_API_H\n"
        "#define IFC_GEOMLIBRARY_API_H\n"
        "#ifdef SWIG\n"
        "#define IFC_GEOMLIBRARY_API\n"
        "#elif defined(_WIN32)\n"
        "#ifdef IFC_GEOMLIBRARY_EXPORTS\n"
        "#define IFC_GEOMLIBRARY_API __declspec(dllexport)\n"
        "#else\n"
        "#define IFC_GEOMLIBRARY_API __declspec(dllimport)\n"
        "#endif\n"
        "#else\n"
        "#define IFC_GEOMLIBRARY_API __attribute__((visibility(\"default\")))\n"
        "#endif\n"
        "#endif\n"
    )
    include_directories("${_ifc_shim}/_")
    message(
        STATUS
        "ifcopenshell lacks ifcgeom/kernels/ifc_geomlibrary_api.h; using the shim in ${_ifc_shim}"
    )
endif()

# The CGAL kernel uses CGAL's exact arithmetic -> GMP/MPFR.
list(APPEND ADA_CPP_LINK_LIBS mpfr gmp)
