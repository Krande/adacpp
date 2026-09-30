# adacpp builds against both ifcopenshell lines while 0.9 is new. The C++ sources pick their
# API through src/cadit/ifc/ifcopenshell_version.h (keyed on the header layout); this file
# picks the link recipe:
#
# * 0.9+ ships shared libraries plus a CMake package (lib/cmake/IfcOpenShell). The imported
#   targets carry the include dir, the compile definitions that change class layouts
#   (IFOPSH_WITH_ROCKSDB on IfcParse, IFOPSH_WITH_CGAL / IFOPSH_WITH_OPENCASCADE on IfcGeom) and
#   the transitive deps (Boost, Eigen, RocksDB, CGAL, OCCT). adacpp and the ifcopenshell Python
#   module then share ONE copy of IfcParse/IfcGeom in the process. adacpp calls the OpenCASCADE
#   and CGAL kernels directly (src/cadit/ifc/ngeom_taxonomy_kernel.cpp); in 0.9 they are plugin
#   libraries IfcGeom normally loads at runtime, but still ordinary shared libraries, so link them.
# * 0.8 ships static libIfcParse.a / libIfcGeom.a (rocksdb embedded) and no CMake package, so it
#   is linked by hand below.
#
# No version argument to find_package: the package's version file only accepts an exact match.
find_package(IfcOpenShell CONFIG QUIET)
if(IfcOpenShell_FOUND AND NOT IfcOpenShell_VERSION VERSION_LESS 0.9)
    message(
        STATUS
        "ifcopenshell ${IfcOpenShell_VERSION}: linking its shared libraries"
    )
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
else()
    message(
        STATUS
        "ifcopenshell 0.8 (no CMake package): linking the static IfcParse/IfcGeom"
    )
    # The geometry kernels (OpenCascadeKernel / CgalKernel) live in their own static libs,
    # separate from libIfcGeom.a (which defines AbstractKernel + the taxonomy + convert dispatch).
    # The NGEOM taxonomy path instantiates these kernels, so link them too. Static-lib symbol
    # resolution between the kernels and IfcGeom is mutual.
    if(APPLE)
        # Apple's ld64 has no --start-group/--end-group (it errors "ld: unknown option:
        # --start-group"); it resolves circular static-archive deps via its own multi-pass,
        # so just list the libs directly.
        list(
            APPEND ADA_CPP_LINK_LIBS
            geometry_kernel_opencascade
            geometry_kernel_cgal
            IfcParse
            IfcGeom
        )
    else()
        # GNU ld / lld need the group to resolve the mutual references in one pass.
        list(
            APPEND ADA_CPP_LINK_LIBS
            -Wl,--start-group
            geometry_kernel_opencascade
            geometry_kernel_cgal
            IfcParse
            IfcGeom
            -Wl,--end-group
        )
    endif()
    # The CGAL kernel uses CGAL's exact arithmetic -> GMP/MPFR. They are leaf C libs, so they
    # must come AFTER the kernel group in static link order.
    list(APPEND ADA_CPP_LINK_LIBS mpfr gmp)

    # ifcopenshell >=0.8.5 statically embeds rocksdb in libIfcParse.a /
    # libIfcGeom.a, because rocksdb's shared library only exposes the C API
    # (c.h), not the C++ API (db.h) ifcopenshell uses — see ifcopenshell's
    # CMakeLists and https://github.com/facebook/rocksdb/issues/981.
    #
    # So we must NOT link the dynamic librocksdb.so: that would put a second
    # rocksdb instance in the process alongside the one embedded in IfcParse.a,
    # causing ODR conflicts and heap corruption (segfault on IfcFile ctor/dtor).
    # Link the static rocksdb instead so the final extension module holds a
    # single, consistent rocksdb.
    #
    # Use rocksdb's own CMake package rather than a hard-coded find_library: the
    # RocksDB::rocksdb target is the STATIC library on every platform (the .a/.lib
    # names differ — e.g. Windows has no librocksdb.a and `rocksdb` resolves to the
    # shared import lib, which is why find_library failed there), and it already
    # carries rocksdb's compression deps (snappy / lz4 / zstd / zlib / gflags) as
    # interface link libraries, so we don't have to enumerate them by hand.
    find_package(RocksDB CONFIG REQUIRED)
    list(APPEND ADA_CPP_LINK_LIBS RocksDB::rocksdb)

    # The conda-forge ifcopenshell is built with -DWITH_ROCKSDB=ON, which defines
    # IFOPSH_WITH_ROCKSDB while compiling libIfcParse.a. That macro gates members
    # of class IfcParse::IfcFile (the rocksdb-backed storage), so it changes the
    # class layout: sizeof(IfcFile) is 896 bytes when built with the macro vs 688
    # without it. ifcopenshell normally propagates the macro to consumers via the
    # IFCOPENSHELL_RocksDB INTERFACE target, but we bare-link IfcParse/IfcGeom by
    # name and so never inherit it. Without the macro our translation units see the
    # 688-byte layout while the linked .a uses the 896-byte one — the IfcFile ctor
    # writes 208 bytes past our stack allocation and at shifted member offsets,
    # corrupting the object (e.g. the file-path std::string is intact entering the
    # ctor but empty by the time initialize() reaches FileReader, which then
    # fopen("")s and segfaults). Define it here to match the .a's layout. rocksdb
    # headers are on the conda -isystem include path, so the #include resolves.
    add_compile_definitions(IFOPSH_WITH_ROCKSDB)

    # ifcgeom/taxonomy.h includes <Eigen/Dense>, which conda-forge installs under
    # <prefix>/include/eigen3 (not directly on the -isystem include path). The NGEOM->taxonomy
    # adapter (src/cadit/ifc/ngeom_taxonomy_*.cpp) is the first adacpp TU to include taxonomy.h,
    # so add Eigen's include dir explicitly.
    find_package(Eigen3 CONFIG REQUIRED)
    # ifcgeom/taxonomy.h includes <Eigen/Dense> (conda installs Eigen under include/eigen3, off
    # the default -isystem path). Link the header-only Eigen3::Eigen target so its INTERFACE
    # include dir propagates to the NGEOM taxonomy TUs — the first adacpp code to include taxonomy.h.
    list(APPEND ADA_CPP_LINK_LIBS Eigen3::Eigen)
endif()
