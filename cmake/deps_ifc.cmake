# link the library files located in %LIBRARY_PREFIX%/lib/ifcparse/IfcParse.lib (on windows as an example)
# to the executable

list(APPEND
        ADA_CPP_LINK_LIBS
        IfcParse
        IfcGeom
)

# ifcopenshell >=0.8.5 statically embeds rocksdb in libIfcParse.a /
# libIfcGeom.a, because rocksdb's shared library only exposes the C API
# (c.h), not the C++ API (db.h) ifcopenshell uses — see ifcopenshell's
# CMakeLists and https://github.com/facebook/rocksdb/issues/981.
#
# So we must NOT link the dynamic librocksdb.so: that would put a second
# rocksdb instance in the process alongside the one embedded in IfcParse.a,
# causing ODR conflicts and heap corruption (segfault on IfcFile ctor/dtor).
# Link the static librocksdb.a instead so the final extension module holds a
# single, consistent rocksdb. snappy / lz4 / zstd are rocksdb's own
# compression deps that surface once we pull it in statically; they are safe
# to link dynamically.
find_library(ROCKSDB_STATIC_LIB NAMES librocksdb.a rocksdb REQUIRED)
list(APPEND ADA_CPP_LINK_LIBS ${ROCKSDB_STATIC_LIB} snappy lz4 zstd)

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
