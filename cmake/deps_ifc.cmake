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
