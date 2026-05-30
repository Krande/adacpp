# link the library files located in %LIBRARY_PREFIX%/lib/ifcparse/IfcParse.lib (on windows as an example)
# to the executable

list(APPEND
        ADA_CPP_LINK_LIBS
        IfcParse
        IfcGeom
        # ifcopenshell >=0.8.5 ships IfcParse.a with a RocksDB-backed cache,
        # so the static archive pulls in rocksdb symbols (e.g.
        # rocksdb::AssociativeMergeOperator). IfcParse/IfcGeom are static, so
        # we must satisfy their transitive deps on our own link line.
        rocksdb
)
