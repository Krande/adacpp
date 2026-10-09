add_test(
    NAME stp_glb_cli_basic
    COMMAND
        STP2GLB
        ${CMAKE_CURRENT_SOURCE_DIR}/files/flat_plate_abaqus_10x10_m_wColors.stp
        ${CMAKE_CURRENT_SOURCE_DIR}/temp/flat_plate_abaqus_10x10_m_wColors.glb
    WORKING_DIRECTORY "${CMAKE_INSTALL_PREFIX}/bin"
)
# --include-guid (IFC->GLB subset streaming). The CLI is the only harness that reaches this path
# without the Python extension, so the two halves of the contract are pinned here: a filter selects
# its matches, and a filter matching nothing FAILS rather than writing an empty GLB.
add_test(
    NAME ifc_glb_cli_include_guid
    COMMAND
        STP2GLB ${CMAKE_CURRENT_SOURCE_DIR}/files/my_test.ifc
        ${CMAKE_CURRENT_SOURCE_DIR}/temp/my_test_subset.glb --include-guid
        1cPZ5wLayHxeFFv5utukF4 --include-guid 1cQMCXLayHxgKdv5utukF4 --quiet
    WORKING_DIRECTORY "${CMAKE_INSTALL_PREFIX}/bin"
)
# --quiet is what prints the compact `solids=N` line; the verbose echo says "Solids written:".
set_tests_properties(
    ifc_glb_cli_include_guid
    PROPERTIES PASS_REGULAR_EXPRESSION "solids=2"
)

add_test(
    NAME ifc_glb_cli_include_guid_no_match
    COMMAND
        STP2GLB ${CMAKE_CURRENT_SOURCE_DIR}/files/my_test.ifc
        ${CMAKE_CURRENT_SOURCE_DIR}/temp/my_test_nomatch.glb --include-guid
        NOTAREALGUID0000000000
    WORKING_DIRECTORY "${CMAKE_INSTALL_PREFIX}/bin"
)
set_tests_properties(
    ifc_glb_cli_include_guid_no_match
    PROPERTIES WILL_FAIL TRUE
)

# The sharded STEP/IFC -> GLB protocol (the browser's N-worker verbs) run natively: N shard objects in
# one process sharing a directory, against a generated model whose big root takes the huge-root face
# split. Natively so the sanitizer builds see the whole protocol; the wasm build runs the same checks
# per mesh in tools/test_glb_shards_wasm.mjs.
find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    add_executable(
        test_glb_shards
        tests/cpp/test_glb_shards.cpp
        src/geom/neutral/ngeom_tessellate.cpp
        src/geom/neutral/ngeom_boolean.cpp
        src/geom/neutral/ngeom_meshopt.cpp
        ${LIBTESS2_SOURCES}
        ${MESHOPT_SOURCES}
    )
    target_link_libraries(test_glb_shards PRIVATE manifold Threads::Threads)
    set(GLB_SHARD_FIXTURES ${CMAKE_CURRENT_BINARY_DIR}/glb_shard_fixtures)
    add_test(
        NAME glb_shards_fixtures
        COMMAND
            ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/tools/gen_faceted_fixtures.py
            ${GLB_SHARD_FIXTURES}
    )
    set_tests_properties(
        glb_shards_fixtures
        PROPERTIES FIXTURES_SETUP glb_shard_models
    )
    foreach(ext stp ifc)
        add_test(
            NAME glb_shards_${ext}
            COMMAND
                test_glb_shards ${GLB_SHARD_FIXTURES}/faceted_huge.${ext}
                ${CMAKE_CURRENT_BINARY_DIR}/glb_shards_${ext} 3
        )
        set_tests_properties(
            glb_shards_${ext}
            PROPERTIES FIXTURES_REQUIRED glb_shard_models
        )
    endforeach()
endif()
