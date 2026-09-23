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
