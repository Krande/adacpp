add_test(NAME stp_glb_cli_basic COMMAND STP2GLB
        --stp ${CMAKE_CURRENT_SOURCE_DIR}/files/basic.stp
        --glb ${CMAKE_CURRENT_BINARY_DIR}/temp/basic.glb
)