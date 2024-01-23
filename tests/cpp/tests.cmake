add_test(NAME odbdump_cli_basic COMMAND STP2GLB
        --stp ${CMAKE_CURRENT_SOURCE_DIR}/basic.stp
        --glb ${CMAKE_CURRENT_BINARY_DIR}/basic.glb
)