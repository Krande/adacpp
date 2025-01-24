# add file located next to this cmake file

add_executable(TestInstantiator
        src/cadit/tinygltf/tinygltf.cpp
        src/cadit/tinygltf/tiny_helpers.cpp
        tests/cpp/test1.cpp
        src/cadit/tinygltf/tiny_helpers.h
)
set_target_properties(TestInstantiator PROPERTIES
        INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/bin"
)
target_link_libraries(TestInstantiator ${ADA_CPP_LINK_LIBS})

install(TARGETS TestInstantiator DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

add_test(NAME TestInstantiator
        COMMAND TestInstantiator)