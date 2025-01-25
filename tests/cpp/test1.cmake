# add file located next to this cmake file

set(SOURCES
        ${ADA_CPP_SOURCES}
        tests/cpp/test1.cpp
)

add_executable(TestInstantiator ${SOURCES} ${ADA_CPP_HEADERS})

set_target_properties(TestInstantiator PROPERTIES
        INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/bin"
)

target_link_libraries(TestInstantiator PRIVATE ${ADA_CPP_LINK_LIBS})

install(TARGETS TestInstantiator DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

add_test(NAME TestInstantiator1 COMMAND TestInstantiator
        WORKING_DIRECTORY "${CMAKE_INSTALL_PREFIX}/bin"
)