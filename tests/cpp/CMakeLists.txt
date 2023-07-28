include(CTest)
find_package(GTest REQUIRED)

file(GLOB EXAMPLES *.cpp)
foreach(EXAMPLE ${EXAMPLES})
    get_filename_component(EXAMPLENAME ${EXAMPLE} NAME_WE)
    add_executable(${EXAMPLENAME}c ${EXAMPLE})

    target_link_libraries(${EXAMPLENAME}c GTest::GTest GTest::Main ada_cpp_lib ${EXTRA_LIBS})
    add_test(NAME ${EXAMPLENAME}_c COMMAND ${EXAMPLENAME}c)
endforeach()
