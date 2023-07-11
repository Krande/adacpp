# Detect the installed nanobind package and import it into CMake
execute_process(
        COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
        OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE NB_DIR)

message(STATUS "NanoBind Cmake directory: " ${NB_DIR})
list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")

# Import nanobind through CMake's find_package mechanism
find_package(nanobind CONFIG REQUIRED)

# Print the list of cpp files separated by spaces without altering it
string(REPLACE ";" " " ADA_CPP_SOURCES_STR "${ADA_CPP_SOURCES}")
message(STATUS "AdaCpp sources: " ${ADA_CPP_SOURCES_STR})

# Create a Python module
nanobind_add_module(_ada_cpp_ext_impl ${ADA_CPP_SOURCES})