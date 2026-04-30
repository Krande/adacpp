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
nanobind_add_module(_ada_cpp_ext_impl STABLE_ABI ${ADA_CPP_SOURCES} ${ADA_CPP_PY_SOURCES})

# Link libraries to the module
target_link_libraries(_ada_cpp_ext_impl PRIVATE ${ADA_CPP_LINK_LIBS})

# Set Python site-packages directory (can be overridden via -DPYTHON_SITE_PACKAGES)
if(NOT DEFINED PYTHON_SITE_PACKAGES)
    execute_process(
            COMMAND "${Python_EXECUTABLE}" -c "import sysconfig; print(sysconfig.get_path('purelib'))"
            OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE PYTHON_SITE_PACKAGES)
endif()

message(STATUS "Python site-packages: ${PYTHON_SITE_PACKAGES}")

# Install the module to site-packages/adacpp
install(TARGETS _ada_cpp_ext_impl LIBRARY DESTINATION ${PYTHON_SITE_PACKAGES}/adacpp)

# Install the Python package files
install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/adacpp/
        DESTINATION ${PYTHON_SITE_PACKAGES}/adacpp
        FILES_MATCHING PATTERN "*.py")

