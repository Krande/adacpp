# Detect the installed nanobind package and import it into CMake
execute_process(
        COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
        OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE NB_DIR)

message(STATUS "NanoBind Cmake directory: " ${NB_DIR})
list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")

# Under the emscripten toolchain, find_package is restricted to CMAKE_FIND_ROOT_PATH.
# nanobind lives in the host conda env, not the cross-compile sysroot, so allow find
# to look outside the sysroot for the package config.
if (EMSCRIPTEN)
    set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)

    # Pyodide Python modules are loaded dynamically by the runtime and never link
    # libpython, so find_package(Python COMPONENTS Development) is unavailable.
    # Provide the Python::Module / Python::SABIModule targets nanobind expects, plus
    # Python_SOABI / Python_SOSABI variables that drive the extension suffix.
    if (NOT TARGET Python::Module)
        add_library(Python::Module INTERFACE IMPORTED)
        target_include_directories(Python::Module INTERFACE "${Python_INCLUDE_DIR}")
    endif ()
    if (NOT TARGET Python::SABIModule)
        add_library(Python::SABIModule INTERFACE IMPORTED)
        target_include_directories(Python::SABIModule INTERFACE "${Python_INCLUDE_DIR}")
    endif ()
    set(Python_SOABI   "cpython-312-wasm32-emscripten")
    set(Python_SOSABI  "abi3")

    # The emscripten toolchain pins TARGET_SUPPORTS_SHARED_LIBS=FALSE globally, which
    # silently demotes MODULE targets (what nanobind_add_module produces) to STATIC
    # archives. Pyodide loads .so files that are wasm side modules, so flip the
    # property back on before nanobind_add_module runs.
    set_property(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS TRUE)
endif ()

# Import nanobind through CMake's find_package mechanism
find_package(nanobind CONFIG REQUIRED)

# Print the list of cpp files separated by spaces without altering it
string(REPLACE ";" " " ADA_CPP_SOURCES_STR "${ADA_CPP_SOURCES}")
message(STATUS "AdaCpp sources: " ${ADA_CPP_SOURCES_STR})

# Create a Python module
nanobind_add_module(_ada_cpp_ext_impl STABLE_ABI ${ADA_CPP_SOURCES} ${ADA_CPP_PY_SOURCES})

# Link libraries to the module
target_link_libraries(_ada_cpp_ext_impl PRIVATE ${ADA_CPP_LINK_LIBS})

if (EMSCRIPTEN)
    # Emscripten + cmake's MODULE target produces an ar archive by default. Pyodide
    # loads .so files that are actually relocatable wasm side modules; force that
    # output here. WASM_BIGINT is required by pyodide (BigInt-aware i64 marshalling).
    # PyInit__ada_cpp_ext_impl must be explicitly retained — wasm-ld --gc-sections
    # would otherwise drop it (it's only reached through the Python C API).
    set_target_properties(_ada_cpp_ext_impl PROPERTIES
            SUFFIX ".so"
            PREFIX "")
    target_link_options(_ada_cpp_ext_impl PRIVATE
            "-sSIDE_MODULE=2"
            "-sWASM_BIGINT"
            "-Wl,--export=PyInit__ada_cpp_ext_impl"
            "-Wl,--no-gc-sections"
    )
endif ()

if (EMSCRIPTEN)
    # Stage the wheel layout into ${CMAKE_INSTALL_PREFIX}/wheel/adacpp/. The
    # build_wheel.py script picks it up from there and builds the .whl.
    install(TARGETS _ada_cpp_ext_impl LIBRARY DESTINATION wheel/adacpp)
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/src/adacpp/
            DESTINATION wheel/adacpp
            FILES_MATCHING PATTERN "*.py")
else ()
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
endif ()

