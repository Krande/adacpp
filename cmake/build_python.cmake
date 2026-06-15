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
    set(Python_SOABI   "cpython-313-wasm32-emscripten")
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

# Wasm builds: statically link the OCCT toolkits cross-built by wasm_occt.cmake.
# The OCCT IMPORTED targets carry their include dir as INTERFACE_INCLUDE_DIRECTORIES,
# so adding them here also makes <BRepPrimAPI_MakeBox.hxx> & friends resolve.
#
# Why --whole-archive (not just --start-group): nanobind-config.cmake forces
# `-Wl,--gc-sections` for Release/RelWithDebInfo builds. That post-link
# section-GC strips OCCT object files that no nanobind-emitted code directly
# references, including the static initializers that register OCCT types and
# the vtables those types rely on. Result is "getWasmTableEntry(...) is not
# a function" at module load — a vtable slot pointing at a discarded function.
# --whole-archive forces every .o in each OCCT toolkit to be linked, defeating
# both --gc-sections and any first-pass-undefined-symbol issue. Cost is wheel
# size (a few MB more); benefit is the .so actually loading.
#
# CMake registers WHOLE_ARCHIVE for many platforms but not the emscripten
# wasm32 target — define it ourselves.
if (EMSCRIPTEN AND DEFINED WASM_OCCT_TARGETS)
    set(CMAKE_C_LINK_GROUP_USING_RESCAN "LINKER:--start-group" "LINKER:--end-group")
    set(CMAKE_C_LINK_GROUP_USING_RESCAN_SUPPORTED TRUE)
    set(CMAKE_CXX_LINK_GROUP_USING_RESCAN "LINKER:--start-group" "LINKER:--end-group")
    set(CMAKE_CXX_LINK_GROUP_USING_RESCAN_SUPPORTED TRUE)
    target_link_libraries(_ada_cpp_ext_impl PRIVATE
        "$<LINK_GROUP:RESCAN,${WASM_OCCT_TARGETS}>"
    )
endif ()

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
            "-fwasm-exceptions"
            "-sSUPPORT_LONGJMP=wasm"
            "-Wl,--export=PyInit__ada_cpp_ext_impl"
            "-Wl,--no-gc-sections"
            # SIDE_MODULE=2 + nanobind's default `-Wl,--gc-sections` (added at
            # Release build) end up dropping OCCT vtable entries whose only
            # callers are intra-module — pyodide can't see those callers
            # because side-module exports go through env imports it then
            # stubs. EXPORT_ALL=1 keeps every symbol in the export table so
            # pyodide's self-binding pass resolves intra-module references
            # to real functions instead of stubs. Without this you get
            # `getWasmTableEntry(...) is not a function` at module load.
            "-sEXPORT_ALL=1"
    )
    # The matching compile flag is required so try/catch in our code (and in
    # nanobind's dispatch) actually emit unwind tables. pyodide 0.29.x/emscripten
    # 4.0.9 use native WebAssembly exception handling (-fwasm-exceptions); every
    # translation unit in the link (adacpp, OCCT, nanobind) must agree on the EH
    # model or thrown exceptions reach std::terminate instead of the catch.
    target_compile_options(_ada_cpp_ext_impl PRIVATE "-fwasm-exceptions")
    # nanobind's static lib (nanobind-static-abi3) holds the dispatch code that
    # actually converts C++ exceptions into Python ones — without -fwasm-exceptions
    # there it'll pass them through as fatal aborts. Apply the same flag.
    if (TARGET nanobind-static-abi3)
        target_compile_options(nanobind-static-abi3 PRIVATE "-fwasm-exceptions")
    endif ()
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

