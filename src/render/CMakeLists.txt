# adacpp WebGPU renderer — a standalone project so it builds without the OCCT/pyodide machinery of the
# root CMakeLists:
#   native : cmake -S src/render -B build-render        (pixi -e render build-render; wgpu-native)
#   browser: emcmake cmake -S src/render -B build-...   (tools/build_render_wasm.sh; emdawnwebgpu)
cmake_minimum_required(VERSION 3.24)
project(adacpp_render LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(ADACPP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(MESHOPT_DIR ${ADACPP_ROOT}/third_party/meshoptimizer/src)
file(GLOB MESHOPT_SOURCES ${MESHOPT_DIR}/*.cpp)

add_library(
    adacpp_render_core
    STATIC
    renderer.cpp
    camera_controller.cpp
    ${MESHOPT_SOURCES}
)
target_include_directories(
    adacpp_render_core
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${MESHOPT_DIR}
)

# The GLB reader (glb_scene.h): the same sources in the browser and the headless hosts.
set(ADACPP_RENDER_GLB_SOURCES
    glb_scene.cpp
    glb_parse.cpp
    glb_math.cpp
    glb_objects.cpp
    glb_animation.cpp
    glb_batcher.cpp
)

# KHR_draco_mesh_compression in the GLB reader. OFF builds a reader that refuses Draco-compressed GLBs.
option(
    ADACPP_RENDER_DRACO
    "Decode Draco-compressed meshes in the GLB reader"
    ON
)

if(EMSCRIPTEN)
    target_compile_options(adacpp_render_core PUBLIC --use-port=emdawnwebgpu)
    # The GLB reader the Python offscreen renderer uses, so the browser reads models the same way.
    # tinygltf (+ json.hpp) is vendored for this build; the native one takes conda's.
    add_executable(adacpp_render wasm_bindings.cpp ${ADACPP_RENDER_GLB_SOURCES})
    target_include_directories(
        adacpp_render
        PRIVATE ${ADACPP_ROOT}/third_party/tinygltf
    )
    target_link_libraries(adacpp_render PRIVATE adacpp_render_core)
    if(ADACPP_RENDER_DRACO)
        # No Draco package exists for this toolchain: its decoder is compiled from the release's sources,
        # fetched at configure time (or taken from ADACPP_DRACO_SOURCE_DIR, an unpacked copy of the same
        # release, for a build without network). Keep the version in step with the conda one (pixi.toml).
        set(ADACPP_DRACO_SOURCE_DIR
            ""
            CACHE PATH
            "An unpacked Draco source tree to build the decoder from, instead of downloading it"
        )
        if(ADACPP_DRACO_SOURCE_DIR)
            set(draco_SOURCE_DIR ${ADACPP_DRACO_SOURCE_DIR})
        else()
            include(FetchContent)
            FetchContent_Declare(
                draco
                URL
                    https://github.com/google/draco/archive/refs/tags/1.5.9.tar.gz
                URL_HASH
                    SHA256=5b84e0e212ccc7b03ace681760a65a66fe2481a5f54df65154cfc8f0cbb87148
                # Sources only: Draco's own CMake project (encoder, tools, JavaScript glue) is not configured.
                SOURCE_SUBDIR
                cmake-not-used
            )
            FetchContent_MakeAvailable(draco)
        endif()
        set(_draco_src ${draco_SOURCE_DIR}/src/draco)
        file(
            GLOB_RECURSE DRACO_SOURCES
            ${_draco_src}/attributes/*.cc
            ${_draco_src}/compression/*.cc
            ${_draco_src}/core/*.cc
            ${_draco_src}/mesh/*.cc
            ${_draco_src}/metadata/*.cc
            ${_draco_src}/point_cloud/*.cc
        )
        # The decoder alone: no tests, no encoders.
        list(
            FILTER DRACO_SOURCES
            EXCLUDE
            REGEX "(_test\\.cc|test_utils\\.cc|encod[^/]*\\.cc)$"
        )
        # What Draco's CMake would generate.
        set(_draco_gen ${CMAKE_CURRENT_BINARY_DIR}/draco_generated)
        file(
            WRITE ${_draco_gen}/draco/draco_features.h
            "#ifndef DRACO_FEATURES_H_
#define DRACO_FEATURES_H_
#define DRACO_POINT_CLOUD_COMPRESSION_SUPPORTED
#define DRACO_MESH_COMPRESSION_SUPPORTED
#define DRACO_NORMAL_ENCODING_SUPPORTED
#define DRACO_STANDARD_EDGEBREAKER_SUPPORTED
#define DRACO_PREDICTIVE_EDGEBREAKER_SUPPORTED
#define DRACO_BACKWARDS_COMPATIBILITY_SUPPORTED
#endif
"
        )
        add_library(adacpp_draco STATIC ${DRACO_SOURCES})
        target_include_directories(
            adacpp_draco
            PUBLIC ${draco_SOURCE_DIR}/src ${_draco_gen}
        )
        target_compile_options(adacpp_draco PRIVATE -w)
        target_link_libraries(adacpp_render PRIVATE adacpp_draco)
        target_compile_definitions(adacpp_render PRIVATE ADACPP_RENDER_DRACO)
    endif()
    target_link_options(
        adacpp_render
        PRIVATE
            --use-port=emdawnwebgpu
            -lembind
            -sMODULARIZE=1
            -sEXPORT_ES6=1
            -sEXPORT_NAME=createAdacppRender
            -sENVIRONMENT=web,worker
            -sALLOW_MEMORY_GROWTH=1
            -sMAXIMUM_MEMORY=4GB
            -sEXPORTED_RUNTIME_METHODS=HEAPU8,HEAPU32,HEAPF32
            -sEXPORTED_FUNCTIONS=_malloc,_free
            -sDISABLE_EXCEPTION_CATCHING=0
    )
    set_target_properties(adacpp_render PROPERTIES SUFFIX ".js")
else()
    find_path(WEBGPU_INCLUDE_DIR webgpu.h REQUIRED)
    find_library(WGPU_NATIVE_LIBRARY NAMES wgpu_native REQUIRED)
    target_include_directories(adacpp_render_core PUBLIC ${WEBGPU_INCLUDE_DIR})
    target_link_libraries(adacpp_render_core PUBLIC ${WGPU_NATIVE_LIBRARY})
    if(WIN32)
        # conda-forge's wgpu_native.lib is the static Rust library (no import library for the DLL),
        # so the Windows system libraries Rust's std and wgpu-hal call into come along, as in
        # wgpu-native's own examples.
        target_link_libraries(
            adacpp_render_core
            PUBLIC
                d3dcompiler
                ws2_32
                userenv
                bcrypt
                ntdll
                opengl32
                propsys
                runtimeobject
        )
    endif()

    # Headless hosts: GLB loading and the CPU-drawn overlay (gizmo, legend; vendored stb_truetype).
    add_library(
        adacpp_render_headless
        STATIC
        native_device.cpp
        ${ADACPP_RENDER_GLB_SOURCES}
        overlay.cpp
        overlay_json.cpp
    )
    # tinygltf (with its json.hpp) is a noarch conda package: it installs to <prefix>/include on
    # every platform, which on Windows is not Library/include, where webgpu.h and the prefix path are.
    find_path(
        TINYGLTF_INCLUDE_DIR
        tiny_gltf.h
        HINTS
        ENV PREFIX
        ENV CONDA_PREFIX
        PATH_SUFFIXES include
        REQUIRED
    )
    target_include_directories(
        adacpp_render_headless
        PUBLIC ${ADACPP_ROOT}/third_party/stb ${TINYGLTF_INCLUDE_DIR}
    )
    target_link_libraries(adacpp_render_headless PUBLIC adacpp_render_core)
    if(ADACPP_RENDER_DRACO)
        # conda-forge's draco. Its static library where the package has one, so the Python module and the
        # CLI carry the decoder and need no libdraco at run time.
        find_package(draco CONFIG REQUIRED)
        if(TARGET draco::draco_static)
            target_link_libraries(
                adacpp_render_headless
                PUBLIC draco::draco_static
            )
        else()
            target_link_libraries(adacpp_render_headless PUBLIC draco::draco)
        endif()
        target_compile_definitions(
            adacpp_render_headless
            PRIVATE ADACPP_RENDER_DRACO
        )
    endif()

    add_executable(adacpp_render cli_main.cpp)
    target_link_libraries(adacpp_render PRIVATE adacpp_render_headless)
    if(UNIX AND NOT APPLE)
        target_link_libraries(adacpp_render PRIVATE dl pthread m)
        set_target_properties(
            adacpp_render
            PROPERTIES BUILD_RPATH "$ENV{CONDA_PREFIX}/lib"
        )
    endif()

    # The camera controller has no GPU in it, so its test runs anywhere: ctest --test-dir <build dir>
    # (pixi run -e render test-render). Skipped where the tests are not shipped with the sources.
    option(ADACPP_RENDER_TESTS "Build the renderer's tests" ON)
    set(ADACPP_RENDER_CAMERA_TEST
        ${ADACPP_ROOT}/tests/cpp/test_camera_controller.cpp
    )
    if(ADACPP_RENDER_TESTS AND EXISTS ${ADACPP_RENDER_CAMERA_TEST})
        enable_testing()
        add_executable(
            adacpp_render_camera_test
            ${ADACPP_RENDER_CAMERA_TEST}
            camera_controller.cpp
        )
        target_include_directories(
            adacpp_render_camera_test
            PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
        )
        add_test(NAME camera_controller COMMAND adacpp_render_camera_test)
    endif()

    # What needs a device to check (GLB animations, Draco, textured quads) draws offscreen and reads the
    # image back; the test skips itself (exit code 77) on a machine with no adapter.
    set(ADACPP_RENDER_SCENE_TEST ${ADACPP_ROOT}/tests/cpp/test_render_scene.cpp)
    if(ADACPP_RENDER_TESTS AND EXISTS ${ADACPP_RENDER_SCENE_TEST})
        enable_testing()
        add_executable(adacpp_render_scene_test ${ADACPP_RENDER_SCENE_TEST})
        target_link_libraries(
            adacpp_render_scene_test
            PRIVATE adacpp_render_headless
        )
        if(ADACPP_RENDER_DRACO)
            target_compile_definitions(
                adacpp_render_scene_test
                PRIVATE ADACPP_RENDER_DRACO
            )
        endif()
        if(UNIX AND NOT APPLE)
            target_link_libraries(adacpp_render_scene_test PRIVATE dl pthread m)
            set_target_properties(
                adacpp_render_scene_test
                PROPERTIES BUILD_RPATH "$ENV{CONDA_PREFIX}/lib"
            )
        endif()
        add_test(NAME render_scene COMMAND adacpp_render_scene_test)
        set_tests_properties(render_scene PROPERTIES SKIP_RETURN_CODE 77)
    endif()

    # adacpp._render, the Python module (nanobind). Its own small extension, so headless rendering does not
    # pull in the OCCT-based _ada_cpp_ext_impl.
    option(ADACPP_RENDER_PYTHON "Build the adacpp._render Python module" OFF)
    set(ADACPP_RENDER_PY_INSTALL_DIR
        "adacpp"
        CACHE STRING
        "Install destination of the _render module"
    )
    if(ADACPP_RENDER_PYTHON)
        set_target_properties(
            adacpp_render_core
            adacpp_render_headless
            PROPERTIES POSITION_INDEPENDENT_CODE ON
        )
        find_package(
            Python
            3.12
            REQUIRED
            COMPONENTS Interpreter Development.Module
        )
        execute_process(
            COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
            OUTPUT_STRIP_TRAILING_WHITESPACE
            OUTPUT_VARIABLE nanobind_ROOT
        )
        find_package(nanobind CONFIG REQUIRED)
        nanobind_add_module(_render NB_STATIC py_bindings.cpp)
        target_link_libraries(_render PRIVATE adacpp_render_headless)
        if(APPLE)
            # conda-forge's libwgpu_native.dylib carries its build directory as its install name, which
            # the linker copies into _render; dlopen then looks there. Point the dependency at @rpath,
            # and the rpath at the env's lib/ (site-packages/adacpp -> ../../..).
            execute_process(
                COMMAND otool -D "${WGPU_NATIVE_LIBRARY}"
                OUTPUT_VARIABLE _wgpu_otool
                OUTPUT_STRIP_TRAILING_WHITESPACE
            )
            string(
                REGEX REPLACE
                "^[^\n]*\n"
                ""
                _wgpu_install_name
                "${_wgpu_otool}"
            )
            if(_wgpu_install_name AND NOT _wgpu_install_name MATCHES "^@rpath/")
                add_custom_command(
                    TARGET _render
                    POST_BUILD
                    COMMAND
                        install_name_tool -change "${_wgpu_install_name}"
                        "@rpath/libwgpu_native.dylib" "$<TARGET_FILE:_render>"
                    VERBATIM
                )
            endif()
            set_target_properties(
                _render
                PROPERTIES
                    BUILD_RPATH "@loader_path/../../..;${CMAKE_PREFIX_PATH}/lib"
                    INSTALL_RPATH "@loader_path/../../.."
            )
        endif()
        install(
            TARGETS _render
            LIBRARY DESTINATION ${ADACPP_RENDER_PY_INSTALL_DIR}
        )
    endif()
endif()
