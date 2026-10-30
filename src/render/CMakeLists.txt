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

add_library(adacpp_render_core STATIC renderer.cpp ${MESHOPT_SOURCES})
target_include_directories(
    adacpp_render_core
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${MESHOPT_DIR}
)

if(EMSCRIPTEN)
    target_compile_options(adacpp_render_core PUBLIC --use-port=emdawnwebgpu)
    add_executable(adacpp_render wasm_bindings.cpp)
    target_link_libraries(adacpp_render PRIVATE adacpp_render_core)
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

    # Headless hosts: GLB loading and the CPU-drawn overlay (gizmo, legend; vendored stb_truetype).
    add_library(
        adacpp_render_headless
        STATIC
        native_device.cpp
        glb_scene.cpp
        overlay.cpp
        overlay_json.cpp
    )
    target_include_directories(adacpp_render_headless PUBLIC ${ADACPP_ROOT}/third_party/stb)
    target_link_libraries(adacpp_render_headless PUBLIC adacpp_render_core)

    add_executable(adacpp_render cli_main.cpp)
    target_link_libraries(adacpp_render PRIVATE adacpp_render_headless)
    if(UNIX AND NOT APPLE)
        target_link_libraries(adacpp_render PRIVATE dl pthread m)
        set_target_properties(
            adacpp_render
            PROPERTIES BUILD_RPATH "$ENV{CONDA_PREFIX}/lib"
        )
    endif()

    # adacpp._render, the Python module (nanobind). Its own small extension, so headless rendering does not
    # pull in the OCCT-based _ada_cpp_ext_impl.
    option(ADACPP_RENDER_PYTHON "Build the adacpp._render Python module" OFF)
    set(ADACPP_RENDER_PY_INSTALL_DIR "adacpp" CACHE STRING "Install destination of the _render module")
    if(ADACPP_RENDER_PYTHON)
        set_target_properties(
            adacpp_render_core
            adacpp_render_headless
            PROPERTIES POSITION_INDEPENDENT_CODE ON
        )
        find_package(Python 3.12 REQUIRED COMPONENTS Interpreter Development.Module)
        execute_process(
            COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
            OUTPUT_STRIP_TRAILING_WHITESPACE
            OUTPUT_VARIABLE nanobind_ROOT
        )
        find_package(nanobind CONFIG REQUIRED)
        nanobind_add_module(_render NB_STATIC py_bindings.cpp)
        target_link_libraries(_render PRIVATE adacpp_render_headless)
        install(TARGETS _render LIBRARY DESTINATION ${ADACPP_RENDER_PY_INSTALL_DIR})
    endif()
endif()
