# cmake/wasm_gmsh.cmake
#
# SPIKE (option B, Aibel-Engineering/adapy-plugins#177 step 2d): gmsh 4.15.2 for Pyodide as its own
# side module, libgmsh.so, linked against the SHARED OCCT (libocct.so, wasm_occt.cmake with
# ADACPP_WASM_SHARED_OCCT=ON). It ships in its own `gmsh` wheel next to upstream api/gmsh.py, whose
# only change is where it finds the library. adacpp's extension links the same libocct.so, so a
# TopoDS_Shape* from adacpp is a valid shape for gmsh.model.occ.importShapesNativePointer.
#
# The native build is untouched: the worker keeps conda-forge gmsh / python-gmsh 4.15.2.
#
# gmsh itself is built as its static library (ENABLE_BUILD_LIB) in its own ExternalProject; the
# shared module is linked here, whole-archive, so every GMSH_API symbol (default visibility) is
# exported for ctypes. gmsh's OCC detection is fed adacpp's OCCT install: OCC_INC directly, OCC_LIBS
# as a placeholder (gmsh then counts the list as complete), and the CAF toolkits are found as the
# static archives in that install. Nothing in gmsh's own build links them; only libgmsh.so links,
# against libocct.so.

include(ExternalProject)

if(NOT ADACPP_WASM_SHARED_OCCT)
    message(
        FATAL_ERROR
        "ADACPP_WASM_GMSH needs ADACPP_WASM_SHARED_OCCT=ON (one OCCT for adacpp + gmsh)"
    )
endif()

set(GMSH_VERSION "4.15.2")
set(GMSH_SOURCE_DIR "${CMAKE_BINARY_DIR}/_deps/gmsh-src")
set(GMSH_BUILD_DIR "${CMAKE_BINARY_DIR}/_deps/gmsh-build")

# Blossom: ON, explicitly. It is gmsh's default recombination algorithm (Mesh.RecombinationAlgorithm=1)
# and conda-forge's gmsh (the native worker) is built with it, so ON keeps browser and worker meshes
# identical for recombined non-transfinite faces. Its licence (contrib/blossom/README.txt: concorde /
# Blossom IV, a written grant from W. Cook for academic AND non-academic use "within the Gmsh
# system") is not a GPL licence; Fedora strips it for that reason. Flagged for the licence review.
set(GMSH_WASM_BLOSSOM ON CACHE BOOL "Build gmsh-wasm with contrib/blossom")

set(_GMSH_FLAGS "-fwasm-exceptions -sSUPPORT_LONGJMP=wasm -fPIC")
set(_GMSH_CMAKE_ARGS
    -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_C_FLAGS=${_GMSH_FLAGS}
    -DCMAKE_CXX_FLAGS=${_GMSH_FLAGS}
    -DCMAKE_C_FLAGS_RELEASE=-O3
    -DCMAKE_CXX_FLAGS_RELEASE=-O3
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
    # find_library/find_path outside emscripten's sysroot (adacpp's OCCT install).
    -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=BOTH
    -DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=BOTH
    -DCMAKE_PREFIX_PATH=${OCCT_INSTALL_DIR}
    # Library only.
    -DENABLE_BUILD_LIB=ON
    -DENABLE_BUILD_SHARED=OFF
    -DENABLE_BUILD_DYNAMIC=OFF
    -DENABLE_OS_SPECIFIC_INSTALL=OFF
    -DENABLE_TESTS=OFF
    -DENABLE_PRIVATE_API=OFF
    -DENABLE_WRAP_PYTHON=OFF
    -DENABLE_WRAP_JAVA=OFF
    # GUI / graphics off.
    -DENABLE_FLTK=OFF
    -DENABLE_GRAPHICS=OFF
    -DENABLE_OSMESA=OFF
    -DENABLE_CAIRO=OFF
    -DENABLE_MPEG_ENCODE=OFF
    -DENABLE_TOUCHBAR=OFF
    -DENABLE_POPPLER=OFF
    # OCC on, against the shared OCCT.
    -DENABLE_OCC=ON
    -DENABLE_OCC_CAF=ON
    -DENABLE_OCC_STATIC=OFF
    -DENABLE_OCC_TBB=OFF
    -DOCC_INC=${OCCT_INCLUDE_DIR}
    -DOCC_LIBS=${OCCT_LIB_DIR}/libTKernel.a
    # Core modules.
    -DENABLE_MESH=ON
    -DENABLE_POST=ON
    -DENABLE_PARSER=ON
    -DENABLE_SOLVER=ON
    -DENABLE_EIGEN=ON
    -DENABLE_BLAS_LAPACK=OFF
    -DENABLE_BLOSSOM=${GMSH_WASM_BLOSSOM}
    # Single-threaded, no parallel runtimes (Pyodide has no pthreads).
    -DENABLE_OPENMP=OFF
    -DENABLE_MPI=OFF
    -DENABLE_OPENACC=OFF
    # Optional meshers / libraries the plate+beam mesh step does not use.
    -DENABLE_HXT=OFF
    -DENABLE_NETGEN=OFF
    -DENABLE_MMG=OFF
    -DENABLE_METIS=OFF
    -DENABLE_ALGLIB=OFF
    -DENABLE_WINSLOWUNTANGLER=OFF
    # Post-processing plugins (gmsh.plugin.*): ada does not use them, and the BoundaryLayer plugin
    # includes winslowUntangler.h unconditionally, which needs ALGLIB. Mesh size fields do not
    # depend on PLUGINS.
    -DENABLE_PLUGINS=OFF
    -DENABLE_GMP=OFF
    -DENABLE_KBIPACK=OFF
    -DENABLE_DOMHEX=OFF
    # QuadMeshingTools ON (self-contained contrib): recombined meshes go through its quad-topology
    # cleanup (Mesh.RecombineOptimizeTopology=5 by default). Without it the same face recombined to
    # 275 nodes / 238 quads vs the worker's 277 / 240, from an identical triangulation.
    -DENABLE_QUADMESHINGTOOLS=ON
    -DENABLE_GEOMETRYCENTRAL=OFF
    -DENABLE_VOROPP=OFF
    -DENABLE_MESQUITE=OFF
    -DENABLE_NII2MESH=OFF
    -DENABLE_REVOROPT=OFF
    -DENABLE_P4EST=OFF
    # File formats / solvers off.
    -DENABLE_MED=OFF
    -DENABLE_CGNS=OFF
    -DENABLE_PETSC=OFF
    -DENABLE_SLEPC=OFF
    -DENABLE_MUMPS=OFF
    -DENABLE_GETDP=OFF
    -DENABLE_ZIPPER=OFF
    # ONELAB talks to solvers over sockets/subprocesses: nothing to talk to in a browser.
    -DENABLE_ONELAB=OFF
    -DENABLE_ONELAB_METAMODEL=OFF
)

ExternalProject_Add(
    gmsh_external
    URL https://gmsh.info/src/gmsh-${GMSH_VERSION}-source.tgz
    URL_HASH
        SHA256=be3f66f225d27ba9fa014f07e83169285da8a051b0e8ab7103d88066b39bdd3e
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_DIR ${GMSH_SOURCE_DIR}
    BINARY_DIR ${GMSH_BUILD_DIR}
    # OCC_CAF: gmsh asks for TKV3d + TKService + Freetype (OCCT's Visualization module) although its
    # CAF code (GModelIO_OCC.cpp, HAVE_OCC_CAF) uses only XCAF/TDocStd/STEPCAF/IGESCAF. Drop the two
    # Visualization toolkits and the Freetype gate rather than build Visualization for nothing.
    #
    # OCCT 8: the two patches conda-forge's gmsh 4.15.2 build applies (gmsh-feedstock
    # recipe/patches, commit 78bc4b5, BSD-3-Clause): C++17 for OCCT 8's headers, and the API compat
    # for the collection-iterator / LProp headers OCCT 8.0.0 removed. Same source as the native worker.
    PATCH_COMMAND
        patch -p1 -d <SOURCE_DIR> -i
        ${CMAKE_CURRENT_LIST_DIR}/patches/gmsh/cxx17_for_occt.patch
    COMMAND
        patch -p1 -d <SOURCE_DIR> -i
        ${CMAKE_CURRENT_LIST_DIR}/patches/gmsh/occt800_api_compat.patch
    COMMAND
        sed -i -e
        "s/TKXCAF TKLCAF TKVCAF TKCAF TKV3d TKService TKCDF)/TKXCAF TKLCAF TKVCAF TKCAF TKCDF)/"
        -e "s/^    find_package(Freetype)$/    set(FREETYPE_FOUND TRUE)/"
        ${GMSH_SOURCE_DIR}/CMakeLists.txt
    # gmsh appends $ENV{CMAKE_PREFIX_PATH}/include (and CMAKE_INCLUDE_PATH) to its include path. In a
    # pixi/conda env that is the HOST sysroot (x86_64 glibc headers: "gnu/stubs-32.h not found"), so
    # configure without them.
    # Conda's compiler activation also exports GNU-ld LDFLAGS (--sort-common, --as-needed, ...): wasm-ld
    # rejects them, so EVERY try_compile check failed and gmsh configured itself with no stdint.h,
    # no socklen_t and no vsnprintf (cmake/wasm_toolchain.cmake clears the same flags for adacpp).
    CONFIGURE_COMMAND
        ${CMAKE_COMMAND} -E env --unset=CMAKE_PREFIX_PATH
        --unset=CMAKE_INCLUDE_PATH --unset=LDFLAGS --unset=CFLAGS
        --unset=CXXFLAGS --unset=CPPFLAGS ${CMAKE_COMMAND} -G ${CMAKE_GENERATOR}
        ${_GMSH_CMAKE_ARGS} <SOURCE_DIR>
    BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --target lib
    INSTALL_COMMAND ""
    BUILD_BYPRODUCTS ${GMSH_BUILD_DIR}/libgmsh.a
    USES_TERMINAL_CONFIGURE TRUE
    USES_TERMINAL_BUILD TRUE
)
if(NOT WASM_OCCT_PREBUILT_DIR)
    add_dependencies(gmsh_external occt_external)
endif()

# libgmsh.so: whole libgmsh.a + NEEDED libocct.so. SIDE_MODULE=1 exports every default-visibility
# symbol: the gmsh* C API that gmsh.py's ctypes calls (GMSH_API is visibility("default")), and gmsh's
# C++ internals (harmless: nothing else carries gmsh).
set(_gmsh_stub "${CMAKE_BINARY_DIR}/gmsh_shared_stub.c")
file(
    WRITE "${_gmsh_stub}"
    "int adacpp_gmsh_shared_marker(void) { return 1; }\n"
)
add_library(gmsh_shared SHARED "${_gmsh_stub}")
set_target_properties(
    gmsh_shared
    PROPERTIES OUTPUT_NAME gmsh PREFIX "lib" SUFFIX ".so"
)
target_link_libraries(gmsh_shared PRIVATE occt_shared)
target_link_options(
    gmsh_shared
    PRIVATE
        "-sSIDE_MODULE=1"
        "-sWASM_BIGINT"
        "-fwasm-exceptions"
        "-sSUPPORT_LONGJMP=wasm"
        "-Wl,--no-gc-sections"
        "-Wl,--whole-archive"
        "${GMSH_BUILD_DIR}/libgmsh.a"
        "-Wl,--no-whole-archive"
)
set_property(
    TARGET gmsh_shared
    APPEND
    PROPERTY LINK_DEPENDS "${GMSH_BUILD_DIR}/libgmsh.a"
)
add_dependencies(gmsh_shared gmsh_external)

# Wheel staging: gmsh.py (patched) at the top level, the library under gmsh.libs/ (the auditwheel
# convention Pyodide's loader searches for a wheel's dylibs).
install(TARGETS gmsh_shared LIBRARY DESTINATION wheel-gmsh/gmsh.libs)
install(
    CODE
        "execute_process(COMMAND \"${Python_EXECUTABLE}\" \"${CMAKE_SOURCE_DIR}/tools/patch_gmsh_py.py\" \"${GMSH_SOURCE_DIR}/api/gmsh.py\" \"\${CMAKE_INSTALL_PREFIX}/wheel-gmsh/gmsh.py\" COMMAND_ERROR_IS_FATAL ANY)"
)
install(
    FILES ${GMSH_SOURCE_DIR}/LICENSE.txt ${GMSH_SOURCE_DIR}/CREDITS.txt
    DESTINATION wheel-gmsh-licenses
)
