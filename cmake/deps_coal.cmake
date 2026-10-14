# coal (BSD-3, github.com/coal-library/coal) -- a mature GJK/EPA + contact-patch collision library.
# Compiled here from coal's own sources into the convex-contact clash library (src/clash), native AND
# wasm, so the numbers are coal's on every target.
#
# WHY NOT coal's OWN BUILD. It requires Boost serialization + filesystem and assimp (for its mesh
# loader and serialization) and jrl-cmakemodules (for three generated headers), none of which
# cross-compiles cheaply and none of which convex-vs-convex collision touches. So the tarball is only
# unpacked (SOURCE_SUBDIR points at a directory without a CMakeLists.txt) and this file compiles coal's
# collision sources itself:
#   - everything under src/ except mesh_loader/ (assimp), serialization/ (Boost.Serialization),
#     broadphase/ (boost::function; we have our own broad phase) and octree.cpp (octomap);
#   - against cmake/coal/include: coal's three generated headers (config/deprecated/warning.hh) and a
#     shim for the one Boost.Math constant (pi) the collision code uses;
#   - against the conda env's Eigen, the same one the rest of adacpp links.
# No coal source is patched.
#
# Bumping coal: change the URL + hash, the version in cmake/coal/include/coal/config.hh, and diff
# coal's src/CMakeLists.txt source list against COAL_SOURCES below.
include(FetchContent)
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW) # extracted files get the extraction time, so a URL bump rebuilds
endif()

FetchContent_Declare(
    coal
    URL https://github.com/coal-library/coal/archive/refs/tags/v3.0.4.tar.gz
    URL_HASH
        SHA256=0a4f58e55b88a3d9f873ce79979aecc08491d212ea0b9311df013df3b22bffac
    SOURCE_SUBDIR
    _adacpp_populate_only
)
FetchContent_MakeAvailable(coal)
set(COAL_SOURCE_DIR ${coal_SOURCE_DIR})

# Eigen: header-only. The native build already has the Eigen3::Eigen target (deps_ifc.cmake); the wasm
# builds find the same conda-installed headers directly, since the emscripten toolchain confines
# find_package to its sysroot.
if(NOT TARGET Eigen3::Eigen AND NOT EMSCRIPTEN)
    find_package(Eigen3 CONFIG QUIET)
endif()
if(NOT TARGET Eigen3::Eigen)
    find_path(
        ADACPP_EIGEN_INCLUDE_DIR
        Eigen/Core
        PATHS
            $ENV{CONDA_PREFIX}/include/eigen3
            $ENV{CONDA_PREFIX}/Library/include/eigen3
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
    )
    if(NOT ADACPP_EIGEN_INCLUDE_DIR)
        message(
            FATAL_ERROR
            "deps_coal: Eigen not found (no Eigen3::Eigen target and no Eigen/Core under $CONDA_PREFIX)"
        )
    endif()
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    target_include_directories(
        Eigen3::Eigen
        INTERFACE ${ADACPP_EIGEN_INCLUDE_DIR}
    )
endif()

set(COAL_SOURCES
    collision.cpp
    contact_patch.cpp
    contact_patch/contact_patch_solver.cpp
    contact_patch_func_matrix.cpp
    distance_func_matrix.cpp
    collision_data.cpp
    collision_node.cpp
    collision_object.cpp
    BV/RSS.cpp
    BV/AABB.cpp
    BV/kIOS.cpp
    BV/kDOP.cpp
    BV/OBBRSS.cpp
    BV/OBB.cpp
    narrowphase/gjk.cpp
    narrowphase/minkowski_difference.cpp
    narrowphase/support_functions.cpp
    shape/convex.cpp
    shape/geometric_shapes.cpp
    shape/geometric_shapes_utility.cpp
    distance/box_halfspace.cpp
    distance/box_plane.cpp
    distance/box_sphere.cpp
    distance/capsule_capsule.cpp
    distance/capsule_halfspace.cpp
    distance/capsule_plane.cpp
    distance/cone_halfspace.cpp
    distance/cone_plane.cpp
    distance/cylinder_halfspace.cpp
    distance/cylinder_plane.cpp
    distance/sphere_sphere.cpp
    distance/sphere_cylinder.cpp
    distance/sphere_halfspace.cpp
    distance/sphere_plane.cpp
    distance/sphere_capsule.cpp
    distance/ellipsoid_halfspace.cpp
    distance/ellipsoid_plane.cpp
    distance/convex_halfspace.cpp
    distance/convex_plane.cpp
    distance/triangle_halfspace.cpp
    distance/triangle_plane.cpp
    distance/triangle_triangle.cpp
    distance/triangle_sphere.cpp
    distance/halfspace_plane.cpp
    distance/plane_plane.cpp
    distance/halfspace_halfspace.cpp
    intersect.cpp
    math/transform.cpp
    traversal/traversal_recurse.cpp
    distance.cpp
    BVH/BVH_utility.cpp
    BVH/BV_fitter.cpp
    BVH/BVH_model.cpp
    BVH/BV_splitter.cpp
    collision_func_matrix.cpp
    collision_utility.cpp
    hfield.cpp
)
list(TRANSFORM COAL_SOURCES PREPEND ${COAL_SOURCE_DIR}/src/)

# coal + the convex-contact pass on top of it, as ONE static library. Only src/clash is public: coal,
# Eigen and the shim headers stay private, so no consumer compiles against coal (and the shim never
# competes with a real Boost on a consumer's include path).
add_library(
    adacpp_clash
    STATIC
    ${COAL_SOURCES}
    ${CMAKE_SOURCE_DIR}/src/clash/convex_contact.cpp
)
set_target_properties(
    adacpp_clash
    PROPERTIES CXX_STANDARD 17 POSITION_INDEPENDENT_CODE ON
)
target_include_directories(
    adacpp_clash
    BEFORE
    PRIVATE
        ${CMAKE_SOURCE_DIR}/cmake/coal/include
        ${COAL_SOURCE_DIR}/include
        ${COAL_SOURCE_DIR}/src
)
target_include_directories(adacpp_clash PUBLIC ${CMAKE_SOURCE_DIR}/src/clash)
# Manifold (deps_manifold.cmake, already on every target) supplies the convex hulls.
target_link_libraries(adacpp_clash PRIVATE Eigen3::Eigen manifold)
if(MSVC)
    # coal's own MSVC settings: big objects, no min/max macros, Eigen's aligned storage.
    target_compile_options(adacpp_clash PRIVATE /bigobj)
    target_compile_definitions(
        adacpp_clash
        PRIVATE NOMINMAX _ENABLE_EXTENDED_ALIGNED_STORAGE
    )
else()
    # No a*b+c fusion, on any target: the same contact on a worker and in a browser is the point.
    target_compile_options(adacpp_clash PRIVATE -ffp-contract=off)
endif()
if(EMSCRIPTEN)
    # Every TU in a wasm link must agree on the EH / longjmp model (see wasm_occt.cmake). Hidden
    # visibility keeps coal's symbols out of the pyodide side module's exports.
    target_compile_options(
        adacpp_clash
        PRIVATE -fwasm-exceptions -sSUPPORT_LONGJMP=wasm -fvisibility=hidden
    )
endif()
