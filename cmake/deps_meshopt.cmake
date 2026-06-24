# meshoptimizer — Arseny Kapoulkine (zeux), MIT license.
# Vendored at third_party/meshoptimizer (v1.1, rev d5cbaa1, github.com/zeux/meshoptimizer) so the
# build needs no network and the same source compiles for native AND wasm. Used for the merged-GLB
# cleanup that step2glb's merge applies (meshopt_simplify, threshold 0.75 / target_error 0.0 — a
# lossless coplanar-triangle collapse) and for EXT_meshopt_compression vertex/index encoding.
#
# Like libtess2, the .cpp sources are added to ADA_CPP_SOURCES directly so they land in both the
# native and wasm extension modules; only the include dir is exported here.
set(MESHOPT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/meshoptimizer)
file(GLOB MESHOPT_SOURCES ${MESHOPT_DIR}/src/*.cpp)
set(MESHOPT_INCLUDE_DIR ${MESHOPT_DIR}/src)
