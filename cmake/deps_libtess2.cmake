# libtess2 — Mihai Vasilian / memononen GLU-tess port (SGI Free License B).
# Vendored at third_party/libtess2 (rev 8dbd6483, github.com/memononen/libtess2) so the
# build needs no network and the same source compiles for native AND wasm. It is the
# boundary-driven polygon triangulator step2glb uses; adacpp ports step2glb's per-face
# tessellation algorithm (UV-project boundary -> metric scale -> tessTesselate -> refine ->
# map back to 3D) on top of it as an alternative to BRepMesh.
#
# Exposes the INTERFACE library `libtess2` (include dir + the .c sources are added to
# ADA_CPP_SOURCES directly so they land in both the native and wasm extension modules).
set(LIBTESS2_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/libtess2)
set(LIBTESS2_SOURCES
        ${LIBTESS2_DIR}/Source/bucketalloc.c
        ${LIBTESS2_DIR}/Source/dict.c
        ${LIBTESS2_DIR}/Source/geom.c
        ${LIBTESS2_DIR}/Source/mesh.c
        ${LIBTESS2_DIR}/Source/priorityq.c
        ${LIBTESS2_DIR}/Source/sweep.c
        ${LIBTESS2_DIR}/Source/tess.c
)
set(LIBTESS2_INCLUDE_DIR ${LIBTESS2_DIR}/Include)
