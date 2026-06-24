// OCC-free CSG for the libtess2 path: boolean of two tessellated solid meshes via Manifold
// (Apache-2.0). Operands are welded into 2-manifolds, then Manifold's robust mesh Boolean runs.
// On builds without Manifold (wasm) mesh_boolean is a no-op stub.
#include "ngeom_boolean.h"

#ifdef ADACPP_HAVE_MANIFOLD

#include <manifold/manifold.h>

#include <cmath>
#include <cstdint>

namespace adacpp::ngeom {
namespace {

// TessMesh (triangle soup: positions xyz + uint32 indices) -> a welded Manifold. MeshGL::Merge
// recovers shared vertices by position so the soup becomes a watertight 2-manifold.
manifold::Manifold to_manifold(const TessMesh &m) {
    manifold::MeshGL gl;
    gl.numProp = 3;
    gl.vertProperties = m.positions;  // interleaved xyz (float)
    gl.triVerts = m.indices;          // 3 uint32 indices per triangle
    gl.Merge();
    return manifold::Manifold(gl);
}

}  // namespace

bool mesh_boolean(int op, const TessMesh &a, const TessMesh &b, TessMesh &out) {
    if (a.indices.empty() || b.indices.empty()) return false;
    using E = manifold::Manifold::Error;
    manifold::Manifold ma = to_manifold(a);
    manifold::Manifold mb = to_manifold(b);
    if (ma.Status() != E::NoError || mb.Status() != E::NoError) return false;

    manifold::OpType ot = op == 1   ? manifold::OpType::Add        // union
                          : op == 2 ? manifold::OpType::Intersect  // intersection
                                    : manifold::OpType::Subtract;  // 0 = difference
    manifold::Manifold r = ma.Boolean(mb, ot);
    if (r.Status() != E::NoError) return false;

    manifold::MeshGL gl = r.GetMeshGL();  // numProp == 3 (positions only)
    if (gl.triVerts.empty()) return false;
    out.positions = gl.vertProperties;
    out.indices = gl.triVerts;

    // smooth per-vertex normals (Manifold output is indexed with shared vertices)
    out.normals.assign(out.positions.size(), 0.0f);
    for (size_t t = 0; t + 2 < out.indices.size(); t += 3) {
        uint32_t ia = out.indices[t], ib = out.indices[t + 1], ic = out.indices[t + 2];
        const float *A = &out.positions[ia * 3], *B = &out.positions[ib * 3], *C = &out.positions[ic * 3];
        float ux = B[0] - A[0], uy = B[1] - A[1], uz = B[2] - A[2];
        float vx = C[0] - A[0], vy = C[1] - A[1], vz = C[2] - A[2];
        float nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
        for (uint32_t k : {ia, ib, ic}) {
            out.normals[k * 3] += nx;
            out.normals[k * 3 + 1] += ny;
            out.normals[k * 3 + 2] += nz;
        }
    }
    for (size_t v = 0; v + 2 < out.normals.size(); v += 3) {
        float nx = out.normals[v], ny = out.normals[v + 1], nz = out.normals[v + 2];
        float l = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (l > 1e-20f) {
            out.normals[v] = nx / l;
            out.normals[v + 1] = ny / l;
            out.normals[v + 2] = nz / l;
        }
    }
    return true;
}

}  // namespace adacpp::ngeom

#else  // no Manifold (e.g. wasm): CSG not available on this build

namespace adacpp::ngeom {
bool mesh_boolean(int, const TessMesh &, const TessMesh &, TessMesh &) { return false; }
}  // namespace adacpp::ngeom

#endif
