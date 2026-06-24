#pragma once

#include "ngeom_tessellate.h" // TessMesh

namespace adacpp::ngeom {

// CSG boolean of two tessellated solid meshes (OCC-free, via Manifold).
//   op: 0 = difference (a - b), 1 = union (a + b), 2 = intersection (a ∩ b)
// The operand meshes are welded into 2-manifolds (MeshGL::Merge) before the
// Boolean. Returns false (and leaves `out` untouched) when Manifold is not
// compiled in (e.g. the wasm build) or an operand is not a valid manifold.
bool mesh_boolean(int op, const TessMesh &a, const TessMesh &b, TessMesh &out);

} // namespace adacpp::ngeom
