#include "cad_py_wrap.h"
#include "../geom/Mesh.h"

namespace {

// Stub used during the wasm spike. Returns a fixed unit-box mesh shape so the
// pipeline (build → wheel → pyodide load → call → return Mesh) can be exercised
// end-to-end without depending on OCCT. Will be replaced by an OCCT-backed
// implementation once OCCT-wasm comes online.
Mesh tessellate_box_stub(float dx, float dy, float dz) {
    const float x = dx * 0.5f;
    const float y = dy * 0.5f;
    const float z = dz * 0.5f;

    std::vector<float> positions = {
        -x, -y, -z,   x, -y, -z,   x,  y, -z,  -x,  y, -z,
        -x, -y,  z,   x, -y,  z,   x,  y,  z,  -x,  y,  z,
    };
    std::vector<uint32_t> faces = {
        0, 1, 2,  0, 2, 3,   // -Z
        4, 6, 5,  4, 7, 6,   // +Z
        0, 4, 5,  0, 5, 1,   // -Y
        3, 2, 6,  3, 6, 7,   // +Y
        0, 3, 7,  0, 7, 4,   // -X
        1, 5, 6,  1, 6, 2,   // +X
    };
    return Mesh(0, std::move(positions), std::move(faces));
}

} // namespace

void cad_module(nb::module_ &m) {
    nb::class_<Mesh>(m, "Mesh")
        .def_ro("positions", &Mesh::positions)
        .def_ro("indices",   &Mesh::indices)
        .def_ro("normals",   &Mesh::normals);

    m.def("tessellate_box", &tessellate_box_stub,
          "dx"_a, "dy"_a, "dz"_a,
          "Return a tessellated axis-aligned box centered at the origin (stub).");
}
