// The convex-contact clash pass (convex_contact.h) as a STANDALONE embind wasm module -- coal and the
// pass, no OCCT, no pyodide, no Python. The same C++ the nanobind module (adacpp.clash) runs, so a
// browser and a worker report the same contacts for the same meshes.
//
// Meshes go in as ONE flat soup, not one JS object per mesh: a model has thousands of members, and
// typed arrays cross the boundary once. Mesh k is vertices [vertexOffsets[k], vertexOffsets[k+1]) of
// `vertices` (xyz triples) and triangles [triangleOffsets[k], triangleOffsets[k+1]) of `triangles`
// (index triples, LOCAL to the mesh). Both offset arrays have one entry more than there are meshes.
// With `hull` true each mesh's convex hull is taken from its vertices and its triangles are ignored
// (pass empty triangle ranges); with it false the meshes must be convex hulls already.
//
// Contacts come back as columns (typed arrays copied out of the wasm heap):
//   { pairs: Uint32Array(2k), near1, near2, normal: Float64Array(3k),
//     penetrationDepth, contactArea: Float64Array(k),
//     failures: [{i, j, reason}], candidatePairs, belowMinArea, degenerate: Uint32Array }
// or { error } when the input is malformed.

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include "convex_contact.h"

using emscripten::val;
using namespace adacpp::clash;

namespace {

template <typename T> val copy_out(const std::vector<T> &v) {
    // A view onto the heap, sliced into a JS-owned copy: the vector dies when this function returns.
    return val(emscripten::typed_memory_view(v.size(), v.data())).call<val>("slice");
}

val find_convex_contacts_js(const val &vertices_js, const val &vertex_offsets_js, const val &triangles_js,
                            const val &triangle_offsets_js, double security_margin, double min_contact_area,
                            double line_contact_thickness, bool hull) {
    try {
        const auto vertices = emscripten::convertJSArrayToNumberVector<double>(vertices_js);
        const auto vertex_offsets = emscripten::convertJSArrayToNumberVector<std::uint32_t>(vertex_offsets_js);
        const auto triangles = emscripten::convertJSArrayToNumberVector<std::uint32_t>(triangles_js);
        const auto triangle_offsets = emscripten::convertJSArrayToNumberVector<std::uint32_t>(triangle_offsets_js);

        if (vertex_offsets.size() != triangle_offsets.size())
            throw std::invalid_argument("vertexOffsets and triangleOffsets must have the same length");
        const std::size_t n = vertex_offsets.empty() ? 0 : vertex_offsets.size() - 1;
        if (n > 0 &&
            (vertex_offsets.back() * 3ull > vertices.size() || triangle_offsets.back() * 3ull > triangles.size()))
            throw std::invalid_argument("offsets run past the vertex or triangle array");

        std::vector<MeshView> meshes(n);
        for (std::size_t k = 0; k < n; ++k) {
            if (vertex_offsets[k + 1] < vertex_offsets[k] || triangle_offsets[k + 1] < triangle_offsets[k])
                throw std::invalid_argument("offsets must not decrease (mesh " + std::to_string(k) + ")");
            meshes[k].vertices = vertices.data() + 3ull * vertex_offsets[k];
            meshes[k].n_vertices = vertex_offsets[k + 1] - vertex_offsets[k];
            meshes[k].triangles = triangles.data() + 3ull * triangle_offsets[k];
            meshes[k].n_triangles = triangle_offsets[k + 1] - triangle_offsets[k];
        }

        ConvexContactOptions options;
        options.security_margin = security_margin;
        options.min_contact_area = min_contact_area;
        options.line_contact_thickness = line_contact_thickness;
        options.hull = hull;
        const ConvexContactResult r = find_convex_contacts(meshes, options);

        const std::size_t c = r.contacts.size();
        std::vector<std::uint32_t> pairs(2 * c);
        std::vector<double> near1(3 * c), near2(3 * c), normal(3 * c), depth(c), area(c);
        for (std::size_t q = 0; q < c; ++q) {
            const ConvexContact &x = r.contacts[q];
            pairs[2 * q] = static_cast<std::uint32_t>(x.i);
            pairs[2 * q + 1] = static_cast<std::uint32_t>(x.j);
            for (int k = 0; k < 3; ++k) {
                near1[3 * q + k] = x.near1[k];
                near2[3 * q + k] = x.near2[k];
                normal[3 * q + k] = x.normal[k];
            }
            depth[q] = x.penetration_depth;
            area[q] = x.contact_area;
        }
        val failures = val::array();
        for (const ConvexContactFailure &f : r.failures) {
            val o = val::object();
            o.set("i", static_cast<double>(f.i));
            o.set("j", static_cast<double>(f.j));
            o.set("reason", f.reason);
            failures.call<void>("push", o);
        }

        val out = val::object();
        out.set("pairs", copy_out(pairs));
        out.set("near1", copy_out(near1));
        out.set("near2", copy_out(near2));
        out.set("normal", copy_out(normal));
        out.set("penetrationDepth", copy_out(depth));
        out.set("contactArea", copy_out(area));
        out.set("failures", failures);
        out.set("candidatePairs", static_cast<double>(r.candidate_pairs));
        out.set("belowMinArea", static_cast<double>(r.below_min_area));
        out.set("degenerate", copy_out(std::vector<std::uint32_t>(r.degenerate.begin(), r.degenerate.end())));
        return out;
    } catch (const std::exception &e) {
        val out = val::object();
        out.set("error", std::string(e.what()));
        return out;
    }
}

} // namespace

EMSCRIPTEN_BINDINGS(adacpp_clash) {
    emscripten::constant("DEFAULT_SECURITY_MARGIN", DEFAULT_SECURITY_MARGIN);
    emscripten::constant("DEFAULT_MIN_CONTACT_AREA", DEFAULT_MIN_CONTACT_AREA);
    emscripten::constant("DEFAULT_LINE_CONTACT_THICKNESS", DEFAULT_LINE_CONTACT_THICKNESS);
    emscripten::function("findConvexContacts", &find_convex_contacts_js);
}
