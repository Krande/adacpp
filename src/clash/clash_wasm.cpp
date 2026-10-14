// The convex-contact clash pass (convex_contact.h, glb_contacts.h) as a STANDALONE embind wasm module --
// coal and the pass, no OCCT, no pyodide, no Python. The same C++ the nanobind module (adacpp.clash)
// runs, so a browser and a worker report the same contacts for the same meshes.
//
// Two verbs:
//
// `findGlbContacts(glb, securityMargin, minContactArea, lineContactThickness, upAxis)` -- the members of
// a viewer GLB (a Uint8Array: the bytes the viewer loaded), one per element of its id_hierarchy /
// draw_ranges tables, hulled and measured. The worker runs the same verb on the same bytes
// (`adacpp.clash.find_glb_contacts`). Adds `memberIds` / `memberNames` (string arrays) to the result;
// the contact indices refer to them.
//
// `findConvexContacts(vertices, vertexOffsets, triangles, triangleOffsets, securityMargin,
// minContactArea, lineContactThickness, hull)` -- meshes as ONE flat soup, not one JS object per mesh.
// Mesh k is vertices [vertexOffsets[k], vertexOffsets[k+1]) of `vertices` (xyz triples) and triangles
// [triangleOffsets[k], triangleOffsets[k+1]) of `triangles` (index triples, LOCAL to the mesh). Both
// offset arrays have one entry more than there are meshes. With `hull` true each mesh's convex hull is
// taken from its vertices and its triangles are ignored (pass empty triangle ranges); with it false
// the meshes must be convex hulls already.
//
// Contacts come back as columns (typed arrays copied out of the wasm heap):
//   { pairs: Uint32Array(2k), near1, near2, normal: Float64Array(3k),
//     penetrationDepth, contactArea: Float64Array(k),
//     patchOffsets: Uint32Array(k+1), patchNormals: Float64Array(3P),
//     patchPointOffsets: Uint32Array(P+1), patchPoints1, patchPoints2: Float64Array(3N),
//     failures: [{i, j, reason}], candidatePairs, belowMinArea, degenerate: Uint32Array }
// Contact q's patches are [patchOffsets[q], patchOffsets[q+1]); patch p's polygon is points
// [patchPointOffsets[p], patchPointOffsets[p+1]) of patchPoints1 (on mesh i) and patchPoints2 (on j).
// Malformed input is { error } rather than a trap.

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include "convex_contact.h"
#include "glb_contacts.h"

using emscripten::val;
using namespace adacpp::clash;

namespace {

template <typename T> val copy_out(const std::vector<T> &v) {
    // A view onto the heap, sliced into a JS-owned copy: the vector dies when this function returns.
    return val(emscripten::typed_memory_view(v.size(), v.data())).call<val>("slice");
}

ConvexContactOptions make_options(double security_margin, double min_contact_area, double line_contact_thickness,
                                  bool hull) {
    ConvexContactOptions options;
    options.security_margin = security_margin;
    options.min_contact_area = min_contact_area;
    options.line_contact_thickness = line_contact_thickness;
    options.hull = hull;
    return options;
}

val contacts_val(const ConvexContactResult &r) {
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
    // The contact patches, flattened: contact q owns patches [patchOffsets[q], patchOffsets[q+1]),
    // patch p owns points [patchPointOffsets[p], patchPointOffsets[p+1]).
    std::vector<std::uint32_t> patch_offsets(c + 1, 0), point_offsets(1, 0);
    std::vector<double> patch_normals, points1, points2;
    for (std::size_t q = 0; q < c; ++q) {
        for (const ContactPatch &p : r.contacts[q].patches) {
            patch_normals.insert(patch_normals.end(), p.normal.begin(), p.normal.end());
            for (std::size_t k = 0; k < p.points1.size(); ++k) {
                points1.insert(points1.end(), p.points1[k].begin(), p.points1[k].end());
                points2.insert(points2.end(), p.points2[k].begin(), p.points2[k].end());
            }
            point_offsets.push_back(static_cast<std::uint32_t>(points1.size() / 3));
        }
        patch_offsets[q + 1] = static_cast<std::uint32_t>(point_offsets.size() - 1);
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
    out.set("patchOffsets", copy_out(patch_offsets));
    out.set("patchNormals", copy_out(patch_normals));
    out.set("patchPointOffsets", copy_out(point_offsets));
    out.set("patchPoints1", copy_out(points1));
    out.set("patchPoints2", copy_out(points2));
    out.set("failures", failures);
    out.set("candidatePairs", static_cast<double>(r.candidate_pairs));
    out.set("belowMinArea", static_cast<double>(r.below_min_area));
    out.set("degenerate", copy_out(std::vector<std::uint32_t>(r.degenerate.begin(), r.degenerate.end())));
    return out;
}

val error_val(const std::exception &e) {
    val out = val::object();
    out.set("error", std::string(e.what()));
    return out;
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
        return contacts_val(find_convex_contacts(
            meshes, make_options(security_margin, min_contact_area, line_contact_thickness, hull)));
    } catch (const std::exception &e) {
        return error_val(e);
    }
}

val find_glb_contacts_js(const val &glb_js, double security_margin, double min_contact_area,
                         double line_contact_thickness, const std::string &up_axis) {
    try {
        // One bulk copy into the heap (TypedArray.set), not element by element: a GLB is megabytes.
        const std::size_t n = glb_js["length"].as<std::size_t>();
        std::vector<unsigned char> bytes(n);
        val(emscripten::typed_memory_view(n, bytes.data())).call<void>("set", glb_js);
        const GlbContactResult r =
            find_glb_contacts(bytes.data(), bytes.size(),
                              make_options(security_margin, min_contact_area, line_contact_thickness, true), up_axis);
        val out = contacts_val(r.contacts);
        val ids = val::array(), names = val::array();
        for (const GlbMember &m : r.members) {
            ids.call<void>("push", m.id);
            names.call<void>("push", m.name);
        }
        out.set("memberIds", ids);
        out.set("memberNames", names);
        return out;
    } catch (const std::exception &e) {
        return error_val(e);
    }
}

} // namespace

EMSCRIPTEN_BINDINGS(adacpp_clash) {
    emscripten::constant("DEFAULT_SECURITY_MARGIN", DEFAULT_SECURITY_MARGIN);
    emscripten::constant("DEFAULT_MIN_CONTACT_AREA", DEFAULT_MIN_CONTACT_AREA);
    emscripten::constant("DEFAULT_LINE_CONTACT_THICKNESS", DEFAULT_LINE_CONTACT_THICKNESS);
    emscripten::function("findConvexContacts", &find_convex_contacts_js);
    emscripten::function("findGlbContacts", &find_glb_contacts_js);
}
