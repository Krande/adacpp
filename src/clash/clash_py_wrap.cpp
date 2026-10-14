// Python bindings for the convex-contact clash pass (convex_contact.h).
//
// Arrays in, arrays out: one (n, 3) float64 vertex array and one (m, 3) uint32 triangle array per
// mesh, and the contacts back as columns (pairs, near points, normals, depths, areas) rather than one
// object per contact -- a plant model has thousands. The GIL is released around the check.
#include "clash_py_wrap.h"

#include <stdexcept>
#include <string>
#include <vector>

#include <nanobind/stl/array.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/tuple.h>

#include "convex_contact.h"

using namespace adacpp::clash;

namespace {

using F64In = nb::ndarray<const double, nb::c_contig, nb::device::cpu>;
using U32In = nb::ndarray<const std::uint32_t, nb::c_contig, nb::device::cpu>;

template <typename T> nb::ndarray<nb::numpy, T> new_array(std::initializer_list<std::size_t> shape) {
    std::size_t n = 1;
    for (std::size_t s : shape)
        n *= s;
    T *data = new T[n > 0 ? n : 1];
    nb::capsule owner(data, [](void *p) noexcept { delete[] static_cast<T *>(p); });
    return nb::ndarray<nb::numpy, T>(data, shape, owner);
}

// (n, 3) or empty: a mesh with no vertices is allowed (it touches nothing), any other shape is not.
std::size_t rows_of_3(std::size_t ndim, std::size_t n0, std::size_t n1, std::size_t size, const char *what,
                      std::size_t k) {
    if (size == 0)
        return 0;
    if (ndim != 2 || n1 != 3)
        throw std::invalid_argument(std::string(what) + "[" + std::to_string(k) + "] must be (n, 3)");
    return n0;
}

nb::dict find_convex_contacts_py(const std::vector<F64In> &vertices, const std::vector<U32In> &triangles,
                                 double security_margin, double min_contact_area, double line_contact_thickness,
                                 bool hull) {
    if (vertices.size() != triangles.size())
        throw std::invalid_argument("find_convex_contacts: one triangle array per vertex array");
    std::vector<MeshView> meshes(vertices.size());
    for (std::size_t k = 0; k < vertices.size(); ++k) {
        const F64In &v = vertices[k];
        const U32In &t = triangles[k];
        meshes[k].vertices = v.data();
        meshes[k].n_vertices =
            rows_of_3(v.ndim(), v.ndim() > 0 ? v.shape(0) : 0, v.ndim() > 1 ? v.shape(1) : 0, v.size(), "vertices", k);
        meshes[k].triangles = t.data();
        meshes[k].n_triangles =
            rows_of_3(t.ndim(), t.ndim() > 0 ? t.shape(0) : 0, t.ndim() > 1 ? t.shape(1) : 0, t.size(), "triangles", k);
    }
    ConvexContactOptions options;
    options.security_margin = security_margin;
    options.min_contact_area = min_contact_area;
    options.line_contact_thickness = line_contact_thickness;
    options.hull = hull;

    ConvexContactResult r;
    {
        nb::gil_scoped_release release;
        r = find_convex_contacts(meshes, options);
    }

    const std::size_t n = r.contacts.size();
    auto pairs = new_array<std::int64_t>({n, 2});
    auto near1 = new_array<double>({n, 3});
    auto near2 = new_array<double>({n, 3});
    auto normal = new_array<double>({n, 3});
    auto depth = new_array<double>({n});
    auto area = new_array<double>({n});
    for (std::size_t c = 0; c < n; ++c) {
        const ConvexContact &x = r.contacts[c];
        pairs.data()[2 * c] = static_cast<std::int64_t>(x.i);
        pairs.data()[2 * c + 1] = static_cast<std::int64_t>(x.j);
        for (int k = 0; k < 3; ++k) {
            near1.data()[3 * c + k] = x.near1[k];
            near2.data()[3 * c + k] = x.near2[k];
            normal.data()[3 * c + k] = x.normal[k];
        }
        depth.data()[c] = x.penetration_depth;
        area.data()[c] = x.contact_area;
    }
    nb::list failures;
    for (const ConvexContactFailure &f : r.failures)
        failures.append(nb::make_tuple(f.i, f.j, f.reason));

    nb::dict out;
    out["pairs"] = pairs;
    out["near1"] = near1;
    out["near2"] = near2;
    out["normal"] = normal;
    out["penetration_depth"] = depth;
    out["contact_area"] = area;
    out["failures"] = failures;
    out["candidate_pairs"] = r.candidate_pairs;
    out["below_min_area"] = r.below_min_area;
    out["degenerate"] = r.degenerate;
    return out;
}

nb::tuple convex_hull_py(const F64In &points) {
    const std::size_t n = rows_of_3(points.ndim(), points.ndim() > 0 ? points.shape(0) : 0,
                                    points.ndim() > 1 ? points.shape(1) : 0, points.size(), "points", 0);
    ConvexMesh h;
    {
        nb::gil_scoped_release release;
        h = convex_hull(points.data(), n);
    }
    auto v = new_array<double>({h.vertices.size() / 3, 3});
    auto t = new_array<std::uint32_t>({h.triangles.size() / 3, 3});
    std::copy(h.vertices.begin(), h.vertices.end(), v.data());
    std::copy(h.triangles.begin(), h.triangles.end(), t.data());
    return nb::make_tuple(v, t);
}

std::vector<std::array<double, 3>> rows3(const F64In &a, const char *what) {
    const std::size_t n =
        rows_of_3(a.ndim(), a.ndim() > 0 ? a.shape(0) : 0, a.ndim() > 1 ? a.shape(1) : 0, a.size(), what, 0);
    std::vector<std::array<double, 3>> out(n);
    for (std::size_t i = 0; i < n; ++i)
        out[i] = {a.data()[3 * i], a.data()[3 * i + 1], a.data()[3 * i + 2]};
    return out;
}

} // namespace

void clash_module(nb::module_ &m) {
    m.attr("DEFAULT_SECURITY_MARGIN") = DEFAULT_SECURITY_MARGIN;
    m.attr("DEFAULT_MIN_CONTACT_AREA") = DEFAULT_MIN_CONTACT_AREA;
    m.attr("DEFAULT_LINE_CONTACT_THICKNESS") = DEFAULT_LINE_CONTACT_THICKNESS;

    m.def("find_convex_contacts", &find_convex_contacts_py, "vertices"_a, "triangles"_a,
          "security_margin"_a = DEFAULT_SECURITY_MARGIN, "min_contact_area"_a = DEFAULT_MIN_CONTACT_AREA,
          "line_contact_thickness"_a = DEFAULT_LINE_CONTACT_THICKNESS, "hull"_a = true,
          "Every pair of meshes whose convex hulls touch or overlap within `security_margin`, measured "
          "by coal: a dict of columns `pairs` (k, 2) int64 with i < j, `near1` / `near2` / `normal` "
          "(k, 3), `penetration_depth` (k,) in coal's sign (negative = overlap) and `contact_area` (k,) "
          "m^2, plus `failures` [(i, j, reason)] for pairs coal could not evaluate, `candidate_pairs`, "
          "`below_min_area` and `degenerate` (meshes with no solid hull). `vertices[k]` is an (n, 3) "
          "float64 array and `triangles[k]` an (m, 3) uint32 array indexing it. With `hull` (the "
          "default) the hull is taken here and the triangles are ignored; without it the meshes must "
          "be convex hulls already.");

    m.def("convex_hull", &convex_hull_py, "points"_a,
          "The convex hull of `points` (n, 3) as (vertices (m, 3) float64, triangles (t, 3) uint32), "
          "outward-wound; both empty when the points span no volume.");

    m.def(
        "candidate_pairs",
        [](const F64In &lo, const F64In &hi, double security_margin) {
            const auto l = rows3(lo, "lo"), h = rows3(hi, "hi");
            if (l.size() != h.size())
                throw std::invalid_argument("candidate_pairs: lo and hi differ in length");
            return candidate_pairs(l, h, security_margin);
        },
        "lo"_a, "hi"_a, "security_margin"_a,
        "The broad phase alone: (i, j) pairs, i < j and sorted, whose boxes (lo, hi: (n, 3) float64) "
        "come within `security_margin` on every axis.");

    m.def(
        "patch_area",
        [](const F64In &points, const std::array<double, 3> &normal, double line_contact_thickness) {
            return patch_area(rows3(points, "points"), normal, line_contact_thickness);
        },
        "points"_a, "normal"_a, "line_contact_thickness"_a = DEFAULT_LINE_CONTACT_THICKNESS,
        "The area of one contact patch: `points` (n, 3) in the plane with `normal`. Two points are a "
        "line contact (length * thickness); three or more their in-plane convex hull, or the patch's "
        "length * thickness where the points are flat.");
}
