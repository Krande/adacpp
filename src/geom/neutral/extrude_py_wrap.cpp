// Python bindings for the prismatic extrusion core (ngeom_extrude.h).
//
// The expansion entry point takes FLAT per-instance arrays rather than a list of record objects.
// A model can carry hundreds of thousands of swept instances, and converting each into a Python
// object would cost more than the expansion it is feeding. The array layout is also the shape the
// data already has on the wire, so a caller streams straight from its buffers.
#include "extrude_py_wrap.h"

#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <nanobind/stl/array.h>

#include "ngeom_extrude.h"

using namespace adacpp::ngeom;

namespace {

// Zero-copy read-only view onto a vector owned by a Python object.
template <typename T> auto view(std::vector<T> &v, nb::handle owner) {
    return nb::ndarray<nb::numpy, const T, nb::ndim<1>>(v.data(), {v.size()}, owner);
}

void expect(bool cond, const char *what) {
    if (!cond)
        throw std::invalid_argument(what);
}

// Result holder: the expansion allocates once, and Python reads it through views rather than
// copying several megabytes of vertex data into lists.
struct ExpandResult {
    BeamSolidMesh mesh;
    std::vector<uint32_t> range_label, range_tri_start, range_tri_count;
};

} // namespace

void extrude_module(nb::module_ &m) {
    nb::class_<ExtrudedSection>(m, "ExtrudedSection",
                                "A cross-section prepared once and swept any number of times.\n"
                                "`triangles` indexes the 2n vertices one sweep produces: [0, n) is the\n"
                                "start ring and [n, 2n) the end ring, covering both caps and the walls.")
        .def_ro("ok", &ExtrudedSection::ok)
        .def_prop_ro("n_points", [](ExtrudedSection &s) { return s.n_points(); })
        .def_prop_ro("points",
                     [](ExtrudedSection &s) {
                         // (n, 2) in section coordinates.
                         return nb::ndarray<nb::numpy, const double, nb::ndim<2>>(
                             s.points.empty() ? nullptr : &s.points[0][0], {s.points.size(), 2}, nb::find(s));
                     })
        .def_prop_ro("triangles", [](ExtrudedSection &s) {
            return nb::ndarray<nb::numpy, const uint32_t, nb::ndim<2>>(
                s.triangles.empty() ? nullptr : &s.triangles[0][0], {s.triangles.size(), 3}, nb::find(s));
        });

    nb::class_<ExpandResult>(m, "ExpandedSolids", "Vertex and index buffers for a batch of swept instances.")
        .def_prop_ro("positions", [](ExpandResult &r) { return view(r.mesh.positions, nb::find(r)); })
        .def_prop_ro("indices", [](ExpandResult &r) { return view(r.mesh.indices, nb::find(r)); })
        .def_prop_ro("node0", [](ExpandResult &r) { return view(r.mesh.node0, nb::find(r)); })
        .def_prop_ro("node1", [](ExpandResult &r) { return view(r.mesh.node1, nb::find(r)); })
        .def_prop_ro("t", [](ExpandResult &r) { return view(r.mesh.t, nb::find(r)); })
        .def_prop_ro("range_label", [](ExpandResult &r) { return view(r.range_label, nb::find(r)); })
        .def_prop_ro("range_tri_start", [](ExpandResult &r) { return view(r.range_tri_start, nb::find(r)); })
        .def_prop_ro("range_tri_count", [](ExpandResult &r) { return view(r.range_tri_count, nb::find(r)); });

    m.def(
        "make_extruded_section",
        [](nb::ndarray<const double, nb::ndim<2>> points, nb::ndarray<const uint32_t, nb::ndim<2>> triangles) {
            expect(points.shape(1) == 2, "points must be (n, 2)");
            expect(triangles.shape(1) == 3, "triangles must be (m, 3)");
            ExtrudedSection sec;
            sec.points.resize(points.shape(0));
            for (size_t i = 0; i < points.shape(0); ++i)
                sec.points[i] = {points(i, 0), points(i, 1)};
            const uint32_t limit = static_cast<uint32_t>(sec.points.size()) * 2;
            sec.triangles.resize(triangles.shape(0));
            for (size_t k = 0; k < triangles.shape(0); ++k) {
                for (size_t c = 0; c < 3; ++c) {
                    // A table read from somewhere else is untrusted input: an index past
                    // the 2n vertices a sweep produces would read off the end of the
                    // output buffer, so reject it here rather than discover it there.
                    expect(triangles(k, c) < limit, "a triangle index is outside the 2n swept vertices");
                    sec.triangles[k][c] = triangles(k, c);
                }
            }
            sec.ok = true;
            return sec;
        },
        "points"_a, "triangles"_a,
        "Rebuild a section table from arrays -- the inverse of reading one back out of an\n"
        "ExtrudedSection. For a caller replaying a table it stored earlier; sections built\n"
        "here skip the cap triangulation entirely, so the arrays are taken as given apart\n"
        "from a bounds check on the indices.");

    m.def("build_extruded_section", &build_extruded_section, "loops"_a,
          "Build a section table from planar outline loops: loops[0] is the outer boundary, any\n"
          "further loops are holes. Winding is normalized internally, so a caller may hand holes\n"
          "in their natural order. Returns a section whose `ok` is False when the cap could not be\n"
          "indexed against the outline points, which is the caller's signal to fall back rather\n"
          "than ship caps that do not meet their walls.");

    m.def(
        "expand_beam_solids",
        [](nb::list sections, nb::ndarray<const uint32_t, nb::ndim<1>> label,
           nb::ndarray<const uint32_t, nb::ndim<1>> section_idx, nb::ndarray<const double, nb::ndim<2>> origin,
           nb::ndarray<const double, nb::ndim<2>> xvec, nb::ndarray<const double, nb::ndim<2>> yvec,
           nb::ndarray<const double, nb::ndim<1>> length, nb::object node0_o, nb::object node1_o, nb::object points_o) {
            // nb::object rather than std::optional<nb::ndarray<...>>: nanobind has no
            // caster for that composition, and an unmatchable signature rejects every
            // call rather than only the ones omitting these.
            const bool with_nodes = !node0_o.is_none() && !node1_o.is_none() && !points_o.is_none();
            const bool any_node = !node0_o.is_none() || !node1_o.is_none() || !points_o.is_none();
            const size_t nb_inst = label.shape(0);
            expect(section_idx.shape(0) == nb_inst && length.shape(0) == nb_inst,
                   "per-instance arrays must all have the same length");
            expect(origin.shape(0) == nb_inst && xvec.shape(0) == nb_inst && yvec.shape(0) == nb_inst,
                   "frame arrays must have one row per instance");
            expect(origin.shape(1) == 3 && xvec.shape(1) == 3 && yvec.shape(1) == 3, "frame arrays must be (n, 3)");
            // node0/node1/points travel together: the axial parameter is measured
            // between the two nodes, so one without the others says nothing.
            expect(with_nodes || !any_node, "node0, node1 and points must be given together or not at all");
            nb::ndarray<const uint32_t, nb::ndim<1>> node0, node1;
            nb::ndarray<const double, nb::ndim<2>> points;
            if (with_nodes) {
                node0 = nb::cast<nb::ndarray<const uint32_t, nb::ndim<1>>>(node0_o);
                node1 = nb::cast<nb::ndarray<const uint32_t, nb::ndim<1>>>(node1_o);
                points = nb::cast<nb::ndarray<const double, nb::ndim<2>>>(points_o);
                expect(node0.shape(0) == nb_inst && node1.shape(0) == nb_inst,
                       "node arrays must have one entry per instance");
                expect(points.shape(1) == 3, "points must be (n, 3)");
            }

            std::vector<ExtrudedSection> secs;
            secs.reserve(sections.size());
            for (nb::handle h : sections)
                secs.push_back(nb::cast<ExtrudedSection>(h));

            std::vector<BeamInstance> beams(nb_inst);
            for (size_t k = 0; k < nb_inst; ++k) {
                BeamInstance &b = beams[k];
                b.label = label(k);
                b.section = section_idx(k);
                b.node0 = with_nodes ? node0(k) : 0;
                b.node1 = with_nodes ? node1(k) : 0;
                b.length = length(k);
                for (size_t c = 0; c < 3; ++c) {
                    b.origin[c] = origin(k, c);
                    b.xvec[c] = xvec(k, c);
                    b.yvec[c] = yvec(k, c);
                }
            }

            std::vector<double> pts;
            if (with_nodes) {
                pts.resize(points.shape(0) * 3);
                for (size_t i = 0; i < points.shape(0); ++i)
                    for (size_t c = 0; c < 3; ++c)
                        pts[i * 3 + c] = points(i, c);
            }

            ExpandResult r;
            r.mesh = expand_beam_solids(secs, beams, pts);
            r.range_label.reserve(r.mesh.ranges.size());
            r.range_tri_start.reserve(r.mesh.ranges.size());
            r.range_tri_count.reserve(r.mesh.ranges.size());
            for (const InstanceRange &ir : r.mesh.ranges) {
                r.range_label.push_back(ir.label);
                r.range_tri_start.push_back(ir.tri_start);
                r.range_tri_count.push_back(ir.tri_count);
            }
            return r;
        },
        "sections"_a, "label"_a, "section_idx"_a, "origin"_a, "xvec"_a, "yvec"_a, "length"_a, "node0"_a = nb::none(),
        "node1"_a = nb::none(), "points"_a = nb::none(),
        "Expand prepared sections plus per-instance frames into vertex and index buffers.\n\n"
        "node0/node1/points are optional and travel together. Give them and every vertex\n"
        "also reports the node pair it interpolates between and its axial parameter along\n"
        "that pair; omit them for ordinary display geometry, which wants neither.\n\n"
        "`points` is the caller's own point buffer that node0/node1 index into. The axial\n"
        "parameter is measured against those NODE positions, not against the sweep axis: when an\n"
        "instance's ends carry different offsets the sweep frame tilts away from the node line, so\n"
        "the parameter varies within a ring and cannot be stored per section.");
}
