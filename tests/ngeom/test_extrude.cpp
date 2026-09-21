// Prismatic profile extrusion (ngeom_extrude.h / .cpp): section tables and instance expansion.
//
// What these pin, in order of what would hurt most if it broke:
//   1. the extrusion is CLOSED — every edge used exactly twice, so caps meet their walls
//   2. it is OUTWARD-wound — signed volume positive and equal to area * length, which a
//      consistently inverted or self-cancelling winding cannot fake
//   3. holes subtract, and their walls close on themselves rather than bridging to the outer
//      boundary
//   4. the axial parameter is measured against the NODE line, not the sweep axis — the two differ
//      exactly when the ends carry different offsets, which is the case that motivated deriving it
//
// Build: see tests/ngeom/run.sh (links ngeom_extrude.cpp + ngeom_tessellate.cpp + libtess2).
#include <array>
#include <cmath>
#include <cstdio>
#include <map>
#include <vector>

#include "ngeom_extrude.h"

using namespace adacpp::ngeom;

static int g_fail = 0;
#define CHECK(cond, msg)                                                                                               \
    do {                                                                                                               \
        if (!(cond)) {                                                                                                 \
            std::printf("FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__);                                               \
            ++g_fail;                                                                                                  \
        }                                                                                                              \
    } while (0)

static bool close_to(double a, double b, double tol) {
    return std::fabs(a - b) <= tol;
}

// Signed volume by the divergence theorem: (1/6) * sum over triangles of v0 . (v1 x v2).
// Positive for a closed, outward-wound surface.
static double mesh_volume(const BeamSolidMesh &m) {
    double vol = 0.0;
    for (size_t i = 0; i < m.indices.size(); i += 3) {
        const float *a = &m.positions[static_cast<size_t>(m.indices[i]) * 3];
        const float *b = &m.positions[static_cast<size_t>(m.indices[i + 1]) * 3];
        const float *c = &m.positions[static_cast<size_t>(m.indices[i + 2]) * 3];
        const double cx = static_cast<double>(b[1]) * c[2] - static_cast<double>(b[2]) * c[1];
        const double cy = static_cast<double>(b[2]) * c[0] - static_cast<double>(b[0]) * c[2];
        const double cz = static_cast<double>(b[0]) * c[1] - static_cast<double>(b[1]) * c[0];
        vol += (static_cast<double>(a[0]) * cx + static_cast<double>(a[1]) * cy + static_cast<double>(a[2]) * cz);
    }
    return vol / 6.0;
}

// Every undirected edge of a closed 2-manifold is used by exactly two triangles.
static bool is_closed(const BeamSolidMesh &m, int *worst) {
    std::map<std::array<uint32_t, 2>, int> uses;
    for (size_t i = 0; i < m.indices.size(); i += 3) {
        for (int e = 0; e < 3; ++e) {
            uint32_t p = m.indices[i + e];
            uint32_t q = m.indices[i + (e + 1) % 3];
            uses[{std::min(p, q), std::max(p, q)}] += 1;
        }
    }
    int bad = 0;
    for (const auto &kv : uses)
        if (kv.second != 2)
            ++bad;
    if (worst)
        *worst = bad;
    return bad == 0;
}

static BeamInstance unit_instance(double length) {
    BeamInstance b;
    b.label = 7;
    b.section = 0;
    b.node0 = 0;
    b.node1 = 1;
    b.origin[0] = 0;
    b.origin[1] = 0;
    b.origin[2] = 0;
    b.xvec[0] = 1;
    b.xvec[1] = 0;
    b.xvec[2] = 0; // sweep along +x
    b.yvec[0] = 0;
    b.yvec[1] = 1;
    b.yvec[2] = 0; // first section axis along +y; derived second axis is +z
    b.length = length;
    return b;
}

static std::vector<Pt2> rect(double w, double h) {
    return {{-w, -h}, {w, -h}, {w, h}, {-w, h}};
}

int main() {
    // ---- 1. solid rectangle ---------------------------------------------------------------
    {
        ExtrudedSection sec = build_extruded_section({rect(0.15, 0.30)});
        CHECK(sec.ok, "rectangular section builds");
        CHECK(sec.n_points() == 4, "rectangle keeps its 4 outline points");
        // 2 cap triangles per ring + 2 wall triangles per edge
        CHECK(sec.triangles.size() == 2 * 2 + 4 * 2, "rectangle triangle count");

        const double L = 3.0;
        BeamSolidMesh m = expand_beam_solids({sec}, {unit_instance(L)}, {0, 0, 0, L, 0, 0});
        CHECK(m.positions.size() == 8 * 3, "two rings of 4 vertices");

        int bad = 0;
        CHECK(is_closed(m, &bad), "rectangular extrusion is closed");
        const double expect = 0.30 * 0.60 * L;
        CHECK(close_to(mesh_volume(m), expect, expect * 1e-5), "rectangular extrusion volume, outward-wound");

        CHECK(m.ranges.size() == 1 && m.ranges[0].label == 7, "instance range carries its label");
        CHECK(m.ranges[0].tri_count == sec.triangles.size(), "range covers the section's triangles");
    }

    // ---- 2. rectangle with a rectangular void ----------------------------------------------
    {
        // Outline and hole given in the SAME winding on purpose: the builder is responsible for
        // normalizing them, so a caller cannot get a hole wrong by handing it in the natural order.
        ExtrudedSection sec = build_extruded_section({rect(0.20, 0.20), rect(0.10, 0.10)});
        CHECK(sec.ok, "section with a void builds");
        CHECK(sec.n_points() == 8, "outer + hole points are concatenated");

        const double L = 2.0;
        BeamSolidMesh m = expand_beam_solids({sec}, {unit_instance(L)}, {0, 0, 0, L, 0, 0});

        int bad = 0;
        CHECK(is_closed(m, &bad), "hollow extrusion is closed");
        const double expect = (0.40 * 0.40 - 0.20 * 0.20) * L;
        CHECK(close_to(mesh_volume(m), expect, expect * 1e-5), "hollow extrusion volume subtracts the void");
    }

    // ---- 3. the axial parameter follows the node line, not the sweep axis -------------------
    {
        ExtrudedSection sec = build_extruded_section({rect(0.10, 0.10)});
        const double L = 4.0;

        // Sweep along +x, but the two nodes are offset from each other in y as well: the node line
        // tilts away from the sweep axis, which is what makes t vary inside a single ring.
        std::vector<double> points = {0, 0, 0, L, 1.0, 0};
        BeamSolidMesh m = expand_beam_solids({sec}, {unit_instance(L)}, points);

        bool ring_varies = false;
        for (uint32_t i = 1; i < sec.n_points(); ++i)
            if (!close_to(m.t[i], m.t[0], 1e-7))
                ring_varies = true;
        CHECK(ring_varies, "t varies within a ring when the node line tilts off the sweep axis");

        for (size_t i = 0; i < m.t.size(); ++i)
            CHECK(m.t[i] >= 0.0f && m.t[i] <= 1.0f, "t stays in [0, 1]");
        for (size_t i = 0; i < m.node0.size(); ++i)
            CHECK(m.node0[i] == 0 && m.node1[i] == 1, "every vertex carries the instance's node pair");

        // Aligned nodes: the cap rings sit at the ends, so t reads 0 and 1 per ring.
        BeamSolidMesh a = expand_beam_solids({sec}, {unit_instance(L)}, {0, 0, 0, L, 0, 0});
        for (uint32_t i = 0; i < sec.n_points(); ++i) {
            CHECK(close_to(a.t[i], 0.0, 1e-6), "start ring at t=0 when nodes align with the sweep");
            CHECK(close_to(a.t[sec.n_points() + i], 1.0, 1e-6), "end ring at t=1 when nodes align");
        }
    }

    // ---- 4. degenerate instances do not poison the buffers ----------------------------------
    {
        ExtrudedSection sec = build_extruded_section({rect(0.10, 0.10)});
        // Coincident nodes: no axis to project onto.
        BeamSolidMesh m = expand_beam_solids({sec}, {unit_instance(0.0)}, {1, 1, 1, 1, 1, 1});
        for (size_t i = 0; i < m.t.size(); ++i)
            CHECK(m.t[i] == 0.0f, "zero-length node line collapses t to 0 rather than dividing by nothing");
        for (size_t i = 0; i < m.positions.size(); ++i)
            CHECK(std::isfinite(m.positions[i]), "positions stay finite for a degenerate instance");
    }

    // ---- 5. many instances share one section -------------------------------------------------
    {
        ExtrudedSection sec = build_extruded_section({rect(0.05, 0.05)});
        std::vector<BeamInstance> beams;
        std::vector<double> points;
        for (int k = 0; k < 64; ++k) {
            BeamInstance b = unit_instance(1.0);
            b.label = static_cast<uint32_t>(k);
            b.node0 = static_cast<uint32_t>(2 * k);
            b.node1 = static_cast<uint32_t>(2 * k + 1);
            b.origin[2] = 0.5 * k;
            beams.push_back(b);
            points.insert(points.end(), {0, 0, 0.5 * k, 1.0, 0, 0.5 * k});
        }
        BeamSolidMesh m = expand_beam_solids({sec}, beams, points);
        CHECK(m.ranges.size() == 64, "one range per instance");
        CHECK(m.positions.size() == 64 * 8 * 3, "vertices scale with instance count, section stored once");
        int bad = 0;
        CHECK(is_closed(m, &bad), "a batch of instances is still edge-closed as a whole");
        const double expect = 0.10 * 0.10 * 1.0 * 64;
        CHECK(close_to(mesh_volume(m), expect, expect * 1e-4), "batch volume is the sum of its instances");

        uint32_t tri_total = 0;
        for (const InstanceRange &r : m.ranges)
            tri_total += r.tri_count;
        CHECK(tri_total * 3 == m.indices.size(), "ranges partition the index buffer exactly");
    }

    if (g_fail == 0)
        std::printf("test_extrude: all checks passed\n");
    return g_fail == 0 ? 0 : 1;
}
