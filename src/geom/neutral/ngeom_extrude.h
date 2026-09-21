// Prismatic profile extrusion — OCC-free, and the expansion half is header-only with no
// dependency beyond <vector>/<array>/<cmath>.
//
// Why the file is split the way it is: a swept cross-section is cheap to DESCRIBE (an outline plus
// a frame) and expensive to STORE (two rings of vertices and a triangle list per instance). So the
// two halves have different homes:
//
//   * build_extruded_section() turns an outline into the per-section table — outline points plus
//     the complete triangle list for one extrusion (both caps and the side walls). It needs the
//     libtess2 triangulator, so it lives in ngeom_extrude.cpp.
//
//   * expand_beam_solids() takes that table plus one small record per instance and produces the
//     vertex/index buffers. It needs no triangulator at all, so it is inline here and compiles
//     into a wasm module without dragging libtess2 along.
//
// A consumer that only ever expands a prepared table (the browser) therefore links nothing but
// this header; a consumer that prepares tables (the writer) links the .cpp as well.
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace adacpp::ngeom {

using Pt2 = std::array<double, 2>;
using Tri3 = std::array<uint32_t, 3>;

// ---- section table ---------------------------------------------------------------------------

// One cross-section, prepared once and swept any number of times.
//
// `points` is the concatenation of the outline loops (outer first, then holes) in ring order.
// `triangles` indexes the 2n vertices ONE sweep of this section produces: [0, n) is the start
// ring, [n, 2n) the end ring, in the same order as `points`. It covers both caps and the side
// walls, so an instance contributes no topology of its own — only a frame.
struct ExtrudedSection {
    std::vector<Pt2> points;
    std::vector<Tri3> triangles;
    bool ok = false;

    uint32_t n_points() const {
        return static_cast<uint32_t>(points.size());
    }
    uint32_t n_verts_per_instance() const {
        return static_cast<uint32_t>(points.size() * 2);
    }
};

// Build the section table from planar outline loops: loops[0] is the outer boundary, any further
// loops are holes. Triangulates the cap once (libtess2, with the shrunk-hole retry) and emits both
// caps plus per-loop side walls, outward-wound for a sweep along +x of the section frame.
//
// ok=false when the cap triangulation fails, or when it produced vertices that are not outline
// points — libtess2 introduces vertices where contours intersect, and such a cap cannot be indexed
// against the two rings this layout is built on. The caller is expected to fall back rather than
// ship a section whose caps do not line up with its walls.
ExtrudedSection build_extruded_section(const std::vector<std::vector<Pt2>> &loops);

// ---- instance expansion ----------------------------------------------------------------------

// One swept instance. The frame is the sweep's own: `origin` is the start-cap origin, `xvec` the
// sweep direction, `yvec` the in-section axis that the outline's first coordinate runs along. The
// second in-section axis is derived (see below) rather than shipped, so the record stays small and
// cannot disagree with itself.
//
// node0/node1 index the caller's point buffer. They are carried through untouched and are the
// reference the axial parameter is measured against — see expand_beam_solids.
struct BeamInstance {
    uint32_t label = 0;
    uint32_t section = 0;
    uint32_t node0 = 0;
    uint32_t node1 = 0;
    double origin[3] = {0, 0, 0};
    double xvec[3] = {0, 0, 0};
    double yvec[3] = {0, 0, 0};
    double length = 0.0;
};

// Triangle range owned by one instance, so a picker can map a triangle back to its source element.
struct InstanceRange {
    uint32_t label = 0;
    uint32_t tri_start = 0;
    uint32_t tri_count = 0;
};

struct BeamSolidMesh {
    std::vector<float> positions;  // flat xyz, 3 per vertex
    std::vector<uint32_t> indices; // flat, 3 per triangle
    std::vector<uint32_t> node0;   // per vertex
    std::vector<uint32_t> node1;   // per vertex
    std::vector<float> t;          // per vertex, axial parameter in [0, 1]
    std::vector<InstanceRange> ranges;
};

// Expand prepared sections + instance records into the buffers a renderer consumes.
//
// `points` is the caller's own point buffer (flat xyz, 3 per point) that node0/node1 index into.
//
// The axial parameter is measured against those NODE positions, not against the sweep axis. When
// an instance's two ends carry different offsets the sweep frame tilts away from the node line, so
// t varies WITHIN a ring; that is exactly why it is derived here instead of stored per section.
//
// Positions are rounded to float32 before t is measured. A consumer that receives only the output
// buffers has no other positions to measure against, so measuring against the unrounded doubles
// here would put this result a rounding step away from anything recomputed downstream.
inline BeamSolidMesh expand_beam_solids(const std::vector<ExtrudedSection> &sections,
                                        const std::vector<BeamInstance> &beams, const std::vector<double> &points) {
    BeamSolidMesh out;

    size_t n_verts = 0, n_tris = 0;
    for (const BeamInstance &b : beams) {
        if (b.section >= sections.size())
            continue;
        const ExtrudedSection &s = sections[b.section];
        n_verts += s.n_verts_per_instance();
        n_tris += s.triangles.size();
    }
    out.positions.resize(n_verts * 3);
    out.node0.resize(n_verts);
    out.node1.resize(n_verts);
    out.t.resize(n_verts);
    out.indices.resize(n_tris * 3);
    out.ranges.reserve(beams.size());

    const size_t n_points = points.size() / 3;
    uint32_t vertex_offset = 0, tri_cursor = 0;

    for (const BeamInstance &b : beams) {
        if (b.section >= sections.size())
            continue;
        const ExtrudedSection &sec = sections[b.section];
        const uint32_t n = sec.n_points();
        if (n == 0)
            continue;

        // The second in-section axis: normalize(cross(xvec, yvec)). Deriving it is what keeps the
        // record from carrying a frame that does not close; a degenerate cross (parallel or zero
        // input axes) leaves it zero, collapsing the section onto its first axis rather than
        // emitting NaN through every downstream buffer.
        double ux = b.xvec[1] * b.yvec[2] - b.xvec[2] * b.yvec[1];
        double uy = b.xvec[2] * b.yvec[0] - b.xvec[0] * b.yvec[2];
        double uz = b.xvec[0] * b.yvec[1] - b.xvec[1] * b.yvec[0];
        const double un = std::sqrt(ux * ux + uy * uy + uz * uz);
        if (un > 0.0) {
            ux /= un;
            uy /= un;
            uz /= un;
        }

        // Both rings: (u, v) -> origin + u * yvec + v * up, and the end ring one length along xvec.
        for (uint32_t i = 0; i < n; ++i) {
            const double u = sec.points[i][0];
            const double v = sec.points[i][1];
            const double px = u * b.yvec[0] + v * ux + b.origin[0];
            const double py = u * b.yvec[1] + v * uy + b.origin[1];
            const double pz = u * b.yvec[2] + v * uz + b.origin[2];
            const size_t a = (static_cast<size_t>(vertex_offset) + i) * 3;
            const size_t c = (static_cast<size_t>(vertex_offset) + n + i) * 3;
            out.positions[a] = static_cast<float>(px);
            out.positions[a + 1] = static_cast<float>(py);
            out.positions[a + 2] = static_cast<float>(pz);
            out.positions[c] = static_cast<float>(px + b.length * b.xvec[0]);
            out.positions[c + 1] = static_cast<float>(py + b.length * b.xvec[1]);
            out.positions[c + 2] = static_cast<float>(pz + b.length * b.xvec[2]);
        }

        const uint32_t ring2 = n * 2;
        double p0x = 0, p0y = 0, p0z = 0, ax = 0, ay = 0, az = 0;
        if (b.node0 < n_points && b.node1 < n_points) {
            p0x = points[static_cast<size_t>(b.node0) * 3];
            p0y = points[static_cast<size_t>(b.node0) * 3 + 1];
            p0z = points[static_cast<size_t>(b.node0) * 3 + 2];
            ax = points[static_cast<size_t>(b.node1) * 3] - p0x;
            ay = points[static_cast<size_t>(b.node1) * 3 + 1] - p0y;
            az = points[static_cast<size_t>(b.node1) * 3 + 2] - p0z;
        }
        const double axis_sq = ax * ax + ay * ay + az * az;

        for (uint32_t i = 0; i < ring2; ++i) {
            const size_t vi = static_cast<size_t>(vertex_offset) + i;
            double tv = 0.0;
            if (axis_sq > 0.0) {
                // Read the float32 positions back, so this is the same measurement a consumer
                // holding only the output buffers would make.
                const double rx = static_cast<double>(out.positions[vi * 3]) - p0x;
                const double ry = static_cast<double>(out.positions[vi * 3 + 1]) - p0y;
                const double rz = static_cast<double>(out.positions[vi * 3 + 2]) - p0z;
                tv = (rx * ax + ry * ay + rz * az) / axis_sq;
                tv = tv < 0.0 ? 0.0 : (tv > 1.0 ? 1.0 : tv);
            }
            // A zero-length instance leaves every vertex at t=0, so an interpolation across the
            // two nodes collapses onto node0 instead of dividing by nothing.
            out.t[vi] = static_cast<float>(tv);
            out.node0[vi] = b.node0;
            out.node1[vi] = b.node1;
        }

        const uint32_t tri_count = static_cast<uint32_t>(sec.triangles.size());
        for (uint32_t k = 0; k < tri_count; ++k) {
            const size_t d = (static_cast<size_t>(tri_cursor) + k) * 3;
            out.indices[d] = sec.triangles[k][0] + vertex_offset;
            out.indices[d + 1] = sec.triangles[k][1] + vertex_offset;
            out.indices[d + 2] = sec.triangles[k][2] + vertex_offset;
        }
        out.ranges.push_back({b.label, tri_cursor, tri_count});

        vertex_offset += ring2;
        tri_cursor += tri_count;
    }

    return out;
}

} // namespace adacpp::ngeom
