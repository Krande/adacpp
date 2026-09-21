// Section-table construction for ngeom_extrude.h. Separate TU because this half — and only this
// half — needs the libtess2 triangulator; the expander stays header-only so a consumer that merely
// replays a prepared table links nothing.
#include "ngeom_extrude.h"

#include <algorithm>
#include <cstdlib>

#include "ngeom_tessellate.h"

namespace adacpp::ngeom {

namespace {

// Signed area of a closed polygon in section coordinates. > 0 is counter-clockwise.
double signed_area(const std::vector<Pt2> &p) {
    double a = 0.0;
    const size_t n = p.size();
    for (size_t i = 0; i < n; ++i) {
        const Pt2 &q = p[i];
        const Pt2 &r = p[(i + 1) % n];
        a += q[0] * r[1] - r[0] * q[1];
    }
    return a * 0.5;
}

// Drop a repeated closing point: callers vary on whether a loop repeats its first point, and a
// duplicate would emit a degenerate side-wall quad and a duplicate ring vertex.
void drop_closing_point(std::vector<Pt2> &p) {
    if (p.size() < 2)
        return;
    const double du = p.front()[0] - p.back()[0];
    const double dv = p.front()[1] - p.back()[1];
    if (du * du + dv * dv < 1e-20)
        p.pop_back();
}

} // namespace

ExtrudedSection build_extruded_section(const std::vector<std::vector<Pt2>> &loops) {
    ExtrudedSection out;
    if (loops.empty())
        return out;

    // Normalize winding up front: outer counter-clockwise, holes clockwise. The section frame is
    // right-handed as (first coord, second coord, sweep), so with that convention one side-wall
    // rule is outward for the outer boundary AND for a hole — a hole's outward face points into
    // the hole, which is the same expression, not a special case.
    std::vector<std::vector<Pt2>> rings;
    rings.reserve(loops.size());
    for (size_t li = 0; li < loops.size(); ++li) {
        std::vector<Pt2> r = loops[li];
        drop_closing_point(r);
        if (r.size() < 3)
            continue;
        const bool want_ccw = rings.empty(); // the first surviving loop is the outer boundary
        if ((signed_area(r) > 0.0) != want_ccw)
            std::reverse(r.begin(), r.end());
        rings.push_back(std::move(r));
    }
    if (rings.empty())
        return out;

    // Flat ring order — this is the vertex order one sweep produces, twice.
    std::vector<uint32_t> loop_start;
    loop_start.reserve(rings.size());
    for (const std::vector<Pt2> &r : rings) {
        loop_start.push_back(static_cast<uint32_t>(out.points.size()));
        out.points.insert(out.points.end(), r.begin(), r.end());
    }
    const uint32_t n = out.points.size() > 0 ? static_cast<uint32_t>(out.points.size()) : 0;
    if (n == 0)
        return out;

    PolyTriangulation cap = triangulate_polygon_with_holes(rings);
    if (!cap.ok)
        return out;

    // Map the triangulated cap back onto the ring vertices. libtess2 introduces vertices where
    // contours intersect; such a cap cannot be indexed against the rings this layout is built on,
    // so report failure rather than emit caps that do not meet their own walls.
    //
    // Tolerance is relative to the section's own extent: sections are authored in whatever units
    // the caller uses, so a fixed epsilon would be either meaningless or punitive depending on
    // scale. It must also clear the triangulator's own precision — libtess2 works in TESSreal,
    // which is float, so a point round-trips with ~1e-7 relative error and a tighter tolerance
    // would reject every section including trivially valid ones. Sections carry tens of points, so
    // the linear scan costs less than building an index.
    double lo_u = out.points[0][0], hi_u = lo_u, lo_v = out.points[0][1], hi_v = lo_v;
    for (const Pt2 &p : out.points) {
        lo_u = std::min(lo_u, p[0]);
        hi_u = std::max(hi_u, p[0]);
        lo_v = std::min(lo_v, p[1]);
        hi_v = std::max(hi_v, p[1]);
    }
    const double extent = std::max(hi_u - lo_u, hi_v - lo_v);
    const double tol = (extent > 0.0 ? extent : 1.0) * 1e-6;
    const double tol_sq = tol * tol;

    std::vector<uint32_t> cap_to_ring(cap.verts.size(), UINT32_MAX);
    for (size_t i = 0; i < cap.verts.size(); ++i) {
        for (uint32_t j = 0; j < n; ++j) {
            const double du = cap.verts[i][0] - out.points[j][0];
            const double dv = cap.verts[i][1] - out.points[j][1];
            if (du * du + dv * dv <= tol_sq) {
                cap_to_ring[i] = j;
                break;
            }
        }
        if (cap_to_ring[i] == UINT32_MAX) {
            out.points.clear();
            return out; // ok stays false
        }
    }

    // Caps. A cap wound counter-clockwise in section coordinates faces along the sweep, so the end
    // ring takes it as-is and the start ring takes it reversed; both then face outward.
    out.triangles.reserve(cap.tris.size() * 2 + out.points.size() * 2);
    for (const Tri3 &t : cap.tris) {
        const uint32_t a = cap_to_ring[t[0]], b = cap_to_ring[t[1]], c = cap_to_ring[t[2]];
        if (a == b || b == c || a == c)
            continue; // collapsed against the ring mapping — contributes no area
        const double ux = out.points[b][0] - out.points[a][0];
        const double uy = out.points[b][1] - out.points[a][1];
        const double vx = out.points[c][0] - out.points[a][0];
        const double vy = out.points[c][1] - out.points[a][1];
        const bool ccw = (ux * vy - uy * vx) > 0.0;
        // start ring: inward-facing winding relative to the sweep
        if (ccw)
            out.triangles.push_back({a, c, b});
        else
            out.triangles.push_back({a, b, c});
        // end ring: the opposite winding, offset onto the second ring
        if (ccw)
            out.triangles.push_back({a + n, b + n, c + n});
        else
            out.triangles.push_back({a + n, c + n, b + n});
    }

    // Side walls, per loop so a hole's wall closes on itself instead of bridging to the outer
    // boundary. With the winding normalized above, one expression is outward for every loop.
    for (size_t li = 0; li < rings.size(); ++li) {
        const uint32_t start = loop_start[li];
        const uint32_t count = static_cast<uint32_t>(rings[li].size());
        for (uint32_t i = 0; i < count; ++i) {
            const uint32_t a = start + i;
            const uint32_t b = start + (i + 1) % count;
            out.triangles.push_back({a, b, b + n});
            out.triangles.push_back({a, b + n, a + n});
        }
    }

    out.ok = true;
    return out;
}

} // namespace adacpp::ngeom
