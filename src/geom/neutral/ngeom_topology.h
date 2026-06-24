// NGEOM neutral topology — edges, loops, face-surfaces (spec §6). The decoded form the
// tessellator consumes. A loop knows how to discretize itself into an ordered 3D polyline.
#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "ngeom_curves.h"
#include "ngeom_math.h"
#include "ngeom_surfaces.h"

namespace adacpp::ngeom {

// Discretize a circular/elliptical arc between two 3D endpoints by projecting them onto the
// curve to recover the angular parameter range — a port of step2glb model.rs curve_polyline's
// CIRCLE/ELLIPSE arm. step2glb never trusts stored edge parameters; it always rebuilds the arc
// from (frame, radii, vertex_a, vertex_b), handling the closed (a==b → full 2pi) case. We mirror
// that so a closed-circle edge whose `has_params` was dropped upstream doesn't collapse to a
// single point (and drop its whole face).
inline std::vector<Vec3> discretize_arc(const Frame &f, double rx, double ry, const Vec3 &a,
                                        const Vec3 &b, double deflection, double max_angle) {
    auto ang = [&](const Vec3 &pt) {
        Vec3 d = f.to_local(pt);
        return std::atan2(d.y / ry, d.x / rx);
    };
    double t0 = ang(a);
    double t1 = ang(b);
    bool full = (a - b).norm() < 1e-9 * (1.0 + std::abs(rx));
    if (full) {
        t1 = t0 + TWO_PI;
    } else {
        while (t1 <= t0 + 1e-12) t1 += TWO_PI;
    }
    double step = angle_step(std::max(std::abs(rx), std::abs(ry)), deflection, max_angle);
    int nseg = std::max(2, (int)std::ceil((t1 - t0) / step));
    std::vector<Vec3> pts;
    pts.reserve(nseg + 1);
    for (int i = 0; i <= nseg; ++i) {
        double t = t0 + (t1 - t0) * i / nseg;
        pts.push_back(f.to_world(rx * std::cos(t), ry * std::sin(t), 0.0));
    }
    pts.front() = a;  // snap exactly to the topological vertices
    pts.back() = b;
    return pts;
}

// Faithful port of step2glb model.rs align_polyline_to_vertices: a basis curve is sampled over its
// FULL natural domain, but the edge trims it to an interior stretch between vertices a,b. Exporters
// close closed edges at a vertex away from the curve's own seam, so blindly snapping the curve's
// natural endpoints onto a,b would fold the polyline through long false chords. Instead: if the
// sampled endpoints already sit on a,b keep it; if the basis curve is closed, walk the ring forward
// from nearest(a) to nearest(b); else trim the open curve to [nearest(a), nearest(b)].
inline std::vector<Vec3> align_polyline_to_vertices(const std::vector<Vec3> &pts, const Vec3 &a,
                                                    const Vec3 &b) {
    const size_t n = pts.size();
    if (n < 3) return pts;
    double step = 0.0, len = 0.0;
    for (size_t i = 1; i < n; ++i) {
        double d = (pts[i] - pts[i - 1]).norm();
        step = std::max(step, d);
        len += d;
    }
    const double tol = std::max(step, 1e-12);
    if ((pts[0] - a).norm() <= tol && (pts[n - 1] - b).norm() <= tol) return pts;  // common case
    auto nearest = [](const Vec3 &q, const std::vector<Vec3> &ring) {
        size_t bi = 0;
        double bd = std::numeric_limits<double>::max();
        for (size_t i = 0; i < ring.size(); ++i) {
            double d = (ring[i] - q).norm();
            if (d < bd) { bd = d; bi = i; }
        }
        return bi;
    };
    if ((pts[0] - pts[n - 1]).norm() <= 1e-6 * std::max(len, 1e-9)) {
        // closed basis curve: walk the ring forward from a to b through the curve's seam
        std::vector<Vec3> ring(pts.begin(), pts.end() - 1);
        const size_t m = ring.size();
        const size_t ia = nearest(a, ring);
        size_t span;
        if ((a - b).norm() <= tol) {
            span = m;  // closed edge: the full ring, re-seamed at the vertex
        } else {
            const size_t ib = nearest(b, ring);
            if (ia == ib) return pts;  // degenerate trim: keep the old behaviour
            span = (ib + m - ia) % m;
        }
        std::vector<Vec3> out;
        out.reserve(span + 1);
        for (size_t k = 0; k <= span; ++k) out.push_back(ring[(ia + k) % m]);
        return out;
    }
    // open curve trimmed to an interior stretch
    const size_t ia = nearest(a, pts), ib = nearest(b, pts);
    if (ia < ib) return std::vector<Vec3>(pts.begin() + ia, pts.begin() + ib + 1);
    return pts;  // vertices against the parameter direction: leave as-is
}

// One oriented edge of a loop. `start`/`end` are the EDGE_CURVE endpoints with the ORIENTED_EDGE
// orientation already applied (decode). `e_start`/`e_end` are the raw EDGE_CURVE endpoints and
// `same_sense`/`orientation` the flags — needed because a *closed* circle/ellipse edge (start==end)
// cannot recover its traversal direction from endpoints. `geometry` is the basis curve (null =>
// straight segment).
struct OrientedEdgeN {
    Vec3 start, end;
    Vec3 e_start, e_end;  // raw EDGE_CURVE endpoints (pre-orientation)
    std::shared_ptr<Curve> geometry;
    bool has_params = false;
    bool same_sense = true;   // EDGE_CURVE.same_sense
    bool orientation = true;  // ORIENTED_EDGE.orientation
    double t_start = 0, t_end = 0;

    // Ordered points start->end along the edge (endpoints included). Circle/ellipse edges are a
    // faithful port of step2glb model.rs edge_polyline+curve_polyline: rebuild the arc CCW from
    // (a,b) = same_sense?(e_start,e_end):(e_end,e_start), then reverse for !same_sense and for
    // !orientation. This is the analog of step2glb's STEP->geometry layer (NOT tessellate.rs); the
    // ported tessellator stays untouched. Other curves keep the stored-param + endpoint-snap path.
    std::vector<Vec3> discretize(double deflection, double max_angle) const {
        const Curve *g = geometry.get();
        // Curve types that rebuild from the endpoint vertices + edge flags (step2glb edge_polyline/
        // curve_polyline): an arc/conic derives its parameter range by projecting the endpoints;
        // a polyline/composite carries its own point chain. All then honour same_sense + orientation.
        Vec3 a = same_sense ? e_start : e_end;
        Vec3 b = same_sense ? e_end : e_start;
        std::vector<Vec3> pts;
        bool handled = true;
        if (const auto *cc = dynamic_cast<const CircleCurve *>(g)) {
            pts = discretize_arc(cc->f, cc->r, cc->r, a, b, deflection, max_angle);
        } else if (const auto *ec = dynamic_cast<const EllipseCurve *>(g)) {
            pts = discretize_arc(ec->f, ec->a1, ec->a2, a, b, deflection, max_angle);
        } else if (const auto *hc = dynamic_cast<const HyperbolaCurve *>(g)) {
            pts = hc->discretize(hc->param(a), hc->param(b), deflection, max_angle);
            if (!pts.empty()) { pts.front() = a; pts.back() = b; }
        } else if (const auto *pc = dynamic_cast<const ParabolaCurve *>(g)) {
            pts = pc->discretize(pc->param(a), pc->param(b), deflection, max_angle);
            if (!pts.empty()) { pts.front() = a; pts.back() = b; }
        } else if (const auto *pl = dynamic_cast<const PolylineCurve *>(g)) {
            pts = pl->pts;
        } else if (const auto *co = dynamic_cast<const CompositeCurveN *>(g)) {
            pts = co->chain(deflection, max_angle);
        } else if (g && g->uniform_edge_segments() > 0) {
            // step2glb sample_bspline_to_polyline: sample the basis curve over its FULL natural
            // parametric domain (edges are trimmed to interior stretches, so the trim params do NOT
            // bound the sampling — using them collapses untrimmed-param edges to a 2-point chord),
            // then align to the edge's vertices a,b (same_sense-adjusted, exactly as step2glb
            // curve_polyline(a,b)); the unified same_sense + orientation reversals below then match
            // step2glb edge_polyline's post-reversals. (Aligning to the orientation-applied
            // start/end instead trims against the wrong vertices and over-samples some edges.)
            int useg = g->uniform_edge_segments();
            double lo, hi, period;
            bool periodic;
            g->range(lo, hi, periodic, period);
            std::vector<Vec3> full;
            full.reserve(useg + 1);
            for (int i = 0; i <= useg; ++i) full.push_back(g->point(lo + (hi - lo) * i / useg));
            pts = align_polyline_to_vertices(full, a, b);
            if (pts.size() >= 2) { pts.front() = a; pts.back() = b; }
        } else {
            handled = false;
        }
        if (handled) {
            if (pts.size() < 2) return {start, end};
            if (!same_sense) std::reverse(pts.begin(), pts.end());
            if (!orientation) std::reverse(pts.begin(), pts.end());
            return pts;
        }
        // generic parametric curve with explicit trim params (no uniform sampler)
        if (!geometry || !has_params) return {start, end};
        pts = geometry->discretize(t_start, t_end, deflection, max_angle);
        if (pts.size() < 2) return {start, end};
        pts.front() = start;  // snap to the topological vertices
        pts.back() = end;
        // orient to go from `start` to `end`
        if ((pts.front() - start).norm() > (pts.back() - start).norm())
            std::reverse(pts.begin(), pts.end());
        return pts;
    }
};

// A boundary loop: either an edge loop (curved/line edges) or a literal polygon.
struct LoopN {
    bool is_poly = false;
    std::vector<OrientedEdgeN> edges;  // when !is_poly
    std::vector<Vec3> polygon;         // when is_poly

    // Ordered closed polyline of the whole loop (shared vertices de-duplicated).
    std::vector<Vec3> discretize(double deflection, double max_angle) const {
        if (is_poly) return polygon;
        std::vector<Vec3> out;
        for (const auto &e : edges) {
            std::vector<Vec3> ep = e.discretize(deflection, max_angle);
            for (const Vec3 &p : ep) {
                if (out.empty() || (p - out.back()).norm() > 1e-9) out.push_back(p);
            }
        }
        // drop a closing duplicate of the first vertex
        if (out.size() > 1 && (out.front() - out.back()).norm() <= 1e-9) out.pop_back();
        return out;
    }
};

struct FaceBoundN {
    std::shared_ptr<LoopN> loop;
    bool orientation = true;  // false => reverse the loop's winding relative to the face
};

struct FaceSurfaceN {
    std::shared_ptr<Surface> surface;
    bool same_sense = true;  // false => face normal is the surface normal flipped
    std::vector<FaceBoundN> bounds;  // bounds[0] outer, rest holes
};

struct ConnectedFaceSetN {
    std::vector<std::shared_ptr<FaceSurfaceN>> faces;
};

// An extruded-area solid: a planar profile face swept along `direction` by
// `depth`, then placed by `frame`. Mapped to ifcopenshell taxonomy::extrusion.
struct ExtrusionN {
    std::shared_ptr<FaceSurfaceN> profile;  // planar profile face (local XY, z=0)
    Frame frame;                            // placement of the swept solid
    Vec3 direction{0, 0, 1};                // local extrusion direction
    double depth = 0;
};

// A revolved-area solid: a planar profile face revolved about (axis_origin,
// axis_dir) by `angle`, then placed by `frame`. Meshed via OCC MakeRevol
// (ifcopenshell's revolve convert derefs a null schema instance -> segfault).
struct RevolveN {
    std::shared_ptr<FaceSurfaceN> profile;
    Frame frame;               // placement of the swept solid
    Vec3 axis_origin{0, 0, 0};
    Vec3 axis_dir{0, 0, 1};
    double angle = 0;          // 0 => full revolution
};

// One top-level streamed Geometry instance + its stable id (for Mesh grouping).
struct NgeomRoot {
    std::string id;
    std::vector<std::shared_ptr<FaceSurfaceN>> faces;  // flattened (a CFS expands to its faces)
    std::shared_ptr<ExtrusionN> extrusion;             // set if this root is an extruded solid
    std::shared_ptr<RevolveN> revolve;                 // set if this root is a revolved solid
};

struct NgeomDoc {
    std::vector<NgeomRoot> roots;
};

}  // namespace adacpp::ngeom
