// NGEOM neutral topology — edges, loops, face-surfaces (spec §6). The decoded form the
// tessellator consumes. A loop knows how to discretize itself into an ordered 3D polyline.
#pragma once

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ngeom_curves.h"
#include "ngeom_math.h"
#include "ngeom_surfaces.h"

namespace adacpp::ngeom {

// Discretize a circular/elliptical arc between two 3D endpoints by projecting them onto the
// curve to recover the angular parameter range — a port of model.rs curve_polyline's
// CIRCLE/ELLIPSE arm. The reference never trusts stored edge parameters; it always rebuilds the arc
// from (frame, radii, vertex_a, vertex_b), handling the closed (a==b → full 2pi) case. We mirror
// that so a closed-circle edge whose `has_params` was dropped upstream doesn't collapse to a
// single point (and drop its whole face).
inline std::vector<Vec3> discretize_arc(const Frame &f, double rx, double ry, const Vec3 &a, const Vec3 &b,
                                        double deflection, double max_angle) {
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
        while (t1 <= t0 + 1e-12)
            t1 += TWO_PI;
    }
    double step = angle_step(std::max(std::abs(rx), std::abs(ry)), deflection, max_angle);
    int nseg = std::max(2, (int) std::ceil((t1 - t0) / step));
    std::vector<Vec3> pts;
    pts.reserve(nseg + 1);
    for (int i = 0; i <= nseg; ++i) {
        double t = t0 + (t1 - t0) * i / nseg;
        pts.push_back(f.to_world(rx * std::cos(t), ry * std::sin(t), 0.0));
    }
    pts.front() = a; // snap exactly to the topological vertices
    pts.back() = b;
    return pts;
}

// Faithful port of model.rs align_polyline_to_vertices: a basis curve is sampled over its
// FULL natural domain, but the edge trims it to an interior stretch between vertices a,b. Exporters
// close closed edges at a vertex away from the curve's own seam, so blindly snapping the curve's
// natural endpoints onto a,b would fold the polyline through long false chords. Instead: if the
// sampled endpoints already sit on a,b keep it; if the basis curve is closed, walk the ring forward
// from nearest(a) to nearest(b); else trim the open curve to [nearest(a), nearest(b)].
inline std::vector<Vec3> align_polyline_to_vertices(const std::vector<Vec3> &pts, const Vec3 &a, const Vec3 &b) {
    const size_t n = pts.size();
    if (n < 3)
        return pts;
    double step = 0.0, len = 0.0;
    for (size_t i = 1; i < n; ++i) {
        double d = (pts[i] - pts[i - 1]).norm();
        step = std::max(step, d);
        len += d;
    }
    const double tol = std::max(step, 1e-12);
    if ((pts[0] - a).norm() <= tol && (pts[n - 1] - b).norm() <= tol)
        return pts; // common case
    auto nearest = [](const Vec3 &q, const std::vector<Vec3> &ring) {
        size_t bi = 0;
        double bd = std::numeric_limits<double>::max();
        for (size_t i = 0; i < ring.size(); ++i) {
            double d = (ring[i] - q).norm();
            if (d < bd) {
                bd = d;
                bi = i;
            }
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
            span = m; // closed edge: the full ring, re-seamed at the vertex
        } else {
            const size_t ib = nearest(b, ring);
            if (ia == ib)
                return pts; // degenerate trim: keep the old behaviour
            span = (ib + m - ia) % m;
        }
        std::vector<Vec3> out;
        out.reserve(span + 1);
        for (size_t k = 0; k <= span; ++k)
            out.push_back(ring[(ia + k) % m]);
        return out;
    }
    // open curve trimmed to an interior stretch
    const size_t ia = nearest(a, pts), ib = nearest(b, pts);
    if (ia < ib)
        return std::vector<Vec3>(pts.begin() + ia, pts.begin() + ib + 1);
    if (ia > ib) {
        // Edge runs against the basis curve's sample direction: take the interior stretch
        // [ib, ia] and reverse it so the result still goes a -> b. Returning the FULL polyline
        // here made a trimmed straight B-spline edge (e.g. an ACIS intcurve boundary of a flat
        // plate) zig-zag back across its own domain -> a self-intersecting face boundary that
        // tess2 collapsed to a single triangle.
        std::vector<Vec3> sub(pts.begin() + ib, pts.begin() + ia + 1);
        std::reverse(sub.begin(), sub.end());
        return sub;
    }
    return std::vector<Vec3>{a, b}; // ia == ib: sub-range within one sample interval
}

// Chord-deflection simplification of a sampled polyline (Douglas-Peucker). The uniform B-spline
// edge sampler (uniform_edge_segments) emits a fixed cps*4 points REGARDLESS of tolerance, so a
// coarse `deflection` never coarsens a B-spline boundary — the dominant source of native's
// over-tessellation on freeform faces (a face's boundary alone can carry thousands of points, each
// forcing a CDT triangle the interior refinement then multiplies). Decimating the sampled polyline
// down to the chord tolerance makes the boundary honour `deflection` like OCC's edge discretization.
//
// Douglas-Peucker is used (not a greedy forward walk) because it is DIRECTION-SYMMETRIC: the two
// faces sharing a manifold edge discretize it in opposite orders, and DP keeps the same point SET
// either way (the recursive max-deviation split does not depend on traversal direction), so the
// shared edge stays watertight. Endpoints are always kept. tol<=0 disables (keeps every sample).
inline std::vector<Vec3> simplify_polyline_dp(const std::vector<Vec3> &p, double tol, double max_angle) {
    const size_t n = p.size();
    if (n <= 2 || tol <= 0.0)
        return p;
    const double cos_ang = (max_angle > 0.0) ? std::cos(max_angle) : -2.0; // -2 => angle test off
    std::vector<char> keep(n, 0);
    keep[0] = keep[n - 1] = 1;
    // explicit stack of [i,j] ranges whose chord approximates the interior within tol or gets split
    std::vector<std::pair<size_t, size_t>> stk;
    stk.emplace_back(0, n - 1);
    while (!stk.empty()) {
        auto [i, j] = stk.back();
        stk.pop_back();
        if (j <= i + 1)
            continue;
        const Vec3 A = p[i], B = p[j];
        const Vec3 ab = B - A;
        const double abl2 = ab.dot(ab);
        double dmax = -1.0;
        size_t im = i;
        for (size_t k = i + 1; k < j; ++k) {
            const Vec3 d = p[k] - A;
            double sag;
            if (abl2 < 1e-24)
                sag = d.norm(); // degenerate chord (closed edge): distance to the shared endpoint
            else {
                const double t = d.dot(ab) / abl2; // UNCLAMPED perpendicular distance (standard DP)
                sag = (d - ab * t).norm();
            }
            if (sag > dmax) {
                dmax = sag;
                im = k;
            }
        }
        // Angular criterion (keeps curved boundaries — e.g. a tube's end ring — ROUND rather than
        // letting a within-sag chord octagonalize them, matching OCC's edge angular deflection). The
        // turn across the chord is measured from the ORIGINAL segments at each end, so it is
        // reversal-symmetric; the split point (max perpendicular deviation) is symmetric too, so the
        // decimated point SET is identical whichever direction the two adjacent faces walk the edge.
        bool angle_split = false;
        if (cos_ang > -1.5) {
            const Vec3 d0 = p[i + 1] - p[i];
            const Vec3 d1 = p[j] - p[j - 1];
            const double l0 = d0.norm(), l1 = d1.norm();
            if (l0 > 1e-12 && l1 > 1e-12 && d0.dot(d1) / (l0 * l1) < cos_ang)
                angle_split = true;
        }
        if ((dmax > tol || angle_split) && im > i) {
            keep[im] = 1;
            stk.emplace_back(i, im);
            stk.emplace_back(im, j);
        }
    }
    std::vector<Vec3> out;
    out.reserve(n);
    for (size_t k = 0; k < n; ++k)
        if (keep[k])
            out.push_back(p[k]);
    return out;
}

// One oriented edge of a loop. `start`/`end` are the EDGE_CURVE endpoints with the ORIENTED_EDGE
// orientation already applied (decode). `e_start`/`e_end` are the raw EDGE_CURVE endpoints and
// `same_sense`/`orientation` the flags — needed because a *closed* circle/ellipse edge (start==end)
// cannot recover its traversal direction from endpoints. `geometry` is the basis curve (null =>
// straight segment).
struct OrientedEdgeN {
    Vec3 start, end;
    Vec3 e_start, e_end; // raw EDGE_CURVE endpoints (pre-orientation)
    std::shared_ptr<Curve> geometry;
    bool has_params = false;
    bool same_sense = true;  // EDGE_CURVE.same_sense
    bool orientation = true; // ORIENTED_EDGE.orientation
    double t_start = 0, t_end = 0;
    // Source EDGE_CURVE slot ref (ORIENTED_EDGE.edge_element), -1 when unknown. The two faces
    // sharing a manifold edge carry the SAME edge_id with opposite `orientation` — an exact
    // integer identity for the shared edge, without quantizing endpoints.
    int edge_id = -1;

    // Ordered points start->end along the edge (endpoints included). Circle/ellipse edges are a
    // faithful port of model.rs edge_polyline+curve_polyline: rebuild the arc CCW from
    // (a,b) = same_sense?(e_start,e_end):(e_end,e_start), then reverse for !same_sense and for
    // !orientation. This is the analog of STEP->geometry layer (NOT tessellate.rs); the
    // ported tessellator stays untouched. Other curves keep the stored-param + endpoint-snap path.
    std::vector<Vec3> discretize(double deflection, double max_angle) const {
        const Curve *g = geometry.get();
        // Curve types that rebuild from the endpoint vertices + edge flags (edge_polyline/
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
            if (!pts.empty()) {
                pts.front() = a;
                pts.back() = b;
            }
        } else if (const auto *pc = dynamic_cast<const ParabolaCurve *>(g)) {
            pts = pc->discretize(pc->param(a), pc->param(b), deflection, max_angle);
            if (!pts.empty()) {
                pts.front() = a;
                pts.back() = b;
            }
        } else if (const auto *pl = dynamic_cast<const PolylineCurve *>(g)) {
            pts = pl->pts;
        } else if (const auto *co = dynamic_cast<const CompositeCurveN *>(g)) {
            pts = co->chain(deflection, max_angle);
        } else if (g && g->uniform_edge_segments() > 0) {
            // sample_bspline_to_polyline: sample the basis curve over its FULL natural
            // parametric domain (edges are trimmed to interior stretches, so the trim params do NOT
            // bound the sampling — using them collapses untrimmed-param edges to a 2-point chord),
            // then align to the edge's vertices a,b (same_sense-adjusted, exactly as
            // curve_polyline(a,b)); the unified same_sense + orientation reversals below then match
            // edge_polyline's post-reversals. (Aligning to the orientation-applied
            // start/end instead trims against the wrong vertices and over-samples some edges.)
            int useg = g->uniform_edge_segments();
            double lo, hi, period;
            bool periodic;
            g->range(lo, hi, periodic, period);
            std::vector<Vec3> full;
            full.reserve(useg + 1);
            for (int i = 0; i <= useg; ++i)
                full.push_back(g->point(lo + (hi - lo) * i / useg));
            pts = align_polyline_to_vertices(full, a, b);
            // Loop assembly needs each edge to run start->end (the oriented topological
            // endpoints). The a/b alignment + the common same_sense/orientation reversals below
            // mis-ordered a trimmed intcurve boundary edge (an ACIS flat plate's B-spline side ran
            // end->start), scrambling the face boundary into a self-intersecting loop that tess2
            // collapsed to one triangle. Emit start->end directly and return, bypassing those
            // reversals (the align clip above already dropped the out-of-trim interior samples).
            if (pts.size() < 2)
                return {start, end};
            // CLOSED edge (start vertex == end vertex, e.g. the full B-spline circle rim of a
            // closed NURBS tube): the endpoint-distance test below is a tie (front == back ==
            // start), so a reversed (.F.) closed edge silently kept the curve's NATURAL direction.
            // Both rim circles of such a tube then wound the SAME way in u, the face's single loop
            // unwound to |w| = 2, and tessellate_periodic_winding (which requires |w| == 1) dropped
            // the whole face (audit fe84a414: every STEP dropped-face bucket was this). Direction
            // cannot come from the endpoints of a closed edge — apply the edge flags exactly as the
            // circle/ellipse arc path does: reverse iff same_sense XOR orientation (align emitted
            // the ring a->b, i.e. already same_sense-adjusted, so only the NET flag parity remains).
            // "Closed" = the edge's own vertices coincide (usually the SAME VERTEX_POINT, so the
            // distance is exactly 0; 1e-7 x edge length tolerates exporters that write two equal
            // vertices). A nearly-closed OPEN edge (C-slit) keeps the endpoint-distance test.
            double _len = 0.0;
            for (size_t i = 1; i < pts.size(); ++i)
                _len += (pts[i] - pts[i - 1]).norm();
            if ((e_start - e_end).norm() <= std::max(1e-7 * _len, 1e-12)) {
                if (same_sense != orientation)
                    std::reverse(pts.begin(), pts.end());
            } else if ((pts.front() - start).norm() > (pts.back() - start).norm()) {
                std::reverse(pts.begin(), pts.end());
            }
            pts.front() = start;
            pts.back() = end;
            // Decimate the fixed-density B-spline sample to the chord tolerance (see
            // simplify_polyline_dp): this is what makes a B-spline boundary respond to `deflection`.
            pts = simplify_polyline_dp(pts, deflection, max_angle);
            return pts;
        } else {
            handled = false;
        }
        if (handled) {
            if (pts.size() < 2)
                return {start, end};
            if (!same_sense)
                std::reverse(pts.begin(), pts.end());
            if (!orientation)
                std::reverse(pts.begin(), pts.end());
            return pts;
        }
        // generic parametric curve with explicit trim params (no uniform sampler)
        if (!geometry || !has_params)
            return {start, end};
        pts = geometry->discretize(t_start, t_end, deflection, max_angle);
        if (pts.size() < 2)
            return {start, end};
        pts.front() = start; // snap to the topological vertices
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
    std::vector<OrientedEdgeN> edges; // when !is_poly
    std::vector<Vec3> polygon;        // when is_poly

    // Ordered closed polyline of the whole loop (shared vertices de-duplicated).
    std::vector<Vec3> discretize(double deflection, double max_angle) const {
        if (is_poly)
            return polygon;
        // Discretize each edge, then re-chain them end-to-end by nearest endpoint (flipping an edge
        // when its tail, not its head, meets the running chain). Concatenating in stored order
        // assumes every edge already runs head-to-tail, but ACIS loops of mixed Line / trimmed-
        // intcurve edges can arrive with inconsistent per-edge direction (and order) — blind
        // concatenation then self-intersects and tess2 collapses the face to a single triangle.
        // For an already-consistent loop each next edge's head is the chain tail, so this reproduces
        // the old plain concatenation (no reorder, no flip).
        std::vector<std::vector<Vec3>> segs;
        segs.reserve(edges.size());
        for (const auto &e : edges) {
            std::vector<Vec3> ep = e.discretize(deflection, max_angle);
            if (ep.size() >= 2)
                segs.push_back(std::move(ep));
        }
        if (segs.empty())
            return {};

        auto append = [](std::vector<Vec3> &out, const std::vector<Vec3> &s) {
            for (const Vec3 &p : s)
                if (out.empty() || (p - out.back()).norm() > 1e-9)
                    out.push_back(p);
        };

        std::vector<char> used(segs.size(), 0);
        std::vector<Vec3> out = segs[0];
        used[0] = 1;
        for (size_t placed = 1; placed < segs.size(); ++placed) {
            const Vec3 tail = out.back();
            size_t best = segs.size();
            bool flip = false;
            double bd = std::numeric_limits<double>::max();
            for (size_t i = 0; i < segs.size(); ++i) {
                if (used[i])
                    continue;
                double df = (segs[i].front() - tail).norm();
                double db = (segs[i].back() - tail).norm();
                if (df < bd) {
                    bd = df;
                    best = i;
                    flip = false;
                }
                if (db < bd) {
                    bd = db;
                    best = i;
                    flip = true;
                }
            }
            if (best == segs.size())
                break;
            used[best] = 1;
            if (flip) {
                std::vector<Vec3> rev(segs[best].rbegin(), segs[best].rend());
                append(out, rev);
            } else {
                append(out, segs[best]);
            }
        }
        // drop a closing duplicate of the first vertex
        if (out.size() > 1 && (out.front() - out.back()).norm() <= 1e-9)
            out.pop_back();
        return out;
    }
};

struct FaceBoundN {
    std::shared_ptr<LoopN> loop;
    bool orientation = true; // false => reverse the loop's winding relative to the face
};

struct FaceSurfaceN {
    std::shared_ptr<Surface> surface;
    bool same_sense = true;         // false => face normal is the surface normal flipped
    std::vector<FaceBoundN> bounds; // bounds[0] outer, rest holes
    // Source entity id (STEP ADVANCED_FACE / IFC IfcFace #id), 0 when unknown. Only used when the
    // caller requests per-face clickable regions (TessParams.capture_face_ranges) — lets the viewer
    // report the exact source id of a clicked face. Costs nothing otherwise.
    int64_t src_id = 0;
    // The `surface` is a PLACEHOLDER identity plane (a plain IfcFace / explicit face-set polygon whose
    // real 3D plane is only implied by its boundary points). The tessellator must fit the plane from
    // the 3D loop up front — otherwise the poly projects flat onto the z=0 placeholder plane.
    bool fit_plane_from_loop = false;
    // Per-face presentation colour (STEP OVER_RIDING_STYLED_ITEM -> COLOUR_RGB, keyed on this face's
    // ADVANCED_FACE #id). Mirrors NgeomRoot's per-solid colour. has_color=false => the face inherits the
    // owning solid's base colour. Populated by the native STEP reader (step_reader.h face()); the neutral
    // decoder / IFC path leave it unset.
    bool has_color = false;
    float cr = 0.5f, cg = 0.5f, cb = 0.5f, ca = 1.0f; // rgba in 0..1 when has_color
};

struct ConnectedFaceSetN {
    std::vector<std::shared_ptr<FaceSurfaceN>> faces;
};

// An extruded-area solid: a planar profile face swept along `direction` by
// `depth`, then placed by `frame`. Mapped to ifcopenshell taxonomy::extrusion.
struct ExtrusionN {
    std::shared_ptr<FaceSurfaceN> profile; // planar profile face (local XY, z=0)
    Frame frame;                           // placement of the swept solid
    Vec3 direction{0, 0, 1};               // local extrusion direction
    double depth = 0;
};

// A revolved-area solid: a planar profile face revolved about (axis_origin,
// axis_dir) by `angle`, then placed by `frame`. Meshed via OCC MakeRevol
// (ifcopenshell's revolve convert derefs a null schema instance -> segfault).
struct RevolveN {
    std::shared_ptr<FaceSurfaceN> profile;
    Frame frame; // placement of the swept solid
    Vec3 axis_origin{0, 0, 0};
    Vec3 axis_dir{0, 0, 1};
    double angle = 0; // 0 => full revolution
};

// A fixed-reference swept-area solid (IfcFixedReferenceSweptAreaSolid over an alignment directrix):
// a planar profile face swept along a precomputed field of per-station frames. A profile point
// (u, v) at station j maps to the local point `origin[j] + u*dir_x[j] + v*dir_y[j]`, then placed by
// `frame`. The analytic directrix (line/clothoid/arc + vertical gradient) is evaluated producer-side
// (adapy) into these frames, so the kernel only rings + caps them (no OCC, no Frenet roll here).
struct SweepN {
    std::shared_ptr<FaceSurfaceN> profile; // planar profile face (local UV, z=0)
    Frame frame;                           // placement of the swept solid
    std::vector<Vec3> origin;              // per-station directrix point (local)
    std::vector<Vec3> dir_x;               // per-station profile local-x axis in 3D (fixed-ref "up")
    std::vector<Vec3> dir_y;               // per-station profile local-y axis in 3D (lateral)
};

struct BooleanN; // fwd (recursive)

// One operand of a boolean (or a root): any of the supported solids.
struct SolidItemN {
    std::shared_ptr<ExtrusionN> extrusion;
    std::shared_ptr<RevolveN> revolve;
    std::shared_ptr<SweepN> sweep;
    std::shared_ptr<BooleanN> boolean;
    std::vector<std::shared_ptr<FaceSurfaceN>> faces; // shell operand
};

// A CSG boolean: op(a, b). op 0=difference (cut), 1=union (fuse), 2=intersection (common).
struct BooleanN {
    int op = 0;
    SolidItemN a, b;
};

// A sphere primitive (centre = frame.o, radius). Taxonomy has no sphere solid, so it's
// meshed analytically (libtess2) / via BRepPrimAPI_MakeSphere (occ/cgal).
struct SphereN {
    Frame frame;
    double radius = 0;
};

// One top-level streamed Geometry instance + its stable id (for Mesh grouping).
struct NgeomRoot {
    std::string id;
    std::vector<std::shared_ptr<FaceSurfaceN>> faces; // flattened (a CFS expands to its faces)
    std::shared_ptr<ExtrusionN> extrusion;            // set if this root is an extruded solid
    std::shared_ptr<RevolveN> revolve;                // set if this root is a revolved solid
    std::shared_ptr<SweepN> sweep;                    // set if this root is a fixed-ref swept solid
    std::shared_ptr<BooleanN> boolean;                // set if this root is a boolean result
    std::shared_ptr<SphereN> sphere;                  // set if this root is a sphere primitive
    std::vector<std::vector<Vec3>> polylines;         // set if this root is a curve-only body (GL_LINES)
    // The product's representation WAS recognized but resolved to no drawable geometry because it is
    // degenerate (e.g. a zero-length DISCONTINUOUS IfcCurveSegment — a topological end-marker the
    // reference kernel also refuses). Distinguishes "intentionally empty" from "unsupported": the
    // stream must NOT count these as products_skipped (they'd else drive a pointless OCC fallback).
    bool recognized_empty = false;
    // Presentation colour (STYLED_ITEM -> COLOUR_RGB), populated by the native STEP reader; the
    // NGEOM byte decoder leaves has_color=false (colour travels out-of-band on that path).
    bool has_color = false;
    float cr = 0.5f, cg = 0.5f, cb = 0.5f, ca = 1.0f; // rgba in 0..1 when has_color
    // Per-instance world placement matrices (column-major, glTF order), from the assembly
    // transform graph. Empty => a single identity instance (flat/baked files). N entries => the
    // solid is placed N times. Populated by the native STEP reader only.
    std::vector<std::array<float, 16>> transforms;
    // Per-instance assembly path: root-first (rep_id, product_name) levels (last level = the
    // solid's own product), parallel to transforms. Empty => the solid sits directly under the
    // assembly root. Populated by the native STEP reader for the from_step part hierarchy.
    std::vector<std::vector<std::pair<int, std::string>>> instance_paths;
};

struct NgeomDoc {
    std::vector<NgeomRoot> roots;
    double unit_scale = 1.0; // file length unit -> metres (e.g. mm -> 0.001), from LENGTH_UNIT
};

} // namespace adacpp::ngeom
