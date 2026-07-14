// Selectable tessellation tracks for the NGEOM (OCC-free) path.
//
// Spec: dap/plan/v3/spec_tess_tracks_watertight.md.
//
// A TRACK is a tessellator. Each track carries its own options struct on TessParams (which is
// already copied to every internal, so adding fields here costs no signature churn). All option
// structs are POD, keeping TessParams trivially copyable so the per-root/per-face copies stay free.
//
// Boundary PINNING is an option of the libtess2 track, not a track of its own: it is a ~3% quality
// toggle on the same tessellator, and the reference kernels treat it as the normal way to build a
// boundary rather than a mode. Default ON — see Libtess2Opts::pin_boundary for the numbers.
//
// WASM-SAFE: no OCC/CGAL/ifcopenshell includes. The taxonomy tracks are named here so the enum is
// universal, but only their DISPATCH is native-only (CMakeLists.txt excludes those sources from the
// wasm branch) — an OCC-backed track can never be a wasm default.
#pragma once

#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

namespace adacpp::ngeom {

enum class TessTrack : uint8_t {
    Libtess2 = 0,    // DEFAULT — libtess2 winding-rule tessellation of the trim loop, + a UV grid
                     // fast path for near-full patches. Byte-identical at default options.
    Cdt = 1,         // IN PROGRESS — constrained Delaunay: boundary as constraint edges, interior
                     // (incl. the surface grid) as Steiner points. One path, boundary-first. See
                     // CdtOpts for why this exists.
    TaxonomyOcc = 2, // native-only (ifcopenshell taxonomy -> OCC)
    TaxonomyCgal = 3,
    TaxonomyHybrid = 4,
};

// Options for the libtess2 track. Defaults are exactly today's shipped behaviour.
struct Libtess2Opts {
    // Emit a boundary vertex at its shared-edge point rather than at this face's own
    // surface.point(uv) re-projection. Both faces of an edge discretize it to byte-identical points,
    // so pinning makes their boundary vertices coincide and the per-solid weld stitches the seam.
    // Only POSITION is pinned — the normal stays per-face, preserving the seam crease.
    //
    // This is what the reference kernels do. OCC (BRepMesh_BaseMeshAlgo.cxx):
    //     registerNode(aCurve->GetPoint(i),   // 3D from the SHARED edge curve, verbatim
    //                  aPCurve->GetPoint(i))  // uv from THIS face's pcurve
    // It never evaluates the face surface at a boundary vertex. gmsh goes further: boundary MVertex*
    // objects are owned by the GEdge and reused by both faces (conformality by pointer identity).
    // truck pins identically via edge_map/boundary_map.
    //
    // ON by default: measured strictly better on every axis we track, at ~3% time.
    //
    //   ventilator  +3.0% time   cracks 19,067 -> 9,880 (-48%)   nonmanifold 12 -> 9   tris IDENTICAL
    //   crane       +3.5% time   solids 7291 = 7291              dropped 78 = 78       GLB -0.7%
    //
    // Triangle count is unchanged (pinning moves positions; it does not change refinement) and the
    // crane GLB gets SMALLER because pinned boundary vertices actually weld. No geometry is lost:
    // dropped_faces is identical at 78/300,394. The displacement introduced is <=0.012% of model
    // bbox — sub-visual, and far below the divergence the surfaces already have from their own edges.
    //
    // Set false for byte-compatibility with pre-2026-07-14 output.
    bool pin_boundary = true;

    // Never split an incidence-1 (boundary) edge in refine_uv. A split puts the new vertex on the
    // chord, off the shared edge, while the neighbour keeps the original segment -> T-junction.
    //
    // OFF — measured a bad trade: ~6 percentage points of extra crack reduction (pin -48% vs
    // pin+freeze -54%) for +23% (crane) to +64% (ventilator) time, and it quadruples nonmanifold
    // edges (9 -> 48). Pinning does essentially all the work; boundary-split T-junctions were never
    // the dominant crack source. The reference kernels have no analogue: they feed the boundary to a
    // CDT as a *constraint*, never split by construction — the effect for free, which is what the
    // Cdt track is for.
    bool freeze_boundary = false;

    // Stop refining once a pass marks fewer than 1/converged_frac of the triangles. 0 = off (run to
    // the tri budget). Only meaningful with freeze_boundary.
    //
    // Freezing slows triangle growth, so refine_uv's `tris > budget` exit fires far later and the
    // loop rescans the whole mesh chasing a few stragglers: 1431 passes / 949k triangles scanned vs
    // 812 / 578k unfrozen, for only 3.7% more marks — the same refinement over 76% more passes. Time
    // tracked triangles-scanned almost exactly (+64% vs +65%). This exit costs +1.4% triangles and
    // took freeze from +65% to +0.3% on ventilator. It did NOT generalize (crane conv=50 is +27%),
    // which is one reason freeze stays off.
    //
    // NB the prior attempt read the +34% as refinement "fighting" an edge-vs-surface density
    // mismatch. Densifying the boundary 2x/4x/8x was measured to not help at all, which ruled that
    // out — and with it the planned per-root edge cache, whose only purpose was to fund the freeze.
    double converged_frac = 50.0;

    // --- measured failures, kept OFF and reproducible; do not re-attempt without new insight ---

    // Grid boundary rows/cols at the trim loop's own UV parameters + pinned ring.
    // STRUCTURALLY IMPOSSIBLE: a tensor grid forces both opposite rows to share ONE line set, but
    // the v=v0 row's loop parameters differ from the v=v1 row's, so the union must contain both —
    // and every line from the OTHER side creates a ring vertex that is not a loop point, hence
    // unpinned, hence a NEW T-junction. It adds more unpinned ring vertices than it pins:
    //   guard=4 (all 53 faces bail)  311,696 tris   9,880 edges    9 nonmanifold
    //   guard=20                     665,012 tris  16,104 edges  181 nonmanifold
    //   guard=100                    669,090 tris  16,185 edges  181 nonmanifold
    bool conforming_grid = false;
    int conform_max_ratio = 4; // bail past this multiple of the uniform line count

    // Route near-rectangular patches through the pinned emit_uv_region instead of the UV grid.
    // Closes 67% of the residual but costs +44% tris and 481 nonmanifold (9 -> 481) because emit
    // over-refines a full patch — reproducing the 2026-07-13 rejection (+48% tris) even with pins.
    bool grid_via_emit = false;
};

// Options for the CDT track.
//
// WHY THIS TRACK EXISTS. libtess2 is a winding-rule polygon tessellator: tessAddContour +
// tessTesselate, no interior Steiner points. That forces a separate UV-grid fast path for near-full
// patches — and the grid path tessellates the UV bbox, never sees the trim loop, and so has nothing
// to pin. It is the ONLY remaining source of cracks (53/305 ventilator faces, 9,880 residual edges).
// All three ways to close it while keeping a grid are now measured dead: conforming grid
// (structurally impossible), grid_via_emit (+44% tris / 481 nonmanifold), annulus (over-refines).
//
// The reference kernels have no grid path at all. OCC classifier-filters grid points
// (getClassifier()->Perform(aPnt2d) == TopAbs_IN) and inserts them as Steiner points into a CDT
// whose boundary is already frontier constraints — one path, boundary-first, everything pins.
// truck does the same via spade. That is what this track is for.
struct CdtOpts {
    // Insert the surface's curvature grid as interior Steiner points (OCC's BRepMesh_Free nodes /
    // truck's parameter_division), classifier-filtered to the trim region. This is what replaces the
    // grid fast path rather than bolting a second one on.
    bool grid_steiner = true;
    // Pin boundary vertices to their shared-edge points. Unlike the libtess2 track this is free by
    // construction: constraint edges are never split, so a boundary vertex is always a loop vertex.
    bool pin_boundary = true;
    // Never split a boundary edge during post-triangulation refinement.
    //
    // NOT optional, and not the same trade as Libtess2Opts::freeze_boundary. detria guarantees the
    // constraint edges it returns are unsplit — but refine_uv runs AFTER, knows nothing about
    // constraints, and will happily split them, manufacturing the very T-junctions detria was chosen
    // to prevent. Without this the CDT track throws away its own invariant: measured 432 residual
    // boundary edges on Ventilator with every one of 305 faces on the CDT path (i.e. no grid path
    // left to blame).
    bool freeze_boundary = true;
    // Companion to freeze_boundary; see Libtess2Opts::converged_frac for why freezing without this
    // makes refine_uv rescan the mesh chasing stragglers.
    double converged_frac = 50.0;
};

inline std::optional<TessTrack> parse_track(std::string_view s) {
    if (s.empty() || s == "libtess2")
        return TessTrack::Libtess2;
    if (s == "cdt")
        return TessTrack::Cdt;
    if (s == "occ" || s == "taxonomy-occ")
        return TessTrack::TaxonomyOcc;
    if (s == "cgal" || s == "taxonomy-cgal")
        return TessTrack::TaxonomyCgal;
    if (s == "hybrid" || s == "taxonomy-hybrid")
        return TessTrack::TaxonomyHybrid;
    return std::nullopt;
}

inline const char *track_name(TessTrack t) {
    switch (t) {
    case TessTrack::Libtess2:
        return "libtess2";
    case TessTrack::Cdt:
        return "cdt";
    case TessTrack::TaxonomyOcc:
        return "occ";
    case TessTrack::TaxonomyCgal:
        return "cgal";
    case TessTrack::TaxonomyHybrid:
        return "hybrid";
    }
    return "?";
}

// Every selectable track name. adapy probes the binding docstrings for these, so keep them in sync.
inline std::vector<std::string_view> track_names() {
    return {"libtess2", "cdt", "occ", "cgal", "hybrid"};
}

} // namespace adacpp::ngeom
