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
    Cdt = 1,         // Constrained Delaunay for EVERY face: boundary as constraint edges, interior
                     // (incl. the surface grid) as Steiner points. One path, boundary-first.
                     // Watertight, but ~+70% on the crane — see CdtOpts.
    Hybrid = 5,      // libtess2+pin everywhere, detria ONLY for the faces libtess2 cannot pin.
                     // The intended default candidate — see HybridOpts.
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

    // Replace the UV-bbox grid on full-patch faces with an INSET grid whose one-cell gap to the
    // trim loop is zippered by a ring of triangles, with the loop's own vertices as the outer ring —
    // so they pin. Keeps the grid's cheap structured interior (the right algorithm for a full patch)
    // and fixes its only defect: a boundary at uniform parameters that cannot pin.
    //
    // The alternative, routing these faces to detria, is correct but ~3-4x the grid's per-face cost:
    // on the crane 10.4% of faces (31,076 full-patch B-splines) cost +25.6% that way.
    //
    // OFF — FAILED ITS GATE (2026-07-15), kept only so the negative result is reproducible.
    //
    // The gate was: reach Ventilator's 0 open edges AND beat hybrid-cdt's +25.6% on the crane.
    // Measured on Ventilator (pin 9,880 edges / 9 non-manifold / 3.37s; cdt 0 / 0 / 3.70s):
    //   annulus          6,948 edges   73 non-manifold   3.85s   325,302 tris
    //   annulus + freeze 6,011 edges   79 non-manifold   3.90s   329,669 tris
    // It closes ~39% of the cracks but ADDS ~70 non-manifold edges, i.e. the zipper strip is
    // malformed (self-overlapping, not merely inverted), and it is already slower than cdt while
    // strictly worse. The crane was not measured: a wrong mesh's cost is not information.
    //
    // NB the inset (one full cell) DID fix 2026-07-13's stated failure mode — that annulus stitched
    // to a loop lying essentially ON the rect border, a zero-width strip of slivers. So the idea is
    // not refuted by the same mechanism twice; this is a new, unfixed defect in the strip itself.
    // Anyone resuming: the strip's winding was inverted relative to the grid and is now fixed, but
    // that changed NOTHING in the metrics because mesh_health sorts edge endpoints and is
    // winding-agnostic — the remaining defect is topological. Start by extracting ONE annulus face
    // and checking its strip in isolation, rather than by reading whole-model counts.
    bool annulus_patch = false;

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

// Options for the hybrid track: libtess2+pin everywhere, detria only where libtess2 CANNOT pin.
//
// WHY. Measured, both tracks on both models:
//
//                    Ventilator (1 solid, 305 faces)     crane (7096 solids, ~299k faces)
//   libtess2+pin     +0%   9,880 open edges              +0.5%   RSS -1.8%
//   cdt              +7.6%     0 open edges  WATERTIGHT  +70.2%  RSS +19.0%
//
// cdt wins outright on Ventilator and is unaffordable on the crane. The reason is face MIX, not
// model size: the crane is ~299k mostly-small quadric faces where libtess2's sweep is fast and a
// full CDT (points + constraints + erase) is pure overhead per face — while pinning already makes
// those faces watertight-equivalent, because tess2 tessellates their real trim loop.
//
// The faces that actually NEED detria are the ones libtess2 hands to tessellate_uv_grid: it fills
// the UV bbox, never sees the trim loop, and so has nothing to pin (53/305 = 17% on Ventilator).
// Route by that, not by model, and the cost lands where the benefit is.
struct HybridOpts {
    // Send would-be full-patch grid faces to detria. full_patch_rect's gate is area_ratio >= 0.995,
    // i.e. the trim loop IS the UV rectangle — which is exactly why detria can replace the grid for
    // them: same region, but with a boundary it can constrain and pin. This is the one grid path
    // proven to work through detria (it is all 53 of Ventilator's, and they went watertight).
    bool cdt_full_patch = true;
    // The OTHER grid paths (full_wrap / full_domain / v-pole) are deliberately NOT routed. Their
    // gates do not guarantee the loop bounds the region — full_domain_bspline gates on bbox EXTENT
    // >= 0.9 with no area check, so the loop can cover 90% of the extent at 50% of the area, and a
    // wrapped/poled loop is not a simple polygon at all. Handing those to a CDT would not just fail,
    // it could triangulate the wrong region. They stay on the grid and stay unpinnable — an honest
    // residual on models that use them. Ventilator uses none.
    bool cdt_wrap_and_pole = false;
};

// ---- self-declaration: adacpp owns the track vocabulary -------------------------------------
//
// adapy must not hardcode which tracks exist — it asks. This is the single source of truth, and it
// reports only what THIS BUILD can actually run (the taxonomy tracks are compiled out of the wasm
// branch, so they are not advertised there). Consumers publish the list onward to the frontend and
// to Python users; adding a track here is the whole change.
struct TessTrackInfo {
    const char *name;        // stable token, e.g. "cdt" — what callers pass back as `pipeline`
    const char *label;       // human-readable, for a UI dropdown
    const char *description; // one line, for a tooltip
    bool watertight;         // does it close shared-edge seams?
    bool is_default;         // the track used when no pipeline is given
};

inline std::vector<TessTrackInfo> track_infos() {
    std::vector<TessTrackInfo> out = {
        {"libtess2", "libtess2 (default)",
         "OCC-free winding-rule tessellation of the trim loop, with a UV-grid fast path for "
         "near-full patches. Boundary vertices are pinned to their shared edge, which halves "
         "shared-edge cracks at no cost.",
         false, true},
        {"cdt", "CDT — watertight (slower)",
         "Constrained Delaunay (detria): trim loops become constraint edges and the surface grid "
         "becomes interior Steiner points. One boundary-first path, so every boundary vertex pins. "
         "Produces watertight, manifold solids with fewer triangles, but costs ~+8% on small models "
         "and can exceed +60% on large multi-solid assemblies.",
         true, false},
        {"hybrid-cdt", "Hybrid CDT (experimental)",
         "libtess2 everywhere, CDT only for faces libtess2 cannot pin. Measured worse than either "
         "on both axes; retained as evidence, not recommended.",
         false, false},
    };
#if !defined(__EMSCRIPTEN__)
    // Taxonomy kernels link OCCT/CGAL via ifcopenshell and are excluded from the wasm build, so they
    // are only real where they are compiled in.
    out.push_back({"occ", "OCC (ifcopenshell taxonomy)", "Meshes via ifcopenshell's taxonomy on OCCT.",
                   false, false});
    out.push_back({"cgal", "CGAL (ifcopenshell taxonomy)", "Meshes via ifcopenshell's taxonomy on CGAL.",
                   false, false});
    out.push_back({"hybrid", "Hybrid OCC/CGAL (ifcopenshell taxonomy)",
                   "ifcopenshell's hybrid OCCT/CGAL kernel selection.", false, false});
#endif
    return out;
}

inline std::optional<TessTrack> parse_track(std::string_view s) {
    if (s.empty() || s == "libtess2")
        return TessTrack::Libtess2;
    if (s == "cdt")
        return TessTrack::Cdt;
    if (s == "hybrid-cdt")
        return TessTrack::Hybrid;
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
    case TessTrack::Hybrid:
        return "hybrid-cdt";
    case TessTrack::TaxonomyOcc:
        return "occ";
    case TessTrack::TaxonomyCgal:
        return "cgal";
    case TessTrack::TaxonomyHybrid:
        return "hybrid";
    }
    return "?";
}

// Every selectable track name, derived from track_infos() so the two cannot drift.
inline std::vector<std::string_view> track_names() {
    std::vector<std::string_view> out;
    for (const TessTrackInfo &t : track_infos())
        out.push_back(t.name);
    return out;
}

} // namespace adacpp::ngeom
