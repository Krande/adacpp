// Selectable tessellation tracks for the NGEOM (OCC-free) path.
//
// Spec: dap/plan/v3/spec_tess_tracks_watertight.md.
//
// One selector, threaded through TessParams (which is already copied to every internal, so adding a
// field here costs no signature churn). Both fields are POD, keeping TessParams trivially copyable
// so the per-root/per-face copies stay free.
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
    Libtess2 = 0,           // DEFAULT — today's behaviour, byte-for-byte. Never regress this.
    Libtess2Watertight = 1, // shared-edge pinned + boundary-frozen
    TaxonomyOcc = 2,        // native-only (ifcopenshell taxonomy -> OCC)
    TaxonomyCgal = 3,       // native-only
    TaxonomyHybrid = 4,     // native-only
};

// The watertight track's OWN configuration (POD: no heap, trivially copyable). Read ONLY when
// track == Libtess2Watertight; every field is inert on the default track.
struct WatertightOpts {
    // Emit a boundary vertex at its shared-edge point rather than at this face's own
    // surface.point(uv) re-projection. Both faces of an edge discretize it to byte-identical points,
    // so pinning makes their boundary vertices coincide and the per-solid weld stitches the seam.
    // Measured displacement (Phase 0b): <=0.012% of model bbox — sub-visual.
    bool pin_boundary = true;
    // Never split an incidence-1 (boundary) edge in refine_uv: a split places the new vertex on the
    // chord, off the shared edge, while the neighbour keeps the original segment -> T-junction.
    //
    // OFF BY DEFAULT — measured to be a bad trade. It buys only ~6 percentage points of crack
    // reduction (pin alone: -48%; pin+freeze: -54%) and costs +23% (crane) to +64% (ventilator).
    // Pinning does essentially all the work; boundary-split T-junctions were never the dominant
    // crack source. Note the reference kernels (OCCT, truck, fornjot) have no analogue of this:
    // they feed the boundary to a CDT as a *constraint*, which is never split by construction, so
    // they get the effect for free rather than by suppressing a post-hoc refinement pass.
    bool freeze_boundary = false;
    // Route near-rectangular B-spline patches (tessellate_full_patch) through the pinned
    // emit_uv_region instead of the UV-bbox grid. The grid path is the ONLY source of residual
    // cracks: it tessellates the bbox, never sees the trim loop, and so has nothing to pin. On
    // Ventilator every unpinned face (53/305) takes this branch, so this closes all of them.
    //
    // The reference kernels have no grid path at all -- OCC classifier-filters grid points and
    // inserts them as Steiner points into a CDT whose boundary is already frontier constraints
    // (one path, boundary-first); truck does the same via spade. libtess2 is a winding-rule
    // tessellator and cannot take Steiner points, so we cannot copy that directly; routing through
    // emit is the closest available equivalent.
    //
    // The 2026-07-13 note measured this as +48% tris / +79% crane -- but that was UNPINNED and
    // before converged_frac. Re-measure before trusting it.
    bool grid_via_emit = false;
    // Stop refining once a pass marks fewer than 1/converged_frac of the triangles. 0 = off (run to
    // the tri budget, as the default track does). Only meaningful with freeze_boundary.
    //
    // THIS is what made the freeze affordable, and it is the whole reason the 2026-07-13 attempt's
    // +34% is not a law of nature. Freezing slows triangle growth, so refine_uv's `tris > budget`
    // exit fires far later and the loop rescans the entire mesh chasing a few stragglers: measured
    // 1431 passes / 949k triangles scanned vs 812 / 578k unfrozen, for only 3.7% more marks — i.e.
    // the SAME refinement spread over 76% more passes. Time tracked triangles-scanned almost
    // exactly (+64% vs +65%). Stopping once a pass marks <1/50 of the triangles costs ~nothing in
    // quality (+1.4% triangles vs the default track) and takes the watertight track from +65% to
    // +0.3%. The prior attempt read this as refinement "fighting" an edge-vs-surface density
    // mismatch; densifying the boundary 2x/4x/8x was measured to NOT help at all, which is what
    // ruled that explanation out.
    double converged_frac = 50.0;
};

inline std::optional<TessTrack> parse_track(std::string_view s) {
    if (s.empty() || s == "libtess2")
        return TessTrack::Libtess2;
    if (s == "libtess2-watertight")
        return TessTrack::Libtess2Watertight;
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
    case TessTrack::Libtess2Watertight:
        return "libtess2-watertight";
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
    return {"libtess2", "libtess2-watertight", "occ", "cgal", "hybrid"};
}

} // namespace adacpp::ngeom
