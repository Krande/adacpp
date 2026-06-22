#include "ngeom_tessellate.h"

#include <cmath>
#include <limits>

#include "tesselator.h"  // vendored libtess2

namespace adacpp::ngeom {

namespace {

constexpr double NAN_D = std::numeric_limits<double>::quiet_NaN();

// One boundary loop projected into (metric-scaled) UV, ready for tess2.
struct UvLoop {
    std::vector<double> u;  // parametric u (un-scaled)
    std::vector<double> v;
};

// Project a loop's 3D polyline to UV with periodic unwrap (continuous along the loop).
bool project_loop(const Surface &s, const std::vector<Vec3> &pts3d, UvLoop &out) {
    bool up, vp;
    double uper, vper;
    s.periods(up, uper, vp, vper);
    double pu = NAN_D, pv = NAN_D;
    out.u.reserve(pts3d.size());
    out.v.reserve(pts3d.size());
    for (const Vec3 &p : pts3d) {
        double u, v;
        if (!s.uv(p, pu, pv, u, v)) return false;
        if (up && pu == pu) {  // unwrap u so the loop follows a continuous path
            while (u - pu > uper * 0.5) u -= uper;
            while (pu - u > uper * 0.5) u += uper;
        }
        if (vp && pv == pv) {
            while (v - pv > vper * 0.5) v -= vper;
            while (pv - v > vper * 0.5) v += vper;
        }
        out.u.push_back(u);
        out.v.push_back(v);
        pu = u;
        pv = v;
    }
    return true;
}

// Metric scale (su, sv): map UV increments to ~arc length at a representative point, so
// tess2 works in a near-isotropic space (step2glb metric_scale). Magnitudes => no flip.
void metric_scale(const Surface &s, const std::vector<UvLoop> &loops, double &su, double &sv) {
    su = sv = 1.0;
    // centroid of the outer loop in UV
    if (loops.empty() || loops[0].u.empty()) return;
    double cu = 0, cv = 0;
    size_t n = loops[0].u.size();
    for (size_t i = 0; i < n; ++i) {
        cu += loops[0].u[i];
        cv += loops[0].v[i];
    }
    cu /= n;
    cv /= n;
    const double h = 1e-5;
    double du = (s.point(cu + h, cv) - s.point(cu - h, cv)).norm() / (2 * h);
    double dv = (s.point(cu, cv + h) - s.point(cu, cv - h)).norm() / (2 * h);
    if (du > 1e-12) su = du;
    if (dv > 1e-12) sv = dv;
    // normalize so the larger axis is ~1 (keeps coords well-scaled for float tess2)
    double m = std::max(su, sv);
    if (m > 1e-12) {
        su /= m;
        sv /= m;
    }
}

}  // namespace

bool tessellate_face(const FaceSurfaceN &face, const TessParams &tp, TessMesh &out) {
    if (!face.surface || face.bounds.empty()) return false;
    const Surface &s = *face.surface;

    // 1. discretize boundary loops to 3D, 2. project to UV (with unwrap)
    std::vector<UvLoop> loops;
    loops.reserve(face.bounds.size());
    for (const FaceBoundN &b : face.bounds) {
        if (!b.loop) continue;
        std::vector<Vec3> pts3d = b.loop->discretize(tp.deflection, tp.max_angle);
        if (pts3d.size() < 3) continue;
        UvLoop uv;
        if (!project_loop(s, pts3d, uv)) continue;
        loops.push_back(std::move(uv));
    }
    if (loops.empty()) return false;

    // 3. metric scale
    double su, sv;
    metric_scale(s, loops, su, sv);

    // 4. sanitize + feed tess2 (contours in metric UV)
    TESStesselator *t = tessNewTess(nullptr);
    if (!t) return false;
    std::vector<std::vector<TESSreal>> contours;  // keep alive until tessTesselate
    contours.reserve(loops.size());
    for (const UvLoop &lp : loops) {
        std::vector<TESSreal> flat;
        flat.reserve(lp.u.size() * 2);
        double lastu = NAN_D, lastv = NAN_D;
        for (size_t i = 0; i < lp.u.size(); ++i) {
            double uu = lp.u[i] * su, vv = lp.v[i] * sv;
            if (lastu == lastu && std::abs(uu - lastu) + std::abs(vv - lastv) < 1e-12) continue;
            flat.push_back((TESSreal)uu);
            flat.push_back((TESSreal)vv);
            lastu = uu;
            lastv = vv;
        }
        // drop a closing duplicate
        if (flat.size() >= 4) {
            double dx = flat[0] - flat[flat.size() - 2];
            double dy = flat[1] - flat[flat.size() - 1];
            if (std::abs(dx) + std::abs(dy) < 1e-9) {
                flat.pop_back();
                flat.pop_back();
            }
        }
        if (flat.size() < 6) continue;  // need >= 3 distinct points
        contours.push_back(std::move(flat));
        tessAddContour(t, 2, contours.back().data(), sizeof(TESSreal) * 2,
                       (int)contours.back().size() / 2);
    }
    if (contours.empty()) {
        tessDeleteTess(t);
        return false;
    }

    // 5. tessellate: odd winding (outer + nested holes), triangles, 2D verts
    if (!tessTesselate(t, TESS_WINDING_ODD, TESS_POLYGONS, 3, 2, nullptr)) {
        tessDeleteTess(t);
        return false;
    }

    const TESSreal *verts = tessGetVertices(t);
    const int nverts = tessGetVertexCount(t);
    const TESSindex *elems = tessGetElements(t);
    const int nelems = tessGetElementCount(t);
    if (nverts <= 0 || nelems <= 0) {
        tessDeleteTess(t);
        return false;
    }

    // 6. map UV verts back to 3D + normals (un-scale UV first)
    uint32_t base = (uint32_t)(out.positions.size() / 3);
    for (int i = 0; i < nverts; ++i) {
        double u = verts[i * 2 + 0] / su;
        double v = verts[i * 2 + 1] / sv;
        Vec3 p = s.point(u, v);
        Vec3 n = s.normal(u, v);
        if (!face.same_sense) n = n * -1.0;
        out.positions.push_back((float)p.x);
        out.positions.push_back((float)p.y);
        out.positions.push_back((float)p.z);
        out.normals.push_back((float)n.x);
        out.normals.push_back((float)n.y);
        out.normals.push_back((float)n.z);
    }
    bool flip = !face.same_sense;
    for (int e = 0; e < nelems; ++e) {
        const TESSindex *poly = &elems[e * 3];
        if (poly[0] == TESS_UNDEF || poly[1] == TESS_UNDEF || poly[2] == TESS_UNDEF) continue;
        uint32_t a = base + poly[0], b = base + poly[1], c = base + poly[2];
        // drop degenerate
        if (a == b || b == c || a == c) continue;
        if (flip) {
            out.indices.push_back(a);
            out.indices.push_back(c);
            out.indices.push_back(b);
        } else {
            out.indices.push_back(a);
            out.indices.push_back(b);
            out.indices.push_back(c);
        }
    }
    tessDeleteTess(t);
    return true;
}

TessMesh tessellate_doc(const NgeomDoc &doc, const TessParams &tp) {
    TessMesh mesh;
    for (const NgeomRoot &root : doc.roots) {
        uint32_t first = (uint32_t)mesh.indices.size();
        for (const auto &face : root.faces) {
            if (face) tessellate_face(*face, tp, mesh);
        }
        TessMesh::Group g;
        g.id = root.id;
        g.first_index = first;
        g.index_count = (uint32_t)mesh.indices.size() - first;
        mesh.groups.push_back(std::move(g));
    }
    return mesh;
}

}  // namespace adacpp::ngeom
