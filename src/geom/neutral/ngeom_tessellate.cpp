#include "ngeom_tessellate.h"

#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <utility>

#include "tesselator.h"  // vendored libtess2 (compile its C with -DNDEBUG; it fails soft
                         // via setjmp/longjmp -> tessTesselate returns 0 on any sweep error)

namespace adacpp::ngeom {

namespace {

constexpr double NAN_D = std::numeric_limits<double>::quiet_NaN();

struct UvLoop {
    std::vector<double> u, v;
};

// Working UV mesh during refinement.
struct UvMesh {
    std::vector<std::pair<double, double>> verts;   // (u,v) parametric
    std::vector<std::array<uint32_t, 3>> tris;
};

// Project a loop's 3D polyline to UV with periodic unwrap (continuous along the loop).
bool project_loop(const Surface &s, const std::vector<Vec3> &pts3d, UvLoop &out) {
    bool up, vp;
    double uper, vper;
    s.periods(up, uper, vp, vper);
    double pu = NAN_D, pv = NAN_D;
    for (const Vec3 &p : pts3d) {
        double u, v;
        if (!s.uv(p, pu, pv, u, v)) return false;
        if (up && pu == pu) {
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

void metric_scale(const Surface &s, const std::vector<UvLoop> &loops, double &su, double &sv) {
    su = sv = 1.0;
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
    double m = std::max(su, sv);
    if (m > 1e-12) {
        su /= m;
        sv /= m;
    }
}

double loop_signed_uv_area(const UvLoop &l) {
    double a = 0;
    size_t n = l.u.size();
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        a += l.u[i] * l.v[j] - l.u[j] * l.v[i];
    }
    return 0.5 * a;
}

// Run tess2 on metric-scaled contours; output parametric-UV verts + tris. `shrink` pulls each
// hole loop toward its centroid (fail-soft retry to avoid touching/ill-formed contours).
bool run_tess2(const std::vector<UvLoop> &loops, double su, double sv, double shrink, UvMesh &out) {
    TESStesselator *t = tessNewTess(nullptr);
    if (!t) return false;
    std::vector<std::vector<TESSreal>> contours;
    contours.reserve(loops.size());
    for (size_t li = 0; li < loops.size(); ++li) {
        const UvLoop &lp = loops[li];
        double cu = 0, cv = 0;
        if (shrink > 0 && li > 0) {  // shrink holes only (loop 0 = outer)
            for (size_t i = 0; i < lp.u.size(); ++i) {
                cu += lp.u[i];
                cv += lp.v[i];
            }
            cu /= lp.u.size();
            cv /= lp.v.size();
        }
        std::vector<TESSreal> flat;
        double lu = NAN_D, lv = NAN_D;
        for (size_t i = 0; i < lp.u.size(); ++i) {
            double u = lp.u[i], v = lp.v[i];
            if (shrink > 0 && li > 0) {
                u = cu + (u - cu) * (1.0 - shrink);
                v = cv + (v - cv) * (1.0 - shrink);
            }
            double uu = u * su, vv = v * sv;
            if (lu == lu && std::abs(uu - lu) + std::abs(vv - lv) < 1e-12) continue;
            flat.push_back((TESSreal)uu);
            flat.push_back((TESSreal)vv);
            lu = uu;
            lv = vv;
        }
        if (flat.size() >= 4) {
            double dx = flat[0] - flat[flat.size() - 2], dy = flat[1] - flat[flat.size() - 1];
            if (std::abs(dx) + std::abs(dy) < 1e-9) {
                flat.pop_back();
                flat.pop_back();
            }
        }
        if (flat.size() < 6) continue;
        contours.push_back(std::move(flat));
        tessAddContour(t, 2, contours.back().data(), sizeof(TESSreal) * 2,
                       (int)contours.back().size() / 2);
    }
    if (contours.empty() || !tessTesselate(t, TESS_WINDING_ODD, TESS_POLYGONS, 3, 2, nullptr)) {
        tessDeleteTess(t);
        return false;
    }
    const TESSreal *verts = tessGetVertices(t);
    const int nv = tessGetVertexCount(t);
    const TESSindex *elems = tessGetElements(t);
    const int ne = tessGetElementCount(t);
    if (nv <= 0 || ne <= 0) {
        tessDeleteTess(t);
        return false;
    }
    out.verts.reserve(nv);
    for (int i = 0; i < nv; ++i) out.verts.push_back({verts[i * 2] / su, verts[i * 2 + 1] / sv});
    for (int e = 0; e < ne; ++e) {
        const TESSindex *p = &elems[e * 3];
        if (p[0] == TESS_UNDEF || p[1] == TESS_UNDEF || p[2] == TESS_UNDEF) continue;
        if (p[0] == p[1] || p[1] == p[2] || p[0] == p[2]) continue;
        out.tris.push_back({(uint32_t)p[0], (uint32_t)p[1], (uint32_t)p[2]});
    }
    tessDeleteTess(t);
    return !out.tris.empty();
}

// Build a coarse regular UV grid over [umin,umax]x[vmin,vmax] (1c: closed/degenerate-UV
// surfaces). Refinement then smooths it. Caps the initial grid; poles collapse harmlessly.
void build_grid(double umin, double umax, double vmin, double vmax, int nu, int nv, UvMesh &out) {
    nu = std::max(1, std::min(nu, 48));
    nv = std::max(1, std::min(nv, 48));
    auto vid = [&](int i, int j) -> uint32_t { return (uint32_t)(i * (nv + 1) + j); };
    for (int i = 0; i <= nu; ++i)
        for (int j = 0; j <= nv; ++j)
            out.verts.push_back({umin + (umax - umin) * i / nu, vmin + (vmax - vmin) * j / nv});
    for (int i = 0; i < nu; ++i)
        for (int j = 0; j < nv; ++j) {
            out.tris.push_back({vid(i, j), vid(i + 1, j), vid(i + 1, j + 1)});
            out.tris.push_back({vid(i, j), vid(i + 1, j + 1), vid(i, j + 1)});
        }
}

void emit_split(uint32_t v0, uint32_t v1, uint32_t v2, long m0, long m1, long m2,
                std::vector<std::array<uint32_t, 3>> &out) {
    auto T = [&](uint32_t a, uint32_t b, uint32_t c) { out.push_back({a, b, c}); };
    int n = (m0 >= 0) + (m1 >= 0) + (m2 >= 0);
    uint32_t M0 = (uint32_t)m0, M1 = (uint32_t)m1, M2 = (uint32_t)m2;
    if (n == 0) {
        T(v0, v1, v2);
    } else if (n == 3) {
        T(v0, M0, M2);
        T(M0, v1, M1);
        T(M2, M1, v2);
        T(M0, M1, M2);
    } else if (n == 1) {
        if (m0 >= 0) {
            T(v0, M0, v2);
            T(M0, v1, v2);
        } else if (m1 >= 0) {
            T(v0, v1, M1);
            T(v0, M1, v2);
        } else {
            T(v0, v1, M2);
            T(v1, v2, M2);
        }
    } else {  // n == 2
        if (m0 >= 0 && m1 >= 0) {
            T(v0, M0, M1);
            T(v0, M1, v2);
            T(M0, v1, M1);
        } else if (m1 >= 0 && m2 >= 0) {
            T(v1, M1, M2);
            T(v1, M2, v0);
            T(M1, v2, M2);
        } else {  // m0, m2
            T(v2, M2, M0);
            T(v2, M0, v1);
            T(M2, v0, M0);
        }
    }
}

// 1b: adaptive curvature refinement via marked-edge subdivision (crack-free: shared edges
// share a midpoint). Splits edges whose 3D chord sags past `deflection` or whose endpoint
// normals turn more than `max_angle`. Bounded passes + triangle budget.
void refine(const Surface &s, UvMesh &m, double deflection, double max_angle) {
    if (deflection <= 0 && max_angle <= 0) return;
    auto pt = [&](uint32_t i) { return s.point(m.verts[i].first, m.verts[i].second); };
    auto nrm = [&](uint32_t i) { return s.normal(m.verts[i].first, m.verts[i].second); };
    const size_t budget = 200000;
    for (int pass = 0; pass < 6; ++pass) {
        std::map<std::pair<uint32_t, uint32_t>, long> mid;
        auto key = [](uint32_t a, uint32_t b) {
            return a < b ? std::make_pair(a, b) : std::make_pair(b, a);
        };
        // mark edges needing a split
        for (auto &t : m.tris)
            for (int i = 0; i < 3; ++i) {
                auto k = key(t[i], t[(i + 1) % 3]);
                if (mid.count(k)) continue;
                uint32_t a = k.first, b = k.second;
                double um = 0.5 * (m.verts[a].first + m.verts[b].first);
                double vm = 0.5 * (m.verts[a].second + m.verts[b].second);
                Vec3 pa = pt(a), pb = pt(b), pmid = s.point(um, vm);
                bool split = false;
                if (deflection > 0 && (pmid - (pa + pb) * 0.5).norm() > deflection) split = true;
                if (!split && max_angle > 0) {
                    double c = clampd(nrm(a).dot(nrm(b)), -1.0, 1.0);
                    if (std::acos(c) > max_angle) split = true;
                }
                if (split) mid[k] = -1;  // marked; midpoint created below
            }
        if (mid.empty()) break;
        for (auto &kv : mid) {
            auto [a, b] = kv.first;
            double um = 0.5 * (m.verts[a].first + m.verts[b].first);
            double vm = 0.5 * (m.verts[a].second + m.verts[b].second);
            m.verts.push_back({um, vm});
            kv.second = (long)m.verts.size() - 1;
        }
        std::vector<std::array<uint32_t, 3>> nt;
        nt.reserve(m.tris.size() * 2);
        for (auto &t : m.tris) {
            auto g = [&](uint32_t a, uint32_t b) -> long {
                auto it = mid.find(key(a, b));
                return it == mid.end() ? -1 : it->second;
            };
            emit_split(t[0], t[1], t[2], g(t[0], t[1]), g(t[1], t[2]), g(t[2], t[0]), nt);
        }
        m.tris.swap(nt);
        if (m.tris.size() > budget) break;
    }
}

}  // namespace

bool tessellate_face(const FaceSurfaceN &face, const TessParams &tp, TessMesh &out) {
    if (!face.surface || face.bounds.empty()) return false;
    const Surface &s = *face.surface;

    // discretize + project boundary loops to UV
    std::vector<UvLoop> loops;
    for (const FaceBoundN &b : face.bounds) {
        if (!b.loop) continue;
        std::vector<Vec3> pts3d = b.loop->discretize(tp.deflection, tp.max_angle);
        if (pts3d.size() < 3) continue;
        UvLoop uv;
        if (!project_loop(s, pts3d, uv)) continue;
        loops.push_back(std::move(uv));
    }
    if (loops.empty()) return false;

    UvMesh mesh;

    // 1c: degenerate-UV (seam/slit) or closed surface -> grid the domain instead of tess2.
    double outer_area = std::abs(loop_signed_uv_area(loops[0]));
    double umin = loops[0].u[0], umax = umin, vmin = loops[0].v[0], vmax = vmin;
    for (const auto &lp : loops)
        for (size_t i = 0; i < lp.u.size(); ++i) {
            umin = std::min(umin, lp.u[i]);
            umax = std::max(umax, lp.u[i]);
            vmin = std::min(vmin, lp.v[i]);
            vmax = std::max(vmax, lp.v[i]);
        }
    double uvdiag2 = (umax - umin) * (umax - umin) + (vmax - vmin) * (vmax - vmin);
    bool degenerate = outer_area < 1e-7 * (uvdiag2 + 1e-30);

    if (degenerate) {
        bool up, vp;
        double uper, vper;
        s.periods(up, uper, vp, vper);
        if (up) {
            umin = 0.0;
            umax = uper;
        }
        if (vp) {
            vmin = 0.0;
            vmax = vper;
        }
        if (umax - umin < 1e-12 || vmax - vmin < 1e-12) return false;
        int nu = up ? std::max(6, (int)std::ceil((umax - umin) / 0.6)) : 4;
        int nv = vp ? std::max(6, (int)std::ceil((vmax - vmin) / 0.6)) : 4;
        build_grid(umin, umax, vmin, vmax, nu, nv, mesh);
    } else {
        double su, sv;
        metric_scale(s, loops, su, sv);
        bool ok = run_tess2(loops, su, sv, 0.0, mesh);
        for (double shrink : {0.01, 0.04, 0.1}) {  // 1d: fail-soft shrunk-hole retry
            if (ok) break;
            mesh = UvMesh{};
            ok = run_tess2(loops, su, sv, shrink, mesh);
        }
        if (!ok) return false;
    }

    // 1b: curvature refinement (skip for planes — never curved)
    if (tp.deflection > 0) refine(s, mesh, tp.deflection, tp.max_angle);

    // map UV -> 3D + normals; honour same_sense (flip normal + winding)
    uint32_t base = (uint32_t)(out.positions.size() / 3);
    bool flip = !face.same_sense;
    for (auto &uv : mesh.verts) {
        Vec3 p = s.point(uv.first, uv.second);
        Vec3 n = s.normal(uv.first, uv.second);
        if (flip) n = n * -1.0;
        out.positions.push_back((float)p.x);
        out.positions.push_back((float)p.y);
        out.positions.push_back((float)p.z);
        out.normals.push_back((float)n.x);
        out.normals.push_back((float)n.y);
        out.normals.push_back((float)n.z);
    }
    for (auto &t : mesh.tris) {
        uint32_t a = base + t[0], b = base + t[1], c = base + t[2];
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
    return true;
}

TessMesh tessellate_doc(const NgeomDoc &doc, const TessParams &tp) {
    TessMesh mesh;
    for (const NgeomRoot &root : doc.roots) {
        uint32_t first = (uint32_t)mesh.indices.size();
        uint32_t vfirst = (uint32_t)(mesh.positions.size() / 3);
        for (const auto &face : root.faces)
            if (face) tessellate_face(*face, tp, mesh);
        mesh.groups.push_back({root.id, first, (uint32_t)mesh.indices.size() - first, vfirst,
                               (uint32_t)(mesh.positions.size() / 3) - vfirst});
    }
    return mesh;
}

}  // namespace adacpp::ngeom
