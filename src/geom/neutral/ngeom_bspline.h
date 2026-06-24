// NGEOM B-spline curves & surfaces + generic Newton UV inversion (no OCC). Spec §4/§5/§8.
//
// de Boor evaluation works in homogeneous coordinates so the SAME code serves rational
// (NURBS) and non-rational: weights default to 1, the final divide is a no-op. Surfaces
// keep the homogeneous division to the very end (correct for rational tensor products).
#pragma once

#include <limits>
#include <vector>

#include "ngeom_curves.h"
#include "ngeom_math.h"
#include "ngeom_surfaces.h"

namespace adacpp::ngeom {

namespace bspline_detail {

struct HPoint {
    double x = 0, y = 0, z = 0, w = 1;
};
inline HPoint mix(const HPoint &a, const HPoint &b, double t) {
    return {a.x * (1 - t) + b.x * t, a.y * (1 - t) + b.y * t, a.z * (1 - t) + b.z * t, a.w * (1 - t) + b.w * t};
}
inline HPoint homog(const Vec3 &p, double w) {
    return {p.x * w, p.y * w, p.z * w, w};
}
inline Vec3 project(const HPoint &h) {
    double w = std::abs(h.w) > 1e-300 ? h.w : 1.0;
    return {h.x / w, h.y / w, h.z / w};
}

// Expand compact (knots, multiplicities) into a flat knot vector.
inline std::vector<double> expand_knots(const std::vector<double> &knots, const std::vector<int> &mults) {
    std::vector<double> U;
    for (size_t i = 0; i < knots.size(); ++i)
        for (int k = 0; k < mults[i]; ++k)
            U.push_back(knots[i]);
    return U;
}

// Find the knot span index for parameter u. n = #ctrl - 1, p = degree, U = flat knots.
inline int find_span(int n, int p, double u, const std::vector<double> &U) {
    if (u >= U[n + 1])
        return n;
    if (u <= U[p])
        return p;
    int lo = p, hi = n + 1, mid = (lo + hi) / 2;
    while (u < U[mid] || u >= U[mid + 1]) {
        if (u < U[mid])
            hi = mid;
        else
            lo = mid;
        mid = (lo + hi) / 2;
    }
    return mid;
}

// de Boor on a pre-gathered control array d[0..p] = cp[span-p .. span] (homogeneous).
inline HPoint deboor_gathered(int p, int span, double u, const std::vector<double> &U, std::vector<HPoint> d) {
    for (int r = 1; r <= p; ++r)
        for (int j = p; j >= r; --j) {
            double den = U[span + 1 + j - r] - U[span - p + j];
            double a = den > 1e-300 ? (u - U[span - p + j]) / den : 0.0;
            d[j] = mix(d[j - 1], d[j], a);
        }
    return d[p];
}

} // namespace bspline_detail

struct BSplineCurve : Curve {
    int degree = 1;
    std::vector<Vec3> ctrl;
    std::vector<double> U;       // flat knot vector
    std::vector<double> weights; // empty => non-rational
    bool closed = false;

    BSplineCurve(int deg, std::vector<Vec3> cp, std::vector<double> knots, std::vector<int> mults,
                 std::vector<double> w, bool closed_)
        : degree(deg), ctrl(std::move(cp)), weights(std::move(w)), closed(closed_) {
        U = bspline_detail::expand_knots(knots, mults);
    }

    Vec3 point(double t) const override {
        using namespace bspline_detail;
        int n = (int) ctrl.size() - 1;
        if (n <= 0)
            return ctrl.empty() ? Vec3{0, 0, 0} : ctrl[0];
        int p = std::min(degree, n); // guard degree>=n_ctrl (Rust geom.rs p=degree.min(n-1)+guards)
        int span = find_span(n, p, t, U);
        std::vector<HPoint> d(p + 1);
        for (int j = 0; j <= p; ++j) {
            int i = span - p + j;
            d[j] = homog(ctrl[i], weights.empty() ? 1.0 : weights[i]);
        }
        return project(deboor_gathered(p, span, t, U, std::move(d)));
    }
    Vec3 tangent(double t) const override {
        double lo, hi, per;
        bool p;
        range(lo, hi, p, per);
        double h = (hi - lo) * 1e-6 + 1e-9;
        double a = t - h < lo ? t : t - h;
        double b = t + h > hi ? t : t + h;
        if (b <= a) {
            a = lo;
            b = hi;
        }
        return (point(b) - point(a)).normalized();
    }
    void range(double &lo, double &hi, bool &per, double &period) const override {
        lo = U[degree];
        hi = U[(int) ctrl.size()]; // U[n+1]
        per = closed;
        period = closed ? (hi - lo) : 0.0;
    }
    // model.rs sample_bspline_to_polyline samples a B-spline EDGE uniformly at this density.
    int uniform_edge_segments() const override {
        int n = std::max((int) ctrl.size(), degree + 1) * 4;
        return n < 8 ? 8 : (n > 512 ? 512 : n);
    }
    int nominal_spans(double a, double b) const override {
        // geom.rs Curve3::BSpline: (flat_knots.len()-(2*deg+1)).max(1)*2, clamped [4,64].
        int spans = std::max((int) U.size() - (2 * degree + 1), 1);
        int n = spans * 2;
        if (n < 4)
            n = 4;
        if (n > 64)
            n = 64;
        double lo, hi, per;
        bool pr;
        range(lo, hi, pr, per);
        double frac = (hi > lo) ? std::abs(b - a) / (hi - lo) : 1.0;
        int m = (int) std::ceil(n * frac);
        return m < 1 ? 1 : m;
    }
};

// Generic seeded-Newton UV inversion for surfaces without a closed form (B-spline, swept).
// Seeds from (useed,vseed) when finite, else a coarse grid scan over the domain. FD Jacobian,
// Gauss-Newton step on the 3x2 system, clamped to the domain. Returns false if it can't converge.
inline bool newton_uv(const Surface &s, const Vec3 &p, double umin, double umax, double vmin, double vmax, double useed,
                      double vseed, double &u, double &v) {
    auto finite = [](double x) { return x == x && std::abs(x) < 1e300; };
    u = useed;
    v = vseed;
    if (!finite(u) || !finite(v) || u < umin || u > umax || v < vmin || v > vmax) {
        // coarse grid seed
        double best = std::numeric_limits<double>::max();
        const int N = 12;
        for (int i = 0; i <= N; ++i)
            for (int j = 0; j <= N; ++j) {
                double uu = umin + (umax - umin) * i / N;
                double vv = vmin + (vmax - vmin) * j / N;
                double d = (s.point(uu, vv) - p).norm();
                if (d < best) {
                    best = d;
                    u = uu;
                    v = vv;
                }
            }
    }
    double hu = (umax - umin) * 1e-6 + 1e-9;
    double hv = (vmax - vmin) * 1e-6 + 1e-9;
    for (int it = 0; it < 40; ++it) {
        Vec3 r = s.point(u, v) - p;
        if (r.norm() < 1e-10)
            return true;
        Vec3 Su = (s.point(u + hu, v) - s.point(u - hu, v)) * (0.5 / hu);
        Vec3 Sv = (s.point(u, v + hv) - s.point(u, v - hv)) * (0.5 / hv);
        // solve [Su Sv] d = -r  (least squares: normal equations 2x2)
        double a = Su.dot(Su), b = Su.dot(Sv), c = Sv.dot(Sv);
        double ru = -Su.dot(r), rv = -Sv.dot(r);
        double det = a * c - b * b;
        if (std::abs(det) < 1e-300)
            break;
        double du = (ru * c - rv * b) / det;
        double dv = (a * rv - b * ru) / det;
        u = clampd(u + du, umin, umax);
        v = clampd(v + dv, vmin, vmax);
        if (std::abs(du) < 1e-12 && std::abs(dv) < 1e-12)
            break;
    }
    return (s.point(u, v) - p).norm() < 1e-6;
}

struct BSplineSurface : Surface {
    int u_degree = 1, v_degree = 1;
    int nu = 0, nv = 0;          // #ctrl in u, v
    std::vector<Vec3> ctrl;      // row-major: index = iu*nv + iv
    std::vector<double> Uu, Uv;  // flat knots
    std::vector<double> weights; // empty => non-rational; else size nu*nv
    bool u_closed = false, v_closed = false;

    void domain(double &umin, double &umax, double &vmin, double &vmax) const {
        umin = Uu[u_degree];
        umax = Uu[nu]; // Uu[ (nu-1)+1 ]
        vmin = Uv[v_degree];
        vmax = Uv[nv];
    }

    Vec3 point(double u, double v) const override {
        using namespace bspline_detail;
        // clamp to the knot domain (Rust BSplineSurface::point) — return the boundary point
        // rather than extrapolating when a FD/Newton probe steps just outside.
        u = clampd(u, Uu[u_degree], Uu[nu]);
        v = clampd(v, Uv[v_degree], Uv[nv]);
        int su = find_span(nu - 1, u_degree, u, Uu);
        int sv = find_span(nv - 1, v_degree, v, Uv);
        std::vector<HPoint> tempV(v_degree + 1);
        for (int l = 0; l <= v_degree; ++l) {
            int vcol = sv - v_degree + l;
            std::vector<HPoint> du(u_degree + 1);
            for (int k = 0; k <= u_degree; ++k) {
                int urow = su - u_degree + k;
                int idx = urow * nv + vcol;
                du[k] = homog(ctrl[idx], weights.empty() ? 1.0 : weights[idx]);
            }
            tempV[l] = deboor_gathered(u_degree, su, u, Uu, std::move(du));
        }
        return project(deboor_gathered(v_degree, sv, v, Uv, std::move(tempV)));
    }
    Vec3 normal(double u, double v) const override {
        return fd_normal(u, v);
    }
    // B-spline UV inversion uses the Greville-seeded, damped, fold-aware inverter (bspline_invert,
    // a port of Rust newton_invert_bspline) — NOT the generic uniform-grid newton_uv, which aliases
    // against folds on coiled/threaded nets and converges onto the wrong fold.
    bool uv(const Vec3 &p, double uhint, double vhint, double &u, double &v) const override;
    void periods(bool &up, double &uper, bool &vp, double &vper) const override {
        double umin, umax, vmin, vmax;
        domain(umin, umax, vmin, vmax);
        up = u_closed;
        vp = v_closed;
        uper = u_closed ? (umax - umin) : 0.0;
        vper = v_closed ? (vmax - vmin) : 0.0;
    }
    // control-net bbox diagonal — characteristic size (BSplineSurface.size)
    double size() const {
        if (ctrl.empty())
            return 1.0;
        Vec3 lo = ctrl[0], hi = ctrl[0];
        for (const Vec3 &p : ctrl) {
            lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
            hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
        }
        return (hi - lo).norm();
    }
    double approx_size() const override {
        return size();
    }
    // loose ≤8-span floor; the perpendicular-sag refinement drives the real density (
    // u_step/v_step BSpline arm — a hard per-knot-span floor explodes on hundreds-of-span coils).
    double u_step(double, double) const override {
        double a = Uu[u_degree], b = Uu[nu];
        int spans = std::min(std::max(nu - u_degree, 1), 8);
        return (b - a) / spans;
    }
    double v_step(double, double) const override {
        double a = Uv[v_degree], b = Uv[nv];
        int spans = std::min(std::max(nv - v_degree, 1), 8);
        return (b - a) / spans;
    }
};

// One damped Gauss-Newton descent of |S(u,v)-p|^2 from a seed, clamped to the domain.
// Port of Rust geom.rs newton_refine_bspline (30 iters, ±span/4 step clamp, 8-step backtracking
// line search until the residual drops). The damping is what keeps it from ping-ponging across
// tight folds where plain Gauss-Newton overshoots.
inline std::pair<double, double> bspline_refine(const BSplineSurface &b, const Vec3 &p, double su, double sv) {
    double u0, u1, v0, v1;
    b.domain(u0, u1, v0, v1);
    double du_span = std::max(u1 - u0, 1e-12), dv_span = std::max(v1 - v0, 1e-12);
    double u = su, v = sv;
    double hu = du_span * 1e-6, hv = dv_span * 1e-6;
    for (int it = 0; it < 30; ++it) {
        Vec3 f = b.point(u, v) - p;
        Vec3 Su = (b.point(u + hu, v) - b.point(u - hu, v)) * (0.5 / hu);
        Vec3 Sv = (b.point(u, v + hv) - b.point(u, v - hv)) * (0.5 / hv);
        double a11 = Su.dot(Su), a12 = Su.dot(Sv), a22 = Sv.dot(Sv);
        double r1 = -f.dot(Su), r2 = -f.dot(Sv);
        double det = a11 * a22 - a12 * a12;
        if (std::abs(det) < 1e-300)
            break;
        double duu = clampd((r1 * a22 - r2 * a12) / det, -du_span / 4.0, du_span / 4.0);
        double dvv = clampd((a11 * r2 - a12 * r1) / det, -dv_span / 4.0, dv_span / 4.0);
        double d0 = f.dot(f);
        double scale = 1.0, nu_ = u, nv_ = v;
        for (int k = 0; k < 8; ++k) {
            nu_ = clampd(u + duu * scale, u0, u1);
            nv_ = clampd(v + dvv * scale, v0, v1);
            Vec3 d = b.point(nu_, nv_) - p;
            if (d.dot(d) < d0)
                break;
            scale *= 0.5;
        }
        double su_ = nu_ - u, sv_ = nv_ - v;
        u = nu_;
        v = nv_;
        if (std::abs(su_) < 1e-12 * du_span && std::abs(sv_) < 1e-12 * dv_span)
            break;
    }
    return {u, v};
}

// Project p onto a B-spline surface. Port of Rust geom.rs newton_invert_bspline: try the hint first
// (accept only if on-surface within (size*1e-4)^2), else seed at the Greville point of the nearest
// control point + its 12 control-net neighbours (early-out on first on-surface hit), else a
// span-resolution domain scan. Always returns a best-effort (u,v) (true), like Rust.
inline bool bspline_invert(const BSplineSurface &b, const Vec3 &p, double uh, double vh, double &outu, double &outv) {
    auto dist2 = [&](double u, double v) {
        Vec3 d = b.point(u, v) - p;
        return d.dot(d);
    };
    double sz = b.size();
    double tol2 = (sz * 1e-4) * (sz * 1e-4);
    double u0, u1, v0, v1;
    b.domain(u0, u1, v0, v1);

    if (uh == uh && vh == vh) { // finite hint
        auto [ru, rv] = bspline_refine(b, p, uh, vh);
        if (dist2(ru, rv) <= tol2) {
            outu = ru;
            outv = rv;
            return true;
        }
    }
    // nearest control point
    int bi = 0;
    double bd = std::numeric_limits<double>::max();
    for (int i = 0; i < (int) b.ctrl.size(); ++i) {
        Vec3 d = b.ctrl[i] - p;
        double dd = d.dot(d);
        if (dd < bd) {
            bd = dd;
            bi = i;
        }
    }
    long ciu = bi / b.nv, civ = bi % b.nv;
    auto greville = [&](const std::vector<double> &knots, int deg, int i) {
        deg = std::max(deg, 1);
        double s = 0;
        for (int k = i + 1; k <= i + deg; ++k)
            s += knots[k];
        return s / deg;
    };
    static const int OFFS[13][2] = {{0, 0}, {0, 1},   {0, -1}, {1, 0},  {-1, 0}, {0, 2}, {0, -2},
                                    {1, 1}, {-1, -1}, {1, -1}, {-1, 1}, {2, 0},  {-2, 0}};
    double r1u = u0, r1v = v0, best1 = std::numeric_limits<double>::max();
    for (auto &o : OFFS) {
        long iu = std::min(std::max(ciu + o[0], 0L), (long) b.nu - 1);
        long iv = std::min(std::max(civ + o[1], 0L), (long) b.nv - 1);
        double seedu = clampd(greville(b.Uu, b.u_degree, (int) iu), u0, u1);
        double seedv = clampd(greville(b.Uv, b.v_degree, (int) iv), v0, v1);
        auto [ru, rv] = bspline_refine(b, p, seedu, seedv);
        double rd = dist2(ru, rv);
        if (rd < best1) {
            best1 = rd;
            r1u = ru;
            r1v = rv;
        }
        if (best1 <= tol2) {
            outu = r1u;
            outv = r1v;
            return true;
        }
    }
    // span-resolution domain scan (last resort for off-surface rational nets)
    double du_span = std::max(u1 - u0, 1e-12), dv_span = std::max(v1 - v0, 1e-12);
    int nu = std::min(std::max((b.nu - b.u_degree > 0 ? b.nu - b.u_degree : 1) * 2, 4), 128);
    int nv = std::min(std::max((b.nv - b.v_degree > 0 ? b.nv - b.v_degree : 1) * 2, 4), 256);
    double bu = u0, bv = v0, bdd = std::numeric_limits<double>::max();
    for (int i = 0; i <= nu; ++i)
        for (int j = 0; j <= nv; ++j) {
            double uu = u0 + du_span * i / nu, vv = v0 + dv_span * j / nv;
            double d = dist2(uu, vv);
            if (d < bdd) {
                bdd = d;
                bu = uu;
                bv = vv;
            }
        }
    auto [r2u, r2v] = bspline_refine(b, p, bu, bv);
    if (dist2(r2u, r2v) < best1) {
        outu = r2u;
        outv = r2v;
    } else {
        outu = r1u;
        outv = r1v;
    }
    return true;
}

inline bool BSplineSurface::uv(const Vec3 &p, double uhint, double vhint, double &u, double &v) const {
    return bspline_invert(*this, p, uhint, vhint, u, v);
}

// Bounding-box diagonal of a curve sampled over [a,b] (9 points) — Rust geom.rs Curve3::approx_size.
inline double profile_bbox_diag(const Curve &c, double a, double b) {
    Vec3 lo = c.point(a), hi = lo;
    for (int i = 0; i <= 8; ++i) {
        Vec3 p = c.point(a + (b - a) * i / 8.0);
        lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
        hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
    }
    return (hi - lo).norm();
}

// Surface swept by translating a profile curve along a direction (spec §5 SURF_LIN_EXTRUSION).
// u = the profile curve's own parameter; v = distance along dir in [0, depth].
struct LinearExtrusionSurface : Surface {
    std::shared_ptr<Curve> profile;
    Vec3 dir; // unit
    double depth;
    LinearExtrusionSurface(std::shared_ptr<Curve> c, const Vec3 &d, double dep)
        : profile(c), dir(d.normalized()), depth(dep) {}
    void domain(double &umin, double &umax, double &vmin, double &vmax) const {
        bool per;
        double period;
        profile->range(umin, umax, per, period);
        vmin = 0;
        vmax = depth;
    }
    Vec3 point(double u, double v) const override {
        return profile->point(u) + dir * v;
    }
    Vec3 normal(double u, double v) const override {
        return profile->tangent(u).cross(dir).normalized();
    }
    bool uv(const Vec3 &p, double uhint, double vhint, double &u, double &v) const override {
        double umin, umax, vmin, vmax;
        domain(umin, umax, vmin, vmax);
        return newton_uv(*this, p, umin, umax, vmin, vmax, uhint, vhint, u, v);
    }
    // density floor along the profile (Rust geom.rs Surface::u_step Extrusion arm); v (along dir)
    // is ruled -> INFINITY (base default), matching Rust's `_ => INFINITY` for extrusion v_step.
    double u_step(double, double) const override {
        double umin, umax, vmin, vmax;
        domain(umin, umax, vmin, vmax);
        int spans = std::max(1, profile->discretize_spans(umin, umax));
        return (umax - umin) / spans;
    }
    double approx_size() const override {
        double umin, umax, vmin, vmax;
        domain(umin, umax, vmin, vmax);
        return profile_bbox_diag(*profile, umin, umax) + depth; // Rust curve.approx_size()+dir.len()
    }
    // a circular directrix makes the extrusion u-periodic (Rust u_period = curve.period())
    std::optional<double> u_period() const override {
        double lo, hi;
        bool per;
        double period;
        profile->range(lo, hi, per, period);
        return per ? std::optional<double>(period) : std::nullopt;
    }
};

// Surface swept by revolving a profile curve about an axis (spec §5 SURF_REVOLUTION).
// u = revolution angle [0,2pi) (periodic); v = the profile curve's own parameter.
struct RevolutionSurface : Surface {
    std::shared_ptr<Curve> profile;
    Vec3 axis_loc, axis_dir; // axis_dir unit
    RevolutionSurface(std::shared_ptr<Curve> c, const Vec3 &loc, const Vec3 &dir)
        : profile(c), axis_loc(loc), axis_dir(dir.normalized()) {}
    void domain(double &umin, double &umax, double &vmin, double &vmax) const {
        umin = 0;
        umax = TWO_PI;
        bool per;
        double period;
        profile->range(vmin, vmax, per, period);
    }
    Vec3 rotate(const Vec3 &pt, double ang) const {
        Vec3 rel = pt - axis_loc;
        Vec3 par = axis_dir * axis_dir.dot(rel);
        Vec3 perp = rel - par;
        Vec3 rotated = par + perp * std::cos(ang) + axis_dir.cross(perp) * std::sin(ang);
        return axis_loc + rotated;
    }
    Vec3 point(double u, double v) const override {
        return rotate(profile->point(v), u);
    }
    Vec3 normal(double u, double v) const override {
        return fd_normal(u, v);
    }
    bool uv(const Vec3 &p, double uhint, double vhint, double &u, double &v) const override {
        double umin, umax, vmin, vmax;
        domain(umin, umax, vmin, vmax);
        return newton_uv(*this, p, umin, umax, vmin, vmax, uhint, vhint, u, v);
    }
    void periods(bool &up, double &uper, bool &vp, double &vper) const override {
        up = true;
        uper = TWO_PI;
        vp = false;
        vper = 0;
    }
    // Rust geom.rs Surface::u_step/v_step Revolution arms: u density from the largest radius along
    // the profile (angle_step), v density from the profile's own span count.
    double u_step(double defl, double ma) const override {
        double umin, umax, vmin, vmax;
        domain(umin, umax, vmin, vmax);
        double rmax = 1e-9;
        for (int i = 0; i <= 16; ++i) {
            Vec3 c = profile->point(vmin + (vmax - vmin) * i / 16.0) - axis_loc;
            Vec3 perp = c - axis_dir * axis_dir.dot(c);
            rmax = std::max(rmax, perp.norm());
        }
        return angle_step(rmax, defl, ma);
    }
    double v_step(double, double) const override {
        double umin, umax, vmin, vmax;
        domain(umin, umax, vmin, vmax);
        int spans = std::max(1, profile->discretize_spans(vmin, vmax));
        return (vmax - vmin) / spans;
    }
    double approx_size() const override {
        double umin, umax, vmin, vmax;
        domain(umin, umax, vmin, vmax);
        return profile_bbox_diag(*profile, vmin, vmax) * 2.0; // Rust curve.approx_size()*2
    }
};

} // namespace adacpp::ngeom
