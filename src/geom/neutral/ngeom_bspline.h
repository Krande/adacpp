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
    return {a.x * (1 - t) + b.x * t, a.y * (1 - t) + b.y * t, a.z * (1 - t) + b.z * t,
            a.w * (1 - t) + b.w * t};
}
inline HPoint homog(const Vec3 &p, double w) { return {p.x * w, p.y * w, p.z * w, w}; }
inline Vec3 project(const HPoint &h) {
    double w = std::abs(h.w) > 1e-300 ? h.w : 1.0;
    return {h.x / w, h.y / w, h.z / w};
}

// Expand compact (knots, multiplicities) into a flat knot vector.
inline std::vector<double> expand_knots(const std::vector<double> &knots, const std::vector<int> &mults) {
    std::vector<double> U;
    for (size_t i = 0; i < knots.size(); ++i)
        for (int k = 0; k < mults[i]; ++k) U.push_back(knots[i]);
    return U;
}

// Find the knot span index for parameter u. n = #ctrl - 1, p = degree, U = flat knots.
inline int find_span(int n, int p, double u, const std::vector<double> &U) {
    if (u >= U[n + 1]) return n;
    if (u <= U[p]) return p;
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
            double a = den > 1e-15 ? (u - U[span - p + j]) / den : 0.0;
            d[j] = mix(d[j - 1], d[j], a);
        }
    return d[p];
}

}  // namespace bspline_detail

struct BSplineCurve : Curve {
    int degree = 1;
    std::vector<Vec3> ctrl;
    std::vector<double> U;        // flat knot vector
    std::vector<double> weights;  // empty => non-rational
    bool closed = false;

    BSplineCurve(int deg, std::vector<Vec3> cp, std::vector<double> knots, std::vector<int> mults,
                 std::vector<double> w, bool closed_)
        : degree(deg), ctrl(std::move(cp)), weights(std::move(w)), closed(closed_) {
        U = bspline_detail::expand_knots(knots, mults);
    }

    Vec3 point(double t) const override {
        using namespace bspline_detail;
        int n = (int)ctrl.size() - 1;
        int span = find_span(n, degree, t, U);
        std::vector<HPoint> d(degree + 1);
        for (int j = 0; j <= degree; ++j) {
            int i = span - degree + j;
            d[j] = homog(ctrl[i], weights.empty() ? 1.0 : weights[i]);
        }
        return project(deboor_gathered(degree, span, t, U, std::move(d)));
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
        hi = U[(int)ctrl.size()];  // U[n+1]
        per = closed;
        period = closed ? (hi - lo) : 0.0;
    }
    int nominal_spans(double a, double b) const override {
        // ~2 per knot interval, clamped [4,64] (step2glb geom.rs)
        int spans = (int)U.size();
        int n = spans * 2;
        if (n < 4) n = 4;
        if (n > 64) n = 64;
        double lo, hi, per;
        bool pr;
        range(lo, hi, pr, per);
        double frac = (hi > lo) ? std::abs(b - a) / (hi - lo) : 1.0;
        int m = (int)std::ceil(n * frac);
        return m < 1 ? 1 : m;
    }
};

// Generic seeded-Newton UV inversion for surfaces without a closed form (B-spline, swept).
// Seeds from (useed,vseed) when finite, else a coarse grid scan over the domain. FD Jacobian,
// Gauss-Newton step on the 3x2 system, clamped to the domain. Returns false if it can't converge.
inline bool newton_uv(const Surface &s, const Vec3 &p, double umin, double umax, double vmin,
                      double vmax, double useed, double vseed, double &u, double &v) {
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
        if (r.norm() < 1e-10) return true;
        Vec3 Su = (s.point(u + hu, v) - s.point(u - hu, v)) * (0.5 / hu);
        Vec3 Sv = (s.point(u, v + hv) - s.point(u, v - hv)) * (0.5 / hv);
        // solve [Su Sv] d = -r  (least squares: normal equations 2x2)
        double a = Su.dot(Su), b = Su.dot(Sv), c = Sv.dot(Sv);
        double ru = -Su.dot(r), rv = -Sv.dot(r);
        double det = a * c - b * b;
        if (std::abs(det) < 1e-300) break;
        double du = (ru * c - rv * b) / det;
        double dv = (a * rv - b * ru) / det;
        u = clampd(u + du, umin, umax);
        v = clampd(v + dv, vmin, vmax);
        if (std::abs(du) < 1e-12 && std::abs(dv) < 1e-12) break;
    }
    return (s.point(u, v) - p).norm() < 1e-6;
}

struct BSplineSurface : Surface {
    int u_degree = 1, v_degree = 1;
    int nu = 0, nv = 0;          // #ctrl in u, v
    std::vector<Vec3> ctrl;      // row-major: index = iu*nv + iv
    std::vector<double> Uu, Uv;  // flat knots
    std::vector<double> weights;  // empty => non-rational; else size nu*nv
    bool u_closed = false, v_closed = false;

    void domain(double &umin, double &umax, double &vmin, double &vmax) const {
        umin = Uu[u_degree];
        umax = Uu[nu];  // Uu[ (nu-1)+1 ]
        vmin = Uv[v_degree];
        vmax = Uv[nv];
    }

    Vec3 point(double u, double v) const override {
        using namespace bspline_detail;
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
    Vec3 normal(double u, double v) const override { return fd_normal(u, v); }
    bool uv(const Vec3 &p, double uhint, double vhint, double &u, double &v) const override {
        double umin, umax, vmin, vmax;
        domain(umin, umax, vmin, vmax);
        return newton_uv(*this, p, umin, umax, vmin, vmax, uhint, vhint, u, v);
    }
    void periods(bool &up, double &uper, bool &vp, double &vper) const override {
        double umin, umax, vmin, vmax;
        domain(umin, umax, vmin, vmax);
        up = u_closed;
        vp = v_closed;
        uper = u_closed ? (umax - umin) : 0.0;
        vper = v_closed ? (vmax - vmin) : 0.0;
    }
};

// Surface swept by translating a profile curve along a direction (spec §5 SURF_LIN_EXTRUSION).
// u = the profile curve's own parameter; v = distance along dir in [0, depth].
struct LinearExtrusionSurface : Surface {
    std::shared_ptr<Curve> profile;
    Vec3 dir;  // unit
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
    Vec3 point(double u, double v) const override { return profile->point(u) + dir * v; }
    Vec3 normal(double u, double v) const override {
        return profile->tangent(u).cross(dir).normalized();
    }
    bool uv(const Vec3 &p, double uhint, double vhint, double &u, double &v) const override {
        double umin, umax, vmin, vmax;
        domain(umin, umax, vmin, vmax);
        return newton_uv(*this, p, umin, umax, vmin, vmax, uhint, vhint, u, v);
    }
};

// Surface swept by revolving a profile curve about an axis (spec §5 SURF_REVOLUTION).
// u = revolution angle [0,2pi) (periodic); v = the profile curve's own parameter.
struct RevolutionSurface : Surface {
    std::shared_ptr<Curve> profile;
    Vec3 axis_loc, axis_dir;  // axis_dir unit
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
    Vec3 point(double u, double v) const override { return rotate(profile->point(v), u); }
    Vec3 normal(double u, double v) const override { return fd_normal(u, v); }
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
};

}  // namespace adacpp::ngeom
