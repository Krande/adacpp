// NGEOM neutral surfaces — native evaluation, no OCC. See spec §5.
//
// Convention: normal(u,v) returns the surface's NATURAL outward normal (before the
// FaceSurface.same_sense flip, which the tessellator applies). Quadrics invert UV in
// closed form; B-spline/swept fall back to seeded Newton (ngeom_surfaces.cpp).
#pragma once

#include <limits>
#include <memory>
#include <optional>
#include <utility>

#include "ngeom_curves.h"
#include "ngeom_math.h"

namespace adacpp::ngeom {

// Angular sampling step (radians) so a circular arc of radius r is approximated within chord
// sag `defl`, clamped to [2deg, max_angle]. Ported from step2glb geom.rs angle_step — the
// curvature-driven density that drives grid resolution on quadric/swept surfaces.
inline double angle_step(double r, double defl, double max_angle) {
    if (r < 1e-12)
        return max_angle;
    double ratio = clampd(1.0 - defl / r, -1.0, 1.0);
    double a = 2.0 * std::acos(ratio);
    double lo = 2.0 * PI / 180.0;
    double hi = std::max(max_angle, lo);
    return clampd(a, lo, hi);
}

struct Surface {
    virtual ~Surface() = default;
    virtual Vec3 point(double u, double v) const = 0;
    virtual Vec3 normal(double u, double v) const = 0;
    // Invert a world point to (u,v); hints seed iterative inverters. Returns false if it
    // can't converge. Closed-form for quadrics (hints ignored).
    virtual bool uv(const Vec3 &p, double uhint, double vhint, double &u, double &v) const = 0;
    // Curvature-driven parametric sampling step (radians/units) per direction; INFINITY means
    // "flat — no density floor, refinement decides". Mirrors step2glb u_step/v_step.
    virtual double u_step(double defl, double max_angle) const {
        return std::numeric_limits<double>::infinity();
    }
    virtual double v_step(double defl, double max_angle) const {
        return std::numeric_limits<double>::infinity();
    }
    // periodicity per parameter (period>0 when periodic)
    virtual void periods(bool &u_periodic, double &u_period, bool &v_periodic, double &v_period) const {
        u_periodic = v_periodic = false;
        u_period = v_period = 0.0;
    }
    // a (u,v) that sits on a degenerate cap (sphere pole / cone apex) — u is meaningless there
    virtual bool degenerate_at(double u, double v) const {
        return false;
    }

    // --- step2glb geom.rs Surface helpers (ported; used by the faithful tessellator) ---
    // Optional period in each parameter direction (nullopt = not periodic). Default derives
    // from periods(); B-spline/swept override where their closed-flag differs.
    virtual std::optional<double> u_period() const {
        bool up, vp;
        double uper, vper;
        periods(up, uper, vp, vper);
        return up ? std::optional<double>(uper) : std::nullopt;
    }
    virtual std::optional<double> v_period() const {
        bool up, vp;
        double uper, vper;
        periods(up, uper, vp, vper);
        return vp ? std::optional<double>(vper) : std::nullopt;
    }
    // v values where the surface degenerates to a point ("poles"): sphere ±pi/2, cone apex.
    virtual std::optional<std::pair<double, double>> v_caps() const {
        return std::nullopt;
    }
    // Characteristic model-space size, for chord-deviation tolerancing.
    virtual double approx_size() const {
        return 1.0;
    }
    // A quadric (cyl/cone/sphere/torus): closed-form exact UV inverse, so point_residual is a
    // reliable distance-to-surface (used to reject boundary geometry off the surface).
    virtual bool is_quadric() const {
        return false;
    }
    // 3D distance from p to this surface via the closed-form UV inverse (meaningful for quadrics).
    double point_residual(const Vec3 &p) const {
        double u, v;
        if (!uv(p, NAN_HINT, NAN_HINT, u, v))
            return 0.0;
        return (point(u, v) - p).norm();
    }
    static constexpr double NAN_HINT = std::numeric_limits<double>::quiet_NaN();

    // Finite-difference normal (fallback for surfaces without a closed form).
    Vec3 fd_normal(double u, double v, double h = 1e-6) const {
        Vec3 du = point(u + h, v) - point(u - h, v);
        Vec3 dv = point(u, v + h) - point(u, v - h);
        Vec3 n = du.cross(dv);
        if (n.norm() < 1e-14) {
            // perturb off a degenerate row
            dv = point(u, v + 4 * h) - point(u, v);
            n = du.cross(dv);
        }
        return n.normalized();
    }
};

struct PlaneSurface : Surface {
    Frame f;
    explicit PlaneSurface(const Frame &fr) : f(fr) {}
    Vec3 point(double u, double v) const override {
        return f.to_world(u, v, 0.0);
    }
    Vec3 normal(double, double) const override {
        return f.z;
    }
    bool uv(const Vec3 &p, double, double, double &u, double &v) const override {
        Vec3 l = f.to_local(p);
        u = l.x;
        v = l.y;
        return true;
    }
};

struct CylinderSurface : Surface {
    Frame f;
    double r;
    CylinderSurface(const Frame &fr, double radius) : f(fr), r(radius) {}
    Vec3 point(double u, double v) const override {
        return f.to_world(r * std::cos(u), r * std::sin(u), v);
    }
    Vec3 normal(double u, double) const override {
        return (f.x * std::cos(u) + f.y * std::sin(u)).normalized();
    }
    bool uv(const Vec3 &p, double, double, double &u, double &v) const override {
        Vec3 l = f.to_local(p);
        u = wrap_2pi(std::atan2(l.y, l.x));
        v = l.z;
        return true;
    }
    void periods(bool &up, double &uper, bool &vp, double &vper) const override {
        up = true;
        uper = TWO_PI;
        vp = false;
        vper = 0;
    }
    double u_step(double defl, double ma) const override {
        return angle_step(std::abs(r), defl, ma);
    }
    double approx_size() const override {
        return std::abs(r) * 2.0;
    }
    bool is_quadric() const override {
        return true;
    }
};

struct ConeSurface : Surface {
    Frame f;
    double r0;         // radius at v = 0
    double semi_angle; // half-angle from axis
    ConeSurface(const Frame &fr, double radius, double a) : f(fr), r0(radius), semi_angle(a) {}
    // step2glb geom.rs Surface::Cone parameterizes v as AXIAL height (z = v), radius = r + v·tan(a).
    // (NOT slant length — using sin/v·cos(a) is a different parameterization that over-samples cones.)
    double radius_at(double v) const {
        return r0 + v * std::tan(semi_angle);
    }
    Vec3 point(double u, double v) const override {
        double rr = radius_at(v);
        return f.to_world(rr * std::cos(u), rr * std::sin(u), v);
    }
    Vec3 normal(double u, double v) const override {
        // outward normal of a cone: radial component cos(a), axial -sin(a)
        Vec3 radial = f.x * std::cos(u) + f.y * std::sin(u);
        return (radial * std::cos(semi_angle) - f.z * std::sin(semi_angle)).normalized();
    }
    bool uv(const Vec3 &p, double uh, double, double &u, double &v) const override {
        Vec3 l = f.to_local(p);
        v = l.z; // axial height (step2glb Cone uv: v = d·axis)
        double rxy = std::sqrt(l.x * l.x + l.y * l.y);
        if (rxy < 1e-9 * std::max(l.norm(), 1.0)) {
            u = std::isnan(uh) ? 0.0 : uh; // at the apex u is undefined: keep the hint meridian
        } else {
            u = std::atan2(l.y, l.x);
            // beyond the apex the signed radius r+v·tan(a) flips, so the point is on the far nappe
            // at u+π (ISO 10303-42 conical_surface spans all v) — match step2glb exactly.
            if (r0 + v * std::tan(semi_angle) < 0.0)
                u += M_PI;
            u = wrap_2pi(u);
        }
        return true;
    }
    void periods(bool &up, double &uper, bool &vp, double &vper) const override {
        up = true;
        uper = TWO_PI;
        vp = false;
        vper = 0;
    }
    bool degenerate_at(double, double v) const override {
        return std::abs(radius_at(v)) < 1e-9;
    }
    double u_step(double defl, double ma) const override {
        return angle_step(std::abs(r0), defl, ma);
    }
    double approx_size() const override {
        return std::max(std::abs(r0), 1.0) * 2.0;
    }
    bool is_quadric() const override {
        return true;
    }
    std::optional<std::pair<double, double>> v_caps() const override {
        // apex is the v where radius_at(v)=r0+v*tan(a)=0 (axial-v parameterization, step2glb geom.rs).
        double t = std::tan(semi_angle);
        if (std::abs(t) < 1e-12)
            return std::nullopt;
        double apex = -r0 / t;
        if (apex <= 0.0)
            return std::make_pair(apex, std::numeric_limits<double>::infinity());
        return std::make_pair(-std::numeric_limits<double>::infinity(), apex);
    }
};

struct SphereSurface : Surface {
    Frame f;
    double r;
    SphereSurface(const Frame &fr, double radius) : f(fr), r(radius) {}
    // u: azimuth (2pi), v: latitude [-pi/2, pi/2]
    Vec3 point(double u, double v) const override {
        double cv = std::cos(v);
        return f.to_world(r * cv * std::cos(u), r * cv * std::sin(u), r * std::sin(v));
    }
    Vec3 normal(double u, double v) const override {
        double cv = std::cos(v);
        return (f.x * (cv * std::cos(u)) + f.y * (cv * std::sin(u)) + f.z * std::sin(v)).normalized();
    }
    bool uv(const Vec3 &p, double, double, double &u, double &v) const override {
        Vec3 l = f.to_local(p);
        double rr = l.norm();
        if (rr < 1e-12)
            return false;
        v = std::asin(clampd(l.z / rr, -1.0, 1.0));
        u = wrap_2pi(std::atan2(l.y, l.x));
        return true;
    }
    void periods(bool &up, double &uper, bool &vp, double &vper) const override {
        up = true;
        uper = TWO_PI;
        vp = false;
        vper = 0;
    }
    bool degenerate_at(double, double v) const override {
        return std::abs(std::cos(v)) < 1e-7;
    }
    double u_step(double defl, double ma) const override {
        return angle_step(std::abs(r), defl, ma);
    }
    double v_step(double defl, double ma) const override {
        return angle_step(std::abs(r), defl, ma);
    }
    double approx_size() const override {
        return std::abs(r) * 2.0;
    }
    bool is_quadric() const override {
        return true;
    }
    std::optional<std::pair<double, double>> v_caps() const override {
        return std::make_pair(-PI / 2.0, PI / 2.0);
    }
};

struct TorusSurface : Surface {
    Frame f;
    double R, r; // major, minor
    TorusSurface(const Frame &fr, double major, double minor) : f(fr), R(major), r(minor) {}
    Vec3 point(double u, double v) const override {
        double rr = R + r * std::cos(v);
        return f.to_world(rr * std::cos(u), rr * std::sin(u), r * std::sin(v));
    }
    Vec3 normal(double u, double v) const override {
        Vec3 radial = f.x * std::cos(u) + f.y * std::sin(u);
        return (radial * std::cos(v) + f.z * std::sin(v)).normalized();
    }
    bool uv(const Vec3 &p, double, double, double &u, double &v) const override {
        Vec3 l = f.to_local(p);
        u = wrap_2pi(std::atan2(l.y, l.x));
        double rho = std::sqrt(l.x * l.x + l.y * l.y) - R;
        v = wrap_2pi(std::atan2(l.z, rho));
        return true;
    }
    void periods(bool &up, double &uper, bool &vp, double &vper) const override {
        up = vp = true;
        uper = vper = TWO_PI;
    }
    double u_step(double defl, double ma) const override {
        return angle_step(std::abs(R) + std::abs(r), defl, ma);
    }
    double v_step(double defl, double ma) const override {
        return angle_step(std::abs(r), defl, ma);
    }
    double approx_size() const override {
        return std::abs(R + r) * 2.0;
    }
    bool is_quadric() const override {
        return true;
    }
};

} // namespace adacpp::ngeom
