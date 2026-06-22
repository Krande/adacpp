// NGEOM neutral surfaces — native evaluation, no OCC. See spec §5.
//
// Convention: normal(u,v) returns the surface's NATURAL outward normal (before the
// FaceSurface.same_sense flip, which the tessellator applies). Quadrics invert UV in
// closed form; B-spline/swept fall back to seeded Newton (ngeom_surfaces.cpp).
#pragma once

#include <memory>

#include "ngeom_curves.h"
#include "ngeom_math.h"

namespace adacpp::ngeom {

struct Surface {
    virtual ~Surface() = default;
    virtual Vec3 point(double u, double v) const = 0;
    virtual Vec3 normal(double u, double v) const = 0;
    // Invert a world point to (u,v); hints seed iterative inverters. Returns false if it
    // can't converge. Closed-form for quadrics (hints ignored).
    virtual bool uv(const Vec3 &p, double uhint, double vhint, double &u, double &v) const = 0;
    // periodicity per parameter (period>0 when periodic)
    virtual void periods(bool &u_periodic, double &u_period, bool &v_periodic, double &v_period) const {
        u_periodic = v_periodic = false;
        u_period = v_period = 0.0;
    }
    // a (u,v) that sits on a degenerate cap (sphere pole / cone apex) — u is meaningless there
    virtual bool degenerate_at(double u, double v) const { return false; }

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
    Vec3 point(double u, double v) const override { return f.to_world(u, v, 0.0); }
    Vec3 normal(double, double) const override { return f.z; }
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
};

struct ConeSurface : Surface {
    Frame f;
    double r0;          // radius at v = 0
    double semi_angle;  // half-angle from axis
    ConeSurface(const Frame &fr, double radius, double a) : f(fr), r0(radius), semi_angle(a) {}
    double radius_at(double v) const { return r0 + v * std::sin(semi_angle); }
    Vec3 point(double u, double v) const override {
        double rr = radius_at(v);
        return f.to_world(rr * std::cos(u), rr * std::sin(u), v * std::cos(semi_angle));
    }
    Vec3 normal(double u, double v) const override {
        // outward normal of a cone: radial component cos(a), axial -sin(a)
        Vec3 radial = f.x * std::cos(u) + f.y * std::sin(u);
        return (radial * std::cos(semi_angle) - f.z * std::sin(semi_angle)).normalized();
    }
    bool uv(const Vec3 &p, double, double, double &u, double &v) const override {
        Vec3 l = f.to_local(p);
        double ca = std::cos(semi_angle);
        v = (std::abs(ca) > 1e-9) ? (l.z / ca) : 0.0;
        u = wrap_2pi(std::atan2(l.y, l.x));
        return true;
    }
    void periods(bool &up, double &uper, bool &vp, double &vper) const override {
        up = true;
        uper = TWO_PI;
        vp = false;
        vper = 0;
    }
    bool degenerate_at(double, double v) const override { return std::abs(radius_at(v)) < 1e-9; }
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
        if (rr < 1e-12) return false;
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
    bool degenerate_at(double, double v) const override { return std::abs(std::cos(v)) < 1e-7; }
};

struct TorusSurface : Surface {
    Frame f;
    double R, r;  // major, minor
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
};

}  // namespace adacpp::ngeom
