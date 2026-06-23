// NGEOM neutral curves — native evaluation, no OCC. See spec §4.
#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "ngeom_math.h"

namespace adacpp::ngeom {

struct Curve {
    virtual ~Curve() = default;
    virtual Vec3 point(double t) const = 0;
    virtual Vec3 tangent(double t) const = 0;
    // Natural parameter range; periodic curves report period > 0.
    virtual void range(double &lo, double &hi, bool &periodic, double &period) const = 0;

    // Discretize into 3D points [t in [a,b]] honouring a chord deflection and a max
    // angular step between successive tangents. Midpoint-refines an initial nominal split
    // (step2glb geom.rs nominal_spans + refine). Endpoints a,b are always included.
    std::vector<Vec3> discretize(double a, double b, double deflection, double max_angle) const;
    // public accessor for nominal_spans (used by composite/trimmed curves)
    int discretize_spans(double a, double b) const;
    // >0 => an edge of this curve is sampled UNIFORMLY at this many segments instead of the adaptive
    // midpoint refine (step2glb samples B-spline edges uniformly at clamp(cps*4,8,512), NOT adaptively).
    virtual int uniform_edge_segments() const { return 0; }

  protected:
    // initial #spans a concrete curve suggests over [a,b] before refinement
    virtual int nominal_spans(double a, double b) const { return 1; }
};

struct LineCurve : Curve {
    Vec3 pnt, dir;
    LineCurve(const Vec3 &p, const Vec3 &d) : pnt(p), dir(d) {}
    Vec3 point(double t) const override { return pnt + dir * t; }
    Vec3 tangent(double) const override { return dir.normalized(); }
    void range(double &lo, double &hi, bool &per, double &period) const override {
        lo = 0;
        hi = 1;
        per = false;
        period = 0;
    }
    int nominal_spans(double, double) const override { return 1; }
};

struct CircleCurve : Curve {
    Frame f;
    double r;
    CircleCurve(const Frame &fr, double radius) : f(fr), r(radius) {}
    Vec3 point(double t) const override { return f.to_world(r * std::cos(t), r * std::sin(t), 0.0); }
    Vec3 tangent(double t) const override {
        return (f.x * (-std::sin(t)) + f.y * std::cos(t)).normalized();
    }
    void range(double &lo, double &hi, bool &per, double &period) const override {
        lo = 0;
        hi = TWO_PI;
        per = true;
        period = TWO_PI;
    }
    int nominal_spans(double, double) const override { return 16; }  // Rust Curve3::nominal_spans
};

struct EllipseCurve : Curve {
    Frame f;
    double a1, a2;  // semi axes along x, y
    EllipseCurve(const Frame &fr, double s1, double s2) : f(fr), a1(s1), a2(s2) {}
    Vec3 point(double t) const override { return f.to_world(a1 * std::cos(t), a2 * std::sin(t), 0.0); }
    Vec3 tangent(double t) const override {
        return (f.x * (-a1 * std::sin(t)) + f.y * (a2 * std::cos(t))).normalized();
    }
    void range(double &lo, double &hi, bool &per, double &period) const override {
        lo = 0;
        hi = TWO_PI;
        per = true;
        period = TWO_PI;
    }
    int nominal_spans(double, double) const override { return 16; }  // Rust Curve3::nominal_spans
};

// POLYLINE — an ordered list of points; the polyline IS the edge (spec §4).
struct PolylineCurve : Curve {
    std::vector<Vec3> pts;
    explicit PolylineCurve(std::vector<Vec3> p) : pts(std::move(p)) {}
    Vec3 point(double t) const override {  // t in [0,1] across the whole polyline
        if (pts.size() < 2) return pts.empty() ? Vec3{0, 0, 0} : pts[0];
        double s = clampd(t, 0.0, 1.0) * (pts.size() - 1);
        int i = std::min((int)s, (int)pts.size() - 2);
        return pts[i] + (pts[i + 1] - pts[i]) * (s - i);
    }
    Vec3 tangent(double t) const override {
        if (pts.size() < 2) return {1, 0, 0};
        double s = clampd(t, 0.0, 1.0) * (pts.size() - 1);
        int i = std::min((int)s, (int)pts.size() - 2);
        return (pts[i + 1] - pts[i]).normalized();
    }
    void range(double &lo, double &hi, bool &per, double &period) const override {
        lo = 0;
        hi = 1;
        per = false;
        period = 0;
    }
    int nominal_spans(double, double) const override { return std::max(1, (int)pts.size() - 1); }
};

// HYPERBOLA — P(u)=C+sa*cosh(u)*x+si*sinh(u)*y (ISO 10303-42; step2glb model.rs curve_polyline).
struct HyperbolaCurve : Curve {
    Frame f;
    double sa, si;
    HyperbolaCurve(const Frame &fr, double semi, double semi_imag) : f(fr), sa(semi), si(semi_imag) {}
    Vec3 point(double u) const override { return f.to_world(sa * std::cosh(u), si * std::sinh(u), 0.0); }
    Vec3 tangent(double u) const override {
        return (f.x * (sa * std::sinh(u)) + f.y * (si * std::cosh(u))).normalized();
    }
    double param(const Vec3 &p) const {  // invert: u = asinh(localY/si)
        return std::asinh(f.to_local(p).y / si);
    }
    void range(double &lo, double &hi, bool &per, double &period) const override {
        lo = -1;
        hi = 1;
        per = false;
        period = 0;
    }
};

// PARABOLA — P(u)=C+fd*u^2*x+2*fd*u*y (ISO 10303-42; step2glb model.rs curve_polyline).
struct ParabolaCurve : Curve {
    Frame f;
    double fd;
    ParabolaCurve(const Frame &fr, double focal) : f(fr), fd(focal) {}
    Vec3 point(double u) const override { return f.to_world(fd * u * u, 2.0 * fd * u, 0.0); }
    Vec3 tangent(double u) const override {
        return (f.x * (2.0 * fd * u) + f.y * (2.0 * fd)).normalized();
    }
    double param(const Vec3 &p) const { return f.to_local(p).y / (2.0 * fd); }
    void range(double &lo, double &hi, bool &per, double &period) const override {
        lo = -1;
        hi = 1;
        per = false;
        period = 0;
    }
};

// COMPOSITE_CURVE — an ordered chain of bounded parent curves (each oriented by same_sense).
struct CompositeCurveN : Curve {
    struct Seg {
        std::shared_ptr<Curve> parent;
        bool same_sense;
    };
    std::vector<Seg> segs;
    explicit CompositeCurveN(std::vector<Seg> s) : segs(std::move(s)) {}
    Vec3 point(double t) const override {
        if (segs.empty()) return {0, 0, 0};
        double s = clampd(t, 0.0, 1.0) * segs.size();
        int i = std::min((int)s, (int)segs.size() - 1);
        double local = s - i;
        const Seg &sg = segs[i];
        return sg.parent->point(sg.same_sense ? local : 1.0 - local);
    }
    Vec3 tangent(double t) const override {
        double s = clampd(t, 0.0, 1.0) * std::max((size_t)1, segs.size());
        int i = std::min((int)s, (int)segs.size() - 1);
        Vec3 tg = segs[i].parent->tangent(s - i);
        return segs[i].same_sense ? tg : tg * -1.0;
    }
    void range(double &lo, double &hi, bool &per, double &period) const override {
        lo = 0;
        hi = 1;
        per = false;
        period = 0;
    }
    // discretize the whole chain (each parent over its own range, oriented, stitched)
    std::vector<Vec3> chain(double deflection, double max_angle) const {
        std::vector<Vec3> out;
        for (const Seg &sg : segs) {
            double a, b;
            bool pr;
            double pe;
            sg.parent->range(a, b, pr, pe);
            std::vector<Vec3> sp = sg.parent->discretize(a, b, deflection, max_angle);
            if (!sg.same_sense) std::reverse(sp.begin(), sp.end());
            for (const Vec3 &p : sp)
                if (out.empty() || (p - out.back()).norm() > 1e-9) out.push_back(p);
        }
        return out;
    }
};

// A bounded segment of a basis curve (spec §4 TRIMMED_CURVE). The trim params are already
// resolved to basis-curve parameters by the serializer.
struct TrimmedCurve : Curve {
    std::shared_ptr<Curve> basis;
    double t1, t2;
    bool sense;  // true: traverse basis from t1->t2 in increasing sense
    TrimmedCurve(std::shared_ptr<Curve> b, double a, double c, bool s) : basis(b), t1(a), t2(c), sense(s) {}
    // local s in [0,1] maps to a basis parameter
    double map(double s) const { return sense ? (t1 + (t2 - t1) * s) : (t2 + (t1 - t2) * s); }
    Vec3 point(double s) const override { return basis->point(map(s)); }
    Vec3 tangent(double s) const override {
        Vec3 t = basis->tangent(map(s));
        return sense ? t : t * -1.0;
    }
    void range(double &lo, double &hi, bool &per, double &period) const override {
        lo = 0;
        hi = 1;
        per = false;
        period = 0;
    }
    int nominal_spans(double, double) const override {
        // borrow the basis' density over the trimmed sub-range
        return basis->discretize_spans(std::min(t1, t2), std::max(t1, t2));
    }
};

// --- discretize (chord-deflection + max-angle midpoint refinement) ---------------------

inline int Curve::discretize_spans(double a, double b) const { return nominal_spans(a, b); }

inline std::vector<Vec3> Curve::discretize(double a, double b, double deflection, double max_angle) const {
    std::vector<Vec3> out;
    int n0 = nominal_spans(a, b);
    if (n0 < 1) n0 = 1;
    // seed parameters
    std::vector<double> ts;
    ts.reserve(n0 + 1);
    for (int i = 0; i <= n0; ++i) ts.push_back(a + (b - a) * (double)i / (double)n0);

    // refine each span until chord sag < deflection AND turn angle < max_angle, bounded depth
    auto sag_ok = [&](double t0, double t1) -> bool {
        double tm = 0.5 * (t0 + t1);
        Vec3 p0 = point(t0), p1 = point(t1), pm = point(tm);
        // perpendicular distance of pm from chord p0-p1
        Vec3 chord = p1 - p0;
        double cl = chord.norm();
        double sag;
        if (cl < 1e-12) {
            sag = (pm - p0).norm();
        } else {
            Vec3 d = pm - p0;
            double proj = d.dot(chord) / (cl * cl);
            sag = (d - chord * proj).norm();
        }
        if (deflection > 0 && sag > deflection) return false;
        if (max_angle > 0) {
            Vec3 t0v = tangent(t0), t1v = tangent(t1);
            double c = clampd(t0v.dot(t1v), -1.0, 1.0);
            if (std::acos(c) > max_angle) return false;
        }
        return true;
    };

    std::vector<double> refined;
    refined.push_back(ts.front());
    for (size_t i = 0; i + 1 < ts.size(); ++i) {
        // recursive bisection of [ts[i], ts[i+1]]
        std::vector<std::pair<double, double>> stack{{ts[i], ts[i + 1]}};
        std::vector<double> pts;
        int guard = 0;
        // iterative subdivision with a depth cap (8 -> up to 256 sub-spans/span)
        std::vector<double> spanpts{ts[i], ts[i + 1]};
        for (int depth = 0; depth < 8; ++depth) {
            std::vector<double> next;
            bool split_any = false;
            for (size_t k = 0; k + 1 < spanpts.size(); ++k) {
                next.push_back(spanpts[k]);
                if (!sag_ok(spanpts[k], spanpts[k + 1])) {
                    next.push_back(0.5 * (spanpts[k] + spanpts[k + 1]));
                    split_any = true;
                }
            }
            next.push_back(spanpts.back());
            spanpts.swap(next);
            if (!split_any) break;
            if (++guard > 12) break;
        }
        for (size_t k = 1; k < spanpts.size(); ++k) refined.push_back(spanpts[k]);
    }
    out.reserve(refined.size());
    for (double t : refined) out.push_back(point(t));
    return out;
}

}  // namespace adacpp::ngeom
