// NGEOM — neutral geometry math primitives (header-only, no external deps).
// Part of adacpp's OCC-free neutral geometry layer (see
// dap/plan/v3/spec_neutral_geometry_schema.md). Deliberately tiny and dependency-free so
// it compiles for native and wasm and can be unit-tested standalone.
#pragma once

#include <array>
#include <cmath>

namespace adacpp::ngeom {

constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;

struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;

    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3 &o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3 &o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }

    double dot(const Vec3 &o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3 &o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    double norm() const { return std::sqrt(dot(*this)); }
    Vec3 normalized() const {
        double n = norm();
        return n > 1e-300 ? Vec3{x / n, y / n, z / n} : Vec3{0, 0, 0};
    }
};

inline Vec3 operator*(double s, const Vec3 &v) { return v * s; }

// Right-handed orthonormal frame: z = main axis, x = ref direction, y = z x x.
// Built from a (location, axis, ref_direction) triple that may be non-orthonormal /
// non-unit (as authored upstream); we re-orthonormalize defensively.
struct Frame {
    Vec3 o{0, 0, 0};
    Vec3 x{1, 0, 0};
    Vec3 y{0, 1, 0};
    Vec3 z{0, 0, 1};

    static Frame from_axis_ref(const Vec3 &loc, const Vec3 &axis, const Vec3 &ref) {
        Frame f;
        f.o = loc;
        f.z = axis.normalized();
        if (f.z.norm() < 0.5) f.z = {0, 0, 1};
        // project ref onto the plane perpendicular to z
        Vec3 rx = ref - f.z * f.z.dot(ref);
        if (rx.norm() < 1e-9) {
            // ref parallel to axis (or absent): pick any perpendicular
            rx = std::abs(f.z.x) < 0.9 ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
            rx = rx - f.z * f.z.dot(rx);
        }
        f.x = rx.normalized();
        f.y = f.z.cross(f.x);
        return f;
    }

    // world point from local coordinates
    Vec3 to_world(double lx, double ly, double lz) const { return o + x * lx + y * ly + z * lz; }
    // local coordinates of a world point (relative to o)
    Vec3 to_local(const Vec3 &p) const {
        Vec3 d = p - o;
        return {d.dot(x), d.dot(y), d.dot(z)};
    }
};

inline double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

// wrap an angle into [0, 2pi)
inline double wrap_2pi(double a) {
    a = std::fmod(a, TWO_PI);
    if (a < 0.0) a += TWO_PI;
    return a;
}

}  // namespace adacpp::ngeom
