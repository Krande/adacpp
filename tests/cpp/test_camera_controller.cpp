// CameraController (src/render/camera_controller.h): no GPU, so everything about it is checked here.
// Built and registered with ctest by src/render/CMakeLists.txt (pixi run -e render test-render).
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>

#include "camera_controller.h"

using namespace adacpp::render;

namespace {

int failures = 0, checks = 0;

void check(bool ok, const char *what, int line) {
    ++checks;
    if (!ok) {
        ++failures;
        std::fprintf(stderr, "FAILED line %d: %s\n", line, what);
    }
}
#define CHECK(cond) check((cond), #cond, __LINE__)

using Vec3 = std::array<double, 3>;
using Mat4 = std::array<float, 16>;
const double kPi = 3.14159265358979323846, kDeg = kPi / 180.0;

bool close(double a, double b, double tol = 1e-9) {
    return std::fabs(a - b) <= tol * std::max({1.0, std::fabs(a), std::fabs(b)});
}
bool close(const Vec3 &a, const Vec3 &b, double tol = 1e-9) {
    return close(a[0], b[0], tol) && close(a[1], b[1], tol) && close(a[2], b[2], tol);
}
bool close(const Mat4 &a, const Mat4 &b, double tol = 2e-6) {
    for (int i = 0; i < 16; ++i)
        if (!close(a[i], b[i], tol))
            return false;
    return true;
}

// The viewer's orbit camera this controller replaces (its matrices(), frame() and eye()), in the same
// arithmetic: the controller must give the same matrices for the same pose.
struct Reference {
    Vec3 target{0, 0, 0};
    double azimuth = -135 * kDeg, elevation = 30 * kDeg, distance = 1, fov = 45 * kDeg, radius = 1;

    void frame(const std::array<float, 6> &b, double aspect, double margin = 1.15) {
        target = {(b[0] + b[3]) / 2.0, (b[1] + b[4]) / 2.0, (b[2] + b[5]) / 2.0};
        radius = std::max(std::hypot(double(b[3]) - b[0], double(b[4]) - b[1], double(b[5]) - b[2]) / 2, 1e-3);
        const double a = std::max(aspect, 1e-3);
        const double fit_v = radius / std::sin(fov / 2);
        const double fit_h = radius / std::sin(std::atan(std::tan(fov / 2) * a));
        distance = std::max(fit_v, fit_h) * margin;
    }
    Vec3 eye() const {
        const double ce = std::cos(elevation);
        return {target[0] + distance * ce * std::sin(azimuth), target[1] + distance * ce * std::cos(azimuth),
                target[2] + distance * std::sin(elevation)};
    }
    void matrices(double aspect, Mat4 &view, Mat4 &proj) const {
        auto cross = [](const Vec3 &a, const Vec3 &b) {
            return Vec3{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
        };
        auto dot = [](const Vec3 &a, const Vec3 &b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; };
        auto norm = [](Vec3 a) {
            const double l = std::hypot(a[0], a[1], a[2]);
            return Vec3{a[0] / l, a[1] / l, a[2] / l};
        };
        const Vec3 e = eye();
        const Vec3 f = norm({target[0] - e[0], target[1] - e[1], target[2] - e[2]});
        const Vec3 s = norm(cross(f, {0, 0, 1}));
        const Vec3 u = cross(s, f);
        view = {};
        for (int k = 0; k < 3; ++k) {
            view[k * 4] = float(s[k]);
            view[k * 4 + 1] = float(u[k]);
            view[k * 4 + 2] = float(-f[k]);
        }
        view[12] = float(-dot(s, e));
        view[13] = float(-dot(u, e));
        view[14] = float(dot(f, e));
        view[15] = 1;
        const double near_z = std::max({distance - radius * 1.5, distance * 1e-3, 1e-4});
        const double far_z = distance + radius * 1.5;
        const double t = 1 / std::tan(fov / 2);
        proj = {};
        proj[0] = float(t / std::max(aspect, 1e-3));
        proj[5] = float(t);
        proj[10] = float((far_z + near_z) / (near_z - far_z));
        proj[11] = -1;
        proj[14] = float((2 * far_z * near_z) / (near_z - far_z));
    }
};

// World point -> pixel (x right, y down) of a width x height viewport, through the controller's matrices.
std::array<double, 2> project(const CameraController &c, const Vec3 &p, double width, double height) {
    const Mat4 v = c.view(), pr = c.projection(width / height);
    double in[4] = {p[0], p[1], p[2], 1}, eye[4], clip[4];
    for (int r = 0; r < 4; ++r)
        eye[r] = v[r] * in[0] + v[4 + r] * in[1] + v[8 + r] * in[2] + v[12 + r] * in[3];
    for (int r = 0; r < 4; ++r)
        clip[r] = pr[r] * eye[0] + pr[4 + r] * eye[1] + pr[8 + r] * eye[2] + pr[12 + r] * eye[3];
    return {(clip[0] / clip[3] * 0.5 + 0.5) * width, (0.5 - clip[1] / clip[3] * 0.5) * height};
}

bool same_pose(const CameraPose &a, const CameraPose &b, double tol = 1e-9) {
    return close(a.eye, b.eye, tol) && close(a.target, b.target, tol) && close(a.up, b.up, tol) &&
           close(a.fov_deg, b.fov_deg, tol) && a.orthographic == b.orthographic &&
           close(a.ortho_height, b.ortho_height, tol);
}

const std::array<float, 6> kBox{-2, -1, 0, 6, 4, 3};
const uint32_t kW = 800, kH = 600;

CameraController framed() {
    CameraController c;
    c.set_viewport(kW, kH);
    c.set_scene_bounds(kBox);
    c.fit(kBox);
    return c;
}

void test_matches_reference() {
    // The default pose, framed: the viewer's first frame.
    for (double aspect : {800.0 / 600.0, 0.5, 2.5, 1.0}) {
        Reference ref;
        ref.frame(kBox, aspect);
        CameraController c;
        c.set_scene_bounds(kBox);
        c.fit_aspect(kBox, aspect);
        CHECK(close(c.distance(), ref.distance, 1e-12));
        CHECK(close(c.target(), ref.target, 1e-12));
        Mat4 view, proj;
        ref.matrices(aspect, view, proj);
        CHECK(close(c.view(), view));
        CHECK(close(c.projection(aspect), proj));
    }
    // Presets: azimuth, elevation, fov, and both a fitted and a given distance.
    const double poses[][4] = {{-135, 30, 45, 0}, {20, -40, 60, 0},  {180, 0, 30, 0},     {90, 75, 45, 12.5},
                               {-60, 10, 20, 40}, {135, -89, 45, 0}, {0.5, 89.9, 50, 7.0}};
    for (const auto &p : poses) {
        const double aspect = 1.6;
        Reference ref;
        ref.azimuth = p[0] * kDeg;
        ref.elevation = p[1] * kDeg;
        ref.fov = p[2] * kDeg;
        ref.frame(kBox, aspect);
        CameraController c;
        c.set_scene_bounds(kBox);
        c.set_fov(p[2]);
        c.set_orientation(p[0], p[1]);
        c.fit_aspect(kBox, aspect);
        if (p[3] > 0) {
            // A given distance, through the pose a host would hand over.
            ref.distance = p[3];
            CameraPose pose;
            pose.eye = ref.eye();
            pose.target = ref.target;
            pose.up = {0, 0, 1};
            pose.fov_deg = p[2];
            c.set_pose(pose);
        }
        CHECK(close(c.distance(), ref.distance, 1e-9));
        CHECK(close(c.pose().eye, ref.eye(), 1e-9));
        Mat4 view, proj;
        ref.matrices(aspect, view, proj);
        CHECK(close(c.view(), view));
        CHECK(close(c.projection(aspect), proj));
    }
}

void test_orbit() {
    CameraController c = framed();
    const double az = c.azimuth(), el = c.elevation(), d = c.distance();
    const Vec3 target = c.target();
    c.orbit(100, 50);
    CHECK(close(c.azimuth(), az + 100 * 180.0 / kH));
    CHECK(close(c.elevation(), el + 50 * 180.0 / kH));
    CHECK(close(c.distance(), d) && close(c.target(), target));
    const CameraPose p = c.pose();
    CHECK(close(std::hypot(p.eye[0] - target[0], p.eye[1] - target[1], p.eye[2] - target[2]), d));
    // A full viewport height turns half way round.
    c.orbit(0, -double(kH) * 4);
    CHECK(close(c.elevation(), -90.0));
    c.orbit(0, double(kH) * 4);
    CHECK(close(c.elevation(), 90.0));
    // Looking straight down is a valid pose: the matrices stay finite and orthonormal.
    const Mat4 v = c.view();
    for (float x : v)
        CHECK(std::isfinite(x));
    CHECK(close(double(v[0]) * v[0] + double(v[4]) * v[4] + double(v[8]) * v[8], 1.0, 1e-6));
    CHECK(close(double(v[0]) * v[1] + double(v[4]) * v[5] + double(v[8]) * v[9], 0.0, 1e-6));
}

void test_pan_follows_pointer() {
    for (bool ortho : {false, true}) {
        CameraController c = framed();
        c.set_orthographic(ortho);
        const Vec3 at = c.target();
        const auto before = project(c, at, kW, kH);
        CHECK(close(before[0], kW / 2.0, 1e-6) && close(before[1], kH / 2.0, 1e-6));
        const double d = c.distance();
        c.pan(37, -52);
        const auto after = project(c, at, kW, kH);
        CHECK(close(after[0], before[0] + 37, 1e-4));
        CHECK(close(after[1], before[1] - 52, 1e-4));
        CHECK(close(c.distance(), d));
    }
}

void test_zoom() {
    for (bool ortho : {false, true}) {
        CameraController c = framed();
        c.set_orthographic(ortho);
        const double d = c.distance(), h = c.ortho_height();
        c.zoom(0.5);
        CHECK(close(c.distance(), d * 0.5) && close(c.ortho_height(), h * 0.5));
        c.zoom(4.0);
        CHECK(close(c.distance(), d * 2.0));
        c.zoom(-1.0); // nonsense is ignored
        CHECK(close(c.distance(), d * 2.0));

        // zoom_at: the point of the target plane under the cursor stays under it.
        c = framed();
        c.set_orthographic(ortho);
        const double x = 610, y = 140;
        // Find that point: a point of the target plane is the target moved along the camera's right and up.
        const CameraPose p0 = c.pose();
        const Mat4 v = c.view();
        const Vec3 right{v[0], v[4], v[8]}, up{v[1], v[5], v[9]};
        const auto origin = project(c, p0.target, kW, kH);
        const Vec3 unit_r{p0.target[0] + right[0], p0.target[1] + right[1], p0.target[2] + right[2]};
        const double px_per_world = project(c, unit_r, kW, kH)[0] - origin[0];
        const double a = (x - origin[0]) / px_per_world, b = -(y - origin[1]) / px_per_world;
        const Vec3 anchor{p0.target[0] + a * right[0] + b * up[0], p0.target[1] + a * right[1] + b * up[1],
                          p0.target[2] + a * right[2] + b * up[2]};
        const auto at0 = project(c, anchor, kW, kH);
        CHECK(close(at0[0], x, 1e-4) && close(at0[1], y, 1e-4));
        for (double factor : {0.4, 1.7}) {
            const double before = c.distance();
            c.zoom_at(factor, x, y);
            const auto at = project(c, anchor, kW, kH);
            CHECK(close(at[0], x, 1e-4) && close(at[1], y, 1e-4));
            CHECK(close(c.distance(), before * factor));
        }
        // At the centre it is plain zoom: the target does not move.
        const Vec3 t = c.target();
        c.zoom_at(0.5, kW / 2.0, kH / 2.0);
        CHECK(close(c.target(), t));
    }
    // Limits come from the scene's size: zoom never runs away from it or through it.
    CameraController c = framed();
    const double radius = std::hypot(8.0, 5.0, 3.0) / 2;
    for (int i = 0; i < 200; ++i)
        c.zoom(0.5);
    CHECK(close(c.distance(), radius * 1e-3));
    for (int i = 0; i < 200; ++i)
        c.zoom(2.0);
    CHECK(close(c.distance(), radius * 200));
}

void test_fit() {
    // The sphere around the bounds is inside the view, and tight to it in the narrower direction.
    for (double aspect : {4.0 / 3.0, 0.4, 3.0}) {
        for (bool ortho : {false, true}) {
            CameraController c;
            const uint32_t w = uint32_t(600 * aspect), h = 600;
            c.set_viewport(w, h);
            c.set_orthographic(ortho);
            c.set_orientation(40, 25);
            c.fit(kBox, 1.0);
            CHECK(close(c.target(), Vec3{2, 1.5, 1.5}));
            double lo_x = 1e30, hi_x = -1e30, lo_y = 1e30, hi_y = -1e30;
            for (int corner = 0; corner < 8; ++corner) {
                const Vec3 p{kBox[(corner & 1) ? 3 : 0], kBox[(corner & 2) ? 4 : 1], kBox[(corner & 4) ? 5 : 2]};
                const auto px = project(c, p, w, h);
                lo_x = std::min(lo_x, px[0]), hi_x = std::max(hi_x, px[0]);
                lo_y = std::min(lo_y, px[1]), hi_y = std::max(hi_y, px[1]);
            }
            CHECK(lo_x >= -1e-6 && hi_x <= w + 1e-6 && lo_y >= -1e-6 && hi_y <= h + 1e-6);
            if (ortho) {
                const double radius = std::hypot(8.0, 5.0, 3.0) / 2;
                CHECK(close(c.ortho_height() * std::min(aspect, 1.0), 2 * radius, 1e-3));
            }
        }
    }
    // A margin scales the distance; nonsense falls back to the default.
    CameraController a = framed(), b = framed(), d = framed();
    a.fit(kBox, 1.0);
    b.fit(kBox, 2.0);
    d.fit(kBox, 0.0);
    CHECK(close(b.distance(), 2 * a.distance()));
    CHECK(close(d.distance(), CameraController::kDefaultFitMargin * a.distance()));
    // fit keeps the direction it is seen from.
    CameraController e = framed();
    e.set_orientation(77, -12, 5);
    e.fit(kBox);
    CHECK(close(e.azimuth(), 77.0) && close(e.elevation(), -12.0) && close(e.roll_angle(), 5.0));
}

void test_named_views() {
    struct Want {
        const char *name;
        Vec3 from; // direction from the target to the camera
        Vec3 right, up;
    };
    const double s = 1 / std::sqrt(3.0), h = 1 / std::sqrt(2.0), q = 1 / std::sqrt(6.0);
    const Want z_up[] = {
        {"top", {0, 0, 1}, {1, 0, 0}, {0, 1, 0}},        {"bottom", {0, 0, -1}, {1, 0, 0}, {0, -1, 0}},
        {"front", {0, -1, 0}, {1, 0, 0}, {0, 0, 1}},     {"back", {0, 1, 0}, {-1, 0, 0}, {0, 0, 1}},
        {"left", {-1, 0, 0}, {0, -1, 0}, {0, 0, 1}},     {"right", {1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
        {"iso", {-s, -s, s}, {h, -h, 0}, {q, q, 2 * q}},
    };
    for (const Want &w : z_up) {
        CameraController c = framed();
        c.roll(20); // a named view clears it
        CHECK(c.set_view(w.name));
        const CameraPose p = c.pose();
        const double d = c.distance();
        CHECK(close(Vec3{(p.eye[0] - p.target[0]) / d, (p.eye[1] - p.target[1]) / d, (p.eye[2] - p.target[2]) / d},
                    w.from, 1e-9));
        const Mat4 v = c.view();
        CHECK(close(Vec3{v[0], v[4], v[8]}, w.right, 1e-6));
        CHECK(close(Vec3{v[1], v[5], v[9]}, w.up, 1e-6));
        CHECK(close(p.up, w.up, 1e-9));
        CHECK(close(c.target(), Vec3{2, 1.5, 1.5}) && close(c.distance(), d));
    }
    CameraController c = framed();
    const CameraPose before = c.pose();
    CHECK(!c.set_view("sideways"));
    CHECK(same_pose(c.pose(), before));

    // y up: the same views in a glTF-style frame.
    const Want y_up[] = {
        {"top", {0, 1, 0}, {1, 0, 0}, {0, 0, -1}},
        {"front", {0, 0, 1}, {1, 0, 0}, {0, 1, 0}},
        {"right", {1, 0, 0}, {0, 0, -1}, {0, 1, 0}},
        {"iso", {-s, s, s}, {h, 0, h}, {q, 2 * q, -q}},
    };
    for (const Want &w : y_up) {
        CameraController y = framed();
        y.set_up_axis(UpAxis::Y);
        CHECK(y.set_view(w.name));
        const CameraPose p = y.pose();
        const double d = y.distance();
        CHECK(close(Vec3{(p.eye[0] - p.target[0]) / d, (p.eye[1] - p.target[1]) / d, (p.eye[2] - p.target[2]) / d},
                    w.from, 1e-9));
        const Mat4 v = y.view();
        CHECK(close(Vec3{v[0], v[4], v[8]}, w.right, 1e-6));
        CHECK(close(Vec3{v[1], v[5], v[9]}, w.up, 1e-6));
    }
}

void test_pose_round_trip() {
    for (UpAxis axis : {UpAxis::Z, UpAxis::Y}) {
        CameraController c;
        c.set_up_axis(axis);
        CameraPose p;
        p.eye = {3, -7, 4};
        p.target = {1, 2, 0.5};
        p.up = {0.1, 0.2, 0.9}; // not perpendicular to the view: only its part across the view counts
        p.fov_deg = 35;
        c.set_pose(p);
        CameraPose got = c.pose();
        CHECK(close(got.eye, p.eye) && close(got.target, p.target) && close(got.fov_deg, 35.0));
        // The returned up is the given one made perpendicular to the view, unit length.
        const Vec3 f{p.target[0] - p.eye[0], p.target[1] - p.eye[1], p.target[2] - p.eye[2]};
        const double ff = f[0] * f[0] + f[1] * f[1] + f[2] * f[2];
        const double uf = (p.up[0] * f[0] + p.up[1] * f[1] + p.up[2] * f[2]) / ff;
        Vec3 flat{p.up[0] - uf * f[0], p.up[1] - uf * f[1], p.up[2] - uf * f[2]};
        const double l = std::hypot(flat[0], flat[1], flat[2]);
        CHECK(close(got.up, Vec3{flat[0] / l, flat[1] / l, flat[2] / l}));
        // What comes out goes back in unchanged.
        c.set_pose(got);
        CHECK(same_pose(c.pose(), got));

        // Straight down, where the up vector alone says which way the picture turns.
        CameraPose down;
        down.target = {1, 2, 3};
        down.eye = axis == UpAxis::Z ? Vec3{1, 2, 13} : Vec3{1, 12, 3};
        down.up = axis == UpAxis::Z ? Vec3{-1, 0, 0} : Vec3{-1, 0, 0};
        down.orthographic = true;
        down.ortho_height = 6;
        c.set_pose(down);
        got = c.pose();
        CHECK(close(got.eye, down.eye) && close(got.up, down.up));
        CHECK(got.orthographic && close(got.ortho_height, 6.0));
        CHECK(close(c.elevation(), 90.0));
    }
    // An orthographic pose without a height takes what the perspective view shows at the target.
    CameraController c;
    CameraPose p;
    p.eye = {0, -10, 0};
    p.orthographic = true;
    p.ortho_height = 0;
    c.set_pose(p);
    CHECK(close(c.ortho_height(), 2 * 10 * std::tan(22.5 * kDeg)));
}

void test_roll() {
    CameraController c = framed();
    const Mat4 v = c.view();
    const Vec3 t = c.target();
    const Vec3 right_of_centre{t[0] + v[0], t[1] + v[4], t[2] + v[8]};
    const auto before = project(c, right_of_centre, kW, kH);
    CHECK(before[0] > kW / 2.0 && close(before[1], kH / 2.0, 1e-4));
    c.roll(90);
    const auto after = project(c, right_of_centre, kW, kH);
    // Counter-clockwise: what was right of the centre is now above it, as far away.
    CHECK(close(after[0], kW / 2.0, 1e-4));
    CHECK(close(kH / 2.0 - after[1], before[0] - kW / 2.0, 1e-4));
    c.roll(300);
    CHECK(close(c.roll_angle(), 30.0));
}

void test_orbit_pivot() {
    CameraController c = framed();
    const Vec3 pivot{5.5, 3.0, 2.5};
    const auto before = project(c, pivot, kW, kH);
    const double d = c.distance();
    c.set_orbit_pivot(pivot);
    c.orbit(140, -60);
    c.orbit(-20, 35);
    const auto after = project(c, pivot, kW, kH);
    CHECK(close(after[0], before[0], 1e-3) && close(after[1], before[1], 1e-3));
    CHECK(close(c.distance(), d));
    CHECK(!close(c.target(), Vec3{2, 1.5, 1.5})); // the target swung round the pivot
    // Without one, the target is the pivot.
    c.clear_orbit_pivot();
    const Vec3 t = c.target();
    c.orbit(50, 50);
    CHECK(close(c.target(), t));
    // fit forgets the pivot.
    c.set_orbit_pivot(pivot);
    c.fit(kBox);
    c.orbit(50, 50);
    CHECK(close(c.target(), Vec3{2, 1.5, 1.5}));
}

void test_transitions() {
    // set_pose
    {
        CameraController c = framed();
        const CameraPose from = c.pose();
        CameraPose to;
        to.eye = {20, 5, 9};
        to.target = {4, 4, 1};
        to.up = {0, 0, 1};
        to.fov_deg = 30;
        CameraController direct = framed();
        direct.set_pose(to);
        const CameraPose want = direct.pose();
        c.set_pose(to, 0.5);
        CHECK(c.moving());
        CHECK(same_pose(c.pose(), CameraPose{from.eye, from.target, from.up, from.fov_deg, false, from.ortho_height}));
        int steps = 0;
        double last = 1e30;
        bool monotonic = true;
        while (c.update(1.0 / 60.0)) {
            ++steps;
            const CameraPose p = c.pose();
            const double left = std::hypot(p.target[0] - 4, p.target[1] - 4, p.target[2] - 1);
            monotonic &= left <= last + 1e-12;
            last = left;
            if (steps > 1000)
                break;
        }
        CHECK(steps == 29 || steps == 30); // half a second of 60 Hz frames, give or take the rounding
        CHECK(monotonic);
        CHECK(!c.moving());
        CHECK(same_pose(c.pose(), want, 1e-12)); // the goal itself, not a step short of it
        CHECK(!c.update(1.0 / 60.0));
    }
    // fit and a named view, together
    {
        CameraController c = framed();
        CameraController direct = framed();
        const std::array<float, 6> part{4, 2, 2, 5, 3, 3};
        direct.set_view("top");
        direct.fit(part);
        c.set_view("top", 0.3);
        c.fit(part, CameraController::kDefaultFitMargin, 0.3);
        CHECK(c.update(0.1));
        CHECK(c.elevation() > 30.0 && c.elevation() < 90.0); // on its way
        CHECK(c.update(0.1));
        CHECK(!c.update(0.2));
        CHECK(same_pose(c.pose(), direct.pose(), 1e-12));
    }
    // The short way round: from azimuth 170 to -170 is 20 degrees, through 180.
    {
        CameraController c = framed();
        c.set_orientation(170, 0);
        c.set_orientation(-170, 0, 0, 1.0);
        c.update(0.5);
        CHECK(close(std::fabs(c.azimuth()), 180.0, 1e-9));
        c.update(0.6);
        CHECK(close(c.azimuth(), -170.0));
    }
    // Input takes over from a transition where it is.
    {
        CameraController c = framed();
        c.set_view("top", 1.0);
        c.update(0.5);
        const double el = c.elevation();
        c.orbit(0, 0);
        CHECK(!c.moving());
        CHECK(!c.update(0.25));
        CHECK(close(c.elevation(), el));
    }
    // One big step lands on the goal too.
    {
        CameraController c = framed();
        c.set_view("front", 0.2);
        CHECK(!c.update(5.0));
        CHECK(close(c.azimuth(), 180.0) && close(c.elevation(), 0.0));
    }
}

void test_damping() {
    // Off: nothing moves after the input.
    CameraController off = framed();
    off.orbit(30, 0);
    const double az_off = off.azimuth();
    CHECK(!off.update(1.0 / 60.0));
    CHECK(close(off.azimuth(), az_off));

    for (bool pan : {false, true}) {
        CameraController c = framed();
        c.set_damping(0.9);
        const double dt = 1.0 / 60.0;
        auto where = [&] { return pan ? c.target()[0] : c.azimuth(); };
        const double start = where();
        if (pan)
            c.pan(30, 0);
        else
            c.orbit(30, 0);
        const double applied = where();
        CHECK(!close(applied, start)); // the drag itself is applied at once, in full
        CHECK(c.update(dt));           // and remembered as a speed
        CHECK(close(where(), applied));
        // Let go: it glides on, each frame a little less, and comes to rest.
        double last = where(), last_step = 1e30;
        int frames = 0;
        bool decaying = true;
        while (c.update(dt) && frames < 10000) {
            const double step = std::fabs(where() - last);
            decaying &= step < last_step;
            last_step = step;
            last = where();
            ++frames;
        }
        CHECK(frames > 10 && frames < 200);
        CHECK(decaying);
        CHECK(!c.moving());
        // The glide adds the geometric tail of the last frame's drag: 30 px * 0.9 / (1 - 0.9), nearly.
        const double per_px = (applied - start) / 30.0;
        const double glided = (where() - applied) / per_px;
        CHECK(glided > 200 && glided < 270);
        // A transition ends a glide.
        if (pan)
            c.pan(30, 0);
        else
            c.orbit(30, 0);
        c.update(dt);
        c.set_view("top");
        CHECK(!c.update(dt));
    }
}

void test_orthographic() {
    CameraController c = framed();
    const Vec3 t = c.target();
    const Mat4 v = c.view();
    const Vec3 near_target{t[0] + 0.7 * v[0] + 0.3 * v[1], t[1] + 0.7 * v[4] + 0.3 * v[5],
                           t[2] + 0.7 * v[8] + 0.3 * v[9]};
    const auto persp = project(c, near_target, kW, kH);
    c.set_orthographic(true);
    const Mat4 p = c.projection(double(kW) / kH);
    CHECK(p[15] == 1.0f && p[11] == 0.0f);
    // The target plane keeps its size on screen across the switch, both ways.
    const auto ortho = project(c, near_target, kW, kH);
    CHECK(close(ortho[0], persp[0], 1e-4) && close(ortho[1], persp[1], 1e-4));
    c.zoom(0.5);
    c.set_orthographic(false);
    const auto back = project(c, near_target, kW, kH);
    CHECK(close(back[0] - kW / 2.0, 2 * (persp[0] - kW / 2.0), 1e-4));
    CHECK(c.projection(1.0)[11] == -1.0f);
}

void test_depth_range() {
    // Wherever the camera goes, the scene's bounding sphere stays between near and far.
    CameraController c = framed();
    const Vec3 center{2, 1.5, 1.5};
    const double radius = std::hypot(8.0, 5.0, 3.0) / 2;
    auto covered = [&] {
        const Mat4 v = c.view(), p = c.projection(double(kW) / kH);
        const double depth = -(v[2] * center[0] + v[6] * center[1] + v[10] * center[2] + v[14]);
        double near_z, far_z;
        if (c.orthographic()) {
            near_z = (p[14] + 1) / p[10], far_z = (p[14] - 1) / p[10];
        } else {
            near_z = p[14] / (p[10] - 1), far_z = p[14] / (p[10] + 1);
        }
        return near_z < far_z && far_z >= depth + radius && (near_z <= depth - radius || near_z <= c.distance() * 2e-3);
    };
    CHECK(covered());
    c.pan(3000, -2000);
    CHECK(covered());
    c.orbit(500, 200);
    c.zoom(0.05);
    CHECK(covered());
    c.zoom(100);
    CHECK(covered());
    c.set_orthographic(true);
    c.pan(-5000, 100);
    CHECK(covered());
    // Before any bounds are known the matrices are still usable.
    CameraController bare;
    for (float x : bare.projection(1.5))
        CHECK(std::isfinite(x));
    for (float x : bare.view())
        CHECK(std::isfinite(x));
}

void test_clip_range() {
    // Set planes are what the projection uses, whatever the camera does; (0, 0) gives the automatic ones back.
    CameraController c = framed();
    const Mat4 automatic = c.projection(1.5);
    const auto range = c.clip_range();
    CHECK(range[0] > 0 && range[1] > range[0]);
    CHECK(close(range[0], automatic[14] / (automatic[10] - 1), 1e-3 * range[0]));
    CHECK(close(range[1], automatic[14] / (automatic[10] + 1), 1e-3 * range[1]));
    auto planes = [&](double aspect) {
        const Mat4 p = c.projection(aspect);
        if (c.orthographic())
            return std::array<double, 2>{(p[14] + 1.0) / p[10], (p[14] - 1.0) / p[10]};
        return std::array<double, 2>{p[14] / (p[10] - 1.0), p[14] / (p[10] + 1.0)};
    };
    c.set_clip_range(0.0125, 4000.0);
    for (int step = 0; step < 3; ++step) {
        const auto np = planes(1.5);
        CHECK(close(np[0], 0.0125, 1e-6) && close(np[1], 4000.0, 4000.0 * 1e-5));
        CHECK(c.clip_range() == (std::array<double, 2>{0.0125, 4000.0}));
        c.zoom(0.1);
        c.pan(300, -200);
    }
    c.set_orthographic(true);
    const auto op = planes(1.5);
    // An orthographic near comes back from the float matrix to about 1e-4 here (it cancels against 1).
    CHECK(close(op[0], 0.0125, 1e-3) && close(op[1], 4000.0, 4000.0 * 1e-5));
    CHECK(c.clip_range() == (std::array<double, 2>{0.0125, 4000.0}));
    c.set_orthographic(false);
    // Only both above 0 set it.
    c.set_clip_range(0.5, 0.0);
    CHECK(c.clip_range()[0] != 0.5);
    c = framed();
    c.set_clip_range(0.0125, 4000.0);
    c.set_clip_range(0.0, 0.0);
    CHECK(c.projection(1.5) == automatic);
    CHECK(c.clip_range() == range);
}

void test_reversed_z() {
    // The renderer's projection: WebGPU depth, near -> 1, far -> 0; x, y and w as the OpenGL one has them.
    CameraController c = framed();
    for (const bool set : {false, true})
        for (const bool ortho : {false, true}) {
            c.set_clip_range(set ? 0.05 : 0.0, set ? 5e4 : 0.0);
            c.set_orthographic(ortho);
            const double aspect = 1.5;
            const Mat4 gl = c.projection(aspect), rz = c.projection_reversed_z(aspect);
            for (int col = 0; col < 4; ++col) {
                CHECK(rz[col * 4] == gl[col * 4] && rz[col * 4 + 1] == gl[col * 4 + 1]);
                CHECK(rz[col * 4 + 3] == gl[col * 4 + 3]);
            }
            const auto range = c.clip_range();
            const double n = range[0], f = range[1];
            // Depth at view distance d (view z = -d), as the GPU gets it: float matrix, double divide.
            auto depth = [&](double d) {
                const double z = double(rz[10]) * -d + rz[14], w = double(rz[11]) * -d + rz[15];
                return z / w;
            };
            CHECK(close(depth(n), 1.0, 1e-6) && std::fabs(depth(f)) < 1e-6);
            const double mid = 0.5 * (n + f);
            const double expected = ortho ? (f - mid) / (f - n) : n * (f - mid) / (mid * (f - n));
            CHECK(close(depth(mid), expected, 1e-5));
            CHECK(depth(n) > depth(mid) && depth(mid) > depth(f));
        }
    // The viewer's range for a large model (a ratio of 1e6): seen from 2000 away, surfaces half a unit apart
    // still land on different float depths -- a relative step of about 1e-7 of the distance.
    c.set_orthographic(false);
    c.set_clip_range(0.05, 5e4);
    const Mat4 rz = c.projection_reversed_z(1.5);
    auto float_depth = [&](float d) { return (rz[10] * -d + rz[14]) / d; };
    CHECK(float_depth(2000.0f) > float_depth(2000.5f));
    CHECK(float_depth(2000.0f) > float_depth(2000.01f));
    CHECK(float_depth(20.0f) > float_depth(20.0001f));
}

} // namespace

int main() {
    test_matches_reference();
    test_orbit();
    test_pan_follows_pointer();
    test_zoom();
    test_fit();
    test_named_views();
    test_pose_round_trip();
    test_roll();
    test_orbit_pivot();
    test_transitions();
    test_damping();
    test_orthographic();
    test_depth_range();
    test_clip_range();
    test_reversed_z();
    std::printf("camera controller: %d checks, %d failed\n", checks, failures);
    return failures ? 1 : 0;
}
