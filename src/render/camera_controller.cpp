#include "camera_controller.h"

#include <algorithm>
#include <cmath>

namespace adacpp::render {

namespace {

using Vec3 = std::array<double, 3>;

constexpr double kPi = 3.14159265358979323846;
constexpr double kRad = kPi / 180.0;
constexpr double kMinRadius = 1e-3;
// Zoom stops at these multiples of the scene's bounding radius.
constexpr double kMinDistanceRadii = 1e-3, kMaxDistanceRadii = 200.0;
// Near and far sit this many radii either side of the scene's centre (room for a morph, a helper).
constexpr double kDepthRadii = 1.5;
// Damping lets go below this drag speed (pixels / s).
constexpr double kRestSpeed = 1.0;
// The shortest time a drag is taken to have lasted when its speed is worked out (s).
constexpr double kShortestFrame = 1.0 / 250.0;

Vec3 add(const Vec3 &a, const Vec3 &b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
Vec3 sub(const Vec3 &a, const Vec3 &b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
Vec3 scale(const Vec3 &a, double s) {
    return {a[0] * s, a[1] * s, a[2] * s};
}
double dot(const Vec3 &a, const Vec3 &b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
double length(const Vec3 &a) {
    return std::sqrt(dot(a, a));
}

// The controller's own frame is z-up; a y-up world is that frame turned -90 degrees about x.
Vec3 to_world(const Vec3 &v, UpAxis axis) {
    return axis == UpAxis::Z ? v : Vec3{v[0], v[2], -v[1]};
}
Vec3 from_world(const Vec3 &v, UpAxis axis) {
    return axis == UpAxis::Z ? v : Vec3{v[0], -v[2], v[1]};
}

double wrap_degrees(double a) {
    a = std::fmod(a, 360.0);
    if (a > 180.0)
        a -= 360.0;
    else if (a <= -180.0)
        a += 360.0;
    return a;
}

double tan_half(double fov_deg) {
    return std::tan(fov_deg * kRad * 0.5);
}

double speed(const std::array<double, 2> &v) {
    return std::hypot(v[0], v[1]);
}

} // namespace

void CameraController::set_viewport(uint32_t width, uint32_t height) {
    width_ = std::max<uint32_t>(width, 1);
    height_ = std::max<uint32_t>(height, 1);
}

double CameraController::aspect() const {
    return double(width_) / double(height_);
}

void CameraController::set_up_axis(UpAxis axis) {
    up_axis_ = axis;
}

void CameraController::set_fov(double fov_deg) {
    const double fov = std::clamp(fov_deg, 1.0, 179.0);
    state_.fov = fov;
    if (in_transition_)
        from_.fov = to_.fov = fov;
}

void CameraController::set_orthographic(bool orthographic) {
    if (orthographic == ortho_)
        return;
    ortho_ = orthographic;
    auto convert = [&](State &s) {
        if (orthographic)
            s.ortho_height = 2.0 * s.distance * tan_half(s.fov);
        else
            s.distance = s.ortho_height / (2.0 * tan_half(s.fov));
    };
    convert(state_);
    if (in_transition_) {
        convert(from_);
        convert(to_);
    }
}

void CameraController::set_scene_bounds(const std::array<float, 6> &b) {
    has_scene_ = true;
    scene_center_ = {0.5 * (double(b[0]) + b[3]), 0.5 * (double(b[1]) + b[4]), 0.5 * (double(b[2]) + b[5])};
    scene_radius_ =
        std::max(std::hypot(double(b[3]) - b[0], double(b[4]) - b[1], double(b[5]) - b[2]) * 0.5, kMinRadius);
}

CameraController::Basis CameraController::basis(const State &s, UpAxis axis) {
    const double sa = std::sin(s.azimuth * kRad), ca = std::cos(s.azimuth * kRad);
    const double se = std::sin(s.elevation * kRad), ce = std::cos(s.elevation * kRad);
    // In closed form rather than by cross products with the up axis, so looking straight down or up is
    // as well defined as any other direction (the azimuth still says which way is up on screen there).
    const Vec3 forward{-ce * sa, -ce * ca, -se};
    const Vec3 right{-ca, sa, 0.0};
    const Vec3 up{-sa * se, -ca * se, ce};
    const double sr = std::sin(s.roll * kRad), cr = std::cos(s.roll * kRad);
    Basis b;
    b.right = to_world(sub(scale(right, cr), scale(up, sr)), axis);
    b.up = to_world(add(scale(up, cr), scale(right, sr)), axis);
    b.forward = to_world(forward, axis);
    return b;
}

CameraController::Vec3 CameraController::eye(const State &s, UpAxis axis) {
    const double sa = std::sin(s.azimuth * kRad), ca = std::cos(s.azimuth * kRad);
    const double se = std::sin(s.elevation * kRad), ce = std::cos(s.elevation * kRad);
    return add(s.target, scale(to_world({ce * sa, ce * ca, se}, axis), s.distance));
}

std::array<float, 16> CameraController::view() const {
    const Basis b = basis(state_, up_axis_);
    const Vec3 e = eye(state_, up_axis_);
    std::array<float, 16> v{};
    for (int k = 0; k < 3; ++k) {
        v[k * 4] = float(b.right[k]);
        v[k * 4 + 1] = float(b.up[k]);
        v[k * 4 + 2] = float(-b.forward[k]);
    }
    v[12] = float(-dot(b.right, e));
    v[13] = float(-dot(b.up, e));
    v[14] = float(dot(b.forward, e));
    v[15] = 1.0f;
    return v;
}

void CameraController::set_clip_range(double near_z, double far_z) {
    clip_ = near_z > 0.0 && far_z > near_z ? std::array<double, 2>{near_z, far_z} : std::array<double, 2>{0, 0};
}

std::array<double, 2> CameraController::clip_range() const {
    if (clip_[0] > 0.0)
        return clip_;
    // Near and far hug the scene's bounding sphere, measured along the viewing direction from where the
    // camera is now, so the depth range stays tight after a pan or a zoom onto a detail.
    const bool scene = scene_radius_ > 0.0;
    const double radius = scene ? scene_radius_ : std::max(state_.distance, kMinRadius);
    const Vec3 center = scene ? scene_center_ : state_.target;
    const double depth = dot(sub(center, eye(state_, up_axis_)), basis(state_, up_axis_).forward);
    if (ortho_)
        return {depth - radius * kDepthRadii, depth + radius * kDepthRadii};
    const double near_z = std::max({depth - radius * kDepthRadii, state_.distance * 1e-3, 1e-4});
    double far_z = depth + radius * kDepthRadii;
    if (far_z <= near_z) // the whole scene is behind the camera: any valid range will do
        far_z = near_z * 2.0;
    return {near_z, far_z};
}

std::array<float, 16> CameraController::projection(double aspect) const {
    const double a = std::max(aspect, 1e-3);
    const auto [near_z, far_z] = clip_range();
    std::array<float, 16> p{};
    if (ortho_) {
        p[0] = float(2.0 / (state_.ortho_height * a));
        p[5] = float(2.0 / state_.ortho_height);
        p[10] = float(-2.0 / (far_z - near_z));
        p[14] = float(-(far_z + near_z) / (far_z - near_z));
        p[15] = 1.0f;
        return p;
    }
    const double t = 1.0 / tan_half(state_.fov);
    p[0] = float(t / a);
    p[5] = float(t);
    p[10] = float((far_z + near_z) / (near_z - far_z));
    p[11] = -1.0f;
    p[14] = float((2.0 * far_z * near_z) / (near_z - far_z));
    return p;
}

double CameraController::world_per_pixel() const {
    return (ortho_ ? state_.ortho_height : 2.0 * state_.distance * tan_half(state_.fov)) / double(height_);
}

void CameraController::stop_transition() {
    in_transition_ = false;
}

void CameraController::apply_orbit(double dx, double dy) {
    const double k = 180.0 / double(height_);
    if (!has_pivot_) {
        state_.azimuth = wrap_degrees(state_.azimuth + dx * k);
        state_.elevation = std::clamp(state_.elevation + dy * k, -90.0, 90.0);
        return;
    }
    // Turn about the pivot: it keeps its camera-space position, so it stays where it is on screen.
    const Basis b0 = basis(state_, up_axis_);
    const Vec3 rel = sub(pivot_, eye(state_, up_axis_));
    const Vec3 local{dot(rel, b0.right), dot(rel, b0.up), dot(rel, b0.forward)};
    state_.azimuth = wrap_degrees(state_.azimuth + dx * k);
    state_.elevation = std::clamp(state_.elevation + dy * k, -90.0, 90.0);
    const Basis b1 = basis(state_, up_axis_);
    Vec3 e = pivot_;
    e = sub(e, scale(b1.right, local[0]));
    e = sub(e, scale(b1.up, local[1]));
    e = sub(e, scale(b1.forward, local[2]));
    state_.target = add(e, scale(b1.forward, state_.distance));
}

void CameraController::apply_pan(double dx, double dy) {
    const Basis b = basis(state_, up_axis_);
    const double per_px = world_per_pixel();
    state_.target = add(state_.target, add(scale(b.right, -dx * per_px), scale(b.up, dy * per_px)));
}

void CameraController::orbit(double dx, double dy) {
    stop_transition();
    apply_orbit(dx, dy);
    if (inertia_ > 0.0) {
        orbit_pending_[0] += dx;
        orbit_pending_[1] += dy;
    }
}

void CameraController::pan(double dx, double dy) {
    stop_transition();
    apply_pan(dx, dy);
    if (inertia_ > 0.0) {
        pan_pending_[0] += dx;
        pan_pending_[1] += dy;
    }
}

double CameraController::clamp_distance_factor(double factor) const {
    if (!(factor > 0.0) || !std::isfinite(factor))
        return 1.0;
    if (!(scene_radius_ > 0.0))
        return factor;
    const double d = state_.distance;
    // A pose set from outside the limits is left where it is; zoom only never moves further out of them.
    const double lo = std::min(scene_radius_ * kMinDistanceRadii, d),
                 hi = std::max(scene_radius_ * kMaxDistanceRadii, d);
    return std::clamp(d * factor, lo, hi) / d;
}

void CameraController::zoom(double factor) {
    stop_transition();
    const double f = clamp_distance_factor(factor);
    state_.distance *= f;
    state_.ortho_height *= f;
}

void CameraController::zoom_at(double factor, double x, double y) {
    stop_transition();
    const double f = clamp_distance_factor(factor);
    const Basis b = basis(state_, up_axis_);
    const double half_h = world_per_pixel() * double(height_) * 0.5;
    const double ndc_x = 2.0 * x / double(width_) - 1.0, ndc_y = 1.0 - 2.0 * y / double(height_);
    const Vec3 anchor = add(state_.target, add(scale(b.right, ndc_x * half_h * aspect()), scale(b.up, ndc_y * half_h)));
    // Scaling eye and target about the anchor leaves it under the same pixel.
    state_.target = add(anchor, scale(sub(state_.target, anchor), f));
    state_.distance *= f;
    state_.ortho_height *= f;
}

void CameraController::roll(double angle_deg) {
    stop_transition();
    state_.roll = wrap_degrees(state_.roll + angle_deg);
}

void CameraController::go_to(const State &goal, double seconds) {
    orbit_pending_ = pan_pending_ = orbit_velocity_ = pan_velocity_ = {0, 0};
    if (!(seconds > 0.0)) {
        state_ = goal;
        state_.azimuth = wrap_degrees(state_.azimuth);
        state_.roll = wrap_degrees(state_.roll);
        in_transition_ = false;
        return;
    }
    from_ = state_;
    to_ = goal;
    // The short way round.
    to_.azimuth = from_.azimuth + wrap_degrees(goal.azimuth - from_.azimuth);
    to_.roll = from_.roll + wrap_degrees(goal.roll - from_.roll);
    transition_time_ = 0.0;
    transition_length_ = seconds;
    in_transition_ = true;
}

void CameraController::fit(const std::array<float, 6> &bounds, double margin, double seconds) {
    fit_aspect(bounds, aspect(), margin, seconds);
}

void CameraController::fit_aspect(const std::array<float, 6> &b, double aspect, double margin, double seconds) {
    if (!(margin > 0.0))
        margin = kDefaultFitMargin;
    const Vec3 center{0.5 * (double(b[0]) + b[3]), 0.5 * (double(b[1]) + b[4]), 0.5 * (double(b[2]) + b[5])};
    const double radius =
        std::max(std::hypot(double(b[3]) - b[0], double(b[4]) - b[1], double(b[5]) - b[2]) * 0.5, kMinRadius);
    // From where a transition under way is heading, so set_view() then fit() land on both.
    State goal = in_transition_ ? to_ : state_;
    goal.target = center;
    const double a = std::max(aspect, 1e-3);
    const double half = goal.fov * kRad * 0.5;
    if (ortho_) {
        goal.ortho_height = 2.0 * radius * margin / std::min(a, 1.0);
        goal.distance = goal.ortho_height / (2.0 * std::tan(half));
    } else {
        const double fit_v = radius / std::sin(half);
        const double fit_h = radius / std::sin(std::atan(std::tan(half) * a));
        goal.distance = std::max(fit_v, fit_h) * margin;
        goal.ortho_height = 2.0 * goal.distance * std::tan(half);
    }
    if (!has_scene_) {
        scene_center_ = center;
        scene_radius_ = radius;
    }
    has_pivot_ = false;
    go_to(goal, seconds);
}

bool CameraController::set_view(const std::string &name, double seconds) {
    double azimuth, elevation = 0.0;
    if (name == "top") {
        azimuth = 180.0, elevation = 90.0; // x to the right, the front view's depth axis up
    } else if (name == "bottom") {
        azimuth = 180.0, elevation = -90.0;
    } else if (name == "front") {
        azimuth = 180.0;
    } else if (name == "back") {
        azimuth = 0.0;
    } else if (name == "left") {
        azimuth = -90.0;
    } else if (name == "right") {
        azimuth = 90.0;
    } else if (name == "iso") {
        azimuth = -135.0, elevation = 35.264389682754654; // along the cube diagonal (-1, -1, 1)
    } else {
        return false;
    }
    set_orientation(azimuth, elevation, 0.0, seconds);
    return true;
}

void CameraController::set_orientation(double azimuth_deg, double elevation_deg, double roll_deg, double seconds) {
    State goal = in_transition_ ? to_ : state_;
    goal.azimuth = azimuth_deg;
    goal.elevation = std::clamp(elevation_deg, -90.0, 90.0);
    goal.roll = roll_deg;
    go_to(goal, seconds);
}

CameraController::State CameraController::state_from_pose(const CameraPose &pose) const {
    State s = state_;
    s.target = pose.target;
    s.fov = std::clamp(pose.fov_deg, 1.0, 179.0);
    const Vec3 offset = from_world(sub(pose.eye, pose.target), up_axis_);
    const double d = length(offset);
    s.roll = 0.0;
    if (d > 0.0) {
        s.distance = d;
        const Vec3 dir = scale(offset, 1.0 / d);
        s.elevation = std::asin(std::clamp(dir[2], -1.0, 1.0)) / kRad;
        const Vec3 up = from_world(pose.up, up_axis_);
        if (std::hypot(dir[0], dir[1]) > 1e-9) {
            s.azimuth = std::atan2(dir[0], dir[1]) / kRad;
            // The roll is the angle from the unrolled up to the given one, about the viewing direction.
            const Basis b = basis(s, UpAxis::Z);
            const Vec3 flat = sub(up, scale(b.forward, dot(up, b.forward)));
            if (length(flat) > 1e-12)
                s.roll = std::atan2(dot(flat, b.right), dot(flat, b.up)) / kRad;
        } else if (std::hypot(up[0], up[1]) > 1e-12) {
            // Straight down or up: the direction leaves the azimuth open, the up vector settles it.
            s.elevation = dir[2] > 0.0 ? 90.0 : -90.0;
            s.azimuth = (dir[2] > 0.0 ? std::atan2(-up[0], -up[1]) : std::atan2(up[0], up[1])) / kRad;
        }
    }
    s.ortho_height = pose.ortho_height > 0.0 ? pose.ortho_height : 2.0 * s.distance * tan_half(s.fov);
    return s;
}

void CameraController::set_pose(const CameraPose &pose, double seconds) {
    ortho_ = pose.orthographic;
    has_pivot_ = false;
    go_to(state_from_pose(pose), seconds);
}

CameraPose CameraController::pose() const {
    CameraPose p;
    p.eye = eye(state_, up_axis_);
    p.target = state_.target;
    p.up = basis(state_, up_axis_).up;
    p.fov_deg = state_.fov;
    p.orthographic = ortho_;
    p.ortho_height = state_.ortho_height;
    return p;
}

void CameraController::set_orbit_pivot(const std::array<double, 3> &point) {
    has_pivot_ = true;
    pivot_ = point;
}

void CameraController::clear_orbit_pivot() {
    has_pivot_ = false;
}

void CameraController::set_damping(double inertia) {
    inertia_ = std::clamp(inertia, 0.0, 0.999);
    if (inertia_ == 0.0)
        orbit_pending_ = pan_pending_ = orbit_velocity_ = pan_velocity_ = {0, 0};
}

bool CameraController::moving() const {
    return in_transition_ || speed(orbit_pending_) > 0.0 || speed(pan_pending_) > 0.0 || speed(orbit_velocity_) > 0.0 ||
           speed(pan_velocity_) > 0.0;
}

bool CameraController::update(double dt) {
    if (!(dt > 0.0))
        return moving();
    if (in_transition_) {
        transition_time_ += dt;
        const double t = transition_time_ / transition_length_;
        if (t >= 1.0) {
            // Exactly the goal, not the last interpolated step.
            state_ = to_;
            state_.azimuth = wrap_degrees(state_.azimuth);
            state_.roll = wrap_degrees(state_.roll);
            in_transition_ = false;
        } else {
            const double e = t * t * (3.0 - 2.0 * t); // ease in and out
            auto mix = [e](double a, double b) { return a + (b - a) * e; };
            // Distances go geometrically: a zoom over several magnitudes moves at an even pace on screen.
            auto mix_log = [e](double a, double b) { return a > 0.0 && b > 0.0 ? a * std::pow(b / a, e) : b; };
            for (int k = 0; k < 3; ++k)
                state_.target[k] = mix(from_.target[k], to_.target[k]);
            state_.azimuth = mix(from_.azimuth, to_.azimuth);
            state_.elevation = mix(from_.elevation, to_.elevation);
            state_.roll = mix(from_.roll, to_.roll);
            state_.fov = mix(from_.fov, to_.fov);
            state_.distance = mix_log(from_.distance, to_.distance);
            state_.ortho_height = mix_log(from_.ortho_height, to_.ortho_height);
        }
    }
    if (inertia_ > 0.0) {
        const double keep = std::pow(inertia_, dt * 60.0);
        auto glide = [&](std::array<double, 2> &pending, std::array<double, 2> &velocity, bool is_orbit) {
            if (speed(pending) > 0.0) {
                // Still being dragged (already applied): remember how fast, to carry on from. A frame that
                // comes right after the last one must not turn a small drag into a fling.
                const double span = std::max(dt, kShortestFrame);
                velocity = {pending[0] / span, pending[1] / span};
                pending = {0, 0};
                return;
            }
            velocity = {velocity[0] * keep, velocity[1] * keep};
            if (speed(velocity) < kRestSpeed) {
                velocity = {0, 0};
                return;
            }
            if (is_orbit)
                apply_orbit(velocity[0] * dt, velocity[1] * dt);
            else
                apply_pan(velocity[0] * dt, velocity[1] * dt);
        };
        glide(orbit_pending_, orbit_velocity_, true);
        glide(pan_pending_, pan_velocity_, false);
    }
    return moving();
}

} // namespace adacpp::render
