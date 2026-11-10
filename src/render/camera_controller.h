// An orbit camera for the renderer: a target, an orientation about it, a distance, and the view and
// projection matrices that follow. No GPU and no input devices here: a host maps its pointer, wheel, touch
// or key events to the semantic operations (orbit, pan, zoom, ...), so every host moves the camera the same
// way. Renderer owns one (Renderer::camera()) and reads its matrices when set_camera_controlled(true).
//
// Conventions
//   angles      degrees.
//   up axis     +Z (the default, as adapy models are) or +Y. Azimuth turns about it, elevation is the angle
//               above the ground plane. With z up the camera sits at
//                   target + distance * (cos(el) sin(az), cos(el) cos(az), sin(el))
//               so azimuth 180 looks along +Y (the front view) and azimuth 90 looks along -X. With y up the
//               same frame is mapped (x, y, z) -> (x, z, -y): the front view looks along -Z.
//   pixels      x right, y down, origin top-left, in the units of set_viewport() (the drawing buffer).
//   matrices    column-major; the projection in OpenGL clip space (z in [-1, 1]), which is what
//               Renderer::set_camera() takes.
#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace adacpp::render {

enum class UpAxis { Z, Y };

struct CameraPose {
    std::array<double, 3> eye{0, 0, 1};
    std::array<double, 3> target{0, 0, 0};
    std::array<double, 3> up{0, 1, 0}; // the camera's up on screen (carries the roll)
    double fov_deg = 45.0;             // vertical
    bool orthographic = false;
    double ortho_height = 1.0; // world height of the orthographic view; <= 0 in set_pose() = derive it
};

class CameraController {
public:
    static constexpr double kDefaultFitMargin = 1.15;

    // The size the pixel arguments are measured in, and the aspect fit() and zoom_at() frame for.
    void set_viewport(uint32_t width, uint32_t height);
    double aspect() const;

    void set_up_axis(UpAxis axis);
    UpAxis up_axis() const {
        return up_axis_;
    }
    // Vertical field of view of the perspective projection. The distance is kept.
    void set_fov(double fov_deg);
    // Switching keeps the picture at the target's depth: to orthographic takes the height the perspective
    // view shows there, back to perspective takes the distance that shows the orthographic height.
    void set_orthographic(bool orthographic);
    bool orthographic() const {
        return ortho_;
    }
    // What near and far wrap (min xyz, max xyz), and what limits zoom. Until it is set, fit() sets it.
    void set_scene_bounds(const std::array<float, 6> &bounds);

    // Drag by (dx, dy) pixels. A drag over the viewport's height turns 180 degrees; dragging right turns
    // the near side of the model right, dragging down tips its top towards the viewer. Elevation stops at
    // +-90. Turns about the orbit pivot when one is set, else about the target.
    void orbit(double dx, double dy);
    // Drag by (dx, dy) pixels: what is at the target's depth follows the pointer.
    void pan(double dx, double dy);
    // Scale the distance (and the orthographic height): below 1 moves in, above 1 moves out.
    void zoom(double factor);
    // The same, towards pixel (x, y): the world point under it on the plane through the target facing the
    // camera stays under it.
    void zoom_at(double factor, double x, double y);
    // Turn the camera about its viewing direction; positive turns the picture counter-clockwise.
    void roll(double angle_deg);

    // Look at the centre of `bounds` (min xyz, max xyz) from the current direction, from as far as makes
    // their bounding sphere fill the view (times `margin`), for the viewport's aspect or the one given.
    // `seconds` above 0 moves there over that time (see update()). Forgets the orbit pivot.
    void fit(const std::array<float, 6> &bounds, double margin = kDefaultFitMargin, double seconds = 0.0);
    void fit_aspect(const std::array<float, 6> &bounds, double aspect, double margin = kDefaultFitMargin,
                    double seconds = 0.0);
    // "top", "bottom", "front", "back", "left", "right" or "iso": turn about the target to that side of
    // the model, roll cleared. Returns false for any other name.
    bool set_view(const std::string &name, double seconds = 0.0);
    void set_orientation(double azimuth_deg, double elevation_deg, double roll_deg = 0.0, double seconds = 0.0);
    // The projection kind switches at once; everything else moves over `seconds`. Forgets the orbit pivot.
    void set_pose(const CameraPose &pose, double seconds = 0.0);
    CameraPose pose() const;

    // A world point to orbit about instead of the target (what the pointer went down on): it keeps its
    // place on screen while the camera turns. fit() and set_pose() forget it.
    void set_orbit_pivot(const std::array<double, 3> &point);
    void clear_orbit_pivot();

    // Inertia of orbit and pan: the share of their speed kept per 1/60 s once the input stops (0 = off,
    // the default; 0.9 glides for about a second). Needs update() every frame.
    void set_damping(double inertia);
    // Advance a transition and the damping by `dt` seconds. True while the camera is still moving: the
    // host keeps drawing frames. Any orbit, pan, zoom or roll ends a transition where it is.
    bool update(double dt);
    bool moving() const;

    std::array<float, 16> view() const;
    std::array<float, 16> projection(double aspect) const;

    std::array<double, 3> target() const {
        return state_.target;
    }
    double azimuth() const {
        return state_.azimuth;
    }
    double elevation() const {
        return state_.elevation;
    }
    double roll_angle() const {
        return state_.roll;
    }
    double distance() const {
        return state_.distance;
    }
    double fov() const {
        return state_.fov;
    }
    double ortho_height() const {
        return state_.ortho_height;
    }

private:
    using Vec3 = std::array<double, 3>;
    struct State {
        Vec3 target{0, 0, 0};
        double azimuth = -135.0, elevation = 30.0, roll = 0.0;
        double distance = 1.0;
        double fov = 45.0;
        double ortho_height = 0.8284271247461901; // 2 * distance * tan(fov / 2)
    };
    struct Basis {
        Vec3 right, up, forward;
    };

    static Basis basis(const State &s, UpAxis axis);
    static Vec3 eye(const State &s, UpAxis axis);
    State state_from_pose(const CameraPose &pose) const;
    void go_to(const State &goal, double seconds);
    void stop_transition();
    void apply_orbit(double dx, double dy);
    void apply_pan(double dx, double dy);
    double clamp_distance_factor(double factor) const;
    double world_per_pixel() const;

    State state_;
    UpAxis up_axis_ = UpAxis::Z;
    bool ortho_ = false;
    uint32_t width_ = 1, height_ = 1;

    bool has_scene_ = false;
    Vec3 scene_center_{0, 0, 0};
    double scene_radius_ = 0.0; // 0: no bounds yet

    bool has_pivot_ = false;
    Vec3 pivot_{0, 0, 0};

    bool in_transition_ = false;
    State from_, to_;
    double transition_time_ = 0.0, transition_length_ = 0.0;

    double inertia_ = 0.0;
    // Pixels dragged since the last update(), and the drag speed (pixels / s) the damping carries on with.
    std::array<double, 2> orbit_pending_{0, 0}, pan_pending_{0, 0};
    std::array<double, 2> orbit_velocity_{0, 0}, pan_velocity_{0, 0};
};

} // namespace adacpp::render
