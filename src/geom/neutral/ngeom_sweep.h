// Shared swept-disk builder: a disk/annulus swept along a directrix polyline -> ng::SweepN with
// rotation-minimising (parallel-transport) per-station frames, so the circular profile doesn't twist.
// Used by BOTH the IFC reader (IfcSweptDiskSolid) and the STEP reader (SWEPT_DISK_SOLID) so the two
// resolve identical geometry — and so an IFC->STEP->IFC round-trip of a rebar recovers the same sweep.
#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "ngeom_math.h"     // Vec3, Frame
#include "ngeom_surfaces.h" // PlaneSurface
#include "ngeom_topology.h" // FaceSurfaceN, LoopN, FaceBoundN, SweepN

namespace adacpp::ngeom {

// A regular n-gon of radius r in the z=0 plane. n<=0 => a segment count from a ~17deg chord angle,
// consistent with the tessellator's angular default (so a disk isn't finer than every other surface).
inline std::vector<Vec3> sweep_circle_poly(double r, int n = 0) {
    if (n <= 0)
        n = std::max(12, std::min(64, (int) std::ceil(TWO_PI / 0.30)));
    std::vector<Vec3> p;
    p.reserve(n);
    for (int i = 0; i < n; ++i) {
        double a = TWO_PI * i / n;
        p.push_back({r * std::cos(a), r * std::sin(a), 0});
    }
    return p;
}

// A planar (z=0) profile face from an outer polygon + optional hole polygons (holes reversed).
inline std::shared_ptr<FaceSurfaceN> sweep_make_profile(std::vector<Vec3> outer,
                                                        std::vector<std::vector<Vec3>> holes = {}) {
    if (outer.size() < 3)
        return nullptr;
    auto prof = std::make_shared<FaceSurfaceN>();
    prof->surface = std::make_shared<PlaneSurface>(Frame{});
    prof->same_sense = true;
    auto add = [&](std::vector<Vec3> poly) {
        auto lp = std::make_shared<LoopN>();
        lp->is_poly = true;
        lp->polygon = std::move(poly);
        FaceBoundN fb;
        fb.loop = lp;
        fb.orientation = true;
        prof->bounds.push_back(fb);
    };
    add(std::move(outer));
    for (auto &h : holes)
        if (h.size() >= 3) {
            std::reverse(h.begin(), h.end());
            add(std::move(h));
        }
    return prof;
}

// Disk/annulus of `radius` (+ optional `inner`) swept along `pts` -> SweepN with parallel-transport
// (rotation-minimising) frames. Mirrors adapy's _sweep_frames. Null on a degenerate directrix.
inline std::shared_ptr<SweepN> make_swept_disk(const std::vector<Vec3> &pts, double radius, double inner) {
    if (pts.size() < 2 || radius <= 0)
        return nullptr;
    int n = (int) pts.size();
    std::vector<Vec3> tan(n);
    for (int i = 0; i < n; ++i) {
        Vec3 tv = (i == 0) ? pts[1] - pts[0] : (i == n - 1) ? pts[n - 1] - pts[n - 2] : pts[i + 1] - pts[i - 1];
        tan[i] = tv.norm() > 1e-12 ? tv.normalized() : Vec3{0, 0, 1};
    }
    auto sw = std::make_shared<SweepN>();
    sw->frame = Frame{}; // origin/dir_x/dir_y are already world
    sw->origin.resize(n);
    sw->dir_x.resize(n);
    sw->dir_y.resize(n);
    Vec3 up0 = std::abs(tan[0].z) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0};
    Vec3 dx = (up0 - tan[0] * tan[0].dot(up0)).normalized();
    for (int i = 0; i < n; ++i) {
        if (i > 0) { // parallel-transport dx across the tangent turn (Rodrigues)
            Vec3 axis = tan[i - 1].cross(tan[i]);
            double s = axis.norm(), cth = tan[i - 1].dot(tan[i]);
            if (s > 1e-9) {
                axis = axis * (1.0 / s);
                double ang = std::atan2(s, cth);
                dx = dx * std::cos(ang) + axis.cross(dx) * std::sin(ang) + axis * (axis.dot(dx) * (1 - std::cos(ang)));
            }
        }
        Vec3 ortho = dx - tan[i] * tan[i].dot(dx);
        if (ortho.norm() < 1e-9) { // dx drifted parallel to the tangent -> reseed perpendicular
            Vec3 up = std::abs(tan[i].z) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0};
            ortho = up - tan[i] * tan[i].dot(up);
        }
        dx = ortho.normalized();
        sw->origin[i] = pts[i];
        sw->dir_x[i] = dx;
        sw->dir_y[i] = tan[i].cross(dx);
    }
    std::vector<std::vector<Vec3>> holes;
    if (inner > 1e-9)
        holes.push_back(sweep_circle_poly(inner));
    sw->profile = sweep_make_profile(sweep_circle_poly(radius), holes);
    return sw->profile ? sw : nullptr;
}

} // namespace adacpp::ngeom
