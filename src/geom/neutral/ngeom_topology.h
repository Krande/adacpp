// NGEOM neutral topology — edges, loops, face-surfaces (spec §6). The decoded form the
// tessellator consumes. A loop knows how to discretize itself into an ordered 3D polyline.
#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "ngeom_curves.h"
#include "ngeom_math.h"
#include "ngeom_surfaces.h"

namespace adacpp::ngeom {

// One oriented edge of a loop. Endpoints are already in traversal order (orientation
// resolved at decode). `geometry` is the basis curve; null => treat as a straight segment.
struct OrientedEdgeN {
    Vec3 start, end;
    std::shared_ptr<Curve> geometry;  // basis curve (may be null)
    bool has_params = false;
    double t_start = 0, t_end = 0;

    // Ordered points start->end along the edge (endpoints included). Curved edges use the
    // basis curve's discretize over [t_start,t_end]; endpoint-snap guards the direction so
    // sense bookkeeping can't flip a loop.
    std::vector<Vec3> discretize(double deflection, double max_angle) const {
        if (!geometry || !has_params) return {start, end};
        std::vector<Vec3> pts = geometry->discretize(t_start, t_end, deflection, max_angle);
        if (pts.size() < 2) return {start, end};
        // orient to go from `start` to `end`
        if ((pts.front() - start).norm() > (pts.back() - start).norm())
            std::reverse(pts.begin(), pts.end());
        return pts;
    }
};

// A boundary loop: either an edge loop (curved/line edges) or a literal polygon.
struct LoopN {
    bool is_poly = false;
    std::vector<OrientedEdgeN> edges;  // when !is_poly
    std::vector<Vec3> polygon;         // when is_poly

    // Ordered closed polyline of the whole loop (shared vertices de-duplicated).
    std::vector<Vec3> discretize(double deflection, double max_angle) const {
        if (is_poly) return polygon;
        std::vector<Vec3> out;
        for (const auto &e : edges) {
            std::vector<Vec3> ep = e.discretize(deflection, max_angle);
            for (const Vec3 &p : ep) {
                if (out.empty() || (p - out.back()).norm() > 1e-9) out.push_back(p);
            }
        }
        // drop a closing duplicate of the first vertex
        if (out.size() > 1 && (out.front() - out.back()).norm() <= 1e-9) out.pop_back();
        return out;
    }
};

struct FaceBoundN {
    std::shared_ptr<LoopN> loop;
    bool orientation = true;  // false => reverse the loop's winding relative to the face
};

struct FaceSurfaceN {
    std::shared_ptr<Surface> surface;
    bool same_sense = true;  // false => face normal is the surface normal flipped
    std::vector<FaceBoundN> bounds;  // bounds[0] outer, rest holes
};

struct ConnectedFaceSetN {
    std::vector<std::shared_ptr<FaceSurfaceN>> faces;
};

// One top-level streamed Geometry instance + its stable id (for Mesh grouping).
struct NgeomRoot {
    std::string id;
    std::vector<std::shared_ptr<FaceSurfaceN>> faces;  // flattened (a CFS expands to its faces)
};

struct NgeomDoc {
    std::vector<NgeomRoot> roots;
};

}  // namespace adacpp::ngeom
