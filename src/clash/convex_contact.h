// Convex-contact clash detection -- which members touch or overlap, and how.
//
// WHAT IT ANSWERS. For a set of CONVEX meshes (a member's convex hull), every pair that touches or
// overlaps within a security margin, with the contact a connection is sized from: the nearest point on
// each shape, the contact normal, the signed penetration depth and the contact-patch area. The geometry
// is coal's (github.com/coal-library/coal): GJK/EPA on the two convex polytopes and coal's
// contact-patch computation, compiled from coal's own sources (cmake/deps_coal.cmake) so the numbers
// are coal's on every target -- native, pyodide and the standalone browser module.
//
// WHY IN C++. The same check runs on a worker and in a browser, and the two have to report the same
// contacts. Compiled once and called from both, that holds by construction; a browser check also
// needs no Python and no coal wheel. The reference it is held to is coal's own Python bindings plus a
// numpy/scipy patch-area measure (tools/gen_clash_cases.py), which the parity fixture records.
//
// This header is coal-free on purpose: coal, Eigen and coal's generated headers stay inside
// convex_contact.cpp, so a consumer (the nanobind module, the embind module) never sees them.

#ifndef ADACPP_CLASH_CONVEX_CONTACT_H
#define ADACPP_CLASH_CONVEX_CONTACT_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace adacpp::clash {

//: The defaults adapy's convex-contact checker advertises.
constexpr double DEFAULT_SECURITY_MARGIN = 0.01;
constexpr double DEFAULT_MIN_CONTACT_AREA = 1e-6;
constexpr double DEFAULT_LINE_CONTACT_THICKNESS = 1e-3;

struct ConvexContactOptions {
    //! Shapes closer than this count as touching (coal's `CollisionRequest::security_margin`).
    double security_margin = DEFAULT_SECURITY_MARGIN;
    //! Contacts with a smaller patch area are dropped (a corner grazing a face).
    double min_contact_area = DEFAULT_MIN_CONTACT_AREA;
    //! Width given to a line (edge-on-face) contact, so a real touching edge has a nonzero area.
    double line_contact_thickness = DEFAULT_LINE_CONTACT_THICKNESS;
    //! Take each mesh's convex hull first (from its vertices alone; its triangles are ignored). Off
    //! only for a caller whose meshes are convex hulls already.
    bool hull = true;
};

/** One mesh, borrowed: `n_vertices` xyz triples and `n_triangles` index triples into them.
 *  coal needs it CONVEX: its support function walks the triangle adjacency, and on a non-convex
 *  soup it silently misses real overlaps. With `ConvexContactOptions::hull` (the default) any mesh
 *  will do -- even bare vertices -- since the hull is taken here. */
struct MeshView {
    const double *vertices = nullptr;
    std::size_t n_vertices = 0;
    const std::uint32_t *triangles = nullptr;
    std::size_t n_triangles = 0;
};

/** One contact patch: the polygon where the two shapes meet, as coal computes it. A face-on-face
 *  contact is a polygon, an edge-on-face contact two points, a corner one. */
struct ContactPatch {
    std::array<double, 3> normal{0, 0, 0}; //!< the patch plane's normal, coal's, from mesh i towards mesh j
    //! The polygon's vertices on mesh i's surface, in coal's order (convex, around `normal`).
    std::vector<std::array<double, 3>> points1;
    //! The same vertices on mesh j's surface: `points1` moved by the penetration depth along `normal`.
    std::vector<std::array<double, 3>> points2;
};

struct ConvexContact {
    std::size_t i = 0;                     //!< first mesh (i < j)
    std::size_t j = 0;                     //!< second mesh
    std::array<double, 3> near1{0, 0, 0};  //!< nearest point on mesh i
    std::array<double, 3> near2{0, 0, 0};  //!< nearest point on mesh j
    std::array<double, 3> normal{0, 0, 0}; //!< coal's contact normal, from mesh i towards mesh j
    //! coal's sign: negative is overlap depth, positive a gap that is still inside the margin.
    double penetration_depth = 0.0;
    double contact_area = 0.0;         //!< m^2, summed over `patches` by `patch_area`
    std::vector<ContactPatch> patches; //!< usually one
};

/** A candidate pair coal could not evaluate. Reported, not thrown: one bad pair must not cost the
 *  rest of the check, and the caller decides whether to log it. */
struct ConvexContactFailure {
    std::size_t i = 0;
    std::size_t j = 0;
    std::string reason;
};

struct ConvexContactResult {
    std::vector<ConvexContact> contacts; //!< in (i, j) order, i < j
    std::vector<ConvexContactFailure> failures;
    std::size_t candidate_pairs = 0; //!< pairs the broad phase passed to coal
    std::size_t below_min_area = 0;  //!< colliding pairs dropped by `min_contact_area`
    //! Meshes that had vertices but no solid hull (all on one plane or line, or too few): they take
    //! part in no pair. Only filled when `hull` is on.
    std::vector<std::size_t> degenerate;
};

/** An owned convex mesh: xyz triples and outward-wound index triples into them. */
struct ConvexMesh {
    std::vector<double> vertices;
    std::vector<std::uint32_t> triangles;
};

/** The convex hull of `n_points` xyz triples (Manifold's QuickHull), its vertices in the order they
 *  appear in the input. Empty when the points span no volume -- fewer than four, or all on one
 *  plane. */
ConvexMesh convex_hull(const double *points, std::size_t n_points);

/** Index pairs (i, j), i < j, whose boxes come within `security_margin` on every axis, sorted.
 *
 *  Sweep-and-prune on x, with y and z tested per box. Conservative -- the gap
 *  between two convex shapes is at least their largest per-axis box gap, so a pair dropped here is
 *  farther apart than the margin and coal would not report it. A box with a non-finite coordinate
 *  (an empty mesh) takes part in no pair. A negative margin counts as zero. */
std::vector<std::pair<std::size_t, std::size_t>> candidate_pairs(const std::vector<std::array<double, 3>> &lo,
                                                                 const std::vector<std::array<double, 3>> &hi,
                                                                 double security_margin);

/** The area of one contact patch: `points` (all on shape 1) lying in the plane with `normal`.
 *
 *  2 points are a line contact: length * `line_contact_thickness`. 3+ points: the area of their
 *  convex hull in the patch plane -- except where qhull (scipy's ConvexHull, which the reference asks
 *  about near-flat patches) would refuse the points as flat; then the largest in-plane extent *
 *  `line_contact_thickness`.
 *  Fewer than 2 points have no area. */
double patch_area(const std::vector<std::array<double, 3>> &points, const std::array<double, 3> &normal,
                  double line_contact_thickness = DEFAULT_LINE_CONTACT_THICKNESS);

/** Every touching / overlapping pair among `meshes`. Throws std::invalid_argument for a mesh whose
 *  triangles index past its vertices. */
ConvexContactResult find_convex_contacts(const std::vector<MeshView> &meshes, const ConvexContactOptions &options = {});

} // namespace adacpp::clash

#endif // ADACPP_CLASH_CONVEX_CONTACT_H
