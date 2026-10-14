// Convex-contact clash detection on coal. See convex_contact.h for what this answers.
//
// Every rule here has a Python twin in tools/gen_clash_cases.py -- one fresh coal request per
// candidate pair, the patch area from the patch points -- and tests/fixtures/clash holds this file
// to what that reference reports.

#include "convex_contact.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>

#include <coal/collision.h>
#include <coal/contact_patch.h>
#include <coal/shape/convex.h>

namespace adacpp::clash {

namespace {

using Vec2 = std::array<double, 2>;

//: Slack added to the broad-phase margin, plus a relative term for large coordinates: rounding must
//: never be what tells a pair coal would call colliding that it cannot touch.
constexpr double BROAD_PHASE_ABS_TOL = 1e-6;
constexpr double BROAD_PHASE_REL_TOL = 1e-9;

//: A patch whose hull area is at least this fraction of its squared extent is unambiguously 2D; below
//: it, the reference asks qhull, which may call the points flat.
constexpr double PLANAR_HULL_MIN_REL_AREA = 1e-3;

std::array<double, 3> cross3(const std::array<double, 3> &a, const std::array<double, 3> &b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

double dot3(const std::array<double, 3> &a, const std::array<double, 3> &b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

std::array<double, 3> normalized(const std::array<double, 3> &a) {
    const double n = std::sqrt(dot3(a, a));
    return {a[0] / n, a[1] / n, a[2] / n};
}

double cross2(const Vec2 &o, const Vec2 &a, const Vec2 &b) {
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
}

// Monotone-chain hull of `pts` (sorted, unique), counter-clockwise, collinear points dropped.
std::vector<Vec2> hull2(const std::vector<Vec2> &pts) {
    std::vector<Vec2> lower, upper;
    for (const Vec2 &p : pts) {
        while (lower.size() >= 2 && cross2(lower[lower.size() - 2], lower.back(), p) <= 0)
            lower.pop_back();
        lower.push_back(p);
    }
    for (auto it = pts.rbegin(); it != pts.rend(); ++it) {
        while (upper.size() >= 2 && cross2(upper[upper.size() - 2], upper.back(), *it) <= 0)
            upper.pop_back();
        upper.push_back(*it);
    }
    lower.pop_back();
    upper.pop_back();
    lower.insert(lower.end(), upper.begin(), upper.end());
    return lower;
}

double shoelace(const std::vector<Vec2> &hull) {
    double area = 0.0;
    for (std::size_t k = 0; k < hull.size(); ++k) {
        const Vec2 &a = hull[k];
        const Vec2 &b = hull[(k + 1) % hull.size()];
        area += a[0] * b[1] - b[0] * a[1];
    }
    return std::abs(area) / 2.0;
}

// Whether qhull (scipy's ConvexHull, no options) would refuse `proj` as a flat input -- QH6154,
// "initial simplex is flat", or too few points. Its 2-d test: the initial simplex is spanned by the
// min-x and max-x points (first occurrence) and the point that maximises the determinant with them,
// and it is flat when |det| < 10 * NEARzero[1] = 800 * MAXsumcoord * DBL_EPSILON, MAXsumcoord being
// the sum over the axes of max(max, -min). (geom2_r.c: qh_maxmin, qh_determinant.) The coordinates are
// the unshifted projected ones, as the reference hands them to qhull.
bool qhull_calls_flat(const std::vector<Vec2> &proj) {
    if (proj.size() < 3)
        return true;
    std::size_t imin = 0, imax = 0;
    double lo[2] = {proj[0][0], proj[0][1]}, hi[2] = {proj[0][0], proj[0][1]};
    for (std::size_t k = 1; k < proj.size(); ++k) {
        if (proj[imax][0] < proj[k][0])
            imax = k;
        else if (proj[imin][0] > proj[k][0])
            imin = k;
        for (int d = 0; d < 2; ++d) {
            lo[d] = std::min(lo[d], proj[k][d]);
            hi[d] = std::max(hi[d], proj[k][d]);
        }
    }
    const double max_sum_coord = std::max(hi[0], -lo[0]) + std::max(hi[1], -lo[1]);
    double det = 0.0;
    for (const Vec2 &p : proj)
        det = std::max(det, std::abs(cross2(p, proj[imin], proj[imax])));
    return det < 800.0 * max_sum_coord * DBL_EPSILON;
}

std::shared_ptr<coal::Convex<coal::Triangle>> to_coal_convex(const MeshView &m, std::size_t idx) {
    auto points = std::make_shared<std::vector<coal::Vec3s>>(m.n_vertices);
    for (std::size_t v = 0; v < m.n_vertices; ++v)
        (*points)[v] = coal::Vec3s(m.vertices[3 * v], m.vertices[3 * v + 1], m.vertices[3 * v + 2]);
    auto tris = std::make_shared<std::vector<coal::Triangle>>(m.n_triangles);
    for (std::size_t t = 0; t < m.n_triangles; ++t) {
        const std::uint32_t *f = m.triangles + 3 * t;
        if (f[0] >= m.n_vertices || f[1] >= m.n_vertices || f[2] >= m.n_vertices)
            throw std::invalid_argument("mesh " + std::to_string(idx) + ": triangle " + std::to_string(t) +
                                        " indexes past its " + std::to_string(m.n_vertices) + " vertices");
        (*tris)[t] = coal::Triangle(f[0], f[1], f[2]);
    }
    return std::make_shared<coal::Convex<coal::Triangle>>(points, static_cast<unsigned int>(m.n_vertices), tris,
                                                          static_cast<unsigned int>(m.n_triangles));
}

std::array<double, 3> arr3(const coal::Vec3s &v) {
    return {v[0], v[1], v[2]};
}

} // namespace

std::vector<std::pair<std::size_t, std::size_t>> candidate_pairs(const std::vector<std::array<double, 3>> &lo,
                                                                 const std::vector<std::array<double, 3>> &hi,
                                                                 double security_margin) {
    std::vector<std::pair<std::size_t, std::size_t>> pairs;
    const std::size_t n = std::min(lo.size(), hi.size());
    if (n < 2)
        return pairs;

    auto finite = [&](std::size_t i) {
        for (int k = 0; k < 3; ++k)
            if (!std::isfinite(lo[i][k]) || !std::isfinite(hi[i][k]))
                return false;
        return true;
    };
    std::vector<std::size_t> order;
    order.reserve(n);
    double scale = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        if (!finite(i))
            continue;
        order.push_back(i);
        for (int k = 0; k < 3; ++k)
            scale = std::max({scale, std::abs(lo[i][k]), std::abs(hi[i][k])});
    }
    const double pad = std::max(security_margin, 0.0) + BROAD_PHASE_ABS_TOL + BROAD_PHASE_REL_TOL * scale;

    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return lo[a][0] < lo[b][0]; });
    std::vector<double> sorted_min_x(order.size());
    for (std::size_t k = 0; k < order.size(); ++k)
        sorted_min_x[k] = lo[order[k]][0];

    for (std::size_t k = 0; k < order.size(); ++k) {
        const std::size_t i = order[k];
        // x: lo_j >= lo_i by the sort, and lo_j <= hi_i + pad by the search. y and z both ways.
        const std::size_t end = static_cast<std::size_t>(
            std::upper_bound(sorted_min_x.begin(), sorted_min_x.end(), hi[i][0] + pad) - sorted_min_x.begin());
        for (std::size_t m = k + 1; m < end; ++m) {
            const std::size_t j = order[m];
            bool ok = true;
            for (int a = 1; a < 3 && ok; ++a)
                ok = lo[j][a] <= hi[i][a] + pad && lo[i][a] <= hi[j][a] + pad;
            if (ok)
                pairs.emplace_back(std::min(i, j), std::max(i, j));
        }
    }
    std::sort(pairs.begin(), pairs.end());
    return pairs;
}

double patch_area(const std::vector<std::array<double, 3>> &points, const std::array<double, 3> &normal_in,
                  double line_contact_thickness) {
    const std::size_t n = points.size();
    if (n < 2)
        return 0.0;
    if (n == 2) {
        const std::array<double, 3> d = {points[1][0] - points[0][0], points[1][1] - points[0][1],
                                         points[1][2] - points[0][2]};
        return std::sqrt(dot3(d, d)) * line_contact_thickness;
    }

    // An in-plane basis: any axis not near the normal, crossed with it.
    const std::array<double, 3> normal = normalized(normal_in);
    std::array<double, 3> arb = {1.0, 0.0, 0.0};
    if (std::abs(dot3(arb, normal)) > 0.9)
        arb = {0.0, 1.0, 0.0};
    const std::array<double, 3> u1 = normalized(cross3(normal, arb));
    const std::array<double, 3> u2 = cross3(normal, u1);

    std::vector<Vec2> proj(n);
    for (std::size_t k = 0; k < n; ++k)
        proj[k] = {dot3(points[k], u1), dot3(points[k], u2)};

    // Relative to the patch's own corner: the projected coordinates are world-sized while a patch is
    // centimetres, and the shoelace sum would cancel the difference away.
    Vec2 corner = proj[0];
    for (const Vec2 &p : proj)
        corner = {std::min(corner[0], p[0]), std::min(corner[1], p[1])};
    std::vector<Vec2> pts(n);
    for (std::size_t k = 0; k < n; ++k)
        pts[k] = {proj[k][0] - corner[0], proj[k][1] - corner[1]};
    std::sort(pts.begin(), pts.end());
    pts.erase(std::unique(pts.begin(), pts.end()), pts.end());

    double extent = 0.0;
    double area = 0.0;
    bool have_hull = false;
    if (pts.size() >= 3) {
        const std::vector<Vec2> hull = hull2(pts);
        if (hull.size() >= 3) {
            have_hull = true;
            area = shoelace(hull);
            double lo0 = pts[0][0], hi0 = pts[0][0], lo1 = pts[0][1], hi1 = pts[0][1];
            for (const Vec2 &p : pts) {
                lo0 = std::min(lo0, p[0]);
                hi0 = std::max(hi0, p[0]);
                lo1 = std::min(lo1, p[1]);
                hi1 = std::max(hi1, p[1]);
            }
            extent = std::max(hi0 - lo0, hi1 - lo1);
            if (area > PLANAR_HULL_MIN_REL_AREA * extent * extent)
                return area;
        }
    }

    // Flat or nearly flat: the reference asks qhull. Where qhull accepts the points the answer is their
    // hull area; where it calls them flat, a line contact as wide as the patch is long.
    if (have_hull && !qhull_calls_flat(proj))
        return area;
    double w0 = 0.0, w1 = 0.0;
    {
        double lo0 = proj[0][0], hi0 = proj[0][0], lo1 = proj[0][1], hi1 = proj[0][1];
        for (const Vec2 &p : proj) {
            lo0 = std::min(lo0, p[0]);
            hi0 = std::max(hi0, p[0]);
            lo1 = std::min(lo1, p[1]);
            hi1 = std::max(hi1, p[1]);
        }
        w0 = hi0 - lo0;
        w1 = hi1 - lo1;
    }
    return std::max(w0, w1) * line_contact_thickness;
}

ConvexContactResult find_convex_contacts(const std::vector<MeshView> &meshes, const ConvexContactOptions &options) {
    ConvexContactResult result;
    const std::size_t n = meshes.size();

    std::vector<std::shared_ptr<coal::Convex<coal::Triangle>>> shapes(n);
    std::vector<std::array<double, 3>> lo(n), hi(n);
    constexpr double inf = std::numeric_limits<double>::infinity();
    for (std::size_t idx = 0; idx < n; ++idx) {
        const MeshView &m = meshes[idx];
        if (m.n_vertices == 0) {
            // No geometry: nothing can touch it, and an inverted box overlaps nothing.
            lo[idx] = {inf, inf, inf};
            hi[idx] = {-inf, -inf, -inf};
            continue;
        }
        shapes[idx] = to_coal_convex(m, idx);
        lo[idx] = {m.vertices[0], m.vertices[1], m.vertices[2]};
        hi[idx] = lo[idx];
        for (std::size_t v = 1; v < m.n_vertices; ++v)
            for (int k = 0; k < 3; ++k) {
                lo[idx][k] = std::min(lo[idx][k], m.vertices[3 * v + k]);
                hi[idx][k] = std::max(hi[idx][k], m.vertices[3 * v + k]);
            }
    }

    // BROAD PHASE. coal reports a collision only when two convex shapes are within the margin of each
    // other, which needs their boxes to be within that margin on every axis. Only those pairs reach
    // coal, so the contacts are exactly the all-pairs ones, in the same (i < j) order.
    const auto pairs = candidate_pairs(lo, hi, options.security_margin);
    result.candidate_pairs = pairs.size();

    // The shapes sit where their vertices say: both placements are the identity.
    const coal::Transform3s T1, T2;
    for (const auto &[i, j] : pairs) {
        const coal::Convex<coal::Triangle> &a = *shapes[i];
        const coal::Convex<coal::Triangle> &b = *shapes[j];
        coal::CollisionRequest req;
        req.security_margin = options.security_margin;
        coal::CollisionResult res;
        try {
            coal::collide(&a, T1, &b, T2, req, res);
        } catch (const std::exception &e) {
            result.failures.push_back({i, j, std::string("collide: ") + e.what()});
            continue;
        }
        if (!res.isCollision())
            continue;

        double area = 0.0;
        try {
            coal::ContactPatchRequest patch_req(req);
            coal::ContactPatchResult patch_res;
            coal::computeContactPatch(&a, T1, &b, T2, res, patch_req, patch_res);
            for (std::size_t p = 0; p < patch_res.numContactPatches(); ++p) {
                const coal::ContactPatch &patch = patch_res.getContactPatch(p);
                std::vector<std::array<double, 3>> points(patch.size());
                for (std::size_t k = 0; k < patch.size(); ++k)
                    points[k] = arr3(patch.getPointShape1(k));
                area += patch_area(points, arr3(patch.getNormal()), options.line_contact_thickness);
            }
        } catch (const std::exception &e) {
            result.failures.push_back({i, j, std::string("contact patch: ") + e.what()});
            continue;
        }
        if (area < options.min_contact_area) {
            ++result.below_min_area;
            continue;
        }

        const coal::Contact &c = res.getContact(0);
        ConvexContact out;
        out.i = i;
        out.j = j;
        out.near1 = arr3(c.nearest_points[0]);
        out.near2 = arr3(c.nearest_points[1]);
        out.normal = arr3(c.normal);
        out.penetration_depth = c.penetration_depth;
        out.contact_area = area;
        result.contacts.push_back(out);
    }
    return result;
}

} // namespace adacpp::clash
