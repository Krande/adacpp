#ifndef ADACPP_CAD_FACETED_SOLID_H
#define ADACPP_CAD_FACETED_SOLID_H

// A solid from a closed set of planar polygons (a faceted B-rep), built entirely in C++.
//
// The general route -- one Python call per face, then BOPAlgo_MakerVolume over the face soup --
// treats the faces as an arbitrary arrangement to partition space with: it intersects every face
// against every other before it can tell which cells they bound. For a shell that is already
// closed that work finds nothing, and it is most of the cost.
//
// So the faces are first taken as the shell they claim to be, and the topology is built directly,
// linear in the number of faces:
//
//   1. Direct: the corners are welded within `tolerance` (a hash grid) into one TopoDS_Vertex
//      each, every undirected edge is made ONCE and shared, oriented, by the two faces that meet
//      on it, and each polygon becomes a wire on the plane of its Newell normal. When every edge
//      has exactly two faces, the faces form one connected shell that can be wound consistently,
//      and every face is a simple, non-degenerate loop, that is the solid: winding made
//      consistent, turned outward by the sign of the enclosed volume, assembled with BRep_Builder.
//   2. Otherwise (an open shell, an edge one face splits where its neighbour does not, three faces
//      on an edge, ...): BOPAlgo_MakerVolume over the faces built on their own -- the general
//      route, and the old cost.
//
// BRepBuilderAPI_Sewing is deliberately not a step: its edge matching is far worse than linear on
// many small faces (minutes for a bead of ~800 triangles, where MakerVolume takes under a second).
//
// Nothing here touches shared or static state: every call builds its own algorithms, so many
// calls can run at once on different threads (see solids_from_polygons_batch).

#include <BOPAlgo_MakerVolume.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <Geom_Plane.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace adacpp::faceted {

using Point = std::array<double, 3>;
using Loop = std::vector<Point>;
using Polygons = std::vector<Loop>;

// Which step built the solid (reported for tests and diagnostics).
enum class Method { Direct = 0, MakerVolume = 1 };

struct Built {
    TopoDS_Shape shape;
    Method method = Method::Direct;
};

inline const char *method_name(Method m) {
    switch (m) {
    case Method::Direct:
        return "direct";
    case Method::MakerVolume:
        return "maker_volume";
    }
    return "?";
}

namespace detail {

// Welds points closer than `tol` into one index (a uniform grid; each lookup scans the 27 cells
// around the point, so two points either side of a cell boundary still meet).
class Welder {
public:
    explicit Welder(double tol) : cell_(tol > 0.0 ? tol : 1e-12), tol2_(cell_ * cell_) {}

    int add(const Point &p) {
        const long long ix = (long long) std::floor(p[0] / cell_);
        const long long iy = (long long) std::floor(p[1] / cell_);
        const long long iz = (long long) std::floor(p[2] / cell_);
        for (long long dx = -1; dx <= 1; ++dx)
            for (long long dy = -1; dy <= 1; ++dy)
                for (long long dz = -1; dz <= 1; ++dz) {
                    auto it = grid_.find(key(ix + dx, iy + dy, iz + dz));
                    if (it == grid_.end())
                        continue;
                    for (int idx : it->second) {
                        const Point &q = points_[idx];
                        const double ddx = p[0] - q[0], ddy = p[1] - q[1], ddz = p[2] - q[2];
                        if (ddx * ddx + ddy * ddy + ddz * ddz <= tol2_)
                            return idx;
                    }
                }
        const int idx = (int) points_.size();
        points_.push_back(p);
        grid_[key(ix, iy, iz)].push_back(idx);
        return idx;
    }

    const std::vector<Point> &points() const {
        return points_;
    }

private:
    static std::uint64_t key(long long x, long long y, long long z) {
        // Mixed, not packed: coordinates far from the origin at a fine tolerance overflow any fixed
        // bit split, and a collision only costs an extra distance test.
        std::uint64_t h = (std::uint64_t) x * 0x9E3779B97F4A7C15ull;
        h ^= (std::uint64_t) y * 0xC2B2AE3D27D4EB4Full + (h << 6) + (h >> 2);
        h ^= (std::uint64_t) z * 0x165667B19E3779F9ull + (h << 6) + (h >> 2);
        return h;
    }

    double cell_, tol2_;
    std::vector<Point> points_;
    std::unordered_map<std::uint64_t, std::vector<int>> grid_;
};

inline std::uint64_t edge_key(int a, int b) {
    const auto lo = (std::uint32_t) std::min(a, b), hi = (std::uint32_t) std::max(a, b);
    return ((std::uint64_t) lo << 32) | hi;
}

struct EdgeUse {
    int poly[2] = {-1, -1};
    bool forward[2] = {false, false}; // the loop runs low -> high index along it
    int count = 0;
    int index = -1; // into the built edge list
};

inline Point sub(const Point &a, const Point &b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

// The face of each loop on its own: what the MakerVolume step starts from.
inline std::vector<TopoDS_Face> polygon_faces(const Polygons &polygons) {
    std::vector<TopoDS_Face> faces;
    faces.reserve(polygons.size());
    for (const Loop &loop : polygons) {
        if (loop.size() < 3)
            continue;
        BRepBuilderAPI_MakePolygon mp;
        for (const Point &p : loop)
            mp.Add(gp_Pnt(p[0], p[1], p[2]));
        mp.Close();
        if (!mp.IsDone())
            continue; // every corner on one point: no face
        BRepBuilderAPI_MakeFace mf(mp.Wire(), Standard_True);
        if (mf.IsDone())
            faces.push_back(mf.Face());
    }
    return faces;
}

// A loop of welded corner ids with repeats in a row (and a closing repeat of the first) dropped.
// Returns false for a loop left with fewer than three corners: a sliver with no area, whose edges
// run there and back along the same line, so leaving it out leaves the rest balanced.
inline bool clean_loop(std::vector<int> &ids) {
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    while (ids.size() > 1 && ids.front() == ids.back())
        ids.pop_back();
    return ids.size() >= 3;
}

// Step 1, on welded corners `pts` and loops of indices into them. Returns a null shape when the
// loops are not a single closed, orientable shell of simple planar faces (the caller then goes on
// to step 2); throws nothing of its own.
inline TopoDS_Shape direct_solid(const std::vector<Point> &pts, std::vector<std::vector<int>> loops, double tol) {
    if (loops.size() < 4)
        return {};

    // Every edge, and which loops use it which way.
    std::unordered_map<std::uint64_t, EdgeUse> edges;
    edges.reserve(loops.size() * 2);
    std::vector<std::vector<std::uint64_t>> loop_edges(loops.size());
    for (int li = 0; li < (int) loops.size(); ++li) {
        const auto &ids = loops[li];
        loop_edges[li].reserve(ids.size());
        for (size_t i = 0; i < ids.size(); ++i) {
            const int a = ids[i], b = ids[(i + 1) % ids.size()];
            const std::uint64_t k = edge_key(a, b);
            EdgeUse &u = edges[k];
            if (u.count >= 2)
                return {}; // three faces on one edge: not a 2-manifold shell
            u.poly[u.count] = li;
            u.forward[u.count] = a < b;
            ++u.count;
            loop_edges[li].push_back(k);
        }
    }
    for (const auto &kv : edges)
        if (kv.second.count != 2 || kv.second.poly[0] == kv.second.poly[1])
            return {}; // an open edge (or a loop meeting itself): not closed as given

    // Wind every loop the same way as its neighbours: across a shared edge the two loops must
    // run opposite ways. One connected shell only; a second component goes to the general route.
    std::vector<int> flip(loops.size(), -1);
    flip[0] = 0;
    std::deque<int> queue{0};
    size_t reached = 1;
    while (!queue.empty()) {
        const int p = queue.front();
        queue.pop_front();
        for (std::uint64_t k : loop_edges[p]) {
            const EdgeUse &u = edges[k];
            const int side = u.poly[0] == p ? 0 : 1;
            const int q = u.poly[1 - side];
            const bool dp = u.forward[side] != (flip[p] == 1);
            const int want = (u.forward[1 - side] == dp) ? 1 : 0; // q must run against p
            if (flip[q] == -1) {
                flip[q] = want;
                queue.push_back(q);
                ++reached;
            } else if (flip[q] != want) {
                return {}; // not orientable
            }
        }
    }
    if (reached != loops.size())
        return {};
    for (size_t li = 0; li < loops.size(); ++li)
        if (flip[li] == 1)
            std::reverse(loops[li].begin(), loops[li].end());

    // Outward: the enclosed volume, as the sum of the fan tetrahedra about the first corner.
    const Point &o = pts[loops[0][0]];
    double vol6 = 0.0;
    for (const auto &ids : loops) {
        const Point a = sub(pts[ids[0]], o);
        for (size_t i = 1; i + 1 < ids.size(); ++i) {
            const Point b = sub(pts[ids[i]], o), c = sub(pts[ids[i + 1]], o);
            vol6 += a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) +
                    a[2] * (b[0] * c[1] - b[1] * c[0]);
        }
    }
    if (!(std::abs(vol6) > 0.0) || !std::isfinite(vol6))
        return {};
    if (vol6 < 0.0)
        for (auto &ids : loops)
            std::reverse(ids.begin(), ids.end());

    BRep_Builder bb;
    const double vtol = tol > 0.0 ? tol : 1e-7;
    std::vector<TopoDS_Vertex> verts(pts.size());
    for (size_t i = 0; i < pts.size(); ++i)
        bb.MakeVertex(verts[i], gp_Pnt(pts[i][0], pts[i][1], pts[i][2]), vtol);

    std::vector<TopoDS_Edge> built_edges;
    built_edges.reserve(edges.size());
    for (auto &kv : edges) {
        const int lo = (int) (kv.first >> 32), hi = (int) (kv.first & 0xffffffffu);
        BRepBuilderAPI_MakeEdge me(verts[lo], verts[hi]);
        if (!me.IsDone())
            return {};
        kv.second.index = (int) built_edges.size();
        built_edges.push_back(me.Edge());
    }

    TopoDS_Shell shell;
    bb.MakeShell(shell);
    std::vector<int> sorted_ids;
    for (const auto &ids : loops) {
        const size_t n = ids.size();
        // A corner visited twice (a loop pinched to a figure eight) is not one face.
        sorted_ids.assign(ids.begin(), ids.end());
        std::sort(sorted_ids.begin(), sorted_ids.end());
        if (std::adjacent_find(sorted_ids.begin(), sorted_ids.end()) != sorted_ids.end())
            return {};
        // Newell's normal (right-handed about the loop, so outward; its length is twice the area),
        // the centroid and the longest side.
        double nx = 0, ny = 0, nz = 0, cx = 0, cy = 0, cz = 0, longest = 0;
        for (size_t i = 0; i < n; ++i) {
            const Point &a = pts[ids[i]], &b = pts[ids[(i + 1) % n]];
            nx += (a[1] - b[1]) * (a[2] + b[2]);
            ny += (a[2] - b[2]) * (a[0] + b[0]);
            nz += (a[0] - b[0]) * (a[1] + b[1]);
            cx += a[0];
            cy += a[1];
            cz += a[2];
            const Point d = sub(b, a);
            longest = std::max(longest, d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        }
        longest = std::sqrt(longest);
        const double nlen = std::sqrt(nx * nx + ny * ny + nz * nz);
        // No area, or a sliver narrower than the tolerance across its longest side (2A / base).
        if (!(nlen > 0.0) || !(nlen / longest > vtol))
            return {};
        nx /= nlen;
        ny /= nlen;
        nz /= nlen;
        cx /= (double) n;
        cy /= (double) n;
        cz /= (double) n;
        double dev = 0.0; // how far the corners stand off the face's plane
        for (int id : ids)
            dev = std::max(dev, std::abs((pts[id][0] - cx) * nx + (pts[id][1] - cy) * ny + (pts[id][2] - cz) * nz));
        const gp_Ax3 frame(gp_Pnt(cx, cy, cz), gp_Dir(nx, ny, nz));
        if (n > 3) {
            // A triangle with area is always a simple loop; a longer loop must not cross itself.
            // Checked in the face's own plane (its sides as 2D segments, adjacent sides skipped).
            const gp_Dir &ux = frame.XDirection(), &uy = frame.YDirection();
            std::vector<std::array<double, 2>> uv(n);
            for (size_t i = 0; i < n; ++i) {
                const Point d = sub(pts[ids[i]], {cx, cy, cz});
                uv[i] = {d[0] * ux.X() + d[1] * ux.Y() + d[2] * ux.Z(), d[0] * uy.X() + d[1] * uy.Y() + d[2] * uy.Z()};
            }
            auto orient = [](const std::array<double, 2> &p, const std::array<double, 2> &q,
                             const std::array<double, 2> &r) {
                return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
            };
            // r on segment pq, given that the three are collinear.
            auto within = [](const std::array<double, 2> &p, const std::array<double, 2> &q,
                             const std::array<double, 2> &r) {
                return std::min(p[0], q[0]) <= r[0] && r[0] <= std::max(p[0], q[0]) && std::min(p[1], q[1]) <= r[1] &&
                       r[1] <= std::max(p[1], q[1]);
            };
            for (size_t i = 0; i < n; ++i)
                for (size_t j = i + 2; j < n; ++j) {
                    if (i == 0 && j == n - 1)
                        continue; // adjacent through the closing side
                    const auto &a = uv[i], &b = uv[(i + 1) % n], &c = uv[j], &d = uv[(j + 1) % n];
                    const double d1 = orient(c, d, a), d2 = orient(c, d, b), d3 = orient(a, b, c), d4 = orient(a, b, d);
                    const bool cross =
                        ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
                    const bool touch = (d1 == 0 && within(c, d, a)) || (d2 == 0 && within(c, d, b)) ||
                                       (d3 == 0 && within(a, b, c)) || (d4 == 0 && within(a, b, d));
                    if (cross || touch)
                        return {}; // two sides cross or touch: not one simple loop
                }
        }
        TopoDS_Wire wire;
        bb.MakeWire(wire);
        for (size_t i = 0; i < n; ++i) {
            const TopoDS_Edge &e = built_edges[edges[edge_key(ids[i], ids[(i + 1) % n])].index];
            bb.Add(wire, ids[i] < ids[(i + 1) % n] ? e : TopoDS::Edge(e.Reversed()));
        }
        wire.Closed(Standard_True);
        // The plane through the centroid, facing along the loop's normal, bounded by the wire as is
        // (it runs counter-clockwise about that normal, so it is the outer boundary). Its sides need
        // no stored pcurves: OCCT derives a line's curve on a plane on demand.
        TopoDS_Face face;
        bb.MakeFace(face, new Geom_Plane(frame), vtol);
        bb.Add(face, wire);
        if (dev > vtol) {
            // A nearly planar polygon (a quad a hair out of plane): the face is the plane through
            // its centroid, and its boundary is allowed to stand off it by that much.
            const double t = dev * 1.01;
            bb.UpdateFace(face, t);
            for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
                const TopoDS_Edge &e = TopoDS::Edge(ex.Current());
                if (BRep_Tool::Tolerance(e) < t)
                    bb.UpdateEdge(e, t);
            }
            for (TopExp_Explorer ex(face, TopAbs_VERTEX); ex.More(); ex.Next()) {
                const TopoDS_Vertex &v = TopoDS::Vertex(ex.Current());
                if (BRep_Tool::Tolerance(v) < t)
                    bb.UpdateVertex(v, t);
            }
        }
        bb.Add(shell, face);
    }
    shell.Closed(Standard_True);
    TopoDS_Solid solid;
    bb.MakeSolid(solid);
    bb.Add(solid, shell);
    // No BRepCheck_Analyzer pass here: it was four fifths of the cost of a 300-face solid, and
    // what it would find is ruled out above by construction -- every edge shared by exactly two
    // faces running opposite ways along it, one connected shell enclosing a positive volume, and
    // every face a simple, non-degenerate loop on its own plane (within the tolerance).
    return solid;
}

// Step 2. Throws std::invalid_argument when the faces bound no volume.
inline TopoDS_Shape maker_volume_solid(const std::vector<TopoDS_Face> &faces, double tol) {
    BOPAlgo_MakerVolume mv;
    TopTools_ListOfShape args;
    for (const TopoDS_Face &f : faces)
        args.Append(f);
    mv.SetArguments(args);
    mv.SetIntersect(Standard_True);
    if (tol > 0.0)
        mv.SetFuzzyValue(tol);
    mv.Perform();
    if (mv.HasErrors())
        throw std::runtime_error("solid_from_polygons: BOPAlgo_MakerVolume reported errors");
    std::vector<TopoDS_Shape> cells;
    for (TopExp_Explorer ex(mv.Shape(), TopAbs_SOLID); ex.More(); ex.Next())
        cells.push_back(ex.Current());
    if (cells.empty())
        throw std::invalid_argument("solid_from_polygons: the polygons bound no volume (the shell is not closed)");
    // More than one cell (faces that touch inside) is fused back into the one solid they make.
    TopoDS_Shape shape = cells[0];
    for (size_t i = 1; i < cells.size(); ++i) {
        BRepAlgoAPI_Fuse fuse(shape, cells[i]);
        if (!fuse.IsDone())
            throw std::runtime_error("solid_from_polygons: fusing the cells MakerVolume found failed");
        shape = fuse.Shape();
    }
    return shape;
}

// Both steps on welded corners and their loops.
inline Built solid_from_welded(const std::vector<Point> &pts, std::vector<std::vector<int>> loops, double tol) {
    if (loops.empty())
        throw std::invalid_argument("solid_from_polygons: no polygon with at least 3 distinct corners");
    try {
        TopoDS_Shape s = direct_solid(pts, loops, tol);
        if (!s.IsNull())
            return {s, Method::Direct};
        Polygons polygons;
        polygons.reserve(loops.size());
        for (const auto &ids : loops) {
            Loop loop;
            loop.reserve(ids.size());
            for (int id : ids)
                loop.push_back(pts[id]);
            polygons.push_back(std::move(loop));
        }
        const std::vector<TopoDS_Face> faces = polygon_faces(polygons);
        if (faces.empty())
            throw std::invalid_argument("solid_from_polygons: no polygon makes a face");
        return {maker_volume_solid(faces, tol), Method::MakerVolume};
    } catch (const Standard_Failure &e) {
        throw std::runtime_error(std::string("solid_from_polygons: OCCT failure: ") + e.GetMessageString());
    }
}

// Runs build(i) for i in [0, n) on `threads` threads (0: `hardware_threads`). An item that throws
// gets a null shape and the reason in errors[i]; the rest go on. Serial where there are no threads
// (the wasm build) or on one item.
template <class Build>
void run_batch(size_t n, int threads, unsigned hardware_threads, Build build, std::vector<TopoDS_Shape> &shapes,
               std::vector<std::string> &errors, std::vector<Method> &methods) {
    shapes.assign(n, TopoDS_Shape());
    errors.assign(n, std::string());
    methods.assign(n, Method::Direct);
    std::atomic<size_t> next{0};
    auto worker = [&] {
        for (;;) {
            const size_t i = next.fetch_add(1, std::memory_order_relaxed);
            if (i >= n)
                return;
            try {
                Built b = build(i);
                shapes[i] = b.shape;
                methods[i] = b.method;
            } catch (const Standard_Failure &e) {
                errors[i] = std::string("OCCT failure: ") + e.GetMessageString();
            } catch (const std::exception &e) {
                errors[i] = e.what();
            } catch (...) {
                errors[i] = "unknown failure";
            }
            if (errors[i].empty() && shapes[i].IsNull())
                errors[i] = "solid_from_polygons: no shape";
        }
    };
    size_t nth = threads > 0 ? (size_t) threads : (size_t) std::max(1u, hardware_threads);
    nth = std::min(nth, n);
#ifdef __EMSCRIPTEN__
    nth = std::min<size_t>(nth, 1);
#endif
    if (nth <= 1) {
        worker();
        return;
    }
    std::vector<std::thread> pool;
    pool.reserve(nth - 1);
    try {
        for (size_t t = 0; t + 1 < nth; ++t)
            pool.emplace_back(worker);
    } catch (const std::system_error &) {
        // No more threads to be had: the ones that started, and this one, share the rest.
    }
    worker();
    for (std::thread &th : pool)
        th.join();
}

} // namespace detail

// The solid the polygons (loops of points) bound; coincident corners are welded within
// `tolerance`. Throws std::invalid_argument when they bound no volume, std::runtime_error when a
// kernel step fails outright.
inline Built solid_from_polygons(const Polygons &polygons, double tolerance) {
    detail::Welder welder(tolerance);
    std::vector<std::vector<int>> loops;
    loops.reserve(polygons.size());
    for (const Loop &loop : polygons) {
        std::vector<int> ids;
        ids.reserve(loop.size());
        for (const Point &p : loop)
            ids.push_back(welder.add(p));
        if (detail::clean_loop(ids))
            loops.push_back(std::move(ids));
    }
    return detail::solid_from_welded(welder.points(), std::move(loops), tolerance);
}

// The same for an indexed mesh: `vertices` and `faces` as loops of 0-based indices into them.
// Vertices that coincide within `tolerance` are still welded, so an unwelded mesh (one copy of a
// corner per face) closes as well as a welded one.
inline Built solid_from_mesh(const std::vector<Point> &vertices, const std::vector<std::vector<int>> &faces,
                             double tolerance) {
    detail::Welder welder(tolerance);
    std::vector<int> welded(vertices.size());
    for (size_t i = 0; i < vertices.size(); ++i)
        welded[i] = welder.add(vertices[i]);
    std::vector<std::vector<int>> loops;
    loops.reserve(faces.size());
    for (const auto &face : faces) {
        std::vector<int> ids;
        ids.reserve(face.size());
        for (int v : face) {
            if (v < 0 || (size_t) v >= vertices.size())
                throw std::invalid_argument("solid_from_mesh: a face index is out of range of the vertices");
            ids.push_back(welded[v]);
        }
        if (detail::clean_loop(ids))
            loops.push_back(std::move(ids));
    }
    return detail::solid_from_welded(welder.points(), std::move(loops), tolerance);
}

using Mesh = std::pair<std::vector<Point>, std::vector<std::vector<int>>>;

inline void solids_from_polygons_batch(const std::vector<Polygons> &items, double tolerance, int threads,
                                       unsigned hardware_threads, std::vector<TopoDS_Shape> &shapes,
                                       std::vector<std::string> &errors, std::vector<Method> &methods) {
    detail::run_batch(
        items.size(), threads, hardware_threads, [&](size_t i) { return solid_from_polygons(items[i], tolerance); },
        shapes, errors, methods);
}

inline void solids_from_meshes_batch(const std::vector<Mesh> &items, double tolerance, int threads,
                                     unsigned hardware_threads, std::vector<TopoDS_Shape> &shapes,
                                     std::vector<std::string> &errors, std::vector<Method> &methods) {
    detail::run_batch(
        items.size(), threads, hardware_threads,
        [&](size_t i) { return solid_from_mesh(items[i].first, items[i].second, tolerance); }, shapes, errors, methods);
}

} // namespace adacpp::faceted

#endif // ADACPP_CAD_FACETED_SOLID_H
