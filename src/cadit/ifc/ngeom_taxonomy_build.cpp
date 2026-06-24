// NGEOM -> taxonomy mapping (Part 2). Builds ifcopenshell taxonomy items from the neutral
// geometry layer. Compile-checked against the real ifcgeom/taxonomy.h.
#include <Eigen/Dense>
#include <ifcgeom/taxonomy.h>

#include "../../geom/neutral/ngeom_bspline.h"
#include "ngeom_taxonomy.h"

namespace tax = ifcopenshell::geometry::taxonomy;

namespace adacpp::ngeom {

namespace {

Eigen::Vector3d ev(const Vec3 &v) {
    return Eigen::Vector3d(v.x, v.y, v.z);
}

tax::matrix4::ptr mat4(const Frame &f) {
    return tax::make<tax::matrix4>(ev(f.o), ev(f.z), ev(f.x));
}

// expanded flat knot vector -> distinct knots + multiplicities (taxonomy wants compact)
void compact_knots(const std::vector<double> &flat, std::vector<double> &knots, std::vector<int> &mults) {
    for (double k : flat) {
        if (!knots.empty() && std::abs(k - knots.back()) < 1e-12)
            mults.back()++;
        else {
            knots.push_back(k);
            mults.push_back(1);
        }
    }
}

tax::curve::ptr to_curve(const Curve *c) {
    if (auto *l = dynamic_cast<const LineCurve *>(c)) {
        auto ln = tax::make<tax::line>();
        ln->matrix = tax::make<tax::matrix4>(ev(l->pnt), ev(l->dir)); // origin + direction(=X)
        return ln;
    }
    if (auto *ci = dynamic_cast<const CircleCurve *>(c)) {
        auto cc = tax::make<tax::circle>();
        cc->radius = ci->r;
        cc->matrix = mat4(ci->f);
        return cc;
    }
    if (auto *el = dynamic_cast<const EllipseCurve *>(c)) {
        auto ee = tax::make<tax::ellipse>();
        ee->radius = el->a1;
        ee->radius2 = el->a2;
        ee->matrix = mat4(el->f);
        return ee;
    }
    if (auto *bs = dynamic_cast<const BSplineCurve *>(c)) {
        auto bc = tax::make<tax::bspline_curve>();
        bc->degree = bs->degree;
        for (const Vec3 &p : bs->ctrl)
            bc->control_points.push_back(tax::make<tax::point3>(ev(p)));
        compact_knots(bs->U, bc->knots, bc->multiplicities);
        if (!bs->weights.empty())
            bc->weights = bs->weights;
        bc->matrix = tax::make<tax::matrix4>(); // identity; control points are world-space
        return bc;
    }
    if (auto *tc = dynamic_cast<const TrimmedCurve *>(c)) {
        // map the basis curve; trimming is carried on the edge (start/end)
        return to_curve(tc->basis.get());
    }
    return nullptr;
}

tax::surface::ptr to_surface(const Surface *s) {
    if (auto *p = dynamic_cast<const PlaneSurface *>(s)) {
        auto pl = tax::make<tax::plane>();
        pl->matrix = mat4(p->f);
        return pl;
    }
    if (auto *cy = dynamic_cast<const CylinderSurface *>(s)) {
        auto c = tax::make<tax::cylinder>();
        c->radius = cy->r;
        c->matrix = mat4(cy->f);
        return c;
    }
    if (auto *sp = dynamic_cast<const SphereSurface *>(s)) {
        auto o = tax::make<tax::sphere>();
        o->radius = sp->r;
        o->matrix = mat4(sp->f);
        return o;
    }
    if (auto *to = dynamic_cast<const TorusSurface *>(s)) {
        auto t = tax::make<tax::torus>();
        t->radius1 = to->R;
        t->radius2 = to->r;
        t->matrix = mat4(to->f);
        return t;
    }
    if (auto *bs = dynamic_cast<const BSplineSurface *>(s)) {
        auto b = tax::make<tax::bspline_surface>();
        b->degree = {bs->u_degree, bs->v_degree};
        b->control_points.resize(bs->nu);
        for (int iu = 0; iu < bs->nu; ++iu)
            for (int iv = 0; iv < bs->nv; ++iv)
                b->control_points[iu].push_back(tax::make<tax::point3>(ev(bs->ctrl[iu * bs->nv + iv])));
        compact_knots(bs->Uu, b->knots[0], b->multiplicities[0]);
        compact_knots(bs->Uv, b->knots[1], b->multiplicities[1]);
        if (!bs->weights.empty()) {
            std::vector<std::vector<double>> w(bs->nu);
            for (int iu = 0; iu < bs->nu; ++iu)
                for (int iv = 0; iv < bs->nv; ++iv)
                    w[iu].push_back(bs->weights[iu * bs->nv + iv]);
            b->weights = w;
        }
        b->matrix = tax::make<tax::matrix4>();
        return b;
    }
    // NOTE: ConeSurface has no taxonomy equivalent (gap) -> skip in the taxonomy path.
    // Extrusion/Revolution surfaces are sweeps, not taxonomy surfaces -> skip too.
    return nullptr;
}

tax::edge::ptr to_edge(const OrientedEdgeN &oe) {
    auto e = tax::make<tax::edge>(tax::make<tax::point3>(ev(oe.start)), tax::make<tax::point3>(ev(oe.end)));
    e->matrix = tax::make<tax::matrix4>();
    if (oe.geometry) {
        e->basis = to_curve(oe.geometry.get());
    }
    if (!e->basis) {
        // straight edge: give it an explicit line basis (origin + direction). The OCC kernel
        // derefs edge->basis directly rather than calling loop::calculate_linear_edge_curves().
        auto ln = tax::make<tax::line>();
        ln->matrix = tax::make<tax::matrix4>(ev(oe.start), ev(oe.end - oe.start));
        e->basis = ln;
    }
    return e;
}

} // namespace

std::shared_ptr<tax::shell> to_taxonomy_shell(const std::vector<std::shared_ptr<FaceSurfaceN>> &faces) {
    auto sh = tax::make<tax::shell>();
    sh->closed = true;
    sh->matrix = tax::make<tax::matrix4>(); // geom_item::matrix defaults to null; the kernel derefs it
    for (const auto &fp : faces) {
        if (!fp || !fp->surface)
            continue;
        tax::surface::ptr surf = to_surface(fp->surface.get());
        if (!surf)
            continue; // unmappable surface (e.g. cone) -> skip this face
        auto f = tax::make<tax::face>();
        f->basis = surf;
        f->matrix = tax::make<tax::matrix4>();
        if (!fp->same_sense)
            f->orientation = false;
        bool outer = true;
        for (const FaceBoundN &b : fp->bounds) {
            if (!b.loop || b.loop->is_poly && b.loop->polygon.size() < 3) {
                // poly loops: synthesize straight edges between consecutive points
            }
            auto lp = tax::make<tax::loop>();
            lp->closed = true;
            lp->external = outer;
            lp->matrix = tax::make<tax::matrix4>();
            if (b.loop && b.loop->is_poly) {
                const auto &poly = b.loop->polygon;
                for (size_t i = 0; i < poly.size(); ++i) {
                    OrientedEdgeN oe;
                    oe.start = poly[i];
                    oe.end = poly[(i + 1) % poly.size()];
                    lp->children.push_back(to_edge(oe));
                }
            } else if (b.loop) {
                for (const OrientedEdgeN &oe : b.loop->edges)
                    lp->children.push_back(to_edge(oe));
            }
            if (!b.orientation)
                lp->reverse();
            if (!lp->children.empty())
                f->children.push_back(lp);
            outer = false;
        }
        if (!f->children.empty())
            sh->children.push_back(f);
    }
    return sh->children.empty() ? nullptr : sh;
}

std::shared_ptr<tax::extrusion> to_taxonomy_extrusion(const ExtrusionN &ex) {
    if (!ex.profile)
        return nullptr;
    // Reuse the face builder: the profile is one planar face (local XY, z=0).
    auto sh = to_taxonomy_shell({ex.profile});
    if (!sh || sh->children.empty())
        return nullptr;
    tax::face::ptr face = sh->children[0];
    // matrix = the solid's placement frame (origin, z=axis, x=ref_direction).
    auto m = tax::make<tax::matrix4>(Eigen::Vector3d(ex.frame.o.x, ex.frame.o.y, ex.frame.o.z),
                                     Eigen::Vector3d(ex.frame.z.x, ex.frame.z.y, ex.frame.z.z),
                                     Eigen::Vector3d(ex.frame.x.x, ex.frame.x.y, ex.frame.x.z));
    auto dir = tax::make<tax::direction3>(Eigen::Vector3d(ex.direction.x, ex.direction.y, ex.direction.z));
    return tax::make<tax::extrusion>(m, face, dir, ex.depth);
}

} // namespace adacpp::ngeom
