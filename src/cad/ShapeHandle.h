#ifndef ADACPP_CAD_SHAPE_HANDLE_H
#define ADACPP_CAD_SHAPE_HANDLE_H

#ifndef __EMSCRIPTEN__
#include <TopoDS_Shape.hxx>
#else
#include <array>
#endif

// Backend-agnostic opaque shape handle. Native builds wrap an OCCT TopoDS_Shape
// (which is itself a value type with refcounted internals). Wasm builds carry a
// kind+params descriptor that the wasm tessellation stub interprets — this slot
// becomes a real handle (CGAL polyhedron, pyodide-OCCT shape, ...) once the
// browser kernel comes online.
//
// The Python class registered for ShapeHandle exposes no readable members, so
// callers can only obtain one via factory functions (make_box, ...) and pass it
// to consumers (tessellate, ...). C++ code accesses the internals directly.
class ShapeHandle {
public:
#ifndef __EMSCRIPTEN__
    explicit ShapeHandle(TopoDS_Shape shape) : shape_(std::move(shape)) {}
    const TopoDS_Shape& topods() const { return shape_; }

private:
    TopoDS_Shape shape_;
#else
    enum class Kind {
        Box,        // params: dx, dy, dz, _
        Cylinder,   // params: radius, height, _, _
        Sphere,     // params: radius, _, _, _
    };

    static ShapeHandle box(float dx, float dy, float dz) {
        return ShapeHandle(Kind::Box, {dx, dy, dz, 0.0f});
    }
    static ShapeHandle cylinder(float radius, float height) {
        return ShapeHandle(Kind::Cylinder, {radius, height, 0.0f, 0.0f});
    }
    static ShapeHandle sphere(float radius) {
        return ShapeHandle(Kind::Sphere, {radius, 0.0f, 0.0f, 0.0f});
    }

    Kind kind() const { return kind_; }
    float param(int i) const { return params_[i]; }

private:
    ShapeHandle(Kind kind, std::array<float, 4> params) : kind_(kind), params_(params) {}
    Kind kind_;
    std::array<float, 4> params_;
#endif
};

#endif // ADACPP_CAD_SHAPE_HANDLE_H
