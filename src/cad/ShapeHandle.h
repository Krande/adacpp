#ifndef ADACPP_CAD_SHAPE_HANDLE_H
#define ADACPP_CAD_SHAPE_HANDLE_H

#ifndef __EMSCRIPTEN__
#include <TopoDS_Shape.hxx>
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
        Box,
    };

    ShapeHandle(Kind kind, float dx, float dy, float dz)
        : kind_(kind), dx_(dx), dy_(dy), dz_(dz) {}

    Kind kind() const { return kind_; }
    float dx() const { return dx_; }
    float dy() const { return dy_; }
    float dz() const { return dz_; }

private:
    Kind kind_;
    float dx_;
    float dy_;
    float dz_;
#endif
};

#endif // ADACPP_CAD_SHAPE_HANDLE_H
