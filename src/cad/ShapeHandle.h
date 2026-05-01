#ifndef ADACPP_CAD_SHAPE_HANDLE_H
#define ADACPP_CAD_SHAPE_HANDLE_H

#include <TopoDS_Shape.hxx>

// Backend-agnostic opaque shape handle wrapping an OCCT TopoDS_Shape (a
// refcounted value type — copy is cheap). Same wrapper on native and wasm:
// both targets statically link OCCT so the kernel and ABI are identical.
//
// The Python class registered for ShapeHandle exposes no readable members,
// so callers can only obtain one via factory functions (make_box, ...) and
// pass it to consumers (tessellate, ...). C++ code accesses internals
// directly via topods().
class ShapeHandle {
public:
    explicit ShapeHandle(TopoDS_Shape shape) : shape_(std::move(shape)) {}
    const TopoDS_Shape& topods() const { return shape_; }

private:
    TopoDS_Shape shape_;
};

#endif // ADACPP_CAD_SHAPE_HANDLE_H
