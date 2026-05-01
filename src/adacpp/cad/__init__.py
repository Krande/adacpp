"""Backend-agnostic CAD operations.

This module is the future home of the CadBackend abstraction that will let
adapy swap between pythonocc-core and adacpp implementations. The Python
surface here is identical between the native and wasm/pyodide builds; the
underlying C++ implementations diverge based on which kernel is available.
"""

from .._ada_cpp_ext_impl import cad as _cad

Color          = _cad.Color
GroupReference = _cad.GroupReference
Mesh           = _cad.Mesh
MeshType       = _cad.MeshType
ShapeHandle    = _cad.ShapeHandle

make_box       = _cad.make_box
make_cylinder  = _cad.make_cylinder
make_sphere    = _cad.make_sphere

tessellate     = _cad.tessellate
tessellate_box = _cad.tessellate_box

# Native-only: bridge from a raw OCCT TopoDS_Shape pointer (eg from pythonocc-
# core's `int(shape.this)`) into a cad ShapeHandle. Absent under wasm/pyodide.
from_topods_pointer = getattr(_cad, "from_topods_pointer", None)

__all__ = [
    "Color",
    "GroupReference",
    "Mesh",
    "MeshType",
    "ShapeHandle",
    "make_box",
    "make_cylinder",
    "make_sphere",
    "tessellate",
    "tessellate_box",
    "from_topods_pointer",
]
