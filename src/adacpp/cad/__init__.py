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

bbox           = _cad.bbox

# Shape-algebra verbs (mirror adapy's CadBackend surface).
boolean        = _cad.boolean
transform      = _cad.transform
distance       = _cad.distance
serialize      = _cad.serialize
is_valid       = _cad.is_valid
faces          = _cad.faces
vertex_points  = _cad.vertex_points
face_plane     = _cad.face_plane

read_step_bytes = _cad.read_step_bytes
write_glb_bytes = _cad.write_glb_bytes

# Bridge from a raw OCCT TopoDS_Shape pointer (e.g. pythonocc-core's
# `int(shape.this)`) into a cad ShapeHandle. Available on both targets;
# real callers only exist where another module produces TopoDS_Shape
# pointers (i.e. native via pythonocc-core).
from_topods_pointer = _cad.from_topods_pointer

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
    "bbox",
    "boolean",
    "transform",
    "distance",
    "serialize",
    "is_valid",
    "faces",
    "vertex_points",
    "face_plane",
    "read_step_bytes",
    "write_glb_bytes",
    "from_topods_pointer",
]
