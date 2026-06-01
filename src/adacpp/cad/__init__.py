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
obb            = _cad.obb

# Placement-aware primitive builders (ada.geom.solids parity).
build_box      = _cad.build_box
build_cylinder = _cad.build_cylinder
build_sphere   = _cad.build_sphere
build_cone     = _cad.build_cone
build_extruded_area_solid = _cad.build_extruded_area_solid
build_revolved_area_solid = _cad.build_revolved_area_solid
build_fixed_reference_swept_area_solid = _cad.build_fixed_reference_swept_area_solid
make_halfspace = _cad.make_halfspace
cut_surfaces   = _cad.cut_surfaces
build_planar_face = _cad.build_planar_face
build_face_based_surface_model = _cad.build_face_based_surface_model
make_wire      = _cad.make_wire

# Shape-algebra verbs (mirror adapy's CadBackend surface).
boolean        = _cad.boolean
transform      = _cad.transform
distance       = _cad.distance
serialize      = _cad.serialize
is_valid       = _cad.is_valid
volume         = _cad.volume
faces          = _cad.faces
solids         = _cad.solids
edges          = _cad.edges
vertex_points  = _cad.vertex_points
face_plane     = _cad.face_plane
to_topods_pointer = _cad.to_topods_pointer

# Topology-kernel ops (non-manifold core: cells from a face soup, non-manifold
# merge, free-face extraction, point-in-solid, centre of mass).
make_volumes_from_faces = _cad.make_volumes_from_faces
merge_cells    = _cad.merge_cells
face_id        = _cad.face_id
non_manifold_merge = _cad.non_manifold_merge
free_faces     = _cad.free_faces
point_in_solid = _cad.point_in_solid
center_of_mass = _cad.center_of_mass
shells         = _cad.shells
wires          = _cad.wires
wire_points    = _cad.wire_points
unify_coplanar_faces = _cad.unify_coplanar_faces

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
    "obb",
    "build_box",
    "build_cylinder",
    "build_sphere",
    "build_cone",
    "build_extruded_area_solid",
    "build_revolved_area_solid",
    "build_fixed_reference_swept_area_solid",
    "make_halfspace",
    "cut_surfaces",
    "build_planar_face",
    "build_face_based_surface_model",
    "make_wire",
    "boolean",
    "transform",
    "distance",
    "serialize",
    "is_valid",
    "volume",
    "faces",
    "solids",
    "edges",
    "vertex_points",
    "face_plane",
    "to_topods_pointer",
    "make_volumes_from_faces",
    "merge_cells",
    "face_id",
    "non_manifold_merge",
    "free_faces",
    "point_in_solid",
    "center_of_mass",
    "shells",
    "wires",
    "wire_points",
    "unify_coplanar_faces",
    "read_step_bytes",
    "write_glb_bytes",
    "from_topods_pointer",
]
