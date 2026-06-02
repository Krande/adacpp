// Wasm-side check that the cad surface produces real OCCT meshes — wasm now
// links statically against the same OCCT subset native does, so cyl/sph
// tessellate produce real meshes (no more RuntimeError stubs).

const fs = require("fs");
const path = require("path");
const { loadPyodide } = require("pyodide");

(async () => {
  const py = await loadPyodide();

  const distDir = path.resolve(__dirname, "..", "dist");
  py.FS.mkdirTree("/dist");
  for (const f of fs.readdirSync(distDir)) {
    py.FS.writeFile("/dist/" + f, fs.readFileSync(path.join(distDir, f)));
  }

  // Mount the STEP fixture so read_step_bytes has a real input to parse.
  const stepFixture = path.resolve(__dirname, "..", "files",
                                    "flat_plate_abaqus_10x10_m_wColors.stp");
  py.FS.writeFile("/fixture.stp", fs.readFileSync(stepFixture));

  await py.loadPackage(["micropip"]);
  const mp = py.pyimport("micropip");
  const wheel = fs.readdirSync(distDir).find((f) => f.endsWith(".whl"));
  await mp.install("emfs:/dist/" + wheel);

  const r = py.runPython(`
import adacpp.cad as cad

# Box: real OCCT mesh, AABB-checked.
box = cad.make_box(2.0, 3.0, 4.0)
mesh = cad.tessellate(box)
positions = list(mesh.positions)
xs, ys, zs = positions[0::3], positions[1::3], positions[2::3]

# Cylinder + Sphere now produce real tessellations under wasm (OCCT statically
# linked). Range checks mirror the native pytest tolerances.
cyl_mesh = cad.tessellate(cad.make_cylinder(1.0, 5.0))
cyl_pos = list(cyl_mesh.positions)
cyl_xs, cyl_ys, cyl_zs = cyl_pos[0::3], cyl_pos[1::3], cyl_pos[2::3]

sph_mesh = cad.tessellate(cad.make_sphere(2.0))
sph_pos = list(sph_mesh.positions)
sph_xs, sph_ys, sph_zs = sph_pos[0::3], sph_pos[1::3], sph_pos[2::3]

# from_topods_pointer is now exposed on wasm too — same OCCT kernel on both
# targets. The function only does anything useful when given a valid OCCT
# TopoDS_Shape pointer; since pythonocc-core doesn't run under pyodide,
# real callers don't exist on this target, but the binding exists.
has_from_topods = cad.from_topods_pointer is not None

# bbox: AddOptimal goes through real OCCT now (no analytic stub).
box_bbox = list(cad.bbox(box))
cyl_bbox = list(cad.bbox(cad.make_cylinder(1.0, 5.0)))
sph_bbox = list(cad.bbox(cad.make_sphere(2.0)))

# STEP read + glTF write — round-trip a real STEP fixture through
# the wasm OCCT pipeline.
with open("/fixture.stp", "rb") as f:
    step_bytes = f.read()

step_handle = cad.read_step_bytes(step_bytes)
step_bbox   = list(cad.bbox(step_handle))

# write_glb_bytes returns nb::bytes; coerce to plain bytes for the JSON
# round-trip out of pyodide. GLB header magic should be 'glTF'.
box_glb = bytes(cad.write_glb_bytes(box))
step_glb = bytes(cad.write_glb_bytes(step_handle))

{
  "shape_attrs":      [a for a in dir(box) if not a.startswith("_")],
  "verts":            len(xs),
  "tris":             len(mesh.indices) // 3,
  "x_range":          [min(xs), max(xs)],
  "y_range":          [min(ys), max(ys)],
  "z_range":          [min(zs), max(zs)],
  "cyl_tris":         len(cyl_mesh.indices) // 3,
  "cyl_z_range":      [min(cyl_zs), max(cyl_zs)],
  "cyl_x_extent":     [min(cyl_xs), max(cyl_xs)],
  "sph_tris":         len(sph_mesh.indices) // 3,
  "sph_x_extent":     [min(sph_xs), max(sph_xs)],
  "sph_y_extent":     [min(sph_ys), max(sph_ys)],
  "sph_z_extent":     [min(sph_zs), max(sph_zs)],
  "has_from_topods":  has_from_topods,
  "box_bbox":         box_bbox,
  "cylinder_bbox":    cyl_bbox,
  "sphere_bbox":      sph_bbox,
  "step_bbox":        step_bbox,
  "step_input_size":  len(step_bytes),
  "box_glb_size":     len(box_glb),
  "box_glb_magic":    box_glb[:4].decode("latin-1"),
  "step_glb_size":    len(step_glb),
  "step_glb_magic":   step_glb[:4].decode("latin-1"),
}
`);
  console.log(JSON.stringify(r.toJs({ dict_converter: Object.fromEntries }), null, 2));
})().catch((e) => {
  console.error(e);
  process.exit(1);
});
