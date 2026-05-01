// Wasm-side check that the cad surface mirrors native semantics where possible
// and fails honestly where the stub doesn't yet have an implementation.

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

  await py.loadPackage(["micropip"]);
  const mp = py.pyimport("micropip");
  const wheel = fs.readdirSync(distDir).find((f) => f.endsWith(".whl"));
  await mp.install("emfs:/dist/" + wheel);

  const r = py.runPython(`
import adacpp.cad as cad

# Box: stub mesh, AABB-checked.
box = cad.make_box(2.0, 3.0, 4.0)
mesh = cad.tessellate(box)
positions = list(mesh.positions)
xs, ys, zs = positions[0::3], positions[1::3], positions[2::3]

# Cylinder / Sphere construct fine but tessellate must throw — we don't fake meshes.
def expect_raises(thunk):
    try:
        thunk()
        return None
    except Exception as e:
        return type(e).__name__ + ": " + str(e)

cyl_err = expect_raises(lambda: cad.tessellate(cad.make_cylinder(1.0, 5.0)))
sph_err = expect_raises(lambda: cad.tessellate(cad.make_sphere(2.0)))

# from_topods_pointer must be absent in wasm builds.
has_from_topods = cad.from_topods_pointer is not None

{
  "shape_attrs":      [a for a in dir(box) if not a.startswith("_")],
  "verts":            len(xs),
  "tris":             len(mesh.indices) // 3,
  "x_range":          [min(xs), max(xs)],
  "y_range":          [min(ys), max(ys)],
  "z_range":          [min(zs), max(zs)],
  "cylinder_error":   cyl_err,
  "sphere_error":     sph_err,
  "has_from_topods":  has_from_topods,
}
`);
  console.log(JSON.stringify(r.toJs({ dict_converter: Object.fromEntries }), null, 2));
})().catch((e) => {
  console.error(e);
  process.exit(1);
});
