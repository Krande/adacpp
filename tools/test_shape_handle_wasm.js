// Wasm-side check that make_box + tessellate work and that ShapeHandle is opaque.
// Mirrors test_basic.py's cad_make_box_and_tessellate / cad_shape_handle_is_opaque
// but in pyodide.

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
box = cad.make_box(2.0, 3.0, 4.0)
mesh = cad.tessellate(box)
positions = list(mesh.positions)
xs = positions[0::3]; ys = positions[1::3]; zs = positions[2::3]
{
  "shape_class":   type(box).__name__,
  "shape_attrs":   [a for a in dir(box) if not a.startswith("_")],
  "verts":         len(xs),
  "tris":          len(mesh.indices) // 3,
  "x_range":       [min(xs), max(xs)],
  "y_range":       [min(ys), max(ys)],
  "z_range":       [min(zs), max(zs)],
}
`);
  console.log(JSON.stringify(r.toJs({ dict_converter: Object.fromEntries }), null, 2));
})().catch((e) => {
  console.error(e);
  process.exit(1);
});
