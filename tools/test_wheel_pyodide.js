// Sanity-check the pyodide wheel by loading it under node-pyodide and calling
// adacpp.cad.tessellate_box. Faster than driving a browser; if this fails the
// browser path will too.
//
// Run via: pixi run -e wasm node tools/test_wheel_pyodide.js [path/to/wheel]

const fs = require("fs");
const path = require("path");

async function main() {
  // node-pyodide ships in pyodide-build's xbuildenv as the python-3.12 install,
  // but the easiest path is to install the pyodide npm package on demand.
  // We do that by shelling out to npm to fetch it into a local node_modules.
  const pyodidePath = require.resolve("pyodide");
  const { loadPyodide } = require(pyodidePath);

  const repoRoot = path.resolve(__dirname, "..");
  const wheelPath =
    process.argv[2] ||
    fs
      .readdirSync(path.join(repoRoot, "dist"))
      .filter((f) => f.endsWith(".whl"))
      .map((f) => path.join(repoRoot, "dist", f))[0];

  if (!wheelPath || !fs.existsSync(wheelPath)) {
    console.error("no wheel found at", wheelPath);
    process.exit(1);
  }
  console.log("wheel:", wheelPath);

  const pyodide = await loadPyodide();
  console.log("pyodide booted, python", pyodide.runPython("import sys; sys.version"));

  // Mount the dist dir into the pyodide FS so micropip can install via file://.
  const distDir = path.dirname(wheelPath);
  pyodide.FS.mkdirTree("/dist");
  for (const f of fs.readdirSync(distDir)) {
    pyodide.FS.writeFile("/dist/" + f, fs.readFileSync(path.join(distDir, f)));
  }

  await pyodide.loadPackage(["micropip"]);
  const micropip = pyodide.pyimport("micropip");
  await micropip.install("emfs:/dist/" + path.basename(wheelPath));
  console.log("wheel installed.");

  const result = pyodide.runPython(`
import adacpp.cad as cad
mesh = cad.tessellate_box(2.0, 3.0, 4.0)
{
    "positions_len": len(mesh.positions),
    "indices_len":   len(mesh.indices),
    "first_positions": list(mesh.positions[:9]),
    "first_indices":   list(mesh.indices[:9]),
}
`);
  console.log("result:", JSON.stringify(result.toJs({ dict_converter: Object.fromEntries }), null, 2));
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
