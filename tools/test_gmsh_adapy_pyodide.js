// adapy's REAL mesh step in node-Pyodide on the gmsh-carrying
// adacpp wheel. Installs the stack the way the viewer's Pyodide worker does for its FEM path
// (adapy src/frontend/src/utils/pyodide/pyodide_worker.js): numpy/Pillow/pydantic/h5py/sqlite3 from
// Pyodide, trimesh + pyquaternion via micropip, adacpp with micropip, adapy with deps=False. Then
// runs tools/gmsh_adapy_panel.py (to_fem_obj(use_quads=True) -> to_fem("sesam") into memfs)
// and compares with tools/gmsh_adapy_panel_native.json.
//
//   node tools/test_gmsh_adapy_pyodide.js <adacpp wheel> <adapy wheel> [mesh_size] [recomb_alg|-]

const fs = require("fs");
const path = require("path");

const SCRIPT = path.join(__dirname, "gmsh_adapy_panel.py");
const NATIVE = path.join(__dirname, "gmsh_adapy_panel_native.json");

async function main() {
    const [adacpp, adapy, size = "0.25", alg = "-"] = process.argv.slice(2);
    if (!adacpp || !adapy) {
        console.error("usage: node tools/test_gmsh_adapy_pyodide.js <adacpp.whl> <adapy.whl> [mesh_size] [alg|-]");
        process.exit(2);
    }
    const t = {};
    let t0 = performance.now();
    const {loadPyodide} = require(require.resolve("pyodide"));
    const py = await loadPyodide();
    t.loadPyodide = performance.now() - t0;

    t0 = performance.now();
    await py.loadPackage([ "micropip", "numpy", "Pillow", "pydantic", "h5py", "sqlite3" ],
                         {messageCallback : () => {}});
    const micropip = py.pyimport("micropip");
    await micropip.install([ "trimesh", "pyquaternion" ]);
    py.FS.mkdirTree("/wheels");
    for (const w of [adacpp, adapy]) {
        py.FS.writeFile("/wheels/" + path.basename(w), fs.readFileSync(w));
    }
    await micropip.install("emfs:/wheels/" + path.basename(adacpp));
    py.globals.set("_adapy_whl", "emfs:/wheels/" + path.basename(adapy));
    await py.runPythonAsync("import micropip\nawait micropip.install(_adapy_whl, deps=False)");
    t.install_stack = performance.now() - t0;

    t0 = performance.now();
    py.runPython("import ada, ada.fem.meshing");
    t.import_ada_meshing = performance.now() - t0;

    py.FS.writeFile("/gmsh_adapy_panel.py", fs.readFileSync(SCRIPT));
    let got;
    try {
        got = JSON.parse(py.runPython(`
import sys, json
sys.path.insert(0, "/")
import gmsh_adapy_panel as p
json.dumps(p.run("/tmp/panel_deck", ${Number(size)}, ${alg === "-" ? "None" : Number(alg)}))
`));
    } catch (e) {
        console.log("FAIL adapy mesh step —", String(e).split("\n").slice(-6).join(" | "));
        process.exit(1);
    }
    console.log("timings (ms):", Object.entries(t).map(([ k, v ]) => `${k}=${v.toFixed(0)}`).join(" "));
    console.log(JSON.stringify(got, null, 1));

    if (fs.existsSync(NATIVE)) {
        const all = JSON.parse(fs.readFileSync(NATIVE, "utf8"));
        const key = `size=${Number(size)} alg=${alg}`;
        const w = all[key];
        if (w) {
            const same = [ "nodes", "elements", "deck_GNODE", "deck_GELMNT1" ].every((k) => got[k] === w[k]) &&
                         JSON.stringify(got.element_types) === JSON.stringify(w.element_types);
            console.log(`\n[parity ${key}] counts ${same ? "IDENTICAL" : "DIFFER"} to native; deck bytes wasm ${
                got.deck_bytes} native ${w.deck_bytes}; deck sha256 ${
                got.deck_sha256 === w.deck_sha256 ? "identical" : "differs"}; deck body (no DATE/IDENT/TEXT) sha256 ${
                got.deck_body_sha256 === w.deck_body_sha256 ? "IDENTICAL"
                                                            : "differs"}; mesh s wasm ${got.mesh_seconds} native ${
                w.mesh_seconds}; write s wasm ${got.write_seconds} native ${w.write_seconds}`);
            process.exit(same ? 0 : 1);
        }
        console.log(`(no native entry "${key}" in ${path.basename(NATIVE)})`);
    }
}

main().catch((err) => {
    console.error(err);
    process.exit(1);
});
