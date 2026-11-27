// SPIKE (option B, adapy-plugins#177 step 2d): gmsh in node-Pyodide over a SHARED OCCT.
//
// Installs the three wheels in dependency order -- ada_occt (libocct.so), ada_cpp (NEEDS libocct.so),
// gmsh (gmsh.py + libgmsh.so, NEEDS libocct.so) -- then runs tools/gmsh_spike_cases.py, the same file
// the native comparison runs, and checks the counts against the native (conda gmsh 4.15.2 + ada-cpp)
// answers pinned below.
//
// Run: node tools/test_gmsh_pyodide.js [ada_occt.whl ada_cpp.whl gmsh.whl]   (default: dist/*.whl)

const fs = require("fs");
const path = require("path");
const {performance} = require("perf_hooks");

const REPO = path.resolve(__dirname, "..");
const CASES = path.join(__dirname, "gmsh_spike_cases.py");

// Native answers: conda-forge gmsh/python-gmsh 4.15.2 + ada-cpp 0.45.0 (linux-64), same cases file.
// Structured (transfinite) meshes must match exactly.
const NATIVE = {
    rect_transfinite: {nodes: 231, quads: 200},
    adacpp_plate: {nodes: 231, quads: 200},
    // the box solid via the pointer, fragmented by a gmsh plane: 2 volumes, 11 faces
    adacpp_box_fragment: {nodes: 531, quads: 540, volumes: 2, surfaces: 11},
};
// Unstructured Frontal-Delaunay + Blossom recombination is NOT bit-portable even between native
// builds: on this face linux-64 conda gives 277/240 and win-64 conda the same, but their variants
// already disagree (no smoothing: 277 vs 274; simple recombiner: 275 vs 277 nodes), and with the
// topology pass off win-64 gives exactly what wasm gives (280/243). The triangulation itself
// (275 nodes, 477 triangles) is identical. So: within 2% of linux-64, reported exactly.
const NATIVE_UNSTRUCTURED = {rect_recombine_unstructured: {nodes: 277, quads: 240}};

let failures = 0;
const check = (name, cond, detail) => {
    if (!cond)
        failures++;
    console.log(`  ${cond ? "ok  " : "FAIL"} ${name}${detail ? " — " + detail : ""}`);
};

function wheels() {
    if (process.argv.length >= 5)
        return process.argv.slice(2, 5);
    const dist = path.join(REPO, "dist");
    const all = fs.readdirSync(dist).filter((f) => f.endsWith(".whl"));
    const pick = (prefix) => {
        const w = all.find((f) => f.startsWith(prefix));
        if (!w) {
            console.error(`no ${prefix}*.whl in dist/`);
            process.exit(1);
        }
        return path.join(dist, w);
    };
    return [ pick("ada_occt-"), pick("ada_cpp-"), pick("gmsh-") ];
}

async function main() {
    const ws = wheels();
    for (const w of ws)
        console.log("wheel:", path.basename(w), `${(fs.statSync(w).size / 1e6).toFixed(2)} MB`);

    const t0 = performance.now();
    const {loadPyodide} = require(require.resolve("pyodide"));
    const py = await loadPyodide();
    const tPyodide = performance.now();
    py.FS.mkdirTree("/dist");
    await py.loadPackage([ "micropip", "numpy" ]);
    const tBase = performance.now();
    const micropip = py.pyimport("micropip");
    const tInstall = {};
    for (const w of ws) {
        const t = performance.now();
        py.FS.writeFile("/dist/" + path.basename(w), fs.readFileSync(w));
        await micropip.install("emfs:/dist/" + path.basename(w), {deps : false});
        tInstall[path.basename(w).split("-")[0]] = Math.round(performance.now() - t);
    }
    const tImport0 = performance.now();
    py.runPython("import adacpp.cad, gmsh");
    const tReady = performance.now();
    console.log(`\n[timing] node cold start (ms): loadPyodide=${Math.round(tPyodide - t0)} micropip+numpy=${
        Math.round(tBase - tPyodide)} install=${JSON.stringify(tInstall)} import=${
        Math.round(tReady - tImport0)} total=${Math.round(tReady - t0)}`);

    // One OCCT: every loaded dylib by name. Two entries ending in libocct.so would mean two OCCTs
    // (Pyodide dedupes by the resolved path string, which is what the absolute rpath is for).
    const libs = Object.keys(py._module.LDSO.loadedLibsByName).filter((n) => n.endsWith(".so"));
    console.log("[dylibs]", JSON.stringify(libs));

    py.FS.writeFile("/gmsh_spike_cases.py", fs.readFileSync(CASES));
    const t1 = performance.now();
    const out = JSON.parse(py.runPython(`
import sys, json
sys.path.insert(0, "/")
import gmsh_spike_cases
json.dumps(gmsh_spike_cases.run_all())
`));
    console.log(`[timing] cases run (ms): ${Math.round(performance.now() - t1)}`);
    console.log("\n[results]\n" + JSON.stringify(out, null, 1));

    console.log("\n[checks] vs native conda gmsh 4.15.2");
    const occt = libs.filter((n) => n.endsWith("libocct.so"));
    check("exactly one libocct.so loaded, shared by adacpp and gmsh", occt.length === 1, JSON.stringify(occt));
    check("gmsh.__version__ is 4.15.2", out.build_info && out.build_info.version === "4.15.2",
          out.build_info && (out.build_info.version || out.build_info.error));
    const opts = (out.build_info && out.build_info.build_options) || "";
    check("built with OpenCASCADE + OpenCASCADE-CAF", opts.includes("OpenCASCADE ") && opts.includes("OpenCASCADE-CAF"),
          opts);
    for (const [name, want] of Object.entries(NATIVE)) {
        const r = out[name] || {};
        const quads = (r.elements_2d || {})["Quadrilateral 4"];
        const topo = want.volumes === undefined || (r.volumes === want.volumes && r.surfaces === want.surfaces);
        check(`${name}: nodes/quads match native (${want.nodes}/${want.quads})`,
              r.nodes === want.nodes && quads === want.quads && topo,
              r.error || `nodes=${r.nodes} quads=${quads}${want.volumes === undefined ? "" : ` volumes=${r.volumes} surfaces=${r.surfaces}`}`);
    }
    for (const [name, want] of Object.entries(NATIVE_UNSTRUCTURED)) {
        const r = out[name] || {};
        const quads = (r.elements_2d || {})["Quadrilateral 4"];
        const near = (a, b) => Math.abs(a - b) <= 0.02 * b;
        check(`${name}: within 2% of native ${want.nodes}/${want.quads} (not bit-portable natively either)`,
              near(r.nodes, want.nodes) && near(quads, want.quads), r.error || `nodes=${r.nodes} quads=${quads}`);
    }
    const tri = out.rect_triangles_unstructured || {};
    check("unstructured triangulation (before recombination) matches native exactly (275 nodes / 477 tris)",
          tri.nodes === 275 && (tri.elements_2d || {})["Triangle 3"] === 477,
          tri.error || `nodes=${tri.nodes} tris=${(tri.elements_2d || {})["Triangle 3"]}`);
    const plate = out.adacpp_plate || {};
    check("importShapesNativePointer took adacpp's face as one surface",
          JSON.stringify(plate.imported) === JSON.stringify([ [ 2, 1 ] ]), JSON.stringify(plate.imported));

    console.log(failures === 0 ? "\nGMSH PYODIDE OK" : `\n${failures} CHECK(S) FAILED`);
    process.exit(failures === 0 ? 0 : 1);
}

main().catch((err) => {
    console.error(err);
    process.exit(1);
});
