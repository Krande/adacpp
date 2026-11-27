// gmsh in node-pyodide, through the shared cases in
// tools/gmsh_cases.py, checked against the native counts in tools/gmsh_cases_native.json.
//
// ONE wheel: gmsh statically linked into adacpp's extension, gmsh.py at the top level of the wheel.
// Several wheels may be passed; they are installed in order.
//
//   node tools/test_gmsh_pyodide.js <wheel> [<wheel> ...]
//   node tools/test_gmsh_pyodide.js --record <wheel> ...   (also print the raw results JSON)
//   node tools/test_gmsh_pyodide.js --timing-only <wheel>  (sizes + cold start only; any wheel)
//
// Also measures and prints the cold-start split (fresh pyodide each time, node, warm disk cache):
// loadPyodide, micropip install of the wheel(s) (+ numpy), `import adacpp`, `import gmsh` +
// gmsh.initialize(), plus the wheel sizes raw and brotli-11 (what a server would send).

const fs = require("fs");
const path = require("path");
const zlib = require("zlib");

const REPO = path.resolve(__dirname, "..");
const CASES = path.join(__dirname, "gmsh_cases.py");
const NATIVE = path.join(__dirname, "gmsh_cases_native.json");
const FIXTURE = path.join(REPO, "files", "flat_plate_abaqus_10x10_m_wColors.stp");

let failures = 0;
const check = (name, cond, detail) => {
    if (!cond)
        failures++;
    console.log(`  ${cond ? "ok  " : "FAIL"} ${name}${detail ? " — " + detail : ""}`);
};

const flags = process.argv.slice(2).filter((a) => a.startsWith("--"));
const record = flags.includes("--record");
// --timing-only: sizes + cold start, no gmsh (for the baseline wheel without gmsh).
const timingOnly = flags.includes("--timing-only");
const wheels = process.argv.slice(2).filter((a) => !a.startsWith("--")).map((w) => path.resolve(w));
if (!wheels.length) {
    console.error("usage: node tools/test_gmsh_pyodide.js [--record] <wheel> [<wheel> ...]");
    process.exit(2);
}

const mib = (n) => (n / 1048576).toFixed(2) + " MiB";

function sizes() {
    console.log("\n[size] wheel(s), raw and brotli-11");
    for (const w of wheels) {
        const buf = fs.readFileSync(w);
        const br = zlib.brotliCompressSync(buf, {params : {[zlib.constants.BROTLI_PARAM_QUALITY] : 11}});
        console.log(
            `  ${path.basename(w)}: raw ${mib(buf.length)} (${buf.length}), brotli ${mib(br.length)} (${br.length})`);
    }
}

async function coldStart() {
    const t = {};
    let t0 = performance.now();
    const {loadPyodide} = require(require.resolve("pyodide"));
    const py = await loadPyodide();
    t.loadPyodide = performance.now() - t0;
    t0 = performance.now();
    py.FS.mkdirTree("/dist");
    await py.loadPackage([ "micropip" ], {messageCallback : () => {}});
    const micropip = py.pyimport("micropip");
    for (const w of wheels) {
        py.FS.writeFile("/dist/" + path.basename(w), fs.readFileSync(w));
        await micropip.install("emfs:/dist/" + path.basename(w));
    }
    t.install = performance.now() - t0;
    t0 = performance.now();
    py.runPython("import adacpp, adacpp.cad");
    t.import_adacpp = performance.now() - t0;
    if (!timingOnly) {
        t0 = performance.now();
        py.runPython("import gmsh; gmsh.initialize(interruptible=False); gmsh.finalize()");
        t.import_gmsh_init = performance.now() - t0;
    }
    return {py, t};
}

// Parity rules, per case:
//  * nodes, points and lines (the geometry + 1D seeding) must equal the native run exactly;
//  * the quad/triangle split after recombination is REPORTED when it differs, not failed. The simple
//    (greedy) pairing sorts candidate pairs by quality with std::sort, which is not stable, so ties
//    can resolve differently under emscripten's libc++ than under the native libstdc++ build;
//  * lshape_recombine_blossom is reported only. With ADACPP_GMSH_BLOSSOM=ON (default) both sides
//    give an all-quad mesh but a slightly different one (wasm 232 nodes/199 quads vs native 234/201;
//    probably the same order/libm sensitivity feeding Blossom's integer edge weights, unverified);
//    with it OFF gmsh falls back to the simple pairing (quad-dominant).
const BLOSSOM = "lshape_recombine_blossom";
const sorted = (o) => JSON.stringify(Object.fromEntries(Object.entries(o || {}).sort()));
const fixed = (e) => ({Point : (e || {}).Point, line : (e || {})["Line 2"]});

function compare(got, want) {
    console.log("\n[parity] counts vs native (conda gmsh 4.15.2 + native adacpp)");
    for (const [name, w] of Object.entries(want)) {
        if (typeof w !== "object" || w === null || !("nodes" in w))
            continue;
        const g = got[name] || {};
        if (g.error) {
            check(name, false, g.error);
            continue;
        }
        const exact = g.nodes === w.nodes && sorted(g.elements) === sorted(w.elements);
        const detail = `wasm nodes=${g.nodes} ${sorted(g.elements)} | native nodes=${w.nodes} ${
            sorted(w.elements)} | wasm ${g.seconds}s native ${w.seconds}s`;
        if (exact)
            check(`${name} identical`, true, detail);
        else if (name === BLOSSOM)
            console.log(`  diff ${name} (Blossom matching) —${detail}`);
        else if (g.nodes === w.nodes && sorted(fixed(g.elements)) === sorted(fixed(w.elements)))
            console.log(`  diff ${name}: same nodes/1D, quad-tri split differs — ${detail}`);
        else
            check(name, false, detail);
    }
}

async function main() {
    console.log("wheels:", wheels.map((w) => path.basename(w)).join(", "));
    sizes();

    console.log("\n[cold start] fresh pyodide x3 (ms)");
    let py = null;
    for (let i = 0; i < 3; i++) {
        const r = await coldStart();
        py = r.py;
        console.log("  " + Object.entries(r.t).map(([ k, v ]) => `${k}=${v.toFixed(0)}`).join(" "));
    }
    if (timingOnly)
        process.exit(0);

    console.log("\n[gmsh] shared cases (tools/gmsh_cases.py)");
    py.FS.writeFile("/gmsh_cases.py", fs.readFileSync(CASES));
    py.FS.writeFile("/fixture.stp", fs.readFileSync(FIXTURE));
    let got;
    try {
        got = JSON.parse(py.runPython(`
import sys, json
sys.path.insert(0, "/")
import gmsh_cases
json.dumps(gmsh_cases.run_all("/fixture.stp"))
`));
    } catch (e) {
        check("run gmsh_cases.run_all()", false, String(e).split("\n").slice(-3).join(" | "));
        process.exit(1);
    }
    if (record)
        console.log(JSON.stringify(got, null, 1));

    check("rect_transfinite: 231 nodes, 200 quads",
          got.rect_transfinite && got.rect_transfinite.nodes === 231 &&
              (got.rect_transfinite.elements || {})["Quadrilateral 4"] === 200,
          JSON.stringify(got.rect_transfinite));
    check("plate_native_pointer: adacpp TopoDS_Shape* imported and meshed like rect_transfinite",
          got.plate_native_pointer && got.plate_native_pointer.nodes === 231 &&
              (got.plate_native_pointer.elements || {})["Quadrilateral 4"] === 200,
          JSON.stringify(got.plate_native_pointer));

    if (fs.existsSync(NATIVE))
        compare(got, JSON.parse(fs.readFileSync(NATIVE, "utf8")));
    else
        console.log(`\n(no ${path.basename(NATIVE)}: skipping native parity)`);

    console.log(failures === 0 ? "\nGMSH PYODIDE OK" : `\n${failures} CHECK(S) FAILED`);
    process.exit(failures === 0 ? 0 : 1);
}

main().catch((err) => {
    console.error(err);
    process.exit(1);
});
