// Test the FEA kernels wasm module (adacpp_fea) under node: load combinations, derived components,
// envelopes and the AFBL/AFEL bytes, against references computed here in plain JS.
//
// The references are exact, not approximate. A float32 product or sum computed in double and rounded
// once with Math.fround IS the correctly rounded float32 result (doubles carry more than 2*24+2 bits),
// so `fround(fround(c * x) + acc)` is bit-for-bit what the kernel's float32 ops must give. The
// derivations run in double in both places, in the same order, and round once at the end.
//
// node has no OPFS, so this exercises WASMFS's in-heap backend: the same read / write code path, the
// files just live in the wasm heap instead of OPFS (the browser deployment target).
//
// Usage:
//   node tools/test_fea_wasm.mjs                 the test suite
//   node tools/test_fea_wasm.mjs --job job.json  run a job file (used by tests/py/test_fea.py to
//                                                compare engines byte for byte)
//   node tools/test_fea_wasm.mjs --bench [n_elements] [n_ips] [n_comp] [n_terms]
//
// ADACPP_FEA_WASM=<path to adacpp_fea.js> overrides the default build location.

import {readFileSync, writeFileSync} from "node:fs";
import {dirname, join} from "node:path";
import {fileURLToPath, pathToFileURL} from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const modPath = process.env.ADACPP_FEA_WASM || join(here, "../build-wasm-fea/wasm_output/adacpp_fea.js");
// A file:// URL, not a bare path: on Windows "C:\..." would be read as a URL scheme.
const createMod = (await import(pathToFileURL(modPath).href)).default;
const Module = await createMod();

let failures = 0;
const check = (name, cond, detail) => {
    if (cond) {
        console.log(`  ok   ${name}${detail ? " - " + detail : ""}`);
    } else {
        failures++;
        console.log(`  FAIL ${name}${detail ? " - " + detail : ""}`);
    }
};

// --- AFBL / AFEL encoding, written out independently of the module --------------------------------

function encodeHeader(obj, magic) {
    // Python json.dumps(obj, separators=(",", ":")) for ASCII names: JSON.stringify writes the same.
    const json = JSON.stringify(obj);
    const bytes = new Uint8Array(1024);
    const dv = new DataView(bytes.buffer);
    for (let i = 0; i < 4; i++)
        bytes[i] = magic.charCodeAt(i);
    dv.setUint32(4, 1, true);
    dv.setUint32(8, json.length, true);
    for (let i = 0; i < json.length; i++)
        bytes[12 + i] = json.charCodeAt(i);
    return bytes;
}

function afelBlob(name, elemType, nSteps, nElements, nIps, nComp, data) {
    const stride = nElements * nIps * nComp * 4;
    const header = encodeHeader({
        name,
        elem_type : elemType,
        n_steps : nSteps,
        n_elements : nElements,
        n_ips : nIps,
        n_components : nComp,
        dtype : "float32",
        stride_bytes : stride
    },
                                "AFEL");
    const out = new Uint8Array(1024 + data.byteLength);
    out.set(header, 0);
    out.set(new Uint8Array(data.buffer, data.byteOffset, data.byteLength), 1024);
    return out;
}

function afblBlob(name, nSteps, nPoints, nComp, data) {
    const header = encodeHeader({
        name,
        n_steps : nSteps,
        n_points : nPoints,
        n_components : nComp,
        dtype : "float32",
        stride_bytes : nPoints * nComp * 4
    },
                                "AFBL");
    const out = new Uint8Array(1024 + data.byteLength);
    out.set(header, 0);
    out.set(new Uint8Array(data.buffer, data.byteOffset, data.byteLength), 1024);
    return out;
}

// Deterministic float32 test data spanning several decades, with a few NaNs.
function randomFloats(n, seed) {
    let s = seed >>> 0;
    const next = () => {
        s = (Math.imul(s, 1664525) + 1013904223) >>> 0;
        return s / 4294967296;
    };
    const out = new Float32Array(n);
    for (let i = 0; i < n; i++)
        out[i] = (next() * 2 - 1) * Math.pow(10, next() * 9 - 3);
    return out;
}

const f = Math.fround;

function refCombine(strides, factors) {
    const n = strides[0].length;
    const out = new Float32Array(n);
    for (let i = 0; i < n; i++) {
        let acc = f(factors[0] * strides[0][i]);
        for (let t = 1; t < strides.length; t++)
            acc = f(acc + f(factors[t] * strides[t][i]));
        out[i] = acc;
    }
    return out;
}

const vonMises = (sx, sy, t) => Math.sqrt(sx * sx + sy * sy - sx * sy + 3.0 * t * t);

function bytesEqual(a, b) {
    if (a.length !== b.length)
        return false;
    for (let i = 0; i < a.length; i++)
        if (a[i] !== b[i])
            return false;
    return true;
}

function floatsOf(bytes, offset, n) {
    const copy = bytes.slice(offset, offset + n * 4);
    return new Float32Array(copy.buffer);
}

// --- modes ------------------------------------------------------------------------------------------

async function runJob(jobPath) {
    // {"inputs": {"<wasm path>": "<host path>"}, "ops": [{"verb": "combine"|"envelope", ...}],
    //  "outputs": {"<wasm path>": "<host path>"}}
    const job = JSON.parse(readFileSync(jobPath, "utf8"));
    Module.FS.mkdirTree("/job");
    for (const [wasmPath, hostPath] of Object.entries(job.inputs))
        Module.FS.writeFile(wasmPath, readFileSync(hostPath));
    const results = [];
    for (const op of job.ops) {
        let res;
        if (op.verb === "combine")
            res = Module.combineField(JSON.stringify(op.paths), op.steps, new Float32Array(op.factors), op.out,
                                      op.derive ? JSON.stringify(op.derive) : "");
        else if (op.verb === "envelope")
            res = Module.envelopeField(JSON.stringify(op.paths), op.steps, op.out, op.gov || "");
        else
            throw new Error(`unknown verb ${op.verb}`);
        results.push(JSON.parse(res));
    }
    for (const [wasmPath, hostPath] of Object.entries(job.outputs))
        writeFileSync(hostPath, Module.FS.readFile(wasmPath));
    console.log(JSON.stringify(results));
    process.exit(results.every((r) => r.ok) ? 0 : 1);
}

function runBench(nElements, nIps, nComp, nTerms) {
    const nSteps = nTerms;
    const per = nElements * nIps * nComp;
    console.log(`bench: ${nElements} elements x ${nIps} ips x ${nComp} comps (${per} floats/stride, ${
        (per * 4 / 1048576).toFixed(1)} MiB), ${nTerms} terms`);
    const data = new Float32Array(per * nSteps);
    for (let s = 0; s < nSteps; s++)
        data.set(randomFloats(per, 1000 + s), s * per);
    Module.FS.writeFile("/bench.bin", afelBlob("G-STRESS", "QUAD4", nSteps, nElements, nIps, nComp, data));
    const paths = JSON.stringify(Array(nTerms).fill("/bench.bin"));
    const steps = Array.from({length : nTerms}, (_, i) => i);
    const factors = new Float32Array(Array.from({length : nTerms}, (_, i) => 1.1 + 0.1 * i));
    const derive = nComp >= 4 ? JSON.stringify([ {op : "plane_von_mises", args : [ 0, 1, 2 ], out : [ 3 ]} ]) : "";
    const times = [];
    let last;
    for (let rep = 0; rep < 7; rep++) {
        const t0 = performance.now();
        last = JSON.parse(Module.combineField(paths, steps, factors, "/bench.out", derive));
        times.push(performance.now() - t0);
    }
    times.sort((a, b) => a - b);
    console.log(JSON.stringify({engine : "wasm", median_ms : times[3], min_ms : times[0], timing_ms : last.timing_ms}));
}

async function runTests() {
    console.log(`adacpp_fea: ${Module.version()}`);

    // An element field: 257 elements x 4 ips x 4 comps (SIGXX, SIGYY, TAUXY, VONMISES), 5 stored steps.
    const nE = 257, nIp = 4, nC = 4, nS = 5;
    const per = nE * nIp * nC;
    const data = randomFloats(per * nS, 42);
    data[17] = NaN;
    const blob = afelBlob("G-STRESS", "QUAD4", nS, nE, nIp, nC, data);
    Module.FS.mkdirTree("/fea");
    Module.FS.writeFile("/fea/base.bin", blob);

    const hdr = JSON.parse(Module.readBlobHeader("/fea/base.bin"));
    check("readBlobHeader", hdr.ok && hdr.kind === "AFEL" && hdr.n_elements === nE && hdr.n_ips === nIp,
          JSON.stringify(hdr));

    // combination: 1.2 * step3 + (-1.1) * step0 + 0.35 * step4, VONMISES re-derived in place
    const steps = [ 3, 0, 4 ];
    const factors = new Float32Array([ 1.2, -1.1, 0.35 ]);
    const res =
        JSON.parse(Module.combineField(JSON.stringify(Array(3).fill("/fea/base.bin")), steps, factors, "/fea/case.bin",
                                       JSON.stringify([ {op : "plane_von_mises", args : [ 0, 1, 2 ], out : [ 3 ]} ])));
    check("combineField ok", res.ok, res.error);
    const strides = steps.map((s) => data.subarray(s * per, (s + 1) * per));
    const want = refCombine(strides, factors);
    for (let r = 0; r < nE * nIp; r++)
        want[r * 4 + 3] = f(vonMises(want[r * 4], want[r * 4 + 1], want[r * 4 + 2]));
    const wantBlob = afelBlob("G-STRESS", "QUAD4", 1, nE, nIp, nC, want);
    const got = Module.FS.readFile("/fea/case.bin");
    check("combination + von Mises: bytes identical to the JS reference (header + payload)", bytesEqual(got, wantBlob));

    // stats: per-component finite min/max
    let mn = Infinity, mx = -Infinity;
    for (let r = 0; r < nE * nIp; r++) {
        const v = want[r * 4];
        if (Number.isFinite(v)) {
            mn = Math.min(mn, v);
            mx = Math.max(mx, v);
        }
    }
    const rng = res.steps[0].scalar_range_per_component[0];
    check("per-step scalar range (SIGXX)", rng[0] === mn && rng[1] === mx, `${rng} vs ${mn},${mx}`);

    // stepIdx as a single number (every term the same step), plain-array factors
    const one = JSON.parse(Module.combineField(JSON.stringify([ "/fea/base.bin" ]), 2, [ 1.0 ], "/fea/one.bin", ""));
    const oneBytes = Module.FS.readFile("/fea/one.bin");
    check("single term, factor 1: the stored stride verbatim",
          one.ok && bytesEqual(oneBytes.subarray(1024), new Uint8Array(data.buffer, 2 * per * 4, per * 4)));

    // P-STRESS: a new 2-column layout derived from the combined G-STRESS
    const pres = JSON.parse(Module.combineField(
        JSON.stringify(Array(3).fill("/fea/base.bin")), steps, factors, "/fea/p.bin", JSON.stringify({
            name : "P-STRESS",
            n_components : 2,
            ops : [ {op : "plane_principal", args : [ 0, 1, 2 ], out : [ 0, 1 ]} ]
        })));
    const pwant = new Float32Array(nE * nIp * 2);
    for (let r = 0; r < nE * nIp; r++) {
        const sx = want[r * 4], sy = want[r * 4 + 1], t = want[r * 4 + 2];
        const c = 0.5 * (sx + sy);
        const h = 0.5 * (sx - sy);
        const rad = Math.sqrt(h * h + t * t);
        pwant[r * 2] = f(c + rad);
        pwant[r * 2 + 1] = f(c - rad);
    }
    check("P-STRESS from the combination (new layout)",
          pres.ok && bytesEqual(Module.FS.readFile("/fea/p.bin"), afelBlob("P-STRESS", "QUAD4", 1, nE, nIp, 2, pwant)));

    // The same with the manifest's single-output op names (one derived component per op).
    const pres2 = JSON.parse(Module.combineField(JSON.stringify(Array(3).fill("/fea/base.bin")), steps, factors,
                                                 "/fea/p2.bin", JSON.stringify({
                                                     name : "P-STRESS",
                                                     n_components : 2,
                                                     ops : [
                                                         {op : "plane_principal_1", args : [ 0, 1, 2 ], out : [ 0 ]},
                                                         {op : "plane_principal_2", args : [ 0, 1, 2 ], out : [ 1 ]}
                                                     ]
                                                 })));
    check("plane_principal_1 / plane_principal_2 == plane_principal",
          pres2.ok && bytesEqual(Module.FS.readFile("/fea/p2.bin"), Module.FS.readFile("/fea/p.bin")));

    // A nodal field: DISPLACEMENT (ALL, X, Y, Z, RX, RY, RZ), ALL = magnitude of X, Y, Z
    const nP = 1001, nD = 7;
    const disp = randomFloats(nP * nD * 3, 7);
    Module.FS.writeFile("/fea/disp.bin", afblBlob("DISPLACEMENT", 3, nP, nD, disp));
    const dres = JSON.parse(Module.combineField(
        JSON.stringify([ "/fea/disp.bin", "/fea/disp.bin" ]), [ 2, 1 ], new Float32Array([ 0.5, 2.0 ]),
        "/fea/dcase.bin", JSON.stringify([ {op : "magnitude3", args : [ 1, 2, 3 ], out : [ 0 ]} ])));
    const dwant =
        refCombine([ disp.subarray(2 * nP * nD, 3 * nP * nD), disp.subarray(nP * nD, 2 * nP * nD) ], [ 0.5, 2.0 ]);
    for (let r = 0; r < nP; r++) {
        const x = dwant[r * 7 + 1], y = dwant[r * 7 + 2], z = dwant[r * 7 + 3];
        dwant[r * 7] = f(Math.sqrt(x * x + y * y + z * z));
    }
    check("nodal DISPLACEMENT combination + magnitude (AFBL bytes)",
          dres.ok && bytesEqual(Module.FS.readFile("/fea/dcase.bin"), afblBlob("DISPLACEMENT", 1, nP, nD, dwant)));

    // envelope over the stored steps: step 0 = max, step 1 = min, governing uint16
    const eres = JSON.parse(Module.envelopeField(JSON.stringify(Array(nS).fill("/fea/base.bin")), [ 0, 1, 2, 3, 4 ],
                                                 "/fea/env.bin", "/fea/env.gov"));
    check("envelopeField ok", eres.ok && eres.n_steps === 2, eres.error);
    const env = Module.FS.readFile("/fea/env.bin");
    const gov = Module.FS.readFile("/fea/env.gov");
    const emax = floatsOf(env, 1024, per), emin = floatsOf(env, 1024 + per * 4, per);
    // AFGV sidecar: "AFGV", uint32 version 1, uint32 n_cases, uint32 0, then uint16 [2 x stride]
    const gh = new DataView(gov.buffer, gov.byteOffset, 16);
    check("governing sidecar header (AFGV, version 1, n_cases)",
          String.fromCharCode(...gov.subarray(0, 4)) === "AFGV" && gh.getUint32(4, true) === 1 &&
              gh.getUint32(8, true) === nS && gh.getUint32(12, true) === 0);
    const g = new Uint16Array(gov.slice(16).buffer);
    let envOk = gov.length === 16 + 2 * per * 2;
    for (let i = 0; i < per && envOk; i++) {
        let bx = NaN, bn = NaN, gx = 0, gn = 0;
        for (let s = 0; s < nS; s++) {
            const v = data[s * per + i];
            if (v > bx || (Number.isNaN(bx) && !Number.isNaN(v))) {
                bx = v;
                gx = s;
            }
            if (v < bn || (Number.isNaN(bn) && !Number.isNaN(v))) {
                bn = v;
                gn = s;
            }
        }
        envOk = Object.is(emax[i], bx) && Object.is(emin[i], bn) && g[i] === gx && g[per + i] === gn;
    }
    check("envelope max / min / governing case", envOk);

    // errors come back as JSON, never as a throw
    const bad = JSON.parse(Module.combineField(JSON.stringify([ "/fea/base.bin" ]), [ 9 ], [ 1 ], "/fea/x.bin", ""));
    check("out-of-range step -> {ok:false}", !bad.ok && /out of range/.test(bad.error), bad.error);
    const bad2 = JSON.parse(Module.combineField(JSON.stringify([ "/fea/nope.bin" ]), 0, [ 1 ], "/fea/x.bin", ""));
    check("missing file -> {ok:false}", !bad2.ok, bad2.error);
    const bad3 = JSON.parse(Module.combineField(JSON.stringify([ "/fea/base.bin" ]), 0, [ 1 ], "/fea/x.bin",
                                                JSON.stringify([ {op : "nope", args : [], out : []} ])));
    check("unknown op -> {ok:false}", !bad3.ok && /unknown derivation op/.test(bad3.error), bad3.error);

    if (failures) {
        console.log(`adacpp_fea: ${failures} check(s) FAILED`);
        process.exit(1);
    }
    console.log("adacpp_fea: all checks passed");
}

const argv = process.argv.slice(2);
if (argv[0] === "--job") {
    await runJob(argv[1]);
} else if (argv[0] === "--bench") {
    const [ne, nip, nc, nt] = argv.slice(1).map(Number);
    runBench(ne || 131072, nip || 10, nc || 4, nt || 6);
} else {
    await runTests();
}
