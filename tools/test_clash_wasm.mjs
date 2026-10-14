// Test the convex-contact clash wasm module (adacpp_clash) under node, against the parity fixture:
// the contacts coal's own Python bindings report for the same meshes (tools/gen_clash_cases.py). The
// Python tests (tests/py/test_clash.py) check the nanobind module against the same file, so the two
// engines are held to one answer.
//
// Usage: node tools/test_clash_wasm.mjs <adacpp_clash.js> <convex_contact_cases.json>

import {readFileSync} from "node:fs";
import {pathToFileURL} from "node:url";

const [modPath, casesPath] = process.argv.slice(2);
if (!modPath || !casesPath) {
    console.error("usage: node tools/test_clash_wasm.mjs <adacpp_clash.js> <convex_contact_cases.json>");
    process.exit(2);
}
// A file:// URL, not a bare path: on Windows "C:\..." would be read as a URL scheme.
const createMod = (await import(pathToFileURL(modPath).href)).default;
const Module = await createMod();
const fixture = JSON.parse(readFileSync(casesPath, "utf-8"));

// coal's GJK/EPA is iterative; the same sources built by another compiler may round differently in
// the last bits. Points, normals and depths agree to well under a micrometre, areas relatively.
const TOL = 1e-9;
const AREA_RTOL = 1e-7;

let failures = 0;
const check = (name, cond, detail) => {
    if (cond) {
        console.log(`  ok   ${name}${detail ? " - " + detail : ""}`);
    } else {
        failures++;
        console.log(`  FAIL ${name}${detail ? " - " + detail : ""}`);
    }
};

function soup(meshes) {
    let nv = 0, nt = 0;
    for (const m of meshes) {
        nv += m.vertices.length;
        nt += m.triangles.length;
    }
    const vertices = new Float64Array(3 * nv), triangles = new Uint32Array(3 * nt);
    const vOff = new Uint32Array(meshes.length + 1), tOff = new Uint32Array(meshes.length + 1);
    meshes.forEach((m, k) => {
        vertices.set(m.vertices.flat(), 3 * vOff[k]);
        triangles.set(m.triangles.flat(), 3 * tOff[k]);
        vOff[k + 1] = vOff[k] + m.vertices.length;
        tOff[k + 1] = tOff[k] + m.triangles.length;
    });
    return [ vertices, vOff, triangles, tOff ];
}

const maxDiff = (a, off, b) => Math.max(...b.map((x, k) => Math.abs(a[off + k] - x)));

// As given (the fixture's meshes are hulls), and re-hulled here: the hull of a hull keeps the input's
// vertex order, so it must give exactly the same contacts (see tests/py/test_clash.py).
for (const [hull, tol, areaRtol] of [[ false, TOL, AREA_RTOL ], [ true, TOL, AREA_RTOL ]])
    for (const c of fixture.cases) {
        console.log(`${c.name}${hull ? " (re-hulled)" : ""}`);
        const r = Module.findConvexContacts(...soup(c.meshes), fixture.security_margin, fixture.min_contact_area,
                                            fixture.line_contact_thickness, hull);
        if (r.error) {
            check("runs", false, r.error);
            continue;
        }
        check("no degenerate meshes", r.degenerate.length === 0, `${Array.from(r.degenerate)}`);
        check("contact count", r.penetrationDepth.length === c.contacts.length,
              `${r.penetrationDepth.length} vs ${c.contacts.length}`);
        check("below min area", r.belowMinArea === c.below_min_area, `${r.belowMinArea} vs ${c.below_min_area}`);
        check("no failures", r.failures.length === 0, r.failures.map((f) => f.reason).join("; "));
        c.contacts.forEach((e, q) => {
            if (q >= r.penetrationDepth.length)
                return;
            const tag = `contact ${e.pair}`;
            check(`${tag} pair`, r.pairs[2 * q] === e.pair[0] && r.pairs[2 * q + 1] === e.pair[1],
                  `${r.pairs[2 * q]},${r.pairs[2 * q + 1]}`);
            const d =
                Math.max(maxDiff(r.near1, 3 * q, e.near1), maxDiff(r.near2, 3 * q, e.near2),
                         maxDiff(r.normal, 3 * q, e.normal), Math.abs(r.penetrationDepth[q] - e.penetration_depth));
            check(`${tag} geometry`, d <= tol, `max |diff| ${d.toExponential(2)}`);
            const ra = Math.abs(r.contactArea[q] - e.contact_area) / Math.max(Math.abs(e.contact_area), 1e-300);
            check(`${tag} area`, ra <= areaRtol, `${r.contactArea[q]} vs ${e.contact_area}`);
            // The contact-patch polygons, from the flat columns.
            const p0 = r.patchOffsets[q], p1 = r.patchOffsets[q + 1];
            let pd = p1 - p0 === e.patches.length ? 0 : Infinity;
            e.patches.forEach((w, k) => {
                const p = p0 + k;
                if (p >= p1)
                    return;
                const v0 = r.patchPointOffsets[p], v1 = r.patchPointOffsets[p + 1];
                if (v1 - v0 !== w.points1.length) {
                    pd = Infinity;
                    return;
                }
                pd = Math.max(pd, maxDiff(r.patchNormals, 3 * p, w.normal),
                              maxDiff(r.patchPoints1, 3 * v0, w.points1.flat()),
                              maxDiff(r.patchPoints2, 3 * v0, w.points2.flat()));
            });
            check(`${tag} patches`, pd <= tol, `max |diff| ${pd}`);
        });
    }

// Malformed input is an {error}, not a trap.
const bad = Module.findConvexContacts(new Float64Array(9), new Uint32Array([ 0, 3 ]), new Uint32Array([ 0, 1, 5 ]),
                                      new Uint32Array([ 0, 1 ]), 0.01, 1e-6, 1e-3, false);
check("bad triangle index is an error", typeof bad.error === "string", bad.error);
const short = Module.findConvexContacts(new Float64Array(3), new Uint32Array([ 0, 2 ]), new Uint32Array(0),
                                        new Uint32Array([ 0, 0 ]), 0.01, 1e-6, 1e-3, true);
check("offsets past the arrays are an error", typeof short.error === "string", short.error);

console.log(failures ? `${failures} FAILED` : "all ok");
process.exit(failures ? 1 : 0);
