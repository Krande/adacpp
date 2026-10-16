// Test the convex-contact clash wasm module (adacpp_clash) under node, against the parity fixture:
// the contacts coal's own Python bindings report for the same meshes (tools/gen_clash_cases.py). The
// Python tests (tests/py/test_clash.py) check the nanobind module against the same file, so the two
// engines are held to one answer.
//
// Usage: node tools/test_clash_wasm.mjs <adacpp_clash.js> <convex_contact_cases.json>

import {readFileSync} from "node:fs";
import {dirname, join} from "node:path";
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

// A real viewer GLB (glb_case.glb, next to the cases file) through findGlbContacts: the record was
// made by the nanobind module on the same bytes (tools/gen_clash_glb_case.py), so this is the browser
// and the worker held to one answer.
{
    const dir = dirname(casesPath);
    const glbCase = JSON.parse(readFileSync(join(dir, "glb_case.json"), "utf-8"));
    const glb = new Uint8Array(readFileSync(join(dir, glbCase.glb)));
    console.log(`viewer GLB ${glbCase.glb}`);
    const r = Module.findGlbContacts(glb, glbCase.security_margin, glbCase.min_contact_area,
                                     glbCase.line_contact_thickness, "z");
    if (r.error) {
        check("findGlbContacts runs", false, r.error);
    } else {
        const members = r.memberIds.map((id, k) => [id, r.memberNames[k]]);
        check("members", JSON.stringify(members) === JSON.stringify(glbCase.members), JSON.stringify(members));
        check("contact count", r.penetrationDepth.length === glbCase.contacts.length,
              `${r.penetrationDepth.length} vs ${glbCase.contacts.length}`);
        let worst = 0, pairsOk = true;
        glbCase.contacts.forEach((e, q) => {
            pairsOk &&= r.pairs[2 * q] === e.pair[0] && r.pairs[2 * q + 1] === e.pair[1];
            worst = Math.max(worst, maxDiff(r.near1, 3 * q, e.near1), maxDiff(r.near2, 3 * q, e.near2),
                             maxDiff(r.normal, 3 * q, e.normal), Math.abs(r.penetrationDepth[q] - e.penetration_depth),
                             Math.abs(r.contactArea[q] - e.contact_area) / e.contact_area);
            const p = r.patchOffsets[q];
            const v0 = r.patchPointOffsets[p], v1 = r.patchPointOffsets[p + 1];
            if (v1 - v0 !== e.patches[0].points1.length)
                worst = Infinity;
            else
                worst = Math.max(worst, maxDiff(r.patchPoints1, 3 * v0, e.patches[0].points1.flat()));
        });
        check("same pairs in the same order", pairsOk);
        check("same contacts and patches", worst <= TOL, `max |diff| ${worst}`);
    }
    const notGlb = Module.findGlbContacts(new Uint8Array([ 1, 2, 3 ]), 0.01, 1e-6, 1e-3, "z");
    check("bytes that are not a GLB are an error", typeof notGlb.error === "string", notGlb.error);

    // glbMembers: the members findGlbContacts measures, as a findConvexContacts soup. All of them,
    // hulled, must give findGlbContacts' contacts exactly (same C++, same doubles); a subset only the
    // contacts among that subset.
    console.log(`glbMembers ${glbCase.glb}`);
    const opts = [ glbCase.security_margin, glbCase.min_contact_area, glbCase.line_contact_thickness ];
    const m = Module.glbMembers(glb, "z");
    if (m.error || r.error) {
        check("glbMembers runs", false, m.error || r.error);
    } else {
        const n = m.ids.length;
        check("ids / names are findGlbContacts' members",
              JSON.stringify(m.ids) === JSON.stringify(r.memberIds) &&
                  JSON.stringify(m.names) === JSON.stringify(r.memberNames),
              `${n} members`);
        check("typed columns", m.vertices instanceof Float64Array && m.vertexOffsets instanceof Uint32Array &&
                                   m.triangles instanceof Uint32Array && m.triangleOffsets instanceof Uint32Array);
        check("offsets have n+1 entries and cover the arrays",
              m.vertexOffsets.length === n + 1 && m.triangleOffsets.length === n + 1 && m.vertexOffsets[0] === 0 &&
                  m.triangleOffsets[0] === 0 && 3 * m.vertexOffsets[n] === m.vertices.length &&
                  3 * m.triangleOffsets[n] === m.triangles.length);
        let local = true, everyoneHasTriangles = true;
        for (let k = 0; k < n; ++k) {
            const nv = m.vertexOffsets[k + 1] - m.vertexOffsets[k];
            everyoneHasTriangles &&= m.triangleOffsets[k + 1] > m.triangleOffsets[k];
            for (let t = 3 * m.triangleOffsets[k]; t < 3 * m.triangleOffsets[k + 1]; ++t)
                local &&= m.triangles[t] < nv;
        }
        check("every member has triangles", everyoneHasTriangles);
        check("triangle indices are member-local", local);

        // The same columns findGlbContacts returns, compared exactly.
        const COLS = [
            "pairs", "near1", "near2", "normal", "penetrationDepth", "contactArea", "patchOffsets", "patchNormals",
            "patchPointOffsets", "patchPoints1", "patchPoints2", "degenerate"
        ];
        const all =
            Module.findConvexContacts(m.vertices, m.vertexOffsets, m.triangles, m.triangleOffsets, ...opts, true);
        if (all.error) {
            check("findConvexContacts(glbMembers) runs", false, all.error);
        } else {
            const differ = COLS.filter((c) => JSON.stringify(Array.from(all[c])) !== JSON.stringify(Array.from(r[c])));
            check("findConvexContacts(all glbMembers) == findGlbContacts",
                  differ.length === 0 && all.candidatePairs === r.candidatePairs && all.belowMinArea === r.belowMinArea,
                  differ.length ? `differ: ${differ}` : `${all.penetrationDepth.length} contacts`);
        }

        // Members `pick` of `m`, concatenated as a caller filtering several GLBs would.
        const subset = (pick) => {
            const vOff = new Uint32Array(pick.length + 1), tOff = new Uint32Array(pick.length + 1);
            pick.forEach((k, s) => {
                vOff[s + 1] = vOff[s] + m.vertexOffsets[k + 1] - m.vertexOffsets[k];
                tOff[s + 1] = tOff[s] + m.triangleOffsets[k + 1] - m.triangleOffsets[k];
            });
            const vertices = new Float64Array(3 * vOff[pick.length]),
                  triangles = new Uint32Array(3 * tOff[pick.length]);
            pick.forEach((k, s) => {
                vertices.set(m.vertices.subarray(3 * m.vertexOffsets[k], 3 * m.vertexOffsets[k + 1]), 3 * vOff[s]);
                triangles.set(m.triangles.subarray(3 * m.triangleOffsets[k], 3 * m.triangleOffsets[k + 1]),
                              3 * tOff[s]);
            });
            return [ vertices, vOff, triangles, tOff ];
        };
        const [a, b] = [ r.pairs[0], r.pairs[1] ]; // the first touching pair, a < b
        const two = Module.findConvexContacts(...subset([ a, b ]), ...opts, true);
        const expect = [];
        for (let q = 0; q < r.penetrationDepth.length; ++q)
            if (r.pairs[2 * q] === a && r.pairs[2 * q + 1] === b)
                expect.push(q);
        if (two.error) {
            check("findConvexContacts(two glbMembers) runs", false, two.error);
        } else {
            let same = two.penetrationDepth.length === expect.length && expect.length > 0;
            expect.forEach((q, s) => {
                same &&= two.pairs[2 * s] === 0 && two.pairs[2 * s + 1] === 1 &&
                         two.penetrationDepth[s] === r.penetrationDepth[q] && two.contactArea[s] === r.contactArea[q] &&
                         [ 0, 1, 2 ].every((k) => two.near1[3 * s + k] === r.near1[3 * q + k] &&
                                                  two.near2[3 * s + k] === r.near2[3 * q + k]);
            });
            check(`two members (${m.ids[a]}, ${m.ids[b]}) give only their own contact`, same,
                  `${two.penetrationDepth.length} vs ${expect.length}`);
        }
        const ym = Module.glbMembers(glb, "y");
        let yOk = !ym.error && ym.vertices.length === m.vertices.length;
        for (let v = 0; yOk && v < m.vertices.length; v += 3)
            yOk = ym.vertices[v] === m.vertices[v] && ym.vertices[v + 1] === -m.vertices[v + 2] &&
                  ym.vertices[v + 2] === m.vertices[v + 1];
        check("upAxis \"y\" maps (x, y, z) -> (x, -z, y)", yOk, ym.error);
    }
    const notGlbMembers = Module.glbMembers(new Uint8Array([ 1, 2, 3 ]), "z");
    check("glbMembers of bytes that are not a GLB is an error", typeof notGlbMembers.error === "string",
          notGlbMembers.error);
    const badAxis = Module.glbMembers(glb, "x");
    check("glbMembers with an unknown upAxis is an error", typeof badAxis.error === "string", badAxis.error);
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
