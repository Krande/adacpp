// Smoke-test the in-browser clash check: load adacpp_ifc_glb, write an IFC into the emscripten FS,
// call clashJoints (embind), read the joints JSON back and check it describes real joints.
//
// Usage: node tools/test_ifc_clash_wasm.mjs <path-to.ifc> [expected-joint-count]
//
// This is the browser's whole clash path minus the browser: read the file, find the joints,
// classify them, all in C++. The same pass adapy calls through nanobind on a worker.

import {readFileSync} from "node:fs";
import {dirname, join} from "node:path";
import {fileURLToPath} from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const modPath = join(here, "../build-wasm-ifcglb/wasm_output/adacpp_ifc_glb.js");

const ifcPath = process.argv[2];
const expected = process.argv[3] ? Number(process.argv[3]) : null;
if (!ifcPath) {
    console.error("usage: node test_ifc_clash_wasm.mjs <ifc-file> [expected-joints]");
    process.exit(2);
}

const createMod = (await import(modPath)).default;
const Module = await createMod();
Module.FS.writeFile("/in.ifc", readFileSync(ifcPath));

const n = Module.clashJoints("/in.ifc", "/joints.json", 0.1, 1e-5);
console.log("clashJoints returned joints =", n);
if (n < 0) {
    console.error("FAIL: clashJoints reported an error");
    process.exit(1);
}

const doc = JSON.parse(Buffer.from(Module.FS.readFile("/joints.json")).toString("utf8"));
console.log("beams =", doc.beams, " joints =", doc.joints.length);
for (const j of doc.joints) {
    console.log(`  ${j.members.map((m) => m.name).join(" + ")}  ${j.type_key}`);
}

const problems = [];
if (doc.schema !== "adacpp.clash_joints/1")
    problems.push(`bad schema: ${doc.schema}`);
if (doc.joints.length !== n)
    problems.push(`count mismatch: ${n} vs ${doc.joints.length}`);
if (expected !== null && n !== expected)
    problems.push(`expected ${expected} joints, got ${n}`);
// A joint is a CONTACT, so it has at least two members and a type key built from them. Empty
// records would satisfy every count above and mean nothing.
for (const j of doc.joints) {
    if (j.members.length < 2)
        problems.push("a joint with fewer than two members");
    if (!j.type_key || !j.type_key.includes("|"))
        problems.push(`a joint with no type key: ${j.type_key}`);
}

if (problems.length) {
    console.error("FAIL:\n  " + problems.join("\n  "));
    process.exit(1);
}
console.log("WASM IFC clash check OK ✓");
