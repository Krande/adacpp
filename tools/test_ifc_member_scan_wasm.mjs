// Smoke-test the IFC member scan through the wasm module under node: load adacpp_ifc_glb, write an
// IFC into the emscripten FS, call scanMembers (embind), read the JSONL back and check that it
// describes the members the file states -- sections, axes, materials.
//
// Usage: node tools/test_ifc_member_scan_wasm.mjs <path-to.ifc> [expected-member-count]
//
// This is the BROWSER's clash-check / take-off path, minus the browser: node has no OPFS, so the
// files live in WASMFS's in-heap backend instead. Same code, same verb; only the backend differs,
// which is exactly what mountOpfs swaps in the worker.

import {readFileSync} from "node:fs";
import {dirname, join} from "node:path";
import {fileURLToPath} from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const modPath = join(here, "../build-wasm-ifcglb/wasm_output/adacpp_ifc_glb.js");

const ifcPath = process.argv[2];
const expected = process.argv[3] ? Number(process.argv[3]) : null;
if (!ifcPath) {
    console.error("usage: node test_ifc_member_scan_wasm.mjs <ifc-file> [expected-count]");
    process.exit(2);
}

const createMod = (await import(modPath)).default;
const Module = await createMod();

Module.FS.writeFile("/in.ifc", readFileSync(ifcPath));

const written = Module.scanMembers("/in.ifc", "/members.jsonl");
console.log("scanMembers returned members =", written);
if (written < 0) {
    console.error("FAIL: scanMembers reported an error");
    process.exit(1);
}

const text = Buffer.from(Module.FS.readFile("/members.jsonl")).toString("utf8");
const lines = text.split("\n").filter((l) => l.length > 0);
const header = JSON.parse(lines[0]);
const records = lines.slice(1).map((l) => JSON.parse(l)); // throws on any malformed line

console.log("header =", JSON.stringify(header));
console.log("records =", records.length);
for (const r of records) {
    console.log(`  ${r.name || "(unnamed)"}  ${r.ifc_class}  profile=${r.profile_name || "-"}  material=${
        r.material || "-"}  outline=${r.outline.length}pts`);
}

const problems = [];
if (header.schema !== "adacpp.ifc_members/1")
    problems.push(`bad schema: ${header.schema}`);
if (records.length !== written)
    problems.push(`header/line mismatch: ${written} vs ${records.length}`);
if (expected !== null && written !== expected)
    problems.push(`expected ${expected} members, got ${written}`);
// The scan exists to answer what the members ARE; a file of empty records would satisfy every
// count above and be worthless, so assert that it actually carries the semantics.
if (!records.some((r) => r.ifc_class && r.outline.length >= 3))
    problems.push("no record carries a profile outline");
if (!records.some((r) => r.material))
    problems.push("no record carries a material");

if (problems.length) {
    console.error("FAIL:\n  " + problems.join("\n  "));
    process.exit(1);
}
console.log("WASM IFC member scan OK ✓");
