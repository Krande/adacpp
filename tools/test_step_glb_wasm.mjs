// Smoke-test the no-pyodide STEP->GLB wasm module under node: load it, write a STEP into the
// emscripten FS, call stepToGlb (embind), read the GLB back, validate the glTF magic + tri count.
//
// Usage: node tools/test_step_glb_wasm.mjs <path-to-small.step>
// (node has no OPFS, so this exercises the WASMFS in-memory backend — same code path, the file just
//  lives in the heap instead of OPFS; OPFS is the browser deployment target.)

import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const modPath = join(here, "../build-wasm-stepglb/wasm_output/adacpp_step_glb.js");

const stepPath = process.argv[2];
if (!stepPath) {
  console.error("usage: node test_step_glb_wasm.mjs <step-file>");
  process.exit(2);
}

const createMod = (await import(modPath)).default;
const Module = await createMod();

const stepBytes = readFileSync(stepPath);
Module.FS.writeFile("/in.step", stepBytes);
Module.FS.mkdir("/spill");

const ntri = Module.stepToGlb("/in.step", "/out.glb", "/spill", 2.0, 20.0, false);
console.log("stepToGlb returned tris =", ntri);

const glb = Module.FS.readFile("/out.glb"); // Uint8Array
const magic = Buffer.from(glb.slice(0, 4)).toString("latin1");
console.log("GLB bytes =", glb.length, " magic =", magic);

if (magic !== "glTF" || ntri <= 0 || glb.length < 100) {
  console.error("FAIL: not a valid non-empty GLB");
  process.exit(1);
}
console.log("WASM STEP->GLB OK ✓");
