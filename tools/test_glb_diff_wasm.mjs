// Smoke test for the in-browser GLB-diff wasm module (adacpp_glb_diff).
//   pixi run -e wasm wbuild-glbdiff            # builds build-wasm-glbdiff/wasm_output/adacpp_glb_diff.js
//   node tools/test_glb_diff_wasm.mjs <scene.glb> <ref.glb> [mode]
// Prints the diff counts + overlay size. Verifies the embind module parses GLBs (incl.
// EXT_meshopt_compression), matches, and emits the removed overlay — entirely in wasm, no server.
//
// Note: the .js output uses import.meta, so node needs it as ESM — either run from a dir with
// {"type":"module"} or copy adacpp_glb_diff.js -> .mjs (this test imports the .js directly under an
// .mjs runner, which works on node>=20).
import { createRequire } from "module";
import fs from "fs";
import path from "path";

const here = path.dirname(new URL(import.meta.url).pathname);
const modPath = path.join(here, "..", "build-wasm-glbdiff", "wasm_output", "adacpp_glb_diff.js");
const require = createRequire(import.meta.url);
// load as ESM default export
const { default: createAdacppGlbDiff } = await import(modPath);

const [scene, ref, mode = "byName"] = process.argv.slice(2);
if (!scene || !ref) {
  console.error("usage: node test_glb_diff_wasm.mjs <scene.glb> <ref.glb> [mode]");
  process.exit(2);
}
const m = await createAdacppGlbDiff();
const u8 = (f) => new Uint8Array(fs.readFileSync(f));
const r = m.diffGlb(u8(scene), u8(ref), mode, 1e-3, 0xd50000ff);
console.log("counts:", JSON.stringify(r.counts));
console.log("ops:", r.ops.length, "overlay bytes:", r.overlay.length);
