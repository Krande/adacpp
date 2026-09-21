// Smoke-test the prismatic-extrusion expander wasm module under node: load it, expand one
// rectangular section, and check the result is a closed, outward-wound solid of the right volume.
//
// The point is not coverage — tests/ngeom/test_extrude.cpp covers the arithmetic against the same
// expectations natively. The point is that the PUBLISHED artifact runs: a module can link cleanly
// and still fail to instantiate, and this build publishes straight to a registry the viewer
// consumes, so "it compiled" is not evidence that anyone can load it.
//
// Usage: node tools/test_extrude_wasm.mjs

import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const here = dirname(fileURLToPath(import.meta.url));
const modPath = join(here, "../build-wasm-extrude/wasm_output/adacpp_extrude.js");

const createMod = (await import(modPath)).default;
const Module = await createMod();

// A 0.30 x 0.60 rectangle, swept 3.0 along +x. Section coordinates run along the frame's first
// in-section axis (+y here) and the derived second axis (+z).
const W = 0.15;
const H = 0.30;
const L = 3.0;

const section = {
  // Counter-clockwise; the writer normalizes winding, but the expander replays whatever it is given,
  // so the table here is already in the order build_extruded_section would have produced.
  points: new Float64Array([-W, -H, W, -H, W, H, -W, H]),
  // Two cap triangles per ring, two wall triangles per edge. Ring 0 is [0,4), ring 1 is [4,8).
  triangles: new Uint32Array([
    // start cap (facing -x)
    0, 2, 1, 0, 3, 2,
    // end cap (facing +x)
    4, 5, 6, 4, 6, 7,
    // walls
    0, 1, 5, 0, 5, 4,
    1, 2, 6, 1, 6, 5,
    2, 3, 7, 2, 7, 6,
    3, 0, 4, 3, 4, 7,
  ]),
};

const instances = {
  label: new Uint32Array([7]),
  section: new Uint32Array([0]),
  node0: new Uint32Array([0]),
  node1: new Uint32Array([1]),
  origin: new Float64Array([0, 0, 0]),
  xvec: new Float64Array([1, 0, 0]),
  yvec: new Float64Array([0, 1, 0]),
  length: new Float64Array([L]),
};

const points = new Float64Array([0, 0, 0, L, 0, 0]);

const out = Module.expandBeamSolids([section], instances, points);

const fail = (msg) => {
  console.error("FAIL:", msg);
  process.exit(1);
};

if (out.positions.length !== 8 * 3) fail(`expected 24 position floats, got ${out.positions.length}`);
if (out.indices.length !== section.triangles.length) fail("index count does not match the section");
if (out.t.length !== 8) fail(`expected 8 axial parameters, got ${out.t.length}`);
if (out.rangeLabel.length !== 1 || out.rangeLabel[0] !== 7) fail("instance range lost its label");

// Signed volume by the divergence theorem — positive only for a closed, outward-wound surface.
let vol = 0;
for (let i = 0; i < out.indices.length; i += 3) {
  const a = out.indices[i] * 3, b = out.indices[i + 1] * 3, c = out.indices[i + 2] * 3;
  const cx = out.positions[b + 1] * out.positions[c + 2] - out.positions[b + 2] * out.positions[c + 1];
  const cy = out.positions[b + 2] * out.positions[c + 0] - out.positions[b + 0] * out.positions[c + 2];
  const cz = out.positions[b + 0] * out.positions[c + 1] - out.positions[b + 1] * out.positions[c + 0];
  vol += out.positions[a] * cx + out.positions[a + 1] * cy + out.positions[a + 2] * cz;
}
vol /= 6;

const expect = 2 * W * 2 * H * L;
if (Math.abs(vol - expect) > expect * 1e-5) fail(`volume ${vol}, expected ${expect} (winding or caps wrong)`);

// Nodes lie on the sweep axis here, so each ring reads a single parameter: 0 at the start, 1 at the end.
for (let i = 0; i < 4; i++) {
  if (Math.abs(out.t[i]) > 1e-6) fail(`start ring t=${out.t[i]}, expected 0`);
  if (Math.abs(out.t[4 + i] - 1) > 1e-6) fail(`end ring t=${out.t[4 + i]}, expected 1`);
}

console.log(`expandBeamSolids ok: ${out.positions.length / 3} verts, volume ${vol.toFixed(6)}`);
