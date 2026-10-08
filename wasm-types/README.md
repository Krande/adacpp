# adacpp wasm modules

OCC-free, dependency-free WebAssembly builds of adacpp's native CAD conversion pipeline — no OCCT,
no pyodide, no Python. Each is an [embind](https://emscripten.org/docs/porting/connecting_cpp_and_javascript/embind.html)
module built with `-sMODULARIZE=1 -sEXPORT_ES6=1`, so the `.js` is an ES module whose **default
export** is an async factory returning the instantiated module.

These files are attached to each adacpp GitHub release (individually and as
`adacpp-wasm-<version>.zip`). The same artifacts also ship inside the
`ghcr.io/krande/adacpp-wasm-base:<version>` base image under `/out/wasm/`.

The `.d.ts` next to each `.js` is **generated at build time** by emscripten (`--emit-tsd`) directly
from the `EMSCRIPTEN_BINDINGS` in `src/cad/*_wasm.cpp` (and the OPFS API's JSDoc in
`src/wasmio/opfs_sync.js`), so it can never drift from the actual exports. embind carries no parameter names, so the generated types use positional names
(`_0, _1, …`) and no doc comments — the tables and the usage examples below are the human-readable
reference for what each argument means.

| Module (`.js` + `.wasm` + `.d.ts`) | Conversion | Entry points |
| --- | --- | --- |
| `adacpp_step_glb` | **STEP → GLB** (Part-21 tokenizer → libtess2 / `cdt` tessellator → glTF) | `stepToGlb` + [OPFS](#opfs) |
| `adacpp_ifc_glb` | **IFC → GLB** (pure-C++ IFC reader → libtess2 → glTF) | `ifcToGlb`, `scanMembers`, `clashJoints` + [OPFS](#opfs) |
| `adacpp_brep_writer` | **STEP → IFC** and **IFC → STEP** (B-rep) | `stepToIfc`, `ifcToStep` + [OPFS](#opfs) |
| `adacpp_glb_diff` | **GLB diff** (element-level, with removed-element overlay) | `diffGlb` |
| `adacpp_extrude` | **Prismatic extrusion** (section table + per-instance frames -> vertex/index buffers) | `expandBeamSolids` |
| `adacpp_fea` | **FEA results**: load combinations, derived components and envelopes over baked AFBL/AFEL strides | `combineField`, `envelopeField`, `readBlobHeader`, `version` + [OPFS](#opfs) |

The `*_glb`, `brep_writer` and `fea` modules do their file I/O through emscripten's WASMFS (`mod.FS`):
in-heap by default, and on an OPFS mount in a **dedicated Web Worker**, so multi-GB inputs stream
through `pread` (bounded RSS) instead of the wasm heap. `adacpp_glb_diff` and `adacpp_extrude` are
in-heap only.

## OPFS

```ts
// In a dedicated Web Worker (FileSystemSyncAccessHandle exists nowhere else).
await mod.opfsMount("/opfs");            // rejects with the reason if this browser cannot back it
await mod.opfsOpen("/opfs/in.stp");      // a file already in OPFS (written by the browser API)
await mod.opfsOpen("/opfs/out.glb", { create: true }); // an output that must outlive the module
mod.stepToGlb("/opfs/in.stp", "/opfs/out.glb", "/opfs/spill", 2.0, 20.0, true);
await mod.opfsDetach("/opfs/out.glb");   // flush + close; the OPFS file stays, unlocked
```

- Every byte of I/O is a synchronous `FileSystemSyncAccessHandle` call straight between the OPFS file
  and the wasm heap buffer of the `read`/`write`: no threads, no `SharedArrayBuffer`, no COOP/COEP, no
  JSPI. Chromium 108+, Firefox 111+, Safari 16.4+ (any browser with synchronous sync access handles).
- Files the module creates under the mount without an `opfsOpen` -- spill lanes, an output you
  only read back through `mod.FS` -- are backed by anonymous **scratch** files from a pool kept in
  `<OPFS root>/.adacpp-scratch/`. They live in OPFS, not the heap, but are not reachable by name
  through the browser's OPFS API, and are discarded on unlink (or swept by the next `opfsMount` once
  their worker is gone). The pool holds `scratch` idle files (`opfsMount(dir, { scratch: 32 })`, the
  default) and refills after each call; `await mod.opfsReserve(n)` raises it ahead of a call that
  creates many files. A call that runs out gets `EIO` from `open`.
- Writing through `mod.FS` onto the mount (e.g. streaming a `fetch` body in with `FS.open`/`FS.write`)
  is visible to the module immediately. A file written by the browser's own OPFS API is visible only
  after `opfsOpen`. Sync access handles are exclusive: while a file is attached, nothing else can
  open it; `opfsDetach` releases it, `FS.unlink` deletes it.
- `mountOpfs(dir)` is kept for old callers: it cannot mount (the setup is async), so it returns `0`
  only after `await opfsMount(dir)` and `-1` otherwise. Builds before this one returned `0` from
  `mountOpfs` and then trapped (`RuntimeError: unreachable`) on the first file operation on the
  mount: emscripten's own OPFS backend needs `-pthread` or JSPI.

## Usage

Keep each `.d.ts` next to its `.js` so TypeScript resolves the types on import.

```ts
import createAdacppStepGlb from "./adacpp_step_glb.js";

const mod = await createAdacppStepGlb({
  locateFile: (path) => `/wasm/${path}`, // resolve adacpp_step_glb.wasm
});

// In a dedicated Web Worker: back I/O with OPFS (see OPFS above).
await mod.opfsMount("/opfs");
mod.FS; // emscripten WASMFS handle, if you need to write the input yourself

// deflection=2.0, angularDeg=20.0 are the adapy production defaults; meshopt=true compresses.
const triangles = mod.stepToGlb("/opfs/in.stp", "/opfs/out.glb", "/opfs/spill", 2.0, 20.0, true);
if (triangles < 0) throw new Error("STEP→GLB failed (I/O error)");
```

Prismatic extrusion, via `adacpp_extrude` -- the leanest module here: its core is header-only, so
it links no tessellator, no mesh compressor and no CAD kernel. Give it a section table and one
frame per instance and it returns the buffers a renderer uploads:

```ts
import createAdacppExtrude from "./adacpp_extrude.js";

const m = await createAdacppExtrude({ locateFile: (p) => `/wasm/${p}` });

// sections: outline points in section coordinates, plus the triangle list for ONE sweep --
// it indexes 2n vertices, [0, n) the start ring and [n, 2n) the end ring, covering caps and walls.
const out = m.expandBeamSolids(
  [{ points: Float64Array, triangles: Uint32Array }],
  { label, section, node0, node1, origin, xvec, yvec, length },  // flat, one entry per instance
  points,                                                        // Float64Array, 3 per point
);
// out.positions Float32Array, out.indices Uint32Array, out.node0/node1 Uint32Array,
// out.t Float32Array (axial parameter, measured against the node line -- see below),
// out.rangeLabel / rangeTriStart / rangeTriCount Uint32Array (one row per instance)
```

The axial parameter is measured against the NODE positions, not the sweep axis. When an instance's
two ends carry different offsets the sweep frame tilts away from the node line, so the parameter
varies within a single ring and cannot be stored per section -- which is exactly why this module
exists rather than shipping the expanded buffers.

STEP ↔ IFC, via `adacpp_brep_writer`:

```ts
import createAdacppBrepWriter from "./adacpp_brep_writer.js";

const mod = await createAdacppBrepWriter({ locateFile: (p) => `/wasm/${p}` });
const solids   = mod.stepToIfc("/in.stp", "/out.ifc", "IFC4X3_ADD2", 2.0, 20.0, 0); // > 0 = ok
const products = mod.ifcToStep("/in.ifc", "/out.stp", 2.0, 20.0, 0);                // > 0 = ok
```

FEA load combinations, via `adacpp_fea` -- the same C++ the server's Python module (`adacpp.fea`)
runs, so a combination materialised in the browser is byte-identical to one the server writes. Inputs
are baked field blobs (AFBL nodal / AFEL per element type: a 1 KB header, then float32
`[steps x rows x components]`); each verb reads one stride per term and writes a new blob. Every verb
returns a JSON string, `{"ok": true, ...}` or `{"ok": false, "error": "..."}`; nothing throws.

```ts
import createAdacppFea from "./adacpp_fea.js";

const fea = await createAdacppFea({ locateFile: (p) => `/wasm/${p}` });
await fea.opfsMount("/opfs"); // in a dedicated Web Worker
await fea.opfsOpen("/opfs/base/fea.G-STRESS.QUAD4.elements.bin"); // blobs already in OPFS
// outputs that must persist by name: opfsOpen(path, { create: true }) them first, opfsDetach after

// A combination = sum(factor * stored case), float32, terms in file order. One path + step per term
// (the same base blob repeated is the usual case); factors as a Float32Array (exact float32).
const base = "/opfs/base/fea.G-STRESS.QUAD4.elements.bin";
const stats = JSON.parse(fea.combineField(
  JSON.stringify([base, base]),           // inPathsJson
  [12, 3],                                 // stepIdx: one per term, or a single number for all
  new Float32Array([1.2, 1.1]),            // factors
  "/opfs/cases/101-abcd1234/fea.G-STRESS.QUAD4.elements.bin",
  // Non-linear components are re-derived from the combined linear ones (columns by index):
  JSON.stringify([{ op: "plane_von_mises", args: [0, 1, 2], out: [3] }]),
));
// stats.steps[0].scalar_range_per_component[c] = [min, max]; stats.timing_ms = {read, combine, ...}

// A field that is entirely derived gets its own layout: P-STRESS from the combined G-STRESS.
fea.combineField(JSON.stringify([base, base]), [12, 3], new Float32Array([1.2, 1.1]), "/opfs/cases/.../p.bin",
  JSON.stringify({ name: "P-STRESS", n_components: 2,
                   ops: [{ op: "plane_principal", args: [0, 1, 2], out: [0, 1] }] }));

// Envelope over materialised cases: a 2-step blob (0 = max, 1 = min) + uint16 governing case indices
// as an AFGV sidecar: 16-byte header ("AFGV", uint32 version 1, uint32 n_cases, uint32 0), then
// little-endian uint16 [2 x rows x components], same step order.
fea.envelopeField(JSON.stringify(casePaths), 0, "/opfs/envelopes/G-STRESS.bin", "/opfs/envelopes/G-STRESS.gov");
```

Derivation ops: `copy`, `plane_von_mises` (3 args), `plane_principal` (3 args -> P1, P2),
`plane_principal_1` / `plane_principal_2` (P1 or P2 alone), `magnitude` (1-7 args), `magnitude3`,
`shell_decompose` (bottom SIGXX/SIGYY/TAUXY + top -> SIGMX, SIGMY, SIGBX, SIGBY, TAUMXY, TAUBXY,
MVONMISES). The single-output names are the ones a lazy-case manifest's `derived_components` uses,
so each entry maps onto one op: `{op, args: <component indices>, out: [<its column>]}`. An `out`
entry of `-1` drops that output.
