# adacpp wasm modules

OCC-free, dependency-free WebAssembly builds of adacpp's native CAD conversion pipeline — no OCCT,
no pyodide, no Python. Each is an [embind](https://emscripten.org/docs/porting/connecting_cpp_and_javascript/embind.html)
module built with `-sMODULARIZE=1 -sEXPORT_ES6=1`, so the `.js` is an ES module whose **default
export** is an async factory returning the instantiated module.

These files are attached to each adacpp GitHub release (individually and as
`adacpp-wasm-<version>.zip`). The same artifacts also ship inside the
`ghcr.io/krande/adacpp-wasm-base:<version>` base image under `/out/wasm/`.

The `.d.ts` next to each `.js` is **generated at build time** by emscripten (`--emit-tsd`) directly
from the `EMSCRIPTEN_BINDINGS` in `src/cad/*_wasm.cpp`, so it can never drift from the actual
exports. embind carries no parameter names, so the generated types use positional names
(`_0, _1, …`) and no doc comments — the tables and the usage examples below are the human-readable
reference for what each argument means.

| Module (`.js` + `.wasm` + `.d.ts`) | Conversion | Entry points |
| --- | --- | --- |
| `adacpp_step_glb` | **STEP → GLB** (Part-21 tokenizer → libtess2 / `cdt` tessellator → glTF) | `stepToGlb`, `mountOpfs` |
| `adacpp_ifc_glb` | **IFC → GLB** (pure-C++ IFC reader → libtess2 → glTF) | `ifcToGlb`, `mountOpfs` |
| `adacpp_brep_writer` | **STEP → IFC** and **IFC → STEP** (B-rep) | `stepToIfc`, `ifcToStep`, `mountOpfs` |
| `adacpp_glb_diff` | **GLB diff** (element-level, with removed-element overlay) | `diffGlb` |

The `*_glb` and `brep_writer` modules are built with `-sWASMFS=1` and expose `mountOpfs(dir)`: call
it from a **Web Worker** to back file I/O with OPFS so multi-GB inputs stream through `pread`
(bounded RSS) instead of the wasm heap. `adacpp_glb_diff` is in-heap (no OPFS).

## Usage

Keep each `.d.ts` next to its `.js` so TypeScript resolves the types on import.

```ts
import createAdacppStepGlb from "./adacpp_step_glb.js";

const mod = await createAdacppStepGlb({
  locateFile: (path) => `/wasm/${path}`, // resolve adacpp_step_glb.wasm
});

// In a Web Worker: back I/O with OPFS (0 = success).
mod.mountOpfs("/opfs");
mod.FS; // emscripten WASMFS handle, if you need to write the input yourself

// deflection=2.0, angularDeg=20.0 are the adapy production defaults; meshopt=true compresses.
const triangles = mod.stepToGlb("/opfs/in.stp", "/opfs/out.glb", "/opfs/spill", 2.0, 20.0, true);
if (triangles < 0) throw new Error("STEP→GLB failed (I/O error)");
```

STEP ↔ IFC, via `adacpp_brep_writer`:

```ts
import createAdacppBrepWriter from "./adacpp_brep_writer.js";

const mod = await createAdacppBrepWriter({ locateFile: (p) => `/wasm/${p}` });
const solids   = mod.stepToIfc("/in.stp", "/out.ifc", "IFC4X3_ADD2", 2.0, 20.0, 0); // > 0 = ok
const products = mod.ifcToStep("/in.ifc", "/out.stp", 2.0, 20.0, 0);                // > 0 = ok
```
