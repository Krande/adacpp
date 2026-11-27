// Production-like test of the pyodide wheel: install the REAL artifact under node-pyodide and
// exercise the surface the browser actually uses.
//
// WHY THIS EXISTS, AND WHY IT HAS TEETH
//
// The wasm Python surface was never imported by CI. `ci-wasm-tests.yaml` runs `pixi run wbuild`,
// which sets BUILD_PYTHON=OFF — it cross-compiles OCCT for ~30 min and then builds a
// `double multiply(a,b)` stub. It cannot, even in principle, catch a bug in the extension.
//
// This file was the only thing that imports the wheel, and it had been dead for months: npm pinned
// pyodide 0.27.7 (emscripten-3.1.58) while the wheel is tagged for 0.29.4, so micropip rejected it
// before adacpp code ran. It also had no assertions — it printed a result and exited 0 regardless.
//
// Both gaps together are why `glb_diff = _cad.glb_diff` shipped: an AttributeError at *import* under
// pyodide that took the whole adacpp package down, and with it ada.cad.select_backend().
//
// So the rules here: assert, exit nonzero, and test the real artifact.
//
// A wasm stack overflow is FATAL — it poisons the pyodide instance ("Pyodide already fatally failed
// and can no longer be used"). Sharing one instance turns the first failure into a cascade of fake
// ones, so each group gets a fresh instance.
//
// Run: pixi run -e wasm test-wheel-pyodide   [path/to/wheel]

const fs = require("fs");
const path = require("path");

const REPO = path.resolve(__dirname, "..");
const FIXTURE = path.join(REPO, "files", "flat_plate_abaqus_10x10_m_wColors.stp");
const GENERATED = path.join(REPO, "src", "adacpp", "cad", "__init__.py");

let failures = 0;
const check = (name, cond, detail) => {
    if (cond) {
        console.log(`  ok   ${name}${detail ? " — " + detail : ""}`);
    } else {
        failures++;
        console.log(`  FAIL ${name}${detail ? " — " + detail : ""}`);
    }
};

function wheelPath() {
    const given = process.argv[2];
    if (given)
        return given;
    const dist = path.join(REPO, "dist");
    const w = fs.readdirSync(dist).filter((f) => f.startsWith("ada_cpp-") && f.endsWith(".whl"));
    if (!w.length) {
        console.error("no wheel in dist/ — run `pixi run -e wasm pack-wheel-pyodide` first");
        process.exit(1);
    }
    return path.join(dist, w[0]);
}

// The surface the NATIVE build publishes. adacpp/cad/__init__.py is generated with a static __all__,
// so it is the same list on both targets BY CONSTRUCTION — which makes it the reference the wasm
// build must reproduce at runtime. This is the invariant glb_diff broke.
function expectedAll() {
    const src = fs.readFileSync(GENERATED, "utf8");
    const block = src.match(/^__all__ = \[([\s\S]*?)^\]/m);
    if (!block)
        throw new Error(`could not parse __all__ out of ${GENERATED}`);
    return block[1].match(/"([^"]+)"/g).map((s) => s.slice(1, -1));
}

async function fresh(wheel) {
    const {loadPyodide} = require(require.resolve("pyodide"));
    const pyodide = await loadPyodide();
    pyodide.FS.mkdirTree("/dist");
    pyodide.FS.writeFile("/dist/" + path.basename(wheel), fs.readFileSync(wheel));
    await pyodide.loadPackage([ "micropip" ]);
    // ADACPP_PYODIDE_PREINSTALL: wheels (path-separated) installed BEFORE adacpp. The shared-OCCT
    // spike (ADACPP_WASM_SHARED_OCCT=ON) needs the ada_occt wheel first: adacpp's extension NEEDS
    // libocct.so and Pyodide resolves it against the copy already loaded.
    for (const pre of (process.env.ADACPP_PYODIDE_PREINSTALL || "").split(path.delimiter).filter(Boolean)) {
        pyodide.FS.writeFile("/dist/" + path.basename(pre), fs.readFileSync(pre));
        await pyodide.pyimport("micropip").install("emfs:/dist/" + path.basename(pre), {deps : false});
    }
    await pyodide.pyimport("micropip").install("emfs:/dist/" + path.basename(wheel));
    pyodide.FS.writeFile("/fixture.stp", fs.readFileSync(FIXTURE));
    return pyodide;
}

// Run python that returns JSON. Any wasm-fatal error is reported against `name` rather than taking
// the whole harness down, so one broken entry point cannot mask the rest of the surface.
function pyjson(pyodide, name, code) {
    try {
        return JSON.parse(pyodide.runPython(`import json\njson.dumps(${code})`));
    } catch (e) {
        failures++;
        console.log(`  FAIL ${name} — ${String(e).split("\n")[0].slice(0, 120)}`);
        return null;
    }
}

async function main() {
    const wheel = wheelPath();
    console.log("wheel:", path.basename(wheel));

    // ---- group 1: the import must survive, and the surface must not depend on the build target ----
    console.log("\n[surface] import + re-export parity");
    {
        const py = await fresh(wheel);
        console.log("  pyodide:", py.runPython("import sys; sys.version").split(" ")[0]);

        // If this throws, it is the exact glb_diff regression: an unguarded re-export of a binding the
        // wasm build does not register.
        const imported = pyjson(py, "import adacpp + adacpp.cad", "(__import__('adacpp') and 'adacpp' or '') and 'ok'");
        check("import adacpp / adacpp.cad succeeds", imported === "ok");

        const want = expectedAll();
        const got = pyjson(py, "read __all__", "__import__('adacpp.cad', fromlist=['x']).__all__");
        if (got) {
            const missing = want.filter((n) => !got.includes(n));
            const extra = got.filter((n) => !want.includes(n));
            check("__all__ matches the generated (native) surface", missing.length === 0 && extra.length === 0,
                  missing.length || extra.length ? `missing=${JSON.stringify(missing)} extra=${JSON.stringify(extra)}`
                                                 : `${got.length} names`);
        }

        // The generalisation of the glb_diff bug: EVERY advertised name must actually resolve.
        const unresolvable = pyjson(
            py, "resolve every __all__ name",
            "[n for n in __import__('adacpp.cad', fromlist=['x']).__all__ if not hasattr(__import__('adacpp.cad', fromlist=['x']), n)]");
        if (unresolvable)
            check("every name in __all__ resolves", unresolvable.length === 0, JSON.stringify(unresolvable));
    }

    // ---- group 2: bindings compiled out of wasm degrade at the CALL, with a reason ----
    console.log("\n[optional] native-only bindings degrade gracefully");
    {
        const py = await fresh(wheel);
        py.runPython(`
import adacpp.cad as cad

def probe():
    if not hasattr(cad, "glb_diff"):
        return {"bound": False, "raised": None, "reason": None}
    try:
        cad.glb_diff("/a.glb", "/b.glb")
        return {"bound": True, "raised": "NO RAISE", "reason": None}
    except NotImplementedError as e:
        return {"bound": True, "raised": "NotImplementedError", "reason": str(e)}
    except Exception as e:
        return {"bound": True, "raised": type(e).__name__, "reason": str(e)}
`);
        const p = pyjson(py, "glb_diff probe", "probe()");
        if (p) {
            check("glb_diff stays bound on wasm (no import-time AttributeError)", p.bound === true);
            check("calling it raises NotImplementedError, not AttributeError", p.raised === "NotImplementedError",
                  p.raised);
            check("the failure names a reason the caller can act on", !!p.reason && p.reason.length > 30, p.reason);
        }
    }

    // ---- group 3: the track vocabulary adacpp declares must be usable on THIS build ----
    console.log("\n[tracks] tess_tracks() is a build-accurate, usable vocabulary");
    {
        const py = await fresh(wheel);
        const tracks =
            pyjson(py, "tess_tracks()", "[t['name'] for t in __import__('adacpp.cad', fromlist=['x']).tess_tracks()]");
        if (tracks) {
            check("tess_tracks() is non-empty", tracks.length > 0, JSON.stringify(tracks));
            // The taxonomy tracks (occ/cgal/hybrid) need ifcopenshell/OCCT/CGAL and are compiled out of
            // wasm on purpose — so this list is SHORTER here than natively. That is correct, and it is why
            // adapy must discover the vocabulary rather than hardcode it.
            check("wasm does not advertise the taxonomy tracks it cannot run",
                  !tracks.some((t) => ["occ", "cgal", "hybrid"].includes(t)), JSON.stringify(tracks));
            check("the OCC-free libtess2 track is available in the browser", tracks.includes("libtess2"));
        }
    }

    // ---- group 4: the actual production conversion ----
    console.log("\n[convert] STEP -> GLB through the real wheel");
    {
        const py = await fresh(wheel);
        py.runPython(`
import adacpp.cad as cad, os

def convert():
    tris = cad.stream_step_to_glb("/fixture.stp", "/out.glb", 0.0, 20.0, 1, True, 0.0, False)
    with open("/out.glb", "rb") as fh:
        head = fh.read(4)
    return {"tris": tris, "bytes": os.path.getsize("/out.glb"), "magic": head.decode("ascii", "replace")}
`);
        const c = pyjson(py, "stream_step_to_glb", "convert()");
        if (c) {
            check("stream_step_to_glb produces triangles", c.tris > 0, `tris=${c.tris}`);
            check("the output is a real GLB", c.magic === "glTF", `magic=${JSON.stringify(c.magic)} bytes=${c.bytes}`);
        }
    }

    // ---- group 5: the mesh buffers, WITHOUT loading numpy by hand ----
    // The wheel must declare its own runtime deps. Mesh.positions/.indices are zero-copy
    // nb::ndarray<nb::numpy> views, so numpy is a hard requirement; the wheel used to declare NO
    // dependencies at all, and micropip installs exactly what is declared. The failure mode is not an
    // ImportError — reading Mesh.indices recursed until the interpreter died with
    // "Maximum call stack size exceeded", which reads like a kernel bug and is not one.
    //
    // So this group deliberately does NOT loadPackage(["numpy"]): a green result here means micropip
    // pulled numpy in via Requires-Dist, which is what a real browser consumer gets. Loading numpy by
    // hand would paper over exactly the bug this catches.
    console.log("\n[mesh] zero-copy buffers resolve without hand-loading numpy");
    {
        const py = await fresh(wheel);
        py.runPython(`
import adacpp.cad as cad, numpy as np

def mesh():
    m = cad.tessellate(cad.make_box(2.0, 3.0, 4.0), -1.0)
    idx, pos = np.asarray(m.indices), np.asarray(m.positions)
    return {"tris": len(idx) // 3, "verts": len(pos) // 3, "dtype": str(idx.dtype)}
`);
        const m = pyjson(py, "mesh buffer access", "mesh()");
        if (m) {
            // A unit box is 12 triangles on every target. Pinning the number (not just ">0") makes this a
            // parity check against the native build, not just a smoke test.
            check("tessellate + read buffers gives the native answer", m.tris === 12,
                  `tris=${m.tris} verts=${m.verts} dtype=${m.dtype}`);
            check("indices are uint32 as natively", m.dtype === "uint32", m.dtype);
        }
    }

    // ---- group 6: the FEA kernels are OCC-free, so the wheel carries them too ----
    console.log("\n[fea] adacpp.fea combines like the native build");
    {
        const py = await fresh(wheel);
        py.runPython(`
import adacpp.fea as fea, numpy as np

def combine():
    x = np.array([1.0 + 2.0**-12, 3.0, -2.5], dtype=np.float32)
    out = fea.combine_strides([x, x], [1.0 + 2.0**-12, -1.0])
    # a*a rounds to float32 before the add (no FMA): exactly 2**-12 in the first slot.
    return {"first": float(out[0]), "want": 2.0**-12, "dtype": str(out.dtype)}
`);
        const r = pyjson(py, "fea.combine_strides", "combine()");
        if (r) {
            check("fea.combine_strides gives the unfused float32 result", r.first === r.want && r.dtype === "float32",
                  JSON.stringify(r));
        }
    }

    // ---- group 7: planar CDT (detria) — what param_models uses instead of `triangle` in the browser ----
    console.log("\n[cdt] cad.triangulate_polygon matches the native contract");
    {
        const py = await fresh(wheel);
        py.runPython(`
import adacpp.cad as cad, numpy as np

def cdt():
    outer = [(0, 0), (10, 0), (10, 10), (0, 10)]
    hole = [(4, 4), (6, 4), (6, 6), (4, 6)]
    v, t = cad.triangulate_polygon([outer, hole])
    a, b, c = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
    areas = 0.5 * ((b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0]))
    try:
        cad.triangulate_polygon([[(0, 0), (1, 1), (1, 0), (0, 1)]])
        raised = "NO RAISE"
    except ValueError:
        raised = "ValueError"
    except Exception as e:
        raised = type(e).__name__
    return {"verts": len(v), "tris": len(t), "area": float(areas.sum()), "ccw": bool((areas > 0).all()),
            "same_verts": bool((v == np.asarray(outer + hole, dtype=float)).all()), "dtype": str(t.dtype),
            "raised": raised}
`);
        const r = pyjson(py, "triangulate_polygon", "cdt()");
        if (r) {
            check("square with hole: input verts kept, 8 CCW tris, area 96",
                  r.verts === 8 && r.tris === 8 && r.same_verts && r.ccw && r.area === 96 && r.dtype === "uint32",
                  JSON.stringify(r));
            check("a self-intersecting outline raises ValueError (no wasm trap)", r.raised === "ValueError", r.raised);
        }
    }

    console.log(failures === 0 ? "\nPYODIDE WHEEL OK ✓" : `\n${failures} CHECK(S) FAILED ✗`);
    process.exit(failures === 0 ? 0 : 1);
}

main().catch((err) => {
    console.error(err);
    process.exit(1);
});
