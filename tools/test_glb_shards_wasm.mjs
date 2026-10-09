// Test the sharded STEP/IFC -> GLB verbs (StepGlbShard / IfcGlbShard + mergeGlbLanes) under node.
//
// N module instances stand in for N browser workers: each has its own memory and file system, and the
// files a shared OPFS directory would hold (the index, huge-root parts, persisted lanes) are copied
// between their file systems. The whole protocol runs -- prepare on one, every shard opens the index,
// huge roots face-split across all of them, batches from planBatches, persist, merge on one -- and
//   1. the merged GLB carries the single-worker GLB's meshes (triangles + vertices per mesh),
//   2. a truncated or foreign index opens as rootCount() == -1 instead of trapping,
//   3. repeating the conversion on the same instances does not grow their memory (no leak per run;
//      a wasm memory never shrinks, so growth after the first runs is retained allocations),
//   4. the browser protocol itself -- Module.glbShards over N modules mounting ONE OPFS root (an
//      in-memory stand-in): shared read-only input, index and parts as named files, lanes copied to
//      the job directory, the merge on one worker -- writes the same GLB and leaves nothing behind.
//
// Usage: node tools/test_glb_shards_wasm.mjs <adacpp_step_glb.js | adacpp_ifc_glb.js> <model> [N]
//   (model: tools/gen_faceted_fixtures.py writes one with a 2560-face root that takes the face split)

import {readFileSync} from "node:fs";
import {pathToFileURL} from "node:url";

import {DirHandle} from "./opfs_memory.mjs";

const [modPath, model, nArg] = process.argv.slice(2);
if (!modPath || !model) {
    console.error("usage: node test_glb_shards_wasm.mjs <module.js> <model> [N]");
    process.exit(2);
}
const N = Number(nArg || 3);
const ifc = model.toLowerCase().endsWith(".ifc");
const DEFL = 2.0, ANG = 20.0;

// The modules don't export their heap: capture each instance's memory as it is instantiated.
const mems = [];
const instantiate = WebAssembly.instantiate.bind(WebAssembly);
WebAssembly.instantiate = async (...a) => {
    const r = await instantiate(...a);
    const m = (r.instance || r).exports?.memory;
    if (m)
        mems.push(m);
    return r;
};

let failures = 0;
const check = (name, cond, detail) => {
    console.log(`  ${cond ? "ok  " : "FAIL"} ${name}${detail ? " - " + detail : ""}`);
    if (!cond)
        failures++;
};

function meshes(glb) { // "name tris verts" per primitive, sorted
    const dv = new DataView(glb.buffer, glb.byteOffset, glb.byteLength);
    const n = dv.getUint32(12, true);
    const g = JSON.parse(new TextDecoder().decode(glb.subarray(20, 20 + n)));
    const out = [];
    for (const m of g.meshes)
        for (const p of m.primitives)
            out.push(`${m.name || ""} ${g.accessors[p.indices].count / 3} ${g.accessors[p.attributes.POSITION].count}`);
    return out.sort();
}
const ls = (M, dir) => M.FS.readdir(dir).filter((x) => x !== "." && x !== "..");
const move = (from, dir, to) => { // every file in `dir` of one instance to the same dir of another
    for (const n of ls(from, dir)) {
        to.FS.writeFile(`${dir}/${n}`, from.FS.readFile(`${dir}/${n}`));
        from.FS.unlink(`${dir}/${n}`);
    }
};

const create = (await import(pathToFileURL(modPath).href)).default;
const input = readFileSync(model);
const W = [];
for (let k = 0; k < N; ++k) {
    const M = await create();
    M.FS.writeFile("/in", input);
    for (const d of ["/lanes", "/parts"])
        M.FS.mkdir(d);
    W.push(M);
}
const memOf = W.map((_, k) => mems[mems.length - N + k]);
const Shard = (M) => (ifc ? M.IfcGlbShard : M.StepGlbShard);

// one whole sharded conversion; returns the merged GLB bytes
let hugeSeen = 0;
function convert() {
    const n = Shard(W[0]).prepare("/in", "/idx", N);
    const idx = W[0].FS.readFile("/idx");
    for (const M of W.slice(1))
        M.FS.writeFile("/idx", idx);
    const sh = W.map((M) => new (Shard(M))("/in", "/idx", DEFL, ANG));
    if (sh.some((s) => s.rootCount() !== n))
        throw new Error("a shard opened a different root count than prepare reported");
    // huge roots: face ranges round-robin over the shards, parts gathered on shard 0, assembled there
    hugeSeen = sh[0].hugeCount();
    for (let h = 0; h < sh[0].hugeCount(); ++h) {
        const nf = sh[0].hugeFaces(h), chunk = Math.max(64, Math.ceil(nf / (N * 4)));
        for (let f0 = 0, k = 0; f0 < nf; f0 += chunk, ++k)
            if (sh[k % N].processHuge(h, f0, Math.min(nf, f0 + chunk), "/parts") < 0)
                throw new Error("processHuge failed");
        for (const M of W.slice(1))
            move(M, "/parts", W[0]);
        if (sh[0].assembleHuge(h, chunk, "/parts", "/lanes", 0) < 0)
            throw new Error("assembleHuge failed");
    }
    const ends = sh[0].planBatches(N * 32);
    for (let b = 0, begin = 0; b < ends.size(); begin = ends.get(b), ++b)
        sh[b % N].process(begin, ends.get(b), "/lanes", b % N);
    ends.delete();
    sh.forEach((s) => s.persist());
    sh.forEach((s) => s.delete());
    for (const M of W.slice(1))
        move(M, "/lanes", W[0]);
    if (W[0].mergeGlbLanes("/lanes", N, "/out.glb", false) < 0)
        throw new Error("mergeGlbLanes failed");
    const glb = W[0].FS.readFile("/out.glb");
    W[0].FS.unlink("/out.glb");
    W.forEach((M) => M.FS.unlink("/idx"));
    return glb;
}

console.log(`GLB shards: ${modPath} ${model} N=${N}`);

// reference: the single-worker verb on its own instance
const R = await create();
R.FS.writeFile("/in", input);
R.FS.mkdir("/spill");
const ref = ifc ? R.ifcToGlb("/in", "/out.glb", "/spill", DEFL, ANG, false)
                : R.stepToGlb("/in", "/out.glb", "/spill", DEFL, ANG, false);
check("single-worker conversion", ref > 0, `returned ${ref}`);
const want = meshes(R.FS.readFile("/out.glb"));

const got = meshes(convert());
check("the model's big root takes the face split", hugeSeen > 0, `${hugeSeen} huge root(s)`);
check("sharded GLB has the single-worker meshes", JSON.stringify(got) === JSON.stringify(want),
      `${got.length} vs ${want.length} primitives`);
const s0 = new (Shard(W[0]))("/in", "/missing.idx", DEFL, ANG);
check("a missing index opens as rootCount() == -1", s0.rootCount() === -1);
s0.delete();

// corrupt indexes: a truncated one and one of random bytes must not trap or allocate wildly
Shard(W[0]).prepare("/in", "/good.idx", N);
const good = W[0].FS.readFile("/good.idx");
W[0].FS.writeFile("/bad.idx", good.subarray(0, Math.floor(good.length / 2)));
let s1 = new (Shard(W[0]))("/in", "/bad.idx", DEFL, ANG);
check("a truncated index opens as rootCount() == -1", s1.rootCount() === -1);
s1.delete();
const junk = new Uint8Array(4096);
for (let i = 0; i < junk.length; ++i)
    junk[i] = (i * 2654435761) >>> 24;
junk.set(good.subarray(0, 8)); // the right magic, then garbage lengths
W[0].FS.writeFile("/bad.idx", junk);
s1 = new (Shard(W[0]))("/in", "/bad.idx", DEFL, ANG);
check("a garbage index opens as rootCount() == -1", s1.rootCount() === -1);
s1.delete();
for (const f of ["/good.idx", "/bad.idx"])
    W[0].FS.unlink(f);

// leak check: memory after 2 more runs == memory after the first 2 (wasm memory never shrinks)
convert();
const before = memOf.map((m) => m.buffer.byteLength);
for (let r = 0; r < 3; ++r)
    convert();
const after = memOf.map((m) => m.buffer.byteLength);
check("repeated conversions do not grow any instance's memory", after.every((a, k) => a === before[k]),
      `${before.map((b) => b >> 20)} MB -> ${after.map((a) => a >> 20)} MB`);

// --- 4. the browser protocol: Module.glbShards over a shared OPFS root ------------------------------
{
    const root = new DirHandle();
    const ext = ifc ? "ifc" : "stp";
    const jobDir = await root.getDirectoryHandle("job", {create : true});
    const fh = await (await jobDir.getFileHandle(`in.${ext}`, {create : true})).createSyncAccessHandle();
    fh.write(input, {at : 0});
    fh.close();
    const P = [];
    for (let k = 0; k < N; ++k) {
        const M = await create();
        await M.opfsMount("/opfs", {root});
        P.push(M);
    }
    const job = "/opfs/job";
    await P[0].glbShards.prepare(job, `in.${ext}`, N);
    const infos = [];
    for (let k = 0; k < N; ++k)
        infos.push(await P[k].glbShards.open(job, `in.${ext}`, k));
    check("every worker opens the index", infos.every((i) => i.roots === infos[0].roots && i.roots >= 0),
          `${infos.map((i) => i.roots)}`);
    check("huge roots listed", infos[0].huge.length > 0, `${infos[0].huge}`);
    for (let h = 0; h < infos[0].huge.length; ++h) {
        const nf = infos[0].huge[h], chunk = Math.max(64, Math.ceil(nf / (N * 4)));
        for (let f0 = 0, k = 0; f0 < nf; f0 += chunk, ++k)
            await P[k % N].glbShards.processHuge(h, f0, Math.min(nf, f0 + chunk));
        await P[h % N].glbShards.assembleHuge(h, chunk);
    }
    const ends = infos[0].batches(N * 32);
    for (let b = 0, begin = 0; b < ends.length; begin = ends[b], ++b)
        P[b % N].glbShards.process(begin, ends[b]);
    for (const M of P)
        await M.glbShards.persist();
    const lanes = await P[0].glbShards.merge(N, false);
    check("merge over the shared OPFS directory", lanes > 0, `${lanes} lanes`);
    const out = jobDir.children.get("out.glb");
    const glb = out && out.bytes.subarray(0, out.size);
    check("browser protocol GLB has the single-worker meshes",
          !!glb && JSON.stringify(meshes(glb)) === JSON.stringify(want));
    for (const M of P)
        await M.opfsSettle();
    const left = (d) => [...(jobDir.children.get(d)?.children.keys() ?? [])];
    check("parts and lanes are consumed", left("parts").length === 0 && left("lanes").length === 0,
          `parts=${left("parts")} lanes=${left("lanes")}`);
    const inFile = jobDir.children.get(`in.${ext}`);
    check("the input is released by every worker", inFile.readers === 0 && !inFile.locked);
}

if (failures) {
    console.error(`${failures} check(s) failed`);
    process.exit(1);
}
console.log("GLB shards OK ✓");
