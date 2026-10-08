// Test the OPFS sync-access-handle backend (src/wasmio) under node, against an in-memory stand-in for
// the browser's OPFS: FileSystemDirectoryHandle / FileSystemFileHandle / FileSystemSyncAccessHandle
// with the same method names, synchronous I/O and exclusive handle locks.
//
// node has no OPFS, so the browser run (a dedicated worker, real OPFS) is not replaced by this; it
// pins the backend's own logic -- mount, named vs scratch files, the read cache and write coalescing,
// unlink/detach, pool exhaustion -- and that every verb gives the same bytes on the mount as in-heap.
//
// Usage: node tools/test_opfs_sync_wasm.mjs <module.js> [step-or-ifc fixture]
//   with adacpp_step_glb / adacpp_ifc_glb the fixture is also converted on the mount and in-heap.

import {createHash} from "node:crypto";
import {readFileSync} from "node:fs";
import {pathToFileURL} from "node:url";

const [modPath, fixture] = process.argv.slice(2);
if (!modPath) {
    console.error("usage: node test_opfs_sync_wasm.mjs <module.js> [fixture]");
    process.exit(2);
}

let failures = 0;
const check = (name, cond, detail) => {
    console.log(`  ${cond ? "ok  " : "FAIL"} ${name}${detail ? " - " + detail : ""}`);
    if (!cond)
        failures++;
};
const domError = (name, msg) => Object.assign(new Error(msg), {name});
const sha = (u8) => createHash("sha256").update(u8).digest("hex").slice(0, 16);

// --- in-memory OPFS ------------------------------------------------------------------------------

const calls = {
    read : 0,
    write : 0
};

class MemFile {
    constructor() {
        this.bytes = new Uint8Array(0);
        this.size = 0;
        this.locked = false;
    }
    ensure(n) {
        if (n > this.bytes.length) {
            const b = new Uint8Array(Math.max(n, this.bytes.length * 2));
            b.set(this.bytes.subarray(0, this.size));
            this.bytes = b;
        }
    }
}

class SyncHandle {
    constructor(file) {
        this.file = file;
        this.open = true;
    }
    live() {
        if (!this.open)
            throw domError("InvalidStateError", "handle closed");
        return this.file;
    }
    read(view, {at = 0} = {}) {
        calls.read++;
        const f = this.live();
        const n = Math.max(0, Math.min(view.byteLength, f.size - at));
        view.set(f.bytes.subarray(at, at + n));
        return n;
    }
    write(view, {at = 0} = {}) {
        calls.write++;
        const f = this.live();
        f.ensure(at + view.byteLength);
        if (at > f.size)
            f.bytes.fill(0, f.size, at);
        f.bytes.set(view, at);
        f.size = Math.max(f.size, at + view.byteLength);
        return view.byteLength;
    }
    getSize() { return this.live().size; }
    truncate(n) {
        const f = this.live();
        f.ensure(n);
        if (n > f.size)
            f.bytes.fill(0, f.size, n);
        f.size = n;
    }
    flush() { this.live(); }
    close() {
        if (this.open)
            this.file.locked = false;
        this.open = false;
    }
}

class FileHandle {
    constructor(file) { this.file = file; }
    async createSyncAccessHandle() {
        if (this.file.locked)
            throw domError("NoModificationAllowedError", "file is locked by another access handle");
        this.file.locked = true;
        return new SyncHandle(this.file);
    }
}

class DirHandle {
    constructor() { this.children = new Map(); }
    async getDirectoryHandle(name, {create = false} = {}) {
        let c = this.children.get(name);
        if (!c && create)
            this.children.set(name, (c = new DirHandle()));
        if (!(c instanceof DirHandle))
            throw domError(c ? "TypeMismatchError" : "NotFoundError", name);
        return c;
    }
    async getFileHandle(name, {create = false} = {}) {
        let c = this.children.get(name);
        if (!c && create)
            this.children.set(name, (c = new MemFile()));
        if (!(c instanceof MemFile))
            throw domError(c ? "TypeMismatchError" : "NotFoundError", name);
        return new FileHandle(c);
    }
    locked() {
        for (const c of this.children.values())
            if (c instanceof MemFile ? c.locked : c.locked())
                return true;
        return false;
    }
    async removeEntry(name, {recursive = false} = {}) {
        const c = this.children.get(name);
        if (!c)
            throw domError("NotFoundError", name);
        if (c instanceof DirHandle && c.children.size && !recursive)
            throw domError("InvalidModificationError", name);
        if (c instanceof MemFile ? c.locked : c.locked())
            throw domError("NoModificationAllowedError", name);
        this.children.delete(name);
    }
    async * entries() { yield* this.children.entries(); }
}

// --- the test ------------------------------------------------------------------------------------

const createMod = (await import(pathToFileURL(modPath).href)).default;
const M = await createMod();
const FS = M.FS;
const enc = new TextEncoder(), dec = new TextDecoder();
const root = new DirHandle();

console.log(`OPFS sync backend: ${modPath}`);

check("mountOpfs before opfsMount returns -1 (no trap later)", M.mountOpfs("/opfs") === -1);
let threw = null;
await M.opfsMount("/opfs").catch((e) => (threw = e));
check("opfsMount without OPFS (node) rejects with a reason", threw && /OPFS|sync access/.test(threw.message),
      threw && threw.message);

// An orphaned scratch pool from a dead worker is swept; a live (locked) one is not.
const pool = await root.getDirectoryHandle(".adacpp-scratch", {create : true});
await (await pool.getDirectoryHandle("dead", {create : true})).getFileHandle("s0", {create : true});
const live = await pool.getDirectoryHandle("live", {create : true});
const liveHandle = await (await live.getFileHandle("s0", {create : true})).createSyncAccessHandle();

await M.opfsMount("/opfs", {root, scratch : 4});
check("opfsMount with an OPFS root", M.mountOpfs("/opfs") === 0);
check("orphaned scratch pool swept, live one kept", !pool.children.has("dead") && pool.children.has("live"));
liveHandle.close();

// FS.writeFile replaces an existing file (WASMFS's own appended to it), in-heap and on the mount
for (const p of ["/w.txt", "/opfs/w.txt"]) {
    FS.writeFile(p, "a much longer first content");
    FS.writeFile(p, enc.encode("second"));
    check(`FS.writeFile replaces ${p}`, dec.decode(FS.readFile(p)) === "second");
    FS.unlink(p);
}

// scratch files: plain POSIX semantics through the module's FS
FS.writeFile("/opfs/t.bin", enc.encode("hello opfs"));
check("scratch write/read", dec.decode(FS.readFile("/opfs/t.bin")) === "hello opfs");
check("scratch stat size", FS.stat("/opfs/t.bin").size === 10);
check("scratch file is not visible by name in OPFS", !root.children.has("t.bin"));
let s = FS.open("/opfs/t.bin", "r+");
FS.write(s, enc.encode("OPFS"), 0, 4, 6);
FS.close(s);
check("write at offset", dec.decode(FS.readFile("/opfs/t.bin")) === "hello OPFS");
FS.truncate("/opfs/t.bin", 5);
check("truncate", dec.decode(FS.readFile("/opfs/t.bin")) === "hello");
s = FS.open("/opfs/t.bin", "r+");
FS.write(s, enc.encode("!"), 0, 1, 9); // a hole: bytes 5..8 read back as zeros
FS.close(s);
const holed = FS.readFile("/opfs/t.bin");
check("write past EOF zero-fills", holed.length === 10 && holed[7] === 0 && holed[9] === 33);
FS.mkdir("/opfs/sub");
FS.rename("/opfs/t.bin", "/opfs/sub/u.bin");
check("rename", FS.readFile("/opfs/sub/u.bin").length === 10);
FS.unlink("/opfs/sub/u.bin");
FS.rmdir("/opfs/sub");
check("unlink + rmdir", FS.readdir("/opfs").filter((n) => n !== "." && n !== "..").length === 0);

// coalescing + cache: 1 MiB written in 4 KiB pieces, read back in 100-byte pieces
const big = new Uint8Array(1 << 20);
for (let i = 0; i < big.length; i++)
    big[i] = (i * 2654435761) >>> 24;
calls.read = calls.write = 0;
s = FS.open("/opfs/c.bin", "w");
for (let off = 0; off < big.length; off += 4096)
    FS.write(s, big.subarray(off, off + 4096), 0, 4096, off);
FS.close(s);
const writes = calls.write;
s = FS.open("/opfs/c.bin", "r");
const piece = new Uint8Array(100);
let same = true;
for (let off = 0; off < big.length; off += 7919) {
    const n = FS.read(s, piece, 0, 100, off);
    same = same && n === Math.min(100, big.length - off) && piece.subarray(0, n).every((v, i) => v === big[off + i]);
}
FS.close(s);
check("coalesced writes", writes <= 2, `${writes} handle writes for 256 FS writes`);
check("cached small reads", same && calls.read <= 4, `${calls.read} handle reads for 133 FS reads`);
FS.unlink("/opfs/c.bin");

// named files: opfsOpen / opfsDetach / unlink
await M.opfsOpen("/opfs/dir/named.bin", {create : true});
FS.writeFile("/opfs/dir/named.bin", enc.encode("persisted"));
await M.opfsDetach("/opfs/dir/named.bin");
const named = (await root.getDirectoryHandle("dir")).children.get("named.bin");
check("opfsOpen(create) + opfsDetach leaves the named OPFS file, unlocked",
      named && dec.decode(named.bytes.subarray(0, named.size)) === "persisted" && !named.locked);
await M.opfsOpen("/opfs/dir/named.bin");
check("opfsOpen of an existing file reads it", dec.decode(FS.readFile("/opfs/dir/named.bin")) === "persisted");
threw = null;
await M.opfsOpen("/opfs/missing.bin").catch((e) => (threw = e));
check("opfsOpen of a missing file rejects", threw && threw.name === "NotFoundError");
FS.unlink("/opfs/dir/named.bin");
await M.opfsSettle();
check("FS.unlink of a named file removes it from OPFS",
      !(await root.getDirectoryHandle("dir")).children.has("named.bin"));

// pool exhaustion: an errno, then usable again once the pool is refilled
threw = null;
const made = [];
try {
    for (let i = 0; i < 16; i++) {
        FS.close(FS.open(`/opfs/p${i}`, "w"));
        made.push(`/opfs/p${i}`);
    }
} catch (e) {
    threw = e;
}
check("pool exhausted -> EIO from open, not a trap", threw && threw.errno === 29, `after ${made.length} files`);
made.forEach((p) => FS.unlink(p));
await M.opfsReserve(4);
FS.writeFile("/opfs/after.bin", enc.encode("ok"));
check("usable again after opfsReserve", dec.decode(FS.readFile("/opfs/after.bin")) === "ok");
FS.unlink("/opfs/after.bin");

// the module's own verb: same bytes on the mount as in-heap
const verb = M.stepToGlb ? "stepToGlb" : M.ifcToGlb ? "ifcToGlb" : null;
if (verb && fixture) {
    const src = readFileSync(fixture);
    const out = {};
    for (const base of ["/heap", "/opfs/run"]) {
        FS.mkdir(base);
        FS.mkdir(`${base}/spill`);
        FS.writeFile(`${base}/in`, src);
        const rc = M[verb](`${base}/in`, `${base}/out.glb`, `${base}/spill`, 2.0, 20.0, true);
        out[base] = {rc, sha : sha(FS.readFile(`${base}/out.glb`))};
    }
    check(`${verb}: GLB on the OPFS mount == in-heap`,
          out["/heap"].rc > 0 && JSON.stringify(out["/heap"]) === JSON.stringify(out["/opfs/run"]),
          JSON.stringify(out));
}

if (failures) {
    console.log(`${failures} FAILED`);
    process.exit(1);
}
console.log("OPFS sync backend OK");
