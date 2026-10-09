// An in-memory stand-in for the browser's OPFS, for the node tests of the wasm modules' OPFS mount:
// FileSystemDirectoryHandle / FileSystemFileHandle / FileSystemSyncAccessHandle with the same method
// names, synchronous I/O, exclusive handle locks and shared {mode: "read-only"} handles. Several
// module instances (the browser's workers) can mount one root.

const domError = (name, msg) => Object.assign(new Error(msg), {name});

export const calls = {
    read : 0,
    write : 0
};

export class MemFile {
    constructor() {
        this.bytes = new Uint8Array(0);
        this.size = 0;
        this.locked = false; // an exclusive (readwrite) handle is open
        this.readers = 0;    // shared read-only handles open
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
    constructor(file, readOnly = false) {
        this.file = file;
        this.open = true;
        this.readOnly = readOnly;
    }
    writable() {
        if (this.readOnly)
            throw domError("NoModificationAllowedError", "read-only handle");
        return this.live();
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
        const f = this.writable();
        f.ensure(at + view.byteLength);
        if (at > f.size)
            f.bytes.fill(0, f.size, at);
        f.bytes.set(view, at);
        f.size = Math.max(f.size, at + view.byteLength);
        return view.byteLength;
    }
    getSize() { return this.live().size; }
    truncate(n) {
        const f = this.writable();
        f.ensure(n);
        if (n > f.size)
            f.bytes.fill(0, f.size, n);
        f.size = n;
    }
    flush() { this.writable(); }
    close() {
        if (this.open) {
            if (this.readOnly)
                this.file.readers--;
            else
                this.file.locked = false;
        }
        this.open = false;
    }
}

class FileHandle {
    constructor(file) {
        this.file = file;
        this.kind = "file";
    }
    // {mode: "read-only"}: shared with other read-only handles; the default is exclusive.
    async createSyncAccessHandle(options) {
        const readOnly = options && options.mode === "read-only";
        if (this.file.locked || (!readOnly && this.file.readers > 0))
            throw domError("NoModificationAllowedError", "file is locked by another access handle");
        if (readOnly)
            this.file.readers++;
        else
            this.file.locked = true;
        return new SyncHandle(this.file, readOnly);
    }
}

export class DirHandle {
    constructor() {
        this.children = new Map();
        this.kind = "directory";
    }
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
            if (c instanceof MemFile ? c.locked || c.readers > 0 : c.locked())
                return true;
        return false;
    }
    async removeEntry(name, {recursive = false} = {}) {
        const c = this.children.get(name);
        if (!c)
            throw domError("NotFoundError", name);
        if (c instanceof DirHandle && c.children.size && !recursive)
            throw domError("InvalidModificationError", name);
        if (c instanceof MemFile ? c.locked || c.readers > 0 : c.locked())
            throw domError("NoModificationAllowedError", name);
        this.children.delete(name);
    }
    // [name, handle] like the real API: a FileSystemFileHandle for a file, the directory handle itself
    async * entries() {
        for (const [name, c] of this.children)
            yield [name, c instanceof MemFile ? new FileHandle(c) : c];
    }
}
