/**
 * JS half of the OPFS sync-access-handle WASMFS backend (opfs_sync_backend.cpp), linked into the
 * single-threaded embind modules with --js-library.
 *
 * Every byte of file I/O is a synchronous FileSystemSyncAccessHandle call (dedicated workers only)
 * reading into / writing from the wasm heap buffer the C++ passed. Acquiring a handle is the one async
 * step, so it happens before the C++ runs:
 *
 *   await Module.opfsMount("/opfs");                   // OPFS root at /opfs (throws if unusable)
 *   await Module.opfsOpen("/opfs/in.step");            // an existing OPFS file, by name
 *   await Module.opfsOpen("/opfs/out.glb", {create: true});
 *   Module.stepToGlb("/opfs/in.step", "/opfs/out.glb", "/opfs/spill", 2, 20, true);
 *   await Module.opfsDetach("/opfs/out.glb");          // flush + close; the OPFS file stays
 *
 * A file the C++ (or Module.FS) creates under the mount without an opfsOpen -- a spill lane, an output
 * nobody named -- is backed by an anonymous scratch file from a pool kept in
 * <root>/.adacpp-scratch/<instance>/. It lives in OPFS, not the heap, but it is not reachable by its
 * name through the browser's OPFS API and is discarded when unlinked (or swept at the next mount once
 * its worker is gone). Name a file with opfsOpen when it has to outlive the module.
 */
addToLibrary({
  $adacppOpfs: `{
    mountPoint: null,  // normalised mount path, e.g. "/opfs"
    root: null,        // FileSystemDirectoryHandle mapped to mountPoint
    scratchDir: null,  // FileSystemDirectoryHandle holding this instance's scratch pool
    files: [],         // id -> {handle, kind: "scratch"|"named", dir, name, path, detach}
    byPath: new Map(), // named path -> id
    free: [],          // ids of idle scratch handles
    nextScratch: 0,
    target: 0,         // scratch handles to keep idle
    pending: -1,       // handle id the next createFile takes (set around opfsOpen's attach)
    refill: null,      // in-flight refill promise
    removals: [],      // in-flight OPFS removals of unlinked named files
  }`,

  $adacppOpfsSplit: (path) => {
    var S = adacppOpfs;
    if (!S.mountPoint) throw new Error('OPFS is not mounted: call `await Module.opfsMount()` first');
    var p = String(path).replace(/\/+/g, '/').replace(/\/$/, '');
    if (!p.startsWith(S.mountPoint + '/')) throw new Error(`${path} is not under the OPFS mount ${S.mountPoint}`);
    var parts = p.slice(S.mountPoint.length + 1).split('/');
    if (parts.some((s) => !s || s === '.' || s === '..')) throw new Error(`bad OPFS path ${path}`);
    return {path: p, parts};
  },

  $adacppOpfsAddScratch__deps: ['$adacppOpfs'],
  $adacppOpfsAddScratch: async () => {
    var S = adacppOpfs;
    var name = 's' + S.nextScratch++;
    var fh = await S.scratchDir.getFileHandle(name, {create: true});
    var handle = await fh.createSyncAccessHandle();
    handle.truncate(0);
    var id = S.files.length;
    S.files.push({handle, kind: 'scratch', dir: S.scratchDir, name, path: null, detach: false});
    S.free.push(id);
  },

  $adacppOpfsFill__deps: ['$adacppOpfs', '$adacppOpfsAddScratch'],
  $adacppOpfsFill: () => {
    var S = adacppOpfs;
    if (!S.refill) {
      S.refill = (async () => {
        try {
          while (S.free.length < S.target) await adacppOpfsAddScratch();
        } finally {
          S.refill = null;
        }
      })();
    }
    return S.refill;
  },

  // ---- imports called by opfs_sync_backend.cpp (all synchronous) ----

  // The public API rides on this import's deps: the C++ references it, so it is always linked, and
  // naming a library symbol in EXPORTED_RUNTIME_METHODS alone does not pull it in (the .d.ts pass
  // would then fail on an undefined opfsMount).
  _adacpp_opfs_take__deps: ['$adacppOpfs', '$adacppOpfsFill', '$opfsMount', '$opfsOpen', '$opfsDetach',
                            '$opfsReserve', '$opfsSettle', '$mountOpfs'],
  _adacpp_opfs_take: () => {
    var S = adacppOpfs;
    if (S.pending >= 0) {
      var named = S.pending;
      S.pending = -1;
      return named;
    }
    var id = S.free.pop();
    if (id === undefined) {
      err(`adacpp OPFS: no idle scratch file for a new file under ${S.mountPoint} ` +
          `(pool of ${S.target}); raise it with \`await Module.opfsReserve(n)\` before the call`);
      return -1;
    }
    // Top the pool back up once the current synchronous call has returned to the event loop.
    queueMicrotask(() => adacppOpfsFill().catch((e) => err(`adacpp OPFS: scratch refill failed: ${e}`)));
    return id;
  },

  _adacpp_opfs_release__deps: ['$adacppOpfs'],
  _adacpp_opfs_release: (id) => {
    var S = adacppOpfs;
    var f = S.files[id];
    if (!f) return;
    try {
      if (f.kind === 'scratch') {
        f.handle.truncate(0);
        S.free.push(id);
        return;
      }
      f.handle.flush();
      f.handle.close();
    } catch (e) {
      err(`adacpp OPFS: releasing ${f.path || f.name}: ${e}`);
    }
    S.files[id] = null;
    S.byPath.delete(f.path);
    if (!f.detach) S.removals.push(f.dir.removeEntry(f.name).catch(() => {}));
  },

  _adacpp_opfs_read__deps: ['$adacppOpfs'],
  _adacpp_opfs_read: (id, buf, len, offset) => {
    try {
      buf >>>= 0;
      return adacppOpfs.files[id].handle.read(HEAPU8.subarray(buf, buf + (len >>> 0)), {at: offset});
    } catch (e) {
      err(`adacpp OPFS: read: ${e}`);
      return -{{{ cDefs.EIO }}};
    }
  },

  _adacpp_opfs_write__deps: ['$adacppOpfs'],
  _adacpp_opfs_write: (id, buf, len, offset) => {
    try {
      buf >>>= 0;
      return adacppOpfs.files[id].handle.write(HEAPU8.subarray(buf, buf + (len >>> 0)), {at: offset});
    } catch (e) {
      // QuotaExceededError lands here: the origin's storage quota is spent.
      err(`adacpp OPFS: write: ${e}`);
      return -(e && e.name === 'QuotaExceededError' ? {{{ cDefs.ENOSPC }}} : {{{ cDefs.EIO }}});
    }
  },

  _adacpp_opfs_size__deps: ['$adacppOpfs'],
  _adacpp_opfs_size: (id) => {
    try {
      return adacppOpfs.files[id].handle.getSize();
    } catch (e) {
      err(`adacpp OPFS: getSize: ${e}`);
      return -{{{ cDefs.EIO }}};
    }
  },

  _adacpp_opfs_truncate__deps: ['$adacppOpfs'],
  _adacpp_opfs_truncate: (id, size) => {
    try {
      adacppOpfs.files[id].handle.truncate(size);
      return 0;
    } catch (e) {
      err(`adacpp OPFS: truncate: ${e}`);
      return -(e && e.name === 'QuotaExceededError' ? {{{ cDefs.ENOSPC }}} : {{{ cDefs.EIO }}});
    }
  },

  _adacpp_opfs_flush__deps: ['$adacppOpfs'],
  _adacpp_opfs_flush: (id) => {
    try {
      adacppOpfs.files[id].handle.flush();
      return 0;
    } catch (e) {
      err(`adacpp OPFS: flush: ${e}`);
      return -{{{ cDefs.EIO }}};
    }
  },

  // ---- public API (EXPORTED_RUNTIME_METHODS) ----

  // The `__docs` JSDoc is what types these in the generated .d.ts. No default parameters in library
  // functions: emscripten 4.0.9 silently drops such a symbol.

  // Mount the Origin Private File System at `mountPoint` (default "/opfs"). Must run in a dedicated
  // Web Worker (sync access handles exist nowhere else). Rejects with an Error naming the reason when
  // this browser cannot back the mount -- the caller should then use in-heap paths instead.
  // Idempotent for the same mount point. options.scratch: idle scratch files kept ready for files the
  // module creates (default 32). options.root: the OPFS directory to map (default: the OPFS root).
  $opfsMount__docs: '/** @param {string=} mountPoint @param {{scratch: (number|undefined), root: (FileSystemDirectoryHandle|undefined)}=} options @return {Promise<void>} */',
  $opfsMount__deps: ['$adacppOpfs', '$adacppOpfsFill', '$adacppOpfsAddScratch'],
  $opfsMount: async (mountPoint, options) => {
    var S = adacppOpfs;
    if (mountPoint === undefined) mountPoint = '/opfs';
    options = options || {};
    var mp = ('/' + String(mountPoint)).replace(/\/+/g, '/').replace(/\/$/, '');
    if (S.mountPoint) {
      if (S.mountPoint === mp) return;
      throw new Error(`OPFS is already mounted at ${S.mountPoint}`);
    }
    if (!mp || mp === '/') throw new Error('the OPFS mount point must be a directory below /');
    var root = options.root;
    if (!root) {
      if (typeof FileSystemFileHandle === 'undefined' ||
          typeof FileSystemFileHandle.prototype.createSyncAccessHandle !== 'function') {
        throw new Error('OPFS sync access handles are unavailable here: they need a dedicated Web Worker ' +
                        'in a secure context, in a browser that implements them');
      }
      if (!globalThis.navigator || !navigator.storage || !navigator.storage.getDirectory) {
        throw new Error('navigator.storage.getDirectory() is unavailable (no OPFS in this context)');
      }
      root = await navigator.storage.getDirectory();
    }
    var pool = await root.getDirectoryHandle('.adacpp-scratch', {create: true});
    // Sweep pools left behind by workers that are gone. A live module's pool holds sync access
    // handles, which locks its files against removal, so this only deletes orphans.
    if (pool.entries) {
      for await (var [name] of pool.entries()) {
        await pool.removeEntry(name, {recursive: true}).catch(() => {});
      }
    }
    var instance = Date.now().toString(36) + '-' + Math.random().toString(36).slice(2, 10);
    S.scratchDir = await pool.getDirectoryHandle(instance, {create: true});
    S.root = root;
    S.mountPoint = mp;
    S.target = options.scratch === undefined ? 32 : Math.max(0, options.scratch | 0);
    try {
      // Acquire one handle up front so a browser that lists the API but refuses it (a private window,
      // a storage policy) fails here, loudly, and not on the first write.
      await adacppOpfsFill();
      if (S.target === 0) await adacppOpfsAddScratch();
      // Browsers that predate the 2023 spec fix return Promises from getSize/truncate/flush/close
      // (Chromium < 108); every call the backend makes has to be synchronous.
      if (typeof S.files[S.free[0]].handle.getSize() !== 'number') {
        throw new Error('this browser implements an old, asynchronous FileSystemSyncAccessHandle');
      }
      var rc = Module['_adacppOpfsMount'](mp);
      if (rc !== 0) throw new Error(`creating the mount directory ${mp} failed (errno ${-rc})`);
    } catch (e) {
      for (var f of S.files) if (f) try { f.handle.close(); } catch (_) {}
      S.files = []; S.free = []; S.mountPoint = null; S.root = null;
      await pool.removeEntry(instance, {recursive: true}).catch(() => {});
      throw e;
    }
  },

  // Back `path` (under the mount) with the OPFS file of the same name, so the module reads it -- or,
  // with options.create, writes it -- in place. Files written by the browser's own OPFS API are only
  // visible to the module after this. A path that is already attached is left as it is.
  $opfsOpen__docs: '/** @param {string} path @param {{create: (boolean|undefined)}=} options @return {Promise<void>} */',
  $opfsOpen__deps: ['$adacppOpfs', '$adacppOpfsSplit', '$FS'],
  $opfsOpen: async (path, options) => {
    var S = adacppOpfs;
    options = options || {};
    var {path: p, parts} = adacppOpfsSplit(path);
    if (S.byPath.has(p)) return;
    var create = !!options.create;
    var dir = S.root;
    for (var i = 0; i < parts.length - 1; i++) dir = await dir.getDirectoryHandle(parts[i], {create});
    var name = parts[parts.length - 1];
    var fh = await dir.getFileHandle(name, {create});
    var handle = await fh.createSyncAccessHandle();
    var id = S.files.length;
    S.files.push({handle, kind: 'named', dir, name, path: p, detach: false});
    // A file the module created at this path in the meantime (scratch-backed) gives way to the named one.
    try { FS.unlink(p); } catch (_) {}
    S.pending = id;
    var rc;
    try {
      rc = Module['_adacppOpfsAttach'](p);
    } finally {
      S.pending = -1;
    }
    if (rc !== 0) {
      handle.close();
      S.files[id] = null;
      throw new Error(`attaching ${p} failed (errno ${-rc})`);
    }
    S.byPath.set(p, id);
  },

  // Flush and close the handle behind a path attached with opfsOpen, and drop it from the module's FS.
  // The OPFS file stays (and is unlocked for other readers). Unlinking it through Module.FS instead
  // deletes the OPFS file.
  $opfsDetach__docs: '/** @param {string} path @return {Promise<void>} */',
  $opfsDetach__deps: ['$adacppOpfs', '$adacppOpfsSplit', '$FS'],
  $opfsDetach: async (path) => {
    var S = adacppOpfs;
    var {path: p} = adacppOpfsSplit(path);
    var id = S.byPath.get(p);
    if (id === undefined) throw new Error(`${p} is not attached (opfsOpen)`);
    S.files[id].detach = true;
    FS.unlink(p);  // last reference -> _adacpp_opfs_release closes the handle
  },

  // Keep at least `n` idle scratch files ready (the pool refills to this after every call that used
  // it). Each file the module creates without an opfsOpen takes one; a call that would need more than
  // are idle fails with EIO.
  $opfsReserve__docs: '/** @param {number} n @return {Promise<void>} */',
  $opfsReserve__deps: ['$adacppOpfs', '$adacppOpfsFill'],
  $opfsReserve: async (n) => {
    var S = adacppOpfs;
    if (!S.mountPoint) throw new Error('OPFS is not mounted: call `await Module.opfsMount()` first');
    S.target = Math.max(S.target, n | 0);
    await adacppOpfsFill();
  },

  // Wait for pending OPFS work: scratch refills and the removal of unlinked named files.
  $opfsSettle__docs: '/** @return {Promise<void>} */',
  $opfsSettle__deps: ['$adacppOpfs'],
  $opfsSettle: async () => {
    var S = adacppOpfs;
    if (S.refill) await S.refill;
    var r = S.removals.splice(0);
    await Promise.all(r);
  },

  // Synchronous mount check kept for callers of the old API. An OPFS mount needs async setup, so this
  // cannot mount anything: it returns 0 when `await opfsMount(mountPoint)` has already completed, and
  // -1 otherwise (after logging why), so the caller takes its in-heap path instead of trapping later.
  $mountOpfs__docs: '/** @param {string} mountPoint @return {number} */',
  $mountOpfs__deps: ['$adacppOpfs'],
  $mountOpfs: (mountPoint) => {
    var mp = ('/' + String(mountPoint)).replace(/\/+/g, '/').replace(/\/$/, '');
    if (adacppOpfs.mountPoint && adacppOpfs.mountPoint === mp) return 0;
    err(`mountOpfs(${mountPoint}): OPFS is mounted asynchronously now -- ` +
        `\`await Module.opfsMount(${JSON.stringify(mp)})\` in a dedicated worker first`);
    return -1;
  },
});
