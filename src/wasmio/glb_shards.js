/**
 * The browser side of the sharded STEP/IFC -> GLB conversion (step_glb_shard.h / ifc_glb_shard.h):
 * one conversion split across N dedicated Web Workers, each with its OWN module instance, sharing
 * nothing but an OPFS directory -- so no SharedArrayBuffer and no cross-origin isolation.
 *
 * The C++ shard verbs read and write files; this is what makes those files visible from one worker
 * to the next. Everything shared lives under a job directory on the OPFS mount:
 *
 *   <job>/<input>       the model, opened by every worker with a SHARED read-only handle
 *   <job>/index         prepare()'s scan + metadata + LPT order, written once, read by every worker
 *   <job>/parts/        huge-root face-range parts (named OPFS files, so any worker can join them)
 *   <job>/lanes/        each worker's spill lane, copied there by persist() for the merge
 *   <job>/out.glb       the merged GLB (merge()), read by the page through the OPFS API
 *
 * A worker's own spill lane is written to its scratch pool first (the C++ names those files itself, and
 * a named OPFS file can only be created asynchronously); persist() copies it to <job>/lanes.
 *
 *   await Module.opfsMount('/opfs');
 *   const s = Module.glbShards;
 *   if (k === 0) await s.prepare(job, 'in.step', N);          // once, before the others open
 *   const info = await s.open(job, 'in.step', k, {deflection, angular});
 *   await s.processHuge(h, f0, f1); ... s.assembleHuge(h, chunk) on one worker per huge root
 *   s.process(begin, end); ...                                 // batches from info.batches(n)
 *   await s.persist();
 *   if (k === 0) await s.merge(N);                             // after every worker persisted
 */
addToLibrary({
  $glbShards__deps: ['$adacppOpfs', '$opfsOpen', '$opfsDetach', '$opfsReserve', '$FS'],
  $glbShards: {
    shard: null,   // this worker's StepGlbShard / IfcGlbShard
    job: null,     // the job directory (under the mount)
    src: null,     // the input path (attached read-only while the shard is open)
    lane: -1,      // this worker's lane index
    local: null,   // this worker's own lane directory (scratch-backed)

    // Scratch files a shard worker keeps ready: every spilled material holds two open (positions +
    // indices), and a conversion can spill a few hundred materials per lane.
    SCRATCH: 512,

    cls: (input) => (/\.ifc$/i.test(input) ? Module['IfcGlbShard'] : Module['StepGlbShard']),
    mkdirs: (path) => {
      var cur = '';
      for (var part of path.split('/').filter(Boolean)) {
        cur += '/' + part;
        try { FS.mkdir(cur); } catch (_) {}
      }
    },

    // Scan the input once and write <job>/index for `nworkers` workers. Run on ONE worker, before any
    // worker calls open(). Returns the ordinary root count (huge roots are listed apart), -1 on error.
    prepare: async (job, input, nworkers) => {
      var G = glbShards;
      var src = `${job}/${input}`, idx = `${job}/index`;
      await opfsOpen(src, {readOnly: true});
      try {
        await opfsOpen(idx, {create: true});
        var n;
        try {
          n = G.cls(input).prepare(src, idx, nworkers | 0);
        } finally {
          await opfsDetach(idx);
        }
        return n;
      } finally {
        await opfsDetach(src);
      }
    },

    // Open this worker's shard (lane `lane`, unique per worker) from <job>/index. Returns
    // {roots, huge: [face counts], batches(n) -> [ends]}; roots is -1 when the index can't be read.
    open: async (job, input, lane, options) => {
      var G = glbShards;
      options = options || {};
      await opfsReserve(G.SCRATCH);
      var src = `${job}/${input}`, idx = `${job}/index`;
      await opfsOpen(src, {readOnly: true});
      await opfsOpen(idx, {readOnly: true});
      var Shard = G.cls(input);
      var defl = options.deflection === undefined ? 2.0 : options.deflection;
      var ang = options.angular === undefined ? 20.0 : options.angular;
      var s = new Shard(src, idx, defl, ang);
      await opfsDetach(idx); // everything the shard needs from it is in memory now
      G.shard = s; G.job = job; G.src = src; G.lane = lane | 0;
      G.local = `${job}/.lane${G.lane}`; // this module's own (scratch-backed) files, never shared
      G.mkdirs(G.local);
      G.mkdirs(`${job}/parts`);
      var huge = [];
      for (var h = 0; h < s.hugeCount(); h++) huge.push(s.hugeFaces(h));
      return {
        roots: s.rootCount(),
        huge,
        batches: (n) => {
          var v = s.planBatches(n | 0), ends = [];
          for (var i = 0; i < v.size(); i++) ends.push(v.get(i));
          v.delete();
          return ends;
        },
      };
    },

    // Ordinary roots [begin, end) into this worker's lane. Returns what the verb returns.
    process: (begin, end) => {
      var G = glbShards;
      return G.shard.process(begin, end, G.local, G.lane);
    },

    // Faces [f0, f1) of huge root `h` into <job>/parts (a named file another worker can join).
    processHuge: async (h, f0, f1) => {
      var G = glbShards;
      var part = `${G.job}/parts/huge${h}_${f0}.part`;
      await opfsOpen(part, {create: true});
      try {
        return G.shard.processHuge(h, f0, f1, `${G.job}/parts`);
      } finally {
        await opfsDetach(part);
      }
    },

    // Join huge root `h`'s parts (ranges of `chunk` faces) into this worker's lane. Run once per huge
    // root, after every one of its ranges is done. The parts are deleted.
    assembleHuge: async (h, chunk) => {
      var G = glbShards;
      var nf = G.shard.hugeFaces(h);
      for (var f0 = 0; f0 < nf; f0 += chunk) await opfsOpen(`${G.job}/parts/huge${h}_${f0}.part`);
      return G.shard.assembleHuge(h, chunk, `${G.job}/parts`, G.local, G.lane);
    },

    // Finish this worker: persist its lane and copy it to <job>/lanes for the merge. Returns false
    // when this worker got no work (nothing to copy).
    persist: async () => {
      var G = glbShards;
      var ok = G.shard.persist();
      G.shard.delete();
      G.shard = null;
      if (ok) {
        var shared = `${G.job}/lanes`;
        G.mkdirs(shared);
        var buf = new Uint8Array(8 << 20);
        for (var name of FS.readdir(G.local)) {
          if (name === '.' || name === '..') continue;
          var from = `${G.local}/${name}`, to = `${shared}/${name}`;
          await opfsOpen(to, {create: true});
          var src = FS.open(from, 'r'), dst = FS.open(to, 'w');
          try {
            for (var pos = 0, n; (n = FS.read(src, buf, 0, buf.length, pos)) > 0; pos += n)
              FS.write(dst, buf, 0, n, pos);
          } finally {
            FS.close(src);
            FS.close(dst);
          }
          await opfsDetach(to);
          FS.unlink(from); // back to the scratch pool
        }
      }
      try { FS.rmdir(G.local); } catch (_) {}
      await opfsDetach(G.src);
      return ok;
    },

    // Merge the lanes every worker persisted into <job>/out.glb. Run on ONE worker after all of them
    // persisted. Returns the lanes merged, or -1.
    merge: async (nworkers, meshopt) => {
      var G = glbShards;
      var shared = `${G.job}/lanes`;
      G.mkdirs(shared);
      // The lane files are named OPFS files written by other modules: attach each one by name.
      var dir = adacppOpfs.root;
      for (var part of shared.slice(adacppOpfs.mountPoint.length + 1).split('/')) dir = await dir.getDirectoryHandle(part);
      var names = [];
      for await (var [name, h] of dir.entries()) if (h.kind === 'file') names.push(name);
      for (var name of names) await opfsOpen(`${shared}/${name}`);
      var out = `${G.job}/out.glb`;
      await opfsOpen(out, {create: true});
      try {
        // the merge removes the lane files (the writers own them once loaded)
        return Module['mergeGlbLanes'](shared, nworkers | 0, out, !!meshopt);
      } finally {
        await opfsDetach(out);
      }
    },
  },
});
