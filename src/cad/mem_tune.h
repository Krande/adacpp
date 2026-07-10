#pragma once
// One-time glibc malloc tuning for the multithreaded streaming CAD pipeline.
//
// The streaming tessellator churns large transient buffers per solid (the resolved B-rep + libtess2's
// per-face DCEL). By default glibc keeps such freed blocks in the arena free-lists instead of returning
// them to the OS, so process RSS tracks the high-water of retained-but-dead memory, not the live set.
// Lowering the mmap/trim threshold makes the LARGE transient blocks round-trip through mmap and get
// unmapped to the OS the instant they are freed, trimming peak RSS at negligible cost.
//
// Threshold choice (measured, crane STEP->GLB, 39.7 M tris, 3 workers; peak RSS / wall):
//     glibc default            3009 MB / 157 s   (baseline)
//     M_*_THRESHOLD = 4 MB     2866 MB / 159 s   <-- chosen: -143 MB for +1% time
//     M_*_THRESHOLD = 128 KB   ~2450 MB / 279 s  (too aggressive: mmap/munmap per libtess2 alloc
//                                                 triples the heavy-solid tessellation time)
//     M_ARENA_MAX  = 2/4       no memory win at 4, -560 MB but +56% time at 2  (dropped: bad trade)
// The 4 MB knee returns the big freed buffers (whole-solid resolve + large DCELs) without forcing the
// many small libtess2 allocations through mmap — those stay fast in the arena. On tessellation-working-
// set-heavy models (few tris but complex faces, e.g. the 469826 STEP that OOMs) the reclaim is larger,
// since there the peak is transient DCEL churn rather than one solid's live mesh.
//
// Call ONCE at the top of each streaming entry point. glibc-only; a no-op on macOS / Windows / wasm.

#include <cstdlib>
#if defined(__GLIBC__)
#include <malloc.h>
#endif

namespace adacpp {
inline void tune_malloc_for_streaming() {
#if defined(__GLIBC__)
    static bool done = false; // idempotent: mallopt is process-global, first call wins
    if (done)
        return;
    done = true;
    if (std::getenv("ADACPP_NO_MALLOC_TUNE")) // escape hatch / A-B switch: leave glibc defaults
        return;
    // Large freed blocks -> straight back to the OS on free (mmap) and trim the arena top; small
    // allocations stay in-arena (fast). 4 MB is the measured knee — memory win with ~no time cost.
    constexpr long kThreshold = 4L * 1024 * 1024;
    mallopt(M_MMAP_THRESHOLD, (int) kThreshold);
    mallopt(M_TRIM_THRESHOLD, (int) kThreshold);
#endif
}
} // namespace adacpp
