// Make OCCT's GLOBAL thread pool survive fork(), so a process that has meshed can still fork.
//
// The hazard, verified with gdb on a hung child: BRepMesh_IncrementalMesh(..., parallel=true)
// lazily populates the process-wide OSD_ThreadPool::DefaultPool(). Those are real OS threads,
// and POSIX gives the child of a fork() only the calling thread — every other thread is gone,
// while the memory they were using (including the pool's condition variables) is inherited
// verbatim. The child's next parallel mesh therefore blocks forever in
//
//     OSD_ThreadPool::EnumeratedThread::WaitIdle()   <- pthread_cond_wait on a worker
//     OSD_ThreadPool::Launcher::wait()
//     BRepMesh_EdgeDiscret::performInternal(...)
//
// waiting to be woken by workers that no longer exist. It is not an adacpp bug — a global
// thread pool simply cannot be inherited across fork() — but adacpp is where it has to be
// fixed, because adacpp is what calls the parallel mesher and OCCT exposes no way to hand
// BRepMesh a private pool (OSD_Parallel hard-codes DefaultPool(), and IMeshTools_Parameters
// carries only an `InParallel` bool).
//
// The fix is to park the pool while its threads are still ALIVE and IDLE, i.e. in the
// pthread_atfork *prepare* handler, which runs in the parent before the address space is
// copied. Init(1) drops the pool to "no worker threads"; the threads are respawned lazily on
// the next Launcher, independently in each process. Both sides then restore the original size.
//
// Two things measured rather than assumed, because both are load-bearing:
//   * Init() must NOT be called in the child to repair the pool after the fact — it deadlocks
//     there too, trying to stop threads that are already gone. Parking has to happen pre-fork.
//   * Init(0) is a no-op (the child still deadlocked); only Init(n >= 1) actually resets the
//     pool. NbThreads() round-trips exactly through Init(), so the size can be saved+restored.
#pragma once

#if !defined(_WIN32) && !defined(__EMSCRIPTEN__)

#include <OSD_ThreadPool.hxx>
#include <pthread.h>

namespace adacpp {

namespace occt_fork_detail {

// Pool size captured in `prepare` and restored in `parent`/`child`. Only ever touched between a
// prepare handler and its matching parent/child handler, and fork() is serialised by the runtime,
// so a plain int needs no synchronisation. -1 means "prepare declined to park" (see below).
inline int &saved_nb_threads() {
    static int value = -1;
    return value;
}

inline void before_fork() {
    saved_nb_threads() = -1;
    const occ::handle<OSD_ThreadPool> &pool = OSD_ThreadPool::DefaultPool();
    if (pool.IsNull() || !pool->HasThreads())
        return; // never meshed in parallel, or already parked: nothing to lose across the fork
    if (pool->IsInUse())
        return; // a Launcher holds threads RIGHT NOW (a mesh is running on another thread).
                // Init() would block on it and hang the fork itself, which is strictly worse
                // than the deadlock we are avoiding — leave the pool alone and let the child
                // fail the way it does today. Forking mid-mesh is not something adacpp does.
    saved_nb_threads() = pool->NbThreads();
    pool->Init(1); // park: threads stop while they are still alive here in the parent
}

inline void restore_pool() {
    const int nb = saved_nb_threads();
    saved_nb_threads() = -1;
    if (nb < 1)
        return; // prepare declined; pool was left exactly as it was
    const occ::handle<OSD_ThreadPool> &pool = OSD_ThreadPool::DefaultPool();
    if (!pool.IsNull())
        pool->Init(nb); // threads respawn lazily, in THIS process
}

} // namespace occt_fork_detail

// Register the fork handlers once per process. Idempotent, so it is safe to call from several
// entry points (the Python extension's module init, a CLI main) without stacking handlers —
// pthread_atfork has no deregistration, so double-registering would park/restore twice.
inline void ensure_fork_safe_occt_threadpool() {
    static const bool registered = []() {
        pthread_atfork(&occt_fork_detail::before_fork,   // parent, pre-fork: park the pool
                       &occt_fork_detail::restore_pool,  // parent, post-fork: put it back
                       &occt_fork_detail::restore_pool); // child: build its own threads
        return true;
    }();
    (void) registered;
}

} // namespace adacpp

#else

namespace adacpp {
// No fork() on Windows, and the wasm build is single-threaded with no pool to park.
inline void ensure_fork_safe_occt_threadpool() {}
} // namespace adacpp

#endif
