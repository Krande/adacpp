// Env-gated (ADACPP_STEP_PROFILE) instrumentation for the native STEP pipeline: phase wall times,
// memory (current VmRSS at phase boundaries + the kernel-tracked peak VmHWM — exact, no sampling),
// and per-solid stats. Zero cost when off; prints a [STEPPROF] summary on destruction. Threading
// utilization hooks (thread count / busy time) are here for when the per-solid loop is parallelized.
#pragma once

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "../../cad/posix_compat.h"

namespace adacpp::prof {

// A /proc/self/status field in kB (e.g. "VmRSS:" = current resident, "VmHWM:" = peak resident).
inline long proc_status_kb(const char *key) {
    std::ifstream f("/proc/self/status");
    std::string line;
    size_t klen = std::strlen(key);
    while (std::getline(f, line)) {
        if (line.compare(0, klen, key) == 0) {
            const char *p = line.c_str() + klen;
            while (*p && (*p < '0' || *p > '9'))
                ++p;
            return std::atol(p);
        }
    }
    return -1;
}

// System-pressure counters from /proc/self — used to diagnose what bounds the threaded run: CPU time
// (vs wall => achieved parallelism), context switches (voluntary => blocked on I/O/lock), disk reads
// (physical vs logical => page-cache vs real I/O), read syscalls, and major page faults.
struct SysSnap {
    double cpu_sec = 0; // utime + stime
    long majflt = 0;
    long long rchar = 0, read_bytes = 0, syscr = 0;
    long vctx = 0, nvctx = 0;
};

inline SysSnap read_sys_snap() {
    SysSnap s;
    { // /proc/self/stat — parse after the last ')' (comm may contain spaces/parens)
        std::ifstream f("/proc/self/stat");
        std::string line;
        std::getline(f, line);
        size_t rp = line.rfind(')');
        if (rp != std::string::npos && rp + 2 < line.size()) {
            std::istringstream is(line.substr(rp + 2)); // first token here is field 3 (state)
            std::vector<std::string> t;
            std::string w;
            while (is >> w)
                t.push_back(w);
            // field N -> t[N-3]: majflt=12, utime=14, stime=15
            if (t.size() > 12) {
                double hz = (double) sysconf(_SC_CLK_TCK);
                s.majflt = std::atol(t[9].c_str());
                s.cpu_sec = (std::atol(t[11].c_str()) + std::atol(t[12].c_str())) / (hz > 0 ? hz : 100.0);
            }
        }
    }
    { // /proc/self/io
        std::ifstream f("/proc/self/io");
        std::string k;
        long long v;
        while (f >> k >> v) {
            if (k == "rchar:")
                s.rchar = v;
            else if (k == "read_bytes:")
                s.read_bytes = v;
            else if (k == "syscr:")
                s.syscr = v;
        }
    }
    { // /proc/self/status — context switches
        std::ifstream f("/proc/self/status");
        std::string line;
        while (std::getline(f, line)) {
            if (line.rfind("voluntary_ctxt_switches:", 0) == 0)
                s.vctx = std::atol(line.c_str() + 24);
            else if (line.rfind("nonvoluntary_ctxt_switches:", 0) == 0)
                s.nvctx = std::atol(line.c_str() + 27);
        }
    }
    return s;
}

class StepProfiler {
public:
    using clock = std::chrono::steady_clock;
    explicit StepProfiler(const char *label)
        : on_(std::getenv("ADACPP_STEP_PROFILE") != nullptr),
          timing_(std::getenv("ADACPP_STEP_SOLID_TIMING") != nullptr), label_(label) {
        on_ = on_ || timing_;
        if (on_) {
            t0_ = last_ = clock::now();
            snap0_ = read_sys_snap();
        }
    }
    bool on() const {
        return on_;
    }

    // End a named phase: record its wall time + the current RSS.
    void phase(const char *name) {
        if (!on_)
            return;
        auto now = clock::now();
        phases_.push_back({name, ms(last_, now), proc_status_kb("VmRSS:") / 1024.0});
        last_ = now;
    }
    void solid(size_t tris) {
        if (!on_)
            return;
        std::lock_guard<std::mutex> lk(mu_); // called from worker threads
        ++n_solids_;
        total_tris_ += tris;
        if (tris > max_tris_)
            max_tris_ = tris;
    }
    void note(const char *key, double val) {
        if (on_)
            notes_.emplace_back(key, val);
    }

    // Per-worker busy time (sum of its per-solid resolve+tess+spill) + solid count, reported at the
    // worker's end. The spread across threads exposes tail idling (a thread draining early while
    // another finishes a heavy solid alone).
    void thread_done(int tid, double busy_ms, size_t solids) {
        if (!on_)
            return;
        std::lock_guard<std::mutex> lk(mu_);
        threads_.push_back({tid, busy_ms, solids});
    }

    bool timing() const {
        return timing_;
    }
    // Record one solid's (id, face count = the LPT proxy, actual tessellation ms) so the destructor
    // can report whether the face-count proxy actually predicts tessellation time. Env-gated
    // separately (ADACPP_STEP_SOLID_TIMING) since it keeps a row per solid.
    void solid_timed(long id, size_t faces, double ms) {
        if (!timing_)
            return;
        std::lock_guard<std::mutex> lk(mu_);
        timed_.push_back({id, faces, ms});
    }

    ~StepProfiler() {
        if (!on_)
            return;
        double wall = ms(t0_, clock::now());
        double hwm = proc_status_kb("VmHWM:") / 1024.0;
        std::fprintf(stderr, "[STEPPROF] %s  wall=%.0fms  peak_RSS(VmHWM)=%.0fMB\n", label_, wall, hwm);
        for (const Phase &p : phases_)
            std::fprintf(stderr, "[STEPPROF]   %-22s %8.0fms  RSS=%.0fMB\n", p.name, p.ms, p.rss_mb);
        if (n_solids_)
            std::fprintf(stderr, "[STEPPROF]   solids=%zu  tris=%zu  max_tris/solid=%zu\n", n_solids_, total_tris_,
                         max_tris_);
        for (const auto &kv : notes_)
            std::fprintf(stderr, "[STEPPROF]   %-22s %.0f\n", kv.first, kv.second);

        // Pressure points (deltas over the run): achieved parallelism, blocking, disk I/O, faults.
        SysSnap e = read_sys_snap();
        double cpu = e.cpu_sec - snap0_.cpu_sec;
        double wall_s = wall / 1000.0;
        std::fprintf(stderr, "[STEPPROF]   cpu_time=%.1fs  parallelism=%.2fx (cpu/wall — cores kept busy)\n", cpu,
                     wall_s > 0 ? cpu / wall_s : 0.0);
        std::fprintf(stderr, "[STEPPROF]   ctxt_switch: %ld voluntary (blocked on I/O/lock) / %ld involuntary\n",
                     e.vctx - snap0_.vctx, e.nvctx - snap0_.nvctx);
        std::fprintf(stderr,
                     "[STEPPROF]   disk_read: %.0fMB physical / %.0fMB logical  read_syscalls=%lld  majflt=%ld\n",
                     (e.read_bytes - snap0_.read_bytes) / 1e6, (e.rchar - snap0_.rchar) / 1e6, e.syscr - snap0_.syscr,
                     e.majflt - snap0_.majflt);

        if (!threads_.empty()) {
            std::sort(threads_.begin(), threads_.end(), [](const Thr &a, const Thr &b) { return a.tid < b.tid; });
            double busy_min = 1e30, busy_max = 0, busy_sum = 0;
            for (const Thr &t : threads_) {
                busy_min = std::min(busy_min, t.busy_ms);
                busy_max = std::max(busy_max, t.busy_ms);
                busy_sum += t.busy_ms;
            }
            std::fprintf(stderr,
                         "[STEPPROF]   thread util: busiest=%.0fms idlest=%.0fms spread=%.0fms (tail idle); "
                         "avg busy=%.0fms over %zu threads\n",
                         busy_max, busy_min, busy_max - busy_min, busy_sum / threads_.size(), threads_.size());
            for (const Thr &t : threads_)
                std::fprintf(stderr, "[STEPPROF]     t%-2d solids=%-6zu busy=%.0fms\n", t.tid, t.solids, t.busy_ms);
        }
        if (timing_ && !timed_.empty())
            dump_solid_timing();
    }

private:
    static double ms(clock::time_point a, clock::time_point b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    }
    struct Phase {
        const char *name;
        double ms;
        double rss_mb;
    };
    struct SolidTime {
        long id;
        size_t faces;
        double ms;
    };
    struct Thr {
        int tid;
        double busy_ms;
        size_t solids;
    };

    // Does the face-count LPT proxy predict tessellation time? Report the slowest solids (with their
    // rank by face count) + the Spearman rank correlation between face count and tessellation ms.
    void dump_solid_timing() {
        size_t n = timed_.size();
        std::vector<size_t> by_ms(n), by_faces(n);
        for (size_t i = 0; i < n; ++i)
            by_ms[i] = by_faces[i] = i;
        std::sort(by_ms.begin(), by_ms.end(), [&](size_t a, size_t b) { return timed_[a].ms > timed_[b].ms; });
        std::sort(by_faces.begin(), by_faces.end(),
                  [&](size_t a, size_t b) { return timed_[a].faces > timed_[b].faces; });
        std::vector<size_t> facerank(n), timerank(n);
        for (size_t r = 0; r < n; ++r) {
            facerank[by_faces[r]] = r; // 0 = most faces
            timerank[by_ms[r]] = r;    // 0 = slowest
        }
        double d2 = 0;
        for (size_t i = 0; i < n; ++i) {
            double d = (double) facerank[i] - (double) timerank[i];
            d2 += d * d;
        }
        double rho = n > 1 ? 1.0 - 6.0 * d2 / ((double) n * ((double) n * n - 1.0)) : 1.0;
        std::fprintf(stderr,
                     "[STEPTIME] solids=%zu  Spearman(faces,ms)=%.3f  (1.0 = face count perfectly "
                     "predicts tessellation time)\n",
                     n, rho);
        std::fprintf(stderr, "[STEPTIME] slowest solids (id  faces  ms  face_rank/%zu):\n", n);
        for (size_t r = 0; r < n && r < 15; ++r) {
            const SolidTime &s = timed_[by_ms[r]];
            std::fprintf(stderr, "[STEPTIME]   #%-8ld %6zu %8.1f   face_rank=%zu\n", s.id, s.faces, s.ms,
                         facerank[by_ms[r]]);
        }
        // How well would LPT's top picks (by faces) cover the actual slow tail? Report the worst
        // time-rank among the top-N-by-faces (if LPT starts the top-N first, the slowest should be there).
        size_t topN = n < 16 ? n : 16;
        size_t worst_timerank = 0;
        for (size_t r = 0; r < topN; ++r)
            worst_timerank = std::max(worst_timerank, timerank[by_faces[r]]);
        std::fprintf(stderr,
                     "[STEPTIME] top-%zu by faces (LPT first picks): worst actual time-rank = %zu "
                     "(want < %zu so the slow ones start early)\n",
                     topN, worst_timerank, topN);
    }

    bool on_;
    bool timing_;
    const char *label_;
    clock::time_point t0_, last_;
    std::vector<Phase> phases_;
    size_t n_solids_ = 0, total_tris_ = 0, max_tris_ = 0;
    std::vector<std::pair<const char *, double>> notes_;
    std::vector<SolidTime> timed_;
    std::vector<Thr> threads_;
    SysSnap snap0_;
    std::mutex mu_;
};

} // namespace adacpp::prof
