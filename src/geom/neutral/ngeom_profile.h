// Env-gated (ADACPP_STEP_PROFILE) instrumentation for the native STEP pipeline: phase wall times,
// memory (current VmRSS at phase boundaries + the kernel-tracked peak VmHWM — exact, no sampling),
// and per-solid stats. Zero cost when off; prints a [STEPPROF] summary on destruction. Threading
// utilization hooks (thread count / busy time) are here for when the per-solid loop is parallelized.
#pragma once

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

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

class StepProfiler {
public:
    using clock = std::chrono::steady_clock;
    explicit StepProfiler(const char *label) : on_(std::getenv("ADACPP_STEP_PROFILE") != nullptr), label_(label) {
        if (on_)
            t0_ = last_ = clock::now();
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
    bool on_;
    const char *label_;
    clock::time_point t0_, last_;
    std::vector<Phase> phases_;
    size_t n_solids_ = 0, total_tris_ = 0, max_tris_ = 0;
    std::vector<std::pair<const char *, double>> notes_;
    std::mutex mu_;
};

} // namespace adacpp::prof
