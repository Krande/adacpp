#pragma once
// Container-aware auto thread count. Inside a container with a CFS cpu quota,
// std::thread::hardware_concurrency() reports the NODE's cores (a k8s pod with
// cpu=4 on a 16-core node sees 16) — the quota throttles time slices, it doesn't
// mask cores. Sizing a solid-resolve pool from the node count both oversubscribes
// the quota AND multiplies peak RSS by the extra threads, since each thread holds
// a whole resolved solid (and the pools sort largest-first, so the N biggest
// solids are resolved concurrently at the start). Clamp the auto count to the
// cgroup quota when one is set; explicit num_threads arguments stay untouched.
// No cgroup files (macOS / Windows / wasm) -> plain hardware_concurrency().

#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

namespace adacpp {

namespace detail {

// Parse "<quota_us>|max <period_us>" (cgroup v2 cpu.max). 0 = no quota.
inline unsigned parse_cpu_max(const char *path) {
    std::FILE *f = std::fopen(path, "r");
    if (!f)
        return 0;
    double quota = 0, period = 0;
    int n = std::fscanf(f, "%lf %lf", &quota, &period); // "max" fails the first %lf -> n < 2
    std::fclose(f);
    if (n == 2 && quota > 0 && period > 0)
        return (unsigned) ((quota + period - 1) / period); // ceil: cpu=3.5 -> 4 threads
    return 0;
}

// The process's cgroup-v2 relative path from /proc/self/cgroup ("0::<path>").
inline std::string self_cgroup_v2_path() {
    std::FILE *f = std::fopen("/proc/self/cgroup", "r");
    if (!f)
        return {};
    char line[512];
    std::string out;
    while (std::fgets(line, sizeof(line), f)) {
        if (std::strncmp(line, "0::", 3) == 0) {
            out.assign(line + 3);
            while (!out.empty() && (out.back() == '\n' || out.back() == '/'))
                out.pop_back();
            break;
        }
    }
    std::fclose(f);
    return out;
}

} // namespace detail

inline unsigned cgroup_cpu_quota() {
    // cgroup v2: the limit can sit on the process's own cgroup or any ancestor
    // (k8s sets it on the container scope; with a cgroup namespace that IS
    // /sys/fs/cgroup, on a plain host it's a nested systemd scope/slice).
    // Walk from the process's cgroup up to the root; nearest quota wins.
    std::string rel = detail::self_cgroup_v2_path();
    for (;;) {
        std::string p = "/sys/fs/cgroup" + rel + "/cpu.max";
        if (unsigned q = detail::parse_cpu_max(p.c_str()))
            return q;
        if (rel.empty())
            break;
        size_t cut = rel.rfind('/');
        rel = (cut == std::string::npos) ? std::string() : rel.substr(0, cut);
    }
    // cgroup v1 (container runtimes mount the container's cgroup at the controller root)
    long quota = -1, period = -1;
    if (std::FILE *f = std::fopen("/sys/fs/cgroup/cpu/cpu.cfs_quota_us", "r")) {
        if (std::fscanf(f, "%ld", &quota) != 1)
            quota = -1;
        std::fclose(f);
    }
    if (std::FILE *f = std::fopen("/sys/fs/cgroup/cpu/cpu.cfs_period_us", "r")) {
        if (std::fscanf(f, "%ld", &period) != 1)
            period = -1;
        std::fclose(f);
    }
    if (quota > 0 && period > 0)
        return (unsigned) ((quota + period - 1) / period);
    return 0; // no quota detected
}

inline unsigned effective_concurrency() {
    unsigned hw = std::thread::hardware_concurrency();
    if (hw < 1)
        hw = 1;
    unsigned q = cgroup_cpu_quota();
    return (q > 0 && q < hw) ? q : hw;
}

} // namespace adacpp
