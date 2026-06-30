#pragma once
// Portable per-loop heap trim. glibc's malloc_trim(0) returns freed arena pages to the OS, which keeps
// the streaming workers' RSS bounded on the Linux deploy. macOS (BSD libc) and Windows (MSVC) have no
// equivalent and no <malloc.h>, so this is a no-op there — the platform allocator manages it.
#if defined(__GLIBC__)
#include <malloc.h>
#endif

namespace adacpp {
inline void mem_trim() {
#if defined(__GLIBC__)
    ::malloc_trim(0);
#endif
}
} // namespace adacpp
