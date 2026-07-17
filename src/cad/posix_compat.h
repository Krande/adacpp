#pragma once
// Portable POSIX file-I/O shim for the native streaming readers. On Linux/macOS this just pulls in the
// real POSIX headers. On Windows (MSVC) — which has no <unistd.h>/<sys/mman.h> and no pread/mmap/
// mkdtemp/ftruncate/madvise — it provides the small subset the readers use (pread via ReadFile+
// OVERLAPPED, mmap via CreateFileMapping, etc.) so the same source compiles + runs cross-platform.
// close/open/lseek/rmdir/read are MSVC's POSIX-name aliases (enabled by the build's
// _CRT_NONSTDC_NO_DEPRECATE) and need no shim.
#if !defined(_WIN32)
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#else
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
//
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <direct.h>
#include <fcntl.h>
#include <io.h>
#include <mutex>
#include <share.h>
#include <string>
#include <sys/stat.h>
#include <unordered_map>

#ifndef _SSIZE_T_DEFINED
using ssize_t = long long;
#define _SSIZE_T_DEFINED
#endif

// mmap()/madvise() flags.
#define PROT_READ 0x1
#define PROT_WRITE 0x2
#define MAP_PRIVATE 0x2
#define MAP_SHARED 0x1
#define MAP_FAILED ((void *) -1)
#define MADV_NORMAL 0
#define MADV_RANDOM 1
#define MADV_SEQUENTIAL 2
#define MADV_DONTNEED 4
#define MS_ASYNC 1
#define MS_INVALIDATE 2
#define MS_SYNC 4
#define _SC_CLK_TCK 2
#define _SC_PAGESIZE 30

namespace adacpp_posix {
inline std::mutex &mtx() {
    static std::mutex m;
    return m;
}
inline std::unordered_map<void *, HANDLE> &handles() {
    static std::unordered_map<void *, HANDLE> m;
    return m;
}
} // namespace adacpp_posix

// Thread-safe positional read (OVERLAPPED carries the offset, so a shared fd is safe across threads).
inline ssize_t pread(int fd, void *buf, size_t count, long long offset) {
    HANDLE h = (HANDLE) _get_osfhandle(fd);
    if (h == INVALID_HANDLE_VALUE)
        return -1;
    OVERLAPPED ov = {};
    ov.Offset = (DWORD) ((unsigned long long) offset & 0xFFFFFFFFULL);
    ov.OffsetHigh = (DWORD) ((unsigned long long) offset >> 32);
    DWORD got = 0;
    if (!ReadFile(h, buf, (DWORD) count, &got, &ov))
        return (GetLastError() == ERROR_HANDLE_EOF) ? 0 : -1;
    return (ssize_t) got;
}

// Whole-file mapping from offset 0 (addr/flags ignored). PROT_WRITE -> a writable mapping sized to
// `length` (the GLB-diff scratch); else read-only. munmap looks the mapping HANDLE up by base pointer.
inline void *mmap(void *, size_t length, int prot, int, int fd, long long) {
    HANDLE fh = (HANDLE) _get_osfhandle(fd);
    if (fh == INVALID_HANDLE_VALUE)
        return MAP_FAILED;
    bool wr = (prot & PROT_WRITE) != 0;
    DWORD hi = (DWORD) ((unsigned long long) length >> 32), lo = (DWORD) ((unsigned long long) length & 0xFFFFFFFFULL);
    HANDLE mh = CreateFileMappingW(fh, nullptr, wr ? PAGE_READWRITE : PAGE_READONLY, hi, lo, nullptr);
    if (!mh)
        return MAP_FAILED;
    void *p = MapViewOfFile(mh, wr ? FILE_MAP_WRITE : FILE_MAP_READ, 0, 0, (SIZE_T) length);
    if (!p) {
        CloseHandle(mh);
        return MAP_FAILED;
    }
    std::lock_guard<std::mutex> lk(adacpp_posix::mtx());
    adacpp_posix::handles()[p] = mh;
    return p;
}
inline int munmap(void *addr, size_t) {
    UnmapViewOfFile(addr);
    std::lock_guard<std::mutex> lk(adacpp_posix::mtx());
    auto &m = adacpp_posix::handles();
    auto it = m.find(addr);
    if (it != m.end()) {
        CloseHandle(it->second);
        m.erase(it);
    }
    return 0;
}
inline int madvise(void *, size_t, int) {
    return 0; // no Windows equivalent — harmless no-op (loses only the RSS-trim hint)
}
inline int msync(void *addr, size_t length, int) {
    return FlushViewOfFile(addr, length) ? 0 : -1; // flush a writable mapping to disk
}

// Windows has no /tmp. The native readers hardcode "/tmp/foo_XXXXXX" templates (fine on Linux/wasm);
// on Windows MSVC resolves "/tmp" to "C:\tmp", which usually doesn't exist, so _mkdir fails and the
// whole conversion returns -1. Redirect a leading "/tmp/" to the process temp dir (GetTempPathA) so
// the same hardcoded templates land under a real directory. Callers use mkdtemp's RETURN value (not
// the passed template), so returning a shim-owned buffer with the longer real path is safe even
// though it no longer aliases `tmpl`.
inline std::string _win_tmp_redirect(const char *tmpl) {
    if (std::strncmp(tmpl, "/tmp/", 5) == 0) {
        char buf[MAX_PATH];
        const DWORD n = GetTempPathA(MAX_PATH, buf); // includes a trailing backslash
        if (n > 0 && n < MAX_PATH)
            return std::string(buf, n) + (tmpl + 5);
    }
    return std::string(tmpl);
}

inline char *mkdtemp(char *tmpl) {
    // tmpl ends in "XXXXXX"; make it unique then create the directory (POSIX mkdtemp semantics).
    // A thread_local buffer holds the /tmp-redirected path so the returned pointer stays valid until
    // this thread's next mkdtemp call — long enough for the caller to copy it into a std::string.
    thread_local std::string path;
    path = _win_tmp_redirect(tmpl);
    if (_mktemp_s(path.data(), path.size() + 1) != 0) // data() buffer spans size()+1 (incl. NUL)
        return nullptr;
    if (_mkdir(path.c_str()) != 0)
        return nullptr;
    return path.data();
}
inline int ftruncate(int fd, long long length) {
    return _chsize_s(fd, length);
}
// POSIX mkstemp: make "....XXXXXX" unique, create+open it O_RDWR binary, return the fd (-1 on error).
inline int mkstemp(char *tmpl) {
    if (_mktemp_s(tmpl, std::strlen(tmpl) + 1) != 0)
        return -1;
    int fd = -1;
    if (_sopen_s(&fd, tmpl, _O_CREAT | _O_EXCL | _O_RDWR | _O_BINARY, _SH_DENYNO, _S_IREAD | _S_IWRITE) != 0)
        return -1;
    return fd;
}
inline long sysconf(int name) {
    if (name == _SC_PAGESIZE) {
        SYSTEM_INFO si;
        GetSystemInfo(&si);
        return (long) si.dwPageSize;
    }
    return (long) CLOCKS_PER_SEC; // _SC_CLK_TCK et al.
}
inline int setenv(const char *name, const char *value, int /*overwrite*/) {
    return _putenv_s(name, value); // _putenv_s always overwrites
}
#endif

#include <cstdlib>
#include <string>

namespace adacpp {
// Base directory for the readers' scratch files/dirs. An explicit override lets a caller redirect
// every hardcoded temp location without touching source (e.g. a sandbox with no writable /tmp, or a
// fast local SSD for spill): $ADACPP_TMPDIR wins, then $TMPDIR, else the platform default (%TEMP%
// via GetTempPath on Windows, /tmp elsewhere). Never returns a trailing separator.
inline std::string temp_base_dir() {
    if (const char *e = std::getenv("ADACPP_TMPDIR"); e && *e)
        return std::string(e);
    if (const char *e = std::getenv("TMPDIR"); e && *e)
        return std::string(e);
#if defined(_WIN32)
    char buf[MAX_PATH];
    const DWORD n = GetTempPathA(MAX_PATH, buf); // ends with a backslash
    if (n > 0 && n < MAX_PATH) {
        std::string s(buf, n);
        while (!s.empty() && (s.back() == '\\' || s.back() == '/'))
            s.pop_back();
        return s;
    }
    return "."; // last resort: process cwd
#else
    return "/tmp";
#endif
}

// A mutable mkdtemp/mkstemp template "<temp_base_dir>/<name>_XXXXXX". Returned by value; pass
// .data() to mkdtemp/mkstemp (they rewrite the XXXXXX in place). Replaces the old hardcoded
// "/tmp/..._XXXXXX" literals so the override above reaches every scratch path.
inline std::string temp_template(const char *name) {
    return temp_base_dir() + "/" + name + "_XXXXXX";
}
} // namespace adacpp
