// A WASMFS backend whose files are OPFS FileSystemSyncAccessHandles, for the single-threaded embind
// modules (adacpp_step_glb, adacpp_ifc_glb, adacpp_brep_writer, adacpp_fea).
//
// WHY NOT emscripten's OPFS backend. wasmfs_create_opfs_backend() only works in a -pthread build
// (every OPFS call is proxied to a helper thread that blocks on Atomics.wait, which needs
// SharedArrayBuffer and therefore a cross-origin-isolated page) or under JSPI. In a plain build the
// backend's JS imports are `async` functions that return a Promise the C++ cannot wait for, so
// mountOpfs "succeeds" and the first open/stat/write on the mount executes WASMFS_UNREACHABLE.
// Even under JSPI, the non-pthread path emulates access handles with createWritable() per write, which
// rewrites the whole file on every chunk.
//
// WHAT THIS DOES INSTEAD. In a dedicated worker, FileSystemSyncAccessHandle.read/write/getSize/
// truncate/flush/close are SYNCHRONOUS. Only acquiring a handle is async. So the JS side
// (opfs_sync.js) acquires handles ahead of time -- the files a caller names (opfsOpen), plus a pool
// of anonymous scratch files -- and the C++ side does every byte of I/O on them synchronously,
// straight between the OPFS file and the wasm heap buffer the caller passed: no copy of the file is
// held in the heap. Directories are in-memory WASMFS directories; a file created by the C++ (an output
// or a spill lane nobody pre-opened) takes a scratch handle from the pool. No threads, no JSPI, no
// COOP/COEP.
//
// This builds on WASMFS's internal backend classes (system/lib/wasmfs), the same ones its own
// backends use, so it is tied to the pinned emscripten (4.0.9).
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <emscripten/bind.h>
#include <emscripten/wasmfs.h>

// WASMFS internals (include dir: <emscripten>/system/lib/wasmfs).
#include "backend.h"
#include "file.h"
#include "memory_backend.h"
#include "wasmfs.h"

extern "C" {
// Implemented in opfs_sync.js. `id` names one sync access handle on the JS side. Offsets and sizes
// travel as doubles (exact to 2^53) so no i64 legalisation is involved.
int _adacpp_opfs_take(void);
void _adacpp_opfs_release(int id);
double _adacpp_opfs_read(int id, uint8_t *buf, size_t len, double offset);
double _adacpp_opfs_write(int id, const uint8_t *buf, size_t len, double offset);
double _adacpp_opfs_size(int id);
int _adacpp_opfs_truncate(int id, double size);
int _adacpp_opfs_flush(int id);
}

namespace {

using wasmfs::DataFile;
using wasmfs::MemoryDirectory;
using wasmfs::MemorySymlink;
using fs_backend_t = wasmfs::backend_t; // not the C API's opaque ::backend_t

// One OPFS file. Every sync-access-handle call costs on the order of 100 us whatever its size (measured
// in Chromium), and the readers here make many small reads -- a statement at a time, scattered over the
// file as a solid's entities are -- while the writers emit 1-4 KiB stdio chunks. So reads go through
// an LRU block cache and sequential writes are coalesced; both are bounded (32 MiB + 1 MiB per open
// file) and dropped when the last descriptor closes. The cache is sized for SEVERAL workers reading one
// large model at once: Chromium serves every worker's handle reads from one browser-process thread, so
// misses queue behind each other -- a 415 MB STEP on 4 workers converted in 41 s with a 4 MiB cache and
// in 17 s with this one (8 workers: 10 s; 64 MiB gains nothing more). Nothing else writes the file
// while it is open (an exclusive handle, or read-only handles that all only read), so the cached size
// and blocks stay valid.
class OpfsSyncFile : public DataFile {
    static constexpr size_t kBlock = 256 * 1024;
    static constexpr size_t kBlocks = 128;
    static constexpr size_t kWriteBuf = 1024 * 1024;

    struct Block {
        off_t start = -1; // -1: empty
        size_t len = 0;   // valid bytes (short at EOF)
        uint64_t used = 0;
        std::vector<uint8_t> data;
    };

    int id_;
    off_t size_ = -1; // -1: not fetched yet
    std::vector<Block> cache_;
    uint64_t tick_ = 0;
    std::vector<uint8_t> wbuf_; // pending bytes for [wbuf_off_, wbuf_off_ + wbuf_.size())
    off_t wbuf_off_ = 0;

    off_t size() {
        if (size_ < 0) {
            double s = _adacpp_opfs_size(id_);
            if (s < 0)
                return (off_t) s;
            size_ = (off_t) s;
        }
        return size_;
    }

    void invalidate(off_t off, size_t len) {
        for (auto &b : cache_)
            if (b.start >= 0 && b.start < off + (off_t) len && off < b.start + (off_t) kBlock)
                b.start = -1;
    }

    int flush_writes() {
        size_t done = 0;
        while (done < wbuf_.size()) {
            double r = _adacpp_opfs_write(id_, wbuf_.data() + done, wbuf_.size() - done, (double) (wbuf_off_ + done));
            if (r <= 0) {
                wbuf_.clear();
                return r < 0 ? (int) r : -EIO;
            }
            done += (size_t) r;
        }
        wbuf_.clear();
        return 0;
    }

    // The cached block starting at `start` (a multiple of kBlock), read in on a miss.
    Block *block(off_t start, int &err) {
        Block *victim = nullptr;
        for (auto &b : cache_) {
            if (b.start == start) {
                b.used = ++tick_;
                return &b;
            }
            if (!victim || (victim->start >= 0 && (b.start < 0 || b.used < victim->used)))
                victim = &b;
        }
        if (cache_.size() < kBlocks && (!victim || victim->start >= 0)) {
            victim = &cache_.emplace_back();
            victim->data.resize(kBlock);
        }
        double r = _adacpp_opfs_read(id_, victim->data.data(), kBlock, (double) start);
        if (r < 0) {
            victim->start = -1;
            err = (int) r;
            return nullptr;
        }
        victim->start = start;
        victim->len = (size_t) r;
        victim->used = ++tick_;
        return victim;
    }

    int open(wasmfs::oflags_t) override {
        return 0;
    }

    int close() override {
        int err = flush_writes();
        std::vector<Block>().swap(cache_);
        std::vector<uint8_t>().swap(wbuf_);
        return err;
    }

    ssize_t read(uint8_t *buf, size_t len, off_t offset) override {
        if (int err = flush_writes())
            return err;
        off_t sz = size();
        if (sz < 0)
            return sz;
        if (offset >= sz || len == 0)
            return 0;
        len = std::min(len, (size_t) (sz - offset));
        if (len >= kBlock) // big reads (the readers' scan windows, stride reads) go straight through
            return (ssize_t) _adacpp_opfs_read(id_, buf, len, (double) offset);
        size_t done = 0;
        while (done < len) {
            off_t pos = offset + (off_t) done;
            int err = 0;
            Block *b = block(pos - pos % (off_t) kBlock, err);
            if (!b)
                return err;
            if (pos >= b->start + (off_t) b->len)
                break;
            size_t n = std::min(len - done, (size_t) (b->start + (off_t) b->len - pos));
            std::memcpy(buf + done, b->data.data() + (pos - b->start), n);
            done += n;
        }
        return (ssize_t) done;
    }

    ssize_t write(const uint8_t *buf, size_t len, off_t offset) override {
        if (len == 0)
            return 0;
        off_t sz = size();
        if (sz < 0)
            return sz;
        invalidate(offset, len);
        bool append = !wbuf_.empty() && offset == wbuf_off_ + (off_t) wbuf_.size();
        if (!append || wbuf_.size() + len > kWriteBuf) {
            if (int err = flush_writes())
                return err;
        }
        if (len >= kWriteBuf) {
            double r = _adacpp_opfs_write(id_, buf, len, (double) offset);
            if (r < 0)
                return (ssize_t) r;
            size_ = std::max(size_, offset + (off_t) r);
            return (ssize_t) r;
        }
        if (wbuf_.empty()) {
            wbuf_.reserve(kWriteBuf);
            wbuf_off_ = offset;
        }
        wbuf_.insert(wbuf_.end(), buf, buf + len);
        size_ = std::max(size_, offset + (off_t) len);
        return (ssize_t) len;
    }

    int flush() override {
        if (int err = flush_writes())
            return err;
        return _adacpp_opfs_flush(id_);
    }

    off_t getSize() override {
        return size();
    }

    int setSize(off_t size) override {
        if (int err = flush_writes())
            return err;
        cache_.clear();
        int err = _adacpp_opfs_truncate(id_, (double) size);
        size_ = err ? -1 : size;
        return err;
    }

public:
    OpfsSyncFile(mode_t mode, fs_backend_t backend, int id) : DataFile(mode, backend), id_(id) {}
    // The last reference goes when the file is unlinked and no descriptor holds it: hand the handle
    // back (a scratch file is truncated and pooled, a named one is flushed, closed and removed from
    // OPFS -- or only closed, for opfsDetach).
    ~OpfsSyncFile() override {
        flush_writes();
        _adacpp_opfs_release(id_);
    }
};

// In-memory directory whose new files are OPFS-backed. MemoryDirectory::insertDataFile would insert
// whatever createFile returned; this one reports a failed create (pool exhausted) as -EIO instead.
class OpfsSyncDirectory : public MemoryDirectory {
protected:
    std::shared_ptr<DataFile> insertDataFile(const std::string &name, mode_t mode) override {
        auto child = getBackend()->createFile(mode);
        if (!child)
            return nullptr;
        insertChild(name, child);
        return child;
    }

public:
    OpfsSyncDirectory(mode_t mode, fs_backend_t backend) : MemoryDirectory(mode, backend) {}
};

class OpfsSyncBackend : public wasmfs::Backend {
public:
    std::shared_ptr<DataFile> createFile(mode_t mode) override {
        int id = _adacpp_opfs_take();
        if (id < 0)
            return nullptr;
        return std::make_shared<OpfsSyncFile>(mode, this, id);
    }
    std::shared_ptr<wasmfs::Directory> createDirectory(mode_t mode) override {
        return std::make_shared<OpfsSyncDirectory>(mode, this);
    }
    std::shared_ptr<wasmfs::Symlink> createSymlink(std::string target) override {
        return std::make_shared<MemorySymlink>(target, this);
    }
};

fs_backend_t g_backend = nullptr;

// Create the mount directory. Called by opfsMount() once its handles are ready; 0 or -errno.
int opfs_mount(const std::string &mount_point) {
    if (!g_backend)
        g_backend = wasmfs::wasmFS.addBackend(std::make_unique<OpfsSyncBackend>());
    // The C API spells the same pointer as an opaque ::backend_t.
    return wasmfs_create_directory(mount_point.c_str(), 0777, reinterpret_cast<::backend_t>(g_backend));
}

// Make `path` (under the mount) a file backed by the handle the JS side has just queued for the next
// createFile. Parent directories are created as needed. 0 or -errno; EEXIST if the path is taken.
int opfs_attach(const std::string &path) {
    for (size_t slash = path.find('/', 1); slash != std::string::npos; slash = path.find('/', slash + 1)) {
        std::string dir = path.substr(0, slash);
        if (mkdir(dir.c_str(), 0777) != 0 && errno != EEXIST)
            return -errno;
    }
    int fd = open(path.c_str(), O_CREAT | O_EXCL | O_RDWR, 0666);
    if (fd < 0)
        return -errno;
    close(fd);
    return 0;
}

} // namespace

EMSCRIPTEN_BINDINGS(adacpp_opfs_sync) {
    // Internal: the public surface is opfsMount / opfsOpen / ... in opfs_sync.js.
    emscripten::function("_adacppOpfsMount", &opfs_mount);
    emscripten::function("_adacppOpfsAttach", &opfs_attach);
}
