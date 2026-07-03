// STEP (Part-21) -> NGEOM neutral records resolver. First slice: planar B-rep.
//
// Consumes a parsed entity map (id -> Instance, from step_part21.h) and builds an
// adacpp::ngeom::NgeomDoc. Neutral records are populated to match ngeom_decode.h field-for-field,
// so a natively-read solid tessellates identically to the same solid round-tripped through the
// adapy Python serializer + the NGEOM decoder (the parity oracle).
//
// Supported now: MANIFOLD_SOLID_BREP / CLOSED_SHELL / OPEN_SHELL / ADVANCED_FACE / FACE_SURFACE /
// FACE_OUTER_BOUND / FACE_BOUND / EDGE_LOOP / POLY_LOOP / ORIENTED_EDGE / EDGE_CURVE /
// VERTEX_POINT / AXIS2_PLACEMENT_3D / CARTESIAN_POINT / DIRECTION; surfaces PLANE /
// CYLINDRICAL_SURFACE / CONICAL_SURFACE / SPHERICAL_SURFACE / TOROIDAL_SURFACE /
// B_SPLINE_SURFACE_WITH_KNOTS (+ rational complex records); edge curves CIRCLE / ELLIPSE /
// B_SPLINE_CURVE_WITH_KNOTS (+ rational) (LINE -> null, straight through endpoints, matching the
// Python serializer's geom=-1 for Line). Metadata: per-solid colour (STYLED_ITEM -> COLOUR_RGB,
// BFS style tree) + per-instance world placement matrices (CDSR/SRR/ITEM_DEFINED_TRANSFORMATION
// assembly graph) on NgeomRoot, and the file length-unit scale (LENGTH_UNIT/SI_UNIT) on NgeomDoc.
// Mixed-unit files (mm/cm/metre contexts in one file) are handled via per-representation rep_factor
// (rep_scale/global_scale), baked into placement translations + each solid's rotation block — so a
// metre-context part in an mm file is sized correctly instead of collapsing 1000x.
#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "../../cad/posix_compat.h"

#include "ngeom_bspline.h"  // BSplineCurve / BSplineSurface / expand_knots
#include "ngeom_topology.h" // pulls ngeom_curves.h / ngeom_surfaces.h / ngeom_math.h
#include "step_part21.h"

namespace adacpp::step {

namespace ng = adacpp::ngeom;

// 4x4 homogeneous matrix, column-major (glTF order): index = col*4 + row; columns 0..2 are the x/
// y/z basis vectors, column 3 is the translation.
using Mat4 = std::array<double, 16>;
namespace mat4 {
inline Mat4 ident() {
    Mat4 m{};
    m[0] = m[5] = m[10] = m[15] = 1.0;
    return m;
}
inline Mat4 from_frame(const ng::Frame &f) {
    Mat4 m{};
    m[0] = f.x.x;
    m[1] = f.x.y;
    m[2] = f.x.z;
    m[4] = f.y.x;
    m[5] = f.y.y;
    m[6] = f.y.z;
    m[8] = f.z.x;
    m[9] = f.z.y;
    m[10] = f.z.z;
    m[12] = f.o.x;
    m[13] = f.o.y;
    m[14] = f.o.z;
    m[15] = 1.0;
    return m;
}
inline Mat4 mul(const Mat4 &A, const Mat4 &B) { // A*B (math), column-major
    Mat4 C{};
    for (int j = 0; j < 4; ++j)
        for (int i = 0; i < 4; ++i) {
            double s = 0;
            for (int k = 0; k < 4; ++k)
                s += A[k * 4 + i] * B[j * 4 + k];
            C[j * 4 + i] = s;
        }
    return C;
}
// Inverse of a rigid transform [R|t] with orthonormal R (axis2 placements): [R^T | -R^T t].
inline Mat4 rigid_inverse(const Mat4 &M) {
    Mat4 r{};
    for (int a = 0; a < 3; ++a)
        for (int b = 0; b < 3; ++b)
            r[a * 4 + b] = M[b * 4 + a]; // transpose the 3x3
    double t[3] = {M[12], M[13], M[14]};
    for (int i = 0; i < 3; ++i) {
        double s = 0;
        for (int j = 0; j < 3; ++j)
            s += M[i * 4 + j] * t[j];
        r[12 + i] = -s;
    }
    r[15] = 1.0;
    return r;
}
inline bool is_identity(const Mat4 &m, double tol = 1e-12) {
    Mat4 I = ident();
    for (int k = 0; k < 16; ++k)
        if (std::abs(m[k] - I[k]) > tol)
            return false;
    return true;
}
} // namespace mat4

// Entity ids grouped by the types the resolver scans wholesale (roots + the metadata entities).
struct TypeLists {
    std::vector<long> roots;  // MANIFOLD_SOLID_BREP / SHELL_BASED_SURFACE_MODEL
    std::vector<long> styled; // STYLED_ITEM
    std::vector<long> absr;   // ADVANCED_BREP_SHAPE_REPRESENTATION
    std::vector<long> srr;    // SHAPE_REPRESENTATION_RELATIONSHIP
    std::vector<long> cdsr;   // CONTEXT_DEPENDENT_SHAPE_REPRESENTATION
    std::vector<long> sdr;    // SHAPE_DEFINITION_REPRESENTATION
    std::vector<long> units;  // LENGTH_UNIT complex / SI_UNIT
};

// Position of the terminating unquoted ';' within `s` (string/comment aware), or s.size() if none
// is contained yet (set `found` accordingly — the file path grows the read and retries).
inline size_t stmt_end_in(std::string_view s, bool &found) {
    const char *p = s.data(), *end = p + s.size(), *s0 = p;
    bool in_str = false;
    while (p < end) {
        char c = *p;
        if (in_str) {
            if (c == '\'') {
                if (p + 1 < end && p[1] == '\'') {
                    p += 2;
                    continue;
                }
                in_str = false;
            }
            ++p;
            continue;
        }
        if (c == '\'') {
            in_str = true;
            ++p;
            continue;
        }
        if (c == '/' && p + 1 < end && p[1] == '*') {
            p += 2;
            while (p + 1 < end && !(p[0] == '*' && p[1] == '/'))
                ++p;
            p = (p + 1 < end) ? p + 2 : end;
            continue;
        }
        if (c == ';') {
            found = true;
            return (size_t) (p - s0);
        }
        ++p;
    }
    found = false;
    return s.size();
}

// Single-pass offset index for streaming: id -> byte offset of its `#id=...;` statement, plus the
// type-classified id lists. Builds NO value trees — instances are parsed lazily on demand. The
// native peer of the Python reader's prepare_stream_index. Two modes: in-memory (scan + serve a
// caller-owned buffer) and file-backed (mmap to scan with free-behind, then keep the fd so each
// statement is pread on demand — the file lives in the OS page cache, NOT process RSS).
class StreamIndex {
public:
    std::vector<long> ids;    // sorted
    std::vector<size_t> offs; // statement-start offset, parallel to ids
    TypeLists lists;

    // In-memory: the caller keeps `b` alive; statement_bytes returns views into it.
    explicit StreamIndex(std::string_view b) : buf_(b) {
        scan(b, nullptr);
    }

    // File-backed, pread-only (NO mmap): for environments where mmap can't lazily page a large file
    // (wasm/OPFS — mmap would force the whole file into the heap). Scans the offset index by preading
    // chunks into a sliding buffer; the fd is kept for the same per-statement pread used by from_file.
    static StreamIndex from_file_pread(const std::string &path) {
        StreamIndex si;
        si.fd_ = ::open(path.c_str(), O_RDONLY);
        if (si.fd_ < 0)
            return si;
        struct stat st;
        if (::fstat(si.fd_, &st) != 0 || st.st_size == 0) {
            ::close(si.fd_);
            si.fd_ = -1;
            return si;
        }
        si.fsize_ = (size_t) st.st_size;
        si.scan_pread();
        return si;
    }

    // File-backed: mmap to scan (free-behind), munmap, keep the fd for per-statement pread.
    static StreamIndex from_file(const std::string &path) {
        StreamIndex si;
        si.fd_ = ::open(path.c_str(), O_RDONLY);
        if (si.fd_ < 0)
            return si;
        struct stat st;
        if (::fstat(si.fd_, &st) != 0 || st.st_size == 0) {
            ::close(si.fd_);
            si.fd_ = -1;
            return si;
        }
        si.fsize_ = (size_t) st.st_size;
        void *m = ::mmap(nullptr, si.fsize_, PROT_READ, MAP_PRIVATE, si.fd_, 0);
        if (m == MAP_FAILED) {
            ::close(si.fd_);
            si.fd_ = -1;
            return si;
        }
        ::madvise(m, si.fsize_, MADV_SEQUENTIAL);
        si.scan(std::string_view((const char *) m, si.fsize_), m); // free-behind during the scan
        ::munmap(m, si.fsize_);
        return si;
    }

    ~StreamIndex() {
        if (fd_ >= 0)
            ::close(fd_);
    }
    StreamIndex(StreamIndex &&o) noexcept {
        *this = std::move(o);
    }
    StreamIndex &operator=(StreamIndex &&o) noexcept {
        if (this != &o) {
            if (fd_ >= 0)
                ::close(fd_);
            ids = std::move(o.ids);
            offs = std::move(o.offs);
            lists = std::move(o.lists);
            buf_ = o.buf_;
            fd_ = o.fd_;
            fsize_ = o.fsize_;
            o.fd_ = -1;
        }
        return *this;
    }
    StreamIndex(const StreamIndex &) = delete;
    StreamIndex &operator=(const StreamIndex &) = delete;

    bool ok() const {
        return fd_ >= 0 || !buf_.empty();
    }

    // The statement text for `id` (no trailing ';'). In-memory mode returns a view into the source
    // buffer (scratch unused); file mode preads into `scratch` and returns a view into it (the
    // caller keeps scratch alive while it uses the parsed Instance). Empty if absent.
    std::string_view statement_bytes(long id, std::string &scratch) const {
        auto lo = std::lower_bound(ids.begin(), ids.end(), id);
        if (lo == ids.end() || *lo != id)
            return {};
        size_t off = offs[lo - ids.begin()];
        if (fd_ < 0) {
            bool found = false;
            return buf_.substr(off, stmt_end_in(buf_.substr(off), found));
        }
        return pread_statement(off, scratch);
    }

private:
    StreamIndex() = default;

    // pread the statement at `off` into `scratch`, growing the read until the terminating ';' is
    // contained (so the string/comment scan runs over one contiguous buffer — no cross-read state).
    std::string_view pread_statement(size_t off, std::string &scratch) const {
        size_t chunk = 1u << 16;
        while (true) {
            size_t want = std::min(chunk, fsize_ - off);
            scratch.resize(want);
            ssize_t r = ::pread(fd_, scratch.data(), want, (off_t) off);
            if (r <= 0) {
                scratch.clear();
                return {};
            }
            scratch.resize((size_t) r);
            bool found = false;
            size_t len = stmt_end_in(scratch, found);
            if (found) {
                scratch.resize(len);
                return scratch;
            }
            if ((size_t) r >= fsize_ - off) // read to EOF without a ';' (last/malformed statement)
                return scratch;
            chunk = chunk < (1u << 24) ? chunk * 2 : chunk + (1u << 24);
        }
    }

    void scan(std::string_view src, const void *mmap_base) {
        std::vector<std::pair<long, size_t>> index;
        const char *base = src.data(), *p = base, *end = base + src.size();
        size_t freed = 0;
        while (p < end) {
            p21_detail::skip_ws(p, end);
            if (p >= end)
                break;
            size_t off = (size_t) (p - base);
            bool in_str = false; // advance to ';'
            while (p < end) {
                char c = *p;
                if (in_str) {
                    if (c == '\'') {
                        if (p + 1 < end && p[1] == '\'') {
                            p += 2;
                            continue;
                        }
                        in_str = false;
                    }
                    ++p;
                    continue;
                }
                if (c == '\'') {
                    in_str = true;
                    ++p;
                    continue;
                }
                if (c == '/' && p + 1 < end && p[1] == '*') {
                    p += 2;
                    while (p + 1 < end && !(p[0] == '*' && p[1] == '/'))
                        ++p;
                    p = (p + 1 < end) ? p + 2 : end;
                    continue;
                }
                if (c == ';')
                    break;
                ++p;
            }
            classify(src, off, off, index);
            if (p < end)
                ++p; // consume ';'
            // Free-behind (file mode): drop scanned mmap pages so the scan doesn't fault in the
            // whole file. Forward-only scan, so dropped pages are never re-touched.
            if (mmap_base) {
                size_t cur = (size_t) (p - base);
                if (cur > freed + (32u << 20)) {
                    size_t up_to = (cur - (1u << 20)) & ~(size_t) 4095; // page-aligned, 1MB margin
                    if (up_to > freed) {
                        ::madvise((char *) const_cast<void *>(mmap_base) + freed, up_to - freed, MADV_DONTNEED);
                        freed = up_to;
                    }
                }
            }
        }
        std::sort(index.begin(), index.end());
        ids.reserve(index.size());
        offs.reserve(index.size());
        for (auto &[id, o] : index) {
            ids.push_back(id);
            offs.push_back(o);
        }
        std::sort(lists.roots.begin(), lists.roots.end()); // deterministic output order
        prune_boolean_operands();
    }

    // A BOOLEAN_RESULT's operands are themselves solid entities (extrusion/revolve/nested boolean) and
    // were classified as roots — drop them so only the top-level CSG tree is a root (else the operands
    // tessellate standalone alongside the boolean). Cheap substring gate so boolean-free files pay ~0.
    void prune_boolean_operands() {
        std::unordered_set<long> operands;
        std::string sc;
        for (long id : lists.roots) {
            std::string_view sv = statement_bytes(id, sc);
            if (sv.find("BOOLEAN_RESULT") == std::string_view::npos)
                continue;
            std::string bytes(sv);
            Instance ins;
            if (!parse_statement(bytes, ins) || ins.type != "BOOLEAN_RESULT")
                continue;
            if (ins.args.size() > 2 && ins.args[2].is_ref()) // ('',op,first,second)
                operands.insert(ins.args[2].i);
            if (ins.args.size() > 3 && ins.args[3].is_ref())
                operands.insert(ins.args[3].i);
        }
        if (!operands.empty())
            lists.roots.erase(
                std::remove_if(lists.roots.begin(), lists.roots.end(), [&](long id) { return operands.count(id) > 0; }),
                lists.roots.end());
    }

    // pread-based variant of scan() (NO mmap): reads the file in chunks into a sliding window and
    // classifies each statement from its start, so the offset index is built with bounded memory and
    // no address-space mapping (wasm/OPFS-safe — mmap there would force the whole file into the heap).
    // String/comment state is carried across chunk boundaries; produces the same ids/offs/lists as scan().
    void scan_pread() {
        std::vector<std::pair<long, size_t>> index;
        const size_t CHUNK = 4u << 20;
        std::string buf; // sliding window: buf[i] is file offset (base + i)
        size_t base = 0, pos = 0;
        bool eof = false;
        auto refill = [&]() -> bool { // append one chunk; false at EOF
            if (eof)
                return false;
            size_t old = buf.size();
            buf.resize(old + CHUNK);
            ssize_t r = ::pread(fd_, buf.data() + old, CHUNK, (off_t) (base + old));
            if (r <= 0) {
                buf.resize(old);
                eof = true;
                return false;
            }
            buf.resize(old + (size_t) r);
            if ((size_t) r < CHUNK)
                eof = true;
            return true;
        };
        refill();
        while (true) {
            // Amortized compaction: drop the fully-scanned prefix only once it exceeds a chunk, so the
            // erase is O(file/CHUNK) total — NOT per statement (that erase-from-front is O(N^2)).
            if (pos > CHUNK) {
                buf.erase(0, pos);
                base += pos;
                pos = 0;
            }
            { // advance to the next statement start
                const char *b = buf.data(), *p = b + pos, *e = b + buf.size();
                p21_detail::skip_ws(p, e);
                pos = (size_t) (p - b);
            }
            if (pos >= buf.size()) {
                if (!refill())
                    break;
                continue;
            }
            // scan to the terminating ';' (string/comment aware) with `pos` as the single advancing
            // cursor; refill across chunk boundaries, carrying in_str/in_comment + never splitting a
            // 2-char token ('' escape, /* */).
            const size_t stmt_start = pos;
            bool in_str = false, in_comment = false, found = false;
            while (!found) {
                const char *b = buf.data();
                bool need_more = false;
                while (pos < buf.size()) {
                    char c = b[pos];
                    if (in_comment) {
                        if (c == '*') {
                            if (pos + 1 >= buf.size() && !eof) {
                                need_more = true;
                                break;
                            }
                            if (pos + 1 < buf.size() && b[pos + 1] == '/') {
                                in_comment = false;
                                pos += 2;
                                continue;
                            }
                        }
                        ++pos;
                        continue;
                    }
                    if (in_str) {
                        if (c == '\'') {
                            if (pos + 1 >= buf.size() && !eof) {
                                need_more = true;
                                break;
                            }
                            if (pos + 1 < buf.size() && b[pos + 1] == '\'') {
                                pos += 2;
                                continue;
                            }
                            in_str = false;
                        }
                        ++pos;
                        continue;
                    }
                    if (c == '\'') {
                        in_str = true;
                        ++pos;
                        continue;
                    }
                    if (c == '/') {
                        if (pos + 1 >= buf.size() && !eof) {
                            need_more = true;
                            break;
                        }
                        if (pos + 1 < buf.size() && b[pos + 1] == '*') {
                            in_comment = true;
                            pos += 2;
                            continue;
                        }
                        ++pos;
                        continue;
                    }
                    if (c == ';') {
                        found = true;
                        break;
                    }
                    ++pos;
                }
                if (found)
                    break;
                if (need_more || !eof) {
                    if (!refill() && pos >= buf.size())
                        break; // EOF without ';' (last/malformed statement)
                    continue;
                }
                break; // eof, no ';'
            }
            classify(std::string_view(buf.data(), buf.size()), stmt_start, base + stmt_start, index);
            if (pos < buf.size())
                ++pos; // consume ';'
        }
        std::sort(index.begin(), index.end());
        ids.reserve(index.size());
        offs.reserve(index.size());
        for (auto &[id, o] : index) {
            ids.push_back(id);
            offs.push_back(o);
        }
        std::sort(lists.roots.begin(), lists.roots.end());
        prune_boolean_operands();
    }

    // Light classify: extract id + type keyword (no arg parse) from the statement at `off` in `src`.
    // `rec_off` is the FILE offset recorded in the index — equal to `off` for the whole-file mmap scan,
    // but base+off for the sliding-window pread scan (where `off` is window-relative).
    void classify(std::string_view src, size_t off, size_t rec_off, std::vector<std::pair<long, size_t>> &index) {
        const char *base = src.data(), *p = base + off, *end = base + src.size();
        if (*p != '#')
            return;
        ++p;
        long id = 0;
        bool any = false;
        while (p < end && *p >= '0' && *p <= '9') {
            id = id * 10 + (*p++ - '0');
            any = true;
        }
        if (!any)
            return;
        index.emplace_back(id, rec_off);
        p21_detail::skip_ws(p, end);
        if (p >= end || *p != '=')
            return;
        ++p;
        p21_detail::skip_ws(p, end);
        if (p >= end)
            return;
        if (*p == '(') { // complex: peek the first sub-record name
            ++p;
            p21_detail::skip_ws(p, end);
            const char *n0 = p;
            while (p < end && (std::isalnum((unsigned char) *p) || *p == '_'))
                ++p;
            if (std::string_view(n0, p - n0) == "LENGTH_UNIT")
                lists.units.push_back(id);
            return;
        }
        const char *t0 = p;
        while (p < end && (std::isalnum((unsigned char) *p) || *p == '_'))
            ++p;
        std::string_view t(t0, p - t0);
        if (t == "MANIFOLD_SOLID_BREP" || t == "SHELL_BASED_SURFACE_MODEL" || t == "BREP_WITH_VOIDS" ||
            t == "EXTRUDED_AREA_SOLID" || t == "REVOLVED_AREA_SOLID" || t == "BOOLEAN_RESULT")
            lists.roots.push_back(id);
        else if (t == "STYLED_ITEM")
            lists.styled.push_back(id);
        else if (t == "ADVANCED_BREP_SHAPE_REPRESENTATION" || t == "SHAPE_REPRESENTATION" ||
                 t == "MANIFOLD_SURFACE_SHAPE_REPRESENTATION" || t == "FACETED_BREP_SHAPE_REPRESENTATION" ||
                 t == "GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION")
            // Every rep type that can carry geometry roots — OCC writes shell models under a
            // plain SHAPE_REPRESENTATION. build_transform_map classifies geometry reps vs
            // placement reps by whether the items actually contain roots.
            lists.absr.push_back(id);
        else if (t == "SHAPE_REPRESENTATION_RELATIONSHIP")
            lists.srr.push_back(id);
        else if (t == "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION")
            lists.cdsr.push_back(id);
        else if (t == "SHAPE_DEFINITION_REPRESENTATION")
            lists.sdr.push_back(id);
        else if (t == "SI_UNIT")
            lists.units.push_back(id);
    }

    std::string_view buf_; // in-memory mode (empty in file mode)
    int fd_ = -1;          // file mode (>=0)
    size_t fsize_ = 0;
};

class Resolver {
public:
    // Full-parse mode (tests / small files): the caller pre-parsed every instance into `by_id`.
    explicit Resolver(const std::unordered_map<long, const Instance *> &by_id) : m_(&by_id) {}
    // Streaming mode (large files): instances are parsed lazily on demand via the offset index.
    explicit Resolver(const StreamIndex &idx) : idx_(&idx) {}

    // Resolve every root into one in-memory document. (Bounds parse memory via the offset index in
    // streaming mode, but still holds all roots' geometry — for fully bounded use, stream_step().)
    ng::NgeomDoc build() {
        TypeLists tl = idx_ ? idx_->lists : collect_type_lists();
        build_metadata(tl);
        ng::NgeomDoc doc;
        doc.unit_scale = unit_scale_;
        doc.roots.reserve(tl.roots.size());
        for (long sid : tl.roots) {
            ng::NgeomRoot root = resolve_root(sid);
            if (!root.id.empty())
                doc.roots.push_back(std::move(root));
        }
        return doc;
    }

    // Build the colour / product-name / transform / unit maps from the type-classified id lists,
    // then free the entities they parsed (the maps hold the results). Call once before resolving.
    // Opt into bounding parse_cache_ during giant-shell resolves — single-threaded callers only
    // (StepNgeomStream). See bound_parse_cache_.
    void enable_parse_cache_bounding() {
        bound_parse_cache_ = true;
    }

    void build_metadata(const TypeLists &tl) {
        build_colour_map(tl.styled);
        build_product_name_map(tl.sdr);
        unit_scale_ = detect_unit_scale(tl.units); // BEFORE build_transform_map — its rep_factor needs it
        build_transform_map(tl.roots, tl.absr, tl.srr, tl.cdsr);
        clear_geom_cache();
    }

    // Copy the already-built, read-only metadata maps from a master resolver so worker threads can
    // resolve+tessellate in parallel without each rebuilding (and re-reading) the metadata. Each
    // worker keeps its OWN per-solid caches; only these resolve-time maps are shared (by copy).
    void copy_metadata_from(const Resolver &src) {
        colour_map_ = src.colour_map_;
        xform_map_ = src.xform_map_;
        path_map_ = src.path_map_;
        name_of_rep_ = src.name_of_rep_;
        unit_scale_ = src.unit_scale_;
    }

    // Resolve one root solid -> NgeomRoot (geometry + colour + per-instance transforms + paths).
    ng::NgeomRoot resolve_root(long sid) {
        ng::NgeomRoot root;
        const Instance *in = inst(sid);
        if (!in || in->args.size() < 2)
            return root;
        root.id = solid_name(in, sid);
        // Collect the shell ids FIRST, then build — `shell_into` may clear parse_cache_ (the
        // bounded giant-shell path), which frees `in` itself. Iterating `in->args` *while* calling
        // shell_into would dereference a freed `in` on the 2nd+ shell (BREP_WITH_VOIDS voids /
        // SHELL_BASED_SURFACE_MODEL shell lists) — the crane's multi-shell solids segfaulted here,
        // while a single-shell MANIFOLD_SOLID_BREP (469826) happened to survive. Do not touch `in`
        // after this block.
        if (in->type == "EXTRUDED_AREA_SOLID") {
            // EXTRUDED_AREA_SOLID('', #profile, #position, #dir, depth) -> ng::ExtrusionN. Profile is
            // 2D-local; position places it; dir is local to position. Mirrors emit_extrusion / the
            // ng:: extrusion tessellation. (no shells)
            root.extrusion = build_extrusion(in);
            in = nullptr;
        } else if (in->type == "REVOLVED_AREA_SOLID") {
            root.revolve = build_revolve(in); // ('', #profile, #position, #axis1, angle)
            in = nullptr;
        } else if (in->type == "BOOLEAN_RESULT") {
            root.boolean = build_boolean(in); // ('', operator, #first, #second)
            in = nullptr;
        } else {
            std::vector<long> shell_ids;
            if (in->type == "MANIFOLD_SOLID_BREP") {
                if (in->args[1].is_ref())
                    shell_ids.push_back(in->args[1].i); // arg1 = the CLOSED_SHELL
            } else if (in->type == "BREP_WITH_VOIDS") {
                if (in->args[1].is_ref())
                    shell_ids.push_back(in->args[1].i); // arg1 = outer CLOSED_SHELL
                if (in->args.size() > 2 && in->args[2].kind == Kind::List)
                    for (const Value &v : in->args[2].items) // arg2 = void (ORIENTED_CLOSED_)SHELLs
                        if (v.is_ref())
                            shell_ids.push_back(v.i);
            } else if (in->args[1].kind == Kind::List) {
                for (const Value &sh : in->args[1].items) // arg1 = list of shells
                    if (sh.is_ref())
                        shell_ids.push_back(sh.i);
            }
            in = nullptr; // shell_into below may clear parse_cache_ and invalidate `in`
            for (long shid : shell_ids)
                shell_into(shid, root.faces);
        } // end B-rep branch
        auto cit = colour_map_.find(sid); // STYLED_ITEM colour usually targets the solid root
        if (cit != colour_map_.end()) {
            root.has_color = true;
            root.cr = cit->second.r;
            root.cg = cit->second.g;
            root.cb = cit->second.b;
            root.ca = cit->second.a;
        }
        auto xit = xform_map_.find(sid); // empty => single identity instance
        if (xit != xform_map_.end()) {
            root.transforms.reserve(xit->second.size());
            for (const Mat4 &m : xit->second) {
                std::array<float, 16> f;
                for (int k = 0; k < 16; ++k)
                    f[k] = (float) m[k];
                root.transforms.push_back(f);
            }
        }
        auto pit = path_map_.find(sid); // root-first (rep_id, product_name) levels
        if (pit != path_map_.end()) {
            root.instance_paths.reserve(pit->second.size());
            for (const Path &path : pit->second) {
                std::vector<std::pair<int, std::string>> levels;
                levels.reserve(path.size());
                for (const auto &[rid, nm] : path)
                    levels.emplace_back((int) rid, nm);
                root.instance_paths.push_back(std::move(levels));
            }
        }
        return root;
    }

    // Cheap per-solid cost proxy for LPT scheduling: count the faces in the solid's shell(s) WITHOUT
    // building geometry. Weak (Spearman ~0.47 vs tessellation time — the real cost is B-spline
    // boundary uv()-inversion, not face count) but ~free; kept as defensive ordering for the case
    // where the heaviest solid would otherwise be dispatched last. The engine-block tail is lever 2
    // (faster B-spline eval), not scheduling. Call clear_geom_cache() after the batch.
    size_t solid_face_count(long sid) {
        const Instance *in = inst(sid);
        if (!in || in->args.size() < 2)
            return 0;
        size_t n = 0;
        std::function<void(long)> count_shell = [&](long shell_id) {
            const Instance *sh = inst(shell_id);
            if (!sh)
                return;
            if (sh->type == "ORIENTED_CLOSED_SHELL") {
                if (sh->args.size() > 2 && sh->args[2].is_ref())
                    count_shell(sh->args[2].i);
            } else if (sh->args.size() > 1 && sh->args[1].kind == Kind::List) {
                n += sh->args[1].items.size();
            }
        };
        if (in->type == "MANIFOLD_SOLID_BREP") {
            if (in->args[1].is_ref())
                count_shell(in->args[1].i);
        } else if (in->type == "BREP_WITH_VOIDS") {
            if (in->args[1].is_ref())
                count_shell(in->args[1].i);
            if (in->args.size() > 2 && in->args[2].kind == Kind::List)
                for (const Value &v : in->args[2].items)
                    if (v.is_ref())
                        count_shell(v.i);
        } else if (in->args[1].kind == Kind::List) {
            for (const Value &sh : in->args[1].items)
                if (sh.is_ref())
                    count_shell(sh.i);
        }
        return n;
    }

    double unit_scale() const {
        return unit_scale_;
    }
    // Free the per-solid working set (parsed instances + geometry caches). Maps are retained.
    void clear_geom_cache() {
        parse_cache_.clear();
        surf_cache_.clear();
        curve_cache_.clear();
        face_cache_.clear();
    }

private:
    // Classify every instance by type (full-map mode only; streaming uses the StreamIndex lists).
    TypeLists collect_type_lists() {
        TypeLists tl;
        for (const auto &[id, in] : *m_) {
            if (in->complex) {
                if (sub(in, "LENGTH_UNIT"))
                    tl.units.push_back(id);
                continue;
            }
            std::string_view t = in->type;
            if (t == "MANIFOLD_SOLID_BREP" || t == "SHELL_BASED_SURFACE_MODEL" || t == "EXTRUDED_AREA_SOLID" ||
                t == "REVOLVED_AREA_SOLID")
                tl.roots.push_back(id);
            else if (t == "STYLED_ITEM")
                tl.styled.push_back(id);
            else if (t == "ADVANCED_BREP_SHAPE_REPRESENTATION" || t == "SHAPE_REPRESENTATION" ||
                     t == "MANIFOLD_SURFACE_SHAPE_REPRESENTATION" || t == "FACETED_BREP_SHAPE_REPRESENTATION" ||
                     t == "GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION")
                tl.absr.push_back(id);
            else if (t == "SHAPE_REPRESENTATION_RELATIONSHIP")
                tl.srr.push_back(id);
            else if (t == "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION")
                tl.cdsr.push_back(id);
            else if (t == "SHAPE_DEFINITION_REPRESENTATION")
                tl.sdr.push_back(id);
            else if (t == "SI_UNIT")
                tl.units.push_back(id);
        }
        std::sort(tl.roots.begin(), tl.roots.end());
        return tl;
    }

    const std::unordered_map<long, const Instance *> *m_ = nullptr; // full-map mode
    const StreamIndex *idx_ = nullptr;                              // streaming mode
    // Lazily parsed instances (streaming). Each owns the statement bytes its Instance's string_views
    // point into (needed in file/pread mode where the source isn't a stable mmap). unordered_map
    // node stability keeps both alive at a fixed address until cleared.
    struct Parsed {
        std::string bytes;
        Instance inst;
    };
    std::unordered_map<long, Parsed> parse_cache_;
    // When set, shell_into() periodically drops parse_cache_ during a huge shell to bound memory
    // (the parsed STEP statements pile up; the built ng:: geometry is retained separately). ONLY safe
    // on the single-threaded StepNgeomStream path: the multi-threaded mesh/glb/ifc path (stream_step /
    // the parallel IFC writer) would force extra concurrent re-parsing through the shared StreamIndex,
    // which is not thread-safe under that load. Default off — multi-threaded callers leave the original
    // (no-clear) behaviour and bound memory per-worker via clear_geom_cache() per solid instead.
    //
    // NOTE (do NOT delete): this is now a SAFE, load-bearing bound — the two earlier use-after-frees
    // were fixed (mid-shell clear gated single-threaded + resolve_root copies shell ids before any
    // clear). It is still REQUIRED by the single-threaded consumers: STEP->STEP (stream_step_to_step),
    // from_step Assembly hydrate (native_stream_read_step), the Python IFC fallback, and the serial
    // native IFC writer. Removing it re-OOMs 469826 (the 67 MB giant solid: 2035 -> 432 MB here). The
    // native PARALLEL IFC writer obviated it for the DEFAULT ifc path only; full removal awaits native
    // parallel writers for those other paths too.
    bool bound_parse_cache_ = false;
    double unit_scale_ = 1.0;
    std::unordered_map<long, std::shared_ptr<ng::Surface>> surf_cache_;
    std::unordered_map<long, std::shared_ptr<ng::Curve>> curve_cache_;
    std::unordered_map<long, std::shared_ptr<ng::FaceSurfaceN>> face_cache_;

public:
    // Zero-area faces with a `$` surface skipped at read time (see face()).
    long degenerate_faces_skipped_ = 0;

private:

    struct RGBA {
        float r, g, b, a;
    };
    std::unordered_map<long, RGBA> colour_map_; // STYLED_ITEM geometric-item id -> colour

    struct Edge {
        long rep_2, i1, i2;
    };
    using Path = std::vector<std::pair<long, std::string>>; // root-first (rep_id, product_name) levels
    std::unordered_map<long, std::vector<Edge>> edges_;     // rep_1 -> placement edges
    std::unordered_map<long, std::vector<Mat4>> xform_map_; // solid id -> world matrices (>1 placed)
    std::unordered_map<long, std::vector<Path>> path_map_;  // solid id -> per-instance assembly paths
    std::unordered_map<long, std::string> name_of_rep_;     // rep id -> product name

    // Resolve an entity. Full-map mode: a direct lookup. Streaming mode: parse on demand from the
    // offset index, caching the parsed Instance (kept alive until clear_geom_cache()).
    const Instance *inst(long id) {
        if (idx_) {
            auto it = parse_cache_.find(id);
            if (it != parse_cache_.end())
                return &it->second.inst;
            // pread (file mode) into ONE reusable buffer, then store a right-sized copy per entity —
            // otherwise each cached entity would retain the (large) initial pread capacity.
            std::string_view stmt = idx_->statement_bytes(id, pread_scratch_);
            if (stmt.empty())
                return nullptr;
            Parsed &slot = parse_cache_[id]; // stable node: bytes + inst stay put
            slot.bytes.assign(stmt.data(), stmt.size());
            if (!parse_statement(slot.bytes, slot.inst)) {
                parse_cache_.erase(id);
                return nullptr;
            }
            return &slot.inst; // inst's string_views point into the owned slot.bytes
        }
        auto it = m_->find(id);
        return it == m_->end() ? nullptr : it->second;
    }
    std::string pread_scratch_; // reusable per-statement pread buffer (file mode)

    // A Part-21 boolean enum is .T. / .F.; anything but explicit F is true (matches the readers).
    static bool enum_true(const Value &v) {
        return !(v.kind == Kind::Enum && v.s == "F");
    }

    static ng::Vec3 vec3_of(const Value &list) {
        ng::Vec3 r{0, 0, 0};
        if (list.kind == Kind::List) {
            if (list.items.size() > 0)
                r.x = list.items[0].as_double();
            if (list.items.size() > 1)
                r.y = list.items[1].as_double();
            if (list.items.size() > 2)
                r.z = list.items[2].as_double();
        }
        return r;
    }

    std::string solid_name(const Instance *in, long id) const {
        if (!in->args.empty() && in->args[0].kind == Kind::Str && !in->args[0].s.empty())
            return unescape(in->args[0].s);
        return "#" + std::to_string(id);
    }

    // CARTESIAN_POINT / DIRECTION (coord list at arg 1) or VERTEX_POINT (ref at arg 1).
    ng::Vec3 point(long id) {
        const Instance *in = inst(id);
        if (!in || in->args.size() < 2)
            return {};
        if (in->type == "VERTEX_POINT" && in->args[1].is_ref())
            return point(in->args[1].i);
        return vec3_of(in->args[1]);
    }

    // AXIS2_PLACEMENT_3D('',#loc,#axis?,#ref?) -> Frame. axis/ref may be $ (defaults +Z / +X).
    ng::Frame placement(long id) {
        const Instance *in = inst(id);
        if (!in || in->args.size() < 2)
            return {};
        ng::Vec3 loc = in->args[1].is_ref() ? point(in->args[1].i) : ng::Vec3{0, 0, 0};
        ng::Vec3 axis{0, 0, 1};
        if (in->args.size() > 2 && in->args[2].is_ref())
            axis = point(in->args[2].i);
        ng::Vec3 ref{1, 0, 0};
        if (in->args.size() > 3 && in->args[3].is_ref())
            ref = point(in->args[3].i);
        return ng::Frame::from_axis_ref(loc, axis, ref);
    }

    // ARBITRARY_CLOSED_PROFILE_DEF(.AREA.,'',#POLYLINE) -> planar profile FaceSurfaceN (z=0 poly loop).
    // First cut: polygonal (POLYLINE) outer curves; other profile curves yield null (-> not extrudable).
    std::shared_ptr<ng::FaceSurfaceN> profile_from_step(long pid) {
        const Instance *in = inst(pid);
        if (!in || in->type != "ARBITRARY_CLOSED_PROFILE_DEF" || in->args.size() < 3 || !in->args[2].is_ref())
            return nullptr;
        const Instance *cv = inst(in->args[2].i);
        if (!cv || cv->type != "POLYLINE" || cv->args.size() < 2 || cv->args[1].kind != Kind::List)
            return nullptr;
        std::vector<ng::Vec3> poly;
        for (const Value &pr : cv->args[1].items)
            if (pr.is_ref()) {
                ng::Vec3 p = point(pr.i);
                poly.push_back({p.x, p.y, 0});
            }
        if (poly.size() > 1 && (poly.front() - poly.back()).norm() < 1e-9)
            poly.pop_back();
        if (poly.size() < 3)
            return nullptr;
        auto prof = std::make_shared<ng::FaceSurfaceN>();
        prof->surface = std::make_shared<ng::PlaneSurface>(ng::Frame{});
        prof->same_sense = true;
        auto lp = std::make_shared<ng::LoopN>();
        lp->is_poly = true;
        lp->polygon = std::move(poly);
        ng::FaceBoundN fb;
        fb.loop = lp;
        fb.orientation = true;
        prof->bounds.push_back(fb);
        return prof;
    }

    // EXTRUDED_AREA_SOLID('', #profile, #position, #dir, depth) -> ng::ExtrusionN (null if no profile).
    std::shared_ptr<ng::ExtrusionN> build_extrusion(const Instance *in) {
        if (!in || in->args.size() < 5)
            return nullptr;
        auto prof = profile_from_step(in->args[1].is_ref() ? in->args[1].i : 0);
        if (!prof)
            return nullptr;
        auto ex = std::make_shared<ng::ExtrusionN>();
        ex->profile = prof;
        ex->frame = in->args[2].is_ref() ? placement(in->args[2].i) : ng::Frame{};
        ex->direction = in->args[3].is_ref() ? point(in->args[3].i) : ng::Vec3{0, 0, 1}; // DIRECTION via point()
        ex->depth = in->args[4].as_double();
        return ex;
    }

    // REVOLVED_AREA_SOLID('', #profile, #position, #axis1, angle) -> ng::RevolveN (null if no profile).
    std::shared_ptr<ng::RevolveN> build_revolve(const Instance *in) {
        if (!in || in->args.size() < 5)
            return nullptr;
        auto prof = profile_from_step(in->args[1].is_ref() ? in->args[1].i : 0);
        if (!prof)
            return nullptr;
        auto rv = std::make_shared<ng::RevolveN>();
        rv->profile = prof;
        rv->frame = in->args[2].is_ref() ? placement(in->args[2].i) : ng::Frame{};
        const Instance *ax = inst(in->args[3].is_ref() ? in->args[3].i : 0); // AXIS1_PLACEMENT('',#loc,#dir)
        rv->axis_origin =
            (ax && ax->args.size() > 1 && ax->args[1].is_ref()) ? point(ax->args[1].i) : ng::Vec3{0, 0, 0};
        rv->axis_dir = (ax && ax->args.size() > 2 && ax->args[2].is_ref()) ? point(ax->args[2].i) : ng::Vec3{0, 0, 1};
        rv->angle = in->args[4].as_double();
        return rv;
    }

    // A BOOLEAN_RESULT operand (#id) -> ng::SolidItemN (extrusion / revolve / nested boolean / brep).
    ng::SolidItemN build_solid_item(long id) {
        ng::SolidItemN it;
        const Instance *in = inst(id);
        if (!in)
            return it;
        if (in->type == "EXTRUDED_AREA_SOLID")
            it.extrusion = build_extrusion(in);
        else if (in->type == "REVOLVED_AREA_SOLID")
            it.revolve = build_revolve(in);
        else if (in->type == "BOOLEAN_RESULT")
            it.boolean = build_boolean(in);
        else if (in->type == "MANIFOLD_SOLID_BREP" && in->args.size() > 1 && in->args[1].is_ref())
            shell_into(in->args[1].i, it.faces);
        return it;
    }
    // BOOLEAN_RESULT('', operator, #first, #second) -> ng::BooleanN (null if an operand is unresolvable).
    std::shared_ptr<ng::BooleanN> build_boolean(const Instance *in) {
        if (!in || in->args.size() < 4)
            return nullptr;
        auto bn = std::make_shared<ng::BooleanN>();
        std::string_view op = (in->args[1].kind == Kind::Enum) ? in->args[1].s : std::string_view("DIFFERENCE");
        bn->op = (op == "UNION") ? 1 : (op == "INTERSECTION") ? 2 : 0;
        bn->a = build_solid_item(in->args[2].is_ref() ? in->args[2].i : 0);
        bn->b = build_solid_item(in->args[3].is_ref() ? in->args[3].i : 0);
        auto ok = [](const ng::SolidItemN &it) {
            return it.extrusion || it.revolve || it.boolean || !it.faces.empty();
        };
        if (!ok(bn->a) || !ok(bn->b))
            return nullptr;
        return bn;
    }

    // --- B-spline helpers ----------------------------------------------------------------
    static std::vector<double> reals(const Value &v) {
        std::vector<double> r;
        if (v.kind == Kind::List) {
            r.reserve(v.items.size());
            for (const Value &x : v.items)
                r.push_back(x.as_double());
        }
        return r;
    }
    static std::vector<int> ints(const Value &v) {
        std::vector<int> r;
        if (v.kind == Kind::List) {
            r.reserve(v.items.size());
            for (const Value &x : v.items)
                r.push_back((int) x.i);
        }
        return r;
    }
    // Find a named sub-record of a complex instance (e.g. "RATIONAL_B_SPLINE_SURFACE").
    static const std::vector<Value> *sub(const Instance *in, std::string_view name) {
        for (const auto &[n, a] : in->subs)
            if (n == name)
                return &a;
        return nullptr;
    }

    // Build a BSplineCurve from raw Part-21 args. closed is forced false to match ngeom_decode.h
    // (the decoder reads B_SPLINE_CURVE.closed_curve but ignores it for evaluation).
    std::shared_ptr<ng::Curve> build_bspline_curve(long deg, const Value &cp, const Value &mults, const Value &knots,
                                                   const Value *weights) {
        std::vector<ng::Vec3> control;
        if (cp.kind == Kind::List) {
            control.reserve(cp.items.size());
            for (const Value &r : cp.items)
                if (r.is_ref())
                    control.push_back(point(r.i));
        }
        std::vector<double> w = weights ? reals(*weights) : std::vector<double>{};
        return std::make_shared<ng::BSplineCurve>((int) deg, std::move(control), reals(knots), ints(mults),
                                                  std::move(w), false);
    }

    // Build a BSplineSurface from raw Part-21 args (grid = list of u-rows of control-point refs;
    // optional weights = same-shaped grid of reals). u_closed/v_closed are left default, matching
    // ngeom_decode.h (which reads but ignores them).
    std::shared_ptr<ng::Surface> build_bspline_surface(long u_deg, long v_deg, const Value &grid, const Value &u_mults,
                                                       const Value &v_mults, const Value &u_knots, const Value &v_knots,
                                                       const Value *weights) {
        auto s = std::make_shared<ng::BSplineSurface>();
        s->u_degree = (int) u_deg;
        s->v_degree = (int) v_deg;
        if (grid.kind != Kind::List || grid.items.empty())
            return s;
        s->nu = (int) grid.items.size();
        s->nv = grid.items[0].kind == Kind::List ? (int) grid.items[0].items.size() : 0;
        s->ctrl.resize((size_t) s->nu * s->nv);
        for (int iu = 0; iu < s->nu; ++iu) {
            const Value &row = grid.items[iu];
            if (row.kind != Kind::List)
                continue;
            for (int iv = 0; iv < s->nv && iv < (int) row.items.size(); ++iv)
                if (row.items[iv].is_ref())
                    s->ctrl[(size_t) iu * s->nv + iv] = point(row.items[iv].i);
        }
        s->Uu = ng::bspline_detail::expand_knots(reals(u_knots), ints(u_mults));
        s->Uv = ng::bspline_detail::expand_knots(reals(v_knots), ints(v_mults));
        if (weights && weights->kind == Kind::List) {
            s->weights.resize((size_t) s->nu * s->nv);
            for (int iu = 0; iu < s->nu && iu < (int) weights->items.size(); ++iu) {
                const Value &row = weights->items[iu];
                if (row.kind != Kind::List)
                    continue;
                for (int iv = 0; iv < s->nv && iv < (int) row.items.size(); ++iv)
                    s->weights[(size_t) iu * s->nv + iv] = row.items[iv].as_double();
            }
        }
        return s;
    }

    // A rational B-spline is a complex record splitting data across sub-types: B_SPLINE_SURFACE
    // (degrees, grid, flags), B_SPLINE_SURFACE_WITH_KNOTS (mults, knots), RATIONAL_B_SPLINE_SURFACE
    // (weights). Sub args have NO leading name string. Same shape for curves.
    std::shared_ptr<ng::Surface> bspline_surface_complex(const Instance *in) {
        const auto *bs = sub(in, "B_SPLINE_SURFACE");
        const auto *bk = sub(in, "B_SPLINE_SURFACE_WITH_KNOTS");
        const auto *rat = sub(in, "RATIONAL_B_SPLINE_SURFACE");
        if (!bs || !bk || bs->size() < 3 || bk->size() < 4)
            return nullptr;
        return build_bspline_surface((*bs)[0].i, (*bs)[1].i, (*bs)[2], (*bk)[0], (*bk)[1], (*bk)[2], (*bk)[3],
                                     (rat && !rat->empty()) ? &(*rat)[0] : nullptr);
    }
    std::shared_ptr<ng::Curve> bspline_curve_complex(const Instance *in) {
        const auto *bc = sub(in, "B_SPLINE_CURVE");
        const auto *bk = sub(in, "B_SPLINE_CURVE_WITH_KNOTS");
        const auto *rat = sub(in, "RATIONAL_B_SPLINE_CURVE");
        if (!bc || !bk || bc->size() < 2 || bk->size() < 2)
            return nullptr;
        return build_bspline_curve((*bc)[0].i, (*bc)[1], (*bk)[0], (*bk)[1],
                                   (rat && !rat->empty()) ? &(*rat)[0] : nullptr);
    }

    // Analytic + B-spline surfaces. Analytic entities are `(name, #position, <params...>)` with
    // A swept surface's generatrix profile. Unlike edge geometry, a swept profile that is a
    // LINE must materialise as a real LineCurve (curve() returns null for LINE — that's only safe
    // for edges, which discretize straight through their endpoints). Circles/ellipses/B-splines
    // reuse curve().
    std::shared_ptr<ng::Curve> swept_profile(long id) {
        const Instance *in = inst(id);
        if (in && in->type == "LINE" && in->args.size() >= 3 && in->args[1].is_ref() && in->args[2].is_ref()) {
            ng::Vec3 pnt = point(in->args[1].i);
            const Instance *vec = inst(in->args[2].i); // VECTOR('',#dir,mag)
            ng::Vec3 dir{0, 0, 1};
            if (vec && vec->args.size() >= 2 && vec->args[1].is_ref())
                dir = point(vec->args[1].i);
            return std::make_shared<ng::LineCurve>(pnt, dir);
        }
        return curve(id);
    }

    // #position an AXIS2_PLACEMENT_3D; B_SPLINE_SURFACE_WITH_KNOTS and rational complex records
    // carry their own data. Swept surfaces (SURFACE_OF_LINEAR_EXTRUSION / SURFACE_OF_REVOLUTION)
    // carry a profile curve + a sweep vector/axis instead of a placement.
    std::shared_ptr<ng::Surface> surface(long id) {
        auto c = surf_cache_.find(id);
        if (c != surf_cache_.end())
            return c->second;
        std::shared_ptr<ng::Surface> s;
        const Instance *in = inst(id);
        if (in && in->complex) {
            s = bspline_surface_complex(in);
        } else if (in) {
            std::string_view t = in->type;
            if (t == "B_SPLINE_SURFACE_WITH_KNOTS" && in->args.size() >= 12)
                s = build_bspline_surface(in->args[1].i, in->args[2].i, in->args[3], in->args[8], in->args[9],
                                          in->args[10], in->args[11], nullptr);
            else if (t == "SURFACE_OF_LINEAR_EXTRUSION" && in->args.size() >= 3 && in->args[1].is_ref() &&
                     in->args[2].is_ref()) {
                // SURFACE_OF_LINEAR_EXTRUSION('',#swept_curve,#VECTOR('',#dir,depth))
                auto prof = swept_profile(in->args[1].i);
                const Instance *vec = inst(in->args[2].i);
                ng::Vec3 dir{0, 0, 1};
                double depth = 1.0;
                if (vec && vec->args.size() >= 3 && vec->args[1].is_ref()) {
                    dir = point(vec->args[1].i);
                    depth = vec->args[2].as_double();
                }
                if (prof)
                    s = std::make_shared<ng::LinearExtrusionSurface>(prof, dir, depth);
            } else if (t == "SURFACE_OF_REVOLUTION" && in->args.size() >= 3 && in->args[1].is_ref() &&
                       in->args[2].is_ref()) {
                // SURFACE_OF_REVOLUTION('',#swept_curve,#AXIS1_PLACEMENT('',#loc,#axis))
                auto prof = swept_profile(in->args[1].i);
                const Instance *ax = inst(in->args[2].i);
                ng::Vec3 loc{0, 0, 0}, adir{0, 0, 1};
                if (ax && ax->args.size() >= 2 && ax->args[1].is_ref())
                    loc = point(ax->args[1].i);
                if (ax && ax->args.size() >= 3 && ax->args[2].is_ref())
                    adir = point(ax->args[2].i);
                if (prof)
                    s = std::make_shared<ng::RevolutionSurface>(prof, loc, adir);
            } else if (in->args.size() >= 2 && in->args[1].is_ref()) {
                ng::Frame fr = placement(in->args[1].i);
                if (t == "PLANE")
                    s = std::make_shared<ng::PlaneSurface>(fr);
                else if (t == "CYLINDRICAL_SURFACE" && in->args.size() >= 3)
                    s = std::make_shared<ng::CylinderSurface>(fr, in->args[2].as_double());
                else if (t == "CONICAL_SURFACE" && in->args.size() >= 4)
                    s = std::make_shared<ng::ConeSurface>(fr, in->args[2].as_double(), in->args[3].as_double());
                else if (t == "SPHERICAL_SURFACE" && in->args.size() >= 3)
                    s = std::make_shared<ng::SphereSurface>(fr, in->args[2].as_double());
                else if (t == "TOROIDAL_SURFACE" && in->args.size() >= 4)
                    s = std::make_shared<ng::TorusSurface>(fr, in->args[2].as_double(), in->args[3].as_double());
            }
        }
        surf_cache_[id] = s;
        return s;
    }

    // Edge geometry. CIRCLE/ELLIPSE -> conic curves; B_SPLINE_CURVE_WITH_KNOTS + rational complex
    // records -> B-spline curves; LINE (and anything unsupported) -> null, so the edge discretizes
    // straight through its endpoints (matches the Python serializer's geom=-1 for Line).
    std::shared_ptr<ng::Curve> curve(long id) {
        auto c = curve_cache_.find(id);
        if (c != curve_cache_.end())
            return c->second;
        std::shared_ptr<ng::Curve> cv;
        const Instance *in = inst(id);
        if (in && in->complex) {
            cv = bspline_curve_complex(in);
        } else if (in) {
            std::string_view t = in->type;
            if (t == "B_SPLINE_CURVE_WITH_KNOTS" && in->args.size() >= 8)
                cv = build_bspline_curve(in->args[1].i, in->args[2], in->args[6], in->args[7], nullptr);
            else if ((t == "SURFACE_CURVE" || t == "SEAM_CURVE" || t == "INTERSECTION_CURVE") && in->args.size() >= 2 &&
                     in->args[1].is_ref())
                // Curve-on-surface wrapper: ('',#curve_3d,(#pcurve,...),master). Unwrap to the 3D
                // curve (arg1). Exporters like OCC wrap CIRCLE/LINE in these; without unwrapping the
                // edge geometry is null and a closed (full-circle) edge collapses to a point.
                cv = curve(in->args[1].i);
            else if (in->args.size() >= 2 && in->args[1].is_ref()) {
                ng::Frame fr = placement(in->args[1].i);
                if (t == "CIRCLE" && in->args.size() >= 3)
                    cv = std::make_shared<ng::CircleCurve>(fr, in->args[2].as_double());
                else if (t == "ELLIPSE" && in->args.size() >= 4)
                    cv = std::make_shared<ng::EllipseCurve>(fr, in->args[2].as_double(), in->args[3].as_double());
            }
        }
        curve_cache_[id] = cv;
        return cv;
    }

    // ORIENTED_EDGE('',*,*,#edge,orient) wrapping EDGE_CURVE('',#v1,#v2,#geom,sense).
    // Field population mirrors ngeom_decode.h's ORIENTED_EDGE case exactly.
    ng::OrientedEdgeN oriented_edge(long id) {
        ng::OrientedEdgeN oe;
        const Instance *in = inst(id);
        if (!in || in->args.size() < 5)
            return oe;
        bool orientation = enum_true(in->args[4]);
        const Instance *ec = in->args[3].is_ref() ? inst(in->args[3].i) : nullptr;
        if (ec && ec->type == "EDGE_CURVE" && ec->args.size() >= 5) {
            if (ec->args[1].is_ref())
                oe.e_start = point(ec->args[1].i);
            if (ec->args[2].is_ref())
                oe.e_end = point(ec->args[2].i);
            if (ec->args[3].is_ref())
                oe.geometry = curve(ec->args[3].i);
            oe.same_sense = enum_true(ec->args[4]);
        }
        oe.orientation = orientation;
        oe.has_params = false;
        if (orientation) {
            oe.start = oe.e_start;
            oe.end = oe.e_end;
        } else {
            oe.start = oe.e_end;
            oe.end = oe.e_start;
        }
        return oe;
    }

    // EDGE_LOOP('',(#oe,...)) or POLY_LOOP('',(#pt,...)).
    std::shared_ptr<ng::LoopN> loop(long id) {
        auto lp = std::make_shared<ng::LoopN>();
        const Instance *in = inst(id);
        if (in && in->type == "POLY_LOOP" && in->args.size() > 1 && in->args[1].kind == Kind::List) {
            lp->is_poly = true;
            for (const Value &p : in->args[1].items)
                if (p.is_ref())
                    lp->polygon.push_back(point(p.i));
            return lp;
        }
        lp->is_poly = false;
        if (in && in->type == "EDGE_LOOP" && in->args.size() > 1 && in->args[1].kind == Kind::List)
            for (const Value &e : in->args[1].items)
                if (e.is_ref())
                    lp->edges.push_back(oriented_edge(e.i));
        return lp;
    }

    // FACE_OUTER_BOUND / FACE_BOUND('',#loop,orient).
    ng::FaceBoundN face_bound(long id) {
        ng::FaceBoundN b;
        const Instance *in = inst(id);
        if (!in || in->args.size() < 3)
            return b;
        if (in->args[1].is_ref())
            b.loop = loop(in->args[1].i);
        b.orientation = enum_true(in->args[2]);
        return b;
    }

    // Best-effort plane through a loop's sampled points (Newell). Null when the
    // loop is zero-area — e.g. two collinear LINE edges between the same two
    // vertices, a sliver face some exporters emit with a `$` surface.
    std::shared_ptr<ng::Surface> plane_from_loop(const ng::LoopN &lp) {
        std::vector<ng::Vec3> pts = lp.discretize(1e-2, 0.5);
        if (pts.size() >= 2 && (pts.front() - pts.back()).norm() < 1e-9)
            pts.pop_back();
        if (pts.size() < 3)
            return nullptr;
        ng::Vec3 c{0, 0, 0};
        for (const ng::Vec3 &p : pts)
            c = c + p;
        c = c * (1.0 / double(pts.size()));
        ng::Vec3 n{0, 0, 0};
        double perimeter = 0;
        for (size_t i = 0; i < pts.size(); ++i) {
            ng::Vec3 a = pts[i] - c, b = pts[(i + 1) % pts.size()] - c;
            n = n + a.cross(b);
            perimeter += (b - a).norm();
        }
        // |n| = 2*area; degenerate when the loop's area vanishes vs its extent.
        if (perimeter <= 0 || n.norm() <= 1e-9 * perimeter * perimeter)
            return nullptr;
        return std::make_shared<ng::PlaneSurface>(ng::Frame::from_axis_ref(c, n, ng::Vec3{0, 0, 0}));
    }

    // ADVANCED_FACE / FACE_SURFACE('',(#bound,...),#surface,same_sense).
    // A face whose surface is `$` (unset) gets a plane fitted through its outer
    // loop when the loop has area; a zero-area loop means the face carries no
    // geometry at all -> returns null and the shell skips it (what OCC's
    // ShapeFix does on import), instead of an ERR-ing surface:null drop later.
    std::shared_ptr<ng::FaceSurfaceN> face(long id) {
        auto c = face_cache_.find(id);
        if (c != face_cache_.end())
            return c->second;
        auto f = std::make_shared<ng::FaceSurfaceN>();
        const Instance *in = inst(id);
        if (in && (in->type == "ADVANCED_FACE" || in->type == "FACE_SURFACE") && in->args.size() >= 4) {
            if (in->args[1].kind == Kind::List)
                for (const Value &b : in->args[1].items)
                    if (b.is_ref())
                        f->bounds.push_back(face_bound(b.i));
            if (in->args[2].is_ref())
                f->surface = surface(in->args[2].i);
            f->same_sense = enum_true(in->args[3]);
            if (!f->surface && !f->bounds.empty() && f->bounds[0].loop) {
                f->surface = plane_from_loop(*f->bounds[0].loop);
                if (!f->surface) {
                    ++degenerate_faces_skipped_;
                    face_cache_[id] = nullptr;
                    return nullptr;
                }
            }
        }
        face_cache_[id] = f;
        return f;
    }

    // CLOSED_SHELL / OPEN_SHELL('',(#face,...)) -> append its faces. ORIENTED_CLOSED_SHELL('',*,
    // #base_shell,orient) (BREP_WITH_VOIDS void shells) -> dereference to its base shell.
    void shell_into(long id, std::vector<std::shared_ptr<ng::FaceSurfaceN>> &out) {
        const Instance *in = inst(id);
        if (!in)
            return;
        if (in->type == "ORIENTED_CLOSED_SHELL") {
            if (in->args.size() > 2 && in->args[2].is_ref())
                shell_into(in->args[2].i, out); // orientation flag ignored for tessellation
            return;
        }
        if (in->type != "CLOSED_SHELL" && in->type != "OPEN_SHELL")
            return;
        if (in->args.size() > 1 && in->args[1].kind == Kind::List) {
            if (!bound_parse_cache_) {
                // Default (incl. the multi-threaded mesh/glb path): unchanged.
                for (const Value &fr : in->args[1].items)
                    if (fr.is_ref())
                        if (auto fp = face(fr.i))
                            out.push_back(fp);
                return;
            }
            // Single-threaded bounded path (StepNgeomStream): copy the face refs out first (clearing
            // frees `in`), then drop parse_cache_ every 1024 faces so a giant shell's parsed STEP
            // statements don't pile up. Built ng:: geometry + surf/curve/face dedup caches persist.
            std::vector<long> face_ids;
            face_ids.reserve(in->args[1].items.size());
            for (const Value &fr : in->args[1].items)
                if (fr.is_ref())
                    face_ids.push_back(fr.i);
            for (size_t i = 0; i < face_ids.size(); ++i) {
                if (auto fp = face(face_ids[i]))
                    out.push_back(fp);
                if ((i & 1023u) == 1023u)
                    parse_cache_.clear();
            }
        }
    }

    // --- presentation colours (STYLED_ITEM -> COLOUR_RGB) ---------------------------------
    static void collect_refs(const std::vector<Value> &args, std::vector<long> &out) {
        for (const Value &v : args) {
            if (v.is_ref())
                out.push_back(v.i);
            else if (v.kind == Kind::List)
                collect_refs(v.items, out);
        }
    }
    static RGBA predefined_colour(std::string_view name, bool &found) {
        std::string n;
        n.reserve(name.size());
        for (char c : name)
            n.push_back((char) std::tolower((unsigned char) c));
        found = true;
        if (n == "red")
            return {1, 0, 0, 1};
        if (n == "green")
            return {0, 1, 0, 1};
        if (n == "blue")
            return {0, 0, 1, 1};
        if (n == "yellow")
            return {1, 1, 0, 1};
        if (n == "magenta")
            return {1, 0, 1, 1};
        if (n == "cyan")
            return {0, 1, 1, 1};
        if (n == "black")
            return {0, 0, 0, 1};
        if (n == "white")
            return {1, 1, 1, 1};
        found = false;
        return {0.5f, 0.5f, 0.5f, 1};
    }

    // BFS an entity's style reference tree for the first COLOUR_RGB / pre-defined colour and the
    // first SURFACE_STYLE_TRANSPARENT (alpha = 1 - transparency). Mirrors the Python _find_colour.
    bool find_colour(long ref_id, RGBA &out) {
        std::deque<std::pair<long, int>> q;
        q.push_back({ref_id, 0});
        std::unordered_set<long> seen;
        bool have_rgb = false, have_t = false;
        float r = 0, g = 0, b = 0, transp = 0;
        while (!q.empty()) {
            auto [rid, d] = q.front();
            q.pop_front();
            if (d > 12 || seen.count(rid))
                continue;
            seen.insert(rid);
            const Instance *rec = inst(rid);
            if (!rec || rec->complex)
                continue;
            std::string_view t = rec->type;
            if (t == "COLOUR_RGB" && !have_rgb && rec->args.size() >= 4) {
                r = (float) rec->args[1].as_double();
                g = (float) rec->args[2].as_double();
                b = (float) rec->args[3].as_double();
                have_rgb = true;
            } else if ((t == "DRAUGHTING_PRE_DEFINED_COLOUR" || t == "PRE_DEFINED_COLOUR") && !have_rgb) {
                if (!rec->args.empty() && rec->args[0].kind == Kind::Str) {
                    bool ok = false;
                    RGBA c = predefined_colour(rec->args[0].s, ok);
                    if (ok) {
                        r = c.r;
                        g = c.g;
                        b = c.b;
                        have_rgb = true;
                    }
                }
            } else if (t == "SURFACE_STYLE_TRANSPARENT" && !have_t) {
                for (const Value &v : rec->args)
                    if (v.kind == Kind::Real || v.kind == Kind::Int) {
                        transp = (float) v.as_double();
                        have_t = true;
                        break;
                    }
            } else {
                std::vector<long> refs;
                collect_refs(rec->args, refs);
                for (long c : refs)
                    q.push_back({c, d + 1});
            }
            if (have_rgb && have_t)
                break;
        }
        if (!have_rgb)
            return false;
        out = {r, g, b, have_t ? std::clamp(1.0f - transp, 0.0f, 1.0f) : 1.0f};
        return true;
    }

    // STYLED_ITEM('',(styles),#item) -> colour_map_[item id]. Search only the style refs.
    void build_colour_map(const std::vector<long> &styled) {
        for (long id : styled) {
            const Instance *in = inst(id);
            if (!in || in->complex || in->args.size() < 3)
                continue;
            const Value &item = in->args[2];
            if (!item.is_ref() || colour_map_.count(item.i))
                continue;
            std::vector<long> style_refs;
            if (in->args[1].kind == Kind::List)
                collect_refs(in->args[1].items, style_refs);
            else if (in->args[1].is_ref())
                style_refs.push_back(in->args[1].i);
            for (long rid : style_refs) {
                RGBA c{};
                if (find_colour(rid, c)) {
                    colour_map_[item.i] = c;
                    break;
                }
            }
        }
    }

    // --- length unit -> metres (LENGTH_UNIT / SI_UNIT) ------------------------------------
    static double si_prefix_factor(std::string_view p) {
        if (p == "EXA")
            return 1e18;
        if (p == "PETA")
            return 1e15;
        if (p == "TERA")
            return 1e12;
        if (p == "GIGA")
            return 1e9;
        if (p == "MEGA")
            return 1e6;
        if (p == "KILO")
            return 1e3;
        if (p == "HECTO")
            return 1e2;
        if (p == "DECA")
            return 1e1;
        if (p == "DECI")
            return 1e-1;
        if (p == "CENTI")
            return 1e-2;
        if (p == "MILLI")
            return 1e-3;
        if (p == "MICRO")
            return 1e-6;
        if (p == "NANO")
            return 1e-9;
        if (p == "PICO")
            return 1e-12;
        return 1.0;
    }
    // SI_UNIT args: (prefix?, .METRE.). Returns the scale to metres, or <0 if not a length unit.
    static double si_length_scale(const std::vector<Value> &a) {
        std::string_view prefix, unit;
        if (a.size() >= 2 && a[1].kind == Kind::Enum) {
            prefix = a[0].kind == Kind::Enum ? a[0].s : std::string_view{};
            unit = a[1].s;
        } else if (a.size() == 1 && a[0].kind == Kind::Enum) {
            unit = a[0].s;
        }
        if (unit != "METRE" && unit != "METER")
            return -1.0;
        return si_prefix_factor(prefix);
    }
    // The lowest-id LENGTH_UNIT in the model -> its SI scale to metres (default 1.0). The
    // ubiquitous form is a complex record ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );
    // a plain top-level SI_UNIT('',prefix,.METRE.) is also handled. Lowest-id for determinism
    // without materializing/sorting all ids.
    double detect_unit_scale(const std::vector<long> &units) {
        long best_id = 0;
        double best = 1.0;
        bool found = false;
        for (long id : units) {
            const Instance *in = inst(id);
            if (!in)
                continue;
            double s = -1.0;
            if (in->complex) {
                if (sub(in, "LENGTH_UNIT"))
                    if (const auto *si = sub(in, "SI_UNIT"))
                        s = si_length_scale(*si);
            } else if (in->type == "SI_UNIT" && !in->args.empty()) {
                std::vector<Value> a(in->args.begin() + 1, in->args.end()); // drop the name
                s = si_length_scale(a);
            }
            if (s > 0 && (!found || id < best_id)) {
                best_id = id;
                best = s;
                found = true;
            }
        }
        return best;
    }

    // Length-unit scale (to metres) of a representation's OWN context (its
    // GLOBAL_UNIT_ASSIGNED_CONTEXT unit list), or -1 if it declares no length unit. Mixed
    // mm/cm/metre files assign units per representation — mirrors the Python
    // _representation_length_scale.
    double representation_length_scale(long rep_id) {
        const Instance *rep = inst(rep_id);
        if (!rep || rep->complex || rep->args.empty())
            return -1.0;
        long ctx_id = -1; // the context is the last ref arg (after the items list)
        for (auto it = rep->args.rbegin(); it != rep->args.rend(); ++it)
            if (it->is_ref()) {
                ctx_id = it->i;
                break;
            }
        if (ctx_id < 0)
            return -1.0;
        const Instance *ctx = inst(ctx_id);
        if (!ctx)
            return -1.0;
        const std::vector<Value> *gua = ctx->complex
                                            ? sub(ctx, "GLOBAL_UNIT_ASSIGNED_CONTEXT")
                                            : (ctx->type == "GLOBAL_UNIT_ASSIGNED_CONTEXT" ? &ctx->args : nullptr);
        if (!gua || gua->empty() || (*gua)[0].kind != Kind::List)
            return -1.0;
        for (const Value &u : (*gua)[0].items) {
            if (!u.is_ref())
                continue;
            const Instance *uin = inst(u.i);
            if (!uin)
                continue;
            double s = -1.0;
            if (uin->complex) {
                if (sub(uin, "LENGTH_UNIT"))
                    if (const auto *si = sub(uin, "SI_UNIT"))
                        s = si_length_scale(*si);
            } else if (uin->type == "SI_UNIT" && !uin->args.empty()) {
                std::vector<Value> a(uin->args.begin() + 1, uin->args.end()); // drop the name
                s = si_length_scale(a);
            }
            if (s > 0)
                return s;
        }
        return -1.0;
    }

    // rep_id -> its length unit relative to the global unit (rep_scale / global_scale). 1.0 when the
    // rep is in the global unit (the common case). A metre-context fastener in an mm file gives 1000.
    std::unordered_map<long, double> rep_factor_cache_;
    double rep_factor(long rep_id) {
        if (rep_id < 0)
            return 1.0;
        auto it = rep_factor_cache_.find(rep_id);
        if (it != rep_factor_cache_.end())
            return it->second;
        double s = representation_length_scale(rep_id);
        double f = (s > 0 && std::abs(unit_scale_) > 1e-300) ? (s / unit_scale_) : 1.0;
        rep_factor_cache_[rep_id] = f;
        return f;
    }

    // --- assembly transforms (CDSR / SRR / MAPPED_ITEM-style placement graph) -------------
    // A port of the Python reader's _build_transform_map / _world_matrices, validated there
    // against OCC to 1e-9. NOTE: per-representation length scaling (rep_factor, for mixed mm/cm/m
    // files) is stubbed at 1.0 here — correct for uniform-unit files (the common case); mixed-unit
    // handling is a follow-up.
    Mat4 placement_mat4(long id) {
        return mat4::from_frame(placement(id));
    }

    std::pair<long, std::string> path_level(long rep_id) {
        auto it = name_of_rep_.find(rep_id);
        return {rep_id, it != name_of_rep_.end() ? it->second : ("asm_" + std::to_string(rep_id))};
    }

    // rep id -> product name, via SHAPE_DEFINITION_REPRESENTATION -> PRODUCT_DEFINITION_SHAPE ->
    // PRODUCT_DEFINITION -> ..._FORMATION -> PRODUCT. Mirrors the Python _build_product_name_map.
    void build_product_name_map(const std::vector<long> &sdr) {
        for (long id : sdr) {
            const Instance *in = inst(id);
            if (!in || in->complex || in->args.size() < 2)
                continue;
            if (!in->args[0].is_ref() || !in->args[1].is_ref() || name_of_rep_.count(in->args[1].i))
                continue;
            const Instance *pds = inst(in->args[0].i); // PRODUCT_DEFINITION_SHAPE(name, desc, #pd)
            const Instance *pd =
                (pds && pds->args.size() > 2 && pds->args[2].is_ref()) ? inst(pds->args[2].i) : nullptr;
            const Instance *pdf = (pd && pd->args.size() > 2 && pd->args[2].is_ref()) ? inst(pd->args[2].i) : nullptr;
            const Instance *prod =
                (pdf && pdf->args.size() > 2 && pdf->args[2].is_ref()) ? inst(pdf->args[2].i) : nullptr;
            if (prod && prod->args.size() >= 2)
                for (int k : {1, 0}) // PRODUCT(id, name, ...): prefer name, fall back to id
                    if (prod->args[k].kind == Kind::Str) {
                        std::string nm = unescape(prod->args[k].s);
                        // Match the Python path's cand.strip(): trim surrounding whitespace and skip
                        // whitespace-only candidates, so product names compare 1:1 (e.g. "M10 x 20").
                        const char *ws = " \t\n\r\f\v";
                        size_t a = nm.find_first_not_of(ws), b = nm.find_last_not_of(ws);
                        if (a == std::string::npos)
                            continue;
                        name_of_rep_[in->args[1].i] = nm.substr(a, b - a + 1);
                        break;
                    }
        }
    }

    // All (world matrix, assembly path) pairs reaching `rep_id` (which is rep_1 of edges). Each
    // edge T = inv(M(item_1)) * M(item_2) maps this rep's coords to its parent; recurse to a root
    // rep (no out-edges). path is root-first; this rep's level is appended after the parent's.
    std::vector<std::pair<Mat4, Path>> world_matrices(long rep_id, std::unordered_set<long> &on_path) {
        auto it = edges_.find(rep_id);
        if (it == edges_.end() || it->second.empty() || on_path.count(rep_id))
            return {{mat4::ident(), Path{path_level(rep_id)}}}; // root rep / cycle guard
        on_path.insert(rep_id);
        std::vector<std::pair<Mat4, Path>> out;
        for (const Edge &e : it->second) {
            // Placement translations are authored in their own rep's unit; bring each into the global
            // unit frame before composing so a mixed-unit child/parent pair maps consistently (R is
            // orthonormal, so rigid_inverse stays valid after only the translation is scaled).
            Mat4 mc = placement_mat4(e.i1), mp = placement_mat4(e.i2);
            const double fc = rep_factor(rep_id), fp = rep_factor(e.rep_2);
            mc[12] *= fc;
            mc[13] *= fc;
            mc[14] *= fc;
            mp[12] *= fp;
            mp[13] *= fp;
            mp[14] *= fp;
            Mat4 t_edge = mat4::mul(mat4::rigid_inverse(mc), mp);
            for (auto &[t_parent, parent_path] : world_matrices(e.rep_2, on_path)) {
                Path p = parent_path;
                p.push_back(path_level(rep_id));
                out.push_back({mat4::mul(t_parent, t_edge), std::move(p)});
            }
        }
        on_path.erase(rep_id);
        return out;
    }

    void build_transform_map(const std::vector<long> &root_ids, const std::vector<long> &absr_ids,
                             const std::vector<long> &srr_ids, const std::vector<long> &cdsr_ids) {
        std::unordered_set<long> root_set(root_ids.begin(), root_ids.end());
        // Geometry-rep items -> solid. `absr_ids` holds every rep type that can carry
        // geometry (ABSR, plain SHAPE_REPRESENTATION, ...); a rep counts as a geometry
        // rep only when its items actually contain a root — a plain SHAPE_REPRESENTATION
        // holding just axis placements is a placement rep and must stay OUT of the set,
        // or the SHAPE_REPRESENTATION_RELATIONSHIP side-picking below flips.
        // Two passes: specific rep types (ABSR & co.) claim their roots first; a plain
        // SHAPE_REPRESENTATION only maps roots nothing else claimed — an aggregating
        // root-level SR (AS1's 'design' listing every solid) must not steal a solid
        // from its own product's rep, or every solid names/paths as the assembly root.
        std::unordered_set<long> absr_set;
        std::unordered_map<long, long> geomrep_of_solid;
        for (int pass = 0; pass < 2; ++pass) {
            for (long id : absr_ids) {
                const Instance *in = inst(id);
                if (!in || in->complex)
                    continue;
                if ((in->type == "SHAPE_REPRESENTATION") != (pass == 1))
                    continue;
                if (in->args.size() < 2 || in->args[1].kind != Kind::List)
                    continue;
                bool has_root = false;
                for (const Value &it : in->args[1].items)
                    if (it.is_ref() && root_set.count(it.i)) {
                        geomrep_of_solid.emplace(it.i, id);
                        has_root = true;
                    }
                if (has_root)
                    absr_set.insert(id);
            }
        }
        // Standalone SHAPE_REPRESENTATION_RELATIONSHIP: the non-ABSR side is the placement rep.
        std::unordered_map<long, long> place_rep_of_geom;
        for (long id : srr_ids) {
            const Instance *in = inst(id);
            if (!in || in->complex || in->args.size() < 4)
                continue;
            if (!in->args[2].is_ref() || !in->args[3].is_ref())
                continue;
            long ra = in->args[2].i, rb = in->args[3].i;
            if (absr_set.count(ra))
                place_rep_of_geom.emplace(ra, rb);
            else if (absr_set.count(rb))
                place_rep_of_geom.emplace(rb, ra);
        }
        // CDSR -> complex rep_rel -> (rep_1, rep_2, IDT) edge.
        edges_.clear();
        for (long id : cdsr_ids) {
            const Instance *in = inst(id);
            if (!in || in->complex || in->args.empty())
                continue;
            if (!in->args[0].is_ref())
                continue;
            const Instance *rr = inst(in->args[0].i);
            if (!rr || !rr->complex)
                continue;
            const auto *rel = sub(rr, "REPRESENTATION_RELATIONSHIP");
            const auto *rrt = sub(rr, "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION");
            if (!rel || !rrt)
                continue;
            std::vector<long> refs; // rep_1, rep_2 = first two refs of REPRESENTATION_RELATIONSHIP
            for (const Value &v : *rel)
                if (v.is_ref())
                    refs.push_back(v.i);
            if (refs.size() < 2)
                continue;
            long idt_id = -1;
            for (const Value &v : *rrt)
                if (v.is_ref()) {
                    idt_id = v.i;
                    break;
                }
            const Instance *idt = idt_id >= 0 ? inst(idt_id) : nullptr;
            if (!idt || idt->complex || idt->type != "ITEM_DEFINED_TRANSFORMATION" || idt->args.size() < 4)
                continue;
            if (!idt->args[2].is_ref() || !idt->args[3].is_ref())
                continue;
            edges_[refs[0]].push_back({refs[1], idt->args[2].i, idt->args[3].i});
        }
        // Per solid: walk from its placement rep to the root, keep non-trivial placements.
        for (long sid : root_ids) {
            auto git = geomrep_of_solid.find(sid);
            if (git == geomrep_of_solid.end())
                continue; // flat file -> identity (single instance)
            long geom_rep = git->second;
            auto pit = place_rep_of_geom.find(geom_rep);
            long place_rep = pit != place_rep_of_geom.end() ? pit->second : geom_rep;
            std::unordered_set<long> on_path;
            auto pairs = world_matrices(place_rep, on_path);
            std::vector<Mat4> mats;
            std::vector<Path> paths;
            mats.reserve(pairs.size());
            paths.reserve(pairs.size());
            for (auto &[m, p] : pairs) {
                mats.push_back(m);
                paths.push_back(std::move(p));
            }
            // Bake the solid's own length-unit factor into each instance's rotation block (uniform
            // scale commutes with rotation, so this scales the local geometry into the global unit
            // frame). A non-global-unit solid with an identity placement thus gains a pure-scale
            // transform instead of collapsing to a near-zero-area sliver (mixed mm/cm/metre files).
            const double gf = rep_factor(geom_rep);
            if (std::abs(gf - 1.0) > 1e-12)
                for (Mat4 &m : mats)
                    for (int col = 0; col < 3; ++col)
                        for (int row = 0; row < 3; ++row)
                            m[col * 4 + row] *= gf;
            path_map_[sid] = std::move(paths); // always (>=1 path: the solid's own product)
            bool nontrivial = false;
            for (const Mat4 &m : mats)
                if (!mat4::is_identity(m)) {
                    nontrivial = true;
                    break;
                }
            if (nontrivial || mats.size() > 1) // skip pure-identity single instance
                xform_map_[sid] = std::move(mats);
        }
    }
};

// Convenience (full-parse): materialize every instance, then resolve into one document. Fine for
// small/medium files + tests; for large files use stream_step (bounded memory).
inline ng::NgeomDoc read_step_brep(std::string_view buf, std::vector<Instance> &store) {
    store.clear();
    scan_instances(buf, [&](const Instance &in) { store.push_back(in); });
    std::unordered_map<long, const Instance *> by_id;
    by_id.reserve(store.size() * 2);
    for (const Instance &in : store)
        by_id[in.id] = &in;
    Resolver r(by_id);
    return r.build();
}

// Streaming resolve (bounded memory): resolve + emit one root at a time, freeing each solid's
// working set after emit. Memory stays at the index (~12 bytes/instance) plus a single solid's
// entities — mirrors the Python prepare_stream_index / build_one_solid + the per-worker discipline.
// `emit(const NgeomRoot&, double unit_scale)`.
template <class F> inline void stream_step(const StreamIndex &idx, F &&emit) {
    Resolver r(idx);
    r.build_metadata(idx.lists);
    for (long sid : idx.lists.roots) {
        ng::NgeomRoot root = r.resolve_root(sid);
        if (!root.id.empty())
            emit(root, r.unit_scale());
        r.clear_geom_cache();
    }
}

// Convenience: stream an in-memory buffer (the caller keeps it alive).
template <class F> inline void stream_step(std::string_view buf, F &&emit) {
    StreamIndex idx(buf);
    stream_step(idx, std::forward<F>(emit));
}

} // namespace adacpp::step
