#include "artefact_io.h"

#include <cstring>
#include <filesystem>
#include <stdexcept>

namespace adacpp::fea {

namespace {

std::FILE *open_file(const std::string &path, const char *mode) {
#ifdef _WIN32
    // UTF-8 path -> wide, so non-ASCII paths open on Windows too.
    std::filesystem::path p(std::u8string(reinterpret_cast<const char8_t *>(path.data()), path.size()));
    std::wstring wmode(mode, mode + std::strlen(mode));
    return _wfopen(p.c_str(), wmode.c_str());
#else
    return std::fopen(path.c_str(), mode);
#endif
}

int seek_to(std::FILE *fh, uint64_t offset) {
#ifdef _WIN32
    return _fseeki64(fh, static_cast<long long>(offset), SEEK_SET);
#else
    return fseeko(fh, static_cast<off_t>(offset), SEEK_SET);
#endif
}

void put_u32le(std::vector<uint8_t> &out, uint32_t v) {
    for (int i = 0; i < 4; ++i)
        out.push_back(static_cast<uint8_t>((v >> (8 * i)) & 0xFFu));
}

uint32_t get_u32le(const uint8_t *p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

void append_hex4(std::string &out, uint32_t cu) {
    static const char *HEX = "0123456789abcdef"; // Python writes lower-case hex
    out += "\\u";
    out += HEX[(cu >> 12) & 0xF];
    out += HEX[(cu >> 8) & 0xF];
    out += HEX[(cu >> 4) & 0xF];
    out += HEX[cu & 0xF];
}

void append_utf8(std::string &out, uint32_t cp) {
    if (cp < 0x80) {
        out += static_cast<char>(cp);
    } else if (cp < 0x800) {
        out += static_cast<char>(0xC0 | (cp >> 6));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else if (cp < 0x10000) {
        out += static_cast<char>(0xE0 | (cp >> 12));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else {
        out += static_cast<char>(0xF0 | (cp >> 18));
        out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    }
}

// Minimal parser for the flat header object: string and integer values are kept; anything else is
// skipped structurally (so an additive key in a future writer does not break the reader).
struct FlatJson {
    const char *p;
    const char *end;

    [[noreturn]] void fail(const char *what) {
        throw std::runtime_error(std::string("blob header JSON: ") + what);
    }
    void ws() {
        while (p < end && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r'))
            ++p;
    }
    void expect(char c) {
        ws();
        if (p >= end || *p != c)
            fail("unexpected character");
        ++p;
    }
    uint32_t hex4() {
        if (end - p < 4)
            fail("short \\u escape");
        uint32_t v = 0;
        for (int i = 0; i < 4; ++i) {
            const char c = *p++;
            v <<= 4;
            if (c >= '0' && c <= '9')
                v |= static_cast<uint32_t>(c - '0');
            else if (c >= 'a' && c <= 'f')
                v |= static_cast<uint32_t>(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F')
                v |= static_cast<uint32_t>(c - 'A' + 10);
            else
                fail("bad \\u escape");
        }
        return v;
    }
    std::string string() {
        expect('"');
        std::string out;
        while (true) {
            if (p >= end)
                fail("unterminated string");
            const char c = *p++;
            if (c == '"')
                return out;
            if (c != '\\') {
                out += c;
                continue;
            }
            if (p >= end)
                fail("unterminated escape");
            const char e = *p++;
            switch (e) {
            case '"':
                out += '"';
                break;
            case '\\':
                out += '\\';
                break;
            case '/':
                out += '/';
                break;
            case 'b':
                out += '\b';
                break;
            case 'f':
                out += '\f';
                break;
            case 'n':
                out += '\n';
                break;
            case 'r':
                out += '\r';
                break;
            case 't':
                out += '\t';
                break;
            case 'u': {
                uint32_t cp = hex4();
                if (cp >= 0xD800 && cp < 0xDC00 && end - p >= 6 && p[0] == '\\' && p[1] == 'u') {
                    p += 2;
                    const uint32_t lo = hex4();
                    if (lo >= 0xDC00 && lo < 0xE000)
                        cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                    else
                        fail("unpaired surrogate");
                }
                append_utf8(out, cp);
                break;
            }
            default:
                fail("bad escape");
            }
        }
    }
    // Returns true and sets `out` for a non-negative integer; skips any other value.
    bool value(std::string *str_out, int64_t *int_out, bool *is_str) {
        ws();
        if (p >= end)
            fail("missing value");
        if (*p == '"') {
            *str_out = string();
            *is_str = true;
            return true;
        }
        *is_str = false;
        if (*p == '-' || (*p >= '0' && *p <= '9')) {
            const char *start = p;
            bool neg = false;
            if (*p == '-') {
                neg = true;
                ++p;
            }
            int64_t v = 0;
            bool integral = true;
            while (p < end && *p >= '0' && *p <= '9')
                v = v * 10 + (*p++ - '0');
            while (p < end &&
                   (*p == '.' || *p == 'e' || *p == 'E' || *p == '+' || *p == '-' || (*p >= '0' && *p <= '9'))) {
                integral = false;
                ++p;
            }
            if (p == start)
                fail("bad number");
            *int_out = neg ? -v : v;
            return integral;
        }
        // true / false / null / nested containers: skip.
        if (*p == '{' || *p == '[') {
            int depth = 0;
            bool in_str = false;
            for (; p < end; ++p) {
                if (in_str) {
                    if (*p == '\\')
                        ++p;
                    else if (*p == '"')
                        in_str = false;
                    continue;
                }
                if (*p == '"')
                    in_str = true;
                else if (*p == '{' || *p == '[')
                    ++depth;
                else if (*p == '}' || *p == ']') {
                    if (--depth == 0) {
                        ++p;
                        break;
                    }
                }
            }
            return false;
        }
        while (p < end && *p != ',' && *p != '}')
            ++p;
        return false;
    }
};

uint64_t as_u64(int64_t v, const char *key) {
    if (v < 0)
        throw std::runtime_error(std::string("blob header: negative ") + key);
    return static_cast<uint64_t>(v);
}

} // namespace

BlobHeader BlobHeader::with_steps(uint64_t steps) const {
    BlobHeader h = *this;
    h.n_steps = steps;
    return h;
}

BlobHeader BlobHeader::with_components(uint64_t ncomp) const {
    BlobHeader h = *this;
    h.n_components = ncomp;
    h.stride_bytes = h.rows() * ncomp * 4;
    return h;
}

BlobHeader make_afbl_header(const std::string &name, uint64_t n_steps, uint64_t n_points, uint64_t n_components) {
    BlobHeader h;
    h.kind = BlobKind::AFBL;
    h.name = name;
    h.n_steps = n_steps;
    h.n_points = n_points;
    h.n_components = n_components;
    h.stride_bytes = n_points * n_components * 4;
    return h;
}

BlobHeader make_afel_header(const std::string &name, const std::string &elem_type, uint64_t n_steps,
                            uint64_t n_elements, uint64_t n_ips, uint64_t n_components) {
    BlobHeader h;
    h.kind = BlobKind::AFEL;
    h.name = name;
    h.elem_type = elem_type;
    h.n_steps = n_steps;
    h.n_elements = n_elements;
    h.n_ips = n_ips;
    h.n_components = n_components;
    h.stride_bytes = n_elements * n_ips * n_components * 4;
    return h;
}

std::string python_json_string(const std::string &s) {
    std::string out = "\"";
    std::size_t i = 0;
    const std::size_t n = s.size();
    while (i < n) {
        const unsigned char c = static_cast<unsigned char>(s[i]);
        uint32_t cp;
        std::size_t len;
        if (c < 0x80) {
            cp = c;
            len = 1;
        } else if ((c & 0xE0) == 0xC0) {
            cp = c & 0x1F;
            len = 2;
        } else if ((c & 0xF0) == 0xE0) {
            cp = c & 0x0F;
            len = 3;
        } else if ((c & 0xF8) == 0xF0) {
            cp = c & 0x07;
            len = 4;
        } else {
            throw std::invalid_argument("field name is not valid UTF-8");
        }
        if (i + len > n)
            throw std::invalid_argument("field name is not valid UTF-8");
        for (std::size_t k = 1; k < len; ++k) {
            const unsigned char cc = static_cast<unsigned char>(s[i + k]);
            if ((cc & 0xC0) != 0x80)
                throw std::invalid_argument("field name is not valid UTF-8");
            cp = (cp << 6) | (cc & 0x3F);
        }
        i += len;

        switch (cp) {
        case '"':
            out += "\\\"";
            continue;
        case '\\':
            out += "\\\\";
            continue;
        case '\b':
            out += "\\b";
            continue;
        case '\f':
            out += "\\f";
            continue;
        case '\n':
            out += "\\n";
            continue;
        case '\r':
            out += "\\r";
            continue;
        case '\t':
            out += "\\t";
            continue;
        default:
            break;
        }
        if (cp >= 0x20 && cp <= 0x7E) {
            out += static_cast<char>(cp);
        } else if (cp < 0x10000) {
            append_hex4(out, cp);
        } else {
            const uint32_t v = cp - 0x10000;
            append_hex4(out, 0xD800 | (v >> 10));
            append_hex4(out, 0xDC00 | (v & 0x3FF));
        }
    }
    out += '"';
    return out;
}

std::string header_json(const BlobHeader &h) {
    std::string j = "{\"name\":" + python_json_string(h.name);
    if (h.kind == BlobKind::AFEL) {
        j += ",\"elem_type\":" + python_json_string(h.elem_type);
        j += ",\"n_steps\":" + std::to_string(h.n_steps);
        j += ",\"n_elements\":" + std::to_string(h.n_elements);
        j += ",\"n_ips\":" + std::to_string(h.n_ips);
    } else {
        j += ",\"n_steps\":" + std::to_string(h.n_steps);
        j += ",\"n_points\":" + std::to_string(h.n_points);
    }
    j += ",\"n_components\":" + std::to_string(h.n_components);
    j += ",\"dtype\":" + python_json_string(h.dtype);
    j += ",\"stride_bytes\":" + std::to_string(h.stride_bytes);
    j += '}';
    return j;
}

std::vector<uint8_t> encode_header(const BlobHeader &h) {
    const std::string j = header_json(h);
    if (12 + j.size() > BLOB_HEADER_BYTES)
        throw std::length_error("blob header for field '" + h.name + "' doesn't fit in 1024 bytes (needs " +
                                std::to_string(12 + j.size()) + ")");
    std::vector<uint8_t> out;
    out.reserve(BLOB_HEADER_BYTES);
    const char *magic = h.kind == BlobKind::AFEL ? AFEL_MAGIC : AFBL_MAGIC;
    out.insert(out.end(), magic, magic + 4);
    put_u32le(out, h.kind == BlobKind::AFEL ? AFEL_VERSION : AFBL_VERSION);
    put_u32le(out, static_cast<uint32_t>(j.size()));
    out.insert(out.end(), j.begin(), j.end());
    out.resize(BLOB_HEADER_BYTES, 0);
    return out;
}

BlobHeader decode_header(const uint8_t *prefix, std::size_t len) {
    if (len < 12)
        throw std::runtime_error("blob header: short read");
    BlobHeader h;
    if (std::memcmp(prefix, AFBL_MAGIC, 4) == 0)
        h.kind = BlobKind::AFBL;
    else if (std::memcmp(prefix, AFEL_MAGIC, 4) == 0)
        h.kind = BlobKind::AFEL;
    else
        throw std::runtime_error("not an AFBL/AFEL blob (bad magic)");
    const uint32_t version = get_u32le(prefix + 4);
    if (version != 1)
        throw std::runtime_error("blob version " + std::to_string(version) + ", expected 1");
    const uint32_t json_len = get_u32le(prefix + 8);
    if (12 + static_cast<std::size_t>(json_len) > len || 12 + static_cast<std::size_t>(json_len) > BLOB_HEADER_BYTES)
        throw std::runtime_error("blob header: JSON length out of range");

    FlatJson pj{reinterpret_cast<const char *>(prefix + 12), reinterpret_cast<const char *>(prefix + 12 + json_len)};
    pj.expect('{');
    pj.ws();
    bool have_stride = false;
    if (pj.p < pj.end && *pj.p == '}') {
        ++pj.p;
    } else {
        while (true) {
            const std::string key = pj.string();
            pj.expect(':');
            std::string sv;
            int64_t iv = 0;
            bool is_str = false;
            const bool ok = pj.value(&sv, &iv, &is_str);
            if (ok && is_str) {
                if (key == "name")
                    h.name = sv;
                else if (key == "elem_type")
                    h.elem_type = sv;
                else if (key == "dtype")
                    h.dtype = sv;
            } else if (ok) {
                if (key == "n_steps")
                    h.n_steps = as_u64(iv, "n_steps");
                else if (key == "n_points")
                    h.n_points = as_u64(iv, "n_points");
                else if (key == "n_elements")
                    h.n_elements = as_u64(iv, "n_elements");
                else if (key == "n_ips")
                    h.n_ips = as_u64(iv, "n_ips");
                else if (key == "n_components")
                    h.n_components = as_u64(iv, "n_components");
                else if (key == "stride_bytes") {
                    h.stride_bytes = as_u64(iv, "stride_bytes");
                    have_stride = true;
                }
            }
            pj.ws();
            if (pj.p < pj.end && *pj.p == ',') {
                ++pj.p;
                continue;
            }
            pj.expect('}');
            break;
        }
    }
    if (h.dtype != "float32")
        throw std::runtime_error("blob dtype '" + h.dtype + "' is not float32");
    if (!have_stride)
        h.stride_bytes = h.floats_per_step() * 4;
    if (h.stride_bytes != h.floats_per_step() * 4)
        throw std::runtime_error("blob header: stride_bytes does not match the declared shape");
    return h;
}

BlobHeader read_header(const std::string &path) {
    std::FILE *fh = open_file(path, "rb");
    if (!fh)
        throw std::runtime_error("cannot open " + path);
    uint8_t buf[BLOB_HEADER_BYTES];
    const std::size_t got = std::fread(buf, 1, BLOB_HEADER_BYTES, fh);
    std::fclose(fh);
    return decode_header(buf, got);
}

void write_blob(const std::string &path, const BlobHeader &h, const float *data) {
    BlobWriter w(path, h);
    const uint64_t per = h.floats_per_step();
    for (uint64_t s = 0; s < h.n_steps; ++s)
        w.write_step(data + s * per);
    w.finish();
}

void write_afbl(const std::string &path, const std::string &name, const float *data, uint64_t n_steps,
                uint64_t n_points, uint64_t n_components) {
    write_blob(path, make_afbl_header(name, n_steps, n_points, n_components), data);
}

void write_afel(const std::string &path, const std::string &name, const std::string &elem_type, const float *data,
                uint64_t n_steps, uint64_t n_elements, uint64_t n_ips, uint64_t n_components) {
    write_blob(path, make_afel_header(name, elem_type, n_steps, n_elements, n_ips, n_components), data);
}

void write_bytes(const std::string &path, const void *data, std::size_t n) {
    std::FILE *fh = open_file(path, "wb");
    if (!fh)
        throw std::runtime_error("cannot create " + path);
    const bool ok = std::fwrite(data, 1, n, fh) == n;
    const bool closed = std::fclose(fh) == 0;
    if (!ok || !closed)
        throw std::runtime_error(path + ": write failed");
}

BlobReader::BlobReader(const std::string &path) : path_(path) {
    fh_ = open_file(path, "rb");
    if (!fh_)
        throw std::runtime_error("cannot open " + path);
    uint8_t buf[BLOB_HEADER_BYTES];
    const std::size_t got = std::fread(buf, 1, BLOB_HEADER_BYTES, fh_);
    try {
        header_ = decode_header(buf, got);
    } catch (const std::exception &e) {
        std::fclose(fh_);
        fh_ = nullptr;
        throw std::runtime_error(path + ": " + e.what());
    }
}

BlobReader::~BlobReader() {
    if (fh_)
        std::fclose(fh_);
}

void BlobReader::read_step(uint64_t step, float *out) {
    if (step >= header_.n_steps)
        throw std::out_of_range(path_ + ": step " + std::to_string(step) + " out of range (n_steps " +
                                std::to_string(header_.n_steps) + ")");
    const uint64_t offset = BLOB_HEADER_BYTES + step * header_.stride_bytes;
    if (seek_to(fh_, offset) != 0)
        throw std::runtime_error(path_ + ": seek failed");
    const std::size_t n = static_cast<std::size_t>(header_.floats_per_step());
    if (std::fread(out, sizeof(float), n, fh_) != n)
        throw std::runtime_error(path_ + ": short read for step " + std::to_string(step));
}

BlobWriter::BlobWriter(const std::string &path, const BlobHeader &h) : path_(path), header_(h) {
    const std::vector<uint8_t> prefix = encode_header(h); // throws before creating the file
    fh_ = open_file(path, "wb");
    if (!fh_)
        throw std::runtime_error("cannot create " + path);
    if (std::fwrite(prefix.data(), 1, prefix.size(), fh_) != prefix.size()) {
        std::fclose(fh_);
        fh_ = nullptr;
        throw std::runtime_error(path + ": write failed");
    }
}

BlobWriter::~BlobWriter() {
    if (fh_)
        std::fclose(fh_);
}

void BlobWriter::write_step(const float *stride) {
    if (!fh_)
        throw std::runtime_error(path_ + ": writer is closed");
    if (written_ >= header_.n_steps)
        throw std::runtime_error(path_ + ": more steps written than the header declares");
    const std::size_t n = static_cast<std::size_t>(header_.floats_per_step());
    if (std::fwrite(stride, sizeof(float), n, fh_) != n)
        throw std::runtime_error(path_ + ": write failed");
    ++written_;
}

void BlobWriter::finish() {
    if (!fh_)
        return;
    const int rc = std::fclose(fh_);
    fh_ = nullptr;
    if (rc != 0)
        throw std::runtime_error(path_ + ": close failed");
    if (written_ != header_.n_steps)
        throw std::runtime_error(path_ + ": wrote " + std::to_string(written_) + " steps, header declares " +
                                 std::to_string(header_.n_steps));
}

} // namespace adacpp::fea
