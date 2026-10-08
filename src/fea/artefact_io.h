// AFBL / AFEL field blobs: read and write, byte-identical to adapy's ada/fem/results/artefacts.
//
// Layout (formats.py): 4-byte magic, uint32 version (LE), uint32 json_len (LE), the JSON header,
// zero-padded to exactly 1024 bytes; then the payload, float32 [n_steps x rows x n_components].
//
// The JSON is what `json.dumps(obj, separators=(",", ":"))` writes for these key orders:
//   AFBL  {"name","n_steps","n_points","n_components","dtype","stride_bytes"}
//   AFEL  {"name","elem_type","n_steps","n_elements","n_ips","n_components","dtype","stride_bytes"}
// including Python's default ensure_ascii escaping (non-ASCII -> \uXXXX, lower-case hex, surrogate
// pairs above the BMP; \" \\ \b \f \n \r \t; other code points outside ' '..'~' -> \u00XX), so a
// materialised case written here and one written by the server are the same file, byte for byte.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "fea_arrays.h"

namespace adacpp::fea {

enum class BlobKind { AFBL, AFEL };

struct BlobHeader {
    BlobKind kind = BlobKind::AFBL;
    std::string name;
    std::string elem_type; // AFEL only
    uint64_t n_steps = 0;
    uint64_t n_points = 0;   // AFBL: rows
    uint64_t n_elements = 0; // AFEL
    uint64_t n_ips = 0;      // AFEL; rows = n_elements * n_ips
    uint64_t n_components = 0;
    std::string dtype = "float32";
    uint64_t stride_bytes = 0;

    uint64_t rows() const {
        return kind == BlobKind::AFBL ? n_points : n_elements * n_ips;
    }
    uint64_t floats_per_step() const {
        return rows() * n_components;
    }
    // Same blob, a different step count / name / component count (stride recomputed).
    BlobHeader with_steps(uint64_t steps) const;
    BlobHeader with_components(uint64_t ncomp) const;
};

BlobHeader make_afbl_header(const std::string &name, uint64_t n_steps, uint64_t n_points, uint64_t n_components);
BlobHeader make_afel_header(const std::string &name, const std::string &elem_type, uint64_t n_steps,
                            uint64_t n_elements, uint64_t n_ips, uint64_t n_components);

// Python json.dumps(s) (ensure_ascii=True) of a UTF-8 string, quotes included.
std::string python_json_string(const std::string &utf8);

// The exact 1024-byte prefix. Throws std::length_error when the JSON does not fit (as adapy raises).
std::vector<uint8_t> encode_header(const BlobHeader &h);
// The header JSON alone (what adapy's json.dumps produced).
std::string header_json(const BlobHeader &h);

// Parse a 1024-byte prefix; throws std::runtime_error on a bad magic / version / JSON.
BlobHeader decode_header(const uint8_t *prefix, std::size_t len);
BlobHeader read_header(const std::string &path);

// Whole blobs from memory: `data` holds n_steps * rows * n_components float32 (C order).
void write_blob(const std::string &path, const BlobHeader &h, const float *data);
void write_afbl(const std::string &path, const std::string &name, const float *data, uint64_t n_steps,
                uint64_t n_points, uint64_t n_components);
void write_afel(const std::string &path, const std::string &name, const std::string &elem_type, const float *data,
                uint64_t n_steps, uint64_t n_elements, uint64_t n_ips, uint64_t n_components);

// Raw bytes to a file (UTF-8 path on every platform). Throws on failure.
void write_bytes(const std::string &path, const void *data, std::size_t n);

// A blob file opened for random step reads (one stride at a time -- the viewer's range read).
class BlobReader {
public:
    explicit BlobReader(const std::string &path);
    ~BlobReader();
    BlobReader(const BlobReader &) = delete;
    BlobReader &operator=(const BlobReader &) = delete;

    const BlobHeader &header() const {
        return header_;
    }
    // Reads step `step` into `out` (floats_per_step() floats). Throws on a short read / bad index.
    void read_step(uint64_t step, float *out);

private:
    std::string path_;
    std::FILE *fh_ = nullptr;
    BlobHeader header_;
};

// A blob written step by step (header first, then strides).
class BlobWriter {
public:
    BlobWriter(const std::string &path, const BlobHeader &h);
    ~BlobWriter();
    BlobWriter(const BlobWriter &) = delete;
    BlobWriter &operator=(const BlobWriter &) = delete;

    void write_step(const float *stride);
    // Flushes and closes; throws if fewer steps were written than the header declares.
    void finish();

private:
    std::string path_;
    std::FILE *fh_ = nullptr;
    BlobHeader header_;
    uint64_t written_ = 0;
};

} // namespace adacpp::fea
