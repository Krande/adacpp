#include "field_ops.h"

#include <chrono>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>

#include "envelope.h"
#include "superpose.h"

namespace adacpp::fea {

namespace {

using Clock = std::chrono::steady_clock;

double ms_since(Clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

void check_same_shape(const BlobHeader &a, const BlobHeader &b, const std::string &path) {
    const bool same = a.kind == b.kind && a.n_components == b.n_components && a.rows() == b.rows() &&
                      a.n_points == b.n_points && a.n_elements == b.n_elements && a.n_ips == b.n_ips &&
                      a.elem_type == b.elem_type;
    if (!same)
        throw std::invalid_argument(path + ": blob shape / kind / element type differs from the first term's");
}

// Opens each distinct path once (terms usually index several steps of ONE base blob).
class ReaderPool {
public:
    BlobReader &get(const std::string &path) {
        auto it = readers_.find(path);
        if (it == readers_.end())
            it = readers_.emplace(path, std::make_unique<BlobReader>(path)).first;
        return *it->second;
    }

private:
    std::map<std::string, std::unique_ptr<BlobReader>> readers_;
};

} // namespace

FieldOpResult combine_field_files(const std::vector<StrideRef> &terms, const std::vector<float> &factors,
                                  const std::string &out_path, const CombineOptions &opts) {
    if (terms.empty())
        throw std::invalid_argument("combine: need at least one term");
    if (terms.size() != factors.size())
        throw std::invalid_argument("combine: one factor per term");

    FieldOpResult res;
    ReaderPool pool;
    auto t0 = Clock::now();
    BlobReader &first = pool.get(terms[0].path);
    const BlobHeader in_h = first.header();
    for (const StrideRef &t : terms)
        check_same_shape(in_h, pool.get(t.path).header(), t.path);
    res.ms_read += ms_since(t0);

    const std::size_t n = static_cast<std::size_t>(in_h.floats_per_step());
    const std::size_t rows = static_cast<std::size_t>(in_h.rows());
    const std::size_t in_nc = static_cast<std::size_t>(in_h.n_components);
    std::vector<float> acc(n);
    std::vector<float> tmp(terms.size() > 1 ? n : 0);

    // out = c0 * x0, read straight into the accumulator and scaled in place.
    t0 = Clock::now();
    first.read_step(terms[0].step, acc.data());
    res.ms_read += ms_since(t0);
    t0 = Clock::now();
    scale_into(acc.data(), acc.data(), factors[0], n);
    res.ms_combine += ms_since(t0);
    for (std::size_t t = 1; t < terms.size(); ++t) {
        t0 = Clock::now();
        pool.get(terms[t].path).read_step(terms[t].step, tmp.data());
        res.ms_read += ms_since(t0);
        t0 = Clock::now();
        accumulate(acc.data(), tmp.data(), factors[t], n);
        res.ms_combine += ms_since(t0);
    }

    // Derivation: in place, or into a new layout.
    t0 = Clock::now();
    const std::size_t out_nc = opts.n_components > 0 ? static_cast<std::size_t>(opts.n_components) : in_nc;
    std::vector<float> relaid;
    float *out = acc.data();
    if (opts.n_components > 0) {
        relaid.assign(rows * out_nc, std::numeric_limits<float>::quiet_NaN());
        out = relaid.data();
    }
    for (const DeriveSpec &d : opts.derive)
        apply_derive(d, acc.data(), in_nc, out, out_nc, rows);
    res.ms_derive += ms_since(t0);

    t0 = Clock::now();
    BlobHeader out_h = in_h.with_steps(1).with_components(out_nc);
    if (!opts.name.empty())
        out_h.name = opts.name;
    res.stats.push_back(compute_step_stats(out, rows, out_nc));
    BlobWriter w(out_path, out_h);
    w.write_step(out);
    w.finish();
    res.ms_write += ms_since(t0);
    res.header = out_h;
    return res;
}

FieldOpResult envelope_field_files(const std::vector<StrideRef> &cases, const std::string &out_path,
                                   const std::string &gov_path) {
    if (cases.empty())
        throw std::invalid_argument("envelope: need at least one case");
    FieldOpResult res;
    ReaderPool pool;
    auto t0 = Clock::now();
    const BlobHeader in_h = pool.get(cases[0].path).header();
    for (const StrideRef &c : cases)
        check_same_shape(in_h, pool.get(c.path).header(), c.path);
    res.ms_read += ms_since(t0);

    const std::size_t n = static_cast<std::size_t>(in_h.floats_per_step());
    const std::size_t rows = static_cast<std::size_t>(in_h.rows());
    const std::size_t nc = static_cast<std::size_t>(in_h.n_components);
    EnvelopeAccumulator env(n);
    std::vector<float> buf(n);
    for (const StrideRef &c : cases) {
        t0 = Clock::now();
        pool.get(c.path).read_step(c.step, buf.data());
        res.ms_read += ms_since(t0);
        t0 = Clock::now();
        env.add(buf.data());
        res.ms_combine += ms_since(t0);
    }

    t0 = Clock::now();
    const BlobHeader out_h = in_h.with_steps(2);
    res.stats.push_back(compute_step_stats(env.max().data(), rows, nc));
    res.stats.push_back(compute_step_stats(env.min().data(), rows, nc));
    {
        BlobWriter w(out_path, out_h);
        w.write_step(env.max().data());
        w.write_step(env.min().data());
        w.finish();
    }
    if (!gov_path.empty()) {
        std::vector<uint8_t> bytes(AFGV_HEADER_BYTES + 4 * n, 0);
        const uint32_t head[3] = {AFGV_VERSION, static_cast<uint32_t>(cases.size()), 0};
        std::memcpy(bytes.data(), AFGV_MAGIC, 4);
        for (int w = 0; w < 3; ++w)
            for (int b = 0; b < 4; ++b)
                bytes[4 + 4 * w + b] = static_cast<uint8_t>((head[w] >> (8 * b)) & 0xFF);
        uint8_t *g = bytes.data() + AFGV_HEADER_BYTES;
        for (std::size_t i = 0; i < n; ++i) {
            g[2 * i] = static_cast<uint8_t>(env.gov_max()[i] & 0xFF);
            g[2 * i + 1] = static_cast<uint8_t>(env.gov_max()[i] >> 8);
            g[2 * (n + i)] = static_cast<uint8_t>(env.gov_min()[i] & 0xFF);
            g[2 * (n + i) + 1] = static_cast<uint8_t>(env.gov_min()[i] >> 8);
        }
        write_bytes(gov_path, bytes.data(), bytes.size());
    }
    res.ms_write += ms_since(t0);
    res.header = out_h;
    return res;
}

std::string field_op_result_json(const FieldOpResult &r) {
    const BlobHeader &h = r.header;
    std::string j = "{\"ok\":true,\"name\":" + python_json_string(h.name);
    j += ",\"kind\":\"";
    j += h.kind == BlobKind::AFEL ? "AFEL" : "AFBL";
    j += "\"";
    if (h.kind == BlobKind::AFEL)
        j += ",\"elem_type\":" + python_json_string(h.elem_type);
    j += ",\"n_steps\":" + std::to_string(h.n_steps) + ",\"rows\":" + std::to_string(h.rows()) +
         ",\"n_components\":" + std::to_string(h.n_components) + ",\"stride_bytes\":" + std::to_string(h.stride_bytes);
    j += ",\"steps\":[";
    for (std::size_t i = 0; i < r.stats.size(); ++i) {
        if (i)
            j += ',';
        j += step_stats_json(r.stats[i]);
    }
    j += "],\"timing_ms\":{\"read\":" + format_double(r.ms_read) + ",\"combine\":" + format_double(r.ms_combine) +
         ",\"derive\":" + format_double(r.ms_derive) + ",\"write\":" + format_double(r.ms_write) + "}}";
    return j;
}

} // namespace adacpp::fea
