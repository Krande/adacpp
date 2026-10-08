// Native unit tests for the format-neutral FEA kernels (src/fea). No Python, no OCC: build with
//   tests/fea/run.sh   (g++/clang++)   or   tests/fea/run.ps1   (MSVC, inside `pixi shell -e test`)
//
// The numpy references live in tests/py/test_fea.py; this file pins the C++ side on its own: the
// superposition order, the derivation formulas written out longhand, the AFBL/AFEL header bytes
// against vectors produced by Python's json.dumps, and the file pipeline round trip.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "../../src/fea/artefact_io.h"
#include "../../src/fea/derive.h"
#include "../../src/fea/envelope.h"
#include "../../src/fea/fea_arrays.h"
#include "../../src/fea/fea_model.h"
#include "../../src/fea/field_ops.h"
#include "../../src/fea/superpose.h"

using namespace adacpp::fea;

static int g_fail = 0;
#define CHECK(cond)                                                                                                    \
    do {                                                                                                               \
        if (!(cond)) {                                                                                                 \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);                                                \
            ++g_fail;                                                                                                  \
        }                                                                                                              \
    } while (0)

static bool same_bits(float a, float b) {
    uint32_t x, y;
    std::memcpy(&x, &a, 4);
    std::memcpy(&y, &b, 4);
    return x == y;
}

static std::vector<float> random_floats(std::size_t n, uint32_t seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<float> nd(0.0f, 1.0f);
    std::uniform_real_distribution<float> scale(-3.0f, 6.0f);
    std::vector<float> v(n);
    for (auto &x : v)
        x = nd(rng) * std::pow(10.0f, scale(rng));
    return v;
}

static void test_superpose() {
    const std::size_t n = 100003; // not a multiple of the block or the vector width
    std::vector<std::vector<float>> xs;
    for (uint32_t t = 0; t < 6; ++t)
        xs.push_back(random_floats(n, 11 + t));
    const std::vector<float> c = {1.2f, -1.1f, 0.35f, 2.0f, -0.7f, 1.0e-3f};
    std::vector<const float *> in;
    for (auto &x : xs)
        in.push_back(x.data());
    std::vector<float> out(n);
    combine_strides(in, c, out.data(), n);

    bool all = true;
    for (std::size_t i = 0; i < n; ++i) {
        volatile float acc = c[0] * xs[0][i];
        for (std::size_t t = 1; t < xs.size(); ++t) {
            volatile float p = c[t] * xs[t][i];
            acc = acc + p;
        }
        all = all && same_bits(out[i], acc);
    }
    CHECK(all);

    // The streaming primitives give the same bits as the blocked whole.
    std::vector<float> s(n);
    scale_into(s.data(), xs[0].data(), c[0], n);
    for (std::size_t t = 1; t < xs.size(); ++t)
        accumulate(s.data(), xs[t].data(), c[t], n);
    CHECK(std::memcmp(s.data(), out.data(), n * 4) == 0);

    // A case that WOULD differ under FMA contraction: 1 + eps^2 style cancellation.
    const float a = 1.0f + std::ldexp(1.0f, -12);
    std::vector<float> x1 = {a}, x2 = {a};
    std::vector<const float *> in2 = {x1.data(), x2.data()};
    std::vector<float> c2 = {a, -1.0f};
    float r = 0;
    // a*a rounds to float before the add: (1 + 2^-11 + 2^-24) -> 1 + 2^-11, then - a = 2^-12
    combine_strides(in2, c2, &r, 1);
    CHECK(same_bits(r, std::ldexp(1.0f, -12)));
}

static void test_derive() {
    const std::size_t rows = 5000;
    auto v = random_floats(rows * 4, 99);
    // columns 0..2 = SIGXX, SIGYY, TAUXY; 3 = VONMISES (overwritten in place)
    DeriveSpec vm{DeriveOp::PlaneVonMises, {0, 1, 2}, {3}};
    apply_derive(vm, v.data(), 4, v.data(), 4, rows);
    bool ok = true;
    for (std::size_t r = 0; r < rows; ++r) {
        const double sx = v[r * 4], sy = v[r * 4 + 1], t = v[r * 4 + 2];
        const float want = static_cast<float>(std::sqrt(sx * sx + sy * sy - sx * sy + 3.0 * t * t));
        ok = ok && same_bits(v[r * 4 + 3], want);
    }
    CHECK(ok);

    // plane_principal into a new 2-column layout
    std::vector<float> p(rows * 2);
    apply_derive({DeriveOp::PlanePrincipal, {0, 1, 2}, {0, 1}}, v.data(), 4, p.data(), 2, rows);
    ok = true;
    for (std::size_t r = 0; r < rows; ++r) {
        const double sx = v[r * 4], sy = v[r * 4 + 1], t = v[r * 4 + 2];
        const double c = 0.5 * (sx + sy);
        const double h = 0.5 * (sx - sy);
        const double rad = std::sqrt(h * h + t * t);
        ok = ok && same_bits(p[r * 2], static_cast<float>(c + rad)) &&
             same_bits(p[r * 2 + 1], static_cast<float>(c - rad));
        ok = ok && p[r * 2] >= p[r * 2 + 1];
    }
    CHECK(ok);

    // magnitude, and a dropped output (-1)
    std::vector<float> m(rows);
    apply_derive({DeriveOp::Magnitude, {0, 1, 2}, {0}}, v.data(), 4, m.data(), 1, rows);
    ok = true;
    for (std::size_t r = 0; r < rows; ++r) {
        const double a = v[r * 4], b = v[r * 4 + 1], c = v[r * 4 + 2];
        ok = ok && same_bits(m[r], static_cast<float>(std::sqrt((a * a + b * b) + c * c)));
    }
    CHECK(ok);
    std::vector<float> p1(rows, -9.0f);
    apply_derive({DeriveOp::PlanePrincipal, {0, 1, 2}, {0, -1}}, v.data(), 4, p1.data(), 1, rows);
    CHECK(same_bits(p1[7], p[7 * 2]));

    // shell_decompose: bottom (0..2), top (3..5)
    auto s = random_floats(rows * 6, 7);
    std::vector<float> d(rows * 7);
    apply_derive({DeriveOp::ShellDecompose, {0, 1, 2, 3, 4, 5}, {0, 1, 2, 3, 4, 5, 6}}, s.data(), 6, d.data(), 7, rows);
    ok = true;
    for (std::size_t r = 0; r < rows; ++r) {
        const float *b = &s[r * 6];
        const float *t = b + 3;
        const double m0 = 0.5 * ((double) t[0] + (double) b[0]);
        const double m1 = 0.5 * ((double) t[1] + (double) b[1]);
        const double m2 = 0.5 * ((double) t[2] + (double) b[2]);
        const double mv = std::sqrt(m0 * m0 + m1 * m1 - m0 * m1 + 3.0 * m2 * m2);
        ok = ok && same_bits(d[r * 7 + 0], (float) m0) && same_bits(d[r * 7 + 1], (float) m1) &&
             same_bits(d[r * 7 + 4], (float) m2) &&
             same_bits(d[r * 7 + 2], (float) (0.5 * ((double) t[0] - (double) b[0]))) &&
             same_bits(d[r * 7 + 6], (float) mv);
    }
    CHECK(ok);

    // shell_resultants: one thickness per 2 rows
    std::vector<double> th(rows / 2 + 1);
    for (std::size_t i = 0; i < th.size(); ++i)
        th[i] = 0.01 + 0.001 * double(i % 17);
    std::vector<float> rs(rows * 6);
    apply_derive({DeriveOp::ShellResultants, {0, 1, 2, 3, 4, 5}, {0, 1, 2, 3, 4, 5}}, d.data(), 7, rs.data(), 6, rows,
                 th.data(), 2);
    {
        const std::size_t r = 11;
        const double t = th[r / 2];
        const double t2 = t * t / 6.0;
        CHECK(same_bits(rs[r * 6 + 0], (float) ((double) d[r * 7 + 0] * t)));
        CHECK(same_bits(rs[r * 6 + 1], (float) ((double) d[r * 7 + 4] * t)));
        CHECK(same_bits(rs[r * 6 + 3], (float) ((double) d[r * 7 + 5] * t2)));
        CHECK(same_bits(rs[r * 6 + 5], (float) ((double) d[r * 7 + 3] * t2)));
    }

    bool threw = false;
    try {
        apply_derive({DeriveOp::PlaneVonMises, {0, 1}, {3}}, v.data(), 4, v.data(), 4, rows);
    } catch (const std::invalid_argument &) {
        threw = true;
    }
    CHECK(threw);
    CHECK(derive_op_from_name("general_stress") == DeriveOp::PlaneVonMises);

    // The lazy-case manifest's single-output names (adapy artefacts/combine.py).
    std::vector<float> q1(rows), q2(rows), mg(rows);
    apply_derive({derive_op_from_name("plane_principal_1"), {0, 1, 2}, {0}}, v.data(), 4, q1.data(), 1, rows);
    apply_derive({derive_op_from_name("plane_principal_2"), {0, 1, 2}, {0}}, v.data(), 4, q2.data(), 1, rows);
    apply_derive({derive_op_from_name("magnitude3"), {0, 1, 2}, {0}}, v.data(), 4, mg.data(), 1, rows);
    ok = true;
    for (std::size_t r = 0; r < rows; ++r)
        ok = ok && same_bits(q1[r], p[r * 2]) && same_bits(q2[r], p[r * 2 + 1]) && same_bits(mg[r], m[r]);
    CHECK(ok);
}

static void test_header() {
    // Vectors from: json.dumps({...}, separators=(",", ":")) -- Python's default ensure_ascii.
    BlobHeader h = make_afbl_header("DISPLACEMENT", 13, 1234, 7);
    CHECK(header_json(h) ==
          "{\"name\":\"DISPLACEMENT\",\"n_steps\":13,\"n_points\":1234,\"n_components\":7,\"dtype\":\"float32\","
          "\"stride_bytes\":34552}");
    BlobHeader e = make_afel_header("G-STRESS", "QUAD4", 1, 10, 8, 4);
    CHECK(header_json(e) == "{\"name\":\"G-STRESS\",\"elem_type\":\"QUAD4\",\"n_steps\":1,\"n_elements\":10,"
                            "\"n_ips\":8,\"n_components\":4,\"dtype\":\"float32\",\"stride_bytes\":1280}");
    // json.dumps("a\"b\\c\nå\U0001F600\x7f\x01/")
    CHECK(python_json_string("a\"b\\c\n\xC3\xA5\xF0\x9F\x98\x80\x7F\x01/") ==
          "\"a\\\"b\\\\c\\n\\u00e5\\ud83d\\ude00\\u007f\\u0001/\"");

    auto bytes = encode_header(e);
    CHECK(bytes.size() == 1024);
    CHECK(std::memcmp(bytes.data(), "AFEL", 4) == 0);
    CHECK(bytes[4] == 1 && bytes[5] == 0 && bytes[8] == header_json(e).size());
    CHECK(bytes[1023] == 0);
    BlobHeader back = decode_header(bytes.data(), bytes.size());
    CHECK(back.kind == BlobKind::AFEL && back.name == "G-STRESS" && back.elem_type == "QUAD4" &&
          back.n_elements == 10 && back.n_ips == 8 && back.n_components == 4 && back.stride_bytes == 1280);

    BlobHeader weird = make_afbl_header("\xC3\xA5\"x", 1, 1, 1);
    auto wb = encode_header(weird);
    CHECK(decode_header(wb.data(), wb.size()).name == weird.name);

    bool threw = false;
    try {
        encode_header(make_afbl_header(std::string(1100, 'x'), 1, 1, 1));
    } catch (const std::length_error &) {
        threw = true;
    }
    CHECK(threw);
}

static void test_envelope() {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    std::vector<float> a = {1, 5, nan, 2, nan};
    std::vector<float> b = {3, 5, 4, 2, nan};
    std::vector<float> c = {-1, 6, 1, 0, nan};
    std::vector<const float *> in = {a.data(), b.data(), c.data()};
    std::vector<float> mx(5), mn(5);
    std::vector<uint16_t> gx(5), gn(5);
    envelope_strides(in, 5, mx.data(), mn.data(), gx.data(), gn.data());
    CHECK(mx[0] == 3 && gx[0] == 1 && mn[0] == -1 && gn[0] == 2);
    CHECK(mx[1] == 6 && gx[1] == 2 && mn[1] == 5 && gn[1] == 0); // tie on 5 keeps case 0
    CHECK(mx[2] == 4 && gx[2] == 1 && mn[2] == 1 && gn[2] == 2); // leading NaN skipped
    CHECK(mx[3] == 2 && gx[3] == 0 && mn[3] == 0 && gn[3] == 2);
    CHECK(std::isnan(mx[4]) && gx[4] == 0 && std::isnan(mn[4]) && gn[4] == 0);
}

static void test_stats() {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    std::vector<float> v = {3, 4, 0, 1, nan, 2, -2, -1, 0.5f, -7, 1, 1};
    StepStats s = compute_step_stats(v.data(), 3, 4);
    // rows: (3,4,0,1) (nan,2,-2,-1) (0.5,-7,1,1)
    CHECK(s.comp_min[0] == 0.5f && s.comp_max[0] == 3);
    CHECK(s.comp_min[1] == -7 && s.comp_max[1] == 4);
    CHECK(s.comp_min[2] == -2 && s.comp_max[2] == 1);
    CHECK(s.comp_min[3] == -1 && s.comp_max[3] == 1);
    // magnitude: row 0 = 5, row 1 has a NaN -> skipped, row 2 = sqrt(0.25 + 49 + 1)
    CHECK(s.mag_min == 5.0f && s.mag_max == std::sqrt((0.25f + 49.0f) + 1.0f));
    std::vector<float> all_nan = {nan, nan};
    StepStats z = compute_step_stats(all_nan.data(), 2, 1);
    CHECK(z.comp_min[0] == 0 && z.comp_max[0] == 0 && !z.has_magnitude);
    CHECK(step_stats_json(z) == "{\"rows\":2,\"n_components\":1,\"scalar_range_per_component\":[[0,0]],"
                                "\"scalar_range_magnitude\":[0,0]}");
}

static void test_files() {
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / "adacpp_fea_test";
    fs::create_directories(dir);
    const std::size_t ne = 300, nip = 4, nc = 4, ns = 3;
    auto data = random_floats(ns * ne * nip * nc, 5);
    const std::string base = (dir / "fea.G-STRESS.QUAD4.elements.bin").string();
    write_afel(base, "G-STRESS", "QUAD4", data.data(), ns, ne, nip, nc);
    CHECK(fs::file_size(base) == 1024 + ns * ne * nip * nc * 4);

    // combine steps 2, 0 with factors, re-derive VONMISES in place
    CombineOptions opts;
    opts.derive.push_back({DeriveOp::PlaneVonMises, {0, 1, 2}, {3}});
    const std::string out = (dir / "case.bin").string();
    FieldOpResult r = combine_field_files({{base, 2}, {base, 0}}, {1.5f, -0.25f}, out, opts);
    CHECK(r.header.n_steps == 1 && r.header.n_elements == ne && r.header.elem_type == "QUAD4");

    const std::size_t per = ne * nip * nc;
    std::vector<float> want(per);
    std::vector<const float *> in = {data.data() + 2 * per, data.data()};
    std::vector<float> c = {1.5f, -0.25f};
    combine_strides(in, c, want.data(), per);
    apply_derive(opts.derive[0], want.data(), nc, want.data(), nc, ne * nip);
    {
        BlobReader rd(out);
        std::vector<float> got(per);
        rd.read_step(0, got.data());
        CHECK(std::memcmp(got.data(), want.data(), per * 4) == 0);
        CHECK(rd.header().name == "G-STRESS");
    }

    // P-STRESS: a new 2-column layout derived from the combined G-STRESS
    CombineOptions popts;
    popts.n_components = 2;
    popts.name = "P-STRESS";
    popts.derive.push_back({DeriveOp::PlanePrincipal, {0, 1, 2}, {0, 1}});
    FieldOpResult pr = combine_field_files({{base, 2}, {base, 0}}, {1.5f, -0.25f}, (dir / "p.bin").string(), popts);
    CHECK(pr.header.n_components == 2 && pr.header.name == "P-STRESS" && pr.header.stride_bytes == ne * nip * 2 * 4);

    // envelope over the three stored steps
    FieldOpResult er =
        envelope_field_files({{base, 0}, {base, 1}, {base, 2}}, (dir / "env.bin").string(), (dir / "env.gov").string());
    CHECK(er.header.n_steps == 2 && er.stats.size() == 2);
    CHECK(fs::file_size(dir / "env.gov") == AFGV_HEADER_BYTES + 2 * per * 2);
    {
        BlobReader er_rd((dir / "env.bin").string());
        std::vector<float> mx(per);
        er_rd.read_step(0, mx.data());
        bool ok = true;
        for (std::size_t i = 0; i < per; ++i)
            ok = ok && mx[i] == std::max(std::max(data[i], data[per + i]), data[2 * per + i]);
        CHECK(ok);
    }

    bool threw = false;
    try {
        combine_field_files({{base, 3}}, {1.0f}, (dir / "bad.bin").string());
    } catch (const std::out_of_range &) {
        threw = true;
    }
    CHECK(threw);
    fs::remove_all(dir);
}

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    try {
        test_superpose();
        test_derive();
        test_header();
        test_envelope();
        test_stats();
        test_files();
    } catch (const std::exception &e) {
        std::printf("FAIL: uncaught exception: %s\n", e.what());
        return 1;
    }
    if (g_fail) {
        std::printf("fea: %d check(s) FAILED\n", g_fail);
        return 1;
    }
    std::printf("fea: all checks passed\n");
    return 0;
}
