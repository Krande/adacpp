// Native run of the sharded STEP/IFC -> GLB protocol (step_glb_shard.h / ifc_glb_shard.h): the same
// verbs the browser workers call through embind, here N shard objects in one process sharing a real
// directory. Pins that the sharded conversion writes what the single-worker path writes (triangles for
// STEP, products for IFC), that a missing or corrupt index opens as root_count() == -1, and gives the
// sanitizer builds (ASan/LSan) the whole protocol to check -- including the huge-root face split, which
// the generated model (tests/fixtures/gen_faceted_fixtures.py) takes.
//
// usage: test_glb_shards <model.stp|model.ifc> <work_dir> [N]

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "../../src/cad/ifc_glb_shard.h"
#include "../../src/cad/step_glb_shard.h"

namespace fs = std::filesystem;

namespace {

int failures = 0;
void check(const char *name, bool ok, long a = 0, long b = 0) {
    std::printf("  %s %s (%ld, %ld)\n", ok ? "ok  " : "FAIL", name, a, b);
    if (!ok)
        ++failures;
}

template <class Shard, class Single> int run(const std::string &model, const fs::path &dir, int n, Single single) {
    const fs::path lanes = dir / "lanes", parts = dir / "parts", spill = dir / "spill";
    for (const fs::path &d : {lanes, parts, spill})
        fs::create_directories(d);
    const std::string idx = (dir / "model.idx").string();

    const long want = single((dir / "single.glb").string(), spill.string());
    check("single-worker conversion", want > 0, want);

    const long nroots = Shard::prepare(model, idx, n);
    check("prepare", nroots >= 0, nroots);
    std::vector<std::unique_ptr<Shard>> sh;
    for (int k = 0; k < n; ++k)
        sh.push_back(std::make_unique<Shard>(model, idx, 2.0, 20.0));
    check("every shard opens the index", sh.back()->root_count() == nroots, sh.back()->root_count(), nroots);
    check("the big root takes the face split", sh[0]->huge_count() > 0, sh[0]->huge_count());

    long got = 0;
    for (long h = 0; h < sh[0]->huge_count(); ++h) {
        const long nf = sh[0]->huge_faces(h), chunk = std::max(64L, (nf + 4L * n - 1) / (4L * n));
        long k = 0;
        for (long f0 = 0; f0 < nf; f0 += chunk, ++k)
            check("process_huge",
                  sh[(size_t) (k % n)]->process_huge(h, f0, std::min(nf, f0 + chunk), parts.string()) >= 0);
        const long r = sh[0]->assemble_huge(h, chunk, parts.string(), lanes.string(), 0);
        check("assemble_huge", r >= 0, r);
        got += r;
    }
    const std::vector<long> ends = sh[0]->plan_batches(32L * n);
    for (size_t b = 0, begin = 0; b < ends.size(); begin = (size_t) ends[b], ++b)
        got += sh[b % (size_t) n]->process((long) begin, ends[b], lanes.string(), (int) (b % (size_t) n));
    for (auto &s : sh)
        s->persist();
    sh.clear();
    const fs::path out = dir / "sharded.glb";
    const long merged = Shard::merge_lanes(lanes.string(), n, out.string());
    check("merge", merged > 0 && fs::file_size(out) > 100, merged);
    check("sharded == single-worker", got == want, got, want);

    // corrupt indexes open as -1 (never trap, never allocate from a garbage length)
    Shard missing(model, (dir / "missing.idx").string(), 2.0, 20.0);
    check("missing index", missing.root_count() == -1);
    const std::string bad = (dir / "bad.idx").string();
    {
        std::ifstream in(idx, std::ios::binary);
        std::string bytes((std::istreambuf_iterator<char>(in)), {});
        std::ofstream(bad, std::ios::binary).write(bytes.data(), (std::streamsize) (bytes.size() / 2));
    }
    Shard truncated(model, bad, 2.0, 20.0);
    check("truncated index", truncated.root_count() == -1);
    {
        std::ifstream in(idx, std::ios::binary);
        std::string head(8, '\0');
        in.read(head.data(), 8);
        std::string junk(4096, '\xff');
        junk.replace(0, 8, head); // the right magic, then huge lengths
        std::ofstream(bad, std::ios::binary).write(junk.data(), (std::streamsize) junk.size());
    }
    Shard garbage(model, bad, 2.0, 20.0);
    check("garbage index", garbage.root_count() == -1);
    return failures;
}

struct StepShard : adacpp::StepGlbShard {
    using adacpp::StepGlbShard::StepGlbShard;
    static long merge_lanes(const std::string &dir, int n, const std::string &out) {
        return adacpp::merge_step_glb_lanes(dir, n, out, false);
    }
};
struct IfcShard : adacpp::IfcGlbShard {
    using adacpp::IfcGlbShard::IfcGlbShard;
    static long merge_lanes(const std::string &dir, int n, const std::string &out) {
        return adacpp::merge_ifc_glb_lanes(dir, n, out, false);
    }
};

} // namespace

int main(int argc, char **argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: test_glb_shards <model.stp|model.ifc> <work_dir> [N]\n");
        return 2;
    }
    const std::string model = argv[1];
    const fs::path dir = argv[2];
    const int n = argc > 3 ? std::atoi(argv[3]) : 3;
    fs::remove_all(dir);
    fs::create_directories(dir);
    const bool ifc =
        model.size() > 4 && (model.substr(model.size() - 4) == ".ifc" || model.substr(model.size() - 4) == ".IFC");
    std::printf("GLB shards (native): %s N=%d\n", model.c_str(), n);
    const int f = ifc ? run<IfcShard>(model, dir, n,
                                      [&](const std::string &out, const std::string &spill) {
                                          return adacpp::stream_ifc_to_glb(model, out, 2.0, 20.0, false, spill, 0.0, 1);
                                      })
                      : run<StepShard>(model, dir, n, [&](const std::string &out, const std::string &spill) {
                            return adacpp::step_to_glb_single(model, out, spill, 2.0, 20.0, false);
                        });
    fs::remove_all(dir);
    return f ? 1 : 0;
}
