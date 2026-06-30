// Regression test for the curved-face -> solid pipeline that the viewer uses for PlateCurved:
// decode an NGEOM face -> libtess2 tessellate (open shell) -> thicken into a closed solid ->
// assert the result is a clean 2-manifold (no edge shared by 3+ triangles, welded by position).
//
// Motivation: hullskin elev13 plates rendered with no black outline edges. The frontend computes
// edges per draw-range with EdgesGeometry semantics (boundary edges always + dihedral feature
// edges), and a NON-manifold mesh (edges shared by 3+ triangles) defeats that classification ->
// the outline vanishes. This test pins the invariant that our tessellate+solidify pipeline emits
// manifold geometry, on the actual problem faces captured as fixtures.
//
// Fixtures (tests/ngeom/fixtures/*.ngeom) are real AdvancedFace buffers serialized from the
// hullskin model by ada.cadit.ngeom.serialize.serialize_geometries — see
// dap project_hullskin_nonmanifold_edges for the capture recipe.
//
// Build: see tests/ngeom/run.sh (links ngeom_tessellate.cpp + libtess2 objects).
#include <cstdint>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

#include "ngeom_decode.h"
#include "ngeom_solidify.h"
#include "ngeom_tessellate.h"

using namespace adacpp::ngeom;

static int g_fail = 0;
#define CHECK(cond, msg)                                                                                               \
    do {                                                                                                               \
        if (!(cond)) {                                                                                                 \
            std::printf("FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__);                                               \
            ++g_fail;                                                                                                  \
        }                                                                                                              \
    } while (0)

// Edge-use multiplicity after welding vertices by quantized position (1e-6 model units) — the
// same position-weld the frontend edge builder uses. once = boundary, twice = manifold interior,
// three_plus = NON-manifold (the bug signature).
struct EdgeStats {
    int once = 0, twice = 0, three_plus = 0, tris = 0, weld_verts = 0;
};

static EdgeStats edge_stats(const TessMesh &m) {
    std::map<std::array<int64_t, 3>, int> weld;
    std::vector<int> rep(m.positions.size() / 3);
    auto q = [](float v) { return static_cast<int64_t>(std::llround(static_cast<double>(v) * 1e6)); };
    for (size_t v = 0; v < rep.size(); ++v) {
        std::array<int64_t, 3> k{q(m.positions[v * 3]), q(m.positions[v * 3 + 1]), q(m.positions[v * 3 + 2])};
        auto it = weld.find(k);
        if (it == weld.end()) {
            int id = static_cast<int>(weld.size());
            weld.emplace(k, id);
            rep[v] = id;
        } else {
            rep[v] = it->second;
        }
    }
    std::map<std::pair<int, int>, int> ecnt;
    const size_t ntri = m.indices.size() / 3;
    for (size_t t = 0; t < ntri; ++t) {
        int v[3] = {rep[m.indices[t * 3]], rep[m.indices[t * 3 + 1]], rep[m.indices[t * 3 + 2]]};
        for (int e = 0; e < 3; ++e) {
            int a = v[e], b = v[(e + 1) % 3];
            if (a == b)
                continue; // degenerate after weld
            if (a > b)
                std::swap(a, b);
            ecnt[{a, b}]++;
        }
    }
    EdgeStats s;
    s.tris = static_cast<int>(ntri);
    s.weld_verts = static_cast<int>(weld.size());
    for (auto &kv : ecnt) {
        if (kv.second == 1)
            s.once++;
        else if (kv.second == 2)
            s.twice++;
        else
            s.three_plus++;
    }
    return s;
}

static std::vector<uint8_t> read_file(const std::string &path) {
    std::vector<uint8_t> b;
    FILE *f = std::fopen(path.c_str(), "rb");
    if (!f)
        return b;
    std::fseek(f, 0, SEEK_END);
    long n = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    b.resize(n > 0 ? n : 0);
    if (n > 0 && std::fread(b.data(), 1, n, f) != static_cast<size_t>(n))
        b.clear();
    std::fclose(f);
    return b;
}

static void test_face(const char *label, const std::string &path) {
    std::vector<uint8_t> buf = read_file(path);
    CHECK(!buf.empty(), "fixture loads");
    if (buf.empty())
        return;

    NgeomDoc doc = decode(buf.data(), buf.size());
    CHECK(!doc.roots.empty(), "decoded a root");
    if (doc.roots.empty())
        return;

    // 1) Tessellate the bare curved face (open shell), exactly like the viewer stream path —
    // same deflection/angular the worker passes (CadConfig default: 2.0 m chord, ~20 deg).
    TessParams tp;
    tp.deflection = 2.0;
    tp.max_angle = 0.349; // ~20 degrees
    TessMesh shell = tessellate_doc(doc, tp);
    CHECK(!shell.indices.empty(), "face tessellates to triangles");

    EdgeStats es = edge_stats(shell);
    std::printf("  %-14s SHELL: tris=%d weldV=%d once=%d twice=%d 3+=%d\n", label, es.tris, es.weld_verts, es.once,
                es.twice, es.three_plus);
    // An open surface shell is a 2-manifold-with-boundary: every interior edge used twice, the
    // boundary loop used once, nothing used 3+.
    CHECK(es.three_plus == 0, "tessellated shell is manifold (no edge shared by 3+ triangles)");
    CHECK(es.once > 0, "open shell has a boundary loop");

    // 2) Solidify (thicken) into a closed solid — the "make it solid" step under test.
    thicken_mesh(shell, 0.025); // 25 mm, the hullskin plate thickness
    EdgeStats es2 = edge_stats(shell);
    std::printf("  %-14s SOLID: tris=%d weldV=%d once=%d twice=%d 3+=%d\n", label, es2.tris, es2.weld_verts, es2.once,
                es2.twice, es2.three_plus);
    // A closed thin solid is a closed 2-manifold: every edge shared by exactly two triangles,
    // no boundary, no non-manifold edge. This is the property the frontend edge builder needs.
    CHECK(es2.three_plus == 0, "thickened solid is manifold (no edge shared by 3+ triangles)");
    CHECK(es2.once == 0, "thickened solid is closed (no boundary edges)");
}

int main() {
    // Concrete hullskin faces: elev13plate1 is one of the 16 that rendered edge-less in the
    // viewer (irregular 6-7-coedge spline patch); elev14plate7 is a clean 4-coedge control.
    test_face("elev13plate1", "tests/ngeom/fixtures/face_elev13plate1.ngeom");
    test_face("elev14plate7", "tests/ngeom/fixtures/face_elev14plate7.ngeom");

    if (g_fail) {
        std::printf("test_solidify_manifold: %d CHECK(s) failed\n", g_fail);
        return 1;
    }
    std::printf("test_solidify_manifold: all checks passed\n");
    return 0;
}
