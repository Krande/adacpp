// Unit test for the native GLB writer (ngeom_glb.h): merge-by-colour + GLB framing.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "ngeom_glb.h"

using namespace adacpp::glb;

static int g_fail = 0;
#define CHECK(cond, msg)                                                                                               \
    do {                                                                                                               \
        if (!(cond)) {                                                                                                 \
            std::printf("FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__);                                               \
            ++g_fail;                                                                                                  \
        }                                                                                                              \
    } while (0)

static uint32_t u32(const std::vector<uint8_t> &b, size_t o) {
    uint32_t v;
    std::memcpy(&v, b.data() + o, 4);
    return v;
}
static size_t count(const std::string &h, const std::string &n) {
    size_t c = 0, p = 0;
    while ((p = h.find(n, p)) != std::string::npos) {
        ++c;
        p += n.size();
    }
    return c;
}

int main() {
    // a red triangle (identity) and a blue quad placed by a +10x transform
    GlbSolid tri;
    tri.positions = {0, 0, 0, 1, 0, 0, 0, 1, 0};
    tri.indices = {0, 1, 2};
    tri.color = {1, 0, 0, 1};
    GlbSolid quad;
    quad.positions = {0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0};
    quad.indices = {0, 1, 2, 0, 2, 3};
    quad.color = {0, 0, 1, 1};
    quad.transforms = {{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 10, 0, 0, 1}}; // +10x

    const std::string path = "/tmp/adacpp_glb_test.glb";
    CHECK(write_glb(path, {tri, quad}), "write_glb returns true");

    std::ifstream f(path, std::ios::binary);
    std::vector<uint8_t> b((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    CHECK(b.size() > 20, "non-empty glb");
    CHECK(u32(b, 0) == 0x46546C67u, "glTF magic");
    CHECK(u32(b, 4) == 2u, "version 2");
    CHECK(u32(b, 8) == b.size(), "total length == file size");
    uint32_t json_len = u32(b, 12);
    CHECK(u32(b, 16) == 0x4E4F534Au, "JSON chunk type");
    std::string json((const char *) b.data() + 20, json_len);
    CHECK(json.find("\"meshes\"") != std::string::npos && json.find("POSITION") != std::string::npos, "glTF keys");
    CHECK(count(json, "pbrMetallicRoughness") == 2, "two colour materials");
    CHECK(json.find("\"alphaMode\"") == std::string::npos, "opaque colours -> no alphaMode");
    size_t bin_hdr = 20 + json_len;
    uint32_t bin_len = u32(b, bin_hdr);
    CHECK(u32(b, bin_hdr + 4) == 0x004E4942u, "BIN chunk type");
    CHECK(20 + json_len + 8 + bin_len == b.size(), "chunk sizes sum to file size");
    // baked transform: the quad's vertices must include a vertex at x>=10 (placed +10x)
    bool placed = false;
    for (size_t o = bin_hdr + 8; o + 4 <= b.size(); o += 4) {
        float v;
        std::memcpy(&v, b.data() + o, 4);
        if (v >= 9.9f && v <= 11.1f)
            placed = true;
    }
    CHECK(placed, "transform baked into vertices (x ~ 10)");

    std::remove(path.c_str());
    if (g_fail == 0)
        std::printf("ngeom glb: ALL PASS\n");
    else
        std::printf("ngeom glb: %d FAILURES\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
