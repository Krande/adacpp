#include "ngeom_meshopt.h"

#include <unordered_map>

#include "meshoptimizer.h"

namespace ngeom {

SimplifiedMesh meshopt_simplify_mesh(const std::vector<float> &positions, const std::vector<uint32_t> &indices,
                                     float threshold, float target_error) {
    SimplifiedMesh out;
    const size_t index_count = indices.size();
    const size_t vertex_count = positions.size() / 3;
    if (index_count < 3 || vertex_count < 3) {
        out.positions = positions;
        out.indices = indices;
        return out;
    }
    const size_t target = static_cast<size_t>(static_cast<float>(index_count) * threshold);
    std::vector<uint32_t> dest(index_count);
    float result_error = 0.0f;
    const size_t new_count =
        meshopt_simplify(dest.data(), indices.data(), index_count, positions.data(), vertex_count, sizeof(float) * 3,
                         target, target_error, meshopt_SimplifyLockBorder, &result_error);

    // compact to surviving vertices, dropping degenerate triangles (step2glb simplify_meshopt)
    std::unordered_map<uint32_t, uint32_t> remap;
    remap.reserve(new_count);
    out.indices.reserve(new_count);
    for (size_t i = 0; i + 2 < new_count; i += 3) {
        const uint32_t a = dest[i], b = dest[i + 1], c = dest[i + 2];
        if (a == b || b == c || a == c)
            continue; // degenerate
        for (uint32_t idx : {a, b, c}) {
            auto it = remap.find(idx);
            uint32_t ni;
            if (it == remap.end()) {
                ni = static_cast<uint32_t>(out.positions.size() / 3);
                remap.emplace(idx, ni);
                out.positions.insert(out.positions.end(), positions.begin() + idx * 3, positions.begin() + idx * 3 + 3);
            } else {
                ni = it->second;
            }
            out.indices.push_back(ni);
        }
    }
    return out;
}

std::vector<unsigned char> meshopt_encode_vertices(const void *data, size_t count, size_t stride) {
    std::vector<unsigned char> buf(meshopt_encodeVertexBufferBound(count, stride));
    size_t n = meshopt_encodeVertexBuffer(buf.data(), buf.size(), data, count, stride);
    buf.resize(n);
    return buf;
}

std::vector<unsigned char> meshopt_encode_indices(const uint32_t *indices, size_t count, size_t vertex_count) {
    std::vector<unsigned char> buf(meshopt_encodeIndexSequenceBound(count, vertex_count));
    size_t n =
        meshopt_encodeIndexSequence(buf.data(), buf.size(), reinterpret_cast<const unsigned int *>(indices), count);
    buf.resize(n);
    return buf;
}

std::vector<unsigned char> meshopt_decode_vertices(const unsigned char *enc, size_t enc_size, size_t count,
                                                   size_t stride) {
    std::vector<unsigned char> dest(count * stride);
    if (meshopt_decodeVertexBuffer(dest.data(), count, stride, enc, enc_size) != 0)
        dest.clear();
    return dest;
}

std::vector<unsigned char> meshopt_decode_indices(const unsigned char *enc, size_t enc_size, size_t count,
                                                  size_t index_size) {
    std::vector<unsigned char> out(count * index_size);
    if (meshopt_decodeIndexSequence(out.data(), count, index_size, enc, enc_size) != 0)
        out.clear();
    return out;
}

} // namespace ngeom
