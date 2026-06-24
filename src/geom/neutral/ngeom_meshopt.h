#pragma once
#include <cstdint>
#include <vector>

namespace ngeom {

struct SimplifiedMesh {
    std::vector<float> positions;  // xyz interleaved
    std::vector<uint32_t> indices;
};

// Faithful port of step2glb mesh.rs simplify_meshopt: meshopt_simplify toward
// threshold*index_count within target_error, border LOCKED (shared seams keep shape), then drop
// degenerate triangles and compact to the surviving vertices. With target_error 0.0 this is a
// lossless coplanar-triangle collapse (the cleanup step2glb's merged GLB applies, ~16% on the crane).
SimplifiedMesh meshopt_simplify_mesh(const std::vector<float> &positions,
                                     const std::vector<uint32_t> &indices, float threshold,
                                     float target_error);

// EXT_meshopt_compression codecs (order-preserving: the INDICES/vertex byte codecs that keep
// draw-range offsets valid). These wrap the vendored meshoptimizer C encoder/decoder so the
// adapy GLB packer doesn't need the sdist-only PyPI `meshoptimizer` (no wheels -> compiler).
// vertex buffer: `count` vertices of `stride` bytes each. index sequence: `count` uint32 indices.
std::vector<unsigned char> meshopt_encode_vertices(const void *data, size_t count, size_t stride);
std::vector<unsigned char> meshopt_encode_indices(const uint32_t *indices, size_t count,
                                                  size_t vertex_count);
// decode back (used for the packer's round-trip self-check). Empty vector => failure.
std::vector<unsigned char> meshopt_decode_vertices(const unsigned char *enc, size_t enc_size,
                                                   size_t count, size_t stride);
// index_size = 2 (uint16) or 4 (uint32) — the output element size, to match the GLB's stride.
std::vector<unsigned char> meshopt_decode_indices(const unsigned char *enc, size_t enc_size,
                                                  size_t count, size_t index_size);

}  // namespace ngeom
