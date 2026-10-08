// Format-neutral FEA result model + the ResultReader interface a solver-format front-end implements.
//
// HEADER ONLY, and deliberately unused by the kernels in this directory for now: the kernels work on
// baked strides (AFBL/AFEL) and need nothing more. These types fix the vocabulary a native result
// reader will produce -- a mesh as flat arrays, element blocks, cases (stored or combined), recipes
// and field tables -- so a later reader can feed the same superpose / derive / envelope / write path
// without any solver-specific type leaking past it.
#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "derive.h"

namespace adacpp::fea {

// One block of same-type elements: `labels[e]`, and `connectivity[e * nodes_per_element + k]` as
// indices into MeshArrays::node_labels / coords (not labels).
struct ElemBlock {
    std::string elem_type; // the blob's elem_type string (AFEL), e.g. a source type name
    uint32_t nodes_per_element = 0;
    std::vector<uint32_t> labels;
    std::vector<uint32_t> connectivity;
};

struct MeshArrays {
    std::vector<uint32_t> node_labels;
    std::vector<double> coords; // 3 per node
    std::vector<ElemBlock> blocks;
};

// One term of a load combination: `factor * basic` (`phase` in degrees for complex basics).
// `factor` / `phase` are float32 exactly as the superposition consumes them.
struct Term {
    uint32_t basic = 0; // stored-case number the term refers to
    float factor = 0.0f;
    float phase = 0.0f;
};

// A combination's recipe, in file term order (the order superposition commits to).
struct Recipe {
    uint32_t case_n = 0;
    bool complex = false;
    std::vector<Term> terms;
    // True when Tier A (baked strides) cannot reproduce it, e.g. a complex basic with a non-zero
    // phase and no stored imaginary companion stride: the server's raw-record path must run.
    bool needs_raw = false;
};

struct CaseInfo {
    uint32_t n = 0; // case number as the source numbers it
    std::string name;
    bool stored = true; // false: a combination, materialised on request
    std::optional<Recipe> recipe;
};

// How one derived component is produced from the field's (combined) linear components.
struct DerivedComponent {
    DeriveOp op = DeriveOp::Copy;
    std::vector<std::string> args; // component names
    int output_index = 0;          // which of the op's outputs (plane_principal -> 0 = P1, 1 = P2)
};

// One field as it is baked: nodal (AFBL, n_ips == 1) or per element type (AFEL).
struct FieldTable {
    std::string name;
    bool nodal = true;
    std::string elem_type;         // element fields only
    std::size_t rows_entities = 0; // n_points or n_elements
    std::size_t n_ips = 1;
    std::vector<std::string> components;
    std::vector<std::string> linear_components;      // superposed
    std::map<std::string, DerivedComponent> derived; // re-derived after superposition

    std::size_t rows() const {
        return rows_entities * n_ips;
    }
    std::size_t floats_per_step() const {
        return rows() * components.size();
    }
};

// A solver-format front-end. Implementations own their file handles; every call reads (or reuses) only
// what it is asked for, so a reader may be lazy end to end.
class ResultReader {
public:
    virtual ~ResultReader() = default;

    virtual const MeshArrays &mesh() = 0;
    virtual std::vector<CaseInfo> cases() = 0;
    virtual std::vector<FieldTable> fields() = 0;

    // One STORED case's stride of `field`, `field.floats_per_step()` floats, rows x components in the
    // table's order. Returns false when the field has no values for that case.
    virtual bool read_stride(const FieldTable &field, uint32_t case_n, std::span<float> out) = 0;
};

} // namespace adacpp::fea
