#include "glb_objects.h"

#include <algorithm>

namespace adacpp::render::detail {

void ObjectTable::add_hierarchy(const tinygltf::Value &hierarchy) {
    std::vector<std::string> parents;
    for (const auto &id : hierarchy.Keys()) {
        const auto &v = hierarchy.Get(id);
        const int32_t at = by_id(id);
        parents.resize(objects_.size());
        if (!v.IsArray() || v.ArrayLen() < 1)
            continue;
        objects_[at].name = text(v.Get(0));
        if (v.ArrayLen() > 1)
            parents[at] = text(v.Get(1));
    }
    for (size_t i = 0; i < parents.size(); ++i)
        if (auto it = index_.find(parents[i]); it != index_.end() && it->second != int32_t(i))
            objects_[i].parent = it->second;
    sort_by_id_ = true;
}

int32_t ObjectTable::by_id(const std::string &id) {
    auto [it, fresh] = index_.try_emplace(id, int32_t(objects_.size()));
    if (fresh) {
        GlbObject o;
        o.id = o.name = id;
        objects_.push_back(std::move(o));
    }
    return it->second;
}

int32_t ObjectTable::for_node(const tinygltf::Model &model, int node_index, int32_t parent) {
    auto [it, fresh] = index_.try_emplace(std::to_string(node_index), int32_t(objects_.size()));
    if (fresh) {
        const auto &node = model.nodes.at(node_index);
        GlbObject o;
        o.id = it->first;
        o.name = !node.name.empty() || node.mesh < 0 ? node.name : model.meshes.at(node.mesh).name;
        o.parent = parent;
        objects_.push_back(std::move(o));
    }
    return it->second;
}

void ObjectTable::add_segment(int32_t object, uint32_t instance, uint32_t segment, const float bounds[6],
                              uint32_t triangles) {
    GlbObject &o = objects_[object];
    o.segments.push_back({instance, segment});
    o.triangles += triangles;
    for (int k = 0; k < 3; ++k) {
        o.bounds[k] = std::min(o.bounds[k], bounds[k]);
        o.bounds[3 + k] = std::max(o.bounds[3 + k], bounds[3 + k]);
    }
}

void ObjectTable::finish(GlbLoadResult &res) {
    const size_t n = objects_.size();
    std::vector<int32_t> order(n);
    for (size_t i = 0; i < n; ++i)
        order[i] = int32_t(i);
    if (sort_by_id_)
        std::sort(order.begin(), order.end(),
                  [&](int32_t a, int32_t b) { return id_less(objects_[a].id, objects_[b].id); });
    std::vector<std::vector<int32_t>> children(n);
    std::vector<int32_t> roots;
    for (int32_t i : order)
        (objects_[i].parent >= 0 ? children[objects_[i].parent] : roots).push_back(i);
    std::vector<int32_t> place(n, -1), stack;
    int32_t next = 0;
    auto descend = [&](int32_t root) {
        stack.assign(1, root);
        while (!stack.empty()) {
            const int32_t i = stack.back();
            stack.pop_back();
            if (place[i] >= 0)
                continue;
            place[i] = next++;
            for (auto c = children[i].rbegin(); c != children[i].rend(); ++c)
                stack.push_back(*c);
        }
    };
    for (int32_t r : roots)
        descend(r);
    // What no root reaches is a cycle of parents: cut it at its first object.
    for (int32_t i : order)
        if (place[i] < 0) {
            objects_[i].parent = -1;
            descend(i);
        }
    res.objects.resize(n);
    for (size_t i = 0; i < n; ++i) {
        GlbObject &o = objects_[i];
        if (o.parent >= 0)
            o.parent = place[o.parent];
        res.objects[place[i]] = std::move(o);
    }
    // Children sit after their parent: one pass from the end sums every subtree into its root.
    for (size_t i = n; i-- > 0;) {
        const GlbObject &o = res.objects[i];
        if (o.parent < 0)
            continue;
        GlbObject &up = res.objects[o.parent];
        up.triangles += o.triangles;
        for (int k = 0; k < 3; ++k) {
            up.bounds[k] = std::min(up.bounds[k], o.bounds[k]);
            up.bounds[3 + k] = std::max(up.bounds[3 + k], o.bounds[3 + k]);
        }
    }
    for (GlbGeometry &g : res.loaded)
        for (int32_t &object : g.segment_objects)
            if (object >= 0)
                object = place[object];
    objects_.clear();
    index_.clear();
}

std::string ObjectTable::text(const tinygltf::Value &v) {
    if (v.IsString())
        return v.Get<std::string>();
    if (v.IsInt())
        return std::to_string(v.Get<int>());
    if (v.IsReal())
        return std::to_string(int64_t(v.Get<double>()));
    return {};
}

bool ObjectTable::id_less(const std::string &a, const std::string &b) {
    auto digits = [](const std::string &s) {
        return !s.empty() && std::all_of(s.begin(), s.end(), [](char c) { return c >= '0' && c <= '9'; });
    };
    const bool da = digits(a), db = digits(b);
    if (da != db)
        return da;
    if (da && a.size() != b.size())
        return a.size() < b.size();
    return a < b;
}

std::vector<RangeSegment> segments_from_extras(const tinygltf::Value *ranges, size_t index_count,
                                               ObjectTable &objects) {
    struct Range {
        uint32_t start, count;
        const std::string *id;
    };
    std::vector<Range> rs;
    std::vector<std::string> keys;
    if (ranges && ranges->IsObject()) {
        keys = ranges->Keys();
        for (const auto &key : keys) {
            const auto &v = ranges->Get(key);
            if (!v.IsArray() || v.ArrayLen() < 2)
                continue;
            auto num = [](const tinygltf::Value &x) {
                return x.IsInt() ? double(x.Get<int>()) : x.IsReal() ? x.Get<double>() : 0.0;
            };
            rs.push_back({uint32_t(num(v.Get(0))), uint32_t(num(v.Get(1))), &key});
        }
    }
    std::stable_sort(rs.begin(), rs.end(), [](const Range &a, const Range &b) {
        return a.start != b.start ? a.start < b.start : a.count < b.count;
    });
    std::vector<RangeSegment> out;
    uint32_t cursor = 0;
    for (const Range &r : rs) {
        if (r.start >= index_count || r.count == 0)
            continue;
        if (r.start > cursor)
            out.push_back({cursor, r.start - cursor, -1});
        const uint32_t c = std::min<uint32_t>(r.count, uint32_t(index_count) - r.start);
        out.push_back({r.start, c, objects.by_id(*r.id)});
        cursor = std::max(cursor, r.start + c);
    }
    if (!out.empty() && cursor < index_count)
        out.push_back({cursor, uint32_t(index_count) - cursor, -1});
    return out;
}

} // namespace adacpp::render::detail
