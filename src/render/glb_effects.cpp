#include "glb_effects.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string_view>
#include <unordered_map>

#include "json.hpp"

namespace adacpp::render {

namespace {

using json = nlohmann::json;
using Mat = std::array<double, 16>;

template <typename T> T num(const json &o, const char *key, T fallback) {
    const auto it = o.find(key);
    return it != o.end() && it->is_number() ? it->get<T>() : fallback;
}

bool flag(const json &o, const char *key, bool fallback) {
    const auto it = o.find(key);
    return it != o.end() && it->is_boolean() ? it->get<bool>() : fallback;
}

std::array<float, 3> vec3(const json &o, const char *key, std::array<float, 3> fallback) {
    const auto it = o.find(key);
    if (it == o.end() || !it->is_array() || it->size() < 3)
        return fallback;
    for (int k = 0; k < 3; ++k)
        if (!(*it)[k].is_number())
            throw std::invalid_argument(std::string("adacpp.render effects: '") + key + "' must be 3 numbers");
    return {(*it)[0].get<float>(), (*it)[1].get<float>(), (*it)[2].get<float>()};
}

std::array<float, 3> point(const Mat &m, const std::array<float, 3> &p) {
    std::array<float, 3> o{};
    for (int k = 0; k < 3; ++k)
        o[k] = float(m[k] * p[0] + m[4 + k] * p[1] + m[8 + k] * p[2] + m[12 + k]);
    return o;
}

std::array<float, 3> direction(const Mat &m, const std::array<float, 3> &d) {
    std::array<float, 3> o{};
    for (int k = 0; k < 3; ++k)
        o[k] = float(m[k] * d[0] + m[4 + k] * d[1] + m[8 + k] * d[2]);
    return o;
}

ToneMapping tone_mapping(const std::string &s) {
    if (s == "none" || s == "linear")
        return ToneMapping::None;
    if (s == "neutral")
        return ToneMapping::Neutral;
    if (s == "aces")
        return ToneMapping::Aces;
    throw std::invalid_argument("adacpp.render effects: unknown toneMapping '" + s + "'");
}

PostSettings read_post(const json &o) {
    PostSettings p;
    p.hdr = flag(o, "hdr", true);
    if (o.contains("toneMapping") && o["toneMapping"].is_string())
        p.tone_mapping = tone_mapping(o["toneMapping"].get<std::string>());
    p.exposure = num(o, "exposure", p.exposure);
    p.bloom = flag(o, "bloom", p.bloom);
    p.bloom_threshold = num(o, "bloomThreshold", p.bloom_threshold);
    p.bloom_knee = num(o, "bloomKnee", p.bloom_knee);
    p.bloom_strength = num(o, "bloomStrength", p.bloom_strength);
    p.bloom_radius = num(o, "bloomRadius", p.bloom_radius);
    return p;
}

EnvironmentDesc read_environment(const json &o, const Mat &m) {
    EnvironmentDesc e;
    e.enabled = flag(o, "enabled", true);
    e.intensity = num(o, "intensity", e.intensity);
    e.rotation = num(o, "rotation", e.rotation);
    e.up = direction(m, vec3(o, "up", e.up));
    return e;
}

PointLightDesc read_light(const json &o, const Mat &m) {
    PointLightDesc l;
    l.position = point(m, vec3(o, "position", l.position));
    l.color = vec3(o, "color", l.color);
    l.intensity = num(o, "intensity", l.intensity);
    l.range = num(o, "range", l.range);
    l.decay = num(o, "decay", l.decay);
    l.flicker = num(o, "flicker", l.flicker);
    l.flicker_rate = num(o, "flickerRate", l.flicker_rate);
    l.seed = num<uint32_t>(o, "seed", l.seed);
    l.start = num(o, "start", l.start);
    l.end = num(o, "end", l.end);
    l.enabled = flag(o, "enabled", true);
    return l;
}

EmitterDesc read_emitter(const json &o, const Mat &m) {
    EmitterDesc e;
    const std::string preset = o.value("preset", std::string("sparks"));
    if (preset == "smoke")
        e = smoke_emitter({0, 0, 0});
    else if (preset != "sparks")
        throw std::invalid_argument("adacpp.render effects: unknown emitter preset '" + preset + "'");
    e.origin = point(m, vec3(o, "origin", e.origin));
    e.velocity = direction(m, vec3(o, "velocity", e.velocity));
    e.direction = direction(m, vec3(o, "direction", e.direction));
    e.gravity = direction(m, vec3(o, "gravity", e.gravity));
    e.spread = num(o, "spread", e.spread);
    e.speed = num(o, "speed", e.speed);
    e.speed_jitter = num(o, "speedJitter", e.speed_jitter);
    e.rate = num(o, "rate", e.rate);
    e.lifetime = num(o, "lifetime", e.lifetime);
    e.lifetime_jitter = num(o, "lifetimeJitter", e.lifetime_jitter);
    e.start = num(o, "start", e.start);
    e.end = num(o, "end", e.end);
    e.drag = num(o, "drag", e.drag);
    e.radius = num(o, "radius", e.radius);
    e.size_start = num(o, "sizeStart", e.size_start);
    e.size_end = num(o, "sizeEnd", e.size_end);
    e.color_start = vec3(o, "colorStart", e.color_start);
    e.intensity_start = num(o, "intensityStart", e.intensity_start);
    e.alpha_start = num(o, "alphaStart", e.alpha_start);
    e.color_end = vec3(o, "colorEnd", e.color_end);
    e.intensity_end = num(o, "intensityEnd", e.intensity_end);
    e.alpha_end = num(o, "alphaEnd", e.alpha_end);
    e.stretch = num(o, "stretch", e.stretch);
    e.seed = num<uint32_t>(o, "seed", e.seed);
    e.enabled = flag(o, "enabled", true);
    if (o.contains("blend") && o["blend"].is_string()) {
        const std::string b = o["blend"].get<std::string>();
        if (b == "additive")
            e.blend = ParticleBlend::Additive;
        else if (b == "alpha")
            e.blend = ParticleBlend::Alpha;
        else
            throw std::invalid_argument("adacpp.render effects: unknown emitter blend '" + b + "'");
    }
    return e;
}

EmissiveRampDesc read_ramp(const json &o) {
    EmissiveRampDesc r;
    r.hot_color = vec3(o, "hotColor", r.hot_color);
    r.hot_intensity = num(o, "hotIntensity", r.hot_intensity);
    r.cool_color = vec3(o, "coolColor", r.cool_color);
    r.cool_intensity = num(o, "coolIntensity", r.cool_intensity);
    r.start = num(o, "start", r.start);
    r.duration = num(o, "duration", r.duration);
    r.curve = num(o, "curve", r.curve);
    return r;
}

const json &array_of(const json &root, const char *key) {
    static const json empty = json::array();
    const auto it = root.find(key);
    if (it == root.end())
        return empty;
    if (!it->is_array())
        throw std::invalid_argument(std::string("adacpp.render effects: '") + key + "' must be an array");
    return *it;
}

// The text of member `key` of the top-level object in `text` (a JSON object), found without parsing the rest:
// scene extras can be megabytes of draw ranges. Empty when absent.
std::string_view member_text(std::string_view text, std::string_view key) {
    const std::string quoted = "\"" + std::string(key) + "\"";
    size_t at = 0;
    int depth = 0;
    bool in_string = false;
    for (size_t i = 0; i < text.size(); ++i) {
        const char c = text[i];
        if (in_string) {
            if (c == '\\')
                ++i;
            else if (c == '"')
                in_string = false;
            continue;
        }
        if (c == '"') {
            if (depth == 1 && text.compare(i, quoted.size(), quoted) == 0) {
                at = i + quoted.size();
                break;
            }
            in_string = true;
        } else if (c == '{' || c == '[') {
            ++depth;
        } else if (c == '}' || c == ']') {
            --depth;
        }
    }
    if (!at)
        return {};
    while (at < text.size() && (text[at] == ' ' || text[at] == '\t' || text[at] == '\n' || text[at] == '\r'))
        ++at;
    if (at >= text.size() || text[at] != ':')
        return {};
    ++at;
    while (at < text.size() && (text[at] == ' ' || text[at] == '\t' || text[at] == '\n' || text[at] == '\r'))
        ++at;
    if (at >= text.size() || text[at] != '{')
        return {};
    depth = 0;
    in_string = false;
    for (size_t i = at; i < text.size(); ++i) {
        const char c = text[i];
        if (in_string) {
            if (c == '\\')
                ++i;
            else if (c == '"')
                in_string = false;
        } else if (c == '"') {
            in_string = true;
        } else if (c == '{' || c == '[') {
            ++depth;
        } else if ((c == '}' || c == ']') && --depth == 0) {
            return text.substr(at, i + 1 - at);
        }
    }
    return {};
}

} // namespace

std::array<double, 16> glb_effects_transform(const GlbOptions &options) {
    Mat m = options.y_up_to_z_up ? Mat{1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1}
                                 : Mat{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    for (int k = 0; k < 3; ++k)
        m[12 + k] = options.offset[k];
    return m;
}

EffectHandles apply_effects(Renderer &renderer, const std::string &text, const std::vector<GlbObject> *objects,
                            const std::array<double, 16> &transform, bool apply_look) {
    json root;
    try {
        root = json::parse(text);
    } catch (const json::exception &e) {
        throw std::invalid_argument(std::string("adacpp.render effects: ") + e.what());
    }
    if (!root.is_object())
        throw std::invalid_argument("adacpp.render effects: expected an object");
    EffectHandles h;
    try {
        if (root.contains("post") && root["post"].is_object()) {
            h.has_post = true;
            h.post = read_post(root["post"]);
        }
        if (root.contains("environment") && root["environment"].is_object()) {
            h.has_environment = true;
            h.environment = read_environment(root["environment"], transform);
        }
        std::vector<PointLightDesc> lights;
        std::vector<EmitterDesc> emitters;
        for (const json &l : array_of(root, "lights"))
            lights.push_back(read_light(l, transform));
        for (const json &e : array_of(root, "emitters"))
            emitters.push_back(read_emitter(e, transform));

        // Emissive ramps: each onto the segments of the objects it names, and of everything below them.
        std::unordered_map<std::string, uint32_t> by_id;
        std::vector<std::vector<uint32_t>> children;
        if (objects) {
            children.resize(objects->size());
            for (uint32_t i = 0; i < objects->size(); ++i) {
                by_id.emplace((*objects)[i].id, i);
                if ((*objects)[i].parent >= 0 && size_t((*objects)[i].parent) < objects->size())
                    children[(*objects)[i].parent].push_back(i);
            }
        }
        for (const json &r : array_of(root, "emissive")) {
            const EmissiveRampDesc desc = read_ramp(r);
            std::unordered_map<uint32_t, std::vector<uint32_t>> per_instance;
            std::vector<uint32_t> stack;
            for (const json &t : array_of(r, "targets")) {
                const std::string id = t.is_string() ? t.get<std::string>() : t.dump();
                const auto it = by_id.find(id);
                if (it == by_id.end()) {
                    h.unresolved.push_back(id);
                    continue;
                }
                stack.push_back(it->second);
            }
            while (!stack.empty()) {
                const uint32_t o = stack.back();
                stack.pop_back();
                for (const auto &seg : (*objects)[o].segments)
                    per_instance[seg[0]].push_back(seg[1]);
                stack.insert(stack.end(), children[o].begin(), children[o].end());
            }
            if (per_instance.empty())
                continue;
            const uint32_t ramp = renderer.add_emissive_ramp();
            renderer.set_emissive_ramp(ramp, desc);
            h.ramps.push_back(ramp);
            for (const auto &[instance, segs] : per_instance)
                renderer.set_segment_emissive(instance, segs.data(), segs.size(), ramp);
        }
        for (const PointLightDesc &l : lights) {
            const uint32_t id = renderer.add_point_light();
            renderer.set_point_light(id, l);
            h.lights.push_back(id);
        }
        for (const EmitterDesc &e : emitters) {
            const uint32_t id = renderer.add_emitter();
            renderer.set_emitter(id, e);
            h.emitters.push_back(id);
        }
    } catch (const json::exception &e) {
        remove_effects(renderer, h);
        throw std::invalid_argument(std::string("adacpp.render effects: ") + e.what());
    } catch (...) {
        remove_effects(renderer, h);
        throw;
    }
    if (apply_look) {
        if (h.has_post)
            renderer.set_post(h.post);
        if (h.has_environment)
            renderer.set_environment(h.environment);
    }
    return h;
}

EffectHandles apply_glb_effects(Renderer &renderer, const GlbLoadResult &loaded, const GlbOptions &options,
                                bool apply_look) {
    const std::string_view text = member_text(loaded.extras, "ada_effects");
    if (text.empty())
        return {};
    return apply_effects(renderer, std::string(text), &loaded.objects, glb_effects_transform(options), apply_look);
}

namespace {
constexpr Mat kIdentity{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
template <typename F> auto from_json(const std::string &text, F read) {
    try {
        const json o = json::parse(text);
        if (!o.is_object())
            throw std::invalid_argument("adacpp.render effects: expected an object");
        return read(o);
    } catch (const json::exception &e) {
        throw std::invalid_argument(std::string("adacpp.render effects: ") + e.what());
    }
}
} // namespace

PostSettings post_from_json(const std::string &text) {
    return from_json(text, [](const json &o) {
        PostSettings p = read_post(o);
        p.hdr = flag(o, "hdr", false); // alone, nothing is switched on unless asked for
        return p;
    });
}
EnvironmentDesc environment_from_json(const std::string &text) {
    return from_json(text, [](const json &o) { return read_environment(o, kIdentity); });
}
PointLightDesc point_light_from_json(const std::string &text) {
    return from_json(text, [](const json &o) { return read_light(o, kIdentity); });
}
EmitterDesc emitter_from_json(const std::string &text) {
    return from_json(text, [](const json &o) { return read_emitter(o, kIdentity); });
}
EmissiveRampDesc emissive_ramp_from_json(const std::string &text) {
    return from_json(text, [](const json &o) { return read_ramp(o); });
}

void remove_effects(Renderer &renderer, const EffectHandles &h) {
    auto quietly = [](auto &&f) {
        try {
            f();
        } catch (const std::out_of_range &) {
            // already gone (the renderer was cleared)
        }
    };
    for (uint32_t id : h.lights)
        quietly([&] { renderer.remove_point_light(id); });
    for (uint32_t id : h.emitters)
        quietly([&] { renderer.remove_emitter(id); });
    for (uint32_t id : h.ramps)
        quietly([&] { renderer.remove_emissive_ramp(id); });
}

} // namespace adacpp::render
