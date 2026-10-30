#include "overlay_json.h"

#ifndef __EMSCRIPTEN__

#include <stdexcept>

// The json.hpp tinygltf is built with, so one nlohmann version ends up in the binary.
#include <json.hpp>

namespace adacpp::render {

namespace {

using json = nlohmann::json;

Rgb rgb(const json &j) {
    if (!j.is_array() || j.size() < 3)
        throw std::invalid_argument("overlay: a colour is [r, g, b] in 0..1");
    return {j[0].get<float>(), j[1].get<float>(), j[2].get<float>()};
}

Rgba rgba(const json &j) {
    const Rgb c = rgb(j);
    return {c[0], c[1], c[2], j.size() > 3 ? j[3].get<float>() : 1.0f};
}

template <class T> void get(const json &j, const char *key, T &out) {
    if (j.contains(key) && !j[key].is_null())
        out = j[key].get<T>();
}

} // namespace

Overlay overlay_from_json(const std::string &text) {
    const json j = json::parse(text);
    Overlay o;
    get(j, "scale", o.scale);
    if (j.contains("gizmo")) {
        const json &g = j["gizmo"];
        o.gizmo.show = g.is_boolean() ? g.get<bool>() : g.is_object();
        if (g.is_object()) {
            get(g, "size", o.gizmo.size);
            get(g, "padding", o.gizmo.padding);
            get(g, "bubble", o.gizmo.bubble);
            get(g, "line_width", o.gizmo.line_width);
            get(g, "font_px", o.gizmo.font_px);
            get(g, "margin_x", o.gizmo.margin_x);
            get(g, "margin_y", o.gizmo.margin_y);
        }
    }
    if (j.contains("legend") && j["legend"].is_object()) {
        const json &l = j["legend"];
        LegendOverlay &lg = o.legend;
        lg.show = true;
        get(l, "header", lg.header);
        get(l, "note", lg.note);
        get(l, "ticks", lg.ticks);
        if (l.contains("gradient"))
            for (const auto &c : l["gradient"])
                lg.gradient.push_back(rgb(c));
        if (l.contains("bands"))
            for (const auto &b : l["bands"])
                lg.bands.push_back({b.at("from").get<double>(), b.at("to").get<double>(), rgb(b.at("color"))});
        if (l.contains("categories"))
            for (const auto &c : l["categories"])
                lg.categories.push_back({c.at("label").get<std::string>(), rgb(c.at("color"))});
        get(l, "width", lg.width);
        get(l, "bar_height", lg.bar_height);
        get(l, "font_px", lg.font_px);
        get(l, "margin_x", lg.margin_x);
        get(l, "margin_y", lg.margin_y);
        if (l.contains("panel_bg"))
            lg.panel_bg = rgba(l["panel_bg"]);
        if (l.contains("panel_border"))
            lg.panel_border = rgba(l["panel_border"]);
        if (l.contains("text"))
            lg.text = rgba(l["text"]);
    }
    return o;
}

} // namespace adacpp::render

#endif
