// The overlay spec as JSON, for the CLI (--overlay) and Python (set_overlay): one parser for both hosts.
//
//   {"scale": 2,
//    "gizmo": true | {"size": 150, "bubble": 10, ...},
//    "legend": {"header": ["Displacement", "Magnitude [m]", "Case: 3"],
//               "gradient": [[r, g, b], ...], "ticks": [max, ..., min],     (continuous)
//               "bands": [{"from": a, "to": b, "color": [r, g, b]}, ...],   (banded, from the minimum up)
//               "categories": [{"label": "S355", "color": [r, g, b]}, ...],
//               "note": "...", "width": 224, "bar_height": 256, ...}}
//
// Colours are sRGB in 0..1. Unknown keys are ignored.
#pragma once

#ifndef __EMSCRIPTEN__

#include <string>

#include "overlay.h"

namespace adacpp::render {

Overlay overlay_from_json(const std::string &json);

} // namespace adacpp::render

#endif
