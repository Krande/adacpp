// Overlay for headless renders: Ada Studio's orientation gizmo and FEA colour legend, drawn onto the
// read-back image on the CPU, so posters and the CLI show what Studio shows around the model. (In the
// browser Studio draws both itself, in Canvas2D / React.)
//
// The gizmo ports OrientationGizmo.ts (jrj2211's three-orientation-gizmo, MIT); the legend ports
// ColorLegend.tsx's layout and number formatting. Sizes are CSS pixels, multiplied by Overlay::scale.
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace adacpp::render {

using Rgb = std::array<float, 3>;  // sRGB, 0..1
using Rgba = std::array<float, 4>; // sRGB, 0..1, straight alpha

struct GizmoOverlay {
    bool show = false;
    // OrientationGizmo's defaults as Studio sets them on desktop (addOrientationGizmo.ts).
    float size = 150.0f;
    float padding = 8.0f;
    float bubble = 10.0f;
    float line_width = 2.0f;
    float font_px = 10.0f;
    float margin_x = 8.0f; // from the right edge
    float margin_y = 8.0f; // from the bottom edge
};

struct LegendBand {
    double from = 0.0, to = 0.0;
    Rgb color{0, 0, 0};
};
struct LegendCategory {
    std::string label;
    Rgb color{0, 0, 0};
};

// One of three scales, by what is filled in: categories, else bands, else the continuous gradient.
struct LegendOverlay {
    bool show = false;
    // Field path (drawn bold), then the reduction [unit], case, ... lines, as the legend's header.
    std::vector<std::string> header;
    std::vector<Rgb> gradient;     // continuous: colours from the minimum (first) to the maximum (last)
    std::vector<double> ticks;     // continuous: values top to bottom (contourTicks order: max first)
    std::vector<LegendBand> bands; // banded: from the minimum up
    std::vector<LegendCategory> categories;
    std::string note;          // drawn instead of a scale (e.g. "Result colouring is off ...")
    float width = 224.0f;      // w-56
    float bar_height = 256.0f; // h-64
    float font_px = 11.0f;
    float margin_x = 20.0f; // from the right edge
    float margin_y = 20.0f; // from the top edge
    // Studio's default panel theme ("--ada-panel-*").
    Rgba panel_bg{30 / 255.0f, 41 / 255.0f, 59 / 255.0f, 0.62f};
    Rgba panel_border{148 / 255.0f, 163 / 255.0f, 184 / 255.0f, 0.35f};
    Rgba text{241 / 255.0f, 245 / 255.0f, 249 / 255.0f, 1.0f};
};

struct Overlay {
    GizmoOverlay gizmo;
    LegendOverlay legend;
    float scale = 1.0f; // device pixels per CSS pixel
};

// Draw `overlay` over a tightly packed RGBA8 image (straight alpha, top row first). `view` is the camera's
// world -> view matrix (column-major), which turns the gizmo.
void draw_overlay(uint8_t *rgba, uint32_t width, uint32_t height, const float view[16], const Overlay &overlay);

// ColorLegend.tsx's formatValue: toExponential(3) outside [1e-3, 1e6), else up to 6 significant digits
// with thousands separators (en-US toLocaleString).
std::string format_legend_value(double value);

} // namespace adacpp::render
