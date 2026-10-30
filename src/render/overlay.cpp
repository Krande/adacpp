#include "overlay.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <utility>

#define STB_TRUETYPE_IMPLEMENTATION
#define STBTT_STATIC
#include <stb_truetype.h>

#include "overlay_fonts.h"

namespace adacpp::render {

namespace {

float clamp01(float v) {
    return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

Rgba hex(uint32_t rgb) {
    return {((rgb >> 16) & 0xFF) / 255.0f, ((rgb >> 8) & 0xFF) / 255.0f, (rgb & 0xFF) / 255.0f, 1.0f};
}

// Straight-alpha "over" onto an RGBA8 image, in sRGB space as Canvas2D blends.
struct Canvas {
    uint8_t *px;
    int w, h;

    void blend(int x, int y, const Rgba &c, float coverage) {
        if (x < 0 || y < 0 || x >= w || y >= h)
            return;
        const float sa = c[3] * coverage;
        if (sa <= 0.0f)
            return;
        uint8_t *p = px + (size_t(y) * w + x) * 4;
        const float da = p[3] / 255.0f;
        const float oa = sa + da * (1.0f - sa);
        for (int k = 0; k < 3; ++k) {
            const float o = (c[k] * sa + p[k] / 255.0f * da * (1.0f - sa)) / oa;
            p[k] = uint8_t(std::lround(clamp01(o) * 255.0f));
        }
        p[3] = uint8_t(std::lround(clamp01(oa) * 255.0f));
    }

    // Fill the shape a signed distance function describes (negative inside), antialiased over a pixel,
    // within the box [x0, x1) x [y0, y1). `color_at` may vary the colour per pixel.
    template <class Sdf, class Color> void fill(float x0, float y0, float x1, float y1, Sdf sdf, Color color_at) {
        const int ix0 = std::max(0, int(std::floor(x0)) - 1), iy0 = std::max(0, int(std::floor(y0)) - 1);
        const int ix1 = std::min(w, int(std::ceil(x1)) + 1), iy1 = std::min(h, int(std::ceil(y1)) + 1);
        for (int y = iy0; y < iy1; ++y)
            for (int x = ix0; x < ix1; ++x) {
                const float cx = x + 0.5f, cy = y + 0.5f;
                const float cov = clamp01(0.5f - sdf(cx, cy));
                if (cov > 0.0f)
                    blend(x, y, color_at(cx, cy), cov);
            }
    }
};

float sd_round_box(float px, float py, float x0, float y0, float x1, float y1, float r) {
    const float cx = 0.5f * (x0 + x1), cy = 0.5f * (y0 + y1);
    const float qx = std::fabs(px - cx) - 0.5f * (x1 - x0) + r, qy = std::fabs(py - cy) - 0.5f * (y1 - y0) + r;
    const float ox = std::max(qx, 0.0f), oy = std::max(qy, 0.0f);
    return std::sqrt(ox * ox + oy * oy) + std::min(std::max(qx, qy), 0.0f) - r;
}

float sd_segment(float px, float py, float ax, float ay, float bx, float by) {
    const float pax = px - ax, pay = py - ay, bax = bx - ax, bay = by - ay;
    const float len2 = bax * bax + bay * bay;
    const float t = len2 > 0.0f ? clamp01((pax * bax + pay * bay) / len2) : 0.0f;
    const float dx = pax - bax * t, dy = pay - bay * t;
    return std::sqrt(dx * dx + dy * dy);
}

// A filled box with rounded corners and an optional 1-pixel-ish border (`border_w` device pixels).
void box(Canvas &cv, float x0, float y0, float x1, float y1, float r, const Rgba &fill, const Rgba &border,
         float border_w) {
    auto shape = [&](float px, float py) { return sd_round_box(px, py, x0, y0, x1, y1, r); };
    cv.fill(x0, y0, x1, y1, shape, [&](float, float) { return fill; });
    if (border_w > 0.0f && border[3] > 0.0f) {
        // The border sits inside the box, as CSS draws it.
        auto ring = [&](float px, float py) { return std::fabs(shape(px, py) + 0.5f * border_w) - 0.5f * border_w; };
        cv.fill(x0, y0, x1, y1, ring, [&](float, float) { return border; });
    }
}

// UTF-8 -> code points (invalid bytes become U+FFFD).
std::vector<int> code_points(const std::string &s) {
    std::vector<int> out;
    for (size_t i = 0; i < s.size();) {
        const unsigned char c = s[i];
        int cp, n;
        if (c < 0x80) {
            cp = c;
            n = 1;
        } else if ((c >> 5) == 0x6) {
            cp = c & 0x1F;
            n = 2;
        } else if ((c >> 4) == 0xE) {
            cp = c & 0x0F;
            n = 3;
        } else if ((c >> 3) == 0x1E) {
            cp = c & 0x07;
            n = 4;
        } else {
            out.push_back(0xFFFD);
            ++i;
            continue;
        }
        if (i + n > s.size()) {
            out.push_back(0xFFFD);
            break;
        }
        for (int k = 1; k < n; ++k)
            cp = (cp << 6) | (static_cast<unsigned char>(s[i + k]) & 0x3F);
        out.push_back(cp);
        i += n;
    }
    return out;
}

class Font {
public:
    explicit Font(const unsigned char *data) {
        stbtt_InitFont(&info_, data, stbtt_GetFontOffsetForIndex(data, 0));
        stbtt_GetFontVMetrics(&info_, &ascent_, &descent_, &line_gap_);
    }
    float scale(float px) const {
        return stbtt_ScaleForMappingEmToPixels(&info_, px);
    }
    float ascent(float px) const {
        return ascent_ * scale(px);
    }
    float descent(float px) const {
        return descent_ * scale(px); // negative
    }
    float measure(const std::string &s, float px) const {
        const float sc = scale(px);
        float x = 0.0f;
        int prev = 0;
        for (int cp : code_points(s)) {
            if (prev)
                x += sc * stbtt_GetCodepointKernAdvance(&info_, prev, cp);
            int adv, lsb;
            stbtt_GetCodepointHMetrics(&info_, cp, &adv, &lsb);
            x += sc * adv;
            prev = cp;
        }
        return x;
    }
    // Text with its baseline at `baseline`, starting at `x`.
    void draw(Canvas &cv, const std::string &s, float x, float baseline, float px, const Rgba &color) const {
        const float sc = scale(px);
        const int base = int(std::lround(baseline));
        int prev = 0;
        std::vector<unsigned char> glyph;
        for (int cp : code_points(s)) {
            if (prev)
                x += sc * stbtt_GetCodepointKernAdvance(&info_, prev, cp);
            int adv, lsb, x0, y0, x1, y1;
            stbtt_GetCodepointHMetrics(&info_, cp, &adv, &lsb);
            const float shift = x - std::floor(x);
            stbtt_GetCodepointBitmapBoxSubpixel(&info_, cp, sc, sc, shift, 0.0f, &x0, &y0, &x1, &y1);
            const int gw = x1 - x0, gh = y1 - y0;
            if (gw > 0 && gh > 0) {
                glyph.assign(size_t(gw) * gh, 0);
                stbtt_MakeCodepointBitmapSubpixel(&info_, glyph.data(), gw, gh, gw, sc, sc, shift, 0.0f, cp);
                const int ox = int(std::floor(x)) + x0, oy = base + y0;
                for (int j = 0; j < gh; ++j)
                    for (int i = 0; i < gw; ++i)
                        if (glyph[size_t(j) * gw + i])
                            cv.blend(ox + i, oy + j, color, glyph[size_t(j) * gw + i] / 255.0f);
            }
            x += sc * adv;
            prev = cp;
        }
    }

private:
    stbtt_fontinfo info_{};
    int ascent_ = 0, descent_ = 0, line_gap_ = 0;
};

const Font &sans() {
    static const Font f(kFontSans);
    return f;
}
const Font &bold() {
    static const Font f(kFontSansBold);
    return f;
}
const Font &mono() {
    static const Font f(kFontMono);
    return f;
}

// Greedy word wrap to `width` (a word longer than a line is broken, as CSS break-words does).
std::vector<std::string> wrap(const Font &font, const std::string &text, float px, float width) {
    std::vector<std::string> lines;
    std::string line, word;
    auto flush_word = [&]() {
        if (word.empty())
            return;
        const std::string candidate = line.empty() ? word : line + " " + word;
        if (font.measure(candidate, px) <= width || line.empty()) {
            line = candidate;
        } else {
            lines.push_back(line);
            line = word;
        }
        while (font.measure(line, px) > width && line.size() > 1) {
            size_t cut = line.size() - 1;
            while (cut > 1 && font.measure(line.substr(0, cut), px) > width)
                --cut;
            while (cut > 1 && (static_cast<unsigned char>(line[cut]) & 0xC0) == 0x80)
                --cut; // keep UTF-8 sequences whole
            lines.push_back(line.substr(0, cut));
            line = line.substr(cut);
        }
        word.clear();
    };
    for (char c : text) {
        if (c == ' ')
            flush_word();
        else
            word.push_back(c);
    }
    flush_word();
    if (!line.empty())
        lines.push_back(line);
    return lines;
}

// A text line box of height `line_h` at `top`: the baseline, centred as CSS centres the glyphs' ascent and
// descent in the line (half-leading).
float baseline_in(const Font &font, float px, float top, float line_h) {
    const float content = font.ascent(px) - font.descent(px);
    return top + 0.5f * (line_h - content) + font.ascent(px);
}

Rgba opaque(const Rgb &c) {
    return {c[0], c[1], c[2], 1.0f};
}

Rgb sample_gradient(const std::vector<Rgb> &stops, float t) {
    if (stops.empty())
        return {0, 0, 0};
    if (stops.size() == 1)
        return stops[0];
    const float f = clamp01(t) * float(stops.size() - 1);
    const size_t i = std::min(size_t(f), stops.size() - 2);
    const float u = f - float(i);
    Rgb c;
    for (int k = 0; k < 3; ++k)
        c[k] = stops[i][k] + (stops[i + 1][k] - stops[i][k]) * u;
    return c;
}

void draw_gizmo(Canvas &cv, const float view[16], const GizmoOverlay &g, float s) {
    struct Bubble {
        float dir[3];
        Rgba primary, secondary;
        const char *label;
        bool line;
        float x = 0, y = 0, z = 0;
    };
    const Rgba xs[2] = {hex(0xf73c3c), hex(0x942424)}, ys[2] = {hex(0x6ccb26), hex(0x417a17)},
               zs[2] = {hex(0x178cf0), hex(0x0e5490)};
    Bubble bubbles[6] = {{{1, 0, 0}, xs[0], xs[1], "X", true},    {{0, 1, 0}, ys[0], ys[1], "Y", true},
                         {{0, 0, 1}, zs[0], zs[1], "Z", true},    {{-1, 0, 0}, xs[0], xs[1], "-X", false},
                         {{0, -1, 0}, ys[0], ys[1], "-Y", false}, {{0, 0, -1}, zs[0], zs[1], "-Z", false}};
    const float size = g.size * s;
    const float cx = cv.w - g.margin_x * s - 0.5f * size, cy = cv.h - g.margin_y * s - 0.5f * size;
    const float reach = 0.5f * size - 0.5f * g.bubble * s - g.padding * s;
    for (Bubble &b : bubbles) {
        // The axis in view space: the camera's inverse rotation, as the gizmo applies to each direction.
        float v[3];
        for (int i = 0; i < 3; ++i)
            v[i] = view[i] * b.dir[0] + view[4 + i] * b.dir[1] + view[8 + i] * b.dir[2];
        b.x = cx + v[0] * reach;
        b.y = cy - v[1] * reach;
        b.z = v[2];
    }
    Bubble *order[6];
    for (int i = 0; i < 6; ++i)
        order[i] = &bubbles[i];
    std::stable_sort(order, order + 6, [](const Bubble *a, const Bubble *b) { return a->z < b->z; });
    const Font &font = bold();
    const float font_px = g.font_px * s;
    const Rgba ink = hex(0x151515);
    for (const Bubble *b : order) {
        const Rgba &color = b->z >= -0.01f ? b->primary : b->secondary;
        const float r = g.bubble * s;
        cv.fill(
            b->x - r, b->y - r, b->x + r, b->y + r,
            [&](float px, float py) { return std::hypot(px - b->x, py - b->y) - r; },
            [&](float, float) { return color; });
        if (b->line) {
            const float hw = 0.5f * g.line_width * s;
            cv.fill(
                std::min(cx, b->x) - hw, std::min(cy, b->y) - hw, std::max(cx, b->x) + hw, std::max(cy, b->y) + hw,
                [&](float px, float py) { return sd_segment(px, py, cx, cy, b->x, b->y) - hw; },
                [&](float, float) { return color; });
        }
        // textAlign center, textBaseline middle: the middle of the em box on the bubble's centre.
        const float w = font.measure(b->label, font_px);
        const float baseline = b->y + 0.5f * (font.ascent(font_px) + font.descent(font_px));
        font.draw(cv, b->label, b->x - 0.5f * w, baseline, font_px, ink);
    }
}

void draw_legend(Canvas &cv, const LegendOverlay &l, float s) {
    const float px = l.font_px * s, line_h = 1.25f * px; // text-[11px] leading-tight
    const float pad = 8.0f * s, gap = 8.0f * s, radius = 4.0f * s, hair = std::max(1.0f, s);
    const float width = l.width * s, inner = width - 2.0f * pad;

    std::vector<std::pair<std::string, const Font *>> header;
    for (size_t i = 0; i < l.header.size(); ++i) {
        const Font &f = i == 0 ? bold() : sans();
        for (auto &line : wrap(f, l.header[i], px, inner))
            header.emplace_back(line, &f);
    }
    std::vector<std::string> note_lines;
    if (!l.note.empty())
        note_lines = wrap(sans(), l.note, px, inner - 2.0f * gap);

    float body = 0.0f;
    if (!note_lines.empty()) {
        body = 2.0f * 12.0f * s + note_lines.size() * line_h; // px-2 py-3, dashed box
    } else if (!l.categories.empty()) {
        const float row = std::max(14.0f * s, line_h);
        body = l.categories.size() * row + (l.categories.size() - 1) * 2.0f * s;
    } else if (!l.bands.empty()) {
        body = l.bands.size() * std::max(20.0f * s, line_h) + line_h;
    } else if (!l.gradient.empty()) {
        body = l.bar_height * s;
    }
    const float height = pad + header.size() * line_h + (header.empty() ? 0.0f : gap) + body + pad;
    const float x0 = cv.w - l.margin_x * s - width, y0 = l.margin_y * s;
    box(cv, x0, y0, x0 + width, y0 + height, radius, l.panel_bg, l.panel_border, hair);

    float y = y0 + pad;
    const float x = x0 + pad;
    for (const auto &[text, font] : header) {
        font->draw(cv, text, x, baseline_in(*font, px, y, line_h), px, l.text);
        y += line_h;
    }
    if (!header.empty())
        y += gap;

    const Font &m = mono();
    if (!note_lines.empty()) {
        Rgba border = l.panel_border;
        box(cv, x, y, x + inner, y + body, radius, {0, 0, 0, 0}, border, hair);
        float ty = y + 12.0f * s;
        Rgba dim = l.text;
        dim[3] *= 0.7f;
        for (const auto &line : note_lines) {
            sans().draw(cv, line, x + gap, baseline_in(sans(), px, ty, line_h), px, dim);
            ty += line_h;
        }
    } else if (!l.categories.empty()) {
        const float sw = 14.0f * s, row = std::max(sw, line_h);
        for (const auto &c : l.categories) {
            const float sy = y + 0.5f * (row - sw);
            box(cv, x, sy, x + sw, sy + sw, 1.0f * s, opaque(c.color), l.panel_border, hair);
            sans().draw(cv, c.label, x + sw + gap, baseline_in(sans(), px, y + 0.5f * (row - line_h), line_h), px,
                        l.text);
            y += row + 2.0f * s;
        }
    } else if (!l.bands.empty()) {
        // flex-col-reverse: the top boundary first, then the bands from the highest down.
        const float sw = 20.0f * s, row = std::max(sw, line_h);
        m.draw(cv, format_legend_value(l.bands.back().to), x + sw + gap, baseline_in(m, px, y, line_h), px, l.text);
        y += line_h;
        for (size_t i = l.bands.size(); i-- > 0;) {
            const LegendBand &b = l.bands[i];
            box(cv, x, y + 0.5f * (row - sw), x + sw, y + 0.5f * (row + sw), 1.0f * s, opaque(b.color), l.panel_border,
                hair);
            m.draw(cv, format_legend_value(b.from), x + sw + gap, baseline_in(m, px, y + 0.5f * (row - line_h), line_h),
                   px, l.text);
            y += row;
        }
    } else if (!l.gradient.empty()) {
        // The bar, minimum at the bottom (linear-gradient to top), and the ticks spread over its height.
        const float bw = 20.0f * s, bh = l.bar_height * s;
        cv.fill(
            x, y, x + bw, y + bh,
            [&](float qx, float qy) { return sd_round_box(qx, qy, x, y, x + bw, y + bh, radius); },
            [&](float, float qy) { return opaque(sample_gradient(l.gradient, 1.0f - (qy - y) / bh)); });
        const size_t n = l.ticks.size();
        for (size_t i = 0; i < n; ++i) {
            const float top = n > 1 ? y + float(i) * (bh - line_h) / float(n - 1) : y;
            m.draw(cv, format_legend_value(l.ticks[i]), x + bw + gap, baseline_in(m, px, top, line_h), px, l.text);
        }
    }
}

} // namespace

std::string format_legend_value(double value) {
    if (!std::isfinite(value))
        return "\xE2\x80\x94"; // em dash
    const double magnitude = std::fabs(value);
    char buf[64];
    if ((magnitude != 0.0 && magnitude < 1e-3) || magnitude >= 1e6) {
        // JavaScript's toExponential(3): "1.235e+7", "-4.560e-4".
        std::snprintf(buf, sizeof(buf), "%.3e", value);
        std::string s(buf);
        const size_t e = s.find('e');
        std::string mant = s.substr(0, e), exp = s.substr(e + 1);
        const char sign = exp[0];
        exp = exp.substr(1);
        exp.erase(0, std::min(exp.find_first_not_of('0'), exp.size() - 1));
        return mant + "e" + sign + exp;
    }
    if (value == 0.0)
        return "0";
    // toLocaleString(undefined, {maximumSignificantDigits: 6}) in en-US.
    const int exponent = int(std::floor(std::log10(magnitude)));
    const int decimals = std::max(0, 5 - exponent);
    std::snprintf(buf, sizeof(buf), "%.*f", decimals, value);
    std::string s(buf);
    if (s.find('.') != std::string::npos) {
        s.erase(s.find_last_not_of('0') + 1);
        if (s.back() == '.')
            s.pop_back();
    }
    const bool negative = s[0] == '-';
    std::string digits = negative ? s.substr(1) : s;
    const size_t dot = digits.find('.');
    std::string integer = digits.substr(0, dot), fraction = dot == std::string::npos ? "" : digits.substr(dot);
    // Rounding can add a digit (999999.5 -> 1000000): the 6 significant digits are still what is shown.
    for (int i = int(integer.size()) - 3; i > 0; i -= 3)
        integer.insert(size_t(i), ",");
    if (integer == "-0" || (integer == "0" && fraction.empty()))
        return "0";
    return (negative ? "-" : "") + integer + fraction;
}

void draw_overlay(uint8_t *rgba, uint32_t width, uint32_t height, const float view[16], const Overlay &overlay) {
    Canvas cv{rgba, int(width), int(height)};
    const float s = overlay.scale > 0.0f ? overlay.scale : 1.0f;
    if (overlay.legend.show)
        draw_legend(cv, overlay.legend, s);
    if (overlay.gizmo.show)
        draw_gizmo(cv, view, overlay.gizmo, s);
}

} // namespace adacpp::render
