// Minimal JSON emission shared by the writers that hand structured data to a consumer -- the glTF
// JSON chunk and the IFC member scan's JSONL.
//
// Deliberately NOT a JSON library. These writers build one known shape each and never parse, so a
// dependency would buy nothing and cost every dep-free wasm module that links them. What is shared
// is the part that is easy to get subtly wrong and impossible to notice: escaping, and printing a
// double a consumer can read back.

#ifndef ADACPP_JSON_WRITE_H
#define ADACPP_JSON_WRITE_H

#include <cstdio>
#include <string>

namespace adacpp::jsonw {

// JSON string escaping. Control characters become a space rather than \uXXXX: these strings are CAD
// names, a literal control byte in one is corruption rather than intent, and a space keeps the name
// readable instead of turning it into an escape nobody will decode by eye. UTF-8 passes through --
// the escape set is ASCII-only, so multi-byte sequences are never split.
inline std::string escape(const std::string &s) {
    std::string o;
    o.reserve(s.size() + 2);
    for (char c : s) {
        switch (c) {
        case '"':
            o += "\\\"";
            break;
        case '\\':
            o += "\\\\";
            break;
        case '\n':
            o += "\\n";
            break;
        case '\r':
            o += "\\r";
            break;
        case '\t':
            o += "\\t";
            break;
        default:
            if ((unsigned char) c < 0x20)
                o += ' ';
            else
                o += c;
        }
    }
    return o;
}

// A double as JSON. 12 significant digits: ~1e-12 relative, which is orders below any CAD tolerance
// these files carry, and far shorter than the 17 a round-trip-exact printing would need on every
// coordinate of every member. Non-finite values are written as `null` -- JSON has no NaN, and a
// consumer reading `null` learns the truth (this value is not a number) instead of failing to parse
// the whole line.
inline std::string num(double v) {
    if (!(v == v) || v > 1e308 || v < -1e308)
        return "null";
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.12g", v);
    return std::string(buf);
}

} // namespace adacpp::jsonw

#endif // ADACPP_JSON_WRITE_H
