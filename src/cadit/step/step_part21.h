// Part-21 (ISO-10303-21) tokenizer for the native STEP -> NGEOM reader.
//
// Header-only, no OCCT, no allocation for scalars (string/keyword/enum are string_views into the
// source buffer). Mirrors the adapy Python reader's grammar (src/ada/cadit/step/read/
// stream_reader.py: _parse_value / _parse_seq / _parse_statement) so the two readers resolve the
// same geometry. A *typed* value like LENGTH_MEASURE(1.0) is NOT a node: the keyword and the
// (1.0) list are two adjacent sequence elements, exactly as the Python reader treats them.
#pragma once

#include <cctype>
#include <charconv>
#include <cstdint>
#if defined(__EMSCRIPTEN__)
#include <cerrno> // strtod fallback (emscripten libc++ deletes floating-point std::from_chars)
#include <cstdlib>
#include <cstring>
#endif
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace adacpp::step {

enum class Kind {
    Int,     // integer literal           -> i
    Real,    // real literal              -> r
    Str,     // 'quoted' (raw inner, '' not unescaped) -> s
    Keyword, // bare non-numeric token (type names, etc.) -> s
    Enum,    // .NAME. (dots stripped)    -> s
    Ref,     // #123                      -> i (the id)
    List,    // ( ... )                   -> items
    Null,    // $ (or empty)
    Star,    // *
};

struct Value {
    Kind kind = Kind::Null;
    long i = 0;               // Int value / Ref id
    double r = 0.0;           // Real value
    std::string_view s;       // Str (raw inner) / Keyword / Enum (no dots)
    std::vector<Value> items; // List elements

    bool is_ref() const {
        return kind == Kind::Ref;
    }
    bool is_list() const {
        return kind == Kind::List;
    }
    bool is_null() const {
        return kind == Kind::Null;
    }
    // numeric accessor that promotes Int->double (STEP often writes integral reals as ints)
    double as_double() const {
        return kind == Kind::Real ? r : (kind == Kind::Int ? (double) i : 0.0);
    }
};

// One parsed instance: simple `#id=TYPE(args)` or complex `#id=(NAME(args)NAME(args)...)`.
struct Instance {
    long id = 0;
    bool complex = false;
    std::string_view type;                                             // simple only
    std::vector<Value> args;                                           // simple only
    std::vector<std::pair<std::string_view, std::vector<Value>>> subs; // complex only
};

namespace p21_detail {

inline void skip_ws(const char *&p, const char *end) {
    while (p < end) {
        char c = *p;
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
            ++p;
            continue;
        }
        if (c == '/' && p + 1 < end && p[1] == '*') { // /* comment */
            p += 2;
            while (p + 1 < end && !(p[0] == '*' && p[1] == '/'))
                ++p;
            p = (p + 1 < end) ? p + 2 : end;
            continue;
        }
        break;
    }
}

inline std::string_view trim(std::string_view t) {
    size_t a = 0, b = t.size();
    while (a < b && (t[a] == ' ' || t[a] == '\t' || t[a] == '\r' || t[a] == '\n'))
        ++a;
    while (b > a && (t[b - 1] == ' ' || t[b - 1] == '\t' || t[b - 1] == '\r' || t[b - 1] == '\n'))
        --b;
    return t.substr(a, b - a);
}

// Classify a trimmed bare token as Int / Real / Keyword (mirrors Python _parse_scalar:
// int() first, then float(), else string).
inline Value classify_scalar(std::string_view tok) {
    Value v;
    if (tok.empty()) {
        v.kind = Kind::Keyword;
        v.s = tok;
        return v;
    }
    // std::from_chars accepts a leading '-' but not '+'; STEP/Python accept '+', so peel it.
    std::string_view num = (tok.front() == '+') ? tok.substr(1) : tok;
    const char *nb = num.data();
    const char *ne = nb + num.size();
    long iv = 0;
    auto [pi, eci] = std::from_chars(nb, ne, iv);
    if (eci == std::errc() && pi == ne) {
        v.kind = Kind::Int;
        v.i = iv;
        return v;
    }
    double dv = 0.0;
#if defined(__EMSCRIPTEN__) || defined(_LIBCPP_VERSION)
    // libc++ (macOS/clang + emscripten) deletes floating-point std::from_chars; parse via strtod on a
    // NUL-terminated copy (STEP reals are short, always C-locale '.'). endp==end keeps "fully consumed".
    const size_t nlen = (size_t) (ne - nb);
    char tmp[64];
    if (nlen < sizeof(tmp)) {
        std::memcpy(tmp, nb, nlen);
        tmp[nlen] = '\0';
        char *endp = nullptr;
        errno = 0;
        dv = std::strtod(tmp, &endp);
        if (errno == 0 && endp == tmp + nlen) {
            v.kind = Kind::Real;
            v.r = dv;
            return v;
        }
    }
#else
    auto [pd, ecd] = std::from_chars(nb, ne, dv);
    if (ecd == std::errc() && pd == ne) {
        v.kind = Kind::Real;
        v.r = dv;
        return v;
    }
#endif
    v.kind = Kind::Keyword;
    v.s = tok;
    return v;
}

inline std::vector<Value> parse_seq(const char *&p, const char *end, char end_char);

inline Value parse_value(const char *&p, const char *end) {
    skip_ws(p, end);
    Value v;
    if (p >= end)
        return v; // Null
    char c = *p;
    if (c == '(') {
        ++p;
        v.kind = Kind::List;
        v.items = parse_seq(p, end, ')');
        return v;
    }
    if (c == '\'') {
        ++p;
        const char *s0 = p;
        while (p < end) {
            if (*p == '\'') {
                if (p + 1 < end && p[1] == '\'') { // '' escaped quote -> stays in the raw range
                    p += 2;
                    continue;
                }
                break;
            }
            ++p;
        }
        v.kind = Kind::Str;
        v.s = std::string_view(s0, p - s0);
        if (p < end)
            ++p; // consume closing quote
        return v;
    }
    if (c == '#') {
        ++p;
        long id = 0;
        while (p < end && *p >= '0' && *p <= '9')
            id = id * 10 + (*p++ - '0');
        v.kind = Kind::Ref;
        v.i = id;
        return v;
    }
    if (c == '.') {
        ++p;
        const char *s0 = p;
        while (p < end && *p != '.')
            ++p;
        v.kind = Kind::Enum;
        v.s = std::string_view(s0, p - s0);
        if (p < end)
            ++p; // consume closing dot
        return v;
    }
    if (c == '*') {
        ++p;
        v.kind = Kind::Star;
        return v;
    }
    if (c == '$') {
        ++p;
        v.kind = Kind::Null;
        return v;
    }
    // bare token: number or keyword. Runs until , ( ) — NOT whitespace (trimmed after), per Python.
    const char *s0 = p;
    while (p < end && *p != ',' && *p != '(' && *p != ')')
        ++p;
    return classify_scalar(trim(std::string_view(s0, p - s0)));
}

inline std::vector<Value> parse_seq(const char *&p, const char *end, char end_char) {
    std::vector<Value> vals;
    while (p < end) {
        skip_ws(p, end);
        if (p >= end)
            break;
        char c = *p;
        if (c == end_char) {
            ++p;
            break;
        }
        if (c == ',') {
            ++p;
            continue;
        }
        vals.push_back(parse_value(p, end));
    }
    return vals;
}

} // namespace p21_detail

// Parse a single statement body (no trailing ';'), e.g. "#12=AXIS2_PLACEMENT_3D('',#10,#11,#13)".
// Returns false for header lines / anything not starting with '#id='.
inline bool parse_statement(std::string_view stmt, Instance &out) {
    using namespace p21_detail;
    const char *p = stmt.data();
    const char *end = p + stmt.size();
    skip_ws(p, end);
    if (p >= end || *p != '#')
        return false;
    ++p;
    long id = 0;
    bool any = false;
    while (p < end && *p >= '0' && *p <= '9') {
        id = id * 10 + (*p++ - '0');
        any = true;
    }
    if (!any)
        return false;
    skip_ws(p, end);
    if (p >= end || *p != '=')
        return false;
    ++p;
    skip_ws(p, end);
    if (p >= end)
        return false;
    out.id = id;
    out.complex = false;
    out.type = {};
    out.args.clear();
    out.subs.clear();
    if (*p == '(') { // complex record: (NAME(args)NAME(args)...)
        out.complex = true;
        ++p;
        while (true) {
            skip_ws(p, end);
            if (p >= end || *p == ')')
                break;
            const char *n0 = p;
            while (p < end && (std::isalnum((unsigned char) *p) || *p == '_'))
                ++p;
            std::string_view name(n0, p - n0);
            skip_ws(p, end);
            if (p < end && *p == '(') {
                ++p;
                out.subs.emplace_back(name, parse_seq(p, end, ')'));
            } else {
                break;
            }
        }
        return true;
    }
    // simple record: TYPE(args)
    const char *t0 = p;
    while (p < end && (std::isalnum((unsigned char) *p) || *p == '_'))
        ++p;
    out.type = std::string_view(t0, p - t0);
    skip_ws(p, end);
    if (p < end && *p == '(') {
        ++p;
        out.args = parse_seq(p, end, ')');
    }
    return true;
}

// Scan a whole Part-21 buffer, invoking cb(const Instance&) for every `#id=...;` record.
// Statement boundaries are the ';' that are outside 'quoted strings' and /* comments */.
// Header keywords (ISO-10303-21, HEADER, FILE_*, ENDSEC, DATA, END-ISO-10303-21) parse to
// `false` and are skipped.
template <class F> inline void scan_instances(std::string_view buf, F &&cb) {
    using namespace p21_detail;
    const char *p = buf.data();
    const char *end = p + buf.size();
    while (p < end) {
        skip_ws(p, end);
        if (p >= end)
            break;
        const char *s0 = p;
        bool in_str = false;
        while (p < end) {
            char c = *p;
            if (in_str) {
                if (c == '\'') {
                    if (p + 1 < end && p[1] == '\'') {
                        p += 2;
                        continue;
                    }
                    in_str = false;
                }
                ++p;
                continue;
            }
            if (c == '\'') {
                in_str = true;
                ++p;
                continue;
            }
            if (c == '/' && p + 1 < end && p[1] == '*') {
                p += 2;
                while (p + 1 < end && !(p[0] == '*' && p[1] == '/'))
                    ++p;
                p = (p + 1 < end) ? p + 2 : end;
                continue;
            }
            if (c == ';')
                break;
            ++p;
        }
        std::string_view stmt(s0, p - s0);
        if (p < end)
            ++p; // consume ';'
        Instance inst;
        if (parse_statement(stmt, inst))
            cb(inst);
    }
}

// Unescape a Part-21 string value (the raw inner of a Kind::Str): collapse '' -> ' . Other
// Part-21 escapes (\X2\..\X0\ unicode etc.) are left as-is for now (geometry needs only names).
inline std::string unescape(std::string_view raw) {
    std::string out;
    out.reserve(raw.size());
    for (size_t i = 0; i < raw.size(); ++i) {
        if (raw[i] == '\'' && i + 1 < raw.size() && raw[i + 1] == '\'') {
            out.push_back('\'');
            ++i;
        } else {
            out.push_back(raw[i]);
        }
    }
    return out;
}

} // namespace adacpp::step
