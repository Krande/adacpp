// Unit tests for the Part-21 tokenizer (src/cadit/step/step_part21.h).
// Build: g++ -std=c++20 -O2 -I src/cadit/step tests/step/test_step_part21.cpp -o /tmp/p21 && /tmp/p21
#include <cstdio>
#include <string>
#include <vector>

#include "step_part21.h"

using namespace adacpp::step;

static int g_fail = 0;
#define CHECK(cond, msg)                                                                                               \
    do {                                                                                                               \
        if (!(cond)) {                                                                                                 \
            std::printf("FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__);                                               \
            ++g_fail;                                                                                                  \
        }                                                                                                              \
    } while (0)

static bool dclose(double a, double b) {
    double d = a - b;
    return (d < 0 ? -d : d) <= 1e-12;
}

// Parse one statement body and return it. Takes a string_view over a STRING LITERAL (static
// storage) so the Instance's string_views into the source stay valid after this returns.
static Instance stmt(std::string_view s) {
    Instance inst;
    parse_statement(s, inst);
    return inst;
}

static void test_scalars() {
    Instance in = stmt("#1=T(1,2.5,-3,4.E-2,'ab',.STEEL.,#7,$,*,+5)");
    CHECK(in.id == 1 && in.type == "T", "id+type");
    CHECK(in.args.size() == 10, "10 args");
    CHECK(in.args[0].kind == Kind::Int && in.args[0].i == 1, "int");
    CHECK(in.args[1].kind == Kind::Real && dclose(in.args[1].r, 2.5), "real");
    CHECK(in.args[2].kind == Kind::Int && in.args[2].i == -3, "neg int");
    CHECK(in.args[3].kind == Kind::Real && dclose(in.args[3].r, 0.04), "real exponent");
    CHECK(in.args[4].kind == Kind::Str && in.args[4].s == "ab", "string");
    CHECK(in.args[5].kind == Kind::Enum && in.args[5].s == "STEEL", "enum");
    CHECK(in.args[6].kind == Kind::Ref && in.args[6].i == 7, "ref");
    CHECK(in.args[7].kind == Kind::Null, "dollar null");
    CHECK(in.args[8].kind == Kind::Star, "star");
    CHECK(in.args[9].kind == Kind::Int && in.args[9].i == 5, "leading plus int");
}

static void test_nested_list_and_typed() {
    // a B-spline-ish: nested list of refs + a typed measure (keyword + list)
    Instance in = stmt("#2=X((#1,#2,#3),LENGTH_MEASURE(1.0))");
    CHECK(in.args.size() == 3, "list + keyword + list = 3 elems");
    CHECK(in.args[0].kind == Kind::List && in.args[0].items.size() == 3, "nested ref list of 3");
    CHECK(in.args[0].items[2].kind == Kind::Ref && in.args[0].items[2].i == 3, "nested ref value");
    CHECK(in.args[1].kind == Kind::Keyword && in.args[1].s == "LENGTH_MEASURE", "typed keyword");
    CHECK(in.args[2].kind == Kind::List && dclose(in.args[2].items[0].as_double(), 1.0), "typed list arg");
}

static void test_string_escape() {
    Instance in = stmt("#3=P('O''Brien',#1)");
    CHECK(in.args[0].kind == Kind::Str && in.args[0].s == "O''Brien", "raw '' kept in view");
    CHECK(unescape(in.args[0].s) == "O'Brien", "unescape collapses ''");
    CHECK(in.args[1].kind == Kind::Ref && in.args[1].i == 1, "arg after escaped string");
}

static void test_complex_record() {
    // rational B-spline style complex instance
    Instance in =
        stmt("#9=(BOUNDED_CURVE()B_SPLINE_CURVE(3,(#1,#2))RATIONAL_B_SPLINE_CURVE((1.,2.))REPRESENTATION_ITEM(''))");
    CHECK(in.complex && in.id == 9, "complex id");
    CHECK(in.subs.size() == 4, "4 sub-records");
    CHECK(in.subs[1].first == "B_SPLINE_CURVE", "sub name");
    CHECK(in.subs[1].second.size() == 2 && in.subs[1].second[0].i == 3, "sub args: degree");
    CHECK(in.subs[1].second[1].kind == Kind::List && in.subs[1].second[1].items.size() == 2, "sub args: ctrl list");
    CHECK(in.subs[2].first == "RATIONAL_B_SPLINE_CURVE", "rational sub name");
}

static void test_scan_instances() {
    const char *step = "ISO-10303-21;\n"
                       "HEADER;\n"
                       "FILE_NAME('a;b','2020',(''),(''),'','','');\n" // ; inside a string!
                       "ENDSEC;\n"
                       "DATA;\n"
                       "#10=CARTESIAN_POINT('',(0.,0.,0.));\n"
                       "#11 = DIRECTION ( '' , ( 0., 0., 1. ) ) ;\n" // whitespace tolerance
                       "/* a comment */\n"
                       "#12=CLOSED_SHELL('shell',(#10,#11));\n"
                       "ENDSEC;\n"
                       "END-ISO-10303-21;\n";
    std::vector<Instance> got;
    scan_instances(step, [&](const Instance &i) { got.push_back(i); });
    CHECK(got.size() == 3, "3 instances (header skipped, ;-in-string not a boundary)");
    CHECK(got[0].id == 10 && got[0].type == "CARTESIAN_POINT", "inst 0");
    CHECK(got[0].args.size() == 2 && got[0].args[1].items.size() == 3, "point coords list");
    CHECK(got[1].id == 11 && got[1].type == "DIRECTION", "inst 1 (whitespace)");
    CHECK(got[2].id == 12 && got[2].type == "CLOSED_SHELL", "inst 2 (after comment)");
    CHECK(got[2].args[1].items.size() == 2 && got[2].args[1].items[0].i == 10, "shell face refs");
}

int main() {
    test_scalars();
    test_nested_list_and_typed();
    test_string_escape();
    test_complex_record();
    test_scan_instances();
    if (g_fail == 0)
        std::printf("step part21: ALL PASS\n");
    else
        std::printf("step part21: %d FAILURES\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
