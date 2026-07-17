// Standalone OCC-free CAD converter CLI (`adacpp`). Drives the native, OCC-free engines shared with
// the wasm modules and the Python bindings — the conversion is chosen from the input/output file
// extensions:
//
//   STEP -> GLB    adacpp::stream_step_to_glb                     (Part-21 reader + libtess2/cdt)
//   IFC  -> GLB    adacpp::stream_ifc_to_glb                      (IfcResolver + same tess stack)
//   STEP -> IFC    adacpp::brep_convert::write_ifc_file_impl      (analytic B-rep -> IFC4X3/IFC4)
//   IFC  -> STEP   adacpp::brep_convert::write_ifc_to_step_impl   (analytic B-rep -> AP242 STEP)
//
// No OCCT, no nanobind, no Python: self-contained.

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "CLI/CLI.hpp"

#include "../cad/brep_file_convert.h" // write_ifc_file_impl / write_ifc_to_step_impl (STEP <-> IFC)
#include "../cad/ifc_to_glb_stream.h" // adacpp::stream_ifc_to_glb (IFC -> GLB)
#include "../cad/step_to_glb_stream.h" // adacpp::stream_step_to_glb (STEP -> GLB)

namespace {

enum class Fmt { Step, Ifc, Glb, Unknown };

Fmt fmt_from_path(const std::string &path) {
    std::string ext = std::filesystem::path(path).extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return (char) std::tolower(c); });
    if (ext == ".stp" || ext == ".step" || ext == ".p21")
        return Fmt::Step;
    if (ext == ".ifc")
        return Fmt::Ifc;
    if (ext == ".glb")
        return Fmt::Glb;
    return Fmt::Unknown;
}

const char *fmt_name(Fmt f) {
    switch (f) {
    case Fmt::Step:
        return "STEP";
    case Fmt::Ifc:
        return "IFC";
    case Fmt::Glb:
        return "GLB";
    default:
        return "?";
    }
}

} // namespace

int main(int argc, char *argv[]) {
    CLI::App app{"adacpp - OCC-free CAD converter: STEP/IFC -> GLB, STEP <-> IFC "
                 "(conversion inferred from the input/output file extensions)"};

    std::string input;
    std::string output;
    double deflection = 2.0;
    double angular_deg = 20.0;
    int num_threads = 0;       // 0 = all hardware cores (GLB paths)
    bool meshopt = true;       // EXT_meshopt_compression baked inline by default (GLB paths)
    bool profile = false;      // env-gated StepProfiler instrumentation (STEP->GLB)
    bool quiet = false;        // suppress the param echo + progress; keep errors + the final result line
    bool face_regions = false; // bake per-face clickable regions into scenes[0].extras (GLB paths)
    std::string pipeline;      // tessellation track: "" / libtess2 (default) | cdt (GLB paths)
    bool pin_boundary = true;  // libtess2 option (GLB paths)
    double model_scale = 0.0;  // >0 => adaptive per-surface density (GLB paths)
    std::string spill_dir;     // per-lane GLB spill dir; empty => private auto-removed mkdtemp
    std::string schema = "IFC4X3_ADD2"; // target IFC schema (STEP->IFC)
    long max_solids = 0;                // cap on solids/products emitted for STEP<->IFC (0 = no cap)

    // Positional `adacpp input output` — the natural CLI shape. --stp/--glb are kept as hidden
    // back-compat aliases for the old STEP->GLB-only CLI so existing scripts keep working.
    app.add_option("input,-i,--input,--stp", input, "Input filepath (.stp/.step/.p21, .ifc)")->required();
    app.add_option("output,-o,--output,--glb", output, "Output filepath (.glb, .ifc, .stp/.step)")->required();

    // --lin-defl kept as an alias for backwards compatibility with the old OCCT CLI.
    app.add_option("--deflection,--lin-defl", deflection, "Linear deflection (GLB paths)")->default_val(2.0);
    app.add_option("--angular-deg", angular_deg, "Angular deflection (degrees)")->default_val(20.0);
    app.add_option("--num-threads", num_threads, "Worker threads (0 = all hardware cores; GLB paths)")
        ->default_val(0);
    app.add_flag("--meshopt,!--no-meshopt", meshopt, "Bake EXT_meshopt_compression inline for GLB (default ON)");
    app.add_flag("--profile", profile, "Print [STEPPROF] phase/memory/per-solid timing to stderr (STEP->GLB)");
    app.add_flag("--face-regions", face_regions,
                 "Bake per-face clickable regions into scenes[0].extras (face_ranges_node<m>); GLB paths, opt-in");
    // Only the NEUTRAL tracks mesh the OCC-free native path this CLI drives; restrict --pipeline to the
    // neutral set derived from track_infos() (so the two can't drift), and reject the rest.
    std::vector<std::string> neutral_tracks;
    std::string pipeline_help = "Tessellation track for GLB (OCC-free native):";
    for (const auto &t : adacpp::ngeom::track_infos())
        if (t.neutral) {
            neutral_tracks.emplace_back(t.name);
            pipeline_help += std::string(" ") + t.name + (t.is_default ? " (default)" : "");
        }
    app.add_option("--pipeline", pipeline, pipeline_help)->check(CLI::IsMember(neutral_tracks));
    app.add_flag("--pin-boundary,!--no-pin-boundary", pin_boundary,
                 "libtess2: emit boundary vertices at their shared-edge point instead of this face's own "
                 "surface re-projection (default ON; halves cracks for ~3%; GLB paths)");
    app.add_option("--model-scale", model_scale,
                   "Model bbox diagonal for adaptive per-surface density (0 = fixed angle; GLB paths)")
        ->default_val(0.0);
    app.add_option("--spill-dir", spill_dir,
                   "Directory for the per-lane GLB spill files (GLB paths; default = a private "
                   "auto-removed /tmp dir)");
    app.add_option("--schema", schema, "Target IFC schema for STEP->IFC")
        ->check(CLI::IsMember({"IFC4X3_ADD2", "IFC4"}))
        ->default_val("IFC4X3_ADD2");
    app.add_option("--max-solids", max_solids, "Cap on solids/products emitted for STEP<->IFC (0 = no cap)")
        ->default_val(0);
    app.add_flag("--quiet", quiet, "Suppress the param echo + progress; keep errors and the final result line");

    CLI11_PARSE(app, argc, argv);

    const Fmt in_fmt = fmt_from_path(input);
    const Fmt out_fmt = fmt_from_path(output);

    auto is = [&](Fmt a, Fmt b) { return in_fmt == a && out_fmt == b; };
    const bool supported = is(Fmt::Step, Fmt::Glb) || is(Fmt::Ifc, Fmt::Glb) || is(Fmt::Step, Fmt::Ifc) ||
                           is(Fmt::Ifc, Fmt::Step);
    if (!supported) {
        std::cerr << "Error: unsupported conversion " << fmt_name(in_fmt) << " -> " << fmt_name(out_fmt) << " (from '"
                  << input << "' -> '" << output << "').\n"
                  << "Supported: STEP->GLB, IFC->GLB, STEP->IFC, IFC->STEP. Give input/output paths with the "
                  << "matching extensions (.stp/.step/.p21, .ifc, .glb).\n";
        return 2;
    }

    // The profiler reads these env vars in its constructor; set them BEFORE calling the engine.
    if (profile) {
        ::setenv("ADACPP_STEP_PROFILE", "1", 1);
        ::setenv("ADACPP_STEP_SOLID_TIMING", "1", 1);
    }

    const int resolved_threads = num_threads > 0 ? num_threads : (int) adacpp::effective_concurrency();

    if (!quiet) {
        std::cout << "adacpp converter (OCC-free native)\n";
        std::cout << "Conversion:         " << fmt_name(in_fmt) << " -> " << fmt_name(out_fmt) << "\n";
        std::cout << "Input:              " << input << "\n";
        std::cout << "Output:             " << output << "\n";
        std::cout << "Angular Deflection: " << angular_deg << " deg\n";
        if (out_fmt == Fmt::Glb) {
            std::cout << "Linear Deflection:  " << deflection << "\n";
            std::cout << "Threads:            " << resolved_threads << (num_threads == 0 ? " (auto)" : "") << "\n";
            std::cout << "Meshopt:            " << (meshopt ? "on" : "off") << "\n";
            if (!spill_dir.empty())
                std::cout << "Spill dir:          " << spill_dir << "\n";
        } else if (out_fmt == Fmt::Ifc) {
            std::cout << "IFC schema:         " << schema << "\n";
        }
        if (max_solids > 0 && (out_fmt == Fmt::Ifc || out_fmt == Fmt::Step))
            std::cout << "Max solids:         " << max_solids << "\n";
        std::cout << "\nStarting conversion...\n";
    }

    const auto start = std::chrono::high_resolution_clock::now();
    long count = -1;   // solids (GLB) / solids_out (IFC, STEP); <0 = failure
    long skipped = 0;  // IFC->STEP: products with non-analytic geometry the native path had to drop
    bool brep = false; // an IFC/STEP writer path (vs a GLB tessellation path)
    try {
        if (is(Fmt::Step, Fmt::Glb)) {
            count = adacpp::stream_step_to_glb(input, output, deflection, angular_deg, num_threads, meshopt, spill_dir,
                                               model_scale, face_regions, pipeline, pin_boundary);
        } else if (is(Fmt::Ifc, Fmt::Glb)) {
            count = adacpp::stream_ifc_to_glb(input, output, deflection, angular_deg, meshopt, spill_dir, model_scale,
                                              num_threads, pipeline, face_regions, pin_boundary);
        } else if (is(Fmt::Step, Fmt::Ifc)) {
            brep = true;
            adacpp::ifc_emit::FileStats fs =
                adacpp::brep_convert::write_ifc_file_impl(input, output, schema, deflection, angular_deg, max_solids);
            count = fs.solids_out;
        } else { // IFC -> STEP
            brep = true;
            adacpp::ifc_emit::FileStats fs =
                adacpp::brep_convert::write_ifc_to_step_impl(input, output, deflection, angular_deg, max_solids);
            count = fs.solids_out;
            skipped = fs.products_skipped;
        }
    } catch (const std::exception &ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }
    const auto stop = std::chrono::high_resolution_clock::now();
    const double seconds = std::chrono::duration<double>(stop - start).count();

    if (count < 0) {
        std::cerr << "Error: conversion failed (could not read input or write output)\n";
        return 1;
    }
    if (count == 0) {
        std::cerr << "Error: nothing written (no convertible geometry in the input)\n";
        return 1;
    }

    std::uintmax_t out_size = 0;
    std::error_code ec;
    if (std::filesystem::exists(output, ec))
        out_size = std::filesystem::file_size(output, ec);

    // IFC->STEP: the OCC-free analytic reader can't represent extrusion/CSG/tessellated products, so a
    // non-zero skip count means the output is INCOMPLETE. Unlike the wasm/python path, this CLI has no
    // OCC fallback, so surface it rather than silently dropping geometry.
    if (skipped > 0)
        std::cerr << "Warning: " << skipped << " product(s) had non-analytic geometry and were skipped "
                  << "(the OCC-free native path is incomplete for this input)\n";

    if (quiet) {
        std::cout << output << "  " << (brep ? "products=" : "solids=") << count << "  bytes=" << out_size << "  "
                  << std::fixed << std::setprecision(2) << seconds << "s\n";
    } else {
        std::cout << (brep ? "Products written:   " : "Solids written:     ") << count << "\n";
        std::cout << "Output size:        " << out_size << " bytes\n";
        std::cout << "Converted in:       " << std::fixed << std::setprecision(2) << seconds << " seconds\n";
    }

    return 0;
}
