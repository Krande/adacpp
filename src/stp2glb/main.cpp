// Standalone OCC-free STEP -> GLB CLI. Drives the native threaded streaming pipeline
// (adacpp::stream_step_to_glb) — the same OCC-free engine the wasm module and the python
// `stream_step_to_glb` binding use. No OCCT, no nanobind, no Python: self-contained.

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>

#include "CLI/CLI.hpp"

#include "../cad/step_to_glb_stream.h" // adacpp::stream_step_to_glb (threaded, OCC-free)

int main(int argc, char *argv[]) {
    CLI::App app{"STEP to GLB converter (OCC-free native streaming pipeline)"};

    std::string stp_file;
    std::string glb_file;
    double deflection = 2.0;
    double angular_deg = 20.0;
    int num_threads = 0; // 0 = all hardware cores
    bool meshopt = true; // EXT_meshopt_compression baked inline by default

    app.add_option("--stp", stp_file, "STEP input filepath")->required();
    app.add_option("--glb", glb_file, "GLB output filepath")->required();
    // --lin-defl kept as an alias for backwards compatibility with the old OCCT CLI.
    app.add_option("--deflection,--lin-defl", deflection, "Linear deflection")->default_val(2.0);
    app.add_option("--angular-deg", angular_deg, "Angular deflection (degrees)")->default_val(20.0);
    app.add_option("--num-threads", num_threads, "Worker threads (0 = all hardware cores)")->default_val(0);
    app.add_flag("--meshopt,!--no-meshopt", meshopt, "Bake EXT_meshopt_compression inline (default ON)");

    CLI11_PARSE(app, argc, argv);

    const unsigned hw = std::thread::hardware_concurrency();
    const int resolved_threads = num_threads > 0 ? num_threads : (int) (hw > 1 ? hw : 1);

    std::cout << "STP2GLB Converter (OCC-free native streaming)\n";
    std::cout << "STP File:           " << stp_file << "\n";
    std::cout << "GLB File:           " << glb_file << "\n";
    std::cout << "Linear Deflection:  " << deflection << "\n";
    std::cout << "Angular Deflection: " << angular_deg << " deg\n";
    std::cout << "Threads:            " << resolved_threads << (num_threads == 0 ? " (auto)" : "") << "\n";
    std::cout << "Meshopt:            " << (meshopt ? "on" : "off") << "\n\n";
    std::cout << "Starting conversion...\n";

    const auto start = std::chrono::high_resolution_clock::now();
    long nsolids = -1;
    try {
        nsolids = adacpp::stream_step_to_glb(stp_file, glb_file, deflection, angular_deg, num_threads, meshopt);
    } catch (const std::exception &ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }
    const auto stop = std::chrono::high_resolution_clock::now();
    const double seconds = std::chrono::duration<double>(stop - start).count();

    if (nsolids < 0) {
        std::cerr << "Error: conversion failed (could not read input or write output)\n";
        return 1;
    }

    std::uintmax_t out_size = 0;
    std::error_code ec;
    if (std::filesystem::exists(glb_file, ec))
        out_size = std::filesystem::file_size(glb_file, ec);

    std::cout << "Solids written:     " << nsolids << "\n";
    std::cout << "Output size:        " << out_size << " bytes\n";
    std::cout << "Converted in:       " << std::fixed << std::setprecision(2) << seconds << " seconds\n";

    return 0;
}
