// Check if the platform is Unix-based
#if defined(__unix__) || defined(__unix)

#include <cstdint>

#endif // Unix platform check

#include <filesystem>
#include "CLI/CLI.hpp"
#include "config_structs.h"
#include <chrono>
#include "core/debug.h"
#include "core/convert.h"
#include "core/helpers.h"
#include "config_utils.h"

void print_status(const GlobalConfig& config) {
    std::cout << "STP2GLB Converter" << "\n";
    std::cout << "STP File: " << config.stpFile << "\n";
    std::cout << "GLB File: " << config.glbFile << "\n\n";
    std::cout << "Tessellation Parameters: " << "\n";
    std::cout << "Linear Deflection: " << config.linearDeflection << "\n";
    std::cout << "Angular Deflection: " << config.angularDeflection << "\n";
    std::cout << "Relative Deflection: " << config.relativeDeflection << "\n\n";
    std::cout << "Debug Parameters: " << "\n";
    std::cout << "Debug Mode: " << config.debug_mode << "\n";
    std::cout << "Solid Only: " << config.solidOnly << "\n";
    std::cout << "Max Geometry Num: " << config.max_geometry_num << "\n";
    std::cout << "Tessellation Timeout: " << config.tessellation_timout << "\n\n";

    // Debug output
    if (!config.filter_names_include.empty())
    {
        std::cout << "Included Filter Names:" << std::endl;
        for (const auto& name : config.filter_names_include)
        {
            std::cout << name << std::endl;
        }
    }
    if (!config.filter_names_exclude.empty())
    {
        std::cout << "Excluded Filter Names:" << std::endl;
        for (const auto& name : config.filter_names_exclude)
        {
            std::cout << name << std::endl;
        }
    }
}

int main(int argc, char* argv[])
{
    CLI::App app{"STEP to GLB converter"};
    app.add_option("--stp", "STEP filepath")->required();
    app.add_option("--glb", "GLB filepath")->required();

    app.add_option("--lin-defl", "Linear deflection")->default_val(0.1)->check(CLI::Range(0.0, 1.0));
    app.add_option("--ang-defl", "Angular deflection")->default_val(0.5)->check(CLI::Range(0.0, 1.0));
    app.add_flag("--rel-defl", "Relative deflection");

    app.add_flag("--debug", "Debug mode. Slower (and experimental), but provides more information about which STEP entities that failed to convert");
    app.add_flag("--solid-only", "Solid only");
    app.add_option("--max-geometry-num", "Maximum number of geometries to convert")->default_val(0);
    app.add_option("--filter-names-include", "Include Filter name. Command separated list")->default_val("");
    app.add_option("--filter-names-file-include", "Include Filter name file")->default_val("");
    app.add_option("--filter-names-exclude", "Exclude Filter name. Command separated list")->default_val("");
    app.add_option("--filter-names-file-exclude", "Exclude Filter name file")->default_val("");
    app.add_option("--tessellation-timeout", "Tessellation timeout")->default_val(30);

    CLI11_PARSE(app, argc, argv);
    GlobalConfig config;

    try {
        config = process_parameters(app);
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }

    print_status(config);
    std::cout << "\n";
    std::cout << "Starting conversion..." << "\n";

    const auto start = std::chrono::high_resolution_clock::now();
    try {
        if (config.debug_mode == 1)
            debug_stp_to_glb(config);
        else
        {
            convert_stp_to_glb(config);
        }
    } catch (std::exception& ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }

    const auto stop = std::chrono::high_resolution_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    const double seconds = static_cast<double>(duration.count()) / 1e6;
    std::cout << "STP converted in: " << std::fixed << std::setprecision(2) << seconds << " seconds" << "\n";

    return 0;
}
