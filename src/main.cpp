// Check if the platform is Unix-based
#if defined(__unix__) || defined(__unix)

#include <cstdint>

#endif // Unix platform check

#include <chrono>
#include "CLI/CLI.hpp"
#include "cadit/occt/step_to_glb.h"


int main(int argc, char *argv[]) {
    CLI::App app{"STEP to GLB converter"};
    app.add_option("--stp", "STEP filepath")->required();
    app.add_option("--glb", "GLB filepath")->required();
    CLI11_PARSE(app, argc, argv);

    auto stp_file = app.get_option("--stp")->results()[0];
    auto glb_file = app.get_option("--glb")->results()[0];

    try {

        auto start = std::chrono::high_resolution_clock::now();

        stp_to_glb(stp_file, glb_file);

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

        std::cout << "STP converted in: " << duration.count() << " microseconds" << std::endl;

    } catch (...) {
        std::cout << "Unknown error occurred." << std::endl;
        return 1;
    }

    return 0;
}
