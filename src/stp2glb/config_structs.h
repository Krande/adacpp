#ifndef CONFIG_STRUCTS_H
#define CONFIG_STRUCTS_H

#include <vector>
#include <filesystem>

struct BuildConfig {
    bool build_bspline_surf;
};

struct GlobalConfig {
    std::filesystem::path stpFile;
    std::filesystem::path glbFile;

    // Conversion parameters
    bool debug_mode;

    double linearDeflection;
    double angularDeflection;
    bool relativeDeflection;

    // Debug parameters
    bool solidOnly;
    int max_geometry_num;
    int tessellation_timout;

    std::vector<std::string> filter_names_include;
    std::vector<std::string> filter_names_exclude;

    BuildConfig buildConfig;
};



#endif //CONFIG_STRUCTS_H
