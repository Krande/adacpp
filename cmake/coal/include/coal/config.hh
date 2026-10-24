// coal's generated config header, for adacpp's static in-tree build of coal (cmake/deps_coal.cmake).
//
// coal's own build generates this with jrl-cmakemodules; adacpp builds only coal's sources, as one
// static library, so all it needs are the macros coal's headers use: no symbol export (static), and the
// version of the sources deps_coal.cmake pins.
#ifndef COAL_CONFIG_HH
#define COAL_CONFIG_HH

#define COAL_VERSION "3.0.4"
#define COAL_MAJOR_VERSION 3
#define COAL_MINOR_VERSION 0
#define COAL_PATCH_VERSION 4

#define COAL_VERSION_AT_LEAST(major, minor, patch)                                                                     \
    (COAL_MAJOR_VERSION > major ||                                                                                     \
     (COAL_MAJOR_VERSION >= major &&                                                                                   \
      (COAL_MINOR_VERSION > minor || (COAL_MINOR_VERSION >= minor && COAL_PATCH_VERSION >= patch))))

#define COAL_DLLAPI
#define COAL_LOCAL

#endif // COAL_CONFIG_HH
