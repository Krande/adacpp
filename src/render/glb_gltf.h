// tinygltf as the GLB reader configures it (internal). Every file of the reader includes it through here:
// the configuration changes the declarations too, and glb_parse.cpp compiles the implementation.
#pragma once

#define TINYGLTF_NO_STB_IMAGE_WRITE
#define TINYGLTF_NO_EXTERNAL_IMAGE
#ifdef __EMSCRIPTEN__
// The renderer does not texture: no image decoder in the browser build (parse_glb installs a no-op image
// loader on every host anyway).
#define TINYGLTF_NO_STB_IMAGE
#endif
#include <tiny_gltf.h>
