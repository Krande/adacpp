// Effects as plain data (JSON) -> Renderer: point lights, emitters and emissive ramps, and optionally the
// post-processing and environment settings. The JSON a scene carries in its GLB scene extras under
// "ada_effects" (adapy's EffectSpec), or that a host hands over on its own. Schema: docs/render_effects.md.
#pragma once

#include <array>
#include <string>
#include <vector>

#include "glb_scene.h"
#include "renderer.h"

namespace adacpp::render {

// What apply_effects added, so a host can remove it with the model (remove_effects), and the look settings
// the description named (applied only with `apply_look`).
struct EffectHandles {
    std::vector<uint32_t> lights, emitters, ramps;
    bool has_post = false, has_environment = false;
    PostSettings post;
    EnvironmentDesc environment;
    // Emissive targets that named no object of the model.
    std::vector<std::string> unresolved;
};

// `json`: the effects object. `objects`: the model's objects (GlbLoadResult::objects), what emissive targets
// name by id; null resolves none. Positions, directions and the gravity are taken through `transform`
// (column-major; see glb_effects_transform), the placement of the model they belong to. Throws
// std::invalid_argument on JSON that does not parse or does not fit the schema.
EffectHandles apply_effects(Renderer &renderer, const std::string &json, const std::vector<GlbObject> *objects,
                            const std::array<double, 16> &transform, bool apply_look);
// The transform load_glb_bytes places a GLB by (GlbOptions::y_up_to_z_up and offset).
std::array<double, 16> glb_effects_transform(const GlbOptions &options);
// The "ada_effects" member of a loaded GLB's scene extras, if any (empty handles otherwise).
EffectHandles apply_glb_effects(Renderer &renderer, const GlbLoadResult &loaded, const GlbOptions &options,
                                bool apply_look);
void remove_effects(Renderer &renderer, const EffectHandles &handles);

// One item of the schema from its JSON object text, in world coordinates (no transform): what the browser and
// Python bindings take, so every host reads the same keys. Missing keys keep the defaults of effects.h.
PostSettings post_from_json(const std::string &json);
EnvironmentDesc environment_from_json(const std::string &json);
PointLightDesc point_light_from_json(const std::string &json);
EmitterDesc emitter_from_json(const std::string &json);
EmissiveRampDesc emissive_ramp_from_json(const std::string &json);

} // namespace adacpp::render
