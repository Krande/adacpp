// Native pure-C++ IFC -> GLB (no ifcopenshell, no OCC). IfcResolver resolves each product to an
// NgeomRoot (geometry + presentation colour + spatial-structure path + length unit), libtess2
// tessellates it, and the SHARED GLB spill writer (ngeom_glb.h — the same one stream_step_to_glb
// uses) bakes it to metres. So the viewer gets geometry + per-mesh colour + the spatial tree +
// names/guids — a viewer-equivalent GLB (IFC property sets never live in the GLB; they are fetched
// on selection from the source model).
//
// Mirrors stream_step_to_glb but drives IfcResolver (proxy_roots / resolve_product / unit_scale)
// instead of the STEP Resolver. Single-threaded v1: IfcResolver's colour/rel maps are lazily built
// per instance, so parallelising needs a copy_metadata_from-style share (follow-up). Curve-only
// bodies (alignment axes -> GL_LINES) are skipped here (GlbSolid is triangles-only).
#pragma once

#include <atomic>
#include <filesystem>
#include <string>
#include <vector>

#include "../geom/neutral/ada_ext_schema.h"
#include "../geom/neutral/ngeom_glb.h"
#include "../geom/neutral/ngeom_tessellate.h"
#include "ifc_reader.h"
#include "step_reader.h" // StreamIndex

namespace adacpp {

inline long stream_ifc_to_glb(const std::string &in_path, const std::string &out_path, double deflection,
                              double angular_deg, bool meshopt, const std::string &spill_dir = "",
                              double model_scale = 0.0) {
    using namespace adacpp::ngeom;
    adacpp::step::StreamIndex idx = adacpp::step::StreamIndex::from_file(in_path);
    if (!idx.ok())
        return -1;
    adacpp::ifc_read::IfcResolver r(idx);
    std::vector<long> roots = r.proxy_roots();

    TessParams tp;
    tp.deflection = deflection;
    tp.max_angle = angular_deg * PI / 180.0;
    tp.threads = 1;
    tp.model_scale = model_scale;
    const double usc = r.unit_scale(); // metres per file length-unit -> bake to metres (viewer default)

    // Spill dir: private mkdtemp (auto-removed) unless the caller supplied one.
    std::string spill;
    bool remove_after = false;
    char tmpl[] = "/tmp/adacpp_ifcglb_XXXXXX";
    if (spill_dir.empty()) {
        if (char *dir = ::mkdtemp(tmpl)) {
            spill = dir;
            remove_after = true;
        }
    } else {
        std::error_code ec;
        std::filesystem::create_directories(spill_dir, ec);
        if (std::filesystem::is_directory(spill_dir, ec))
            spill = spill_dir;
    }
    if (spill.empty())
        return -1;

    long nwritten = 0;
    bool ok = false;
    { // lane scoped so its temp files are gone before the (maybe) rmdir
        adacpp::glb::GlbSpillWriter lane(spill, 0);
        for (long pid : roots) {
            NgeomRoot root = r.resolve_product(pid);
            r.clear_cache(); // bounded memory: statement/surface caches don't grow across products
            NgeomDoc one;
            one.roots.push_back(std::move(root));
            TessMesh tm = tessellate_doc(one, tp);
            if (tm.indices.empty())
                continue;
            if (tm.mesh_type == MeshType::LINES)
                continue; // curve-only body (alignment axis); GlbSolid is triangles-only in v1
            const NgeomRoot &rr = one.roots[0];
            adacpp::glb::GlbSolid gs;
            gs.positions = std::move(tm.positions);
            gs.indices = std::move(tm.indices);
            gs.color = {rr.cr, rr.cg, rr.cb, rr.ca}; // grey default when !has_color
            gs.transforms = rr.transforms;
            gs.id = rr.id;
            if (!rr.instance_paths.empty() && !rr.instance_paths[0].empty())
                gs.product_name = rr.instance_paths[0].back().second;
            gs.instance_paths = rr.instance_paths;
            if (usc != 1.0) {
                const float s = (float) usc;
                for (float &p : gs.positions)
                    p *= s;
                for (auto &M : gs.transforms) {
                    M[12] *= s;
                    M[13] *= s;
                    M[14] *= s;
                }
            }
            lane.add(gs);
            ++nwritten;
        }
        std::vector<adacpp::glb::GlbSpillWriter *> lane_ptrs{&lane};
        const std::string ada_ext = adacpp::ada_ext::AdaDesignAndAnalysisExtension{}.to_json();
        ok = adacpp::glb::write_glb_merged(out_path, lane_ptrs, ada_ext, meshopt);
    }
    if (remove_after)
        ::rmdir(spill.c_str());
    return ok ? nwritten : -1;
}

} // namespace adacpp
