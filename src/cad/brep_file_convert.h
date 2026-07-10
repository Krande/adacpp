// Shared, dep-free B-rep file→file writers (STEP↔IFC), extracted from cad_py_wrap.cpp so BOTH the
// nanobind module AND the OCC-free embind wasm writer (brep_writer_wasm.cpp) call ONE implementation.
// No nanobind / OCC / ifcopenshell — only the native readers (StreamIndex/Resolver, IfcResolver) and
// the dep-free emitters (ifc_emit::BrepEmitter, step_emit::StepBrepEmitter). See ifc_to_glb_stream.h
// for the sibling pattern.
//
// Contents (moved verbatim; `static` → `inline` for the header):
//   STEP→IFC:  IfcPath, emit_solid_ifc, emit_spatial_tree, ifc_length_unit_line, ifc_header_block,
//              write_ifc_file_impl
//   IFC→STEP:  step_header_block, emit_solid_step, IfcPath2, emit_step_assembly_tree,
//              write_ifc_to_step_impl
// The nanobind file's remaining writers (blobs_to_ifc_impl, *_parallel_impl, write_step_file_impl,
// step_parity_impl) call these via `using namespace adacpp::brep_convert;`.
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "../cadit/step/step_reader.h"
#include "../geom/neutral/ngeom_decode.h"
#include "../geom/neutral/ngeom_profile.h"
#include "ifc_emit.h"
#include "ifc_reader.h"
#include "step_emit.h"

namespace adacpp::brep_convert {

// Emit ONE solid's IFC into `buf` via `em` (its id counter must be pre-seeded — to a continuous
// running id for the serial path, or to the solid's disjoint id block for the parallel path). Shared
// by both writers. Flat solids (transforms empty) -> one IfcBuildingElementProxy; instanced solids ->
// one IfcRepresentationMap + one IfcMappedItem+IfcCartesianTransformationOperator3D+proxy per world
// placement (geometry shared). Appends every proxy id to `proxies`. guid_seed must be unique per
// solid (instances use guid_seed+k; reserve a gap > max instances).
using IfcPath = std::vector<std::pair<int, std::string>>; // root-first (rep_id, product_name) levels

inline void emit_solid_ifc(adacpp::ifc_emit::BrepEmitter &em, std::string &buf, const adacpp::ngeom::NgeomRoot &root,
                           long sid, uint64_t guid_seed, std::vector<long> &proxies,
                           std::vector<IfcPath> *proxy_paths) {
    using namespace adacpp::ifc_emit;
    std::string rep_type = "AdvancedBrep";
    long solid = em.emit_solid(buf, root, rep_type); // brep for face sets, else the analytic IFC solid
    if (!solid)
        return;
    // Presentation colour: style the shared solid item so it colours every (mapped) instance. The
    // NgeomRoot already carries the colour resolved by the STEP/IFC reader (has_color/cr/cg/cb/ca) —
    // emit it as IfcStyledItem -> IfcSurfaceStyle -> IfcColourRgb (was dropped; the viewer fell back
    // to grey). Matches adapy's add_colour + the Python streaming writer.
    if (root.has_color) {
        auto R = [](float v) { return adacpp::ifc_emit::ifc_real(v); };
        long col = em.emit_entity(buf, "IfcColourRgb($," + R(root.cr) + "," + R(root.cg) + "," + R(root.cb) + ")");
        long sh = em.emit_entity(buf, "IfcSurfaceStyleShading(#" + std::to_string(col) + "," + R(1.0f - root.ca) + ")");
        long ss = em.emit_entity(buf, "IfcSurfaceStyle($,.BOTH.,(#" + std::to_string(sh) + "))");
        em.emit_entity(buf, "IfcStyledItem(#" + std::to_string(solid) + ",(#" + std::to_string(ss) + "),$)");
    }
    std::string nm = root.id.empty() ? ("solid_" + std::to_string(sid)) : ifc_str(root.id);
    auto record = [&](long proxy, size_t k) {
        proxies.push_back(proxy);
        if (proxy_paths)
            proxy_paths->push_back(k < root.instance_paths.size() ? root.instance_paths[k] : IfcPath{});
    };
    // Proxy Name = the solid's own (leaf) product name; the assembly PATH is carried by proxy_paths
    // and emitted as a nested IfcElementAssembly tree by emit_spatial_tree (aligned with the STEP->GLB
    // product tree). Falls back to root.id when there's no path.
    auto leaf_name = [&](size_t k) -> std::string {
        if (k < root.instance_paths.size() && !root.instance_paths[k].empty()) {
            const std::string &ln = root.instance_paths[k].back().second;
            if (!ln.empty())
                return ifc_str(ln);
        }
        return nm;
    };
    long mrep =
        em.emit_entity(buf, "IfcShapeRepresentation(#6,'Body','" + rep_type + "',(#" + std::to_string(solid) + "))");
    if (root.transforms.empty()) {
        long pds = em.emit_entity(buf, "IfcProductDefinitionShape($,$,(#" + std::to_string(mrep) + "))");
        long proxy = em.emit_entity(buf, "IfcBuildingElementProxy('" + ifc_guid(guid_seed) + "',$,'" + leaf_name(0) +
                                             "',$,$,#11,#" + std::to_string(pds) + ",$,$)");
        record(proxy, 0);
        return;
    }
    long repmap = em.emit_entity(buf, "IfcRepresentationMap(#4,#" + std::to_string(mrep) + ")");
    int k = 0;
    for (const auto &T : root.transforms) {
        auto R = [](float v) { return ifc_real(v); };
        auto D = [&](float a, float b, float cc) {
            return em.emit_entity(buf, "IfcDirection((" + R(a) + "," + R(b) + "," + R(cc) + "))");
        };
        long axx = D(T[0], T[1], T[2]), axy = D(T[4], T[5], T[6]), axz = D(T[8], T[9], T[10]);
        long org = em.emit_entity(buf, "IfcCartesianPoint((" + R(T[12]) + "," + R(T[13]) + "," + R(T[14]) + "))");
        long op = em.emit_entity(buf, "IfcCartesianTransformationOperator3D(#" + std::to_string(axx) + ",#" +
                                          std::to_string(axy) + ",#" + std::to_string(org) + ",1.,#" +
                                          std::to_string(axz) + ")");
        long mi = em.emit_entity(buf, "IfcMappedItem(#" + std::to_string(repmap) + ",#" + std::to_string(op) + ")");
        long sr = em.emit_entity(buf, "IfcShapeRepresentation(#6,'Body','MappedRepresentation',(#" +
                                          std::to_string(mi) + "))");
        long pds = em.emit_entity(buf, "IfcProductDefinitionShape($,$,(#" + std::to_string(sr) + "))");
        long proxy = em.emit_entity(buf, "IfcBuildingElementProxy('" + ifc_guid(guid_seed + 1 + k) + "',$,'" +
                                             leaf_name(k) + "',$,$,#11,#" + std::to_string(pds) + ",$,$)");
        record(proxy, k);
        ++k;
    }
}

// Emit the nested IfcElementAssembly tree from per-proxy assembly paths, aligned with the STEP->GLB
// product tree (scene_from_step_stream._group_parent): assembly nodes keyed by the path's REP-ID
// prefix (names repeat across branches), named by product name. The proxy (leaf) is aggregated under
// its deepest assembly via IfcRelAggregates; assemblies nest via IfcRelAggregates; top-level elements
// (top assemblies + path-less proxies) are contained in the storey (#14). `next_id` allocates entity
// ids; appends SPF to `out`.
inline void emit_spatial_tree(std::string &out, const std::function<long()> &next_id, const std::vector<long> &proxies,
                              const std::vector<IfcPath> &paths) {
    using adacpp::ifc_emit::ifc_guid;
    using adacpp::ifc_emit::ifc_str;
    // Generic, domain-neutral spatial hierarchy: each assembly-path level is an IfcSpatialZone (not a
    // building-specific IfcBuildingStorey / element-grouping IfcElementAssembly). Zones nest under the
    // root zone (#12) via IfcRelAggregates; the leaf elements are contained in their deepest zone via
    // IfcRelContainedInSpatialStructure (IfcSpatialZone is a valid IfcSpatialElement RelatingStructure).
    const long ROOT = 12; // the header's root IfcSpatialZone (under IfcProject)
    std::map<std::vector<int>, long> zone_id;        // rep-id prefix -> IfcSpatialZone id
    std::map<long, std::vector<long>> agg_children;  // parent zone -> child zones (IfcRelAggregates)
    std::map<long, std::vector<long>> contained;     // zone -> contained leaf elements (IfcRelContained…)
    uint64_t aguid = 0xE0000000ull; // zone/rel GUID namespace (disjoint from header 0xF.. + proxies)

    for (size_t i = 0; i < proxies.size(); ++i) {
        const IfcPath &path = (i < paths.size()) ? paths[i] : IfcPath{};
        // Intermediate spatial levels = path[0 .. n-2]; path.back() is the solid's own (leaf) product.
        long parent_zone = ROOT;
        std::vector<int> prefix;
        for (size_t d = 0; d + 1 < path.size(); ++d) {
            prefix.push_back(path[d].first);
            auto it = zone_id.find(prefix);
            long zid;
            if (it == zone_id.end()) {
                zid = next_id();
                zone_id[prefix] = zid;
                std::string zn =
                    path[d].second.empty() ? ("zone_" + std::to_string(path[d].first)) : ifc_str(path[d].second);
                out += "#" + std::to_string(zid) + "=IFCSPATIALZONE('" + ifc_guid(aguid++) + "',$,'" + zn +
                       "',$,$,#11,$,$,.NOTDEFINED.);\n";
                agg_children[parent_zone].push_back(zid);
            } else {
                zid = it->second;
            }
            parent_zone = zid;
        }
        contained[parent_zone].push_back(proxies[i]);
    }
    for (const auto &[pid, kids] : agg_children) {
        std::string refs = "(";
        for (size_t j = 0; j < kids.size(); ++j)
            refs += (j ? ",#" : "#") + std::to_string(kids[j]);
        refs += ")";
        out += "#" + std::to_string(next_id()) + "=IFCRELAGGREGATES('" + ifc_guid(aguid++) + "',$,$,$,#" +
               std::to_string(pid) + "," + refs + ");\n";
    }
    for (const auto &[zid, kids] : contained) {
        std::string refs = "(";
        for (size_t j = 0; j < kids.size(); ++j)
            refs += (j ? ",#" : "#") + std::to_string(kids[j]);
        refs += ")";
        out += "#" + std::to_string(next_id()) + "=IFCRELCONTAINEDINSPATIALSTRUCTURE('" + ifc_guid(aguid++) +
               "',$,$,$," + refs + ",#" + std::to_string(zid) + ");\n";
    }
}

// Shared IFC header + spatial block (ids #1..#13). schema = "IFC4X3_ADD2" | "IFC4".
// The #7 length-unit line for the header. ng:: geometry is kept in the file's NATIVE units (the
// mesh/glb path scales by unit_scale at bake time; the IFC writer emits native coords), so declare
// the matching unit. unit_scale = metres per file-unit. SI prefixes cover m/mm/cm/dm/km/µm (≈ every
// real STEP file); a non-SI scale (e.g. inch 0.0254) keeps METRE (no regression — was always METRE)
// and is logged by the caller. Stays a SINGLE entity at #7 so the fixed #1..#13 header id layout holds.
inline std::string ifc_length_unit_line(double unit_scale) {
    auto close = [](double a, double b) { return std::abs(a - b) <= 1e-9 * std::max(1.0, std::abs(b)); };
    const char *prefix = nullptr; // null => plain METRE
    if (close(unit_scale, 1e-3))
        prefix = ".MILLI.";
    else if (close(unit_scale, 1e-2))
        prefix = ".CENTI.";
    else if (close(unit_scale, 1e-1))
        prefix = ".DECI.";
    else if (close(unit_scale, 1e3))
        prefix = ".KILO.";
    else if (close(unit_scale, 1e-6))
        prefix = ".MICRO.";
    std::string pf = prefix ? prefix : "$";
    return "#7=IFCSIUNIT(*,.LENGTHUNIT.," + pf + ",.METRE.);\n#8=IFCUNITASSIGNMENT((#7));\n";
}

inline std::string ifc_header_block(const std::string &schema, double unit_scale) {
    using adacpp::ifc_emit::ifc_guid;
    std::string b;
    b += "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\n";
    b += "FILE_NAME('','',(''),(''),'adacpp','','');\nFILE_SCHEMA(('" + schema + "'));\nENDSEC;\nDATA;\n";
    // Generic, domain-neutral spatial hierarchy: IfcProject -> a root IfcSpatialZone (#12), aggregated
    // by IfcRelAggregates (#13). Assembly-path levels become nested IfcSpatialZones under #12 and the
    // leaf elements are contained in their deepest zone (emit_spatial_tree). No building semantics.
    // Reserved header ids #1..#13 (K) — keep in sync with the parallel writer's K + renumber threshold.
    b += "#1=IFCCARTESIANPOINT((0.,0.,0.));\n#2=IFCDIRECTION((0.,0.,1.));\n#3=IFCDIRECTION((1.,0.,0.));\n"
         "#4=IFCAXIS2PLACEMENT3D(#1,#2,#3);\n#5=IFCDIRECTION((1.,0.));\n"
         "#6=IFCGEOMETRICREPRESENTATIONCONTEXT($,'Model',3,1.E-5,#4,#5);\n";
    b += ifc_length_unit_line(unit_scale);
    b += "#9=IFCPROJECT('" + ifc_guid(0xF0000001ull) + "',$,'adacpp model',$,$,$,$,(#6),#8);\n";
    b += "#10=IFCAXIS2PLACEMENT3D(#1,$,$);\n#11=IFCLOCALPLACEMENT($,#10);\n";
    b += "#12=IFCSPATIALZONE('" + ifc_guid(0xF0000002ull) + "',$,'Model',$,$,#11,$,$,.NOTDEFINED.);\n";
    b += "#13=IFCRELAGGREGATES('" + ifc_guid(0xF0000005ull) + "',$,$,$,#9,(#12));\n";
    return b;
}

// Phase 2 STEP->IFC file writer: drives the native reader (StreamIndex/Resolver) + the dep-free
// ifc_emit::BrepEmitter. Lives here (not in ifc_emit.h) so the emitter header stays reader-free.
// Streams to disk per chunk; bounds parse_cache_ (single-threaded -> safe per fc37d71).
inline adacpp::ifc_emit::FileStats write_ifc_file_impl(const std::string &in_path, const std::string &out_path,
                                                       const std::string &schema, double deflection, double angular_deg,
                                                       long max_solids) {
    using namespace adacpp::ifc_emit;
    FileStats fs;
    adacpp::prof::StepProfiler prof("stream_step_to_ifc(st)");
    auto idx = adacpp::step::StreamIndex::from_file(in_path);
    prof.phase("scan_index");
    adacpp::step::Resolver r(idx);
    r.build_metadata(idx.lists);
    r.enable_parse_cache_bounding();
    fs.unit_scale = r.unit_scale();
    prof.phase("metadata");
    std::FILE *fp = std::fopen(out_path.c_str(), "wb");
    if (!fp)
        return fs;
    std::string buf;
    buf.reserve(1 << 22);
    auto flush = [&](bool force) {
        if (buf.size() >= (4u << 20) || force) {
            std::fwrite(buf.data(), 1, buf.size(), fp);
            buf.clear();
        }
    };
    buf += ifc_header_block(schema, r.unit_scale());
    BrepEmitter em(100, nullptr, deflection, angular_deg);
    std::vector<long> proxies;
    std::vector<IfcPath> proxy_paths;
    for (long sid : idx.lists.roots) {
        if (max_solids > 0 && fs.solids_in >= max_solids)
            break;
        adacpp::ngeom::NgeomRoot root = r.resolve_root(sid);
        ++fs.solids_in;
        prof.solid(root.faces.size());
        size_t before = proxies.size();
        emit_solid_ifc(em, buf, root, sid, (uint64_t) fs.solids_in * 1000u, proxies, &proxy_paths);
        if (proxies.size() > before)
            ++fs.solids_out;
        r.clear_geom_cache();
        flush(false);
    }
    prof.phase("resolve+emit");
    emit_spatial_tree(buf, [&]() { return em.alloc_id(); }, proxies, proxy_paths);
    buf += "ENDSEC;\nEND-ISO-10303-21;\n";
    flush(true);
    std::fclose(fp);
    prof.phase("spatial_tree+write");
    fs.geom = em.stats();
    return fs;
}

// AP242 STEP header + shared context block (ids #1..#13). unit_scale -> SI length prefix (matches the
// IFC writer's mapping). K=13 reserved.
inline std::string step_header_block(double unit_scale) {
    auto close = [](double a, double b) { return std::abs(a - b) <= 1e-9 * std::max(1.0, std::abs(b)); };
    const char *prefix = "$"; // plain METRE
    if (close(unit_scale, 1e-3))
        prefix = ".MILLI.";
    else if (close(unit_scale, 1e-2))
        prefix = ".CENTI.";
    else if (close(unit_scale, 1e-1))
        prefix = ".DECI.";
    else if (close(unit_scale, 1e3))
        prefix = ".KILO.";
    else if (close(unit_scale, 1e-6))
        prefix = ".MICRO.";
    std::string b;
    b += "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('adacpp native STEP->STEP'),'2;1');\n";
    b += "FILE_NAME('','',(''),(''),'adacpp','','');\n";
    b += "FILE_SCHEMA(('AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF { 1 0 10303 442 1 1 4 }'));\n";
    b += "ENDSEC;\nDATA;\n";
    b += "#1=APPLICATION_CONTEXT('managed model based 3d engineering');\n";
    b += "#2=APPLICATION_PROTOCOL_DEFINITION('international standard',"
         "'ap242_managed_model_based_3d_engineering_mim_lf',2014,#1);\n";
    b += "#3=PRODUCT_CONTEXT('',#1,'mechanical');\n#4=PRODUCT_DEFINITION_CONTEXT('part definition',#1,'design');\n";
    b += "#5=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(" + std::string(prefix) + ",.METRE.));\n";
    b += "#6=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));\n";
    b += "#7=(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT());\n";
    b += "#8=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-6),#5,'distance_accuracy_value','edge/vertex');\n";
    b += "#9=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#8))"
         "GLOBAL_UNIT_ASSIGNED_CONTEXT((#5,#6,#7))REPRESENTATION_CONTEXT('Context','3D'));\n";
    b += "#10=CARTESIAN_POINT('',(0.,0.,0.));\n#11=DIRECTION('',(0.,0.,1.));\n#12=DIRECTION('',(1.,0.,0.));\n";
    b += "#13=AXIS2_PLACEMENT_3D('',#10,#11,#12);\n";
    return b;
}

// Emit one solid instance (geometry baked by `em`'s transform) as a self-contained AP242 part:
// MANIFOLD_SOLID_BREP / EXTRUDED_AREA_SOLID -> (ADVANCED_BREP_)SHAPE_REPRESENTATION (placement #13,
// context #9) -> PRODUCT chain -> SHAPE_DEFINITION_REPRESENTATION. Returns true if a solid was emitted.
// Returns the leaf PRODUCT_DEFINITION id (0 on failure) so the caller can hang a NEXT_ASSEMBLY_USAGE_
// OCCURRENCE assembly tree off it; treat nonzero as success.
inline long emit_solid_step(adacpp::step_emit::StepBrepEmitter &em, std::string &buf,
                            const adacpp::ngeom::NgeomRoot &root, long sid) {
    std::string nm = root.id.empty() ? ("solid_" + std::to_string(sid)) : adacpp::ifc_emit::ifc_str(root.id);
    long solid = 0;
    const char *rep_kw = "ADVANCED_BREP_SHAPE_REPRESENTATION";
    if (root.extrusion) {
        bool hollow = root.extrusion->profile && root.extrusion->profile->bounds.size() > 1;
        if (em.tf_rigid() && !hollow) { // rigid, solid profile -> native EXTRUDED_AREA_SOLID
            solid = em.emit_extrusion(buf, *root.extrusion);
            rep_kw = "SHAPE_REPRESENTATION";
        } else { // scale/shear or hollow profile -> bake to a B-rep (annular caps for voids)
            solid = em.emit_extrusion_baked(buf, *root.extrusion, nm);
        }
    } else if (root.revolve) { // non-rigid revolves are dropped to OCC by the reader -> rigid here
        solid = em.emit_revolve(buf, *root.revolve);
        rep_kw = "SHAPE_REPRESENTATION";
    } else if (root.sweep) { // disk/annulus swept along a directrix -> SWEPT_DISK_SOLID
        solid = em.emit_swept_disk(buf, *root.sweep);
        rep_kw = "SHAPE_REPRESENTATION";
    } else if (root.sphere) { // CSG sphere primitive
        solid = em.emit_sphere(buf, *root.sphere);
        rep_kw = "SHAPE_REPRESENTATION";
    } else if (root.boolean) { // CSG tree (BOOLEAN_RESULT), preserved not evaluated
        solid = em.emit_boolean(buf, *root.boolean);
        rep_kw = "SHAPE_REPRESENTATION";
    } else {
        solid = em.emit_manifold_brep(buf, root, nm);
    }
    if (!solid)
        return 0;
    long rep = em.emit_entity(buf, std::string(rep_kw) + "('" + nm + "',(#13,#" + std::to_string(solid) + "),#9)");
    long product = em.emit_entity(buf, "PRODUCT('" + nm + "','" + nm + "','',(#3))");
    long pdf = em.emit_entity(buf, "PRODUCT_DEFINITION_FORMATION('','',#" + std::to_string(product) + ")");
    long pd = em.emit_entity(buf, "PRODUCT_DEFINITION('design','',#" + std::to_string(pdf) + ",#4)");
    long pds = em.emit_entity(buf, "PRODUCT_DEFINITION_SHAPE('','',#" + std::to_string(pd) + ")");
    em.emit_entity(buf, "SHAPE_DEFINITION_REPRESENTATION(#" + std::to_string(pds) + ",#" + std::to_string(rep) + ")");
    return pd;
}

// Emit a NEXT_ASSEMBLY_USAGE_OCCURRENCE assembly tree from per-leaf (product_definition, path): each
// intermediate path level becomes an assembly PRODUCT_DEFINITION, linked to its children (assemblies or
// leaf solids) by a NAUO — the STEP counterpart of emit_spatial_tree, so the IFC/STEP hierarchy round-
// trips. Assembly nodes carry no own shape (grouping only); placements stay baked into the leaf geometry.
using IfcPath2 = std::vector<std::pair<int, std::string>>;
inline void emit_step_assembly_tree(adacpp::step_emit::StepBrepEmitter &em, std::string &buf,
                                    const std::vector<long> &leaf_pds, const std::vector<IfcPath2> &paths) {
    using adacpp::ifc_emit::ifc_str;
    std::map<std::vector<int>, long> asm_pd; // rep-id prefix -> assembly PRODUCT_DEFINITION id
    auto asm_node = [&](const std::vector<int> &prefix, const std::string &name) -> long {
        auto it = asm_pd.find(prefix);
        if (it != asm_pd.end())
            return it->second;
        std::string nm = name.empty() ? ("asm_" + std::to_string(prefix.back())) : ifc_str(name);
        long product = em.emit_entity(buf, "PRODUCT('" + nm + "','" + nm + "','',(#3))");
        long pdf = em.emit_entity(buf, "PRODUCT_DEFINITION_FORMATION('','',#" + std::to_string(product) + ")");
        long pd = em.emit_entity(buf, "PRODUCT_DEFINITION('design','',#" + std::to_string(pdf) + ",#4)");
        asm_pd[prefix] = pd;
        return pd;
    };
    auto nauo = [&](long parent_pd, long child_pd, const std::string &nm) {
        em.emit_entity(buf, "NEXT_ASSEMBLY_USAGE_OCCURRENCE('','" + nm + "','',#" + std::to_string(parent_pd) + ",#" +
                                std::to_string(child_pd) + ",$)");
    };
    for (size_t i = 0; i < leaf_pds.size(); ++i) {
        if (!leaf_pds[i])
            continue;
        const IfcPath2 &path = (i < paths.size()) ? paths[i] : IfcPath2{};
        long parent_pd = 0;
        std::vector<int> prefix;
        for (size_t d = 0; d + 1 < path.size(); ++d) {
            prefix.push_back(path[d].first);
            long apd = asm_node(prefix, path[d].second);
            if (parent_pd)
                nauo(parent_pd, apd, path[d].second);
            parent_pd = apd;
        }
        if (parent_pd) // hang the leaf solid under its deepest assembly (top-level leaves stay roots)
            nauo(parent_pd, leaf_pds[i], path.empty() ? "" : path.back().second);
    }
}

// Native IFC->STEP (AP242): IfcResolver reads each product's analytic B-rep -> ng::, then the STEP
// emitter re-writes it (instances baked). Serial (per-product resolve is cheap; the shared
// IfcRepresentationMap brep is re-resolved per instance = baked). Round-trip with the STEP->IFC writer.
inline adacpp::ifc_emit::FileStats write_ifc_to_step_impl(const std::string &in_path, const std::string &out_path,
                                                          double deflection, double angular_deg, long max_solids) {
    using adacpp::ngeom::NgeomRoot;
    using adacpp::step_emit::StepBrepEmitter;
    adacpp::ifc_emit::FileStats fs;
    adacpp::prof::StepProfiler prof("stream_ifc_to_step");
    auto idx = adacpp::step::StreamIndex::from_file(in_path);
    prof.phase("scan_index");
    if (!idx.ok())
        return fs;
    adacpp::ifc_read::IfcResolver r(idx);
    fs.unit_scale = r.unit_scale();
    std::vector<long> roots = r.proxy_roots();
    if (max_solids > 0 && (long) roots.size() > max_solids)
        roots.resize(max_solids);
    fs.products_total = (long) roots.size();
    prof.phase("metadata");
    std::FILE *fp = std::fopen(out_path.c_str(), "wb");
    if (!fp)
        return fs;
    std::string buf;
    buf.reserve(1 << 22);
    std::string hdr = step_header_block(fs.unit_scale);
    std::fwrite(hdr.data(), 1, hdr.size(), fp);
    long nid = 13; // continue after the shared header block
    auto flush = [&](bool force) {
        if (buf.size() >= (4u << 20) || force) {
            std::fwrite(buf.data(), 1, buf.size(), fp);
            buf.clear();
        }
    };
    std::vector<long> leaf_pds;     // per emitted solid: its PRODUCT_DEFINITION id (for the NAUO tree)
    std::vector<IfcPath2> leaf_paths; // parallel: the solid's assembly path
    for (long pid : roots) {
        NgeomRoot root = r.resolve_product(pid);
        if (root.faces.empty() && !root.extrusion && !root.revolve && !root.boolean && !root.sweep && !root.sphere) {
            ++fs.products_skipped; // a product whose geometry the analytic reader couldn't represent
            continue;
        }
        ++fs.solids_in;
        prof.solid(root.faces.size());
        size_t ninst = root.transforms.empty() ? 1 : root.transforms.size();
        bool any = false;
        for (size_t k = 0; k < ninst; ++k) {
            double tf[16];
            const double *tfp = nullptr;
            if (!root.transforms.empty()) {
                const std::array<float, 16> &M = root.transforms[k];
                tf[0] = M[0];
                tf[1] = M[4];
                tf[2] = M[8];
                tf[3] = M[12];
                tf[4] = M[1];
                tf[5] = M[5];
                tf[6] = M[9];
                tf[7] = M[13];
                tf[8] = M[2];
                tf[9] = M[6];
                tf[10] = M[10];
                tf[11] = M[14];
                tfp = tf;
            }
            StepBrepEmitter em(nid, tfp, deflection, angular_deg);
            long pd = emit_solid_step(em, buf, root, pid);
            if (pd) {
                nid = em.current_id();
                any = true;
                leaf_pds.push_back(pd);
                leaf_paths.push_back(k < root.instance_paths.size() ? root.instance_paths[k] : IfcPath2{});
                const auto &s = em.stats();
                fs.geom.faces_in += s.faces_in;
                fs.geom.faces_out += s.faces_out;
                fs.geom.faces_dropped += s.faces_dropped;
                fs.geom.edges_analytic += s.edges_analytic;
                fs.geom.edges_polyline_approx += s.edges_polyline_approx;
                fs.geom.edges_degenerate += s.edges_degenerate;
                for (const auto &[rk, rv] : s.drop_reasons)
                    fs.geom.drop_reasons[rk] += rv;
            }
        }
        if (any)
            ++fs.solids_out;
        flush(false);
    }
    prof.phase("resolve+emit");
    // NAUO assembly tree over all emitted leaves (STEP counterpart of the IFC spatial tree).
    StepBrepEmitter emtree(nid, nullptr, deflection, angular_deg);
    emit_step_assembly_tree(emtree, buf, leaf_pds, leaf_paths);
    const char *foot = "ENDSEC;\nEND-ISO-10303-21;\n";
    buf += foot;
    flush(true);
    std::fclose(fp);
    prof.phase("write_tail");
    return fs;
}

} // namespace adacpp::brep_convert
