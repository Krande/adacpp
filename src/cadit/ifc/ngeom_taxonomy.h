// NGEOM -> IfcOpenShell taxonomy adapter (Part 2). Maps the neutral geometry hub onto
// ifcopenshell::geometry::taxonomy items so AbstractKernel::convert (OCC / CGAL / hybrid)
// can run IfcOpenShell's full conversion+healing pipeline on adapy geometry.
//
// This path DOES use IfcOpenShell (and hence OCC/CGAL) — that's its purpose. It is a
// separate pipeline from the OCC-free libtess2 path; both consume the same neutral layer.
#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "../../geom/neutral/ngeom_tessellate.h"  // TessMesh
#include "../../geom/neutral/ngeom_topology.h"

// Forward-declare the taxonomy shell so callers don't need the heavy ifcopenshell headers.
namespace ifcopenshell {
namespace geometry {
namespace taxonomy {
struct shell;
struct extrusion;
}
}  // namespace geometry
}  // namespace ifcopenshell

namespace adacpp::ngeom {

// Build a taxonomy shell (faces -> loops -> edges + basis surfaces/curves) from neutral
// faces. Returns nullptr if nothing mappable. Defined in ngeom_taxonomy_build.cpp.
std::shared_ptr<ifcopenshell::geometry::taxonomy::shell> to_taxonomy_shell(
    const std::vector<std::shared_ptr<FaceSurfaceN>> &faces);

// Build an ifcopenshell taxonomy::extrusion (profile face + placement matrix +
// direction + depth) from a decoded ExtrusionN. Defined in ngeom_taxonomy_build.cpp.
std::shared_ptr<ifcopenshell::geometry::taxonomy::extrusion> to_taxonomy_extrusion(const ExtrusionN &ex);

// Descriptor for one ifcopenshell ConversionSettings option, exposed so
// Python / the frontend can enumerate + tune the taxonomy kernel settings.
struct TaxonomySetting {
    std::string name;           // e.g. "no-wire-intersection-check", "precision"
    std::string type;           // "bool" | "int" | "double" | "std::string" | ...
    std::string default_value;  // stringified default (empty for non-scalar types)
};

// Enumerate every ifcopenshell ConversionSettings option with its type +
// default. Defined in ngeom_taxonomy_kernel.cpp.
std::vector<TaxonomySetting> taxonomy_settings_info();

// Tessellate a decoded document through the taxonomy + kernel pipeline.
//   kernel:   "occ" | "cgal" | "hybrid"
//   settings: (name, string-value) overrides applied to the kernel's
//             ifcopenshell ConversionSettings (parsed per the setting's type).
// Returns one TessMesh with a Group per root. Defined in ngeom_taxonomy_kernel.cpp.
TessMesh tessellate_via_taxonomy(const NgeomDoc &doc, const std::string &kernel, double deflection,
                                 double angular_deg,
                                 const std::vector<std::pair<std::string, std::string>> &settings = {});

}  // namespace adacpp::ngeom
