// NGEOM -> IfcOpenShell taxonomy adapter (Part 2). Maps the neutral geometry hub onto
// ifcopenshell::geometry::taxonomy items so AbstractKernel::convert (OCC / CGAL / hybrid)
// can run IfcOpenShell's full conversion+healing pipeline on adapy geometry.
//
// This path DOES use IfcOpenShell (and hence OCC/CGAL) — that's its purpose. It is a
// separate pipeline from the OCC-free libtess2 path; both consume the same neutral layer.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "../../geom/neutral/ngeom_tessellate.h"  // TessMesh
#include "../../geom/neutral/ngeom_topology.h"

// Forward-declare the taxonomy shell so callers don't need the heavy ifcopenshell headers.
namespace ifcopenshell {
namespace geometry {
namespace taxonomy {
struct shell;
}
}  // namespace geometry
}  // namespace ifcopenshell

namespace adacpp::ngeom {

// Build a taxonomy shell (faces -> loops -> edges + basis surfaces/curves) from neutral
// faces. Returns nullptr if nothing mappable. Defined in ngeom_taxonomy_build.cpp.
std::shared_ptr<ifcopenshell::geometry::taxonomy::shell> to_taxonomy_shell(
    const std::vector<std::shared_ptr<FaceSurfaceN>> &faces);

// Tessellate a decoded document through the taxonomy + kernel pipeline.
//   kernel: "occ" | "cgal" | "hybrid"
// Returns one TessMesh with a Group per root. Defined in ngeom_taxonomy_kernel.cpp.
TessMesh tessellate_via_taxonomy(const NgeomDoc &doc, const std::string &kernel, double deflection,
                                 double angular_deg);

}  // namespace adacpp::ngeom
