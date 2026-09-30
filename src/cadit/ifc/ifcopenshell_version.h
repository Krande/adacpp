// Which ifcopenshell C++ API this build compiles against.
//
// adacpp supports both the 0.8 line and 0.9 while 0.9 is new. 0.9 renamed its C++ API to
// snake_case under ifcopenshell::geom (IfcGeom::OpenCascadeKernel -> open_cascade_kernel,
// ifcopenshell::geometry::Settings -> ifcopenshell::geom::settings, IfcParse::IfcFile ->
// ifcopenshell::file, ...), renamed the headers to match, and moved settings values from
// boost::variant to std::variant. Neither release defines a version macro, so key on the header
// layout: ifcparse/file.h only exists from 0.9 (0.8 ships ifcparse/IfcFile.h).
//
// ADACPP_IFCOPENSHELL_09 is 1 for 0.9+, 0 for 0.8. Define it on the command line to override.
//
// Including this header is cheap: it pulls no ifcopenshell headers unless
// ADACPP_IFCOPENSHELL_KERNELS is defined first, in which case it also includes the settings +
// OCC/CGAL kernel headers and provides the version-neutral aliases in adacpp::ifc.
#pragma once

#ifndef ADACPP_IFCOPENSHELL_09
#if __has_include(<ifcparse/file.h>)
#define ADACPP_IFCOPENSHELL_09 1
#elif __has_include(<ifcparse/IfcFile.h>)
#define ADACPP_IFCOPENSHELL_09 0
#else
// No ifcopenshell headers at all: the wasm build, which compiles src/cad/cad_py_wrap.cpp (and
// so ngeom_taxonomy.h's declarations) but none of the taxonomy sources. Only the forward
// declarations below are used there, so either value works; ADACPP_IFCOPENSHELL_KERNELS
// would still fail on the missing kernel headers, which is the right error.
#define ADACPP_IFCOPENSHELL_09 1
#endif
#endif

// Forward declarations of the taxonomy items adacpp builds, in whichever namespace this
// ifcopenshell uses, plus a stable alias for signatures: adacpp::ifc::taxonomy.
#if ADACPP_IFCOPENSHELL_09
namespace ifcopenshell::geom::taxonomy {
struct shell;
struct extrusion;
} // namespace ifcopenshell::geom::taxonomy
namespace adacpp::ifc {
namespace taxonomy = ::ifcopenshell::geom::taxonomy;
} // namespace adacpp::ifc
#else
namespace ifcopenshell::geometry::taxonomy {
struct shell;
struct extrusion;
} // namespace ifcopenshell::geometry::taxonomy
namespace adacpp::ifc {
namespace taxonomy = ::ifcopenshell::geometry::taxonomy;
} // namespace adacpp::ifc
#endif

#ifdef ADACPP_IFCOPENSHELL_KERNELS
#include <ifcgeom/taxonomy.h>

#include <string>
#if ADACPP_IFCOPENSHELL_09
#include <ifcgeom/conversion_settings.h>
#include <ifcgeom/kernels/cgal/cgal_kernel.h>
#include <ifcgeom/kernels/opencascade/opencascade_kernel.h>

#include <variant>
#else
#include <ifcgeom/ConversionSettings.h>
#include <ifcgeom/kernels/cgal/CgalKernel.h>
#include <ifcgeom/kernels/opencascade/OpenCascadeKernel.h>

#include <boost/variant/get.hpp>
#endif

namespace adacpp::ifc {
#if ADACPP_IFCOPENSHELL_09
using settings = ::ifcopenshell::geom::settings;
using occ_kernel = ::ifcopenshell::geom::open_cascade_kernel;
using cgal_kernel = ::ifcopenshell::geom::kernels::cgal_kernel;
using cgal_shape = ::cgal_polyhedron;
using NoWireIntersectionCheck = settings::NoWireIntersectionCheck;
#else
using settings = ::ifcopenshell::geometry::Settings;
using occ_kernel = ::IfcGeom::OpenCascadeKernel;
using cgal_kernel = ::ifcopenshell::geometry::kernels::CgalKernel;
using cgal_shape = ::cgal_shape_t;
// 0.8 keeps the setting types in an inline namespace beside the Settings class, not in it.
using NoWireIntersectionCheck = ::ifcopenshell::geometry::settings::NoWireIntersectionCheck;
#endif

// Pointer to the T held by a settings value, or nullptr (std::get_if / boost::get semantics).
template <typename T> const T *settings_value_if(const settings::value_variant_t &v) {
#if ADACPP_IFCOPENSHELL_09
    return std::get_if<T>(&v);
#else
    return boost::get<T>(&v);
#endif
}
} // namespace adacpp::ifc
#endif // ADACPP_IFCOPENSHELL_KERNELS
