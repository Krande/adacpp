#include "ifcop.h"
#include "ifcopenshell_version.h"

#if ADACPP_IFCOPENSHELL_09
#include "ifcparse/file.h"
#else
#include "ifcparse/IfcFile.h"
#endif

#include <iostream>

int read_ifc_file(const std::string &file_name) {
#if ADACPP_IFCOPENSHELL_09
    ifcopenshell::file file(file_name);
#else
    IfcParse::IfcFile file(file_name);
#endif
    if (!file.good()) {
        std::cout << "Unable to parse .ifc file" << std::endl;
        return 1;
    }

    // Looked up by name rather than through a schema's IfcBeam class, so any schema the file
    // declares works (ifcopenshell >= 0.9 loads each schema as its own plugin library).
    const auto elements = file.instances_by_type("IfcBeam");
#if ADACPP_IFCOPENSHELL_09
    const size_t n = elements.size();
#else
    const size_t n = elements ? elements->size() : 0;
#endif

    std::cout << "Found " << n << " elements in " << file_name << ":" << std::endl;

    // Print from plain accessors rather than the instance's to_string(std::ostream&). In 0.9 that
    // call imbues and restores the stream's std::locale inside libifcopenshell.parse; on macOS a
    // conda process also has the system libc++ loaded, and a locale created on one runtime and
    // destroyed on the other aborts with "pointer being freed was not allocated".
#if ADACPP_IFCOPENSHELL_09
    for (const auto &element : elements) {
        std::cout << "#" << element.id() << "=" << element.declaration().name() << "\n";
    }
#else
    if (elements) {
        for (const auto *element : *elements) {
            std::cout << "#" << element->id() << "=" << element->declaration().name() << "\n";
        }
    }
#endif
    std::cout << std::flush;

    return 0;
}
