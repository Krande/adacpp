#include "ifcop.h"
#include "ifcparse/file.h"

#include <iostream>

int read_ifc_file(const std::string &file_name) {
    ifcopenshell::file file(file_name);
    if (!file.good()) {
        std::cout << "Unable to parse .ifc file" << std::endl;
        return 1;
    }

    // Looked up by name rather than through a schema's IfcBeam class, so any schema the file
    // declares works (ifcopenshell >= 0.9 loads each schema as its own plugin library).
    const auto elements = file.instances_by_type("IfcBeam");

    std::cout << "Found " << elements.size() << " elements in " << file_name << ":" << std::endl;

    // Print from plain accessors rather than express::base::to_string(std::ostream&). That call
    // imbues and restores the stream's std::locale inside libifcopenshell.parse; on macOS a
    // conda process also has the system libc++ loaded, and a locale created on one runtime and
    // destroyed on the other aborts with "pointer being freed was not allocated".
    for (const auto &element : elements) {
        std::cout << "#" << element.id() << "=" << element.declaration().name() << "\n";
    }
    std::cout << std::flush;

    return 0;
}
