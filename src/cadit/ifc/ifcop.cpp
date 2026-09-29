#include "ifcop.h"
#include "ifcparse/file.h"

#include <iostream>
#include <sstream>

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

    std::ostringstream oss;
    for (const auto &element : elements) {
        element.to_string(oss);
        oss << "\n";
        std::cout << oss.str();
        oss.str(""); // Clear the contents of the stringstream
        oss.clear(); // Reset any error flags
    }

    return 0;
}
