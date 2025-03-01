#include "ifcop.h"
#include "ifcparse/Ifc4x1.h"
#include "ifcparse/IfcFile.h"
#define IfcSchema Ifc4x1


using namespace std::string_literals;


int read_ifc_file(const std::string &file_name) {
    IfcParse::IfcFile file(file_name);
    if (!file.good()) {
        std::cout << "Unable to parse .ifc file" << std::endl;
        return 1;
    }

//    std::cout << "file name: " << file.header().file_name().name() << std::endl;

    IfcSchema::IfcBeam::list::ptr elements = file.instances_by_type<IfcSchema::IfcBeam>();

    std::cout << "Found " << elements->size() << " elements in " << file_name << ":" << std::endl;

    std::ostringstream oss;
    for (const auto element : *elements) {
        element->data().toString(oss);
        oss << "\n";
        std::cout << oss.str();
        oss.str("");  // Clear the contents of the stringstream
        oss.clear();  // Reset any error flags
    }

    return 0;
}
