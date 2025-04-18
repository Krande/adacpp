#include <optional>
#include <utility>
#include <vector>
#include <TopoDS_Shape.hxx>
#include <stdexcept>
#include "OccShape.h"
#include "MeshType.h"
#include "Mesh.h"
#include "GroupReference.h"
#include "Color.h"

MeshType from_int(int value) {
    if (value < 0 || value > 6) {
        throw std::out_of_range("Invalid MeshType value");
    }
    return static_cast<MeshType>(value);
}

Mesh::Mesh(const int id, std::vector<float> positions, std::vector<uint32_t> faces,
           std::vector<uint32_t> edges, std::vector<float> normals,
           const MeshType mesh_type, Color color, std::vector<GroupReference> group_reference)
        : id(id),
          positions(std::move(positions)),
          indices(std::move(faces)),
          edges(std::move(edges)),
          normals(std::move(normals)),
          mesh_type(mesh_type),
          color(std::move(color)),
          group_reference(std::move(group_reference)) {}

OccShape::OccShape(TopoDS_Shape shape,
                   const Color color,
                   const int num_tot_entities,
                   std::optional<std::string> name)
        : shape(std::move(shape)),
          color(color),
          num_tot_entities(num_tot_entities),
          name(std::move(name)) {}

GroupReference::GroupReference(const int node_id, const int start, const int length)
        : node_id(node_id), start(start), length(length) {}