//
// Created by ofskrand on 23.01.2024.
//
#include "step_to_glb.h"

#include <STEPControl_Reader.hxx>
#include <BRep_Tool.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Face.hxx>

#include <vector>
#include <string>
#include "tiny_gltf.h"
#include <TopoDS.hxx>

// Function to process a single shape and add it to the GLB model
void ProcessShape(const TopoDS_Shape&shape, tinygltf::Model&gltfModel) {
    BRepMesh_IncrementalMesh mesh(shape, 0.1);

    // For each shape, a new mesh will be created
    tinygltf::Mesh gltfMesh;
    tinygltf::Primitive primitive;

    std::vector<float> vertices;
    std::vector<unsigned int> indices;

    TopExp_Explorer explorer(shape, TopAbs_FACE);
    for (; explorer.More(); explorer.Next()) {
        TopoDS_Face face = TopoDS::Face(explorer.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, loc);

        if (!triangulation.IsNull()) {
            const TColgp_Array1OfPnt&nodes = triangulation->MapNodeArray()->Array1();
            for (int i = 1; i <= nodes.Length(); i++) {
                gp_Pnt pnt = nodes(i).Transformed(loc.Transformation());
                vertices.push_back(static_cast<float>(pnt.X()));
                vertices.push_back(static_cast<float>(pnt.Y()));
                vertices.push_back(static_cast<float>(pnt.Z()));
            }

            const Poly_Array1OfTriangle&triangles = triangulation->Triangles();
            for (int i = 1; i <= triangulation->NbTriangles(); i++) {
                Standard_Integer n1, n2, n3;
                triangles(i).Get(n1, n2, n3);
                indices.push_back(n1 - 1);
                indices.push_back(n2 - 1);
                indices.push_back(n3 - 1);
            }
        }
    }

    // Here you should add the logic to create buffer views and accessors for vertices and indices
    // ...

    // Setup the primitive
    // ...

    gltfMesh.primitives.push_back(primitive);
    gltfModel.meshes.push_back(gltfMesh);
}

void stp_to_glb(const std::string&stp_file, const std::string&glb_file) {
    STEPControl_Reader reader;
    IFSelect_ReturnStatus status = reader.ReadFile(stp_file.c_str());

    if (status == IFSelect_RetDone) {
        tinygltf::Model gltfModel;

        Standard_Integer nbRoots = reader.NbRootsForTransfer();
        for (Standard_Integer n = 1; n <= nbRoots; n++) {
            reader.TransferRoot(n);
            int nbShapes = reader.NbShapes();
            for (int i = 1; i <= nbShapes; i++) {
                TopoDS_Shape shape = reader.Shape(i);
                ProcessShape(shape, gltfModel);
            }
        }

        tinygltf::TinyGLTF gltfContext;
        gltfContext.WriteGltfSceneToFile(&gltfModel, glb_file, true, true, true, true);
    }
    else {
        throw std::runtime_error("Error reading STEP file");
    }
}