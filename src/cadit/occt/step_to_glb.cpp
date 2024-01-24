#include <STEPCAFControl_Reader.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <RWGltf_CafWriter.hxx>
#include <TDocStd_Document.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <XCAFDoc_ShapeTool.hxx>


void stp_to_glb(const std::string& stp_file,
                const std::string& glb_file,
                double linearDeflection = 0.1,
                double angularDeflection = 0.5,
                bool relativeDeflection = false)
{
    // Initialize the STEPCAFControl_Reader
    STEPCAFControl_Reader reader;
    reader.SetColorMode(true);
    reader.SetNameMode(true);
    reader.SetLayerMode(true);
    reader.SetPropsMode(true);
    reader.SetGDTMode(true);
    reader.SetMatMode(true);
    reader.SetViewMode(true);

    // Read the STEP file
    // print to console
    std::cout << "Reading STEP file: " << stp_file << std::endl;
    if (reader.ReadFile(stp_file.c_str()) != IFSelect_RetDone)
        throw std::runtime_error("Error reading STEP file");

    // Transfer to a document
    Handle(TDocStd_Document) doc = new TDocStd_Document("MDTV-XCAF");
    if (!reader.Transfer(doc))
        throw std::runtime_error("Error transferring data to document");

    // Tessellation
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    TDF_LabelSequence labelSeq;
    shapeTool->GetShapes(labelSeq);
    for (Standard_Integer i = 1; i <= labelSeq.Length(); ++i)
    {
        TopoDS_Shape shape = shapeTool->GetShape(labelSeq.Value(i));
        BRepMesh_IncrementalMesh(shape, linearDeflection, relativeDeflection, angularDeflection, true);
    }

    // Write to GLB
    RWGltf_CafWriter writer(glb_file.c_str(), true); // true for binary format

    // Additional file information (can be empty if not needed)
    TColStd_IndexedDataMapOfStringString file_info;

    // Progress indicator (can be null if progress tracking is not needed)
    Message_ProgressRange progress;

    if (!writer.Perform(doc, file_info, progress))
    {
        throw std::runtime_error("Error writing GLB file");
    }
}

