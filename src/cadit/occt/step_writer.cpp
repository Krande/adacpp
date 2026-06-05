#include <iostream>
#include <memory>
#include <filesystem>

#include "static_param_guard.h"

#include <BRep_Builder.hxx>
#include <Interface_Static.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDocStd_Application.hxx>
#include <TDocStd_Document.hxx>
#include <TopoDS_Compound.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XSControl_WorkSession.hxx>
#include <TDF_Label.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include "../../helpers/helpers.h"
#include "step_writer.h"


class AdaCPPStepWriter {
public:
    explicit AdaCPPStepWriter(const std::string &top_level_name = "Assembly") {
        app_ = new TDocStd_Application();
        doc_ = new TDocStd_Document(TCollection_ExtendedString("XmlOcaf"));
        app_->InitDocument(doc_);

        // The shape tool
        shape_tool_ = XCAFDoc_DocumentTool::ShapeTool(doc_->Main());
        // Set auto naming to false
        shape_tool_->SetAutoNaming(false);

        // The color tool
        color_tool_ = XCAFDoc_DocumentTool::ColorTool(doc_->Main());

        // Set up the compound
        comp_builder_.MakeCompound(comp_);

        tll_ = shape_tool_->AddShape(comp_, Standard_True);
        set_name(tll_, top_level_name);
    }

    void add_shape(const TopoDS_Shape &shape, const std::string &name,
                   const Color &rgb_color,
                   const TDF_Label &parent = TDF_Label()) {
        comp_builder_.Add(comp_, shape);
        TDF_Label parent_label = parent.IsNull() ? tll_ : parent;
        TDF_Label shape_label = shape_tool_->AddSubShape(parent_label, shape);
        if (shape_label.IsNull()) {
            shape_label = shape_tool_->AddShape(shape, Standard_False, Standard_False);
//            std::cout << "Adding as SubShape label generated an IsNull label. Adding as shape instead" << std::endl;
        }
        set_color(shape_label, rgb_color, color_tool_);
        set_name(shape_label, name);
    }

    void add_shape_handle(const TopoDS_Shape &shape, const std::string &name, const Color &rgb_color) {
        add_shape(shape, name, rgb_color);
    }

    void export_step(const std::filesystem::path &step_file,
                     const std::string &unit = "m", const std::string &schema = "AP214") const
    {
        // Create the directory if it doesn't exist and check that step_file.parent_path() is not ""

        if (!step_file.parent_path().empty() && step_file.parent_path() != "") {
            std::filesystem::create_directories(step_file.parent_path());
        }

        // Set up the writer
        Handle(XSControl_WorkSession) session = new XSControl_WorkSession();

        STEPCAFControl_Writer writer(session, Standard_False);
        writer.SetColorMode(Standard_True);
        writer.SetNameMode(Standard_True);

        // Interface_Static is process-global; restore these on scope exit so a
        // STEP write doesn't leak its unit/schema into later OCC operations.
        const InterfaceStaticCValGuard unit_guard("write.step.unit");
        const InterfaceStaticCValGuard schema_guard("write.step.schema");
        Interface_Static::SetCVal("write.step.unit", unit.c_str());
        Interface_Static::SetCVal("write.step.schema", schema.c_str());

        writer.Transfer(doc_, STEPControl_AsIs);
        IFSelect_ReturnStatus status = writer.Write(step_file.string().c_str());

        if (status != IFSelect_RetDone) {
            throw std::runtime_error("STEP export failed");
        } else {
            std::cout << "STEP export status: " << static_cast<int>(status) << std::endl;
        }
    }

private:
    Handle(TDocStd_Application) app_;
    Handle(TDocStd_Document) doc_;
    Handle(XCAFDoc_ShapeTool) shape_tool_;
    Handle(XCAFDoc_ColorTool) color_tool_;
    TopoDS_Compound comp_;
    BRep_Builder comp_builder_;
    TDF_Label tll_;
};

// take in a list of box dimensions and origins and write to step file using the AdaCPPStepWriter class
void write_boxes_to_step(const std::string &filename, const std::vector<std::vector<float>> &box_origins,
                         const std::vector<std::vector<float>> &box_dims) {
    AdaCPPStepWriter writer(filename);
    for (int i = 0; i < box_origins.size(); i++) {
        TopoDS_Solid box = create_box(box_origins[i], box_dims[i]);
        Color color = random_color();
        writer.add_shape(box, "box_" + std::to_string(i), color);
    }
    writer.export_step(filename);
}

// Write arbitrary shapes (each with a name + color) to a STEP file with OCAF
// names/colors. Backs adapy's StepWriter under the adacpp CAD backend.
void write_shapes_to_step(const std::string &filename, const std::vector<TopoDS_Shape> &shapes,
                          const std::vector<std::string> &names, const std::vector<Color> &colors,
                          const std::string &unit, const std::string &schema,
                          const std::string &top_level_name) {
    AdaCPPStepWriter writer(top_level_name);
    for (std::size_t i = 0; i < shapes.size(); ++i) {
        if (shapes[i].IsNull()) continue;
        const std::string name = i < names.size() ? names[i] : ("shape_" + std::to_string(i));
        const Color color = i < colors.size() ? colors[i] : Color();
        writer.add_shape_handle(shapes[i], name, color);
    }
    writer.export_step(filename, unit, schema);
}

// take in a single box dimension and origin and write to step file using the STEPControl_Writer class
void write_box_to_step(const std::string &filename, const std::vector<float> &box_origin,
                       const std::vector<float> &box_dims) {

    const TopoDS_Solid aBox = create_box(box_origin, box_dims);
    STEPControl_Writer writer;
    writer.Transfer(aBox, STEPControl_AsIs);
    writer.Write(filename.c_str());
}

std::string step_writer_to_string(STEPCAFControl_Writer& writer) {
    std::ostringstream stream;
    writer.WriteStream(stream);  // Write to the string stream
    return stream.str();  // Convert the stream to a string and return it
}
