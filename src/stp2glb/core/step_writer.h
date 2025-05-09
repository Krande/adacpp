// AdaCPPStepWriter.h

#pragma once

#include <BRep_Builder.hxx>
#include <filesystem>
#include <TDocStd_Application.hxx>
#include <TDocStd_Document.hxx>
#include <TopoDS_Compound.hxx>
#include <TDF_Label.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include "step_tree.h"
#include "../../geom/Color.h"



class StepStore {
public:
    Handle(TDocStd_Document) doc_;

    explicit StepStore(const std::string& top_level_name = "Assembly");

    explicit StepStore(const std::vector<std::unique_ptr<ProductNode>>& product_hierarchy);

    void add_shape(const TopoDS_Shape& shape, const std::string& name, const Color& rgb_color,
        const ProductNode& parent_node);

    void to_step(const std::filesystem::path& step_file) const;

    void to_glb(const std::filesystem::path& glb_file) const;

private:
    // Handles (smart pointers) to OCC classes
    Handle(TDocStd_Application) app_;
    Handle(XCAFDoc_ShapeTool) shape_tool_;
    Handle(XCAFDoc_ColorTool) color_tool_;

    TopoDS_Compound comp_;
    BRep_Builder comp_builder_;
    TDF_Label tll_;

    // Map to store product name to TDF_Label mapping for hierarchy
    std::unordered_map<std::string, TDF_Label> product_labels_;

    // Map to store source entity index to TDF_Label mapping for shapes
    std::unordered_map<int, TDF_Label> entity_labels_;

    void initialize();
    void create_hierarchy(const std::vector<std::unique_ptr<ProductNode>> &nodes, const TDF_Label &parent_label);
    static void set_name(const TDF_Label& label, const std::string& name);
    static void set_color(const TDF_Label& label, const Color& rgb_color, const Handle(XCAFDoc_ColorTool)& color_tool);
};
