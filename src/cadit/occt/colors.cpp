//
// Created by Kristoffer on 17/09/2023.
//

#include "colors.h"
#include <TDF_Label.hxx>
#include <TopoDS_Shape.hxx>
#include <Quantity_Color.hxx>
#include <XCAFDoc_ColorTool.hxx>


void setInstanceColorIfAvailable(XCAFDoc_ColorTool *color_tool, const TDF_Label &lab, const TopoDS_Shape &shape,
                                         Quantity_Color &c) {
    const auto c1 = static_cast<const XCAFDoc_ColorType>(0);
    const auto c2 = static_cast<const XCAFDoc_ColorType>(1);
    const auto c3 = static_cast<const XCAFDoc_ColorType>(2);

    if (XCAFDoc_ColorTool::GetColor(lab, c1, c) || XCAFDoc_ColorTool::GetColor(lab, c2, c) ||
        XCAFDoc_ColorTool::GetColor(lab, c3, c)) {
        color_tool->SetInstanceColor(shape, c1, c);
        color_tool->SetInstanceColor(shape, c2, c);
        color_tool->SetInstanceColor(shape, c3, c);
    }
}
