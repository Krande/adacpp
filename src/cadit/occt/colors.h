// Filename: SetInstanceColor.hpp

#ifndef SETINSTANCECOLOR_HPP
#define SETINSTANCECOLOR_HPP

#include <TDF_Label.hxx>
#include <TopoDS_Shape.hxx>
#include <Quantity_Color.hxx>
#include <XCAFDoc_ColorTool.hxx>
void setInstanceColorIfAvailable(XCAFDoc_ColorTool* color_tool, const TDF_Label& lab, const TopoDS_Shape& shape,
                                 Quantity_Color& c);
#endif // SETINSTANCECOLOR_HPP
