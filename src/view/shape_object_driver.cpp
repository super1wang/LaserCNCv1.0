#include "view/shape_object_driver.h"

#include <AIS_Shape.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Prs3d_ShadingAspect.hxx>
#include <Graphic3d_MaterialAspect.hxx>
#include <Quantity_Color.hxx>

Handle(AIS_Shape) ShapeObjectDriver::createAisShape(const TopoDS_Shape& shape)
{
    Handle(AIS_Shape) ais = new AIS_Shape(shape);
    applyDefaultStyle(ais);
    return ais;
}

void ShapeObjectDriver::applyDefaultStyle(const Handle(AIS_Shape)& aisShape)
{
    // Use the shape's own drawer (don't modify context defaults)
    aisShape->Attributes()->SetFaceBoundaryDraw(true);
    aisShape->Attributes()->FaceBoundaryAspect()
        ->SetColor(Quantity_NOC_GRAY35);
    aisShape->Attributes()->FaceBoundaryAspect()
        ->SetWidth(1.0);

    // Selection highlight colour
    aisShape->DynamicHilightAttributes()->ShadingAspect()
        ->SetColor(Quantity_NOC_CYAN1);
    aisShape->HilightAttributes()->ShadingAspect()
        ->SetColor(Quantity_NOC_CYAN1);
}

void ShapeObjectDriver::setMaterial(const Handle(AIS_Shape)& aisShape,
                                    Graphic3d_NameOfMaterial  material,
                                    double                    transparency)
{
    aisShape->SetMaterial(Graphic3d_MaterialAspect(material));
    if (transparency > 0.0)
        aisShape->SetTransparency(static_cast<float>(transparency));
}
