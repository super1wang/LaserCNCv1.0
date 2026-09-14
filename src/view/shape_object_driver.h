#pragma once

#include <Standard_Handle.hxx>
#include <AIS_Shape.hxx>
#include <TopoDS_Shape.hxx>
#include <AIS_InteractiveContext.hxx>
#include <Prs3d_Drawer.hxx>

/**
 * @brief Thin helper that creates and configures AIS_Shape objects.
 *
 * Centralises appearance defaults (shading, edge colour, line width) so
 * every shape displayed via the scene looks consistent.
 */
class ShapeObjectDriver
{
public:
    /// Create a display-ready AIS_Shape from raw topology.
    static Handle(AIS_Shape) createAisShape(const TopoDS_Shape& shape);

    /// Apply standard visual defaults to an existing AIS_Shape.
    static void applyDefaultStyle(const Handle(AIS_Shape)& aisShape);

    /// Set material / transparency on an AIS_Shape.
    static void setMaterial(const Handle(AIS_Shape)& aisShape,
                             Graphic3d_NameOfMaterial material,
                             double transparency = 0.0);
};
