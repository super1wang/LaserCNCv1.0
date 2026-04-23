#pragma once

/**
 * @brief Static utility functions wrapping the XDE (XCAF) layer of OpenCASCADE.
 *
 * Provides thin, named helpers so callsites don't need to include the full
 * set of XCAF headers directly. All methods are static (no state).
 */

#include <QString>
#include <Standard_Handle.hxx>
#include <TDF_Label.hxx>
#include <TDF_Attribute.hxx>
#include <TopoDS_Shape.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <Quantity_Color.hxx>
#include <TDataStd_Name.hxx>
#include <TDataStd_Integer.hxx>

class XcafUtils
{
public:
    // ── Label helpers ─────────────────────────────────────────────────────────

    /// Get or create a child label with a given integer tag under @a parent.
    static TDF_Label findOrCreateChild(const TDF_Label& parent, int tag);

    /// Attach a name attribute to a label.
    static void setName(const TDF_Label& label, const QString& name);

    /// Read the name attribute, or "" if absent.
    static QString name(const TDF_Label& label);

    /// Return the entry string e.g. "0:1:2:3".
    static QString entry(const TDF_Label& label);

    // ── Shape helpers ─────────────────────────────────────────────────────────

    /// Retrieve the TopoDS_Shape stored at this label (via XCAFDoc_ShapeTool).
    static TopoDS_Shape shape(const TDF_Label& label);

    /// True when this label represents an assembly (has shape references).
    static bool isAssembly(const TDF_Label& label,
                           const Handle(XCAFDoc_ShapeTool)& shapeTool);

    // ── Color helpers ─────────────────────────────────────────────────────────
    static bool getColor(const TDF_Label&               label,
                         const Handle(XCAFDoc_ColorTool)& colorTool,
                         Quantity_Color&                 color);
};
