#include "core/document/xcaf_utils.h"

#include <TDF_Tool.hxx>
#include <TDF_ChildIterator.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TCollection_AsciiString.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <TNaming_NamedShape.hxx>
#include <XCAFDoc_ShapeTool.hxx>

// ── Label helpers ──────────────────────────────────────────────────────────────

TDF_Label XcafUtils::findOrCreateChild(const TDF_Label& parent, int tag)
{
    return parent.FindChild(tag, /*create=*/Standard_True);
}

void XcafUtils::setName(const TDF_Label& label, const QString& name)
{
    TDataStd_Name::Set(label,
        TCollection_ExtendedString(name.toStdWString().c_str()));
}

QString XcafUtils::name(const TDF_Label& label)
{
    Handle(TDataStd_Name) nameAttr;
    if (label.FindAttribute(TDataStd_Name::GetID(), nameAttr)) {
        const TCollection_ExtendedString& ext = nameAttr->Get();
        std::wstring ws;
        ws.reserve(static_cast<size_t>(ext.Length()));
        for (Standard_Integer i = 1; i <= ext.Length(); ++i)
            ws.push_back(static_cast<wchar_t>(ext.Value(i)));
        return QString::fromStdWString(ws);
    }
    return QString();
}

QString XcafUtils::entry(const TDF_Label& label)
{
    TCollection_AsciiString ent;
    TDF_Tool::Entry(label, ent);
    return QString::fromLatin1(ent.ToCString());
}

// ── Shape helpers ──────────────────────────────────────────────────────────────

TopoDS_Shape XcafUtils::shape(const TDF_Label& label)
{
    // Retrieve shape stored by XDE (via TNaming_NamedShape attribute)
    Handle(TNaming_NamedShape) ns;
    if (label.FindAttribute(TNaming_NamedShape::GetID(), ns))
        return ns->Get();
    return TopoDS_Shape();
}

bool XcafUtils::isAssembly(const TDF_Label& label,
                            const Handle(XCAFDoc_ShapeTool)& shapeTool)
{
    return shapeTool->IsAssembly(label);
}

// ── Color helpers ──────────────────────────────────────────────────────────────

bool XcafUtils::getColor(const TDF_Label&               label,
                         const Handle(XCAFDoc_ColorTool)& colorTool,
                         Quantity_Color&                  color)
{
    return colorTool->GetColor(label, XCAFDoc_ColorGen, color);
}
