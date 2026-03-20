#include "gui/gui_document.h"
#include "base/lcnc_application.h"
#include "base/lcnc_document.h"
#include "base/xcaf_utils.h"
#include "graphics/shape_object_driver.h"

#include <TDF_LabelSequence.hxx>
#include <XCAFDoc_ShapeTool.hxx>

GuiDocument::GuiDocument(DocumentId id, QObject* parent)
    : QObject(parent)
    , m_docId(id)
    , m_scene(new GraphicsScene(this))
{}

GuiDocument::~GuiDocument() = default;

LcncDocument* GuiDocument::document() const
{
    return LcncApplication::instance()->documentById(m_docId);
}

Handle(AIS_Shape) GuiDocument::displayShape(const TopoDS_Shape& shape,
                                             const QString&      name,
                                             bool                fitAll)
{
    Handle(AIS_Shape) ais = m_scene->displayShape(shape, fitAll);
    m_aisMap.insert(name, ais);
    emit displayUpdated();
    return ais;
}

void GuiDocument::eraseEntity(const QString& labelEntry)
{
    if (m_aisMap.contains(labelEntry)) {
        m_scene->eraseShape(m_aisMap.value(labelEntry));
        m_aisMap.remove(labelEntry);
        emit displayUpdated();
    }
}

void GuiDocument::rebuildDisplay()
{
    LcncDocument* doc = document();
    if (!doc) return;

    m_scene->eraseAll();
    m_aisMap.clear();

    Handle(XCAFDoc_ShapeTool) st = doc->shapeTool();
    TDF_LabelSequence freeShapes;
    st->GetFreeShapes(freeShapes);

    for (int i = 1; i <= freeShapes.Length(); ++i) {
        TDF_Label lbl   = freeShapes.Value(i);
        TopoDS_Shape sh = XcafUtils::shape(lbl);
        if (!sh.IsNull()) {
            Handle(AIS_Shape) ais = m_scene->displayShape(sh);
            m_aisMap.insert(XcafUtils::entry(lbl), ais);
        }
    }

    emit displayUpdated();
}

Handle(AIS_Shape) GuiDocument::aisShape(const QString& labelEntry) const
{
    return m_aisMap.value(labelEntry, Handle(AIS_Shape)());
}
