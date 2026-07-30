#include "modules/cam/services/machine_io.h"

#include "core/document/lcnc_document.h"
#include "core/document/xcaf_utils.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/task/task_manager.h"

#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <StlAPI_Reader.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopoDS_Compound.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <QFileInfo>
#include <QSet>
#include <QString>
#include <QStringList>

namespace lcnc::cam::machine_io {

bool loadMachineFromFile(LcncDocument* doc,
                         const QString& filePath,
                         TaskProgress* progress,
                         const std::function<void(const QString& entry, const TopoDS_Shape& shape)>& onShapeLoaded)
{
    if (!doc || filePath.isEmpty())
        return false;

    QFileInfo fi(filePath);
    if (!fi.exists())
        return false;

    const QString ext = fi.suffix().toLower();
    if (progress)
        progress->setRange(0, 100);

    if (ext == "stp" || ext == "step") {
        if (progress)
            // 中文翻译：读取 STEP...
            progress->setStepName(QStringLiteral("Read STEP..."));
        Handle(TDocStd_Document) xdeDoc =
            new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
        XCAFDoc_DocumentTool::Set(xdeDoc->Main());
        STEPCAFControl_Reader cafReader;
        cafReader.SetNameMode(Standard_True);
        if (cafReader.ReadFile(filePath.toUtf8().constData()) != IFSelect_RetDone) {
            if (progress)
                progress->setValue(100);
            return false;
        }
        if (progress) {
            progress->setValue(50);
            // 中文翻译：转换形体...
            progress->setStepName(QStringLiteral("Transform body..."));
        }
        cafReader.Transfer(xdeDoc);
        if (progress)
            progress->setValue(80);
        QSet<QString> existingEntries;
        if (onShapeLoaded) {
            const TDF_LabelSequence existing = doc->entityLabels(LcncDocument::EntityKind::Machine);
            for (int i = 1; i <= existing.Length(); ++i)
                existingEntries.insert(XcafUtils::entry(existing.Value(i)));
        }

        doc->importFromXcafRoots(xdeDoc, LcncDocument::EntityKind::Machine);

        if (onShapeLoaded) {
            const TDF_LabelSequence imported = doc->entityLabels(LcncDocument::EntityKind::Machine);
            for (int i = 1; i <= imported.Length(); ++i) {
                const TDF_Label label = imported.Value(i);
                const QString entry = XcafUtils::entry(label);
                if (!existingEntries.contains(entry))
                    onShapeLoaded(entry, XcafUtils::shape(label));
            }
        }
    } else if (ext == "stl") {
        if (progress)
            // 中文翻译：读取 STL...
            progress->setStepName(QStringLiteral("Read STL..."));
        TopoDS_Shape shape;
        StlAPI_Reader stlReader;
        stlReader.Read(shape, filePath.toUtf8().constData());
        if (progress)
            progress->setValue(80);
        if (shape.IsNull())
            return false;
        const TDF_Label label = doc->addShapeEntity(shape, fi.baseName(), LcncDocument::EntityKind::Machine);
        if (onShapeLoaded)
            onShapeLoaded(XcafUtils::entry(label), shape);
    } else if (ext == "brep") {
        if (progress)
            // 中文翻译：读取 BREP...
            progress->setStepName(QStringLiteral("Read BREP..."));
        TopoDS_Shape shape;
        BRep_Builder builder;
        BRepTools::Read(shape, filePath.toUtf8().constData(), builder);
        if (shape.IsNull())
            return false;
        const TDF_Label label = doc->addShapeEntity(shape, fi.baseName(), LcncDocument::EntityKind::Machine);
        if (onShapeLoaded)
            onShapeLoaded(XcafUtils::entry(label), shape);
    } else {
        return false;
    }

    if (progress)
        progress->setValue(100);
    return true;
}

bool exportMachineToFile(LcncDocument* doc,
                         MachineKinematics* kin,
                         const QString& filePath)
{
    if (!doc || !kin || filePath.isEmpty())
        return false;

    Handle(TDocStd_Document) xdeExport =
        new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
    XCAFDoc_DocumentTool::Set(xdeExport->Main());
    Handle(XCAFDoc_ShapeTool) stExp = XCAFDoc_DocumentTool::ShapeTool(xdeExport->Main());
    Handle(XCAFDoc_ShapeTool) stMach = doc->shapeTool();

    QSet<QString> assignedEntries;

    for (const MachineAxisDef& axis : kin->axes()) {
        const QStringList entries = kin->shapesForAxis(axis.name);
        if (entries.isEmpty())
            continue;

        BRep_Builder bb;
        TopoDS_Compound axisCompound;
        bb.MakeCompound(axisCompound);
        bool hasShape = false;

        TDF_LabelSequence freeShapes;
        stMach->GetFreeShapes(freeShapes);

        for (const QString& entry : entries) {
            assignedEntries.insert(entry);
            for (int i = 1; i <= freeShapes.Length(); ++i) {
                if (XcafUtils::entry(freeShapes.Value(i)) == entry) {
                    TopoDS_Shape sh = stMach->GetShape(freeShapes.Value(i));
                    if (!sh.IsNull()) {
                        bb.Add(axisCompound, sh);
                        hasShape = true;
                    }
                    break;
                }
            }
        }

        if (!hasShape)
            continue;

        const QString axisLabel = QStringLiteral("LCNC_AXIS_") + axis.name;
        TDF_Label lbl = stExp->AddShape(axisCompound, Standard_False);
        TDataStd_Name::Set(lbl, TCollection_ExtendedString(axisLabel.toStdString().c_str()));
    }

    // 未分配组：导出剩余自由形体。
    {
        TDF_LabelSequence freeShapes;
        stMach->GetFreeShapes(freeShapes);
        BRep_Builder bb;
        TopoDS_Compound unassigned;
        bb.MakeCompound(unassigned);
        bool hasUnassigned = false;

        for (int i = 1; i <= freeShapes.Length(); ++i) {
            const QString entry = XcafUtils::entry(freeShapes.Value(i));
            if (!assignedEntries.contains(entry)) {
                TopoDS_Shape sh = stMach->GetShape(freeShapes.Value(i));
                if (!sh.IsNull()) {
                    bb.Add(unassigned, sh);
                    hasUnassigned = true;
                }
            }
        }

        if (hasUnassigned) {
            TDF_Label lbl = stExp->AddShape(unassigned, Standard_False);
            TDataStd_Name::Set(lbl, TCollection_ExtendedString("LCNC_AXIS_UNASSIGNED"));
        }
    }

    STEPCAFControl_Writer writer;
    writer.SetNameMode(Standard_True);
    if (writer.Transfer(xdeExport) != IFSelect_RetDone)
        return false;
    return writer.Write(filePath.toUtf8().constData()) == IFSelect_RetDone;
}

} // namespace lcnc::cam::machine_io
