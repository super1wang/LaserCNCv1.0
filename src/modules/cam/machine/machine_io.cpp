#include "modules/cam/machine/machine_io.h"

#include "core/document/lcnc_document.h"
#include "core/document/xcaf_utils.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/logging/logger.h"
#include "core/task/task_manager.h"

#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <NCollection_Sequence.hxx>
#include <QFileInfo>
#include <QSet>
#include <QString>
#include <QStringList>
#include <STEPCAFControl_Reader.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <Standard_Failure.hxx>
#include <StlAPI_Reader.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDF_Label.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS_Compound.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_Location.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <exception>

namespace lcnc::cam::machine_io {

static bool readMachineFileImpl(const QString& filePath,
                                TaskProgress* progress,
                                MachineImportResult* result)
{
    if (!result)
        return false;
    *result = {};
    if (filePath.isEmpty()) {
        result->error = QStringLiteral("Machine model path is empty");
        return false;
    }

    QFileInfo fi(filePath);
    if (!fi.exists()) {
        result->error = QStringLiteral("Machine model file does not exist");
        return false;
    }

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
        cafReader.SetNameMode(true);
        if (cafReader.ReadFile(filePath.toUtf8().constData()) != IFSelect_RetDone) {
            result->error = QStringLiteral("Unable to read STEP machine model");
            if (progress)
                progress->setValue(100);
            return false;
        }
        if (progress) {
            progress->setValue(50);
            // 中文翻译：转换形体...
            progress->setStepName(QStringLiteral("Transform body..."));
        }
        if (!cafReader.Transfer(xdeDoc)) {
            result->error = QStringLiteral("Unable to transfer STEP machine model");
            return false;
        }
        if (progress)
            progress->setValue(80);
        Handle(XCAFDoc_ShapeTool) shapes = XCAFDoc_DocumentTool::ShapeTool(xdeDoc->Main());
        NCollection_Sequence<TDF_Label> roots;
        shapes->GetFreeShapes(roots);
        for (int i = 1; i <= roots.Length(); ++i) {
            const TDF_Label root = roots.Value(i);
            NCollection_Sequence<TDF_Label> components;
            shapes->GetComponents(root, components);
            if (components.IsEmpty()) {
                const TopoDS_Shape shape = shapes->GetShape(root);
                if (shape.IsNull())
                    continue;
                QString name = XcafUtils::name(root);
                if (name.isEmpty())
                    name = QStringLiteral("Part_%1").arg(i);
                result->parts.append({name, shape});
                continue;
            }

            // Keep one direct assembly component per selectable machine part.
            // Recursing into every design subshape would make a real machine
            // unusable in the tree and collision-role UI; direct components
            // retain a practical axis-assignment granularity.
            for (int componentIndex = 1; componentIndex <= components.Length(); ++componentIndex) {
                const TDF_Label component = components.Value(componentIndex);
                QString name = XcafUtils::name(component);
                TopLoc_Location location;
                Handle(XCAFDoc_Location) locationAttribute;
                if (component.FindAttribute(XCAFDoc_Location::GetID(), locationAttribute))
                    location = locationAttribute->Get();

                TDF_Label referred;
                TopoDS_Shape shape;
                if (shapes->GetReferredShape(component, referred)) {
                    if (name.isEmpty())
                        name = XcafUtils::name(referred);
                    shape = shapes->GetShape(referred);
                } else {
                    shape = shapes->GetShape(component);
                }
                if (shape.IsNull())
                    continue;
                if (!location.IsIdentity())
                    shape = shape.Located(location * shape.Location());
                if (name.isEmpty())
                    name = QStringLiteral("Part_%1_%2").arg(i).arg(componentIndex);
                result->parts.append({name, shape});
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
        if (shape.IsNull()) {
            result->error = QStringLiteral("Unable to read STL machine model");
            return false;
        }
        result->parts.append({fi.baseName(), shape});
    } else if (ext == "brep") {
        if (progress)
            // 中文翻译：读取 BREP...
            progress->setStepName(QStringLiteral("Read BREP..."));
        TopoDS_Shape shape;
        BRep_Builder builder;
        BRepTools::Read(shape, filePath.toUtf8().constData(), builder);
        if (shape.IsNull()) {
            result->error = QStringLiteral("Unable to read BREP machine model");
            return false;
        }
        result->parts.append({fi.baseName(), shape});
    } else {
        result->error = QStringLiteral("Unsupported machine model format");
        return false;
    }

    if (result->parts.isEmpty()) {
        result->error = QStringLiteral("Machine model contains no usable shape");
        return false;
    }
    if (progress)
        progress->setValue(100);
    return true;
}

bool readMachineFile(const QString& filePath,
                     TaskProgress* progress,
                     MachineImportResult* result)
{
    try {
        return readMachineFileImpl(filePath, progress, result);
    } catch (const Standard_Failure& failure) {
        LCNC_ERR(lcnc::LogCode::Generic, "cam.machine_io: OCCT import failure: {}", failure.what());
        if (result)
            result->error = QString::fromUtf8(failure.what());
    } catch (const std::exception& failure) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "cam.machine_io: machine import failure: {}", failure.what());
        if (result)
            result->error = QString::fromUtf8(failure.what());
    } catch (...) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "cam.machine_io: unknown machine import failure");
        if (result)
            result->error = QStringLiteral("Unknown machine import failure");
    }
    return false;
}

static bool exportMachineToFileImpl(LcncDocument* doc,
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

        NCollection_Sequence<TDF_Label> freeShapes;
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
        TDF_Label lbl = stExp->AddShape(axisCompound, false);
        TDataStd_Name::Set(lbl, TCollection_ExtendedString(axisLabel.toStdString().c_str()));
    }

    // 未分配组：导出剩余自由形体。
    {
        NCollection_Sequence<TDF_Label> freeShapes;
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
            TDF_Label lbl = stExp->AddShape(unassigned, false);
            TDataStd_Name::Set(lbl, TCollection_ExtendedString("LCNC_AXIS_UNASSIGNED"));
        }
    }

    STEPCAFControl_Writer writer;
    writer.SetNameMode(true);
    if (writer.Transfer(xdeExport) != IFSelect_RetDone)
        return false;
    return writer.Write(filePath.toUtf8().constData()) == IFSelect_RetDone;
}

bool exportMachineToFile(LcncDocument* doc,
                         MachineKinematics* kin,
                         const QString& filePath)
{
    try {
        return exportMachineToFileImpl(doc, kin, filePath);
    } catch (const Standard_Failure& failure) {
        LCNC_ERR(lcnc::LogCode::Generic, "cam.machine_io: OCCT export failure: {}", failure.what());
    } catch (const std::exception& failure) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "cam.machine_io: machine export failure: {}", failure.what());
    } catch (...) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "cam.machine_io: unknown machine export failure");
    }
    return false;
}

} // namespace lcnc::cam::machine_io
