#include "app/commands_machine.h"
#include "app/dialog_mark_axes.h"
#include "app/i_app_context.h"

#include "base/lcnc_application.h"
#include "base/lcnc_document.h"
#include "base/machine_kinematics.h"
#include "base/xcaf_utils.h"
#include "base/task_manager.h"
#include "gui/gui_application.h"
#include "gui/gui_document.h"

#include <QAction>
#include <QIcon>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QComboBox>
#include <QLabel>

#include <QSet>

#include <STEPCAFControl_Reader.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <TCollection_ExtendedString.hxx>
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <StlAPI_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TDataStd_Name.hxx>
#include <TopoDS_Compound.hxx>

// ── CmdLoadMachine ─────────────────────────────────────────────────────────────

CmdLoadMachine::CmdLoadMachine(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/machine.svg"), tr("加载机台"), this);
    a->setStatusTip(tr("加载机台三维模型并初始化轴系配置"));
    setAction(a);
}

void CmdLoadMachine::execute()
{
    // ── Step 1: pick machine kinematic configuration ───────────────────────
    QDialog cfgDlg;
    cfgDlg.setWindowTitle(tr("机台配置"));
    auto* vl  = new QVBoxLayout(&cfgDlg);
    auto* frm = new QFormLayout;
    auto* cbConfig = new QComboBox;
    cbConfig->addItem(tr("AC 转台（垂直主轴）"), QStringLiteral("VERTICAL_AC_TABLE"));
    cbConfig->addItem(tr("BC 转台（垂直主轴）"), QStringLiteral("VERTICAL_BC_TABLE"));
    cbConfig->addItem(tr("AB 摆头"),             QStringLiteral("AB_HEAD"));
    cbConfig->addItem(tr("AC 摆头"),             QStringLiteral("AC_HEAD"));
    frm->addRow(tr("机台构型:"), cbConfig);
    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(btns, &QDialogButtonBox::accepted, &cfgDlg, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, &cfgDlg, &QDialog::reject);
    vl->addLayout(frm);
    vl->addWidget(btns);

    // Pre-select config from existing machine doc kinematics if already loaded
    if (LcncDocument* existing = context()->machineDocument()) {
        const QString cur = existing->machineKinematics()->configType();
        for (int i = 0; i < cbConfig->count(); ++i) {
            if (cbConfig->itemData(i).toString() == cur) {
                cbConfig->setCurrentIndex(i);
                break;
            }
        }
    }

    if (cfgDlg.exec() != QDialog::Accepted) return;
    const QString configType = cbConfig->currentData().toString();

    // ── Step 2: pick model file ────────────────────────────────────────────
    const QString path = QFileDialog::getOpenFileName(
        nullptr, tr("选择机台模型文件"), QString(),
        tr("三维模型文件 (*.stp *.step *.stl *.brep);;"
           "STEP (*.stp *.step);;"
           "STL (*.stl);;"
           "BREP (*.brep)"));
    if (path.isEmpty()) return;

    QFileInfo fi(path);
    const QString ext = fi.suffix().toLower();

    // ── Step 3: operate on the unique machine workspace document ─────────
    LcncDocument* doc = context()->machineDocument();
    if (!doc) return;

    // Clear any previously loaded machine entities
    {
        TDF_LabelSequence existing = doc->entityLabels(LcncDocument::EntityKind::Machine);
        QStringList entriesToRemove;
        for (int i = 1; i <= existing.Length(); ++i)
            entriesToRemove << XcafUtils::entry(existing.Value(i));
        for (const QString& e : entriesToRemove)
            doc->removeShapeEntity(e);
    }

    // Reset and reconfigure kinematics
    doc->machineKinematics()->clear();
    doc->machineKinematics()->loadPreset(configType);

    // ── Step 4: import in background ──────────────────────────────────────
    TaskId taskId = taskMgr()->run(tr("加载机台: %1").arg(fi.fileName()),
        [path, ext, doc](TaskProgress* prog) {
            prog->setRange(0, 100);

            if (ext == "stp" || ext == "step") {
                prog->setStepName(QStringLiteral("读取 STEP..."));
                Handle(TDocStd_Document) xdeDoc =
                    new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
                XCAFDoc_DocumentTool::Set(xdeDoc->Main());
                STEPCAFControl_Reader cafReader;
                cafReader.SetNameMode(Standard_True);
                if (cafReader.ReadFile(path.toUtf8().constData()) == IFSelect_RetDone) {
                    prog->setValue(50);
                    prog->setStepName(QStringLiteral("转换形体..."));
                    cafReader.Transfer(xdeDoc);
                    prog->setValue(80);
                    doc->importFromXcaf(xdeDoc, LcncDocument::EntityKind::Machine);
                } else {
                    prog->setStepName(QStringLiteral("读取STEP失败"));
                }
            } else if (ext == "stl") {
                prog->setStepName(QStringLiteral("读取 STL..."));
                TopoDS_Shape shape;
                StlAPI_Reader stlReader;
                stlReader.Read(shape, path.toUtf8().constData());
                prog->setValue(80);
                if (!shape.IsNull())
                    doc->addShapeEntity(shape, QFileInfo(path).baseName(), LcncDocument::EntityKind::Machine);
            } else if (ext == "brep") {
                prog->setStepName(QStringLiteral("读取 BREP..."));
                TopoDS_Shape shape;
                BRep_Builder builder;
                BRepTools::Read(shape, path.toUtf8().constData(), builder);
                if (!shape.IsNull())
                    doc->addShapeEntity(shape, QFileInfo(path).baseName(), LcncDocument::EntityKind::Machine);
            }

            prog->setValue(100);
        });

    connect(TaskManager::instance(), &TaskManager::taskFinished, this,
            [this, doc, taskId](TaskId id, bool ok) {
                if (id != taskId) return;
                if (!ok) return;

                // Auto-detect axis assignments
                TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Machine);
                QMap<QString,QString> entryToName;
                for (int i = 1; i <= labels.Length(); ++i) {
                    TDF_Label lbl = labels.Value(i);
                    entryToName.insert(XcafUtils::entry(lbl), XcafUtils::name(lbl));
                }
                doc->machineKinematics()->autoDetect(entryToName);

                // Rebuild 3D display
                if (auto* gd = guiApp()->guiDocument(doc->id()))
                    gd->rebuildDisplay();

                app()->notifyDocumentModified(doc->id());
                context()->updateCommandStates();
            });
}

// ── CmdMarkAxes ───────────────────────────────────────────────────────────────

CmdMarkAxes::CmdMarkAxes(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/coordinate.svg"), tr("标记轴系"), this);
    a->setStatusTip(tr("手动为机台各零件指定所属轴系"));
    setAction(a);
}

bool CmdMarkAxes::isEnabled() const
{
    LcncDocument* doc = context()->machineDocument();
    if (!doc) return false;
    return doc->entityLabels(LcncDocument::EntityKind::Machine).Length() > 0;
}

void CmdMarkAxes::execute()
{
    LcncDocument* doc = context()->machineDocument();
    if (!doc) return;

    MachineKinematics* kin = doc->machineKinematics();
    if (kin->axes().isEmpty()) {
        QMessageBox::information(nullptr, tr("标记轴系"),
            tr("请先通过[加载机台]命令选择机台构型并加载模型。"));
        return;
    }

    DialogMarkAxes dlg(doc, kin, nullptr);
    if (dlg.exec() == QDialog::Accepted) {
        app()->notifyDocumentModified(doc->id());
        context()->updateCommandStates();
    }
}

// ── CmdMountWorkpiece ─────────────────────────────────────────────────────────

CmdMountWorkpiece::CmdMountWorkpiece(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/workpiece.svg"), tr("挂载工件"), this);
    a->setStatusTip(tr("将工件模型绑定到指定轴系，随轴运动"));
    setAction(a);
}

bool CmdMountWorkpiece::isEnabled() const
{
    LcncDocument* machDoc = context()->machineDocument();
    if (!machDoc) return false;
    if (machDoc->entityLabels(LcncDocument::EntityKind::Machine).Length() == 0)
        return false;
    // At least one workpiece document must exist
    return !app()->workpieceDocuments().isEmpty();
}

void CmdMountWorkpiece::execute()
{
    LcncDocument* machDoc = context()->machineDocument();
    if (!machDoc) return;

    MachineKinematics* kin = machDoc->machineKinematics();
    if (kin->axes().isEmpty()) {
        QMessageBox::information(nullptr, tr("挂载工件"),
            tr("请先加载机台模型并配置轴系。"));
        return;
    }

    // Gather all open workpiece documents
    const QList<LcncDocument*> wpcDocs = app()->workpieceDocuments();
    if (wpcDocs.isEmpty()) {
        QMessageBox::information(nullptr, tr("挂载工件"),
            tr("请先打开至少一个工件文档。"));
        return;
    }

    // ── Build dialog ───────────────────────────────────────────────────────
    QDialog dlg;
    dlg.setWindowTitle(tr("工件挂载"));
    auto* frm = new QFormLayout;

    auto* cbDoc  = new QComboBox;
    auto* cbAxis = new QComboBox;

    // Populate document combo — show document name + workpiece entity count
    for (LcncDocument* d : wpcDocs) {
        const int wpcCount = d->entityLabels(LcncDocument::EntityKind::Workpiece).Length();
        if (wpcCount == 0) continue;
        cbDoc->addItem(tr("%1  (%2 形体)").arg(d->name()).arg(wpcCount), d->id());
    }
    if (cbDoc->count() == 0) {
        QMessageBox::information(nullptr, tr("挂载工件"),
            tr("打开的文档中没有工件模型。"));
        return;
    }

    // Populate axis combo
    cbAxis->addItem(tr("— 解除已有挂载 —"), QString());
    for (const auto& axis : kin->axes()) {
        if (axis.name == "BASE") {
            cbAxis->addItem(tr("BASE（固定基座）"), axis.name);
        } else if (axis.motionType == MachineAxisDef::Rotary) {
            cbAxis->addItem(tr("%1 轴（旋转）").arg(axis.name), axis.name);
        } else {
            cbAxis->addItem(tr("%1 轴（线性）").arg(axis.name), axis.name);
        }
    }

    frm->addRow(tr("工件文档:"), cbDoc);
    frm->addRow(tr("挂载到:"),   cbAxis);

    // Info label showing what will happen
    auto* lblInfo = new QLabel(tr("将选中文档的所有工件合并为一个整体加载到机台文档"), &dlg);
    lblInfo->setStyleSheet("color: gray; font-size: 11px;");
    lblInfo->setWordWrap(true);

    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    auto* vl = new QVBoxLayout(&dlg);
    vl->addLayout(frm);
    vl->addWidget(lblInfo);
    vl->addWidget(btns);

    if (dlg.exec() != QDialog::Accepted) return;

    const DocumentId srcDocId = cbDoc->currentData().toInt();
    const QString    axisName = cbAxis->currentData().toString();

    LcncDocument* srcDoc = app()->documentById(srcDocId);
    if (!srcDoc) return;

    // ── Handle "解除已有挂载" — unmount all workpieces ───────────────────
    if (axisName.isEmpty()) {
        // Remove all workpiece entities from machine doc
        TDF_LabelSequence existingWpc = machDoc->entityLabels(LcncDocument::EntityKind::Workpiece);
        QStringList toRemove;
        for (int i = 1; i <= existingWpc.Length(); ++i)
            toRemove << XcafUtils::entry(existingWpc.Value(i));
        for (const QString& e : toRemove) {
            kin->unmountWorkpiece(e);
            machDoc->removeShapeEntity(e);
        }

        if (auto* gd = guiApp()->guiDocument(machDoc->id()))
            gd->rebuildDisplay();
        app()->notifyDocumentModified(machDoc->id());
        context()->updateCommandStates();
        return;
    }

    // ── Merge all workpiece shapes from source doc into one compound ──────
    TDF_LabelSequence srcLabels = srcDoc->entityLabels(LcncDocument::EntityKind::Workpiece);
    Handle(XCAFDoc_ShapeTool) srcSt = srcDoc->shapeTool();

    BRep_Builder bb;
    TopoDS_Compound compound;
    bb.MakeCompound(compound);
    bool hasShape = false;

    for (int i = 1; i <= srcLabels.Length(); ++i) {
        TopoDS_Shape sh = srcSt->GetShape(srcLabels.Value(i));
        if (!sh.IsNull()) {
            bb.Add(compound, sh);
            hasShape = true;
        }
    }

    if (!hasShape) {
        QMessageBox::information(nullptr, tr("挂载工件"),
            tr("源文档中无有效工件形体。"));
        return;
    }

    // Remove any previously mounted workpiece entities from machine doc first
    {
        TDF_LabelSequence existingWpc = machDoc->entityLabels(LcncDocument::EntityKind::Workpiece);
        QStringList toRemove;
        for (int i = 1; i <= existingWpc.Length(); ++i)
            toRemove << XcafUtils::entry(existingWpc.Value(i));
        for (const QString& e : toRemove) {
            kin->unmountWorkpiece(e);
            machDoc->removeShapeEntity(e);
        }
    }

    // Add the merged compound to machine doc as a single Workpiece entity
    const QString wpcName = tr("工件 — %1").arg(srcDoc->name());
    TDF_Label wpcLabel = machDoc->addShapeEntity(compound, wpcName,
                                                  LcncDocument::EntityKind::Workpiece);
    const QString wpcEntry = XcafUtils::entry(wpcLabel);

    // Mount the compound entity to the chosen axis
    kin->mountWorkpiece(wpcEntry, axisName);

    // Rebuild 3D display and refresh
    if (auto* gd = guiApp()->guiDocument(machDoc->id())) {
        gd->rebuildDisplay();
        gd->updateAxisTransforms();
    }

    app()->notifyDocumentModified(machDoc->id());
    context()->updateCommandStates();
}

// ── CmdUnloadMachine ──────────────────────────────────────────────────────────

CmdUnloadMachine::CmdUnloadMachine(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/machine.svg"), tr("卸载机台"), this);
    a->setStatusTip(tr("删除当前机台模型及轴系配置"));
    setAction(a);
}

bool CmdUnloadMachine::isEnabled() const
{
    LcncDocument* doc = context()->machineDocument();
    if (!doc) return false;
    return doc->entityLabels(LcncDocument::EntityKind::Machine).Length() > 0;
}

void CmdUnloadMachine::execute()
{
    LcncDocument* doc = context()->machineDocument();
    if (!doc) return;

    if (QMessageBox::question(nullptr, tr("卸载机台"),
            tr("确定要卸载当前机台模型吗？此操作将清除所有轴系配置。"),
            QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes)
        return;

    // Remove all machine entities
    TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Machine);
    QStringList entries;
    for (int i = 1; i <= labels.Length(); ++i)
        entries << XcafUtils::entry(labels.Value(i));
    for (const QString& e : entries)
        doc->removeShapeEntity(e);

    // Remove any mounted workpiece entities from the machine doc
    {
        TDF_LabelSequence wpcLabels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
        QStringList wpcEntries;
        for (int i = 1; i <= wpcLabels.Length(); ++i)
            wpcEntries << XcafUtils::entry(wpcLabels.Value(i));
        for (const QString& e : wpcEntries)
            doc->removeShapeEntity(e);
    }

    // Reset kinematics
    doc->machineKinematics()->clear();

    // Rebuild display
    if (auto* gd = guiApp()->guiDocument(doc->id()))
        gd->rebuildDisplay();

    app()->notifyDocumentModified(doc->id());
    context()->updateCommandStates();
}

// ── CmdExportMachine ──────────────────────────────────────────────────────────

CmdExportMachine::CmdExportMachine(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/export.svg"), tr("导出机台"), this);
    a->setStatusTip(tr("导出机台模型为 STEP 文件，轴系按 LCNC_AXIS_* 命名以支持自动识别"));
    setAction(a);
}

bool CmdExportMachine::isEnabled() const
{
    LcncDocument* doc = context()->machineDocument();
    if (!doc) return false;
    return doc->entityLabels(LcncDocument::EntityKind::Machine).Length() > 0;
}

void CmdExportMachine::execute()
{
    LcncDocument* machDoc = context()->machineDocument();
    if (!machDoc) return;

    MachineKinematics* kin = machDoc->machineKinematics();

    const QString path = QFileDialog::getSaveFileName(
        nullptr, tr("导出机台模型"), QString(),
        tr("STEP 文件 (*.stp *.step);;所有文件 (*)"));
    if (path.isEmpty()) return;

    // Build a fresh XDE document with per-axis named compound shapes.
    // Each compound uses AddShape(..., Standard_False) so it is NOT split
    // into sub-components — on reimport it appears as a single named entity
    // and its LCNC_AXIS_<name> label triggers autoDetect() recognition.
    Handle(TDocStd_Document) xdeExport =
        new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
    XCAFDoc_DocumentTool::Set(xdeExport->Main());
    Handle(XCAFDoc_ShapeTool) stExp =
        XCAFDoc_DocumentTool::ShapeTool(xdeExport->Main());
    Handle(XCAFDoc_ShapeTool) stMach = machDoc->shapeTool();

    QSet<QString> assignedEntries;

    // One named compound per axis group
    for (const MachineAxisDef& axis : kin->axes()) {
        const QStringList entries = kin->shapesForAxis(axis.name);
        if (entries.isEmpty()) continue;

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

        if (!hasShape) continue;

        const QString axisLabel = QStringLiteral("LCNC_AXIS_") + axis.name;
        TDF_Label lbl = stExp->AddShape(axisCompound, Standard_False);
        TDataStd_Name::Set(lbl,
            TCollection_ExtendedString(axisLabel.toStdString().c_str()));
    }

    // Group unassigned shapes under LCNC_AXIS_UNASSIGNED
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

    // Write via STEPCAFControl_Writer (preserves XDE shape names in STEP)
    STEPCAFControl_Writer writer;
    writer.SetNameMode(Standard_True);
    if (writer.Transfer(xdeExport) != IFSelect_RetDone) {
        QMessageBox::critical(nullptr, tr("导出失败"),
            tr("无法序列化机台模型。"));
        return;
    }
    if (writer.Write(path.toUtf8().constData()) != IFSelect_RetDone)
        QMessageBox::critical(nullptr, tr("导出失败"),
            tr("写入文件失败: %1").arg(path));
    else
        QMessageBox::information(nullptr, tr("导出成功"),
            tr("机台模型已导出到:\n%1").arg(path));
}
