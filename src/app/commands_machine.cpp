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
#include "modules/cam_module.h"

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

// Bounding box / shape transform (for workpiece auto-snap on mount)
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <cmath>

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
    context()->camModule()->loadMachine(path, configType);
    context()->updateCommandStates();
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

    if (context()->camModule()->axisOptions().isEmpty()) {
        QMessageBox::information(nullptr, tr("标记轴系"),
            tr("请先通过[加载机台]命令选择机台构型并加载模型。"));
        return;
    }

    DialogMarkAxes dlg(doc, doc->machineKinematics(), nullptr);
    if (dlg.exec() == QDialog::Accepted) {
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

    const auto axisOptions = context()->camModule()->axisOptions(true);
    if (axisOptions.size() <= 1) {
        QMessageBox::information(nullptr, tr("挂载工件"),
            tr("请先加载机台模型并配置轴系。"));
        return;
    }

    const auto mountCandidates = context()->camModule()->mountableWorkpieces();
    if (mountCandidates.isEmpty()) {
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

    for (const auto& candidate : mountCandidates)
        cbDoc->addItem(candidate.displayName, candidate.documentId);

    for (const auto& axis : axisOptions)
        cbAxis->addItem(axis.displayName, axis.name);

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

    context()->camModule()->mountWorkpiece(srcDocId, axisName);
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
    if (QMessageBox::question(nullptr, tr("卸载机台"),
            tr("确定要卸载当前机台模型吗？此操作将清除所有轴系配置。"),
            QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes)
        return;
    context()->camModule()->unloadMachine();
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
    const QString path = QFileDialog::getSaveFileName(
        nullptr, tr("导出机台模型"), QString(),
        tr("STEP 文件 (*.stp *.step);;所有文件 (*)"));
    if (path.isEmpty()) return;
    context()->camModule()->exportMachine(path);
}
