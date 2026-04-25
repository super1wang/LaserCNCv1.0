#include "modules/cam/commands/commands_machine.h"
#include "modules/cam/ui/dialog_mark_axes.h"
#include "core/command/command_context.h"

#include "core/document/lcnc_application.h"
#include "core/document/lcnc_document.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/algorithms/cam/machine_model_compressor.h"
#include "core/document/xcaf_utils.h"
#include "core/task/task_manager.h"
#include "view/gui_application.h"
#include "view/gui_document.h"
#include "modules/cam/cam_module.h"

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
    a->setStatusTip(tr("加载机台三维模型，保留当前轴系配置"));
    setAction(a);
}

void CmdLoadMachine::execute()
{
    LcncDocument* doc = context()->machineDocument();
    if (!doc || doc->machineKinematics()->axes().isEmpty()) {
        QMessageBox::information(nullptr, tr("加载机台"),
            tr("请先在准备页的轴系配置页面中选择机台构型并完成轴系配置。"));
        return;
    }

    const QString configuredPath = context()->camModule()->machineModelPath();
    QString pathToLoad;
    if (!configuredPath.isEmpty() && QFileInfo::exists(configuredPath)) {
        pathToLoad = configuredPath;
    } else {
        pathToLoad = QFileDialog::getOpenFileName(
            nullptr, tr("选择机台模型文件"), configuredPath,
            tr("三维模型文件 (*.stp *.step *.stl *.brep);;"
               "STEP (*.stp *.step);;"
               "STL (*.stl);;"
               "BREP (*.brep)"));
        if (pathToLoad.isEmpty())
            return;

        context()->camModule()->setMachineModelPath(pathToLoad);
    }

    context()->camModule()->loadMachine(pathToLoad);
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
            tr("请先在准备页的轴系配置页面中配置机台构型，并加载机台模型。"));
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

// ── CmdCompressMachine ───────────────────────────────────────────────────────

CmdCompressMachine::CmdCompressMachine(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/machine.svg"), tr("压缩机台"), this);
    a->setStatusTip(tr("按轴系对机台模型执行独立压缩，并在压缩前选择算法策略"));
    setAction(a);
}

bool CmdCompressMachine::isEnabled() const
{
    LcncDocument* doc = context()->machineDocument();
    if (!doc)
        return false;
    return doc->entityLabels(LcncDocument::EntityKind::Machine).Length() > 0;
}

void CmdCompressMachine::execute()
{
    LcncDocument* doc = context()->machineDocument();
    if (!doc)
        return;

    if (context()->camModule()->axisOptions().isEmpty()) {
        QMessageBox::information(nullptr, tr("压缩机台"),
            tr("请先在准备页配置机台构型。"));
        return;
    }

    QDialog dlg;
    dlg.setWindowTitle(tr("压缩机台"));
    auto* layout = new QVBoxLayout(&dlg);
    auto* form = new QFormLayout;
    auto* combo = new QComboBox(&dlg);
    combo->addItem(tr("实体填充并集（最快，适合先验证外形）"),
                   static_cast<int>(CamModule::MachineCompressionStrategy::FilledSolid));
    combo->addItem(tr("外壳抽取（去内部结构，保留外壳）"),
                   static_cast<int>(CamModule::MachineCompressionStrategy::ExteriorShell));
    combo->addItem(tr("缝合壳体（布尔失败时更稳，结果可能更松散）"),
                   static_cast<int>(CamModule::MachineCompressionStrategy::SewingShell));
    combo->addItem(tr("平衡外观代理（保留大件外形，小件盒化）"),
                   static_cast<int>(CamModule::MachineCompressionStrategy::BoundingBoxProxy));
    form->addRow(tr("压缩策略:"), combo);
    layout->addLayout(form);

    auto* info = new QLabel(
          tr("如果想在体积和外观之间折中，优先试‘平衡外观代理’。\n"
              "它会保留大件外形，把螺钉、附件和碎细节盒化。\n"
              "前面三种策略仍然保留较多解析几何，通常不会显著缩小 STEP 文件。\n"
              "注：网格三角化不是 STEP 的 B-Rep 简化，本次不作为压缩选项。"),
        &dlg);
    info->setWordWrap(true);
    info->setStyleSheet("color: #888; font-size: 11px;");
    layout->addWidget(info);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    layout->addWidget(buttons);

    if (dlg.exec() != QDialog::Accepted)
        return;

    const CamModule::MachineCompressionStrategy strategy =
        static_cast<CamModule::MachineCompressionStrategy>(combo->currentData().toInt());
    context()->camModule()->compressMachineModel(strategy);
}

// ── CmdUnloadMachine ──────────────────────────────────────────────────────────

CmdUnloadMachine::CmdUnloadMachine(IAppContext* ctx)
    : CommandBase(ctx)
{
    auto* a = new QAction(QIcon(":/icons/machine.svg"), tr("卸载机台"), this);
    a->setStatusTip(tr("删除当前机台模型和挂载工件，保留当前轴系配置"));
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
            tr("确定要卸载当前机台模型吗？此操作会删除机台几何和挂载工件，但会保留当前轴系配置。"),
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
