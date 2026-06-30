#include "modules/cam/commands/commands_machine.h"
#include "modules/cam/ui/dialog_mark_axes.h"
#include "app/app_command_context.h"

#include "core/project/project_types.h"
#include "core/document/lcnc_document.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/document/xcaf_utils.h"
#include "view/gui_application.h"
#include "view/gui_document.h"
#include "modules/cam/cam_module.h"

#include <QAction>
#include <QIcon>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QDialog>

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
    auto* a = new QAction(QIcon(":/icons/workpiece.svg"), tr("安装工件"), this);
    a->setStatusTip(tr("将工件源模型平移到当前安装位置"));
    setAction(a);
}

bool CmdMountWorkpiece::isEnabled() const
{
    LcncDocument* workpieceDoc = context()->workpieceDocument();
    return workpieceDoc
        && workpieceDoc->entityLabels(LcncDocument::EntityKind::Workpiece).Length() > 0;
}

void CmdMountWorkpiece::execute()
{
    const auto mountCandidates = context()->camModule()->mountableWorkpieces();
    if (mountCandidates.isEmpty()) {
        QMessageBox::information(nullptr, tr("安装工件"),
            tr("请先导入一个工件模型。"));
        return;
    }

    context()->camModule()->mountWorkpiece(mountCandidates.first().documentId, QString());
    context()->updateCommandStates();
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
