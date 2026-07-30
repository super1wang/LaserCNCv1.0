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
    // 中文翻译：加载机台
    auto* a = new QAction(QIcon("themeicons:machine.svg"), tr("Loading machine"), this);
    // 中文翻译：加载机台三维模型，保留当前轴系配置
    a->setStatusTip(tr("Load the 3D model of the machine and retain the current axis configuration"));
    setAction(a);
}

void CmdLoadMachine::execute()
{
    LcncDocument* doc = context()->machineDocument();
    if (!doc || doc->machineKinematics()->axes().isEmpty()) {
        // 中文翻译：加载机台
        QMessageBox::information(nullptr, tr("Loading machine"),
            // 中文翻译：请先在准备页的轴系配置页面中选择机台构型并完成轴系配置。
            tr("Please first select the machine configuration and complete the axis system configuration on the axis system configuration page of the preparation page."));
        return;
    }

    const QString configuredPath = context()->camModule()->machineModelPath();
    QString pathToLoad;
    if (!configuredPath.isEmpty() && QFileInfo::exists(configuredPath)) {
        pathToLoad = configuredPath;
    } else {
        pathToLoad = QFileDialog::getOpenFileName(
            // 中文翻译：选择机台模型文件
            nullptr, tr("Select machine model file"), configuredPath,
            // 中文翻译：三维模型文件 (*.stp *.step *.stl *.brep);;
            tr("3D model files (*.stp *.step *.stl *.brep);;"
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
    // 中文翻译：标记轴系
    auto* a = new QAction(QIcon("themeicons:coordinate.svg"), tr("Mark axis system"), this);
    // 中文翻译：手动为机台各零件指定所属轴系
    a->setStatusTip(tr("Manually assign the axis system to each part of the machine"));
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
        // 中文翻译：标记轴系
        QMessageBox::information(nullptr, tr("Mark axis system"),
            // 中文翻译：请先在准备页的轴系配置页面中配置机台构型，并加载机台模型。
            tr("Please configure the machine configuration in the axis system configuration page of the preparation page first, and load the machine model."));
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
    // 中文翻译：安装工件
    auto* a = new QAction(QIcon("themeicons:workpiece.svg"), tr("Install workpieces"), this);
    // 中文翻译：将工件源模型平移到当前安装位置
    a->setStatusTip(tr("Translate the workpiece source model to the current installation location"));
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
        // 中文翻译：安装工件
        QMessageBox::information(nullptr, tr("Install workpieces"),
            // 中文翻译：请先导入一个工件模型。
            tr("Please import an workpiece model first."));
        return;
    }

    context()->camModule()->mountWorkpiece(mountCandidates.first().documentId, QString());
    context()->updateCommandStates();
}

// ── CmdUnloadMachine ──────────────────────────────────────────────────────────

CmdUnloadMachine::CmdUnloadMachine(IAppContext* ctx)
    : CommandBase(ctx)
{
    // 中文翻译：卸载机台
    auto* a = new QAction(QIcon("themeicons:machine.svg"), tr("Unload the machine"), this);
    // 中文翻译：删除当前机台参考模型，保留工件、刀路和轴系配置
    a->setStatusTip(tr("Delete the current machine reference model and retain the workpiece, tool path and axis system configuration"));
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
    // 中文翻译：卸载机台
    if (QMessageBox::question(nullptr, tr("Unload the machine"),
            // 中文翻译：确定要卸载当前机台参考模型吗？此操作仅删除机台几何，工件、刀路和轴系配置会保留。
            tr("Are you sure you want to uninstall the current machine reference model? This operation only deletes the machine geometry; the workpiece, tool path and axis configuration will be retained."),
            QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes)
        return;
    context()->camModule()->unloadMachine();
    context()->updateCommandStates();
}

// ── CmdExportMachine ──────────────────────────────────────────────────────────

CmdExportMachine::CmdExportMachine(IAppContext* ctx)
    : CommandBase(ctx)
{
    // 中文翻译：导出机台
    auto* a = new QAction(QIcon("themeicons:export.svg"), tr("Export machine"), this);
    // 中文翻译：导出机台模型为 STEP 文件，轴系按 LCNC_AXIS_* 命名以支持自动识别
    a->setStatusTip(tr("Export the machine model as a STEP file, and name the axis system according to LCNC_AXIS_* to support automatic identification."));
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
        // 中文翻译：导出机台模型
        nullptr, tr("Export machine model"), QString(),
        // 中文翻译：STEP 文件 (*.stp *.step);;所有文件 (*)
        tr("STEP files (*.stp *.step);;all files (*)"));
    if (path.isEmpty()) return;
    context()->camModule()->exportMachine(path);
}
