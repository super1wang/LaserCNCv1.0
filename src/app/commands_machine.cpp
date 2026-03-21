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

#include <STEPCAFControl_Reader.hxx>
#include <TCollection_ExtendedString.hxx>
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <StlAPI_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <TDF_LabelSequence.hxx>

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

    // Pre-select config from existing kinematics if already loaded
    if (LcncDocument* existing = context()->activeDocument()) {
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

    // ── Step 3: ensure active document ────────────────────────────────────
    LcncDocument* doc = context()->activeDocument();
    if (!doc) {
        doc = app()->newDocument(fi.baseName());
    }

    // Initialize kinematics preset
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
    LcncDocument* doc = context()->activeDocument();
    if (!doc) return false;
    return doc->entityLabels(LcncDocument::EntityKind::Machine).Length() > 0;
}

void CmdMarkAxes::execute()
{
    LcncDocument* doc = context()->activeDocument();
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
    LcncDocument* doc = context()->activeDocument();
    if (!doc) return false;
    return doc->entityLabels(LcncDocument::EntityKind::Workpiece).Length() > 0
        && doc->entityLabels(LcncDocument::EntityKind::Machine).Length() > 0;
}

void CmdMountWorkpiece::execute()
{
    LcncDocument* doc = context()->activeDocument();
    if (!doc) return;

    MachineKinematics* kin = doc->machineKinematics();
    if (kin->axes().isEmpty()) {
        QMessageBox::information(nullptr, tr("挂载工件"),
            tr("请先加载机台模型并配置轴系。"));
        return;
    }

    // Build workpiece list
    TDF_LabelSequence wpcLabels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    if (wpcLabels.Length() == 0) {
        QMessageBox::information(nullptr, tr("挂载工件"), tr("文档中无工件模型。"));
        return;
    }

    // Build dialog
    QDialog dlg;
    dlg.setWindowTitle(tr("工件挂载"));
    auto* frm = new QFormLayout;

    auto* cbWpc  = new QComboBox;
    auto* cbAxis = new QComboBox;

    for (int i = 1; i <= wpcLabels.Length(); ++i) {
        TDF_Label lbl = wpcLabels.Value(i);
        QString entry = XcafUtils::entry(lbl);
        QString name  = XcafUtils::name(lbl);
        const QString mountedOn = kin->mountedAxis(entry);
        if (mountedOn.isEmpty())
            cbWpc->addItem(name, entry);
        else
            cbWpc->addItem(tr("%1  [→ %2]").arg(name, mountedOn), entry);
    }

    cbAxis->addItem(tr("— 解除挂载 —"), QString());
    for (const auto& axis : kin->axes()) {
        if (axis.name == "BASE") {
            cbAxis->addItem(tr("BASE（固定基座）"), axis.name);
        } else if (axis.motionType == MachineAxisDef::Rotary) {
            cbAxis->addItem(tr("%1 轴（旋转）").arg(axis.name), axis.name);
        } else {
            cbAxis->addItem(tr("%1 轴（线性）").arg(axis.name), axis.name);
        }
    }

    // Pre-select based on existing mount for first item
    const QString firstEntry = cbWpc->itemData(0).toString();
    const QString curAxis    = kin->mountedAxis(firstEntry);
    for (int i = 0; i < cbAxis->count(); ++i) {
        if (cbAxis->itemData(i).toString() == curAxis) {
            cbAxis->setCurrentIndex(i);
            break;
        }
    }

    // Update axis combo when workpiece selection changes
    connect(cbWpc, QOverload<int>::of(&QComboBox::currentIndexChanged), &dlg,
            [cbWpc, cbAxis, kin](int) {
                const QString entry = cbWpc->currentData().toString();
                const QString cur   = kin->mountedAxis(entry);
                for (int i = 0; i < cbAxis->count(); ++i) {
                    if (cbAxis->itemData(i).toString() == cur) {
                        cbAxis->setCurrentIndex(i);
                        return;
                    }
                }
                cbAxis->setCurrentIndex(0);  // not mounted
            });

    frm->addRow(tr("工件:"), cbWpc);
    frm->addRow(tr("挂载到:"), cbAxis);

    auto* btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    auto* vl = new QVBoxLayout(&dlg);
    vl->addLayout(frm);
    vl->addWidget(btns);

    if (dlg.exec() != QDialog::Accepted) return;

    const QString wpcEntry  = cbWpc->currentData().toString();
    const QString axisName  = cbAxis->currentData().toString();

    if (axisName.isEmpty())
        kin->unmountWorkpiece(wpcEntry);
    else
        kin->mountWorkpiece(wpcEntry, axisName);

    // Apply transform immediately
    if (auto* gd = guiApp()->guiDocument(doc->id()))
        gd->updateAxisTransforms();

    app()->notifyDocumentModified(doc->id());
    context()->updateCommandStates();
}
