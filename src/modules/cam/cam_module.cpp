
#include "modules/cam/cam_module.h"
#include "core/kernel/kernel.h"
#include "modules/cad/services/shape_service.h"

#include "modules/cam/services/cam_config.h"
#include "modules/cam/services/face_classifier.h"
#include "core/document/lcnc_application.h"
#include "core/document/lcnc_document.h"
#include "modules/cam/services/laser_toolpath.h"
#include "core/kinematics/machine_kinematics.h"
#include "modules/cam/services/machine_model_compressor.h"
#include "core/kernel/i_kernel.h"
#include "core/kernel/service_registry.h"
#include "core/logging/logger.h"
#include "core/task/task_manager.h"
#include "core/document/xcaf_utils.h"
#include "view/gui_application.h"
#include "view/gui_document.h"
#include "view/widget_occ_view.h"
#include "view/graphics_scene.h"

#include <QFileInfo>
#include <QPoint>
#include <QSignalBlocker>
#include <QTimer>
#include <QSet>
#include <memory>
#include <stdexcept>
#include <vector>

#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRep_Builder.hxx>
#include <BRepGProp.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepTools.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <StlAPI_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDocStd_Document.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Name.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <Quantity_Color.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <GProp_GProps.hxx>
#include <Precision.hxx>
#include <SelectMgr_EntityOwner.hxx>
#include <SelectMgr_SelectableObject.hxx>
#include <StdSelect_BRepOwner.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax2.hxx>

namespace {

void watchTask(QObject* owner, TaskId taskId, std::function<void(bool)> onFinished)
{
    auto connection = std::make_shared<QMetaObject::Connection>();
    *connection = QObject::connect(
        lcnc::Kernel::current().taskManager(),
        &TaskManager::taskFinished,
        owner,
        [taskId, onFinished = std::move(onFinished), connection](TaskId finishedId, bool success) mutable {
            if (finishedId != taskId)
                return;
            QObject::disconnect(*connection);
            onFinished(success);
        });
}

QSet<QString> toEntrySet(const QStringList& entries)
{
    QSet<QString> result;
    for (const QString& entry : entries)
        result.insert(entry);
    return result;
}

QStringList entityEntries(LcncDocument* doc, LcncDocument::EntityKind kind)
{
    QStringList result;
    if (!doc)
        return result;

    const TDF_LabelSequence labels = doc->entityLabels(kind);
    for (int i = 1; i <= labels.Length(); ++i)
        result << XcafUtils::entry(labels.Value(i));
    return result;
}

MachineModelCompressor::Strategy toCompressorStrategy(CamModule::MachineCompressionStrategy strategy)
{
    switch (strategy) {
    case CamModule::MachineCompressionStrategy::FilledSolid:
        return MachineModelCompressor::Strategy::FilledSolid;
    case CamModule::MachineCompressionStrategy::ExteriorShell:
        return MachineModelCompressor::Strategy::ExteriorShell;
    case CamModule::MachineCompressionStrategy::SewingShell:
        return MachineModelCompressor::Strategy::SewingShell;
    case CamModule::MachineCompressionStrategy::BoundingBoxProxy:
        return MachineModelCompressor::Strategy::BoundingBoxProxy;
    }

    return MachineModelCompressor::Strategy::FilledSolid;
}

QString compressionStrategyText(CamModule::MachineCompressionStrategy strategy)
{
    switch (strategy) {
    case CamModule::MachineCompressionStrategy::FilledSolid:
        return QObject::tr("实体填充并集");
    case CamModule::MachineCompressionStrategy::ExteriorShell:
        return QObject::tr("外壳抽取");
    case CamModule::MachineCompressionStrategy::SewingShell:
        return QObject::tr("缝合壳体");
    case CamModule::MachineCompressionStrategy::BoundingBoxProxy:
        return QObject::tr("外观示意代理");
    }

    return QObject::tr("机台压缩");
}

TopoDS_Shape translatedShapeCopy(const TopoDS_Shape& shape, const gp_Vec& translation)
{
    if (shape.IsNull() || translation.SquareMagnitude() < 1e-12)
        return shape;

    gp_Trsf trsf;
    trsf.SetTranslation(translation);
    BRepBuilderAPI_Transform xform(shape, trsf, Standard_True);
    return xform.IsDone() ? xform.Shape() : shape;
}

gp_Pnt bboxCenter(const Bnd_Box& box)
{
    if (box.IsVoid())
        return gp_Pnt(0.0, 0.0, 0.0);

    Standard_Real xmin = 0.0;
    Standard_Real ymin = 0.0;
    Standard_Real zmin = 0.0;
    Standard_Real xmax = 0.0;
    Standard_Real ymax = 0.0;
    Standard_Real zmax = 0.0;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    return gp_Pnt(0.5 * (xmin + xmax),
                  0.5 * (ymin + ymax),
                  0.5 * (zmin + zmax));
}

gp_Pnt shapeCenter(const TopoDS_Shape& shape)
{
    if (shape.IsNull())
        return gp_Pnt(0.0, 0.0, 0.0);

    Bnd_Box box;
    BRepBndLib::Add(shape, box);
    return bboxCenter(box);
}

} // namespace

// ── IModule ───────────────────────────────────────────────────────────────────────
lcnc::ModuleInfo CamModule::info() const
{
    return {
        QStringLiteral("cam"),
        QStringLiteral("CAM模块"),
        QStringLiteral("1.0.0"),
        { QStringLiteral("cad") }
    };
}

bool CamModule::init(lcnc::IKernel& kernel)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CamModule::init begin");
    auto svc = std::shared_ptr<CamModule>(this, [](CamModule*) {});
    kernel.services().registerService<CamModule>(svc);
    auto facade = std::shared_ptr<lcnc::ICamFacade>(svc, static_cast<lcnc::ICamFacade*>(this));
    kernel.services().registerService<lcnc::ICamFacade>(facade);
    m_initialized = true;
    LCNC_INFO(lcnc::LogCode::Generic, "CamModule init done");
    return true;
}

bool CamModule::start()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CamModule::start (no-op)");
    return true;
}

void CamModule::stop()
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CamModule::stop begin");
    if (!m_initialized) return;
    m_initialized = false;
    LCNC_INFO(lcnc::LogCode::Generic, "CamModule stop done");
}

CamModule::CamModule(QObject* parent)
    : QObject(parent)
{
    // 加载持久化配置（首次启动会自动迁移旧版 CamConfig.json -> cam.toml）。
    m_config.loadDefault();

    lcnc::Kernel::current().app()->ensureMachineDocument();

    CamConfig& config = m_config;
    m_machineModelPath = config.machineModelPath();
    m_machineRenderQuality = config.machineRenderQuality();
    m_toolpath.setGlobalLeadInLength(config.leadInLength());
    m_toolpath.setGlobalNormalAngle(config.normalAngle());
    m_deflection = config.deflection();
    m_smoothAngle = config.smoothAngle();
    m_useFaceClassification = config.useFaceClassification();
    m_showNormals = config.showNormals();
    m_normalSampleStep = config.normalSampleStep();

    m_simTimer = new QTimer(this);
    m_simTimer->setInterval(50);
    connect(m_simTimer, &QTimer::timeout, this, &CamModule::onSimTick);

    connect(lcnc::Kernel::current().app(), &LcncApplication::documentModified,
            this, [this](DocumentId id) {
                if (id == machineDocumentId())
                    emit machineWorkspaceChanged();
            });
}

// ── Machine Document ──────────────────────────────────────────────────────────

LcncDocument* CamModule::machineDocument() const
{
    return lcnc::Kernel::current().app()->machineDocument();
}

GuiDocument* CamModule::machineGuiDocument() const
{
    return lcnc::Kernel::current().guiApp()->machineGuiDocument();
}

DocumentId CamModule::machineDocumentId() const
{
    return lcnc::Kernel::current().app()->machineDocumentId();
}

MachineKinematics* CamModule::kinematics() const
{
    if (auto* doc = machineDocument())
        return doc->machineKinematics();
    return nullptr;
}

void CamModule::requestMachineView()
{
    emit machineViewRequested();
}

QString CamModule::machineModelPath() const
{
    return m_machineModelPath;
}

void CamModule::setMachineModelPath(const QString& filePath)
{
    const QString normalized = filePath.trimmed().isEmpty()
        ? QString()
        : QFileInfo(filePath).absoluteFilePath();
    if (m_machineModelPath == normalized)
        return;

    m_machineModelPath = normalized;
    m_config.setMachineModelPath(m_machineModelPath);
}

MachineRenderQuality CamModule::machineRenderQuality() const
{
    return m_machineRenderQuality;
}

void CamModule::setMachineRenderQuality(MachineRenderQuality quality)
{
    if (m_machineRenderQuality == quality)
        return;

    m_machineRenderQuality = quality;
    m_config.setMachineRenderQuality(quality);

    if (auto* gd = machineGuiDocument())
        gd->setMachineRenderQuality(quality);
}

// ── Machine Management ────────────────────────────────────────────────────────

void CamModule::configureMachine(const QString& presetName)
{
    LcncDocument* doc = machineDocument();
    if (!doc || presetName.isEmpty())
        return;

    MachineKinematics* kin = doc->machineKinematics();
    if (!kin)
        return;

    const bool sameConfig = (kin->configType() == presetName);
    kin->loadPreset(presetName);
    m_config.setMachinePreset(presetName);
    if (m_machineModelPath.isEmpty())
        m_workpieceInstallPosition = defaultWorkpieceInstallPosition();
    else
        applyStoredMachineProfile(m_machineModelPath);

    if (!sameConfig)
        clearToolpath();

    displayAxisGuides();
    refreshMachineTransforms();
    emit axisAssignmentsChanged();
    lcnc::Kernel::current().app()->notifyDocumentModified(machineDocumentId());
}

void CamModule::loadMachine(const QString& filePath)
{
    LcncDocument* doc = machineDocument();
    if (!doc || filePath.isEmpty()) return;

    QFileInfo fi(filePath);
    if (!fi.exists())
        return;

    const QString normalizedPath = fi.absoluteFilePath();
    const QString ext = fi.suffix().toLower();

    // Clear previous machine entities
    {
        TDF_LabelSequence existing = doc->entityLabels(LcncDocument::EntityKind::Machine);
        QStringList entriesToRemove;
        for (int i = 1; i <= existing.Length(); ++i)
            entriesToRemove << XcafUtils::entry(existing.Value(i));
        for (const QString& e : entriesToRemove)
            ShapeService::deleteShape(doc, e);
    }

    // Clear mounted workpieces as well
    unmountAllWorkpieces();

    TaskId taskId = lcnc::Kernel::current().taskManager()->run(tr("加载机台: %1").arg(fi.fileName()),
        [filePath, ext, doc](TaskProgress* prog) {
            prog->setRange(0, 100);

            if (ext == "stp" || ext == "step") {
                prog->setStepName(QStringLiteral("读取 STEP..."));
                Handle(TDocStd_Document) xdeDoc =
                    new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
                XCAFDoc_DocumentTool::Set(xdeDoc->Main());
                STEPCAFControl_Reader cafReader;
                cafReader.SetNameMode(Standard_True);
                if (cafReader.ReadFile(filePath.toUtf8().constData()) == IFSelect_RetDone) {
                    prog->setValue(50);
                    prog->setStepName(QStringLiteral("转换形体..."));
                    cafReader.Transfer(xdeDoc);
                    prog->setValue(80);
                    doc->importFromXcafRoots(xdeDoc, LcncDocument::EntityKind::Machine);
                }
            } else if (ext == "stl") {
                prog->setStepName(QStringLiteral("读取 STL..."));
                TopoDS_Shape shape;
                StlAPI_Reader stlReader;
                stlReader.Read(shape, filePath.toUtf8().constData());
                prog->setValue(80);
                if (!shape.IsNull())
                    doc->addShapeEntity(shape, QFileInfo(filePath).baseName(), LcncDocument::EntityKind::Machine);
            } else if (ext == "brep") {
                prog->setStepName(QStringLiteral("读取 BREP..."));
                TopoDS_Shape shape;
                BRep_Builder builder;
                BRepTools::Read(shape, filePath.toUtf8().constData(), builder);
                if (!shape.IsNull())
                    doc->addShapeEntity(shape, QFileInfo(filePath).baseName(), LcncDocument::EntityKind::Machine);
            }

            prog->setValue(100);
        });

    watchTask(this, taskId, [this, normalizedPath](bool ok) {
        if (!ok)
            return;

        m_machineModelPath = normalizedPath;
        m_config.setMachineModelPath(m_machineModelPath);
        autoDetectAxes();
        applyStoredMachineProfile(m_machineModelPath);
        refreshMachineDisplay();
        emit machineLoaded();
    });
}

void CamModule::unloadMachine()
{
    LcncDocument* doc = machineDocument();
    if (!doc) return;

    // Remove machine entities
    TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Machine);
    QStringList entries;
    for (int i = 1; i <= labels.Length(); ++i)
        entries << XcafUtils::entry(labels.Value(i));
    for (const QString& e : entries)
        ShapeService::deleteShape(doc, e);

    // Remove mounted workpieces
    unmountAllWorkpieces();

    clearToolpath();
    refreshMachineDisplay();
    emit machineUnloaded();
}

void CamModule::exportMachine(const QString& filePath)
{
    LcncDocument* machDoc = machineDocument();
    if (!machDoc || filePath.isEmpty()) return;

    MachineKinematics* kin = machDoc->machineKinematics();

    Handle(TDocStd_Document) xdeExport =
        new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
    XCAFDoc_DocumentTool::Set(xdeExport->Main());
    Handle(XCAFDoc_ShapeTool) stExp = XCAFDoc_DocumentTool::ShapeTool(xdeExport->Main());
    Handle(XCAFDoc_ShapeTool) stMach = machDoc->shapeTool();

    QSet<QString> assignedEntries;

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
        TDataStd_Name::Set(lbl, TCollection_ExtendedString(axisLabel.toStdString().c_str()));
    }

    // Unassigned group
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
    if (writer.Transfer(xdeExport) != IFSelect_RetDone) return;
    writer.Write(filePath.toUtf8().constData());
}

void CamModule::autoDetectAxes()
{
    LcncDocument* doc = machineDocument();
    if (!doc) return;

    TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Machine);
    QMap<QString, QString> entryToName;
    for (int i = 1; i <= labels.Length(); ++i) {
        TDF_Label lbl = labels.Value(i);
        entryToName.insert(XcafUtils::entry(lbl), XcafUtils::name(lbl));
    }
    doc->machineKinematics()->autoDetect(entryToName);
    autoDetectAxisOrigins();
    applyStoredMachineProfile(m_machineModelPath);
    if (auto* gd = machineGuiDocument())
        gd->applyMachineDisplayStyle();
    refreshMachineTransforms();
    emit axisAssignmentsChanged();
    lcnc::Kernel::current().app()->notifyDocumentModified(machineDocumentId());
}

void CamModule::applyAxisAssignments(const QMap<QString, QString>& entryToAxis)
{
    LcncDocument* doc = machineDocument();
    if (!doc || entryToAxis.isEmpty())
        return;

    MachineKinematics* kin = doc->machineKinematics();
    for (auto it = entryToAxis.cbegin(); it != entryToAxis.cend(); ++it) {
        if (it.value().isEmpty())
            kin->unassignShape(it.key());
        else
            kin->assignShape(it.key(), it.value());
    }

    if (auto* gd = machineGuiDocument())
        gd->applyMachineDisplayStyle();
    refreshMachineTransforms();
    emit axisAssignmentsChanged();
    lcnc::Kernel::current().app()->notifyDocumentModified(machineDocumentId());
}

void CamModule::assignShapesToAxis(const QStringList& entries, const QString& axisName)
{
    if (entries.isEmpty() || axisName.isEmpty())
        return;

    QMap<QString, QString> entryToAxis;
    for (const QString& entry : entries)
        entryToAxis.insert(entry, axisName);
    applyAxisAssignments(entryToAxis);
}

void CamModule::unassignShape(const QString& entry)
{
    if (entry.isEmpty())
        return;

    QMap<QString, QString> entryToAxis;
    entryToAxis.insert(entry, QString());
    applyAxisAssignments(entryToAxis);
}

void CamModule::clearAxisAssignments(const QString& axisName)
{
    MachineKinematics* kin = kinematics();
    if (!kin || axisName.isEmpty())
        return;

    QMap<QString, QString> entryToAxis;
    for (const QString& entry : kin->shapesForAxis(axisName))
        entryToAxis.insert(entry, QString());
    applyAxisAssignments(entryToAxis);
}

QList<CamModule::AxisOption> CamModule::axisOptions(bool includeDetachOption) const
{
    QList<AxisOption> result;
    MachineKinematics* kin = kinematics();
    if (!kin)
        return result;

    if (includeDetachOption)
        result.append({QString(), tr("— 解除已有挂载 —")});

    for (const MachineAxisDef& axis : kin->axes()) {
        QString displayName;
        if (axis.name == QStringLiteral("BASE")) {
            displayName = tr("BASE（固定基座）");
        } else if (axis.motionType == MachineAxisDef::Rotary) {
            displayName = tr("%1 轴（旋转）").arg(axis.name);
        } else {
            displayName = tr("%1 轴（线性）").arg(axis.name);
        }
        result.append({axis.name, displayName});
    }

    return result;
}

gp_Pnt CamModule::axisOrigin(const QString& axisName) const
{
    if (MachineKinematics* kin = kinematics())
        return kin->axisOrigin(axisName);
    return gp_Pnt(0, 0, 0);
}

void CamModule::setAxisOrigin(const QString& axisName, const gp_Pnt& origin)
{
    MachineKinematics* kin = kinematics();
    if (!kin)
        return;

    const gp_Pnt current = kin->axisOrigin(axisName);
    if (current.SquareDistance(origin) < 1e-12)
        return;

    if (!kin->setAxisOrigin(axisName, origin))
        return;

    if (!m_machineModelPath.isEmpty())
        m_config.setAxisOriginForMachine(m_machineModelPath, axisName, origin);

    displayAxisGuides();
    refreshMachineTransforms();
    lcnc::Kernel::current().app()->notifyDocumentModified(machineDocumentId());
}

bool CamModule::setAxisLimits(const QString& axisName, double minVal, double maxVal)
{
    MachineKinematics* kin = kinematics();
    if (!kin)
        return false;

    if (!kin->setAxisLimits(axisName, minVal, maxVal))
        return false;

    displayAxisGuides();
    refreshMachineTransforms();
    lcnc::Kernel::current().app()->notifyDocumentModified(machineDocumentId());
    return true;
}

bool CamModule::supportsAcCenterCalibration() const
{
    return ensureAcCenterCalibrationAvailable();
}

bool CamModule::currentAcRotationCenter(gp_Pnt& center) const
{
    if (!ensureAcCenterCalibrationAvailable())
        return false;

    const MachineKinematics* kin = kinematics();
    if (!kin)
        return false;

    const gp_Pnt aOrigin = kin->axisOrigin(QStringLiteral("A"));
    const gp_Pnt cOrigin = kin->axisOrigin(QStringLiteral("C"));
    center = gp_Pnt(cOrigin.X(), aOrigin.Y(), aOrigin.Z());
    return true;
}

bool CamModule::currentWorkpieceRotationCenter(gp_Pnt& center) const
{
    const MachineKinematics* kin = kinematics();
    if (!kin)
        return false;

    const QString configType = kin->configType();
    if (configType == QStringLiteral("VERTICAL_AC_TABLE"))
        return currentAcRotationCenter(center);

    if (configType == QStringLiteral("VERTICAL_BC_TABLE")) {
        if (!kin->findAxis(QStringLiteral("B")) || !kin->findAxis(QStringLiteral("C")))
            return false;

        const gp_Pnt bOrigin = kin->axisOrigin(QStringLiteral("B"));
        const gp_Pnt cOrigin = kin->axisOrigin(QStringLiteral("C"));
        center = gp_Pnt(bOrigin.X(), cOrigin.Y(), bOrigin.Z());
        return true;
    }

    if (configType == QStringLiteral("XYZA")) {
        if (!kin->findAxis(QStringLiteral("A")))
            return false;

        center = kin->axisOrigin(QStringLiteral("A"));
        return true;
    }

    return false;
}

gp_Pnt CamModule::cutterHeadModelPosition() const
{
    return m_cutterHeadModelPosition;
}

gp_Pnt CamModule::cutterHeadPhysicalPosition() const
{
    return m_cutterHeadPhysicalPosition;
}

void CamModule::setCutterHeadModelPosition(const gp_Pnt& position)
{
    if (m_cutterHeadModelPosition.SquareDistance(position) < 1e-12)
        return;

    m_cutterHeadModelPosition = position;
    if (!m_machineModelPath.isEmpty()) {
        m_config.setCutterHeadModelPositionForMachine(
            m_machineModelPath,
            m_cutterHeadModelPosition);
    }
    displayAxisGuides();
    if (auto* gd = machineGuiDocument()) {
        if (gd->hasView())
            gd->view()->Redraw();
    }
    emit machineWorkspaceChanged();
}

void CamModule::setCutterHeadPhysicalPosition(const gp_Pnt& position)
{
    if (m_cutterHeadPhysicalPosition.SquareDistance(position) < 1e-12)
        return;

    m_cutterHeadPhysicalPosition = position;
    if (!m_machineModelPath.isEmpty()) {
        m_config.setCutterHeadPhysicalPositionForMachine(
            m_machineModelPath,
            m_cutterHeadPhysicalPosition);
    }
    emit machineWorkspaceChanged();
}

bool CamModule::fillAxisOriginFromReferenceFace(WidgetOccView* occView,
                                                const QPoint& screenPos,
                                                const QString& axisName)
{
    QString errorMessage;
    if (!ensureAcCenterCalibrationAvailable(&errorMessage)) {
        emit operationFailed(tr("轴心快速填充"), errorMessage);
        return false;
    }

    const QString normalizedAxis = axisName.trimmed().toUpper();
    if (normalizedAxis != QStringLiteral("A") && normalizedAxis != QStringLiteral("C")) {
        emit operationFailed(tr("轴心快速填充"),
                             tr("当前快速填充仅支持 A 轴和 C 轴。"));
        return false;
    }

    gp_Pnt faceCenter;
    if (!resolveReferencePlaneCenter(occView, screenPos, faceCenter, &errorMessage)) {
        emit operationFailed(tr("轴心快速填充"), errorMessage);
        return false;
    }

    MachineKinematics* kin = kinematics();
    if (!kin) {
        emit operationFailed(tr("轴心快速填充"), tr("找不到机台轴系配置。"));
        return false;
    }

    if (normalizedAxis == QStringLiteral("A")) {
        const gp_Pnt current = kin->axisOrigin(QStringLiteral("A"));
        setAxisOrigin(QStringLiteral("A"), gp_Pnt(current.X(), faceCenter.Y(), faceCenter.Z()));
    } else {
        const gp_Pnt current = kin->axisOrigin(QStringLiteral("C"));
        setAxisOrigin(QStringLiteral("C"), gp_Pnt(faceCenter.X(), current.Y(), current.Z()));
    }

    if (hasToolpath())
        recalcToolpath();

    return true;
}

bool CamModule::setCutterHeadModelPositionFromReferenceFace(WidgetOccView* occView,
                                                            const QPoint& screenPos)
{
    gp_Pnt faceCenter;
    QString errorMessage;
    if (!resolveReferencePlaneCenter(occView, screenPos, faceCenter, &errorMessage)) {
        emit operationFailed(tr("切割头对齐"), errorMessage);
        return false;
    }

    setCutterHeadModelPosition(faceCenter);
    return true;
}

bool CamModule::alignMachineToPhysicalCenter(const gp_Pnt& physicalCenter)
{
    QString errorMessage;
    if (!ensureAcCenterCalibrationAvailable(&errorMessage)) {
        emit operationFailed(tr("机台坐标系转换"), errorMessage);
        return false;
    }

    gp_Pnt currentCenter;
    if (!currentAcRotationCenter(currentCenter)) {
        emit operationFailed(tr("机台坐标系转换"), tr("无法计算当前模型 AC 中心。"));
        return false;
    }

    const gp_Vec translation(currentCenter, physicalCenter);
    if (translation.SquareMagnitude() < 1e-12)
        return true;

    return translateMachineWorkspace(translation, tr("机台坐标系转换"));
}

bool CamModule::alignMachineToPhysicalCutterHead()
{
    const gp_Vec translation(m_cutterHeadModelPosition, m_cutterHeadPhysicalPosition);
    if (translation.SquareMagnitude() < 1e-12)
        return true;

    return translateMachineWorkspace(translation, tr("切割头物理对齐"));
}

bool CamModule::translateMachineWorkspace(const gp_Vec& translation, const QString& operationTitle)
{
    if (translation.SquareMagnitude() < 1e-12)
        return true;

    LcncDocument* doc = machineDocument();
    MachineKinematics* kin = kinematics();
    if (!doc || !kin) {
        emit operationFailed(operationTitle, tr("找不到机台文档或轴系配置。"));
        return false;
    }

    const TDF_LabelSequence machineLabels = doc->entityLabels(LcncDocument::EntityKind::Machine);
    const TDF_LabelSequence workpieceLabels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    const QList<MachineAxisDef> axisSnapshot = kin->axes();
    const gp_Pnt cutterHeadSnapshot = m_cutterHeadModelPosition;
    QList<TDF_Label> movedLabels;

    auto moveLabels = [&](const TDF_LabelSequence& labels) {
        for (int i = 1; i <= labels.Length(); ++i) {
            const TDF_Label label = labels.Value(i);
            if (label.IsNull())
                continue;

            if (!ShapeService::moveShape(doc, label, translation))
                return false;

            movedLabels.append(label);
        }
        return true;
    };

    auto rollback = [&]() {
        const gp_Vec reverse(-translation.X(), -translation.Y(), -translation.Z());
        for (int i = movedLabels.size() - 1; i >= 0; --i)
            ShapeService::moveShape(doc, movedLabels.at(i), reverse);

        for (const MachineAxisDef& axis : axisSnapshot)
            kin->setAxisOrigin(axis.name, axis.origin);

        m_cutterHeadModelPosition = cutterHeadSnapshot;
    };

    if (!moveLabels(machineLabels) || !moveLabels(workpieceLabels)) {
        rollback();
        emit operationFailed(operationTitle, tr("整机平移失败，当前模型已恢复原始位置。"));
        return false;
    }

    for (const MachineAxisDef& axis : axisSnapshot) {
        const gp_Pnt shiftedOrigin = axis.origin.Translated(translation);
        if (!kin->setAxisOrigin(axis.name, shiftedOrigin)) {
            rollback();
            emit operationFailed(operationTitle, tr("轴心平移失败，当前模型已恢复原始位置。"));
            return false;
        }
    }

    m_cutterHeadModelPosition.Translate(translation);
    m_workpieceInstallPosition.Translate(translation);
    if (!m_machineModelPath.isEmpty()) {
        CamConfig& config = m_config;
        for (const MachineAxisDef& axis : kin->axes())
            config.setAxisOriginForMachine(m_machineModelPath, axis.name, axis.origin);
        config.setCutterHeadModelPositionForMachine(m_machineModelPath, m_cutterHeadModelPosition);
        config.setWorkpieceInstallPositionForMachine(m_machineModelPath, m_workpieceInstallPosition);
    }
    translateToolpathWorldData(translation);
    refreshMachineDisplay();
    if (hasToolpath() || m_previewLeadInValid)
        refreshToolpathDisplay();

    emit machineWorkspaceChanged();
    return true;
}

bool CamModule::compressMachineModel(MachineCompressionStrategy strategy)
{
    LcncDocument* doc = machineDocument();
    MachineKinematics* kin = kinematics();
    if (!doc || !kin) {
        emit operationFailed(tr("压缩机台"), tr("找不到机台文档或轴系配置。"));
        return false;
    }

    if (m_machineCompressionRunning) {
        emit operationFailed(tr("压缩机台"), tr("机台压缩任务正在执行，请稍候。"));
        return false;
    }

    TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Machine);
    if (labels.Length() == 0) {
        emit operationFailed(tr("压缩机台"), tr("当前没有可压缩的机台模型。"));
        return false;
    }

    QMap<QString, TDF_Label> labelByEntry;
    QStringList oldEntries;
    for (int i = 1; i <= labels.Length(); ++i) {
        const TDF_Label lbl = labels.Value(i);
        const QString entry = XcafUtils::entry(lbl);
        labelByEntry.insert(entry, lbl);
        oldEntries.append(entry);
    }

    QList<MachineModelCompressor::GroupInput> groups;
    auto collectGroup = [&](const QString& axisName,
                            const QString& displayName,
                            const QStringList& entries) {
        if (entries.isEmpty())
            return;

        QList<TopoDS_Shape> shapes;
        shapes.reserve(entries.size());
        for (const QString& entry : entries) {
            const auto it = labelByEntry.constFind(entry);
            if (it == labelByEntry.cend())
                continue;

            const TopoDS_Shape shape = XcafUtils::shape(it.value());
            if (shape.IsNull())
                continue;

            shapes.push_back(shape);
        }

        if (shapes.empty())
            return;

        groups.append({axisName, displayName, shapes});
    };

    for (const MachineAxisDef& axis : kin->axes()) {
        collectGroup(axis.name,
                     QStringLiteral("LCNC_AXIS_%1").arg(axis.name),
                     kin->shapesForAxis(axis.name));
    }

    QStringList unassignedEntries;
    for (const QString& entry : oldEntries) {
        if (kin->axisForShape(entry).isEmpty())
            unassignedEntries.append(entry);
    }
    collectGroup(QString(), QStringLiteral("LCNC_AXIS_UNASSIGNED"), unassignedEntries);

    if (groups.isEmpty()) {
        emit operationFailed(tr("压缩机台"), tr("没有收集到可压缩的机台形体。"));
        return false;
    }

    auto resultHolder = std::make_shared<MachineModelCompressor::Result>();
    auto errorMessage = std::make_shared<QString>();
    const QStringList snapshotEntries = oldEntries;
    const QString strategyName = compressionStrategyText(strategy);
    const MachineModelCompressor::Options options{toCompressorStrategy(strategy)};

    m_machineCompressionRunning = true;
    const TaskId taskId = lcnc::Kernel::current().taskManager()->run(
        tr("压缩机台模型 - %1").arg(strategyName),
        [groups, options, resultHolder, errorMessage](TaskProgress* progress) {
            *resultHolder = MachineModelCompressor::compressGroups(
                groups,
                options,
                progress);

            if (resultHolder->aborted) {
                *errorMessage = resultHolder->error.isEmpty()
                    ? QStringLiteral("机台压缩已中止，当前模型未修改。")
                    : resultHolder->error;
                throw std::runtime_error("machine compression aborted");
            }

            if (!resultHolder->success() || resultHolder->groups.isEmpty()) {
                *errorMessage = resultHolder->error.isEmpty()
                    ? QStringLiteral("机台压缩失败，当前模型已保持不变。")
                    : resultHolder->error;
                throw std::runtime_error("machine compression failed");
            }
        });

    watchTask(this, taskId, [this, doc, kin, snapshotEntries, resultHolder, errorMessage](bool success) {
        m_machineCompressionRunning = false;

        if (!success) {
            emit operationFailed(tr("压缩机台"),
                                 errorMessage->isEmpty()
                                     ? tr("机台压缩失败，当前模型已保持不变。")
                                     : *errorMessage);
            return;
        }

        if (!doc || !kin) {
            emit operationFailed(tr("压缩机台"), tr("压缩完成后找不到机台文档，结果未写回。"));
            return;
        }

        TDF_LabelSequence currentLabels = doc->entityLabels(LcncDocument::EntityKind::Machine);
        QStringList currentEntries;
        for (int i = 1; i <= currentLabels.Length(); ++i)
            currentEntries.append(XcafUtils::entry(currentLabels.Value(i)));

        if (toEntrySet(currentEntries) != toEntrySet(snapshotEntries)) {
            emit operationFailed(tr("压缩机台"),
                                 tr("机台模型在压缩期间已发生变化，压缩结果已丢弃，请重新执行压缩。"));
            return;
        }

        {
            QSignalBlocker blocker(kin);
            for (const QString& entry : snapshotEntries)
                kin->unassignShape(entry);
        }

        for (const QString& entry : snapshotEntries)
            doc->removeShapeEntity(entry);

        {
            QSignalBlocker blocker(kin);
            for (const MachineModelCompressor::GroupOutput& group : resultHolder->groups) {
                const TDF_Label lbl = doc->addShapeEntity(group.shape,
                                                          group.displayName,
                                                          LcncDocument::EntityKind::Machine);
                if (!group.axisName.isEmpty())
                    kin->assignShape(XcafUtils::entry(lbl), group.axisName);
            }
        }

        refreshMachineDisplay();
        emit axisAssignmentsChanged();
    });

    return true;
}

QList<CamModule::WorkpieceMountCandidate> CamModule::mountableWorkpieces() const
{
    QList<WorkpieceMountCandidate> result;
    const QList<LcncDocument*> docs = lcnc::Kernel::current().app()->workpieceDocuments();
    for (LcncDocument* doc : docs) {
        if (!doc)
            continue;

        const int workpieceCount = doc->entityLabels(LcncDocument::EntityKind::Workpiece).Length();
        if (workpieceCount <= 0)
            continue;

        result.append({
            doc->id(),
            tr("%1  (%2 形体)").arg(doc->name()).arg(workpieceCount),
            workpieceCount,
        });
    }

    return result;
}

gp_Pnt CamModule::workpieceInstallPosition() const
{
    return m_workpieceInstallPosition;
}

void CamModule::setWorkpieceInstallPosition(const gp_Pnt& position)
{
    if (m_workpieceInstallPosition.SquareDistance(position) < 1e-12)
        return;

    LcncDocument* doc = machineDocument();
    const gp_Pnt previousPosition = m_workpieceInstallPosition;
    const gp_Vec translation(previousPosition, position);
    bool movedWorkpieces = false;

    if (doc && translation.SquareMagnitude() > 1e-12) {
        const TDF_LabelSequence workpieceLabels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
        if (workpieceLabels.Length() > 0) {
            QList<TDF_Label> movedLabels;
            for (int i = 1; i <= workpieceLabels.Length(); ++i) {
                const TDF_Label label = workpieceLabels.Value(i);
                if (label.IsNull())
                    continue;

                if (!ShapeService::moveShape(doc, label, translation)) {
                    const gp_Vec rollback(-translation.X(), -translation.Y(), -translation.Z());
                    for (int index = movedLabels.size() - 1; index >= 0; --index)
                        ShapeService::moveShape(doc, movedLabels.at(index), rollback);

                    emit operationFailed(tr("工件安装位置"), tr("更新工件安装位置失败，当前挂载位置未修改。"));
                    return;
                }

                movedLabels.append(label);
            }

            movedWorkpieces = !movedLabels.isEmpty();
            if (movedWorkpieces) {
                translateToolpathWorldData(translation);
                updateToolpathMachineCoordinates();
            }
        }
    }

    m_workpieceInstallPosition = position;
    if (!m_machineModelPath.isEmpty()) {
        m_config.setWorkpieceInstallPositionForMachine(
            m_machineModelPath,
            m_workpieceInstallPosition);
    }

    if (movedWorkpieces) {
        refreshMachineDisplay();
        if (hasToolpath() || m_previewLeadInValid)
            refreshToolpathDisplay();
        return;
    }

    emit machineWorkspaceChanged();
}

bool CamModule::supportsWorkpieceRotationAlignment() const
{
    gp_Pnt center;
    return currentWorkpieceRotationCenter(center);
}

bool CamModule::alignWorkpieceInstallPositionToRotationCenter()
{
    gp_Pnt center;
    if (!currentWorkpieceRotationCenter(center)) {
        emit operationFailed(tr("工件安装位置"), tr("当前构型没有可用于对齐的工件旋转中心。"));
        return false;
    }

    gp_Pnt alignedPosition = m_workpieceInstallPosition;
    alignedPosition.SetX(center.X());
    alignedPosition.SetY(center.Y());
    setWorkpieceInstallPosition(alignedPosition);
    return true;
}

void CamModule::autoDetectAxisOrigins()
{
    LcncDocument* doc = machineDocument();
    MachineKinematics* kin = kinematics();
    if (!doc || !kin)
        return;

    TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Machine);
    QMap<QString, TDF_Label> labelByEntry;
    for (int i = 1; i <= labels.Length(); ++i) {
        const TDF_Label lbl = labels.Value(i);
        labelByEntry.insert(XcafUtils::entry(lbl), lbl);
    }

    for (const MachineAxisDef& axis : kin->axes()) {
        if (axis.name == QStringLiteral("BASE") || axis.motionType != MachineAxisDef::Rotary)
            continue;

        Bnd_Box bbox;
        bool hasShape = false;
        for (const QString& entry : kin->shapesForAxis(axis.name)) {
            const auto it = labelByEntry.constFind(entry);
            if (it == labelByEntry.cend())
                continue;

            const TopoDS_Shape shape = XcafUtils::shape(it.value());
            if (shape.IsNull())
                continue;

            BRepBndLib::Add(shape, bbox);
            hasShape = true;
        }

        if (!hasShape || bbox.IsVoid())
            continue;

        Standard_Real xmin = 0.0;
        Standard_Real ymin = 0.0;
        Standard_Real zmin = 0.0;
        Standard_Real xmax = 0.0;
        Standard_Real ymax = 0.0;
        Standard_Real zmax = 0.0;
        bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        kin->setAxisOrigin(axis.name,
                           gp_Pnt(0.5 * (xmin + xmax),
                                  0.5 * (ymin + ymax),
                                  0.5 * (zmin + zmax)));
    }
}

void CamModule::applyStoredMachineProfile(const QString& machinePath)
{
    MachineKinematics* kin = kinematics();
    if (!kin || machinePath.isEmpty())
        return;

    CamConfig& config = m_config;
    for (const MachineAxisDef& axis : kin->axes()) {
        gp_Pnt storedOrigin;
        if (config.axisOriginForMachine(machinePath, axis.name, &storedOrigin))
            kin->setAxisOrigin(axis.name, storedOrigin);
    }

    m_workpieceInstallPosition = defaultWorkpieceInstallPosition();

    gp_Pnt storedPosition;
    if (config.cutterHeadModelPositionForMachine(machinePath, &storedPosition))
        m_cutterHeadModelPosition = storedPosition;

    if (config.cutterHeadPhysicalPositionForMachine(machinePath, &storedPosition))
        m_cutterHeadPhysicalPosition = storedPosition;

    if (config.workpieceInstallPositionForMachine(machinePath, &storedPosition))
        m_workpieceInstallPosition = storedPosition;
}

gp_Pnt CamModule::defaultWorkpieceInstallPosition() const
{
    gp_Pnt center;
    if (currentWorkpieceRotationCenter(center))
        return gp_Pnt(center.X(), center.Y(), 0.0);

    return gp_Pnt(0.0, 0.0, 0.0);
}

// ── Workpiece Mounting ────────────────────────────────────────────────────────

void CamModule::mountWorkpiece(DocumentId sourceDocId, const QString& axisName)
{
    LcncDocument* machDoc = machineDocument();
    LcncDocument* srcDoc = lcnc::Kernel::current().app()->documentById(sourceDocId);
    if (!machDoc || !srcDoc) return;

    MachineKinematics* kin = machDoc->machineKinematics();
    if (kin->axes().isEmpty()) return;

    if (axisName.isEmpty()) {
        unmountAllWorkpieces();
        return;
    }

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
    if (!hasShape) return;

    const QStringList existingWorkpieces = entityEntries(machDoc, LcncDocument::EntityKind::Workpiece);
    if (auto* gd = machineGuiDocument()) {
        for (const QString& entry : existingWorkpieces)
            gd->eraseEntity(entry);
    }
    for (const QString& entry : existingWorkpieces)
        ShapeService::deleteShape(machDoc, entry);

    clearToolpath();

    TopoDS_Shape mountShape = compound;
    const gp_Pnt currentCenter = shapeCenter(compound);
    const gp_Vec placement(currentCenter, m_workpieceInstallPosition);
    if (placement.SquareMagnitude() > 1e-12) {
        gp_Trsf trsf;
        trsf.SetTranslation(placement);
        BRepBuilderAPI_Transform xform(compound, trsf, Standard_True);
        if (xform.IsDone())
            mountShape = xform.Shape();
    }

    const QString wpcName = tr("工件 — %1").arg(srcDoc->name());
    TDF_Label wpcLabel = machDoc->addShapeEntity(mountShape, wpcName,
                                                 LcncDocument::EntityKind::Workpiece);
    const QString wpcEntry = XcafUtils::entry(wpcLabel);
    kin->mountWorkpiece(wpcEntry, axisName);

    if (auto* gd = machineGuiDocument())
        gd->displayShape(mountShape, wpcEntry);

    refreshMachineTransforms();
    lcnc::Kernel::current().app()->notifyDocumentModified(machineDocumentId());
    emit workpieceMounted(wpcEntry);
}

void CamModule::unmountAllWorkpieces()
{
    LcncDocument* machDoc = machineDocument();
    if (!machDoc) return;

    const QStringList toRemove = entityEntries(machDoc, LcncDocument::EntityKind::Workpiece);
    if (auto* gd = machineGuiDocument()) {
        for (const QString& entry : toRemove)
            gd->eraseEntity(entry);
    }
    for (const QString& entry : toRemove)
        ShapeService::deleteShape(machDoc, entry);

    const bool hadWorkpieces = !toRemove.isEmpty();
    const bool hadToolpath = hasToolpath();
    clearToolpath();

    if (hadWorkpieces || hadToolpath)
        refreshMachineTransforms();

    if (hadWorkpieces) {
        lcnc::Kernel::current().app()->notifyDocumentModified(machineDocumentId());
        emit workpieceUnmounted();
    }
}

// ── Shape Operations ──────────────────────────────────────────────────────────

bool CamModule::moveShape(const QString& entry, const gp_Vec& translation)
{
    LcncDocument* doc = machineDocument();
    if (!doc || entry.isEmpty()) return false;

    TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Machine);
    for (int i = 1; i <= labels.Length(); ++i) {
        if (XcafUtils::entry(labels.Value(i)) == entry) {
            bool ok = ShapeService::moveShape(doc, labels.Value(i), translation);
            if (ok) refreshMachineDisplay();
            return ok;
        }
    }

    TDF_LabelSequence wpcLabels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    for (int i = 1; i <= wpcLabels.Length(); ++i) {
        if (XcafUtils::entry(wpcLabels.Value(i)) == entry) {
            bool ok = ShapeService::moveShape(doc, wpcLabels.Value(i), translation);
            if (ok) refreshMachineDisplay();
            return ok;
        }
    }
    return false;
}

bool CamModule::rotateShape(const QString& entry, const gp_Ax1& axis, double angleDeg)
{
    LcncDocument* doc = machineDocument();
    if (!doc || entry.isEmpty()) return false;

    TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Machine);
    for (int i = 1; i <= labels.Length(); ++i) {
        if (XcafUtils::entry(labels.Value(i)) == entry) {
            bool ok = ShapeService::rotateShape(doc, labels.Value(i), axis, angleDeg);
            if (ok) refreshMachineDisplay();
            return ok;
        }
    }

    TDF_LabelSequence wpcLabels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    for (int i = 1; i <= wpcLabels.Length(); ++i) {
        if (XcafUtils::entry(wpcLabels.Value(i)) == entry) {
            bool ok = ShapeService::rotateShape(doc, wpcLabels.Value(i), axis, angleDeg);
            if (ok) refreshMachineDisplay();
            return ok;
        }
    }
    return false;
}

void CamModule::deleteShape(const QString& entry)
{
    LcncDocument* doc = machineDocument();
    if (!doc || entry.isEmpty()) return;

    if (auto* gd = machineGuiDocument())
        gd->eraseEntity(entry);

    ShapeService::deleteShape(doc, entry);
    refreshMachineTransforms();
    lcnc::Kernel::current().app()->notifyDocumentModified(machineDocumentId());
}

// ── Toolpath ──────────────────────────────────────────────────────────────────

TopoDS_Shape CamModule::collectWorkpieceShape() const
{
    const QList<WorkpieceShapeSource> sources = collectWorkpieceShapes();
    if (sources.isEmpty())
        return {};

    if (sources.size() == 1)
        return sources.first().shape;

    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    for (const WorkpieceShapeSource& source : sources) {
        if (!source.shape.IsNull())
            builder.Add(compound, source.shape);
    }
    return compound;
}

QList<CamModule::WorkpieceShapeSource> CamModule::collectWorkpieceShapes() const
{
    LcncDocument* doc = machineDocument();
    QList<WorkpieceShapeSource> result;
    if (!doc)
        return result;

    TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    if (labels.Length() == 0)
        return result;

    Handle(XCAFDoc_ShapeTool) st = doc->shapeTool();
    for (int i = 1; i <= labels.Length(); ++i) {
        const TDF_Label label = labels.Value(i);
        const QString workpieceEntry = XcafUtils::entry(label);
        const TopoDS_Shape shape = st->GetShape(label);
        if (shape.IsNull())
            continue;

        bool expanded = false;
        if (shape.ShapeType() == TopAbs_COMPOUND || shape.ShapeType() == TopAbs_COMPSOLID) {
            int componentIndex = 0;
            for (TopoDS_Iterator it(shape, Standard_True, Standard_True); it.More(); it.Next()) {
                const TopoDS_Shape child = it.Value();
                if (child.IsNull())
                    continue;

                result.append({workpieceEntry, child, componentIndex});
                ++componentIndex;
                expanded = true;
            }
        }

        if (!expanded)
            result.append({workpieceEntry, shape, 0});
    }

    return result;
}

bool CamModule::generateToolpath(double smoothAngle, bool useFaceClassification, double deflection)
{
    const QList<WorkpieceShapeSource> workpieceSources = collectWorkpieceShapes();
    if (workpieceSources.isEmpty())
        return false;

    clearToolpath();
    m_workpieceShape = collectWorkpieceShape();
    m_smoothAngle = smoothAngle;
    m_useFaceClassification = useFaceClassification;
    m_deflection = deflection;

    ContourExtractionParams params;
    params.smoothAngleThresholdDeg = smoothAngle;
    params.useFaceClassification = useFaceClassification;
    params.deflection = deflection;

    std::vector<LaserContour> allContours;

    LcncDocument* machDoc = machineDocument();
    MachineKinematics* kin = machDoc ? machDoc->machineKinematics() : nullptr;

    for (const WorkpieceShapeSource& source : workpieceSources) {
        if (source.shape.IsNull())
            continue;

        auto contours = LaserToolpathBuilder::extractContours(source.shape, params);
        if (contours.empty())
            continue;

        const gp_Trsf wpcTrsf = (kin && !source.workpieceEntry.isEmpty())
            ? kin->computeWpcTransform(source.workpieceEntry)
            : gp_Trsf();

        for (auto& contour : contours) {
            contour.workpieceEntry = source.workpieceEntry;
            contour.sourceShape = source.shape;
            if (workpieceSources.size() > 1) {
                contour.sourceInfo = contour.sourceInfo.isEmpty()
                    ? tr("工件源 #%1").arg(source.componentIndex + 1)
                    : tr("%1 · 工件源 #%2").arg(contour.sourceInfo).arg(source.componentIndex + 1);
            }

            if (contour.points.empty())
                LaserToolpathBuilder::discretizeContour(contour, source.shape, deflection);

            LaserToolpathBuilder::computeMachineCoordinates(contour, kin, wpcTrsf);
            allContours.push_back(std::move(contour));
        }
    }

    if (allContours.empty())
        return false;

    m_toolpath.contours() = std::move(allContours);
    m_toolpathVisible = true;

    refreshToolpathDisplay();
    emit toolpathGenerated();
    return true;
}

void CamModule::clearToolpath()
{
    eraseToolpathDisplay();
    m_toolpath.clear();
    m_workpieceShape.Nullify();
    m_previewLeadInContour = -1;
    m_previewLeadInParam = 0.0;
    m_previewLeadInValid = false;
    emit toolpathCleared();
}

bool CamModule::resolveReferencePlaneCenter(WidgetOccView* occView,
                                            const QPoint& screenPos,
                                            gp_Pnt& center,
                                            QString* errorMessage) const
{
    if (!occView || occView->view().IsNull() || occView->context().IsNull()) {
        if (errorMessage)
            *errorMessage = tr("当前没有可用的机台视图用于参考面拾取。");
        return false;
    }

    const Handle(AIS_InteractiveContext)& context = occView->context();
    context->MoveTo(screenPos.x(), screenPos.y(), occView->view(), Standard_False);

    const Handle(SelectMgr_EntityOwner) owner = context->DetectedOwner();
    Handle(StdSelect_BRepOwner) brepOwner = Handle(StdSelect_BRepOwner)::DownCast(owner);
    if (brepOwner.IsNull() || !brepOwner->HasShape()) {
        if (errorMessage)
            *errorMessage = tr("请将光标放在机台模型或挂载工件的平面上。\n当前未检测到可用参考面。");
        return false;
    }

    const TopoDS_Shape pickedShape = brepOwner->Shape();
    if (pickedShape.IsNull() || pickedShape.ShapeType() != TopAbs_FACE) {
        if (errorMessage)
            *errorMessage = tr("当前拾取的不是平面面片，请重新选择参考平面。");
        return false;
    }

    const TopoDS_Face face = TopoDS::Face(pickedShape);
    const BRepAdaptor_Surface surface(face);
    if (surface.GetType() != GeomAbs_Plane) {
        if (errorMessage)
            *errorMessage = tr("当前拾取的面不是平面，请选择平面参考面。");
        return false;
    }

    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    center = props.CentreOfMass();

    if (owner->HasSelectable() && !owner->Selectable().IsNull())
        center.Transform(owner->Selectable()->LocalTransformation());

    return true;
}

bool CamModule::ensureAcCenterCalibrationAvailable(QString* errorMessage) const
{
    const MachineKinematics* kin = kinematics();
    if (!kin) {
        if (errorMessage)
            *errorMessage = tr("找不到机台轴系配置。请先选择 AC 转台构型。");
        return false;
    }

    if (kin->configType() != QStringLiteral("VERTICAL_AC_TABLE")) {
        if (errorMessage)
            *errorMessage = tr("当前仅 VERTICAL_AC_TABLE 构型支持 A/C 轴心快填与 AC 中心平移对齐。");
        return false;
    }

    if (!kin->findAxis(QStringLiteral("A")) || !kin->findAxis(QStringLiteral("C"))) {
        if (errorMessage)
            *errorMessage = tr("当前 AC 转台轴定义不完整，缺少 A 轴或 C 轴。\n请先完成机台构型配置。");
        return false;
    }

    return true;
}

void CamModule::translateToolpathWorldData(const gp_Vec& translation)
{
    if (translation.SquareMagnitude() < 1e-12)
        return;

    for (LaserContour& contour : m_toolpath.contours()) {
        const TopoDS_Shape movedWire = translatedShapeCopy(contour.wire, translation);
        if (!movedWire.IsNull() && movedWire.ShapeType() == TopAbs_WIRE)
            contour.wire = TopoDS::Wire(movedWire);
        contour.sourceShape = translatedShapeCopy(contour.sourceShape, translation);

        for (ToolpathPoint& point : contour.points)
            point.position.Translate(translation);

        if (contour.leadIn.valid)
            contour.leadIn.entryPoint.Translate(translation);
    }

    m_workpieceShape = translatedShapeCopy(m_workpieceShape, translation);

    if (m_previewLeadInValid)
        m_previewLeadInPoint.Translate(translation);
}

void CamModule::updateToolpathMachineCoordinates()
{
    LcncDocument* machDoc = machineDocument();
    MachineKinematics* kin = machDoc ? machDoc->machineKinematics() : nullptr;
    if (!kin)
        return;

    for (LaserContour& contour : m_toolpath.contours()) {
        const gp_Trsf wpcTrsf = !contour.workpieceEntry.isEmpty()
            ? kin->computeWpcTransform(contour.workpieceEntry)
            : gp_Trsf();
        LaserToolpathBuilder::computeMachineCoordinates(contour, kin, wpcTrsf);
    }
}

const LaserToolpath& CamModule::toolpath() const
{
    return m_toolpath;
}

LaserToolpath& CamModule::toolpathRef()
{
    return m_toolpath;
}

bool CamModule::hasToolpath() const
{
    return m_toolpath.contourCount() > 0;
}

void CamModule::setLeadInEntry(int contourIdx, const gp_Pnt& entryPoint, double entryParam)
{
    if (contourIdx < 0 || contourIdx >= m_toolpath.contourCount()) return;

    LaserContour& c = m_toolpath.contour(contourIdx);
    c.leadIn.entryPoint = entryPoint;
    c.leadIn.entryParam = entryParam;
    c.leadIn.valid = true;

    refreshToolpathDisplay();
}

void CamModule::setLeadInLength(double mm)
{
    m_toolpath.setGlobalLeadInLength(mm);
    if (m_toolpathVisible)
        refreshToolpathDisplay();
}

double CamModule::leadInLength() const
{
    return m_toolpath.globalLeadInLength();
}

void CamModule::setNormalAngle(double deg)
{
    m_toolpath.setGlobalNormalAngle(deg);
    if (m_toolpathVisible)
        refreshToolpathDisplay();
}

double CamModule::normalAngle() const
{
    return m_toolpath.globalNormalAngle();
}

void CamModule::setDeflection(double mm)
{
    if (mm <= 0.0)
        return;

    m_deflection = mm;
}

double CamModule::deflection() const
{
    return m_deflection;
}

bool CamModule::showNormals() const
{
    return m_showNormals;
}

void CamModule::setShowNormals(bool on)
{
    if (m_showNormals == on)
        return;

    m_showNormals = on;
    refreshToolpathDisplay();
}

double CamModule::normalSampleStep() const
{
    return m_normalSampleStep;
}

void CamModule::setNormalSampleStep(double mm)
{
    if (mm <= 0.0)
        return;

    if (qFuzzyCompare(m_normalSampleStep + 1.0, mm + 1.0))
        return;

    m_normalSampleStep = mm;
    if (m_showNormals)
        refreshToolpathDisplay();
}

bool CamModule::resolveLeadInHit(WidgetOccView* occView,
                                 const QPoint& screenPos,
                                 int& contourIdx,
                                 gp_Pnt& entryPoint,
                                 double& entryParam) const
{
    if (!occView || occView->view().IsNull())
        return false;

    const Handle(V3d_View)& view = occView->view();
    constexpr double kMaxScreenDistanceSq = 24.0 * 24.0;
    double bestDistanceSq = kMaxScreenDistanceSq;
    int bestContour = -1;
    int bestPoint = -1;

    for (int contourIndex = 0; contourIndex < m_toolpath.contourCount(); ++contourIndex) {
        const LaserContour& contour = m_toolpath.contour(contourIndex);
        if (!contour.enabled)
            continue;

        for (int pointIndex = 0; pointIndex < static_cast<int>(contour.points.size()); ++pointIndex) {
            const ToolpathPoint& point = contour.points[pointIndex];
            Standard_Integer px = 0;
            Standard_Integer py = 0;
            view->Convert(point.position.X(), point.position.Y(), point.position.Z(), px, py);

            const double dx = static_cast<double>(px - screenPos.x());
            const double dy = static_cast<double>(py - screenPos.y());
            const double distanceSq = dx * dx + dy * dy;
            if (distanceSq > bestDistanceSq)
                continue;

            bestDistanceSq = distanceSq;
            bestContour = contourIndex;
            bestPoint = pointIndex;
        }
    }

    if (bestContour < 0 || bestPoint < 0)
        return false;

    const ToolpathPoint& hitPoint = m_toolpath.contour(bestContour).points[bestPoint];
    contourIdx = bestContour;
    entryPoint = hitPoint.position;
    entryParam = hitPoint.param;
    return true;
}

bool CamModule::updateLeadInPreview(WidgetOccView* occView, const QPoint& screenPos)
{
    int contourIdx = -1;
    gp_Pnt entryPoint;
    double entryParam = 0.0;

    if (!resolveLeadInHit(occView, screenPos, contourIdx, entryPoint, entryParam)) {
        if (m_previewLeadInValid)
            cancelLeadInPreview();
        return false;
    }

    if (m_previewLeadInValid
        && m_previewLeadInContour == contourIdx
        && m_previewLeadInPoint.Distance(entryPoint) < 1e-6
        && std::abs(m_previewLeadInParam - entryParam) < 1e-6) {
        return true;
    }

    m_previewLeadInContour = contourIdx;
    m_previewLeadInPoint = entryPoint;
    m_previewLeadInParam = entryParam;
    m_previewLeadInValid = true;
    refreshToolpathDisplay();
    return true;
}

bool CamModule::commitLeadInPreview(WidgetOccView* occView, const QPoint& screenPos)
{
    if (!updateLeadInPreview(occView, screenPos) || !m_previewLeadInValid)
        return false;

    if (m_previewLeadInContour < 0 || m_previewLeadInContour >= m_toolpath.contourCount())
        return false;

    LaserContour& contour = m_toolpath.contour(m_previewLeadInContour);
    contour.leadIn.entryPoint = m_previewLeadInPoint;
    contour.leadIn.entryParam = m_previewLeadInParam;
    contour.leadIn.valid = true;

    m_previewLeadInContour = -1;
    m_previewLeadInParam = 0.0;
    m_previewLeadInValid = false;
    refreshToolpathDisplay();
    return true;
}

void CamModule::cancelLeadInPreview()
{
    if (!m_previewLeadInValid)
        return;

    m_previewLeadInContour = -1;
    m_previewLeadInParam = 0.0;
    m_previewLeadInValid = false;
    refreshToolpathDisplay();
}

void CamModule::setContourEnabled(int contourIdx, bool enabled)
{
    if (contourIdx < 0 || contourIdx >= m_toolpath.contourCount())
        return;

    LaserContour& contour = m_toolpath.contour(contourIdx);
    if (contour.enabled == enabled)
        return;

    contour.enabled = enabled;
    if (m_toolpathVisible)
        refreshToolpathDisplay();
}

void CamModule::reorderContours(const QList<int>& order)
{
    if (order.size() != m_toolpath.contourCount())
        return;

    QSet<int> seen;
    std::vector<LaserContour> current = std::move(m_toolpath.contours());
    std::vector<LaserContour> reordered;
    reordered.reserve(current.size());

    for (int index : order) {
        if (index < 0 || index >= static_cast<int>(current.size()) || seen.contains(index)) {
            m_toolpath.contours() = std::move(current);
            return;
        }

        seen.insert(index);
        reordered.push_back(std::move(current[index]));
    }

    m_toolpath.contours() = std::move(reordered);
}

void CamModule::recalcToolpath()
{
    LcncDocument* machDoc = machineDocument();
    MachineKinematics* kin = machDoc ? machDoc->machineKinematics() : nullptr;

    for (int i = 0; i < m_toolpath.contourCount(); ++i) {
        LaserContour& contour = m_toolpath.contour(i);
        const TopoDS_Shape sourceShape = contour.sourceShape.IsNull()
            ? m_workpieceShape
            : contour.sourceShape;
        if (sourceShape.IsNull())
            continue;

        if (m_useFaceClassification) {
            FaceClassification classification =
                FaceClassifier::classifyFaces(sourceShape, m_smoothAngle);

            std::vector<TopoDS_Face> outerFaces;
            std::vector<TopoDS_Face> crossFaces;
            if (classification.outerGroup())
                outerFaces = classification.outerGroup()->faces;
            for (const auto* group : classification.crossSectionGroups()) {
                if (!group)
                    continue;
                for (const auto& face : group->faces)
                    crossFaces.push_back(face);
            }

            if (!outerFaces.empty() && !crossFaces.empty())
                LaserToolpathBuilder::discretizeContourWithClassification(
                    contour, outerFaces, crossFaces, m_deflection);
            else
                LaserToolpathBuilder::discretizeContour(contour, sourceShape, m_deflection);
        } else {
            LaserToolpathBuilder::discretizeContour(contour, sourceShape, m_deflection);
        }

        const gp_Trsf wpcTrsf = (kin && !contour.workpieceEntry.isEmpty())
            ? kin->computeWpcTransform(contour.workpieceEntry)
            : gp_Trsf();
        LaserToolpathBuilder::computeMachineCoordinates(contour, kin, wpcTrsf);
    }

    refreshToolpathDisplay();
}

void CamModule::setToolpathVisible(bool visible)
{
    if (m_toolpathVisible == visible) return;
    m_toolpathVisible = visible;

    GuiDocument* gd = machineGuiDocument();
    if (!gd) return;
    const Handle(AIS_InteractiveContext)& aisCtx = gd->context();
    if (aisCtx.IsNull()) return;

    auto toggleList = [&](QList<Handle(AIS_Shape)>& list) {
        for (auto& ais : list) {
            if (ais.IsNull()) continue;
            if (m_toolpathVisible)
                aisCtx->Display(ais, Standard_False);
            else
                aisCtx->Erase(ais, Standard_False);
        }
    };

    toggleList(m_contourAis);
    toggleList(m_leadInAis);
    toggleList(m_normalAis);

    if (gd->hasView())
        gd->view()->Redraw();

    emit toolpathVisibilityChanged(visible);
}

bool CamModule::isToolpathVisible() const
{
    return m_toolpathVisible;
}

double CamModule::smoothAngle() const
{
    return m_smoothAngle;
}

void CamModule::setSmoothAngle(double deg)
{
    m_smoothAngle = deg;
}

bool CamModule::useFaceClassification() const
{
    return m_useFaceClassification;
}

void CamModule::setUseFaceClassification(bool on)
{
    m_useFaceClassification = on;
}

void CamModule::displayContours()
{
    GuiDocument* gd = machineGuiDocument();
    if (!gd) return;
    GraphicsScene* scene = gd->scene();
    const Handle(AIS_InteractiveContext)& context = gd->context();
    const Quantity_Color green(0.1, 0.8, 0.2, Quantity_TOC_RGB);

    for (int i = 0; i < m_toolpath.contourCount(); ++i) {
        const LaserContour& c = m_toolpath.contour(i);
        if (!c.enabled || c.wire.IsNull()) continue;

        Handle(AIS_Shape) ais = scene->displayShape(c.wire, false, true);
        scene->setShapeColor(ais, green);
        if (!context.IsNull())
            context->Deactivate(ais);
        m_contourAis.append(ais);
    }
}

void CamModule::displayLeadIns()
{
    GuiDocument* gd = machineGuiDocument();
    if (!gd) return;
    GraphicsScene* scene = gd->scene();
    const Handle(AIS_InteractiveContext)& context = gd->context();
    const Quantity_Color red(0.9, 0.15, 0.15, Quantity_TOC_RGB);
    const Quantity_Color yellow(0.95, 0.8, 0.1, Quantity_TOC_RGB);

    const double length = m_toolpath.globalLeadInLength();
    const double angle = m_toolpath.globalNormalAngle();

    for (int i = 0; i < m_toolpath.contourCount(); ++i) {
        const LaserContour& c = m_toolpath.contour(i);
        if (!c.enabled || !c.leadIn.valid) continue;

        TopoDS_Edge leadEdge = LaserToolpathBuilder::computeLeadInEdge(c, length, angle);
        if (leadEdge.IsNull()) continue;

        Handle(AIS_Shape) ais = scene->displayShape(leadEdge, false, false);
        scene->setShapeColor(ais, red);
        ais->SetWidth(2.0);
        if (!context.IsNull())
            context->Deactivate(ais);
        m_leadInAis.append(ais);
    }

    if (m_previewLeadInValid
        && m_previewLeadInContour >= 0
        && m_previewLeadInContour < m_toolpath.contourCount()) {
        const LaserContour& sourceContour = m_toolpath.contour(m_previewLeadInContour);
        if (sourceContour.enabled) {
            LaserContour previewContour = sourceContour;
            previewContour.leadIn.entryPoint = m_previewLeadInPoint;
            previewContour.leadIn.entryParam = m_previewLeadInParam;
            previewContour.leadIn.valid = true;

            TopoDS_Edge previewEdge =
                LaserToolpathBuilder::computeLeadInEdge(previewContour, length, angle);
            if (!previewEdge.IsNull()) {
                Handle(AIS_Shape) ais = scene->displayShape(previewEdge, false, false);
                scene->setShapeColor(ais, yellow);
                ais->SetWidth(2.5);
                if (!context.IsNull())
                    context->Deactivate(ais);
                m_leadInAis.append(ais);
            }
        }
    }
}

void CamModule::displayNormals()
{
    if (!m_showNormals)
        return;

    GuiDocument* gd = machineGuiDocument();
    if (!gd)
        return;

    GraphicsScene* scene = gd->scene();
    const Handle(AIS_InteractiveContext)& context = gd->context();
    const Quantity_Color amber(0.95, 0.55, 0.10, Quantity_TOC_RGB);
    constexpr double kNormalLength = 5.0;

    for (int contourIndex = 0; contourIndex < m_toolpath.contourCount(); ++contourIndex) {
        const LaserContour& contour = m_toolpath.contour(contourIndex);
        if (!contour.enabled || contour.points.empty())
            continue;

        BRep_Builder builder;
        TopoDS_Compound compound;
        builder.MakeCompound(compound);
        bool hasSegments = false;
        double accumulatedDistance = 0.0;
        gp_Pnt previousPoint;
        bool hasPreviousPoint = false;

        for (const ToolpathPoint& point : contour.points) {
            if (hasPreviousPoint)
                accumulatedDistance += previousPoint.Distance(point.position);

            const bool shouldEmit = !hasPreviousPoint || accumulatedDistance >= m_normalSampleStep;
            if (shouldEmit) {
                const gp_Pnt endPoint(
                    point.position.X() + point.normal.X() * kNormalLength,
                    point.position.Y() + point.normal.Y() * kNormalLength,
                    point.position.Z() + point.normal.Z() * kNormalLength);
                BRepBuilderAPI_MakeEdge edgeMaker(point.position, endPoint);
                if (edgeMaker.IsDone()) {
                    builder.Add(compound, edgeMaker.Edge());
                    hasSegments = true;
                }
                accumulatedDistance = 0.0;
            }

            previousPoint = point.position;
            hasPreviousPoint = true;
        }

        if (!hasSegments)
            continue;

        Handle(AIS_Shape) ais = scene->displayShape(compound, false, false);
        scene->setShapeColor(ais, amber);
        ais->SetWidth(1.5);
        if (!context.IsNull())
            context->Deactivate(ais);
        m_normalAis.append(ais);
    }
}

void CamModule::eraseAxisGuideDisplay()
{
    GuiDocument* gd = machineGuiDocument();
    if (!gd)
        return;

    GraphicsScene* scene = gd->scene();
    if (!scene)
        return;

    for (auto& ais : m_axisGuideAis) {
        if (!ais.IsNull())
            scene->eraseObject(ais, false);
    }
    m_axisGuideAis.clear();
}

void CamModule::displayAxisGuides()
{
    eraseAxisGuideDisplay();

    GuiDocument* gd = machineGuiDocument();
    MachineKinematics* kin = kinematics();
    if (!gd || !kin)
        return;

    GraphicsScene* scene = gd->scene();
    const Handle(AIS_InteractiveContext)& context = gd->context();
    if (!scene || context.IsNull())
        return;

    auto axisColor = [](const QString& axisName) {
        if (axisName == QStringLiteral("A"))
            return Quantity_Color(0.95, 0.55, 0.10, Quantity_TOC_RGB);
        if (axisName == QStringLiteral("C"))
            return Quantity_Color(0.90, 0.20, 0.90, Quantity_TOC_RGB);
        return Quantity_Color(0.20, 0.45, 0.95, Quantity_TOC_RGB);
    };

    const QStringList visibleAxes = {QStringLiteral("A"), QStringLiteral("C")};
    for (const QString& axisName : visibleAxes) {
        const MachineAxisDef* axisDef = kin->findAxis(axisName);
        if (!axisDef)
            continue;

        const gp_Vec axisVector(axisDef->direction);
        constexpr double kRotaryGuideHalfLength = 50.0;
        const gp_Pnt p0 = axisDef->origin.Translated(axisVector * -kRotaryGuideHalfLength);
        const gp_Pnt p1 = axisDef->origin.Translated(axisVector * kRotaryGuideHalfLength);

        BRepBuilderAPI_MakeEdge edgeMaker(p0, p1);
        if (!edgeMaker.IsDone())
            continue;

        Handle(AIS_Shape) ais = scene->displayShape(edgeMaker.Edge(), false, false, false);
        scene->setShapeColor(ais, axisColor(axisName), false);
        ais->SetWidth(3.0);
        context->Deactivate(ais);
        m_axisGuideAis.insert(QStringLiteral("axis:%1").arg(axisName), ais);
    }

    constexpr double kHeadGuideLength = 80.0;
    constexpr double kHeadConeHeight = 18.0;
    constexpr double kHeadConeRadius = 5.0;
    const gp_Pnt headTip = m_cutterHeadModelPosition;
    const gp_Pnt headTop = headTip.Translated(gp_Vec(0.0, 0.0, kHeadGuideLength));

    BRepBuilderAPI_MakeEdge headAxisMaker(headTop, headTip);
    if (headAxisMaker.IsDone()) {
        Handle(AIS_Shape) axisAis = scene->displayShape(headAxisMaker.Edge(), false, false, false);
        scene->setShapeColor(axisAis, axisColor(QString()), false);
        axisAis->SetWidth(2.5);
        context->Deactivate(axisAis);
        m_axisGuideAis.insert(QStringLiteral("head:axis"), axisAis);
    }

    gp_Ax2 coneAxis(headTip.Translated(gp_Vec(0.0, 0.0, kHeadConeHeight)), gp_Dir(0.0, 0.0, -1.0));
    BRepPrimAPI_MakeCone coneMaker(coneAxis, kHeadConeRadius, 0.0, kHeadConeHeight);
    if (coneMaker.IsDone()) {
        Handle(AIS_Shape) coneAis = scene->displayShape(coneMaker.Shape(), false, false, false);
        scene->setShapeColor(coneAis, Quantity_Color(0.15, 0.55, 0.95, Quantity_TOC_RGB), false);
        context->Deactivate(coneAis);
        m_axisGuideAis.insert(QStringLiteral("head:cone"), coneAis);
    }

    updateAxisGuideTransforms();
}

void CamModule::updateAxisGuideTransforms()
{
    GuiDocument* gd = machineGuiDocument();
    MachineKinematics* kin = kinematics();
    if (!gd || !kin)
        return;

    const Handle(AIS_InteractiveContext)& context = gd->context();
    if (context.IsNull())
        return;

    auto applyTransform = [&](const QString& key, const gp_Trsf& trsf) {
        const auto it = m_axisGuideAis.constFind(key);
        if (it == m_axisGuideAis.cend() || it.value().IsNull())
            return;

        it.value()->SetLocalTransformation(trsf);
        context->RecomputePrsOnly(it.value(), Standard_False);
    };

    applyTransform(QStringLiteral("axis:A"), kin->computeAxisTransform(QStringLiteral("A")));
    applyTransform(QStringLiteral("axis:C"), kin->computeAxisTransform(QStringLiteral("C")));

    const gp_Trsf headTransform = kin->computeAxisTransform(QStringLiteral("Z"));
    applyTransform(QStringLiteral("head:axis"), headTransform);
    applyTransform(QStringLiteral("head:cone"), headTransform);
}

void CamModule::refreshToolpathDisplay()
{
    eraseToolpathDisplay();
    if (m_toolpathVisible) {
        displayContours();
        displayLeadIns();
        displayNormals();
    }

    if (auto* gd = machineGuiDocument()) {
        if (gd->hasView())
            gd->view()->Redraw();
    }
}

void CamModule::eraseToolpathDisplay()
{
    GuiDocument* gd = machineGuiDocument();
    if (!gd) return;
    GraphicsScene* scene = gd->scene();

    for (auto& ais : m_contourAis)
        if (!ais.IsNull()) scene->eraseShape(ais);
    for (auto& ais : m_leadInAis)
        if (!ais.IsNull()) scene->eraseShape(ais);
    for (auto& ais : m_normalAis)
        if (!ais.IsNull()) scene->eraseShape(ais);

    m_contourAis.clear();
    m_leadInAis.clear();
    m_normalAis.clear();
}

const QList<Handle(AIS_Shape)>& CamModule::contourAis() const
{
    return m_contourAis;
}

// ── Simulation ────────────────────────────────────────────────────────────────

void CamModule::simulatePlay()
{
    if (m_simPaused) {
        m_simPaused = false;
        m_simPlaying = true;
        m_simTimer->start();
        emit simulationStateChanged(true);
        return;
    }

    m_simCurrentContour = 0;
    m_simCurrentPoint = 0;
    m_simTotalPoints = 0;
    for (int i = 0; i < m_toolpath.contourCount(); ++i) {
        const auto& c = m_toolpath.contour(i);
        if (c.enabled)
            m_simTotalPoints += static_cast<int>(c.points.size());
    }
    if (m_simTotalPoints == 0) return;

    while (m_simCurrentContour < m_toolpath.contourCount()
           && !m_toolpath.contour(m_simCurrentContour).enabled)
        ++m_simCurrentContour;

    m_simPlaying = true;
    m_simPaused = false;
    m_simTimer->setInterval(std::max(10, static_cast<int>(50.0 / m_simSpeed)));
    m_simTimer->start();
    emit simulationStateChanged(true);
}

void CamModule::simulatePause()
{
    m_simTimer->stop();
    m_simPaused = true;
    m_simPlaying = false;
    emit simulationStateChanged(false);
}

void CamModule::simulateStop()
{
    m_simTimer->stop();
    m_simPlaying = false;
    m_simPaused = false;
    m_simCurrentContour = 0;
    m_simCurrentPoint = 0;
    emit simulationStateChanged(false);
    emit simulationFinished();
}

void CamModule::setSimulationSpeed(double factor)
{
    m_simSpeed = (factor > 0.1) ? factor : 0.1;
    if (m_simTimer->isActive())
        m_simTimer->setInterval(std::max(10, static_cast<int>(50.0 / m_simSpeed)));
}

bool CamModule::isSimulating() const
{
    return m_simPlaying;
}

bool CamModule::isSimPaused() const
{
    return m_simPaused;
}

void CamModule::setAxisPosition(const QString& axisName, double value, bool refreshNow)
{
    if (auto* kin = kinematics()) {
        kin->setAxisPosition(axisName, value);
        if (refreshNow)
            refreshMachineTransforms();
    }
}

void CamModule::setEntityVisible(const QString& entry, bool visible)
{
    if (entry.isEmpty())
        return;

    if (auto* gd = machineGuiDocument()) {
        Handle(AIS_Shape) ais = gd->aisShape(entry);
        if (ais.IsNull())
            return;

        if (visible)
            gd->scene()->displayObject(ais);
        else
            gd->scene()->eraseObject(ais);

        if (gd->hasView())
            gd->view()->Redraw();
    }
}

void CamModule::setSelectedEntries(const QStringList& entries)
{
    GuiDocument* gd = machineGuiDocument();
    if (!gd)
        return;

    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull())
        return;

    ctx->ClearSelected(false);
    for (const QString& entry : entries) {
        Handle(AIS_Shape) ais = gd->aisShape(entry);
        if (!ais.IsNull())
            ctx->AddOrRemoveSelected(ais, false);
    }

    if (gd->hasView())
        gd->view()->Redraw();

    emit selectionChanged(gd->selectedEntries());
}

QStringList CamModule::selectedEntries() const
{
    if (auto* gd = machineGuiDocument())
        return gd->selectedEntries();

    return {};
}

void CamModule::syncSelectionFromView()
{
    emit selectionChanged(selectedEntries());
}

void CamModule::onSimTick()
{
    if (m_simCurrentContour >= m_toolpath.contourCount()) {
        simulateStop();
        return;
    }

    const LaserContour& c = m_toolpath.contour(m_simCurrentContour);
    if (m_simCurrentPoint >= static_cast<int>(c.points.size())) {
        ++m_simCurrentContour;
        m_simCurrentPoint = 0;
        while (m_simCurrentContour < m_toolpath.contourCount()
               && !m_toolpath.contour(m_simCurrentContour).enabled)
            ++m_simCurrentContour;
        if (m_simCurrentContour >= m_toolpath.contourCount()) {
            simulateStop();
            return;
        }
        return;
    }

    const MachineCoord& mc = c.points[m_simCurrentPoint].machineCoord;
    if (!mc.valid) {
        ++m_simCurrentPoint;
        return;
    }

    if (kinematics()) {
        setAxisPosition("X", mc.x, false);
        setAxisPosition("Y", mc.y, false);
        setAxisPosition("Z", mc.z, false);
        if (!mc.r1Name.isEmpty())
            setAxisPosition(mc.r1Name, mc.r1, false);
        if (!mc.r2Name.isEmpty())
            setAxisPosition(mc.r2Name, mc.r2, false);

        refreshMachineTransforms();
    }

    emit simulationTick(m_simCurrentContour, m_simCurrentPoint, m_simTotalPoints);
    ++m_simCurrentPoint;
}

void CamModule::refreshMachineTransforms()
{
    if (auto* gd = machineGuiDocument()) {
        gd->updateAxisTransforms();
        updateAxisGuideTransforms();
        if (gd->hasView())
            gd->view()->Redraw();
    }
}

void CamModule::refreshMachineDisplay()
{
    if (auto* gd = machineGuiDocument()) {
        if (gd->machineRenderQuality() != m_machineRenderQuality)
            gd->setMachineRenderQuality(m_machineRenderQuality);
        gd->rebuildDisplay();
        gd->updateAxisTransforms();
        displayAxisGuides();
        if (gd->hasView())
            gd->view()->Redraw();
    }
    lcnc::Kernel::current().app()->notifyDocumentModified(machineDocumentId());
}
