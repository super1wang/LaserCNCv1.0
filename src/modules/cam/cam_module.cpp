
#include "modules/cam/cam_module.h"
#include "view/toolpath_renderer.h"
#include "view/machine_guide_renderer.h"
#include "modules/cam/services/cam_data_manager.h"
#include "modules/cam/services/toolpath_simulator.h"
#include "modules/cam/services/machine_axis_detector.h"
#include "modules/cam/services/machine_io.h"
#include "modules/cam/services/reference_pick.h"
#include "core/kernel/kernel.h"
#include "modules/cad/services/shape_service.h"

#include "modules/cam/settings/cam_config.h"
#include "core/algorithms/cam/face_classifier.h"
#include "core/document/lcnc_document.h"
#include "core/algorithms/cam/laser_toolpath.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/kinematics/machine_pose.h"
#include "core/algorithms/cam/machine_model_compressor.h"
#include "core/kernel/i_kernel.h"
#include "core/kernel/service_registry.h"
#include "core/logging/logger.h"
#include "core/project/lcnc_project_manager.h"
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
#include <TopLoc_Location.hxx>
#include <gp_Trsf.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <TDF_LabelSequence.hxx>
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

// Out-of-line dtor for unique_ptr<前置声明类型>。
CamModule::~CamModule() = default;

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
    if (m_pose) {
        // 把 MachinePose 也作为共享 IService 暴露，跨模块（process/UI）可读写姿态。
        auto poseSvc = std::shared_ptr<lcnc::MachinePose>(m_pose.get(), [](lcnc::MachinePose*) {});
        kernel.services().registerService<lcnc::MachinePose>(poseSvc);
    }
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
    , m_toolpathRenderer(std::make_unique<lcnc::view::ToolpathRenderer>())
    , m_guideRenderer(std::make_unique<lcnc::view::MachineGuideRenderer>())
    , m_camData(std::make_unique<lcnc::cam::CamDataManager>())
    , m_toolpath(m_camData->toolpath())
{
    // 加载持久化配置（首次启动会自动迁移旧版 CamConfig.json -> cam.toml）。
    m_config.loadDefault();

    auto* project = lcnc::Kernel::current().projectManager();
    project->ensureProject();

    CamConfig& config = m_config;
    m_machineModelPath = config.machineModelPath();
    m_machineRenderQualityPreset = config.machineRenderQualityPreset();
    m_toolpath.setGlobalLeadInLength(config.leadInLength());
    m_toolpath.setGlobalNormalAngle(config.normalAngle());
    m_deflection = config.deflection();
    m_smoothAngle = config.smoothAngle();
    m_useFaceClassification = config.useFaceClassification();
    m_toolpathRenderer->setShowNormals(config.showNormals());
    m_toolpathRenderer->setNormalSampleStep(config.normalSampleStep());

    m_simulator = std::make_unique<lcnc::cam::ToolpathSimulator>(this);
    m_simulator->setToolpath(&m_toolpath);
    m_simulator->setApplyAxisFn([this](const QString& axis, double value, bool refreshNow) {
        setAxisPosition(axis, value, refreshNow);
    });
    m_simulator->setRefreshFn([this]() { refreshMachineTransforms(); });
    connect(m_simulator.get(), &lcnc::cam::ToolpathSimulator::simulationTick,
            this, &CamModule::simulationTick);
    connect(m_simulator.get(), &lcnc::cam::ToolpathSimulator::simulationStateChanged,
            this, &CamModule::simulationStateChanged);
    connect(m_simulator.get(), &lcnc::cam::ToolpathSimulator::simulationFinished,
            this, &CamModule::simulationFinished);

    connect(project, &lcnc::LcncProjectManager::domainDataChanged,
            this, [this](lcnc::ProjectDomain domain) {
                if (domain == lcnc::ProjectDomain::Machine) {
                    // 项目工作区变更（构型切换/重新装载）后同步 pose 的轴集合
                    if (m_pose && m_pose->kinematics() != kinematics())
                        m_pose->setKinematics(kinematics());
                    emit machineWorkspaceChanged();
                }
            });

    // 姿态状态对象 + 16ms 单次定时合并器（dirty-axis 局部刷新）。
    m_pose = std::make_unique<lcnc::MachinePose>(this);
    m_refreshCoalescer = new QTimer(this);
    m_refreshCoalescer->setSingleShot(true);
    m_refreshCoalescer->setInterval(16); // ≈ 60Hz 上限
    connect(m_refreshCoalescer, &QTimer::timeout, this, [this]() {
        QStringList dirty(m_pendingDirtyAxes.cbegin(), m_pendingDirtyAxes.cend());
        m_pendingDirtyAxes.clear();
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "CamModule coalescer flush n={}", dirty.size());
        refreshMachineTransforms(dirty);
    });
    connect(m_pose.get(), &lcnc::MachinePose::poseChanged, this,
            [this](const QStringList& dirtyAxes) {
        if (m_inPoseSelfUpdate)
            return; // 由 setAxisPosition 触发的自更新已自行处理刷新
        for (const QString& a : dirtyAxes)
            m_pendingDirtyAxes.insert(a);
        if (!m_refreshCoalescer->isActive())
            m_refreshCoalescer->start();
    });
}

// ── Domain documents ──────────────────────────────────────────────────────────

LcncDocument* CamModule::machineDocument() const
{
    return lcnc::Kernel::current().projectManager()->machineDocument();
}

DocumentId CamModule::machineDocumentId() const
{
    return lcnc::Kernel::current().projectManager()->machineDocumentId();
}

LcncDocument* CamModule::camDocument() const
{
    return lcnc::Kernel::current().projectManager()->camDocument();
}

DocumentId CamModule::camDocumentId() const
{
    return lcnc::Kernel::current().projectManager()->camDocumentId();
}

LcncDocument* CamModule::workpieceDocument() const
{
    return lcnc::Kernel::current().projectManager()->workpieceDocument();
}

DocumentId CamModule::workpieceDocumentId() const
{
    return lcnc::Kernel::current().projectManager()->workpieceDocumentId();
}

GuiDocument* CamModule::workspaceGuiDocument() const
{
    return lcnc::Kernel::current().guiApp()->workspaceGuiDocument();
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

lcnc::RenderQualityPreset CamModule::machineRenderQualityPreset() const
{
    return m_machineRenderQualityPreset;
}

void CamModule::setMachineRenderQualityPreset(lcnc::RenderQualityPreset quality)
{
    if (m_machineRenderQualityPreset == quality)
        return;

    m_machineRenderQualityPreset = quality;
    m_config.setMachineRenderQualityPreset(quality);
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

    autoInstallCurrentWorkpieceInternal(m_config.autoInstallWorkpiece());

    displayAxisGuides();
    refreshMachineTransforms();
    emit axisAssignmentsChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
}

void CamModule::loadMachine(const QString& filePath)
{
    LcncDocument* doc = machineDocument();
    if (!doc || filePath.isEmpty()) return;

    QFileInfo fi(filePath);
    if (!fi.exists())
        return;

    const QString normalizedPath = fi.absoluteFilePath();

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
        [filePath, doc](TaskProgress* prog) {
            lcnc::cam::machine_io::loadMachineFromFile(doc, filePath, prog);
        });

    watchTask(this, taskId, [this, normalizedPath](bool ok) {
        if (!ok)
            return;

        m_machineModelPath = normalizedPath;
        m_config.setMachineModelPath(m_machineModelPath);
        autoDetectAxes();
        applyStoredMachineProfile(m_machineModelPath);
        const bool installed = autoInstallCurrentWorkpieceInternal(m_config.autoInstallWorkpiece());
        if (!installed)
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
    if (!machDoc || filePath.isEmpty())
        return;

    lcnc::cam::machine_io::exportMachineToFile(machDoc, machDoc->machineKinematics(), filePath);
}

void CamModule::autoDetectAxes()
{
    LcncDocument* doc = machineDocument();
    if (!doc) return;

    lcnc::cam::machine_axis_detector::autoDetectAxisNames(doc, doc->machineKinematics());
    autoDetectAxisOrigins();
    applyStoredMachineProfile(m_machineModelPath);
    if (auto* gd = workspaceGuiDocument())
        gd->applyMachineDisplayStyle();
    refreshMachineTransforms();
    emit axisAssignmentsChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
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

    if (auto* gd = workspaceGuiDocument())
        gd->applyMachineDisplayStyle();
    refreshMachineTransforms();
    emit axisAssignmentsChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
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

QString CamModule::defaultWorkpieceMountAxis() const
{
    const MachineKinematics* kin = kinematics();
    if (!kin || kin->axes().isEmpty())
        return QString();

    const auto hasAxis = [kin](const QString& axisName) {
        return kin->findAxis(axisName) != nullptr;
    };

    const QString configType = kin->configType();
    if (configType == QStringLiteral("VERTICAL_AC_TABLE")
        || configType == QStringLiteral("VERTICAL_BC_TABLE")) {
        if (hasAxis(QStringLiteral("C")))
            return QStringLiteral("C");
    } else if (configType == QStringLiteral("XYZA")) {
        if (hasAxis(QStringLiteral("A")))
            return QStringLiteral("A");
    }

    if (hasAxis(QStringLiteral("BASE")))
        return QStringLiteral("BASE");

    return kin->axes().isEmpty() ? QString() : kin->axes().first().name;
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
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
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
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
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
    if (auto* gd = workspaceGuiDocument()) {
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

bool CamModule::pickReferenceFaceCenter(WidgetOccView* occView,
                                        const QPoint& screenPos,
                                        gp_Pnt& center,
                                        QString* errorMessage) const
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CamModule::pickReferenceFaceCenter begin");
    const bool ok = resolveReferencePlaneCenter(occView, screenPos, center, errorMessage);
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CamModule::pickReferenceFaceCenter end ok={}", ok);
    return ok;
}

bool CamModule::enterStandardCalibrationPose(const AxisCalibrationInputs& inputs,
                                             QString* errorMessage)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CamModule::enterStandardCalibrationPose begin");

    auto fail = [&](const QString& msg) {
        if (errorMessage)
            *errorMessage = msg;
        LCNC_ERR(lcnc::LogCode::Generic,
                 "CamModule::enterStandardCalibrationPose failed: {}",
                 msg.toStdString());
        emit operationFailed(tr("机台标定位"), msg);
        return false;
    };

    QString reason;
    if (!ensureAcCenterCalibrationAvailable(&reason))
        return fail(reason);

    MachineKinematics* kin = kinematics();
    if (!kin)
        return fail(tr("找不到机台轴系配置。"));

    try {
        // 1) A 轴原点：保留当前 X，Y/Z 取 A 参考面中心
        const gp_Pnt currentA = kin->axisOrigin(QStringLiteral("A"));
        const gp_Pnt newA(currentA.X(),
                          inputs.aFaceCenter.Y(),
                          inputs.aFaceCenter.Z());
        // 2) C 轴原点：X 取 C 参考面中心，Y/Z 投影到 A 轴线
        const gp_Pnt newC(inputs.cFaceCenter.X(),
                          newA.Y(),
                          newA.Z());

        if (!kin->setAxisOrigin(QStringLiteral("A"), newA))
            return fail(tr("写入 A 轴原点失败。"));
        if (!kin->setAxisOrigin(QStringLiteral("C"), newC))
            return fail(tr("写入 C 轴原点失败。"));

        // 3) 切割头模型点（BASE 局部坐标），直接采用拾取面中心
        m_cutterHeadModelPosition = inputs.cutterHeadFaceCenter;

        // 4) 进入"机台标定位"：A=0, C=0；XY 调整为切割头世界 XY 与 AC 中心 XY 对齐
        kin->setAxisPosition(QStringLiteral("A"), 0.0);
        kin->setAxisPosition(QStringLiteral("C"), 0.0);
        const gp_Pnt acCenter(newC.X(), newA.Y(), newA.Z());
        if (kin->findAxis(QStringLiteral("X")))
            kin->setAxisPosition(QStringLiteral("X"),
                                 acCenter.X() - m_cutterHeadModelPosition.X());
        if (kin->findAxis(QStringLiteral("Y")))
            kin->setAxisPosition(QStringLiteral("Y"),
                                 acCenter.Y() - m_cutterHeadModelPosition.Y());

        LCNC_INFO(lcnc::LogCode::Generic,
                  "Standard pose entered: A.origin=({:.3f},{:.3f},{:.3f}) "
                  "C.origin=({:.3f},{:.3f},{:.3f}) head.model=({:.3f},{:.3f},{:.3f})",
                  newA.X(), newA.Y(), newA.Z(),
                  newC.X(), newC.Y(), newC.Z(),
                  m_cutterHeadModelPosition.X(),
                  m_cutterHeadModelPosition.Y(),
                  m_cutterHeadModelPosition.Z());
    } catch (const Standard_Failure& f) {
        return fail(tr("OCC 异常：%1").arg(QString::fromUtf8(f.GetMessageString())));
    } catch (const std::exception& e) {
        return fail(tr("异常：%1").arg(QString::fromUtf8(e.what())));
    } catch (...) {
        return fail(tr("发生未知异常。"));
    }

    displayAxisGuides();
    refreshMachineTransforms();
    return true;
}

gp_Pnt CamModule::cutterHeadWorldPosition() const
{
    const MachineKinematics* kin = kinematics();
    if (!kin)
        return m_cutterHeadModelPosition;
    // 切割头几何挂在 Z 轴链下（BASE→Y→X→Z），由该链变换 m_cutterHeadModelPosition。
    gp_Pnt pos = m_cutterHeadModelPosition;
    pos.Transform(kin->computeAxisTransform(QStringLiteral("Z")));
    return pos;
}

bool CamModule::acAngleOffset(double& outA, double& outC) const
{
    if (!m_hasAcAngleOffset)
        return false;
    outA = m_acAngleOffsetA;
    outC = m_acAngleOffsetC;
    return true;
}

bool CamModule::physicalAcCenter(gp_Pnt& outCenter) const
{
    if (!m_hasPhysicalAcCenter)
        return false;
    outCenter = m_physicalAcCenter;
    return true;
}

bool CamModule::isMachineCalibrated() const
{
    // 标定完整的判据：已记录物理中心 + AC 角度映射 + 切割头模型点。
    // 三者通常由 applyAxisCalibration 一次性写入。
    return m_hasPhysicalAcCenter && m_hasAcAngleOffset;
}

bool CamModule::applyAxisCalibration(const AxisCalibrationInputs& inputs,
                                     QString* errorMessage)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CamModule::applyAxisCalibration begin");

    auto fail = [&](const QString& msg) {
        if (errorMessage)
            *errorMessage = msg;
        LCNC_ERR(lcnc::LogCode::Generic,
                 "CamModule::applyAxisCalibration failed: {}",
                 msg.toStdString());
        emit operationFailed(tr("机台坐标系标定"), msg);
        return false;
    };

    // ── 1) 进入标定位（写入轴心、切割头模型点、A=C=0、XY 对齐） ────────
    QString reason;
    if (!enterStandardCalibrationPose(inputs, &reason))
        return fail(reason);

    try {
        // ── 2) 持久化轴心与切割头模型点 ────────────────────────────────
        if (!m_machineModelPath.isEmpty()) {
            MachineKinematics* kin = kinematics();
            if (kin) {
                m_config.setAxisOriginForMachine(m_machineModelPath, QStringLiteral("A"),
                                                  kin->axisOrigin(QStringLiteral("A")));
                m_config.setAxisOriginForMachine(m_machineModelPath, QStringLiteral("C"),
                                                  kin->axisOrigin(QStringLiteral("C")));
            }
            m_config.setCutterHeadModelPositionForMachine(m_machineModelPath,
                                                          m_cutterHeadModelPosition);
        }

        // ── 3) 整机平移：将当前模型 AC 中心搬到物理 AC 中心 ───────────
        if (inputs.hasPhysicalCenter) {
            gp_Pnt currentCenter;
            if (!currentAcRotationCenter(currentCenter))
                return fail(tr("无法计算当前模型 AC 中心。"));

            const gp_Vec translation(currentCenter, inputs.physicalAcCenter);
            if (translation.SquareMagnitude() >= 1e-12) {
                if (!translateMachineWorkspace(translation, tr("机台坐标系标定")))
                    return false; // translateMachineWorkspace 已发 operationFailed
            }
        }

        // ── 4) 持久化 AC 角度偏移（标定位 → 物理角度的映射） ─────────
        m_hasAcAngleOffset = true;
        m_acAngleOffsetA   = inputs.physicalAAngle;
        m_acAngleOffsetC   = inputs.physicalCAngle;
        // 物理 AC 中心 XYZ：作为标定记录持久化，向导回显时回填。
        if (inputs.hasPhysicalCenter) {
            m_hasPhysicalAcCenter = true;
            m_physicalAcCenter    = inputs.physicalAcCenter;
        }
        if (!m_machineModelPath.isEmpty()) {
            m_config.setAcAngleOffsetForMachine(m_machineModelPath,
                                                inputs.physicalAAngle,
                                                inputs.physicalCAngle);
            if (inputs.hasPhysicalCenter) {
                m_config.setPhysicalAcCenterForMachine(m_machineModelPath,
                                                       inputs.physicalAcCenter);
            }
        }

        // ── 5) 自动 STEP 回写：标定后的姿态作为下次启动的初始模型 ─────
        if (!m_machineModelPath.isEmpty()) {
            LCNC_INFO(lcnc::LogCode::Generic,
                      "Saving calibrated machine model back to: {}",
                      m_machineModelPath.toStdString());
            LcncDocument* doc = machineDocument();
            if (doc && !lcnc::cam::machine_io::exportMachineToFile(
                            doc, doc->machineKinematics(), m_machineModelPath)) {
                LCNC_WARN(lcnc::LogCode::Generic,
                          "Auto-save of machine model failed: {}",
                          m_machineModelPath.toStdString());
                // 仅警告，不中断标定流程
            }
        }
    } catch (const Standard_Failure& f) {
        return fail(tr("OCC 异常：%1").arg(QString::fromUtf8(f.GetMessageString())));
    } catch (const std::exception& e) {
        return fail(tr("异常：%1").arg(QString::fromUtf8(e.what())));
    } catch (...) {
        return fail(tr("发生未知异常。"));
    }

    displayAxisGuides();
    refreshMachineDisplay();
    if (hasToolpath())
        recalcToolpath();

    LCNC_INFO(lcnc::LogCode::Generic,
              "CamModule::applyAxisCalibration done (origins/head/center/angles applied, model saved)");
    emit machineWorkspaceChanged();
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
        emit operationFailed(operationTitle, tr("找不到项目文档或轴系配置。"));
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
        emit operationFailed(tr("压缩机台"), tr("找不到项目文档或轴系配置。"));
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
            emit operationFailed(tr("压缩机台"), tr("压缩完成后找不到项目文档，结果未写回。"));
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
    LcncDocument* doc = lcnc::Kernel::current().projectManager()->workpieceDocument();
    if (!doc)
        return result;

    const int workpieceCount = doc->entityLabels(LcncDocument::EntityKind::Workpiece).Length();
    if (workpieceCount <= 0)
        return result;

    const QString stateName = lcnc::Kernel::current().projectManager()->session().workpiece().displayName.trimmed();
    const QString displayName = stateName.isEmpty() ? tr("当前工件") : stateName;

    result.append({doc->id(), tr("%1  (%2 形体)").arg(displayName).arg(workpieceCount), workpieceCount});

    return result;
}

gp_Pnt CamModule::workpieceInstallPosition() const
{
    return m_workpieceInstallPosition;
}

bool CamModule::autoInstallWorkpiece() const
{
    return m_config.autoInstallWorkpiece();
}

void CamModule::setAutoInstallWorkpiece(bool enabled)
{
    m_config.setAutoInstallWorkpiece(enabled);
    autoInstallCurrentWorkpieceInternal(enabled);
    emit machineWorkspaceChanged();
}

bool CamModule::autoInstallCurrentWorkpiece()
{
    return autoInstallCurrentWorkpieceInternal(m_config.autoInstallWorkpiece());
}

QStringList CamModule::mountedWorkpieceEntriesForSourceEntries(const QStringList& sourceEntries) const
{
    QStringList result;
    const auto appendEntry = [&result](const QString& entry) {
        if (!entry.isEmpty() && !result.contains(entry))
            result.append(entry);
    };

    if (sourceEntries.isEmpty()) {
        if (LcncDocument* doc = workpieceDocument()) {
            const TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
            for (int i = 1; i <= labels.Length(); ++i)
                appendEntry(XcafUtils::entry(labels.Value(i)));
        }
        return result;
    }

    for (const QString& sourceEntry : sourceEntries)
        appendEntry(sourceEntry);
    return result;
}

QStringList CamModule::sourceWorkpieceEntriesForMountedEntries(const QStringList& mountedEntries) const
{
    QStringList result;
    GuiDocument* gd = workspaceGuiDocument();
    LcncDocument* doc = workpieceDocument();
    if (!gd || !doc)
        return result;

    const QStringList selectedSourceEntries = gd->selectedEntries(doc->id());
    if (selectedSourceEntries.isEmpty())
        return result;

    for (const QString& mountedEntry : mountedEntries) {
        if (!mountedEntry.isEmpty()
            && selectedSourceEntries.contains(mountedEntry)
            && !result.contains(mountedEntry))
            result.append(mountedEntry);
    }
    return result;
}

void CamModule::setMountedWorkpieceEntriesVisible(const QStringList& sourceEntries, bool visible)
{
    GuiDocument* gd = workspaceGuiDocument();
    const DocumentId docId = workpieceDocumentId();
    if (!gd || docId == kInvalidDocumentId)
        return;

    const QStringList entries = mountedWorkpieceEntriesForSourceEntries(sourceEntries);
    for (const QString& entry : entries) {
        Handle(AIS_Shape) ais = gd->aisShape(docId, entry);
        if (ais.IsNull())
            continue;
        if (visible)
            gd->scene()->displayObject(ais, false);
        else
            gd->scene()->eraseObject(ais, false);
    }
    if (!gd->context().IsNull())
        gd->context()->UpdateCurrentViewer();
    if (gd->hasView())
        gd->view()->Redraw();
}

void CamModule::setSelectedMountedWorkpieceEntries(const QStringList& sourceEntries)
{
    GuiDocument* gd = workspaceGuiDocument();
    const DocumentId docId = workpieceDocumentId();
    if (!gd || docId == kInvalidDocumentId)
        return;

    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull())
        return;

    ctx->ClearSelected(false);
    for (const QString& entry : mountedWorkpieceEntriesForSourceEntries(sourceEntries)) {
        Handle(AIS_Shape) ais = gd->aisShape(docId, entry);
        if (!ais.IsNull())
            ctx->AddOrRemoveSelected(ais, false);
    }

    if (gd->hasView())
        gd->view()->Redraw();
}

void CamModule::setWorkpieceInstallPosition(const gp_Pnt& position)
{
    const bool currentUnchanged = m_workpieceInstallPosition.SquareDistance(position) < 1e-12;
    const bool bakedUnchanged = m_workpieceInstallPositionBaked.SquareDistance(position) < 1e-12;
    if (currentUnchanged && bakedUnchanged)
        return;

    const gp_Vec translation(m_workpieceInstallPositionBaked, position);
    const bool movedWorkpieces = translation.SquareMagnitude() > 1e-12;
    if (movedWorkpieces && !translateWorkpieceDocument(translation)) {
        emit operationFailed(tr("工件安装位置"), tr("更新工件安装位置失败，当前安装位置未修改。"));
        return;
    }

    if (movedWorkpieces) {
        translateToolpathWorldData(translation);
        updateToolpathMachineCoordinates();
    }

    m_workpieceInstallPosition = position;
    m_workpieceInstallPositionBaked = position;
    if (!m_machineModelPath.isEmpty()) {
        m_config.setWorkpieceInstallPositionForMachine(
            m_machineModelPath,
            m_workpieceInstallPosition);
    }

    if (movedWorkpieces) {
        resetWorkpieceDisplayLocation();
        refreshWorkpieceDisplay();
        if (hasToolpath())
            syncCamDocumentContours();
        if (hasToolpath() || m_previewLeadInValid)
            refreshToolpathDisplay();
    }

    emit machineWorkspaceChanged();
}

void CamModule::updateWorkpieceInstallLocation(const gp_Pnt& position)
{
    if (m_workpieceInstallPosition.SquareDistance(position) < 1e-12)
        return;

    m_workpieceInstallPosition = position;
    if (!m_machineModelPath.isEmpty()) {
        m_config.setWorkpieceInstallPositionForMachine(
            m_machineModelPath, m_workpieceInstallPosition);
    }

    auto* gd = workspaceGuiDocument();
    LcncDocument* doc = workpieceDocument();
    if (!gd || !doc)
        return;

    // 计算 baked → 当前 的平移增量，对所有已展示的工件 AIS 调 SetLocation。
    const gp_Vec delta(m_workpieceInstallPositionBaked, m_workpieceInstallPosition);
    gp_Trsf trsf;
    trsf.SetTranslation(delta);
    const TopLoc_Location loc(trsf);
    const auto& ctx = gd->scene()->context();
    if (ctx.IsNull())
        return;

    const TDF_LabelSequence wpcLabels =
        doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    int updated = 0;
    for (int i = 1; i <= wpcLabels.Length(); ++i) {
        const QString entry = XcafUtils::entry(wpcLabels.Value(i));
        Handle(AIS_Shape) ais = gd->aisShape(doc->id(), entry);
        if (ais.IsNull())
            continue;
        ctx->SetLocation(ais, loc);
        ++updated;
    }
    if (updated > 0)
        ctx->UpdateCurrentViewer();
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
    autoInstallCurrentWorkpieceInternal(true);
    return true;
}

void CamModule::autoDetectAxisOrigins()
{
    lcnc::cam::machine_axis_detector::autoDetectAxisOrigins(machineDocument(), kinematics());
}

void CamModule::applyStoredMachineProfile(const QString& machinePath)
{
    MachineKinematics* kin = kinematics();
    if (!kin || machinePath.isEmpty())
        return;

    m_workpieceInstallPosition = defaultWorkpieceInstallPosition();

    const auto profile = lcnc::cam::machine_axis_detector::applyStoredMachineProfile(
        kin, m_config, machinePath);

    if (profile.hasCutterHeadModel)
        m_cutterHeadModelPosition = profile.cutterHeadModelPosition;
    if (profile.hasCutterHeadPhysical)
        m_cutterHeadPhysicalPosition = profile.cutterHeadPhysicalPosition;
    if (profile.hasWorkpieceInstall)
        m_workpieceInstallPosition = profile.workpieceInstallPosition;

    double aOff = 0.0;
    double cOff = 0.0;
    if (m_config.acAngleOffsetForMachine(machinePath, &aOff, &cOff)) {
        m_hasAcAngleOffset = true;
        m_acAngleOffsetA   = aOff;
        m_acAngleOffsetC   = cOff;
        LCNC_INFO(lcnc::LogCode::Generic,
                  "Loaded AC angle offset for machine: A={:.3f}° C={:.3f}°", aOff, cOff);
    } else {
        m_hasAcAngleOffset = false;
        m_acAngleOffsetA = 0.0;
        m_acAngleOffsetC = 0.0;
    }

    gp_Pnt physCenter;
    if (m_config.physicalAcCenterForMachine(machinePath, &physCenter)) {
        m_hasPhysicalAcCenter = true;
        m_physicalAcCenter    = physCenter;
        LCNC_INFO(lcnc::LogCode::Generic,
                  "Loaded physical AC center for machine: ({:.3f},{:.3f},{:.3f})",
                  physCenter.X(), physCenter.Y(), physCenter.Z());
    } else {
        m_hasPhysicalAcCenter = false;
        m_physicalAcCenter    = gp_Pnt(0.0, 0.0, 0.0);
    }
}

gp_Pnt CamModule::defaultWorkpieceInstallPosition() const
{
    gp_Pnt center;
    if (currentWorkpieceRotationCenter(center))
        return gp_Pnt(center.X(), center.Y(), 0.0);

    return gp_Pnt(0.0, 0.0, 0.0);
}

// ── Workpiece Installation ───────────────────────────────────────────────────

void CamModule::mountWorkpiece(DocumentId sourceDocId, const QString& axisName, bool alignToInstallPosition)
{
    auto* project = lcnc::Kernel::current().projectManager();
    LcncDocument* srcDoc = project->domainDocumentById(sourceDocId);
    if (!srcDoc || !project->isDomainDocument(sourceDocId, lcnc::ProjectDomain::Workpiece))
        return;

    MachineKinematics* kin = kinematics();
    const QString targetAxis = axisName.trimmed().isEmpty()
        ? defaultWorkpieceMountAxis()
        : axisName.trimmed();

    TDF_LabelSequence sourceLabels = srcDoc->entityLabels(LcncDocument::EntityKind::Workpiece);
    if (sourceLabels.Length() == 0)
        return;

    BRep_Builder bb;
    TopoDS_Compound compound;
    bb.MakeCompound(compound);
    bool hasShape = false;
    Handle(XCAFDoc_ShapeTool) sourceShapeTool = srcDoc->shapeTool();
    for (int i = 1; i <= sourceLabels.Length(); ++i) {
        TopoDS_Shape sh = sourceShapeTool->GetShape(sourceLabels.Value(i));
        if (!sh.IsNull()) {
            bb.Add(compound, sh);
            hasShape = true;
        }
    }
    if (!hasShape) return;

    const gp_Pnt currentCenter = shapeCenter(compound);
    const gp_Pnt targetPosition = alignToInstallPosition ? m_workpieceInstallPosition : currentCenter;
    const gp_Vec placement(currentCenter, targetPosition);
    bool mountChanged = false;
    QString firstEntry;
    QMap<QString, QString> mountedEntries;
    for (int i = 1; i <= sourceLabels.Length(); ++i) {
        const QString entry = XcafUtils::entry(sourceLabels.Value(i));
        if (entry.isEmpty())
            continue;
        if (firstEntry.isEmpty())
            firstEntry = entry;
        mountedEntries.insert(entry, entry);
        if (!kin)
            continue;

        const QString currentAxis = kin->mountedAxis(entry);
        if (currentAxis == targetAxis)
            continue;

        if (targetAxis.isEmpty())
            kin->unmountWorkpiece(entry);
        else
            kin->mountWorkpiece(entry, targetAxis);
        mountChanged = true;
    }

    const bool movedWorkpiece = placement.SquareMagnitude() > 1e-12;
    const bool positionUnchanged = m_workpieceInstallPosition.SquareDistance(targetPosition) < 1e-12
        && m_workpieceInstallPositionBaked.SquareDistance(targetPosition) < 1e-12;
    if (!movedWorkpiece && !mountChanged && positionUnchanged)
        return;

    if (movedWorkpiece) {
        clearToolpath();
        if (!translateWorkpieceDocument(placement)) {
            emit operationFailed(tr("工件安装"), tr("移动工件到安装位置失败。"));
            return;
        }
    }

    m_workpieceInstallPosition = targetPosition;
    m_workpieceInstallPositionBaked = targetPosition;
    m_mountedWorkpieceEntryBySourceEntry = mountedEntries;
    if (!m_machineModelPath.isEmpty())
        m_config.setWorkpieceInstallPositionForMachine(m_machineModelPath, m_workpieceInstallPosition);

    resetWorkpieceDisplayLocation();
    if (movedWorkpiece)
        refreshWorkpieceDisplay();
    else
        refreshMachineTransforms();
    emit machineWorkspaceChanged();
    emit workpieceMounted(firstEntry);
}

bool CamModule::autoInstallCurrentWorkpieceInternal(bool alignToInstallPosition)
{
    if (!alignToInstallPosition)
        return false;

    auto* project = lcnc::Kernel::current().projectManager();
    const DocumentId sourceDocId = project->workpieceDocumentId();
    LcncDocument* sourceDoc = project->domainDocumentById(sourceDocId);
    if (!sourceDoc || sourceDoc->entityLabels(LcncDocument::EntityKind::Workpiece).Length() == 0) {
        clearMountedWorkpieceDisplay(false);
        return false;
    }

    mountWorkpiece(sourceDocId, QString(), true);
    return true;
}

bool CamModule::clearMountedWorkpieceDisplay(bool refreshView)
{
    LcncDocument* doc = machineDocument();
    if (!doc)
        return false;

    MachineKinematics* kin = doc->machineKinematics();
    const QStringList workpieceEntries = entityEntries(doc, LcncDocument::EntityKind::Workpiece);
    const bool hadEntries = !workpieceEntries.isEmpty() || !m_mountedWorkpieceEntryBySourceEntry.isEmpty();
    for (const QString& entry : workpieceEntries) {
        if (kin)
            kin->unmountWorkpiece(entry);
    }
    if (kin) {
        for (const QString& entry : m_mountedWorkpieceEntryBySourceEntry.keys())
            kin->unmountWorkpiece(entry);
        if (LcncDocument* srcDoc = workpieceDocument()) {
            const TDF_LabelSequence sourceLabels = srcDoc->entityLabels(LcncDocument::EntityKind::Workpiece);
            for (int i = 1; i <= sourceLabels.Length(); ++i)
                kin->unmountWorkpiece(XcafUtils::entry(sourceLabels.Value(i)));
        }
    }
    doc->clearEntityKind(LcncDocument::EntityKind::Workpiece);
    m_mountedWorkpieceEntryBySourceEntry.clear();

    if (!hadEntries)
        return false;

    if (refreshView)
        refreshMachineDisplay();
    emit workpieceUnmounted();
    emit machineWorkspaceChanged();
    return true;
}

void CamModule::refreshWorkpieceDisplay()
{
    if (GuiDocument* gd = workspaceGuiDocument()) {
        gd->rebuildDomain(lcnc::ProjectDomain::Workpiece, workpieceDocument());
        gd->updateMachineWorkspaceTransforms(machineDocument(), workpieceDocument());
        if (gd->hasView())
            gd->view()->Redraw();
    }
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Workpiece);
}

void CamModule::resetWorkpieceDisplayLocation()
{
    GuiDocument* gd = workspaceGuiDocument();
    LcncDocument* doc = workpieceDocument();
    if (!gd || !doc)
        return;

    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull())
        return;

    const TopLoc_Location identityLoc;
    const TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    for (int i = 1; i <= labels.Length(); ++i) {
        const QString entry = XcafUtils::entry(labels.Value(i));
        Handle(AIS_Shape) ais = gd->aisShape(doc->id(), entry);
        if (!ais.IsNull())
            ctx->SetLocation(ais, identityLoc);
    }
}

bool CamModule::translateWorkpieceDocument(const gp_Vec& translation)
{
    LcncDocument* doc = workpieceDocument();
    if (!doc || translation.SquareMagnitude() <= 1e-12)
        return true;

    const TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    QList<TDF_Label> movedLabels;
    for (int i = 1; i <= labels.Length(); ++i) {
        const TDF_Label label = labels.Value(i);
        if (label.IsNull())
            continue;

        if (!ShapeService::moveShape(doc, label, translation)) {
            const gp_Vec rollback(-translation.X(), -translation.Y(), -translation.Z());
            for (int index = movedLabels.size() - 1; index >= 0; --index)
                ShapeService::moveShape(doc, movedLabels.at(index), rollback);
            return false;
        }
        movedLabels.append(label);
    }
    return true;
}

void CamModule::unmountAllWorkpieces()
{
    LcncDocument* doc = machineDocument();
    if (!doc) return;

    MachineKinematics* kin = doc->machineKinematics();
    const QStringList workpieceEntries = entityEntries(doc, LcncDocument::EntityKind::Workpiece);
    bool hadMountedWorkpieces = false;
    for (const QString& entry : workpieceEntries) {
        if (!kin->mountedAxis(entry).isEmpty()) {
            kin->unmountWorkpiece(entry);
            hadMountedWorkpieces = true;
        }
    }
    for (const QString& entry : m_mountedWorkpieceEntryBySourceEntry.keys()) {
        if (!kin->mountedAxis(entry).isEmpty()) {
            kin->unmountWorkpiece(entry);
            hadMountedWorkpieces = true;
        }
    }
    if (LcncDocument* srcDoc = workpieceDocument()) {
        const TDF_LabelSequence sourceLabels = srcDoc->entityLabels(LcncDocument::EntityKind::Workpiece);
        for (int i = 1; i <= sourceLabels.Length(); ++i) {
            const QString entry = XcafUtils::entry(sourceLabels.Value(i));
            if (!kin->mountedAxis(entry).isEmpty()) {
                kin->unmountWorkpiece(entry);
                hadMountedWorkpieces = true;
            }
        }
    }
    m_mountedWorkpieceEntryBySourceEntry.clear();

    const bool hadToolpath = hasToolpath();
    clearToolpath();

    if (hadMountedWorkpieces || hadToolpath)
        refreshMachineTransforms();

    if (hadMountedWorkpieces || hadToolpath) {
        lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
        emit machineWorkspaceChanged();
    }

    if (hadMountedWorkpieces)
        emit workpieceUnmounted();
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

    if (auto* gd = workspaceGuiDocument())
        gd->eraseEntity(doc->id(), entry);

    ShapeService::deleteShape(doc, entry);
    refreshMachineTransforms();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
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
    LcncDocument* doc = workpieceDocument();
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

    for (const WorkpieceShapeSource& source : workpieceSources) {
        if (source.shape.IsNull())
            continue;

        auto contours = LaserToolpathBuilder::extractContours(source.shape, params);
        if (contours.empty())
            continue;

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

            LaserToolpathBuilder::computeMachineCoordinates(contour, kinematics(), gp_Trsf());
            allContours.push_back(std::move(contour));
        }
    }

    if (allContours.empty())
        return false;

    m_toolpath.contours() = std::move(allContours);
    m_camData->ensureContourIds();
    syncCamDocumentContours();
    m_toolpathRenderer->setVisible(workspaceGuiDocument(), true);

    refreshToolpathDisplay();
    emit toolpathGenerated();
    return true;
}

void CamModule::clearToolpath()
{
    eraseToolpathDisplay();
    m_camData->clearToolpath();
    if (LcncDocument* doc = camDocument()) {
        doc->clearEntityKind(LcncDocument::EntityKind::Cam);
        if (GuiDocument* gd = workspaceGuiDocument())
            gd->eraseDomain(lcnc::ProjectDomain::Cam);
        lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
    }
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
    return lcnc::cam::reference_pick::resolveReferencePlaneCenter(occView, screenPos, center, errorMessage);
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
    for (LaserContour& contour : m_toolpath.contours()) {
        LaserToolpathBuilder::computeMachineCoordinates(contour, kinematics(), gp_Trsf());
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

int CamModule::toolpathContourCount() const
{
    return m_toolpath.contourCount();
}

int CamModule::toolpathContourPointCount(int contourIndex) const
{
    if (contourIndex < 0 || contourIndex >= m_toolpath.contourCount())
        return 0;
    return static_cast<int>(m_toolpath.contour(contourIndex).points.size());
}

void CamModule::setLeadInEntry(int contourIdx, const gp_Pnt& entryPoint, double entryParam)
{
    if (contourIdx < 0 || contourIdx >= m_toolpath.contourCount()) return;

    LaserContour& c = m_toolpath.contour(contourIdx);
    c.leadIn.entryPoint = entryPoint;
    c.leadIn.entryParam = entryParam;
    c.leadIn.valid = true;

    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refreshLeadIns(workspaceGuiDocument(), m_toolpath, kinematics(), preview);
}

void CamModule::setLeadInLength(double mm)
{
    m_toolpath.setGlobalLeadInLength(mm);
    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refreshLeadIns(workspaceGuiDocument(), m_toolpath, kinematics(), preview);
}

double CamModule::leadInLength() const
{
    return m_toolpath.globalLeadInLength();
}

void CamModule::setNormalAngle(double deg)
{
    m_toolpath.setGlobalNormalAngle(deg);
    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refreshLeadIns(workspaceGuiDocument(), m_toolpath, kinematics(), preview);
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
    return m_toolpathRenderer->showNormals();
}

void CamModule::setShowNormals(bool on)
{
    if (m_toolpathRenderer->showNormals() == on)
        return;
    m_toolpathRenderer->setShowNormals(on);
    m_toolpathRenderer->refreshNormals(workspaceGuiDocument(), m_toolpath, kinematics());
}

double CamModule::normalSampleStep() const
{
    return m_toolpathRenderer->normalSampleStep();
}

void CamModule::setNormalSampleStep(double mm)
{
    const double current = m_toolpathRenderer->normalSampleStep();
    if (mm <= 0.0)
        return;
    if (qFuzzyCompare(current + 1.0, mm + 1.0))
        return;
    m_toolpathRenderer->setNormalSampleStep(mm);
    if (m_toolpathRenderer->showNormals())
        m_toolpathRenderer->refreshNormals(workspaceGuiDocument(), m_toolpath, kinematics());
}

bool CamModule::resolveLeadInHit(WidgetOccView* occView,
                                 const QPoint& screenPos,
                                 int& contourIdx,
                                 gp_Pnt& entryPoint,
                                 double& entryParam) const
{
    return lcnc::cam::reference_pick::resolveLeadInHit(occView, screenPos, m_toolpath,
                                                       contourIdx, entryPoint, entryParam);
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
    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refreshLeadIns(workspaceGuiDocument(), m_toolpath, kinematics(), preview);
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
    m_toolpathRenderer->refreshLeadIns(workspaceGuiDocument(), m_toolpath, kinematics());
    return true;
}

void CamModule::cancelLeadInPreview()
{
    if (!m_previewLeadInValid)
        return;

    m_previewLeadInContour = -1;
    m_previewLeadInParam = 0.0;
    m_previewLeadInValid = false;
    m_toolpathRenderer->refreshLeadIns(workspaceGuiDocument(), m_toolpath, kinematics());
}

void CamModule::setContourEnabled(int contourIdx, bool enabled)
{
    if (contourIdx < 0 || contourIdx >= m_toolpath.contourCount())
        return;

    LaserContour& contour = m_toolpath.contour(contourIdx);
    if (contour.enabled == enabled)
        return;

    contour.enabled = enabled;
    setCamContourVisible(contourIdx, enabled && m_toolpathRenderer->isVisible(), false);
    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refreshContour(workspaceGuiDocument(), m_toolpath, kinematics(), contourIdx, preview);
}

void CamModule::setAllContoursEnabled(bool enabled)
{
    bool changed = false;
    for (int index = 0; index < m_toolpath.contourCount(); ++index) {
        LaserContour& contour = m_toolpath.contour(index);
        if (contour.enabled == enabled)
            continue;
        contour.enabled = enabled;
        changed = true;
    }

    if (!changed)
        return;

    setCamContoursVisible(m_toolpathRenderer->isVisible(), false);
    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refresh(workspaceGuiDocument(), m_toolpath, kinematics(), preview);
}

lcnc::cam::ContourId CamModule::contourIdAt(int contourIdx) const
{
    return m_camData ? m_camData->contourIdAt(contourIdx) : 0;
}

int CamModule::contourIndexById(lcnc::cam::ContourId contourId) const
{
    return m_camData ? m_camData->contourIndexById(contourId) : -1;
}

void CamModule::reorderContoursById(const QList<lcnc::cam::ContourId>& order)
{
    if (!m_camData || !m_camData->reorderContoursById(order))
        return;

    syncCamDocumentContours();
    if (m_toolpathRenderer->isVisible())
        refreshToolpathDisplay();
}

void CamModule::reorderContours(const QList<int>& order)
{
    if (!m_camData || !m_camData->reorderContours(order))
        return;

    syncCamDocumentContours();
    if (m_toolpathRenderer->isVisible())
        refreshToolpathDisplay();
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

        LaserToolpathBuilder::computeMachineCoordinates(contour, kin, gp_Trsf());
    }

    refreshToolpathDisplay();
    m_camData->ensureContourIds();
    syncCamDocumentContours();
}

void CamModule::setToolpathVisible(bool visible)
{
    if (m_toolpathRenderer->isVisible() == visible) {
        setCamContoursVisible(visible);
        return;
    }
    m_toolpathRenderer->setVisible(workspaceGuiDocument(), visible);
    setCamContoursVisible(visible);
    if (visible)
        refreshToolpathDisplay();
    emit toolpathVisibilityChanged(visible);
}

bool CamModule::isToolpathVisible() const
{
    return m_toolpathRenderer->isVisible();
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

void CamModule::eraseAxisGuideDisplay()
{
    m_guideRenderer->erase(workspaceGuiDocument());
}

void CamModule::displayAxisGuides()
{
    m_guideRenderer->refresh(workspaceGuiDocument(), kinematics(), m_cutterHeadModelPosition);
}

void CamModule::updateAxisGuideTransforms()
{
    m_guideRenderer->updateTransforms(workspaceGuiDocument(), kinematics());
}

void CamModule::refreshToolpathDisplay()
{
    lcnc::view::ToolpathRenderer::LeadInPreview preview{
        m_previewLeadInContour, m_previewLeadInPoint, m_previewLeadInParam, m_previewLeadInValid
    };
    m_toolpathRenderer->refresh(workspaceGuiDocument(), m_toolpath, kinematics(), preview);
}

void CamModule::eraseToolpathDisplay()
{
    m_toolpathRenderer->erase(workspaceGuiDocument());
}

const QList<Handle(AIS_Shape)>& CamModule::contourAis() const
{
    m_camContourAisCache.clear();
    GuiDocument* gd = workspaceGuiDocument();
    LcncDocument* doc = camDocument();
    if (!gd || !doc)
        return m_camContourAisCache;

    const TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Cam);
    for (int i = 1; i <= labels.Length(); ++i) {
        const QString entry = XcafUtils::entry(labels.Value(i));
        m_camContourAisCache.append(gd->aisShape(doc->id(), entry));
    }
    return m_camContourAisCache;
}

// ── Simulation (delegated to ToolpathSimulator) ──────────────────────────────

void CamModule::simulatePlay()           { m_simulator->play(); }
void CamModule::simulatePause()          { m_simulator->pause(); }
void CamModule::simulateStop()           { m_simulator->stop(); }
void CamModule::setSimulationSpeed(double f) { m_simulator->setSpeed(f); }
bool CamModule::isSimulating() const     { return m_simulator->isPlaying(); }
bool CamModule::isSimPaused() const      { return m_simulator->isPaused(); }

void CamModule::setAxisPosition(const QString& axisName, double value, bool refreshNow)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CamModule::setAxisPosition {}={} refreshNow={}",
               axisName.toStdString(), value, refreshNow);
    if (!m_pose || !kinematics()) {
        LCNC_DEBUG(lcnc::LogCode::Generic,
                   "CamModule::setAxisPosition: no pose/kinematics, skip");
        return;
    }
    // 由 pose 统一写值；写入会同步回 kinematics 并 emit poseChanged。
    // 通过 m_inPoseSelfUpdate 抑制 coalescer，让本调用同步刷新（保留旧语义）。
    m_inPoseSelfUpdate = true;
    const bool changed = m_pose->setAxisValue(axisName, value, /*emitChanged*/ false);
    m_inPoseSelfUpdate = false;
    if (!changed)
        return;
    if (refreshNow) {
        refreshMachineTransforms(QStringList{axisName});
    } else {
        m_pendingDirtyAxes.insert(axisName);
        if (m_refreshCoalescer && !m_refreshCoalescer->isActive())
            m_refreshCoalescer->start();
    }
}

lcnc::MachinePose* CamModule::machinePose() const
{
    return m_pose.get();
}

void CamModule::setEntityVisible(const QString& entry, bool visible)
{
    if (entry.isEmpty())
        return;

    if (auto* gd = workspaceGuiDocument()) {
        Handle(AIS_Shape) ais = gd->aisShape(machineDocumentId(), entry);
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
    GuiDocument* gd = workspaceGuiDocument();
    if (!gd)
        return;

    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull())
        return;

    ctx->ClearSelected(false);
    for (const QString& entry : entries) {
        Handle(AIS_Shape) ais = gd->aisShape(machineDocumentId(), entry);
        if (!ais.IsNull())
            ctx->AddOrRemoveSelected(ais, false);
    }

    if (gd->hasView())
        gd->view()->Redraw();

    emit selectionChanged(gd->selectedEntries(machineDocumentId()));
}

QStringList CamModule::selectedEntries() const
{
    if (auto* gd = workspaceGuiDocument())
        return gd->selectedEntries(machineDocumentId());

    return {};
}

void CamModule::syncSelectionFromView()
{
    GuiDocument* gd = workspaceGuiDocument();
    QStringList selectedEntries;
    if (gd) {
        selectedEntries = gd->selectedEntries(workpieceDocumentId());
        if (selectedEntries.isEmpty())
            selectedEntries = gd->selectedEntries(machineDocumentId());
    }
    emit selectionChanged(selectedEntries);

    const QList<int> contourIndexes = selectedCamContourIndexes();
    if (!contourIndexes.isEmpty()) {
        emit toolpathContoursSelected(contourIndexes);
        if (contourIndexes.size() == 1)
            emit toolpathContourSelected(contourIndexes.first());
    }
}

void CamModule::refreshMachineTransforms()
{
    if (auto* gd = workspaceGuiDocument()) {
        gd->updateMachineWorkspaceTransforms(machineDocument(), workpieceDocument());
        applyCamContourTransforms();
        m_toolpathRenderer->updateTransforms(gd, m_toolpath, kinematics());
        updateAxisGuideTransforms();
        if (gd->hasView())
            gd->view()->Redraw();
    }
}

void CamModule::refreshMachineTransforms(const QStringList& dirtyAxes)
{
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CamModule::refreshMachineTransforms(dirty) n={}", dirtyAxes.size());
    // 现阶段 GuiDocument::updateAxisTransforms 与 MachineGuideRenderer::updateTransforms
    // 内部已是就地 SetLocalTransformation，dirty 集合主要用于：
    //   1) 跳过 m_pose 与几何已一致的"无变化"刷新（上层早 return）；
    //   2) 为后续按子轴粒度的精细化刷新预留接入点。
    // dirty 为空时退化为全量。
    if (dirtyAxes.isEmpty()) {
        refreshMachineTransforms();
        return;
    }
    if (auto* gd = workspaceGuiDocument()) {
        gd->updateMachineWorkspaceTransforms(machineDocument(), workpieceDocument());
        applyCamContourTransforms();
        m_toolpathRenderer->updateTransforms(gd, m_toolpath, kinematics());
        updateAxisGuideTransforms();
        if (gd->hasView())
            gd->view()->Redraw();
    }
}

void CamModule::setCamContoursVisible(bool visible, bool updateView)
{
    LcncDocument* doc = camDocument();
    GuiDocument* gd = workspaceGuiDocument();
    if (!doc || !gd)
        return;

    const TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Cam);
    for (int index = 0; index < labels.Length(); ++index) {
        const bool contourEnabled = index < m_toolpath.contourCount()
            ? m_toolpath.contour(index).enabled
            : true;
        const bool showContour = visible && contourEnabled;
        const QString entry = XcafUtils::entry(labels.Value(index + 1));
        Handle(AIS_Shape) ais = gd->aisShape(doc->id(), entry);
        if (ais.IsNull())
            continue;
        if (showContour)
            gd->scene()->displayObject(ais, false);
        else
            gd->scene()->eraseObject(ais, false);
    }

    if (!updateView)
        return;

    if (!gd->context().IsNull())
        gd->context()->UpdateCurrentViewer();
    if (gd->hasView())
        gd->view()->Redraw();
}

void CamModule::setCamContourVisible(int contourIndex, bool visible, bool updateView)
{
    LcncDocument* doc = camDocument();
    GuiDocument* gd = workspaceGuiDocument();
    if (!doc || !gd || contourIndex < 0)
        return;

    const TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Cam);
    if (contourIndex >= labels.Length())
        return;

    const QString entry = XcafUtils::entry(labels.Value(contourIndex + 1));
    Handle(AIS_Shape) ais = gd->aisShape(doc->id(), entry);
    if (ais.IsNull())
        return;

    if (visible)
        gd->scene()->displayObject(ais, false);
    else
        gd->scene()->eraseObject(ais, false);

    if (!updateView)
        return;

    if (!gd->context().IsNull())
        gd->context()->UpdateCurrentViewer();
    if (gd->hasView())
        gd->view()->Redraw();
}

void CamModule::applyCamContourVisibility()
{
    setCamContoursVisible(m_toolpathRenderer->isVisible());
}

void CamModule::applyCamContourTransforms()
{
    GuiDocument* gd = workspaceGuiDocument();
    LcncDocument* doc = camDocument();
    MachineKinematics* kin = kinematics();
    if (!gd || !doc || !kin)
        return;

    const Handle(AIS_InteractiveContext)& ctx = gd->context();
    if (ctx.IsNull())
        return;

    const TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Cam);
    const int count = qMin(labels.Length(), m_toolpath.contourCount());
    for (int index = 0; index < count; ++index) {
        const QString entry = XcafUtils::entry(labels.Value(index + 1));
        Handle(AIS_Shape) ais = gd->aisShape(doc->id(), entry);
        if (ais.IsNull())
            continue;

        gp_Trsf transform;
        const QString workpieceEntry = m_toolpath.contour(index).workpieceEntry;
        if (!workpieceEntry.isEmpty())
            transform = kin->computeWpcTransform(workpieceEntry);

        ais->SetLocalTransformation(transform);
        ctx->RecomputePrsOnly(ais, Standard_False);
    }
}

QList<int> CamModule::selectedCamContourIndexes() const
{
    QList<int> result;
    GuiDocument* gd = workspaceGuiDocument();
    LcncDocument* doc = camDocument();
    if (!gd || !doc)
        return result;

    const QStringList selectedEntries = gd->selectedEntries(doc->id());
    if (selectedEntries.isEmpty())
        return result;

    const TDF_LabelSequence labels = doc->entityLabels(LcncDocument::EntityKind::Cam);
    for (int index = 0; index < labels.Length(); ++index) {
        const QString entry = XcafUtils::entry(labels.Value(index + 1));
        if (selectedEntries.contains(entry))
            result.append(index);
    }
    return result;
}

void CamModule::refreshMachineDisplay()
{
    if (auto* gd = workspaceGuiDocument()) {
        gd->rebuildDomain(lcnc::ProjectDomain::Machine, machineDocument());
        gd->updateMachineWorkspaceTransforms(machineDocument(), workpieceDocument());
        displayAxisGuides();
        if (gd->hasView())
            gd->view()->Redraw();
    }
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
}

void CamModule::syncCamDocumentContours()
{
    LcncDocument* doc = camDocument();
    if (!doc)
        return;

    doc->clearEntityKind(LcncDocument::EntityKind::Cam);
    for (int index = 0; index < m_toolpath.contourCount(); ++index) {
        const LaserContour& contour = m_toolpath.contour(index);
        if (contour.wire.IsNull())
            continue;

        const QString name = contour.name.trimmed().isEmpty()
            ? tr("轮廓 %1").arg(index + 1)
            : contour.name;
        doc->addShapeEntity(contour.wire, name, LcncDocument::EntityKind::Cam);
    }

    if (GuiDocument* gd = workspaceGuiDocument()) {
        gd->rebuildDomain(lcnc::ProjectDomain::Cam, doc);
        applyCamContourTransforms();
        applyCamContourVisibility();
    }
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Cam);
}

