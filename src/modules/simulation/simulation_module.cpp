#include "modules/simulation/simulation_module.h"

#include "core/document/lcnc_document.h"
#include "core/algorithms/cam/collision_scan_policy.h"
#include "core/algorithms/occt_exact_operation_lock.h"
#include "core/document/xcaf_utils.h"
#include "core/kernel/i_kernel.h"
#include "core/kernel/kernel.h"
#include "core/kinematics/machine_configuration_service.h"
#include "core/logging/logger.h"
#include "core/project/lcnc_project_manager.h"
#include "core/settings/app_settings.h"
#include "core/task/task_manager.h"
#include "modules/cam/cam_module.h"
#include "modules/cam/i_cam_toolpath_provider.h"
#include "view/gui_document.h"
#include "view/machine_guide_renderer.h"
#include "view/toolpath_renderer.h"
#include "view/travel_path_renderer.h"
#include "view/widget_occ_view.h"

#include <AIS_Shape.hxx>
#include <BRepBndLib.hxx>
#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <Bnd_Box.hxx>
#include <Bnd_OBB.hxx>
#include <Graphic3d_Camera.hxx>
#include <OSD_Parallel.hxx>
#include <OSD_ThreadPool.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_MapOfShape.hxx>
#include <TopoDS_Compound.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <Quantity_NameOfColor.hxx>
#include <Quantity_Color.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <Precision.hxx>

#include <QBoxLayout>
#include <QElapsedTimer>
#include <QHash>
#include <QLabel>
#include <QMutex>
#include <QPainter>
#include <QPointer>
#include <QSet>
#include <QSlider>
#include <QTimer>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <vector>

namespace {

enum class NodeKind { Rapid, LeadIn, Cutting };

QString collisionAxisSourceId(const QString& axisName)
{
    return QStringLiteral("axis:") + axisName.trimmed().toUpper();
}

struct SimulationNode {
    NodeKind kind{NodeKind::Cutting};
    std::uint64_t contourId{0};
    std::array<double, lcnc::MachineAxisLayout::kMaxAxes> axes{};
    std::uint8_t mask{0};
    double durationMs{1.0};
    // The solved CAM TCP and outward surface normal are deliberately retained
    // alongside axes.  The independent simulation cutter is positioned from
    // these values and never feeds a pose back into CAM or Process.
    double tcpX{0.0};
    double tcpY{0.0};
    double tcpZ{0.0};
    double normalX{0.0};
    double normalY{0.0};
    double normalZ{1.0};
};

/// Collision geometry is deliberately decomposed more finely than its view
/// presentation. A machine label may be a large Compound, but treating that
/// Compound as one collision primitive is what made an axis' maximum bounding
/// box appear to be a collision with an unrelated child part.
/// 中文翻译：碰撞几何比视图显示更细地分解；大 Compound 不能作为一个碰撞原语。
struct CollisionLeaf {
    TopoDS_Shape shape;
    Bnd_Box localAabb;
    Bnd_OBB localObb;
};

QVector<CollisionLeaf> buildCollisionLeaves(const TopoDS_Shape& source);

struct CollisionGeometry {
    QVector<CollisionLeaf> leaves;
    Bnd_Box groupAabb;
    Bnd_OBB groupObb;
};

struct SimulationBody {
    QString id;
    QString attachment;
    QString collisionRole;
    QString collisionSource;
    bool workpiece{false};
    bool collisionActive{false};
    bool collisionPassive{false};
    bool cutterProxy{false};
    TopoDS_Shape shape;
    QVector<CollisionLeaf> collisionLeaves;
    Bnd_Box collisionGroupAabb;
    Bnd_OBB collisionGroupObb;
    Handle(AIS_Shape) ais;
};

CollisionGeometry buildCollisionGeometry(const TopoDS_Shape& source)
{
    CollisionGeometry result;
    if (source.IsNull())
        return result;
    // Keep a private immutable topology and calculate bounds directly from
    // BRep. Collision workers therefore never create or share mesh caches.
    // 中文翻译：碰撞几何使用私有只读拓扑，并直接按 BRep 计算包围盒，不共享网格缓存。
    BRepBuilderAPI_Copy copy(source, Standard_True, Standard_True);
    const TopoDS_Shape collisionShape = copy.IsDone() ? copy.Shape() : TopoDS_Shape{};
    if (collisionShape.IsNull())
        return result;
    BRepBndLib::AddOptimal(collisionShape, result.groupAabb, Standard_False, Standard_False);
    BRepBndLib::AddOBB(collisionShape, result.groupObb, Standard_False, Standard_False,
                        Standard_False);
    result.leaves = buildCollisionLeaves(collisionShape);
    return result;
}

QVector<CollisionLeaf> buildCollisionLeaves(const TopoDS_Shape& source)
{
    QVector<CollisionLeaf> leaves;
    if (source.IsNull())
        return leaves;

    TopTools_MapOfShape seen;
    const auto append = [&leaves, &seen](const TopoDS_Shape& shape) {
        if (shape.IsNull() || !seen.Add(shape))
            return;
        CollisionLeaf leaf;
        leaf.shape = shape;
        BRepBndLib::AddOptimal(shape, leaf.localAabb, Standard_False, Standard_False);
        BRepBndLib::AddOBB(shape, leaf.localObb, Standard_False, Standard_False,
                            Standard_False);
        if (!leaf.localAabb.IsVoid() && !leaf.localObb.IsVoid())
            leaves.append(std::move(leaf));
    };

    for (TopExp_Explorer explorer(source, TopAbs_SOLID); explorer.More(); explorer.Next())
        append(explorer.Current());
    if (!leaves.isEmpty())
        return leaves;
    for (TopExp_Explorer explorer(source, TopAbs_SHELL); explorer.More(); explorer.Next())
        append(explorer.Current());
    if (leaves.isEmpty())
        append(source);
    return leaves;
}

Bnd_Box transformAabb(const Bnd_Box& local, const gp_Trsf& trsf, double gapMm)
{
    Bnd_Box result;
    if (local.IsVoid())
        return result;
    Standard_Real xmin = 0.0, ymin = 0.0, zmin = 0.0, xmax = 0.0, ymax = 0.0, zmax = 0.0;
    local.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    for (const double x : {xmin, xmax}) for (const double y : {ymin, ymax})
        for (const double z : {zmin, zmax}) {
            gp_Pnt point(x, y, z);
            point.Transform(trsf);
            result.Add(point);
        }
    result.SetGap(gapMm);
    return result;
}

Bnd_OBB transformObb(const Bnd_OBB& local, const gp_Trsf& trsf, double gapMm)
{
    if (local.IsVoid())
        return {};
    gp_Pnt center(local.Center());
    center.Transform(trsf);
    gp_Dir x(local.XDirection()); x.Transform(trsf);
    gp_Dir y(local.YDirection()); y.Transform(trsf);
    gp_Dir z(local.ZDirection()); z.Transform(trsf);
    Bnd_OBB result(center, x, y, z, local.XHSize(), local.YHSize(), local.ZHSize());
    result.Enlarge(gapMm);
    return result;
}

/// Immutable-at-session-entry presentation data.  TopoDS topology is shared
/// read-only (as in the main view); all mutable AIS, locations, kinematics and
/// renderer state remain private to SimulationSession.
struct SimulationSceneSnapshot {
    LaserToolpath toolpath;
    QVector<lcnc::view::TravelPathRenderer::Segment> rapidSegments;
    gp_Pnt cutterHeadModelPosition{0.0, 0.0, 0.0};
    TopoDS_Shape cutterProxy;
    bool showToolpath{true};
    bool showTravel{false};
    bool showNormals{false};
    bool showMachine{true};
    QSet<QString> visibleMachineEntries;
    bool showRotaryGuides{true};
    bool showCutterHeadGuide{true};
    double normalSampleStep{2.0};
    Handle(Graphic3d_Camera) camera;
};

class CollisionTimeline final : public QWidget
{
public:
    explicit CollisionTimeline(QWidget* parent = nullptr) : QWidget(parent) { setMinimumHeight(14); }
    void setStates(QVector<qint8> states) { m_states = std::move(states); update(); }
protected:
    void paintEvent(QPaintEvent*) override {
        QPainter painter(this);
        painter.fillRect(rect(), QColor(105, 105, 105));
        if (m_states.isEmpty() || width() <= 0) return;
        const double step = static_cast<double>(width()) / m_states.size();
        for (int i = 0; i < m_states.size(); ++i) {
            const qint8 state = m_states.at(i);
            const QColor color = state == 1 ? QColor(210, 55, 55)       // confirmed intersection
                               : state == 2 ? QColor(224, 179, 42)      // clearance warning
                               : state == 3 ? QColor(224, 125, 42)      // indeterminate
                               : state == 0 ? QColor(55, 160, 90)       // safe
                                            : QColor(115, 115, 115);    // pending/disabled
            const int left = qRound(i * step);
            const int right = qMax(left + 1, qRound((i + 1) * step));
            painter.fillRect(QRect(left, 0, right - left, height()), color);
        }
    }
private:
    QVector<qint8> m_states; // -1 pending, 0 safe, 1 intersection, 2 clearance, 3 indeterminate
};

TopoDS_Shape transformed(const TopoDS_Shape& shape, const gp_Trsf& trsf)
{
    if (shape.IsNull()) return {};
    BRepBuilderAPI_Transform transform(shape, trsf, Standard_False);
    return transform.IsDone() ? transform.Shape() : TopoDS_Shape{};
}

gp_Trsf cutterProxyTransform(const SimulationNode& node, const gp_Pnt& headTip)
{
    gp_Vec normal(node.normalX, node.normalY, node.normalZ);
    if (normal.SquareMagnitude() <= 1e-16)
        normal = gp_Vec(0.0, 0.0, 1.0);
    normal.Normalize();
    // The visual and collision cutter follows the solved machine pose exactly.
    // CAM has already included any rapid offset before solving these axes.
    const gp_Pnt tip = headTip;
    // The shared CAM proxy has its tip at local origin and longitudinal axis
    // along +Z.  Map that local frame directly onto the solved TCP frame.
    gp_Trsf trsf;
    trsf.SetDisplacement(gp_Ax3(), gp_Ax3(tip, gp_Dir(normal)));
    return trsf;
}

gp_Pnt cutterHeadWorldPosition(const MachineKinematics& kinematics,
                               const gp_Pnt& modelPosition)
{
    gp_Pnt position = modelPosition;
    position.Transform(kinematics.computeAxisTransform(QStringLiteral("Z")));
    return position;
}

} // namespace

namespace lcnc::simulation {

class SimulationSession final
{
public:
    explicit SimulationSession(SimulationModule* owner) : m_owner(owner) {}
    ~SimulationSession() { close(); }

    QWidget* open(QString* error)
    {
        QElapsedTimer openTimer;
        openTimer.start();
        auto* cam = lcnc::Kernel::current().service<CamModule>();
        auto toolpathProvider = lcnc::Kernel::current().service<lcnc::cam::ICamToolpathProvider>();
        if (!cam || !toolpathProvider || !cam->hasToolpath() || !cam->kinematics()) {
            if (error) *error = QObject::tr("A solved CAM toolpath and machine kinematics are required");
            return nullptr;
        }
        m_sourceRevision = cam->toolpathRevision();
        const auto snapshot = toolpathProvider->exportCommittedExecutionSnapshot();
        if (!snapshot.motionPlan.collision.complete
            || snapshot.motionPlan.collision.state
                == lcnc::cam::CollisionValidationState::Pending) {
            if (error) {
                *error = QObject::tr(
                    "CAM collision validation is still running; wait for the CAM task to finish before entering offline simulation.");
            }
            return nullptr;
        }
        QVector<lcnc::cam::ContourId> orderedContourIds;
        orderedContourIds.reserve(snapshot.contours.size());
        for (const auto& contour : snapshot.contours)
            orderedContourIds.append(contour.contourId);
        const qint64 snapshotMs = openTimer.elapsed();
        if (orderedContourIds.isEmpty() || snapshot.machineConfigurationFingerprint.isEmpty()) {
            if (error) *error = QObject::tr("The CAM contour sequence has not been solved for the current machine");
            return nullptr;
        }
        // A collision result makes a plan non-executable, but does not erase
        // its already solved rapid nodes.  The sandbox must retain those nodes
        // so an operator can replay the exact offending movement.  Only omit
        // rapid geometry when planning itself produced no transition.
        // 中文翻译：碰撞会使计划不可执行，但不应抹掉已求解空程；沙箱须回放异常运动，仅在规划本身没有过渡节点时省略空程。
        const bool includeRapid = !snapshot.travelPlan.stale
            && (snapshot.travelPlan.failureReason.isEmpty()
                || !snapshot.travelPlan.transitions.isEmpty());
        captureSceneSnapshot(cam, snapshot);
        m_layout = snapshot.machineAxisLayout;
        const auto* machineConfig = lcnc::Kernel::current()
            .service<lcnc::MachineConfigurationService>();
        const lcnc::MachineModeDefinition definition = machineConfig
            ? machineConfig->modeDefinition(snapshot.machiningMode)
            : lcnc::MachineModeDefinition{};
        copyKinematics(cam->kinematics(), definition);
        if (!buildNodes(snapshot, orderedContourIds, includeRapid, error) || m_nodes.isEmpty()) {
            if (error && error->isEmpty())
                *error = QObject::tr("The current CAM sequence has no resolved cutting nodes");
            return nullptr;
        }
        m_page = new QWidget;
        auto* root = new QVBoxLayout(m_page);
        root->setContentsMargins(0, 0, 0, 0);
        root->setSpacing(3);
        m_gui = new GuiDocument(m_page);
        m_view = new WidgetOccView(m_page);
        // The sandbox is for machine-motion inspection, not CAD face editing.
        // Mode 0 selects a complete AIS component, matching the machine/file
        // view's component-level behaviour instead of default face picking.
        // 中文翻译：仿真用于机台运动检查，默认整体部件拾取，不使用 CAD 的面拾取。
        m_view->setCadSnapMode(WidgetOccView::CadSnapMode::None);
        root->addWidget(m_view, 1);
        m_slider = new QSlider(Qt::Horizontal, m_page);
        m_slider->setRange(0, m_nodes.size() - 1);
        root->addWidget(m_slider);
        m_timeline = new CollisionTimeline(m_page);
        root->addWidget(m_timeline);
        m_status = new QLabel(m_page);
        root->addWidget(m_status);
        m_view->attachDocument(m_gui);
        m_toolpathRenderer = std::make_unique<lcnc::view::ToolpathRenderer>();
        m_travelPathRenderer = std::make_unique<lcnc::view::TravelPathRenderer>();
        m_guideRenderer = std::make_unique<lcnc::view::MachineGuideRenderer>();
        collectBodies(cam);
        if (!m_collisionDetectionEnabled) {
            m_status->setText(QObject::tr("Collision detection is disabled; simulation playback is not verified."));
        } else if (!m_collisionConfigurationValid) {
            m_status->setText(QObject::tr("Collision detection is enabled but the active/passive source configuration is incomplete."));
        } else if (snapshot.motionPlan.collision.complete) {
            m_status->setText(QObject::tr("Showing collision validation completed by CAM."));
        } else {
            m_status->setText(includeRapid
                ? QObject::tr("CAM collision validation is pending; offline simulation will not start another scan.")
                : QObject::tr("Rapid travel is unavailable; simulating solved cutting path only. CAM collision validation is pending."));
        }
        // GuiDocument deliberately owns the same CAM-view rendering profile
        // and colour settings as the main machine view.  Submit the whole
        // projection as one batch: applying styles for every assembly part
        // separately is quadratic and made entering the tab visibly stall.
        // 中文翻译：仿真投影复用 CAM 视图外观；全部模型登记后一次性套用，避免逐件重复处理造成卡顿。
        m_gui->finalizeDisplayBatch();
        for (const SimulationBody& body : std::as_const(m_bodies)) {
            if (body.cutterProxy)
                applyCutterProxyAppearance(body);
        }
        projectFrozenCamOverlays();
        refreshFrozenGuides();
        applyNode(0);
        QObject::connect(m_view, &WidgetOccView::selectionChanged, m_page, [this] {
            synchronizeAxisGroupSelection();
        });
        const auto invalidateFrozenSource = [this] {
            if (m_sourceInvalidated)
                return;
            m_sourceInvalidated = true;
            pause();
            if (m_status)
                m_status->setText(QObject::tr("CAM or machine data changed; exit and re-enter offline simulation"));
            // A frozen scan is valid only for the exact CAM/machine revision
            // used to create it.  Tell the host to close this independent
            // view immediately, releasing its graphics context and cache.
            if (m_owner) {
                SimulationModule* const owner = m_owner;
                QTimer::singleShot(0, owner, [owner] { emit owner->sessionInvalidated(); });
            }
        };
        // The page intentionally never hot-reloads.  Freeze CAM geometry,
        // render choices and machine calibration as one coherent scene, then
        // reject playback if any of those source inputs changes.
        QObject::connect(cam, &CamModule::machineWorkspaceChanged, m_page, invalidateFrozenSource);
        QObject::connect(cam, &CamModule::axisAssignmentsChanged, m_page, invalidateFrozenSource);
        QObject::connect(cam, &CamModule::toolpathGenerated, m_page, invalidateFrozenSource);
        QObject::connect(cam, &CamModule::toolpathCleared, m_page, invalidateFrozenSource);
        QObject::connect(cam, &CamModule::toolpathLayersChanged, m_page, invalidateFrozenSource);
        QObject::connect(cam, &CamModule::cutterCollisionConfigurationChanged, m_page, invalidateFrozenSource);
        QObject::connect(cam, &CamModule::collisionConfigurationChanged, m_page, invalidateFrozenSource);
        QObject::connect(cam, &CamModule::contourOrderTravelPlanRebuilt, m_page,
                         [invalidateFrozenSource](const QVector<std::uint64_t>&) { invalidateFrozenSource(); });
        // The simulation page is not visible until MainWindow adds it to the
        // central tabs.  WidgetOccView therefore defers its OCC window setup;
        // fit again on the next event-loop turn after that setup completed.
        // 中文翻译：仿真页加入中央标签后才有 OCC 窗口，下一事件循环重新附着并适配视图。
        QTimer::singleShot(0, m_page, [this] {
            if (!m_page || !m_view || !m_gui) return;
            m_view->attachDocument(m_gui);
            if (!m_sceneSnapshot.camera.IsNull() && !m_gui->view().IsNull())
                m_gui->view()->SetCamera(m_sceneSnapshot.camera);
            if (m_slider) applyNode(m_slider->value());
            // Collision validation is deliberately CAM-owned.  The sandbox
            // only renders the immutable CAM result and never starts a second
            // expensive scan that could disagree with the machining preflight.
            // 中文翻译：碰撞校验统一归 CAM 所有；沙箱只显示不可变结果，绝不重复启动可能与加工前检查不一致的扫描。
        });
        m_timer = new QTimer(m_page);
        m_timer->setInterval(16);
        QObject::connect(m_timer, &QTimer::timeout, m_page, [this] { tick(); });
        QObject::connect(m_slider, &QSlider::valueChanged, m_page, [this](int value) {
            // A user seek pauses playback, but a timer-driven step must keep
            // the timer alive.  They share the same slider signal.
            // 中文翻译：手动拖动暂停播放；定时器推进滑块不能反过来停止定时器。
            if (!m_advancingPlayback) {
                m_playing = false;
                if (m_timer) m_timer->stop();
            }
            applyNode(value);
        });
        m_collisionStates.fill(-1, m_nodes.size());
        if (snapshot.motionPlan.collision.complete
            && snapshot.motionPlan.collision.nodeStates.size() == m_nodes.size()) {
            for (int index = 0; index < m_nodes.size(); ++index) {
                switch (snapshot.motionPlan.collision.nodeStates.at(index)) {
                case lcnc::cam::CollisionValidationState::Safe: m_collisionStates[index] = 0; break;
                case lcnc::cam::CollisionValidationState::Warning: m_collisionStates[index] = 2; break;
                case lcnc::cam::CollisionValidationState::Collision: m_collisionStates[index] = 1; break;
                case lcnc::cam::CollisionValidationState::Indeterminate: m_collisionStates[index] = 3; break;
                default: m_collisionStates[index] = -1; break;
                }
            }
            m_scanStarted = true;
            m_scanFinished = true;
        }
        m_timeline->setStates(m_collisionStates);
        LCNC_INFO(lcnc::LogCode::Generic,
                  "Offline simulation prepared: nodes={}, rapidSegmentsIncluded={}, machineBodies={}, workpieceBodies={}, mountedWorkpieces={}, snapshotMs={}, projectionMs={}",
                  m_nodes.size(), includeRapid, m_machineBodyCount, m_workpieceBodyCount,
                  m_mountedWorkpieceBodyCount, snapshotMs, openTimer.elapsed() - snapshotMs);
        return m_page;
    }

    QWidget* page() const { return m_page; }
    bool isOpen() const { return m_page != nullptr; }
    void run() {
        auto* cam = lcnc::Kernel::current().service<CamModule>();
        if (m_sourceInvalidated || (cam && cam->toolpathRevision() != m_sourceRevision)) {
            m_scanFinished = false;
            if (m_status) m_status->setText(QObject::tr("CAM data changed; exit and re-enter offline simulation"));
            return;
        }
        if (m_page && !m_nodes.isEmpty()) {
            m_playing = true;
            if (m_status) m_status->setText(m_scanFinished
                    ? QObject::tr("Running at %1x").arg(m_speed, 0, 'g', 3)
                    : QObject::tr("Running at %1x; CAM collision validation is pending").arg(m_speed, 0, 'g', 3));
            m_timer->start();
        }
    }
    void pause() { m_playing = false; if (m_timer) m_timer->stop(); }
    void stopPlayback() { pause(); if (m_slider) m_slider->setValue(0); }
    void setSpeed(double value) {
        m_speed = qBound(0.1, value, 10.0);
        if (m_timer) m_timer->setInterval(qMax(1, qRound(16.0 / m_speed)));
    }
    void jumpCollision(int direction) {
        if (!m_slider) return;
        const int start = m_slider->value();
        for (int delta = 1; delta < m_collisionStates.size(); ++delta) {
            const int index = (start + direction * delta + m_collisionStates.size()) % m_collisionStates.size();
            const qint8 state = m_collisionStates.at(index);
            if (state == 1 || state == 3) { m_slider->setValue(index); return; }
        }
    }
    void close()
    {
        pause();
        if (m_scanTask != kInvalidTaskId) {
            if (auto* tasks = lcnc::Kernel::current().taskManager()) tasks->requestAbort(m_scanTask);
            m_scanTask = kInvalidTaskId;
        }
        if (m_collisionPreparationTask != kInvalidTaskId) {
            if (auto* tasks = lcnc::Kernel::current().taskManager()) tasks->requestAbort(m_collisionPreparationTask);
            m_collisionPreparationTask = kInvalidTaskId;
        }
        ++m_generation;
        if (m_guideRenderer && m_gui)
            m_guideRenderer->erase(m_gui);
        if (m_toolpathRenderer && m_gui)
            m_toolpathRenderer->erase(m_gui);
        if (m_travelPathRenderer && m_gui)
            m_travelPathRenderer->erase(m_gui);
        if (m_gui && m_gui->scene()) {
            for (const auto& body : std::as_const(m_bodies))
                if (!body.ais.IsNull()) m_gui->scene()->removeShape(body.ais, false);
        }
        m_bodies.clear();
        m_toolpathRenderer.reset();
        m_travelPathRenderer.reset();
        m_guideRenderer.reset();
        m_sceneSnapshot = {};
        if (m_page) { delete m_page; m_page = nullptr; }
        m_gui = nullptr; m_view = nullptr; m_slider = nullptr; m_timeline = nullptr; m_status = nullptr; m_timer = nullptr;
    }

private:
    void captureSceneSnapshot(CamModule* cam,
                              const lcnc::cam::ToolpathExportSnapshot& snapshot)
    {
        m_sceneSnapshot.toolpath = cam->toolpath();
        m_sceneSnapshot.cutterHeadModelPosition = cam->cutterHeadModelPosition();
        m_sceneSnapshot.showToolpath = cam->isToolpathVisible();
        m_sceneSnapshot.showTravel = cam->isTravelPathVisible();
        m_sceneSnapshot.showNormals = cam->showNormals();
        m_sceneSnapshot.showMachine = cam->isMachineModelVisible();
        const QStringList visibleMachineEntries = cam->visibleMachineEntries();
        m_sceneSnapshot.visibleMachineEntries = QSet<QString>(
            visibleMachineEntries.cbegin(), visibleMachineEntries.cend());
        m_sceneSnapshot.showRotaryGuides = cam->rotaryAxisGuidesVisible();
        m_sceneSnapshot.showCutterHeadGuide = cam->cutterHeadGuideVisible();
        m_sceneSnapshot.normalSampleStep = cam->normalSampleStep();
        if (GuiDocument* sourceView = cam->activeGuiDocument(); sourceView && !sourceView->view().IsNull()) {
            m_sceneSnapshot.camera = new Graphic3d_Camera();
            m_sceneSnapshot.camera->Copy(sourceView->view()->Camera());
        }
        QString proxyError;
        m_sceneSnapshot.cutterProxy = cam->cutterCollisionProxyShape(&proxyError);
        if (m_sceneSnapshot.cutterProxy.IsNull() && !proxyError.isEmpty()) {
            LCNC_WARN(lcnc::LogCode::Generic,
                      "Offline simulation scene snapshot has no cutter proxy: {}",
                      proxyError.toStdString());
        }

        QHash<std::uint64_t, const lcnc::cam::ToolpathExportContour*> contours;
        for (const auto& contour : snapshot.contours)
            contours.insert(contour.contourId, &contour);
        for (const auto& transition : snapshot.travelPlan.transitions) {
            const auto* source = contours.value(transition.fromContourId, nullptr);
            if (!source || transition.surfacePreviewPoints.size() < 2)
                continue;
            lcnc::view::TravelPathRenderer::Segment segment;
            segment.contourId = transition.toContourId;
            segment.workpieceEntry = source->workpieceEntry;
            segment.verified = snapshot.travelPlan.isExecutable();
            const auto& points = transition.workpieceLocalPreviewPoints.isEmpty()
                ? transition.surfacePreviewPoints : transition.workpieceLocalPreviewPoints;
            for (int index = 0; index < points.size(); ++index) {
                const auto& point = points.at(index);
                const auto phase = index > 0 && index - 1 < transition.segments.size()
                    ? transition.segments.at(index - 1).phase
                    : lcnc::cam::RapidSegmentPhase::Traverse;
                segment.waypoints.append({point.x, point.y, point.z, phase});
            }
            m_sceneSnapshot.rapidSegments.append(std::move(segment));
        }
    }

    bool buildNodes(const lcnc::cam::ToolpathExportSnapshot& snapshot,
                    const QVector<lcnc::cam::ContourId>& order,
                    bool includeRapid,
                    QString* error)
    {
        // CAM publishes the complete final-coordinate motion sequence.  Do
        // not reconstruct a different path in the sandbox when it is present.
        if (!snapshot.motionPlan.nodes.isEmpty()) {
            for (const auto& item : snapshot.motionPlan.nodes) {
                if (!includeRapid && item.phase == lcnc::cam::CamMotionPhase::Rapid)
                    continue;
                SimulationNode node;
                node.kind = item.phase == lcnc::cam::CamMotionPhase::Rapid ? NodeKind::Rapid
                    : item.phase == lcnc::cam::CamMotionPhase::LeadIn ? NodeKind::LeadIn
                    : NodeKind::Cutting;
                node.contourId = item.contourId;
                node.axes = item.axes;
                node.mask = item.axisMask;
                node.durationMs = qMax(1.0, item.estimatedTimeMs);
                node.tcpX = item.tcpX; node.tcpY = item.tcpY; node.tcpZ = item.tcpZ;
                node.normalX = item.normalX; node.normalY = item.normalY; node.normalZ = item.normalZ;
                m_nodes.append(std::move(node));
            }
            return !m_nodes.isEmpty();
        }
        const QList<MachineAxisDef> projectionBaselineAxes = m_planningBaselineAxes;
        QHash<std::uint64_t, const lcnc::cam::ToolpathExportContour*> contourById;
        for (const auto& contour : snapshot.contours) contourById.insert(contour.contourId, &contour);
        for (const auto id : order) {
            const auto* contour = contourById.value(id, nullptr);
            const auto points = snapshot.pointsByContourId.value(id);
            if (!contour || points.isEmpty()) continue;
            // Legacy snapshots follow the same boundary as the canonical
            // plan: node zero is the first lead-in/cutting point.  Never
            // replay a transition into the first simulated contour.
            // 中文翻译：旧快照同样从首轮廓下刀/切割首点开始，不回放进入首轮廓的空程。
            if (includeRapid && !m_nodes.isEmpty()) {
                const auto* transition = snapshot.travelPlan.transitionTo(id);
                if (transition) {
                    // RapidMoveSegment stores targets only.  Insert the matching
                    // source pose once so the virtual nozzle follows the complete
                    // offset rapid curve from its first millimetre, rather than
                    // jumping from the preceding cutting TCP to the first target.
                    SimulationNode source = m_nodes.constLast();
                    source.kind = NodeKind::Rapid;
                    source.contourId = id;
                    source.durationMs = 1.0;
                    m_nodes.append(source);
                    for (const auto& segment : transition->segments) {
                        SimulationNode node; node.kind = NodeKind::Rapid; node.contourId = id;
                        node.axes = segment.target.kinematicAxes; node.mask = segment.target.kinematicAxisMask;
                        node.durationMs = qMax(1.0, segment.estimatedTimeMs);
                        node.tcpX = segment.target.tcpX; node.tcpY = segment.target.tcpY;
                        node.tcpZ = segment.target.tcpZ;
                        node.normalX = segment.target.surfaceNormalX;
                        node.normalY = segment.target.surfaceNormalY;
                        node.normalZ = segment.target.surfaceNormalZ;
                        m_nodes.append(node);
                    }
                }
            }
            const QString workpieceEntry = contour->workpieceEntry;
            const auto append = [this, id, workpieceEntry, projectionBaselineAxes](
                                    const lcnc::cam::ToolpathExportPoint& point,
                                    NodeKind kind) {
                if (!point.machineCoordValid) return false;
                SimulationNode node; node.kind = kind; node.contourId = id;
                node.axes = point.machineAxes; node.mask = point.machineAxisMask;
                node.durationMs = 20.0;
                lcnc::cam::CamMotionNode geometry;
                geometry.tcpX = point.x; geometry.tcpY = point.y; geometry.tcpZ = point.z;
                geometry.normalX = point.normalX;
                geometry.normalY = point.normalY;
                geometry.normalZ = point.normalZ;
                lcnc::cam_algo::applyOfflineMotionPose(
                    &m_kinematics, projectionBaselineAxes, m_layout,
                    point.machineAxes, point.machineAxisMask);
                lcnc::cam_algo::transformMotionNodeGeometry(
                    &geometry, m_kinematics.computeWpcTransform(workpieceEntry));
                node.tcpX = geometry.tcpX; node.tcpY = geometry.tcpY; node.tcpZ = geometry.tcpZ;
                node.normalX = geometry.normalX;
                node.normalY = geometry.normalY;
                node.normalZ = geometry.normalZ;
                m_nodes.append(node); return true;
            };
            if (contour->hasLeadIn && contour->leadInPoint.machineCoordValid && !append(contour->leadInPoint, NodeKind::LeadIn)) {
                if (error) *error = QObject::tr("The lead-in has no solved machine coordinates"); return false;
            }
            for (const auto& point : points) if (!append(point, NodeKind::Cutting)) {
                if (error) *error = QObject::tr("The CAM toolpath contains unresolved machine coordinates"); return false;
            }
        }
        return true;
    }

    void copyKinematics(const MachineKinematics* source,
                        const lcnc::MachineModeDefinition& definition)
    {
        if (!source)
            return;
        m_planningBaselineAxes = lcnc::cam_algo::offlinePlanningAxisBaseline(
            source->axes(), definition);
        m_kinematics.setAxes(m_planningBaselineAxes, source->configType());
        m_kinematics.setWorkpieceSetupTransform(source->workpieceSetupTransform());
        for (auto it = source->shapeAssignments().cbegin(); it != source->shapeAssignments().cend(); ++it)
            m_kinematics.assignShape(it.key(), it.value());
        for (auto it = source->wpcMounts().cbegin(); it != source->wpcMounts().cend(); ++it)
            m_kinematics.mountWorkpiece(it.key(), it.value());
    }

    void collectBodies(CamModule* cam)
    {
        const auto collision = cam->collisionConfiguration();
        m_collisionDetectionEnabled = collision.enabled;
        m_collisionConfigurationValid = collision.valid;
        m_collisionConfigurationRevision = collision.revision;
        const auto add = [this, cam, collision](LcncDocument* document, LcncDocument::EntityKind kind, bool workpiece) {
            if (!document || !document->shapeTool()) return;
            const auto labels = document->entityLabels(kind);
            for (int index = 1; index <= labels.Length(); ++index) {
                const TDF_Label& label = labels.Value(index);
                const TopoDS_Shape shape = document->shapeTool()->GetShape(label);
                if (shape.IsNull()) continue;
                SimulationBody body;
                body.id = XcafUtils::entry(label); body.workpiece = workpiece; body.shape = shape;
                body.attachment = workpiece ? m_kinematics.mountedAxis(body.id) : m_kinematics.axisForShape(body.id);
                if (workpiece) {
                    ++m_workpieceBodyCount;
                    if (!body.attachment.isEmpty()) ++m_mountedWorkpieceBodyCount;
                } else {
                    ++m_machineBodyCount;
                }
                body.collisionSource = workpiece ? QStringLiteral("workpiece")
                    : collisionAxisSourceId(body.attachment);
                body.collisionRole = workpiece ? QStringLiteral("workpiece")
                    : (body.attachment == QStringLiteral("BASE")
                        ? QStringLiteral("static") : QStringLiteral("moving"));
                body.collisionActive = m_collisionDetectionEnabled
                    && collision.activeSources.contains(body.collisionSource);
                body.collisionPassive = m_collisionDetectionEnabled
                    && collision.passiveSources.contains(body.collisionSource);
                // Register the projection with GuiDocument instead of only
                // displaying it in GraphicsScene.  GuiDocument::fitAll()
                // intentionally frames registered objects, so bypassing it
                // left the sandbox camera with no model bounds to fit.
                // 中文翻译：投影必须登记到 GuiDocument，否则 FitAll 无法取得模型包围盒。
                body.ais = m_gui->displayShape(
                    workpiece ? lcnc::ProjectDomain::Workpiece : lcnc::ProjectDomain::Machine,
                    document, static_cast<int>(kind), shape, body.id, false,
                    /*deferPresentationUpdate=*/true);
                if (!body.ais.IsNull() && !body.ais->Attributes().IsNull()) {
                    // The main document view displays imported model meshes
                    // prepared by CadDocumentIoService and explicitly turns
                    // off AIS lazy triangulation.  The sandbox used the
                    // generic display path, leaving it enabled; its rendering
                    // profile could therefore replace circles with a coarse
                    // per-view mesh.  Reuse the prepared source mesh exactly
                    // as the file/machine view does.  This affects rendering
                    // only; collision still operates on the untouched BRep.
                    // 中文翻译：仿真视图复用文件/机台视图已准备好的显示网格，
                    // 禁止 AIS 按本视图质量配置重新粗略三角化；碰撞仍使用原始 BRep。
                    body.ais->Attributes()->SetAutoTriangulation(Standard_False);
                    body.ais->Attributes()->SetIsoOnTriangulation(Standard_False);
                    body.ais->Attributes()->SetFaceBoundaryDraw(Standard_False);
                }
                if (!workpiece && m_gui->context()
                    && (!m_sceneSnapshot.showMachine
                        || !m_sceneSnapshot.visibleMachineEntries.contains(body.id))) {
                    m_gui->context()->Erase(body.ais, Standard_False);
                }
                m_bodies.append(std::move(body));
            }
        };
        add(cam->machineDocument(), LcncDocument::EntityKind::Machine, false);
        if (auto* project = lcnc::Kernel::current().projectManager())
            add(project->workpieceDocument(), LcncDocument::EntityKind::Workpiece, true);

        // The cutter/nozzle is not necessarily part of the imported machine
        // assembly.  Project CAM's exact local +Z collision proxy as an
        // independent sandbox-only body, so it is visible and participates in
        // the same scan as the configured machine/workpiece collision roles.
        // 中文翻译：将 CAM 的刀嘴/模拟锥代理作为沙箱私有部件投影并参与碰撞，
        // 不修改机台文档或主视图。
        const TopoDS_Shape& proxy = m_sceneSnapshot.cutterProxy;
        if (proxy.IsNull()) {
            LCNC_WARN(lcnc::LogCode::Generic,
                      "Offline simulation cutter proxy is unavailable");
        } else {
            // Match MachineGuideRenderer: generate the proxy's presentation
            // mesh once, then disable per-view lazy meshing below.  This keeps
            // the visible cone/nozzle deterministic without remeshing it on
            // every sandbox entry.
            BRepMesh_IncrementalMesh(proxy, 0.5);
            SimulationBody body;
            body.id = QStringLiteral("__offline_simulation_cutter_proxy__");
            body.collisionRole = QStringLiteral("cutter");
            body.collisionSource = QStringLiteral("cutter");
            body.cutterProxy = true;
            body.collisionActive = m_collisionDetectionEnabled
                && collision.activeSources.contains(body.collisionSource);
            body.shape = proxy;
            if (body.attachment.isEmpty())
                body.attachment = QStringLiteral("Z");
            // The visible cutter is owned by the same MachineGuideRenderer
            // used in the ordinary machine view.  Keep this BRep collision
            // only, otherwise two independently transformed nozzle visuals
            // would appear in the sandbox.
            m_bodies.append(std::move(body));
        }
        if (m_workpieceBodyCount == 0) {
            LCNC_WARN(lcnc::LogCode::Generic,
                      "Offline simulation has no project workpiece to project; machine model remains available");
        }
    }

    void setPose(const SimulationNode& node)
    {
        lcnc::cam_algo::applyOfflineMotionPose(
            &m_kinematics, m_planningBaselineAxes, m_layout,
            node.axes, node.mask);
    }

    gp_Trsf bodyTransform(const SimulationBody& body, const SimulationNode& node) const
    {
        if (body.cutterProxy)
            return cutterProxyTransform(node, cutterHeadWorldPosition());
        return body.workpiece ? m_kinematics.computeWpcTransform(body.id)
                              : m_kinematics.computeShapeTransform(body.id);
    }

    gp_Pnt cutterHeadWorldPosition() const
    {
        // Keep this exactly aligned with CamModule::cutterHeadWorldPosition:
        // the frozen model-space tip rides the virtual Z-axis chain.  Unlike
        // CAM, this method only reads the session's private kinematics copy.
        return ::cutterHeadWorldPosition(m_kinematics,
                                         m_sceneSnapshot.cutterHeadModelPosition);
    }

    void projectFrozenCamOverlays()
    {
        if (!m_gui || !m_toolpathRenderer || !m_travelPathRenderer)
            return;

        // Project the frozen contour wires as CAM-domain AIS objects.  The
        // same ToolpathRenderer then supplies lead-ins/normals, and the same
        // TravelPathRenderer supplies the dashed rapid curve as the file view.
        // 中文翻译：冻结轮廓按 CAM 域投影，再复用正式 View 的引入线、法线和空程渲染器。
        for (int index = 0; index < m_sceneSnapshot.toolpath.contourCount(); ++index) {
            const LaserContour& contour = m_sceneSnapshot.toolpath.contour(index);
            if (contour.contourId == 0 || contour.points.size() < 2)
                continue;
            const QString name = contour.name.isEmpty()
                ? QObject::tr("Outline %1").arg(index + 1) : contour.name;
            const Handle(AIS_Shape) ais = m_gui->displayContourBody(
                contour.contourId,
                LaserToolpathBuilder::buildOffsetDisplayShape(contour), name,
                /*updateViewer=*/false, /*configureSelection=*/false);
            if (ais.IsNull())
                continue;
            const auto layer = std::find_if(m_sceneSnapshot.toolpath.layers().cbegin(),
                                            m_sceneSnapshot.toolpath.layers().cend(),
                                            [&contour](const ToolpathLayer& value) {
                                                return value.layerId == contour.layerId;
                                            });
            if (layer != m_sceneSnapshot.toolpath.layers().cend() && layer->color.isValid()) {
                const QColor color = layer->color;
                ais->SetColor(Quantity_Color(color.redF(), color.greenF(), color.blueF(),
                                              Quantity_TOC_RGB));
            }
            if (m_gui->context()) {
                m_gui->context()->Deactivate(ais);
                if (m_sceneSnapshot.showToolpath && contour.enabled)
                    m_gui->context()->Display(ais, Standard_False);
                else
                    m_gui->context()->Erase(ais, Standard_False);
            }
        }

        m_toolpathRenderer->setShowNormals(m_sceneSnapshot.showNormals);
        m_toolpathRenderer->setNormalSampleStep(m_sceneSnapshot.normalSampleStep);
        m_toolpathRenderer->setVisible(m_gui, m_sceneSnapshot.showToolpath);
        m_toolpathRenderer->refresh(m_gui, m_sceneSnapshot.toolpath, &m_kinematics);
        m_travelPathRenderer->setVisible(m_sceneSnapshot.showTravel);
        m_travelPathRenderer->refresh(m_gui, m_sceneSnapshot.rapidSegments);
        updateFrozenCamOverlayTransforms();
    }

    void updateFrozenCamOverlayTransforms()
    {
        if (!m_gui)
            return;
        const Handle(AIS_InteractiveContext)& context = m_gui->context();
        if (!context.IsNull()) {
            for (int index = 0; index < m_sceneSnapshot.toolpath.contourCount(); ++index) {
                const LaserContour& contour = m_sceneSnapshot.toolpath.contour(index);
                const Handle(AIS_Shape) ais = m_gui->aisShapeForContour(contour.contourId);
                if (ais.IsNull())
                    continue;
                ais->SetLocalTransformation(m_kinematics.computeWpcTransform(contour.workpieceEntry));
                context->RecomputePrsOnly(ais, Standard_False);
            }
        }
        if (m_toolpathRenderer)
            m_toolpathRenderer->updateTransforms(m_gui, m_sceneSnapshot.toolpath, &m_kinematics);
        if (m_travelPathRenderer)
            m_travelPathRenderer->updateTransforms(m_gui, &m_kinematics);
    }

    void refreshFrozenGuides()
    {
        if (!m_guideRenderer || !m_gui)
            return;
        if (const auto* settings = lcnc::Kernel::current().appSettings()) {
            m_guideRenderer->setCutterHeadAppearance({settings->colors.cutterHeadColor,
                                                       settings->colors.cutterHeadTransparency,
                                                       settings->colors.cutterHeadScale});
        }
        m_guideRenderer->setCutterCollisionProxy(m_sceneSnapshot.cutterProxy);
        m_guideRenderer->refresh(m_gui, &m_kinematics, cutterHeadWorldPosition());
        m_guideRenderer->setRotaryAxisVisible(m_gui, m_sceneSnapshot.showRotaryGuides);
        m_guideRenderer->setCutterHeadVisible(m_gui, m_sceneSnapshot.showCutterHeadGuide);
    }

    void applyCutterProxyAppearance(const SimulationBody& body) const
    {
        if (body.ais.IsNull())
            return;
        QColor color(255, 0, 0);
        double transparency = 0.0;
        if (const auto* settings = lcnc::Kernel::current().appSettings()) {
            color = settings->colors.cutterHeadColor;
            transparency = settings->colors.cutterHeadTransparency;
        }
        body.ais->SetColor(Quantity_Color(color.redF(), color.greenF(), color.blueF(),
                                           Quantity_TOC_RGB));
        body.ais->SetTransparency(qBound(0.0, transparency, 1.0));
        if (!body.ais->Attributes().IsNull()) {
            body.ais->Attributes()->SetAutoTriangulation(Standard_False);
            body.ais->Attributes()->SetFaceBoundaryDraw(Standard_False);
        }
    }

    QString selectionGroupKey(const SimulationBody& body) const
    {
        // Keep an unmounted workpiece separate from static BASE components,
        // while parts mounted on the same motion axis behave as one group.
        // 中文翻译：未挂载工件不能并入 BASE；同一运动轴的部件作为一个选择组。
        return (body.workpiece ? QStringLiteral("workpiece:") : QStringLiteral("machine:"))
            + body.attachment;
    }

    void synchronizeAxisGroupSelection()
    {
        if (m_applyingAxisGroupSelection || !m_gui || !m_gui->context())
            return;

        const Handle(AIS_InteractiveContext)& ctx = m_gui->context();
        QString group;
        for (ctx->InitSelected(); ctx->MoreSelected() && group.isEmpty(); ctx->NextSelected()) {
            const AIS_InteractiveObject* selected = ctx->SelectedInteractive().get();
            for (const SimulationBody& body : std::as_const(m_bodies)) {
                if (!body.ais.IsNull() && body.ais.get() == selected) {
                    group = selectionGroupKey(body);
                    break;
                }
            }
        }
        if (group.isEmpty())
            return;

        m_applyingAxisGroupSelection = true;
        ctx->ClearSelected(Standard_False);
        for (const SimulationBody& body : std::as_const(m_bodies)) {
            if (!body.ais.IsNull() && selectionGroupKey(body) == group)
                ctx->AddOrRemoveSelected(body.ais, Standard_False);
        }
        ctx->UpdateCurrentViewer();
        if (m_gui->hasView())
            m_gui->view()->Redraw();
        m_applyingAxisGroupSelection = false;
    }

    void applyNode(int index)
    {
        if (index < 0 || index >= m_nodes.size()) return;
        setPose(m_nodes.at(index));
        for (auto& body : m_bodies) {
            const gp_Trsf trsf = bodyTransform(body, m_nodes.at(index));
            if (!body.ais.IsNull()) body.ais->SetLocalTransformation(trsf);
        }
        updateFrozenCamOverlayTransforms();
        if (m_guideRenderer)
            m_guideRenderer->updateTransforms(m_gui, &m_kinematics, cutterHeadWorldPosition());
        if (m_gui->context()) m_gui->context()->UpdateCurrentViewer();
        if (m_gui->hasView()) m_gui->view()->Redraw();
        showCollision(index);
    }
    void tick()
    {
        if (!m_slider || m_nodes.isEmpty()) return;
        const int next = m_slider->value() + 1;
        if (next >= m_nodes.size()) { pause(); return; }
        m_advancingPlayback = true;
        m_slider->setValue(next);
        m_advancingPlayback = false;
    }

    void startCollisionPreparation()
    {
        if (m_collisionPreparationStarted || m_collisionPreparationFinished)
            return;
        m_collisionPreparationStarted = true;
        auto* tasks = lcnc::Kernel::current().taskManager();
        if (!tasks) {
            m_collisionStates.fill(3, m_nodes.size());
            m_collisionPreparationFinished = true;
            if (m_timeline) m_timeline->setStates(m_collisionStates);
            if (m_status) m_status->setText(QObject::tr("Collision geometry preparation is unavailable"));
            return;
        }

        struct Source {
            TopoDS_Shape shape;
            bool needed{false};
        };
        QVector<Source> sources;
        sources.reserve(m_bodies.size());
        for (const SimulationBody& body : std::as_const(m_bodies))
            sources.append({body.shape, body.collisionActive || body.collisionPassive});
        const quint64 generation = m_generation;
        auto geometry = std::make_shared<QVector<CollisionGeometry>>(sources.size());
        TaskSpec spec;
        spec.label = QObject::tr("Preparing collision geometry");
        spec.scope = QStringLiteral("simulation.collision.prepare");
        spec.userVisible = true;
        spec.cancellable = true;
        m_collisionPreparationTask = tasks->run(spec,
            [sources, geometry](TaskProgress* progress) {
                progress->setRange(0, qMax(1, sources.size()));
                std::unique_lock<std::timed_mutex> scanLock(
                    lcnc::cam_algo::collisionScanExecutionMutex(), std::defer_lock);
                if (!lcnc::cam_algo::acquireCollisionScanExecution(scanLock, [progress]() {
                        return progress->isAbortRequested();
                    }))
                    return;
                for (int index = 0; index < sources.size(); ++index) {
                    if (progress->isAbortRequested())
                        return;
                    if (sources.at(index).needed)
                        geometry->operator[](index) = buildCollisionGeometry(sources.at(index).shape);
                    progress->setValue(index + 1);
                }
            });
        QObject::connect(tasks, &TaskManager::taskProgressChanged, m_page,
                         [this, generation](TaskId id, int percent) {
            if (id != m_collisionPreparationTask || generation != m_generation)
                return;
            if (m_status && !m_playing)
                m_status->setText(QObject::tr("Preparing collision geometry: %1%").arg(percent));
        });
        QObject::connect(tasks, &TaskManager::taskFinishedDetailed, m_page,
                         [this, generation, geometry](TaskId id, TaskExecutionStatus status, const QString&) {
            if (id != m_collisionPreparationTask || generation != m_generation)
                return;
            m_collisionPreparationTask = kInvalidTaskId;
            if (status != TaskExecutionStatus::Succeeded) {
                m_collisionStates.fill(3, m_nodes.size());
                if (m_timeline) m_timeline->setStates(m_collisionStates);
                if (m_status) m_status->setText(QObject::tr("Collision geometry preparation was cancelled or failed"));
                return;
            }
            for (int index = 0; index < m_bodies.size() && index < geometry->size(); ++index) {
                m_bodies[index].collisionLeaves = std::move(geometry->operator[](index).leaves);
                m_bodies[index].collisionGroupAabb = geometry->at(index).groupAabb;
                m_bodies[index].collisionGroupObb = geometry->at(index).groupObb;
            }
            m_collisionPreparationFinished = true;
            if (m_status) m_status->setText(QObject::tr("Collision geometry prepared; scanning collisions…"));
            startCollisionScan();
        });
    }

    void startCollisionScan()
    {
        if (!m_collisionDetectionEnabled) {
            m_scanStarted = true;
            m_scanFinished = true;
            if (m_status) m_status->setText(QObject::tr("Collision detection is disabled; simulation playback is not verified."));
            return;
        }
        if (!m_collisionConfigurationValid) {
            m_scanStarted = true;
            m_scanFinished = true;
            if (m_status) m_status->setText(QObject::tr("Collision source configuration is incomplete; no scan was started."));
            return;
        }
        if (!m_collisionPreparationFinished) {
            startCollisionPreparation();
            return;
        }
        m_scanStarted = true;
        auto* tasks = lcnc::Kernel::current().taskManager();
        if (!tasks) {
            m_collisionStates.fill(3, m_nodes.size());
            m_scanFinished = true;
            if (m_timeline) m_timeline->setStates(m_collisionStates);
            if (m_status) m_status->setText(QObject::tr("Collision scanner is unavailable; all positions are marked indeterminate"));
            return;
        }
        const auto nodes = m_nodes;
        QVector<SimulationBody> activeBodies;
        QVector<SimulationBody> passiveBodies;
        QVector<int> activeIndices;
        QVector<int> passiveIndices;
        QSet<QString> activeSourceSet;
        activeBodies.reserve(m_bodies.size());
        passiveBodies.reserve(m_bodies.size());
        for (int index = 0; index < m_bodies.size(); ++index) {
            const auto& body = m_bodies.at(index);
            if (body.collisionActive) {
                activeBodies.append(body);
                activeIndices.append(index);
                activeSourceSet.insert(body.collisionSource);
            }
            if (body.collisionPassive) { passiveBodies.append(body); passiveIndices.append(index); }
        }
        const auto axes = m_planningBaselineAxes;
        const auto config = m_kinematics.configType();
        const gp_Trsf workpieceSetup = m_kinematics.workpieceSetupTransform();
        const auto assignments = m_kinematics.shapeAssignments();
        const auto mounts = m_kinematics.wpcMounts();
        const auto layout = m_layout;
        const auto cutterHeadModelPosition = m_sceneSnapshot.cutterHeadModelPosition;
        const QStringList activeSources =
            lcnc::cam_algo::orderedActiveCollisionSources(activeSourceSet);
        const double clearanceMm = lcnc::Kernel::current().service<CamModule>()->config()
            .cutterCollisionClearanceMm();
        const quint64 generation = m_generation;
        // Pending nodes remain gray until the worker publishes their result.
        auto result = std::make_shared<QVector<qint8>>(nodes.size(), -1);
        auto pairs = std::make_shared<QVector<QPair<int, int>>>(nodes.size(), QPair<int, int>{-1, -1});
        auto resultMutex = std::make_shared<QMutex>();
        auto progressMutex = std::make_shared<QMutex>();
        auto completed = std::make_shared<std::atomic_int>(0);
        TaskSpec spec; spec.label = QObject::tr("Offline collision scan"); spec.scope = QStringLiteral("simulation.collision"); spec.userVisible = false;
        m_scanTask = tasks->run(spec, [nodes, activeBodies, passiveBodies, activeIndices, activeSources,
            passiveIndices, axes, config, workpieceSetup, assignments, mounts, layout,
            cutterHeadModelPosition, clearanceMm, result, pairs, resultMutex, progressMutex,
            completed](TaskProgress* progress) {
            const int scanSteps = nodes.size() * activeSources.size();
            progress->setRange(0, qMax(1, scanSteps));
            std::unique_lock<std::timed_mutex> scanLock(
                lcnc::cam_algo::collisionScanExecutionMutex(), std::defer_lock);
            if (!lcnc::cam_algo::acquireCollisionScanExecution(scanLock, [progress]() {
                    return progress->isAbortRequested();
                }))
                return;
            const int threadCount = qMin(4, qMax(1, OSD_Parallel::NbLogicalProcessors() / 2));
            auto workerKinematics = std::make_shared<std::vector<std::unique_ptr<MachineKinematics>>>();
            workerKinematics->reserve(static_cast<std::size_t>(threadCount));
            for (int worker = 0; worker < threadCount; ++worker) {
                auto kin = std::make_unique<MachineKinematics>();
                kin->setAxes(axes, config);
                kin->setWorkpieceSetupTransform(workpieceSetup);
                for (auto it = assignments.cbegin(); it != assignments.cend(); ++it) kin->assignShape(it.key(), it.value());
                for (auto it = mounts.cbegin(); it != mounts.cend(); ++it) kin->mountWorkpiece(it.key(), it.value());
                workerKinematics->push_back(std::move(kin));
            }
            Handle(OSD_ThreadPool) pool = new OSD_ThreadPool(threadCount);
            OSD_ThreadPool::Launcher launcher(*pool, threadCount);
            for (int phase = 0; phase < activeSources.size(); ++phase) {
                if (progress->isAbortRequested())
                    break;
                const QString activeSource = activeSources.at(phase);
                const bool finalPhase = phase + 1 == activeSources.size();
                launcher.Perform(0, nodes.size(), [nodes, activeBodies, passiveBodies,
                activeIndices, passiveIndices, axes, layout, cutterHeadModelPosition,
                activeSource, finalPhase, scanSteps, clearanceMm, result, pairs, resultMutex,
                progressMutex, completed, workerKinematics, progress](int threadIndex, int n) {
                if (progress->isAbortRequested()) return;
                // Every OCCT worker owns a distinct kinematics state and all
                // narrow phase instances below have their internal parallelism disabled.
                // 中文翻译：每个 OCCT 线程独占运动学副本；窄相算法关闭内部并行，避免嵌套线程池。
                MachineKinematics& kin = *workerKinematics->at(static_cast<std::size_t>(threadIndex));
                const auto& node = nodes.at(n);
                // OSD workers receive nodes out of sequence. Restore the
                // frozen entry pose before applying the node mask so omitted
                // extension axes never inherit an unrelated worker's previous
                // item. This matches the independent sandbox pose semantics.
                // 中文翻译：并行节点无固定顺序；每次先恢复冻结姿态，再按节点掩码应用轴值。
                for (const MachineAxisDef& axis : axes)
                    kin.setAxisPosition(axis.name, axis.currentPos);
                for (int axis = 0; axis < lcnc::MachineAxisLayout::kMaxAxes; ++axis)
                    if ((node.mask & (1u << axis)) != 0
                        && !layout.axes[axis].name.isEmpty())
                        kin.setAxisPosition(layout.axes[axis].name, node.axes[axis]);
                qint8 worstState = 0;
                int worstSeverity = 0;
                QPair<int, int> collisionPair{-1, -1};
                struct PoseLeaf {
                    const CollisionLeaf* leaf{nullptr};
                    gp_Trsf transform;
                    Bnd_Box aabb;
                    Bnd_OBB obb;
                    int bodyIndex{-1};
                    bool cutterProxy{false};
                    bool workpiece{false};
                };
                QVector<PoseLeaf> activeLeaves;
                QVector<PoseLeaf> passiveLeaves;
                struct PoseGroup {
                    int bodyIndex{-1};
                    int sourceIndex{-1};
                    gp_Trsf transform;
                    Bnd_Box aabb;
                    Bnd_OBB obb;
                };
                QVector<PoseGroup> activeGroups;
                QVector<PoseGroup> passiveGroups;
                const auto bodyTransformForNode = [&kin, &node, &cutterHeadModelPosition](const SimulationBody& body) {
                    const gp_Trsf trsf = body.cutterProxy ? cutterProxyTransform(
                        node, ::cutterHeadWorldPosition(kin, cutterHeadModelPosition))
                        : (body.workpiece ? kin.computeWpcTransform(body.id)
                                          : kin.computeShapeTransform(body.id));
                    return trsf;
                };
                const auto appendGroups = [&bodyTransformForNode, clearanceMm, &activeSource](const QVector<SimulationBody>& bodies,
                                                                                   const QVector<int>& bodyIndices,
                                                                                   bool activeSet,
                                                                                   QVector<PoseGroup>* target) {
                    for (int body = 0; body < bodies.size(); ++body) {
                        const SimulationBody& source = bodies.at(body);
                        if (activeSet && source.collisionSource != activeSource)
                            continue;
                        if (source.collisionGroupAabb.IsVoid() || source.collisionGroupObb.IsVoid())
                            continue;
                        const gp_Trsf trsf = bodyTransformForNode(source);
                        target->append({bodyIndices.at(body), body, trsf,
                            transformAabb(source.collisionGroupAabb, trsf, clearanceMm),
                            transformObb(source.collisionGroupObb, trsf, clearanceMm)});
                    }
                };
                appendGroups(activeBodies, activeIndices, true, &activeGroups);
                appendGroups(passiveBodies, passiveIndices, false, &passiveGroups);
                QSet<int> activeRelevant;
                QSet<int> passiveRelevant;
                for (const PoseGroup& active : activeGroups) for (const PoseGroup& passive : passiveGroups) {
                    if (!active.aabb.IsOut(passive.aabb) && !active.obb.IsOut(passive.obb)) {
                        activeRelevant.insert(active.sourceIndex);
                        passiveRelevant.insert(passive.sourceIndex);
                    }
                }
                const auto appendLeaves = [&bodyTransformForNode, clearanceMm](const QVector<SimulationBody>& bodies,
                                                                                  const QVector<int>& bodyIndices,
                                                                                  const QSet<int>& relevant,
                                                                                  QVector<PoseLeaf>* target) {
                    for (int body = 0; body < bodies.size(); ++body) {
                        if (!relevant.contains(body))
                            continue;
                        const SimulationBody& source = bodies.at(body);
                        const gp_Trsf trsf = bodyTransformForNode(source);
                        for (const CollisionLeaf& leaf : source.collisionLeaves) {
                            PoseLeaf world;
                            world.leaf = &leaf;
                            world.transform = trsf;
                            world.aabb = transformAabb(leaf.localAabb, trsf, clearanceMm);
                            world.obb = transformObb(leaf.localObb, trsf, clearanceMm);
                            world.bodyIndex = bodyIndices.at(body);
                            world.cutterProxy = source.cutterProxy;
                            world.workpiece = source.workpiece;
                            target->append(std::move(world));
                        }
                    }
                };
                appendLeaves(activeBodies, activeIndices, activeRelevant, &activeLeaves);
                appendLeaves(passiveBodies, passiveIndices, passiveRelevant, &passiveLeaves);
                for (int a = 0; a < activeLeaves.size(); ++a) for (int b = 0; b < passiveLeaves.size(); ++b) {
                    if (progress->isAbortRequested()) return;
                    const PoseLeaf& active = activeLeaves.at(a);
                    const PoseLeaf& passive = passiveLeaves.at(b);
                    qint8 pairState = 0;
                    if (!active.leaf || !passive.leaf || active.aabb.IsVoid() || passive.aabb.IsVoid()
                        || active.obb.IsVoid() || passive.obb.IsVoid()) {
                        pairState = 3;
                    } else if (active.aabb.IsOut(passive.aabb) || active.obb.IsOut(passive.obb)) {
                        continue;
                    }
                    if (pairState == 0) {
                        const TopoDS_Shape one = transformed(active.leaf->shape, active.transform);
                        const TopoDS_Shape two = transformed(passive.leaf->shape, passive.transform);
                        if (one.IsNull() || two.IsNull()) {
                            pairState = 3;
                        }
                        if (pairState == 3) {
                            const int severity = 3;
                            if (severity > worstSeverity) {
                                worstSeverity = severity;
                                worstState = pairState;
                                collisionPair = {active.bodyIndex, passive.bodyIndex};
                            }
                            continue;
                        }
                        lcnc::OcctExactOperationLock exactOperationLock;
                        BRepExtrema_DistShapeShape distance(one, two);
                        distance.SetDeflection(0.025);
                        distance.SetMultiThread(Standard_False);
                        distance.Perform();
                        if (!distance.IsDone()) {
                            pairState = 3;
                        } else if (distance.Value() <= Precision::Confusion()) {
                            pairState = 1;
                            // At a cutting node only the nozzle tip's intended
                            // zero contact may be ignored. Move the nozzle a
                            // tiny distance along the outward normal: a true
                            // penetration remains at zero distance, while a
                            // tangent tip contact separates. This avoids an
                            // expensive and crash-prone Boolean Common.
                            if (active.cutterProxy && passive.workpiece
                                && node.kind == NodeKind::Cutting) {
                                gp_Vec outward(node.normalX, node.normalY, node.normalZ);
                                if (outward.SquareMagnitude() <= Precision::SquareConfusion())
                                    outward = gp_Vec(0.0, 0.0, 1.0);
                                outward.Normalize();
                                outward.Multiply(qMax(0.05, Precision::Confusion() * 10.0));
                                gp_Trsf probeTransform;
                                probeTransform.SetTranslation(outward);
                                const TopoDS_Shape probe = transformed(one, probeTransform);
                                BRepExtrema_DistShapeShape probeDistance(probe, two);
                                probeDistance.SetDeflection(0.025);
                                probeDistance.SetMultiThread(Standard_False);
                                probeDistance.Perform();
                                if (!probeDistance.IsDone())
                                    pairState = 3;
                                else if (probeDistance.Value() > Precision::Confusion())
                                    pairState = 0;
                            }
                        } else if (distance.Value() <= clearanceMm) {
                            pairState = 2;
                        }
                    }
                    const int severity = pairState == 1 ? 4 : pairState == 3 ? 3 : pairState == 2 ? 2 : 0;
                    if (severity > worstSeverity) {
                        worstSeverity = severity;
                        worstState = pairState;
                        collisionPair = {active.bodyIndex, passive.bodyIndex};
                    }
                }
                {
                    QMutexLocker lock(resultMutex.get());
                    const qint8 current = result->at(n);
                    if (worstState != 0
                        && lcnc::cam_algo::collisionStateSeverity(worstState)
                            > lcnc::cam_algo::collisionStateSeverity(current)) {
                        result->operator[](n) = worstState;
                        pairs->operator[](n) = collisionPair;
                    } else if (finalPhase && current < 0) {
                        result->operator[](n) = 0;
                        pairs->operator[](n) = {-1, -1};
                    }
                }
                const int finished = completed->fetch_add(1) + 1;
                if ((finished & 0x0f) == 0 || finished == scanSteps) {
                    QMutexLocker progressLock(progressMutex.get());
                    progress->setValue(finished);
                }
                });
            }
            if (!progress->isAbortRequested()) progress->setValue(scanSteps);
        });
        QObject::connect(tasks, &TaskManager::taskProgressChanged, m_page,
                         [this, generation, result, pairs, resultMutex](TaskId id, int percent) {
            if (id != m_scanTask || generation != m_generation) return;
            {
                QMutexLocker lock(resultMutex.get());
                m_collisionStates = *result;
                m_collisionPairs = *pairs;
            }
            if (m_timeline) m_timeline->setStates(m_collisionStates);
            if (m_status && !m_playing)
                m_status->setText(QObject::tr("Scanning collisions: %1%").arg(percent));
        });
        QObject::connect(tasks, &TaskManager::taskFinishedDetailed, m_page, [this, tasks, generation, result, pairs, resultMutex](TaskId id, TaskExecutionStatus status, const QString&) {
            if (id != m_scanTask || generation != m_generation) return;
            m_scanTask = kInvalidTaskId;
            if (status != TaskExecutionStatus::Succeeded) {
                // A failed scan is not a safe result. Keep the session usable
                // but distinguish uncertainty from a confirmed intersection.
                m_collisionStates.fill(3, m_nodes.size());
                m_collisionPairs.fill({-1, -1}, m_nodes.size());
                m_scanFinished = true;
                if (m_timeline) m_timeline->setStates(m_collisionStates);
                if (m_status) m_status->setText(QObject::tr("Collision scan failed; all positions are marked indeterminate"));
                return;
            }
            {
                QMutexLocker lock(resultMutex.get());
                m_collisionStates = *result; m_collisionPairs = *pairs;
            }
            m_scanFinished = true;
            if (m_timeline) m_timeline->setStates(m_collisionStates);
            if (m_status) m_status->setText(QObject::tr("Collision scan complete"));
            if (m_slider) applyNode(m_slider->value());
        });
    }
    void showCollision(int index)
    {
        if (!m_gui || index < 0 || index >= m_collisionStates.size()) return;
        clearCollisionHighlight();
        const qint8 state = m_collisionStates.at(index);
        if (state <= 0) return;
        const auto pair = index < m_collisionPairs.size() ? m_collisionPairs.at(index) : QPair<int, int>{-1, -1};
        if (pair.first >= 0 && pair.second >= 0 && pair.first < m_bodies.size() && pair.second < m_bodies.size()) {
            auto& first = m_bodies[pair.first]; auto& second = m_bodies[pair.second];
            // Collision detection is the safety result. Highlight the two
            // confirmed source bodies without running a Boolean Common on the
            // GUI thread; the latter was costly and could crash inside OCCT.
            // 中文翻译：碰撞结果直接高亮双方，不在 GUI 线程求精确布尔交集。
            if (state != 1) {
                if (m_gui->context()) m_gui->context()->UpdateCurrentViewer();
                return;
            }
            const Quantity_Color collisionColor(Quantity_NOC_RED);
            if (!first.ais.IsNull()) first.ais->SetColor(collisionColor);
            if (!second.ais.IsNull()) second.ais->SetColor(collisionColor);
            m_highlightedCollisionPair = pair;
        }
        if (m_gui->context()) m_gui->context()->UpdateCurrentViewer();
    }

    void clearCollisionHighlight()
    {
        if (m_highlightedCollisionPair.first < 0 || !m_gui)
            return;
        const auto clear = [this](int bodyIndex) {
            if (bodyIndex >= 0 && bodyIndex < m_bodies.size() && !m_bodies[bodyIndex].ais.IsNull())
                m_bodies[bodyIndex].ais->UnsetColor();
        };
        clear(m_highlightedCollisionPair.first);
        clear(m_highlightedCollisionPair.second);
        // Restore the configured per-axis/workpiece colours after a transient
        // red collision marker rather than leaving AIS objects at OCC's
        // generic default colour.
        // 中文翻译：碰撞红色标记撤销后恢复应用程序配置的轴/工件颜色。
        m_gui->applyMachineDisplayStyle();
        for (const SimulationBody& body : std::as_const(m_bodies)) {
            if (body.cutterProxy)
                applyCutterProxyAppearance(body);
        }
        m_highlightedCollisionPair = {-1, -1};
    }

    SimulationModule* m_owner{nullptr};
    QWidget* m_page{nullptr}; GuiDocument* m_gui{nullptr}; WidgetOccView* m_view{nullptr};
    QSlider* m_slider{nullptr}; CollisionTimeline* m_timeline{nullptr}; QLabel* m_status{nullptr}; QTimer* m_timer{nullptr};
    MachineKinematics m_kinematics; lcnc::MachineAxisLayout m_layout;
    QList<MachineAxisDef> m_planningBaselineAxes;
    SimulationSceneSnapshot m_sceneSnapshot;
    std::unique_ptr<lcnc::view::ToolpathRenderer> m_toolpathRenderer;
    std::unique_ptr<lcnc::view::TravelPathRenderer> m_travelPathRenderer;
    std::unique_ptr<lcnc::view::MachineGuideRenderer> m_guideRenderer;
    QVector<SimulationNode> m_nodes; QVector<SimulationBody> m_bodies; QVector<qint8> m_collisionStates;
    QVector<QPair<int, int>> m_collisionPairs;
    QPair<int, int> m_highlightedCollisionPair{-1, -1};
    TaskId m_scanTask{kInvalidTaskId}; TaskId m_collisionPreparationTask{kInvalidTaskId};
    quint64 m_generation{1}; std::uint64_t m_sourceRevision{0};
    int m_machineBodyCount{0}; int m_workpieceBodyCount{0}; int m_mountedWorkpieceBodyCount{0};
    double m_speed{1.0}; bool m_playing{false}; bool m_scanFinished{false};
    bool m_advancingPlayback{false}; bool m_scanStarted{false}; bool m_applyingAxisGroupSelection{false};
    bool m_collisionPreparationStarted{false}; bool m_collisionPreparationFinished{false};
    bool m_sourceInvalidated{false};
    bool m_collisionDetectionEnabled{false};
    bool m_collisionConfigurationValid{false};
    std::uint64_t m_collisionConfigurationRevision{0};
};

SimulationModule::SimulationModule(QObject* parent) : QObject(parent) {}
SimulationModule::~SimulationModule() { stop(); }
lcnc::ModuleInfo SimulationModule::info() const { return {QStringLiteral("simulation"), QStringLiteral("Offline machine simulation"), QStringLiteral("1.0.0"), {QStringLiteral("cam")}}; }
bool SimulationModule::init(lcnc::IKernel& kernel) { auto self = std::shared_ptr<SimulationModule>(this, [](SimulationModule*) {}); kernel.services().registerService<SimulationModule>(self); return true; }
bool SimulationModule::start() { return true; }
void SimulationModule::stop() { exitSimulation(); }
QWidget* SimulationModule::enterSimulation() {
    if (m_session && m_session->isOpen()) return m_session->page();
    m_session = std::make_unique<SimulationSession>(this);
    QString error;
    QWidget* page = m_session->open(&error);
    if (!page) {
        LCNC_WARN(lcnc::LogCode::Generic, "Offline simulation was not opened: {}", error.toStdString());
        m_session.reset();
        emit operationFailed(tr("Offline machine simulation"), error);
        return nullptr;
    }
    emit sessionOpened(page);
    return page;
}
void SimulationModule::exitSimulation() { if (!m_session) return; m_session->close(); m_session.reset(); emit sessionClosed(); }
bool SimulationModule::hasActiveSession() const { return m_session && m_session->isOpen(); }
void SimulationModule::run() { if (m_session) m_session->run(); }
void SimulationModule::pause() { if (m_session) m_session->pause(); }
void SimulationModule::stopPlayback() { if (m_session) m_session->stopPlayback(); }
void SimulationModule::setSpeedMultiplier(double multiplier) { if (m_session) m_session->setSpeed(multiplier); }
void SimulationModule::jumpToCollision(int direction) { if (m_session) m_session->jumpCollision(direction < 0 ? -1 : 1); }

} // namespace lcnc::simulation
