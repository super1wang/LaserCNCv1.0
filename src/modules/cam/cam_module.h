
#pragma once

#include <QObject>
#include <QByteArray>
#include <QColor>
#include <QList>
#include <QMap>
#include <QSet>

#include <atomic>
#include <QString>
#include <QVector>

#include <memory>

#include "core/task/module_task_scope.h"

#include "core/project/cam/cam_data_contracts.h"
#include "core/project/cam/cam_data_manager.h"
#include "modules/cam/settings/cam_config.h"
#include "modules/cam/contracts/i_cam_facade.h"
#include "modules/cam/contracts/i_cam_contour_sequence_provider.h"
#include "modules/cam/contracts/i_cam_collision_configuration_provider.h"
#include "modules/cam/contracts/i_cam_collision_safety_domain.h"
#include "modules/cam/contracts/i_cam_initial_approach_planner.h"
#include "modules/cam/contracts/i_cam_project_explorer_projection.h"
#include "modules/cam/pipeline/machining_face_pipeline_service.h"
#include "modules/cam/safety/machine_safety_package_manager.h"
#include "modules/cam/safety/job_safety_overlay_manager.h"
#include "modules/cam/contracts/i_cam_toolpath_provider.h"
#include "core/algorithms/cam/laser_toolpath.h"
#include "core/kernel/i_module.h"
#include "core/kernel/i_service.h"
#include "core/kernel/event_bus.h"
#include "core/project/project_types.h"
#include "core/task/task_manager.h"

#include <AIS_Shape.hxx>
#include <TopoDS_Shape.hxx>

class LcncDocument;
class GuiDocument;
class MachineKinematics;
class GraphicsScene;
class QTimer;
namespace lcnc { class MachinePose; }
namespace lcnc { class MachineConfigurationService; }
class QPoint;
class WidgetOccView;
class gp_Vec;
class gp_Ax1;
class gp_Pnt;

namespace lcnc::view {
class ToolpathRenderer;
class MachineGuideRenderer;
class TravelPathRenderer;
class ContourOrderLabelRenderer;
} // namespace lcnc::view

namespace lcnc::cam {
class CamDisplayProjectionService;
struct TravelCollisionGeometryCache;
struct MotionCompilationInput;
struct ToolpathGenerationStamp;
}

namespace lcnc::cam {
class CamDataManager;
class MachineWorkspace;
} // namespace lcnc::cam

/**
 * @brief CAM module singleton — manages machine, toolpath, and CAM data.
 *
 * Responsible for:
 *  - Machine model loading / unloading / export / axis configuration
 *  - Workpiece mounting onto machine axes inside the machine document
 *  - Toolpath generation, lead-in computation, preview display
 *  - Managing the "准备" (Prepare) tab page
 *  - Coordinates the machine domain view and CAM runtime display
 *
 * For shape operations (move/rotate/delete) on machine entities, delegates
 * to ShapeService — sharing the same geometry code as the CAD module.
 *
 * 微内核集成：同 CadModule，实现 @ref lcnc::IModule + @ref lcnc::IService，
 * 生命周期由 Kernel 接管，依赖 "cad" 模块（三域文档由 LcncProjectManager 管理）。
 */
class CamModule : public QObject,
                  public lcnc::IModule,
                  public lcnc::ICamFacade
{
    Q_OBJECT
public:
    /// @brief OCC-free info about one machining face, for the project tree.
    struct MachiningFaceInfo {
        std::uint64_t faceId{0};
        QString displayName;
        QString workpieceEntry;
        bool manual{false};
        lcnc::cam::MachiningFaceRole role{lcnc::cam::MachiningFaceRole::MachiningSurface};
    };
    /// 公开构造：由 Kernel 拥有。
    explicit CamModule(QObject* parent = nullptr);
    /// 显式析构（用于 unique_ptr<前置声明类型>）。
    ~CamModule() override;

    struct AxisOption {
        QString name;
        QString displayName;
    };

    struct WorkpieceMountCandidate {
        DocumentId documentId{kInvalidDocumentId};
        QString displayName;
        int workpieceCount{0};
    };

    /**
     * @brief 三段式机台坐标系标定输入。
     *
     * 由 @ref DialogAxisCalibrationWizard 按 TableTilt、TableSpin 语义依次
     * 拾取父旋转轴参考面、子旋转轴参考面和切割头下端面后整合而成。
     */
    struct AxisCalibrationInputs {
        gp_Pnt tiltFaceCenter{0.0, 0.0, 0.0};    ///< TableTilt 父旋转轴参考面中心
        gp_Pnt spinFaceCenter{0.0, 0.0, 0.0};    ///< TableSpin 子旋转轴参考面中心
        gp_Pnt cutterHeadFaceCenter{0.0, 0.0, 0.0}; ///< 切割头下端面中心（模型坐标）
    };


    // ── IModule ─────────────────────────────────────────────────────
    /// id="cam"，依赖 ["cad"]。
    lcnc::ModuleInfo info() const override;
    bool init(lcnc::IKernel& kernel) override;
    bool start() override;
    void stop() override;

    /// CAM 持久化设置（旧 CamConfig 单例改由本模块拥有）。
    CamConfig&       config() override       { return m_config; }
    const CamConfig& config() const override { return m_config; }

    /// ICamFacade：用于让调用方挂接 Qt 信号。
    QObject* asQObject() override { return this; }
    lcnc::cam::ProjectExplorerSnapshot projectExplorerSnapshot() const;

    // ── Domain Workspaces ────────────────────────────────────────────────
    LcncDocument*      machineDocument() const;
    DocumentId         machineDocumentId() const;
    LcncDocument*      camDocument() const;
    DocumentId         camDocumentId() const;
    GuiDocument*       activeGuiDocument() const;
    MachineKinematics* kinematics() const;
    void               requestMachineView() override;
    QString            machineModelPath() const;
    void               setMachineModelPath(const QString& filePath);
    lcnc::RenderQualityPreset machineRenderQualityPreset() const;
    void               setMachineRenderQualityPreset(lcnc::RenderQualityPreset quality);

    // ── Machine Management ───────────────────────────────────────────────
    /// Configure machine kinematics without requiring a machine model.
    void configureMachine(const QString& presetName);

    /// Load machine model geometry using the current machine configuration.
    void loadMachine(const QString& filePath);

    /// Build or update a .lmsp package in the offline tool. On success the
    /// configured machine path is switched to the package and reloaded.
    bool buildOrUpdateMachineSafetyPackage(QString* errorMessage = nullptr);

    bool hasValidMachineSafetyPackage() const;
    QString loadedMachineSafetyPackagePath() const;
    lcnc::cam::MachineSafetyPackageStatus machineSafetyPackageStatus() const;

    /// True between accepting a load request and accepting or rejecting its
    /// latest detached parse result.  Process uses this to reject stale rapid
    /// plans while the physical environment is indeterminate.
    bool isMachineLoadPending() const { return m_machineLoadPending.load(); }

    /// Remove all machine entities but keep the current machine configuration.
    void unloadMachine();

    /// Export machine model as STEP with LCNC_AXIS_* naming.
    void exportMachine(const QString& filePath);

    /// Generate a conservative envelope for the currently opened and marked
    /// machine, then atomically export an AP242 tessellated STEP.
    bool exportSimplifiedMachine(const QString& filePath,
                                 QString* errorMessage = nullptr);
    bool isModelEnvelopeExportPending() const
    {
        return m_modelEnvelopeExportTask != kInvalidTaskId;
    }

    /// Auto-detect axis assignments by shape name heuristics.
    void autoDetectAxes();
    void applyAxisAssignments(const QMap<QString, QString>& entryToAxis);
    void assignShapesToAxis(const QStringList& entries, const QString& axisName);
    void unassignShape(const QString& entry);
    lcnc::cam::CollisionConfigurationSnapshot collisionConfiguration() const;
    lcnc::cam::CollisionSafetyDomainSnapshot collisionSafetyDomain() const;
    lcnc::cam::CollisionValidationSnapshot validateCollisionPath(
        const lcnc::cam::CollisionSafetyPathRequest& request,
        std::atomic_bool* cancelRequested = nullptr) const;
    lcnc::cam::CamMotionPermit requestMotionPermit(
        const lcnc::cam::CamMotionPermitRequest& request) const;
    lcnc::cam::InitialApproachSnapshot planInitialApproach(
        const lcnc::cam::InitialApproachRequest& request,
        std::atomic_bool* cancelRequested = nullptr) const;
    bool setCollisionDetectionEnabled(bool enabled,
                                      QString* errorMessage = nullptr);
    void setCollisionSources(const QSet<QString>& active,
                             const QSet<QString>& passive);
    QList<AxisOption> axisOptions(bool includeDetachOption = false) const;
    /// Return the preferred workpiece mount axis for the current machine preset.
    QString defaultWorkpieceMountAxis() const;
    gp_Pnt axisOrigin(const QString& axisName) const;
    void setAxisOrigin(const QString& axisName, const gp_Pnt& origin);
    bool setAxisLimits(const QString& axisName, double minVal, double maxVal);
    /// Runtime/OCC world point used by geometry, collision and calibration.
    bool currentAcRotationCenter(gp_Pnt& center) const;
    /// User-facing controller-axis coordinate of the same physical center.
    bool currentAcRotationCenterAxisCoordinates(gp_Pnt& center) const;
    gp_Pnt cutterHeadModelPosition() const;
    gp_Pnt cutterHeadPhysicalPosition() const;
    /// 拾取一个平面参考面，返回其几何中心（已叠加 LocalTransformation）。
    /// 仅做查询、不修改任何模块状态，供标定向导使用。
    bool pickReferenceFaceCenter(WidgetOccView* occView,
                                 const QPoint& screenPos,
                                 gp_Pnt& center,
                                 QString* errorMessage = nullptr) const;
    /// 三段式模型对齐：从 TableTilt/TableSpin 轴方向与参考面推导旋转中心，
    /// 先刚体平移整机，再沿实际刀头承载链牵引对应直线轴模型。
    /// 此流程不写入/修改物理旋转中心或实时轴坐标。
    bool applyAxisCalibration(const AxisCalibrationInputs& inputs, QString* errorMessage = nullptr);
    // 中文翻译：绝对标定目标
    /// Validate the picked calibration references and expose the immutable
    /// absolute AC-center/TCP targets. This preview never changes live axes.
    bool enterStandardCalibrationPose(const AxisCalibrationInputs& inputs,
                                      QString* errorMessage = nullptr);
    /// 切割头当前世界坐标（受当前 X/Y/Z 轴位置影响）。
    gp_Pnt cutterHeadWorldPosition() const;
    /// 切割头当前轴系坐标（与控制器反馈方向一致）。
    gp_Pnt cutterHeadAxisPosition() const;
    QList<WorkpieceMountCandidate> mountableWorkpieces() const;
    bool autoInstallWorkpiece() const;
    void setAutoInstallWorkpiece(bool enabled);
    bool autoInstallCurrentWorkpiece();
    QStringList sourceWorkpieceEntriesForMountedEntries(const QStringList& mountedEntries) const;
    /// Sets the unified CAD-to-fixture setup origin to the configured workpiece
    /// rotation center.  It never moves or rewrites CAD geometry.
    bool alignWorkpieceSetupToRotationCenter();

    // ── Workpiece Installation ───────────────────────────────────────────
    /// Mount the current Workpiece source document to an axis.  Placement is
    /// exclusively defined by WorkpieceSetupTransform and geometry is never moved.
    void mountWorkpiece(DocumentId sourceDocId, const QString& axisName, bool alignToInstallPosition = true);

    // ── Shape Operations on Machine Doc (delegates to ShapeService) ──────
    bool moveShape(const QString& entry, const gp_Vec& translation);
    bool rotateShape(const QString& entry, const gp_Ax1& axis, double angleDeg);
    void deleteShape(const QString& entry);

    /// Controls how the automatic pipeline treats an existing machining-face
    /// set when a global generation is triggered.
    ///  - \c AutoSeparate re-runs face separation and appends auto-detected
    ///    faces on top of any manual picks (the default global behavior).
    ///  - \c ReuseCurrent skips face separation and runs the remaining stages
    ///    on the current face set as-is, so manual picks are not stacked with
    ///    auto-detected faces.
    enum class AutoPipelineFaceMode { AutoSeparate, ReuseCurrent };

    // ── Toolpath ─────────────────────────────────────────────────────────
    TaskId generateToolpathAsync(double smoothAngle, bool useFaceClassification,
                                 double deflection = 0.1,
                                 AutoPipelineFaceMode mode = AutoPipelineFaceMode::AutoSeparate);
    void clearToolpath();

    // ── Explicit CAM pipeline ───────────────────────────────────────────
    /// Asynchronously identify automatic machining faces from the workpiece.
    /// Manual mode remains an explicit Apply action because it has no geometry
    /// recognition work to schedule.
    TaskId separateMachiningFacesAsync();
    /// Commit the current manually edited face set and invalidate all
    /// downstream stages.  It never regenerates faces behind the user's back.
    bool applyMachiningFaces();
    TaskId extractContoursFromMachiningFacesAsync();
    TaskId discretizeCurrentContoursAsync();
    TaskId buildCurrentGeometricToolpathAsync();
    TaskId solveCurrentGeometricToolpathAsync();
    /// Starts the automatic pipeline after face separation.  Remaining stages
    /// are scheduled one-by-one as cancellable TaskManager jobs.  \p mode
    /// selects how the existing machining-face set is treated (see
    /// \c AutoPipelineFaceMode).
    TaskId runAutoPipelineAsync(AutoPipelineFaceMode mode = AutoPipelineFaceMode::AutoSeparate);
    bool runAutoPipeline(AutoPipelineFaceMode mode = AutoPipelineFaceMode::AutoSeparate);
    lcnc::cam::CamPipelineStageState pipelineStageState(lcnc::cam::CamPipelineStage stage) const;

    // 刀路持久化（cam_toolpath.toml + points.bin）已下沉到 core
    // （lcnc::cam::saveCamToolpath / loadCamToolpath，由 LcncProjectManager 统一调度）。
    const LaserToolpath& toolpath() const;
    LaserToolpath& toolpathRef();
    const LaserToolpath& toolpathRef() const;
    bool hasToolpath() const override;
    int toolpathContourCount() const override;
    int toolpathContourPointCount(int contourIndex) const override;
    std::uint64_t toolpathRevision() const;
    /// CAM is the sole authority for the enabled, ordered contour sequence.
    /// Consumers (Process and offline simulation) must use this snapshot and
    /// must not rebuild an independent order.
    lcnc::cam::ContourSequenceSnapshot contourSequenceSnapshot() const;
    /// Applies CAM's software-configured automatic order and resolves the complete
    /// machine-coordinate sequence at the single CAM solve boundary.
    bool applyAutoContourSort(lcnc::cam::AutoSortAxis axis, QString* errorMessage = nullptr);
    /// 无视自动碰撞检测开关，对当前已求解刀路执行一次完整碰撞校验。
    bool validateCurrentToolpathCollisions(QString* errorMessage = nullptr);
    /// Sets the persisted CAM manual order, resolves it, then rebuilds the
    /// matching rapid plan used by both the view and Process.
    bool setManualContourOrder(const QVector<lcnc::cam::ContourId>& orderedContourIds,
                               QString* errorMessage = nullptr);
    QVector<lcnc::cam::ContourId> manualContourOrder() const;
    lcnc::cam::AutoSortAxis lastAutoContourSortAxis() const;
    void setLastAutoContourSortAxis(lcnc::cam::AutoSortAxis axis);
    bool solveToolpathForOrder(const QVector<std::uint64_t>& orderedContourIds,
        std::shared_ptr<const lcnc::cam::MotionCompilationInput> input = {});
    lcnc::cam::ToolpathExportSnapshot exportToolpathBaseSnapshot() const;
    lcnc::cam::ToolpathExportSnapshot exportToolpathSnapshotForOrder(
        const QVector<std::uint64_t>& orderedContourIds) const;

    QList<lcnc::MachiningMode> supportedMachiningModes() const;
    lcnc::MachiningMode machiningMode() const;
    bool setMachiningMode(lcnc::MachiningMode mode);
    lcnc::WorkpieceSetupTransform workpieceSetupTransform() const;
    bool setWorkpieceSetupTransform(const lcnc::WorkpieceSetupTransform& setup);

    void setLeadInLength(double mm);
    double leadInLength() const;
    void setDeflection(double mm);
    double deflection() const;
    bool setCuttingOffset(double mm);
    double cuttingOffset() const;
    bool setRapidOffset(double mm);
    double rapidOffset() const;
    void setActiveContourId(lcnc::cam::ContourId contourId);
    lcnc::cam::ContourId activeContourId() const { return m_activeContourId; }
    int activeContourIndex() const;
    bool setActiveContourLeadInLength(double mm);
    bool setActiveContourDeflection(double mm);
    bool setActiveContourCuttingOffset(double mm);
    bool setActiveContourRapidOffset(double mm);
    void setContourEnabled(int contourIdx, bool enabled);
    void setAllContoursEnabled(bool enabled);
    lcnc::cam::ContourId contourIdAt(int contourIdx) const;
    int contourIndexById(lcnc::cam::ContourId contourId) const;
    /// 当前在 3D 视图中选中的轮廓 id 列表（用于"移动到图层"等跨树操作）。
    QList<lcnc::cam::ContourId> selectedContourIds() const;
    void reorderContoursById(const QList<lcnc::cam::ContourId>& order);
    void reorderContours(const QList<int>& order);
    const std::vector<ToolpathLayer>& toolpathLayers() const;
    QList<int> contourIndexesInLayer(std::uint64_t layerId) const;
    /// 工程文档级图层管理：新建图层，返回 layerId（重命名/改色走 updateToolpathLayer）。
    std::uint64_t addToolpathLayer(const QString& name, const QColor& color = QColor());
    /// 删除图层及其下所有轮廓（连同 XCAF/AIS/选择状态一并清理）。
    bool removeToolpathLayerWithContours(std::uint64_t layerId);
    /// 把一组轮廓移动到指定图层下。
    bool assignContoursToLayer(const QList<lcnc::cam::ContourId>& contourIds,
                               std::uint64_t layerId);
    /// 仅更新图层的外观/名称，保留现有工具映射。
    bool updateToolpathLayer(std::uint64_t layerId,
                             const QString& name,
                             const QColor& color);
    /// 更新图层的外观/名称/工具映射。
    bool updateToolpathLayer(std::uint64_t layerId,
                             const QString& name,
                             const QColor& color,
                             const QString& toolName);
    bool setToolpathLayerEnabled(std::uint64_t layerId, bool enabled);

    /// Asynchronous variant used by UI commands. OCC calculation runs against
    /// a copied contour/kinematics snapshot; document and view updates remain
    /// on the GUI thread.
    TaskId recalcToolpathAsync();

    /// Toggle toolpath display visibility.
    void setToolpathVisible(bool visible);
    bool isToolpathVisible() const;

    /// CAM 运行时数据管理器（图层容器、轮廓 id 表、signature 表）。
    /// 由 CamLayerProviderAdapter / 部分命令路径使用；保持非空。
    lcnc::cam::CamDataManager*       camData();
    const lcnc::cam::CamDataManager* camData() const;
    lcnc::cam::MachineWorkspace*       machineWorkspace()       { return m_machineWorkspace; }
    const lcnc::cam::MachineWorkspace* machineWorkspace() const { return m_machineWorkspace; }

    // ── Toolpath Parameters ──────────────────────────────────────────────
    double smoothAngle() const;
    void   setSmoothAngle(double deg);
    bool   useFaceClassification() const;
    void   setUseFaceClassification(bool on);
    /// ExtractionStrategy value (largest-smooth/planar/manual). Drives contour
    /// extraction; machine config still drives discretization independently.
    int    extractionStrategy() const;
    void   setExtractionStrategy(int strategy);
    /// Laser beam travel direction in \a wpcEntry workpiece coordinates at the
    /// home posture, derived from the machine beam axis and the wpc mount.
    /// Used only where a machining-ray context is required; planar extraction
    /// is fixed to the workpiece XY/Z coordinate system.
    gp_Dir beamDirectionWpc(const QString& wpcEntry) const;
    /// @name Manual machining-face selection (ManualFaceSelection strategy)
    /// @{
    int    machiningFaceCount() const;
    /// True when the current face set contains any manually picked face.
    bool   hasManualMachiningFaces() const;
    QList<MachiningFaceInfo> machiningFacesForTree() const;
    void   addMachiningFace(const TopoDS_Face& face);
    bool   removeMachiningFace(std::uint64_t faceId);
    bool   setMachiningFaceRole(std::uint64_t faceId, lcnc::cam::MachiningFaceRole role);
    void   clearMachiningFaces();
    bool   pickMachiningFace(WidgetOccView* view, const QPoint& pos, QString* error);
    /// Toggle only the displayed machining-surface highlights. Cross-section
    /// faces remain internal reference data and are never rendered here.
    void   setMachiningFacesVisible(bool visible);
    bool   machiningFacesVisible() const;
    /// Manual picks only (fed into ManualFaceSelection extraction).
    std::vector<TopoDS_Face> manualMachiningFaces() const;
    /// Refresh the semi-transparent highlight AIS for the current face set.
    void   refreshMachiningFaceDisplay();
    /// Push the current machining-face state into CamDataManager for persistence.
    void   pushMachiningFaceRecordsToCamData();
    /// After project load, rebind stored face signatures to workpiece geometry.
    void   rebindMachiningFacesFromRecords();
    /// @}
    bool   showNormals() const;
    void   setShowNormals(bool on);
    double normalSampleStep() const;
    void   setNormalSampleStep(double mm);
    bool   updateLeadInPreview(WidgetOccView* occView, const QPoint& screenPos);
    bool   commitLeadInPreview(WidgetOccView* occView, const QPoint& screenPos);
    void   cancelLeadInPreview();

    // ── Toolpath Display ─────────────────────────────────────────────────
    /// Redisplay all toolpath AIS objects (contours + lead-ins).
    void refreshToolpathDisplay();

    /// Erase all toolpath AIS objects from the machine scene.
    void eraseToolpathDisplay();

    const QList<Handle(AIS_Shape)>& contourAis() const;

    // ── Axis Position ────────────────────────────────────────────────────
    void setAxisPosition(const QString& axisName, double value, bool refreshNow = true);
    void refreshMachineTransforms();

    /// 仅刷新指定 dirty 轴对应的几何变换链（局部刷新，避免整体重绘）。
    /// dirty 为空等价于 @ref refreshMachineTransforms 全量刷新。
    void refreshMachineTransforms(const QStringList& dirtyAxes);

    // ── Selection / Visibility ──────────────────────────────────────────
    void setEntityVisible(const QString& entry, bool visible);
    QStringList visibleMachineEntries() const;
    void setMachineModelVisible(bool visible);
    bool isMachineModelVisible() const;
    void setRotaryAxisGuidesVisible(bool visible);
    bool rotaryAxisGuidesVisible() const;
    void setCutterHeadGuideVisible(bool visible);
    bool cutterHeadGuideVisible() const;
    /// 按 AppSettings 中的刀头外观（颜色/透明度/缩放）重建刀头锥指示器。
    void refreshCutterHeadAppearance();
    /// 应用刀嘴/模拟锥显示配置；不参与碰撞，不使执行缓存失效。
    bool refreshCutterCollisionConfiguration();
    /// Applies clearance/path-safety settings and rebuilds only the workpiece
    /// overlay whose geometry policy actually changed.
    void refreshCollisionSafetyPolicy();
    /// Presentation-only nozzle/cone. Collision uses the complete Z-axis
    /// geometry from the immutable machine package.
    TopoDS_Shape cutterDisplayProxyShape(QString* errorMessage = nullptr);
    void setSelectedEntries(const QStringList& entries);
    QStringList selectedEntries() const;
    void syncSelectionFromView();

    // ── Travel path 虚线显示 ────────────────────────────────────────────
    // 中文翻译：切割路径显示
    /// 切换"Cutting path display"。OFF 时立即擦除；ON 时立刻按当前 plan 重绘。
    void setTravelPathVisible(bool on);
    bool isTravelPathVisible() const;
    /// 按 CAM 权威轮廓顺序刷新虚线（仅 visible=true 时）。
    void refreshTravelPath();

    // ── 切割链表序号标注显示 ────────────────────────────────────────────
    // 中文翻译：切割链表序号显示
    /// 切换"Cutting sequence number display"。OFF 时立即擦除；ON 时立刻按当前 plan 重绘。
    void setContourOrderLabelVisible(bool on);
    bool isContourOrderLabelVisible() const;
    /// 按 CAM 权威轮廓顺序刷新序号标注（仅 visible=true 时）。
    void refreshContourOrderLabels();
    /// 同时刷新空程虚线与序号标注（两者数据同源，几何/顺序变化时一并刷新）。
    void refreshCuttingOrderOverlays();

signals:
    void machineViewRequested();
    void machineWorkspaceChanged();
    void operationFailed(const QString& title, const QString& message);
    /// Emitted when generation completed but some contours could not resolve a
    /// lead-in. The contour is kept without a lead-in instead of aborting the
    /// whole toolpath; the user is informed so they can fix it manually.
    void operationWarning(const QString& title, const QString& message);
    void machineLoaded();
    void machineUnloaded();
    void machineSafetyPackageChanged();
    void modelEnvelopeExported(const QString& filePath);
    void modelEnvelopeExportStateChanged();
    void workpieceMounted(const QString& entry);
    void workpieceUnmounted();
    void toolpathGenerated();
    void toolpathCleared();
    void autoSortAxisChanged(lcnc::cam::AutoSortAxis axis);
    /// Emitted when the machining-face set changes (auto-capture / manual add /
    /// remove / clear) so the project tree and view highlight can refresh.
    void machiningFacesChanged();
    void pipelineStageChanged(lcnc::cam::CamPipelineStage stage);
    void toolpathVisibilityChanged(bool visible);
    void toolpathContourSelected(int contourIndex);
    void toolpathContoursSelected(const QList<int>& contourIndexes);
    void toolpathLayersChanged();
    void activeToolpathContourChanged(std::uint64_t contourId, int contourIndex);
    void activeContourParametersChanged();
    void selectionChanged(const QStringList& entries);
    void axisAssignmentsChanged();
    void machineVisibilityChanged();
    void machiningModeChanged(lcnc::MachiningMode mode);
    void workpieceSetupTransformChanged();
    void cutterCollisionConfigurationChanged();
    void collisionConfigurationChanged();
    /// A CAM-owned contour order now has a matching committed rapid plan.
    void contourOrderTravelPlanRebuilt(const QVector<std::uint64_t>& orderedContourIds);

private:
    friend struct CamMotionCompilationTestAccess;
    struct WorkpieceShapeSource {
        QString workpieceEntry;
        TopoDS_Shape shape;
        int componentIndex{0};
    };

    /// Collect the workpiece compound shape from the project document.
    TopoDS_Shape collectWorkpieceShape() const;
    QList<WorkpieceShapeSource> collectWorkpieceShapes() const;
    static std::vector<lcnc::cam::MachiningFacePipelineService::Candidate>
    selectAutomaticMachiningFaces(const QList<WorkpieceShapeSource>& sources,
                                  ExtractionStrategy strategy,
                                  double smoothAngle);
    bool rejectConflictingPipelineOperation(const QString& operation);
    std::uint64_t machiningFaceSetRevision() const;
    std::uint64_t machineSetupRevision() const;
    /// Revision of the fixed machine plus the current workpiece overlay.
    /// Display settings, collision enablement and toolpath order are excluded.
    std::uint64_t collisionEnvironmentRevision() const;
    QByteArray machineSafetyConfigurationFingerprint() const;
    /// Identity of the geometry currently committed to the machine workspace.
    /// It differs from the configured next-startup path while an operator edits
    /// options, preventing roles/profiles for one model being applied to another.
    QString activeMachineProfilePath() const;
    /// Invalidate collision/travel caches after a machine shape, assignment,
    /// or role mutation.  Geometry is intentionally versioned separately from
    /// the kinematic configuration because both participate in safety checks.
    void invalidateMachineEnvironment();
    void invalidateMachineSafetyPackage();

    /// Refresh axis guide AIS via MachineGuideRenderer.
    void displayAxisGuides();
    void updateAxisGuideTransforms();
    bool resolveLeadInHit(WidgetOccView* occView,
                          const QPoint& screenPos,
                          int& contourIdx,
                          int& pointIdx,
                          gp_Pnt& entryPoint,
                          double& entryParam) const;
    bool resolveReferencePlaneCenter(WidgetOccView* occView,
                                     const QPoint& screenPos,
                                     gp_Pnt& center,
                                     QString* errorMessage) const;
    bool ensureAcCenterCalibrationAvailable(QString* errorMessage = nullptr) const;
    bool currentWorkpieceRotationCenter(gp_Pnt& center) const;
    void autoDetectAxisOrigins();
    void applyStoredMachineProfile(const QString& machinePath);
    bool applyConfiguredMachineAxes(bool updateView);
    lcnc::cam::ToolpathExportSnapshot buildToolpathExportSnapshot(
        const std::vector<LaserContour>& contours,
        std::uint64_t revision,
        const QString& description) const;
    void attachMotionPlan(lcnc::cam::ToolpathExportSnapshot& snapshot) const;
    std::shared_ptr<const lcnc::cam::MotionCompilationInput> captureMotionCompilationInput(
        const lcnc::cam::ToolpathGenerationStamp* requested = nullptr) const;
    void retainMotionCompilationInput(
        const std::shared_ptr<const lcnc::cam::MotionCompilationInput>& input);
    void applyToolMotionOffsets(lcnc::cam::ToolpathExportSnapshot& snapshot) const;
    void attachTravelPlan(lcnc::cam::ToolpathExportSnapshot& snapshot) const;
    void scheduleFullEnvironmentVerification(const lcnc::cam::ToolpathExportSnapshot& snapshot,
                                             bool force = false);
    void scheduleCollisionSafetyDomainPreparation(
        const lcnc::cam::ToolpathExportSnapshot& snapshot);
    /// Starts the Job Overlay as soon as a workpiece and a valid immutable
    /// machine package coexist; no toolpath is required.
    void scheduleWorkpieceSafetyOverlayPreparation();
    bool rebuildTravelPlanForCurrentOrder(QString* errorMessage = nullptr);
    QVector<lcnc::cam::ContourId> planAutoContourOrder(
        lcnc::cam::AutoSortAxis axis, QString* errorMessage = nullptr) const;
    bool prepareConfiguredAutoSort(QString* errorMessage = nullptr,
        std::shared_ptr<const lcnc::cam::MotionCompilationInput> input = {});
    void clearToolpathSelectionState();
    void updateToolpathMachineCoordinates();
    bool autoInstallCurrentWorkpieceInternal(bool alignToInstallPosition);
    bool clearMountedWorkpieceDisplay(bool refreshView);
    LcncDocument* workpieceDocument() const;
    DocumentId workpieceDocumentId() const;
    void setCamContoursVisible(bool visible, bool updateView = true);
    void setCamContourVisible(int contourIndex, bool visible, bool updateView = true);
    void applyCamContourVisibility();
    void applyCamContourTransforms();
    void applyToolpathLayerColors(bool updateView = true);
    QList<int> selectedCamContourIndexes() const;

    /// Rebuilds the machine presentation. Pure visibility restoration must not
    /// publish a machine-domain data change because that can trigger unrelated
    /// workspace resynchronization.
    void refreshMachineDisplay(bool notifyDomainChange = true);
    void syncCamDocumentContours(bool forceRebuild = false);
    /// 把当前刀路的轮廓 wire 作为 EntityKind::Cam 实体写入统一工程文档，记录 xcafEntry。
    void writeContourGeometryToDocument();
    /// 读档后按 xcafEntry 从工程文档的 Cam 实体重连每条轮廓的 wire 几何。
    void relinkContourGeometryFromDocument();
    /// 把当前运行时刀路生成参数固化到工程核心数据（随工程持久化）。
    void pushGenerationParamsToCamData();
    /// 读档后把工程级生成参数应用回运行时。
    void applyGenerationParamsFromCamData();
    /// core 完成工程 CAM 数据加载后，刷新 OCC 文档镜像 + 渲染 + 相关信号。
    void onCamDataLoaded();
    /// 重置跟随项目 view 的显示状态；机台数据本身仍由 MachineWorkspace 独立持有。
    void resetProjectViewState();
    /// 清理 CAM 视图侧状态；用于模块内清刀路和项目核心外部清 CAM 域两条路径。
    void clearToolpathViewState(bool emitSignals);
    bool cancelOwnedTasks(int timeoutMs);

    bool              m_initialized{false};
    lcnc::ModuleTaskScope m_taskScope;

    CamConfig         m_config;

    // ── Renderers (v2.2: AIS state moved to view/ layer) ──────────────
    std::unique_ptr<lcnc::view::ToolpathRenderer>      m_toolpathRenderer;
    std::unique_ptr<lcnc::view::MachineGuideRenderer>  m_guideRenderer;
    std::unique_ptr<lcnc::view::TravelPathRenderer>    m_travelPathRenderer;
    std::unique_ptr<lcnc::view::ContourOrderLabelRenderer> m_contourOrderLabelRenderer;
    std::unique_ptr<lcnc::cam::CamDisplayProjectionService> m_displayProjectionService;

    // ── CAM data managers ─────────────────────────────────────────────
    /// 借用自 LcncProjectManager（工程核心数据，core 层拥有）；本模块不负责其生命周期。
    lcnc::cam::CamDataManager*                          m_camData{nullptr};
    std::shared_ptr<const lcnc::cam::MotionCompilationInput> m_motionCompilationInput;
    std::uint64_t m_motionCompilationResultRevision{0};
    QVector<std::uint64_t> m_motionCompilationResultOrder;
    /// 借用自 Kernel（独立机台参考资产，core 拥有）；本模块不负责其生命周期。
    lcnc::cam::MachineWorkspace*                         m_machineWorkspace{nullptr};

    TopoDS_Shape                m_workpieceShape;
    TopoDS_Shape                m_cutterDisplayProxyShape;
    std::uint64_t               m_collisionConfigurationRevision{1};
    mutable lcnc::cam::TravelPlanSnapshot m_travelPlanCache;
    /// Companion geometry: continuous IK also rewrites successor contour axes.
    mutable lcnc::cam::ToolpathExportSnapshot m_travelSolutionCache;
    /// Immutable collision-only meshes/bounds.  It is filled by the background
    /// rapid verifier and reused while the machine/environment key is stable.
    /// 中文翻译：仅碰撞使用的不可变网格/包围盒，由后台任务建立并按环境键复用。
    lcnc::cam::JobSafetyOverlayManager m_jobSafetyOverlayManager;
    TaskId                      m_travelVerificationTask{kInvalidTaskId};
    TaskId                      m_collisionDomainPreparationTask{kInvalidTaskId};
    TaskId                      m_machineSafetyPackageBuildTask{kInvalidTaskId};
    TaskId                      m_modelEnvelopeExportTask{kInvalidTaskId};
    QMap<QString, QString>      m_mountedWorkpieceEntryBySourceEntry;
    mutable QList<Handle(AIS_Shape)> m_camContourAisCache;
    QString                     m_machineModelPath;
    QString                     m_loadedMachineModelPath;
    lcnc::cam::MachineSafetyPackageManager m_machineSafetyPackageManager;
    std::uint64_t               m_machineGeometryRevision{0};
    std::uint64_t               m_machineLoadGeneration{0};
    std::atomic_bool            m_machineLoadPending{false};
    lcnc::RenderQualityPreset   m_machineRenderQualityPreset{lcnc::RenderQualityPreset::Medium};
    gp_Pnt                      m_cutterHeadModelPosition{0.0, 0.0, 0.0};
    gp_Pnt                      m_cutterHeadPhysicalPosition{0.0, 0.0, 0.0};
    double                      m_smoothAngle{5.0};
    bool                        m_useFaceClassification{true};
    int                         m_extractionStrategy{0}; ///< LargestSmoothConnectedSurface

    using MachiningFaceEntry = lcnc::cam::MachiningFacePipelineService::Entry;
    std::unique_ptr<lcnc::cam::MachiningFacePipelineService> m_machiningFacePipeline;
    const std::vector<MachiningFaceEntry>& machiningFaces() const noexcept
    {
        return m_machiningFacePipeline->entries();
    }
    bool                        m_machiningFacesVisible{true};

    double                      m_deflection{0.1};
    lcnc::MachineConfigurationService* m_machineConfig{nullptr};
    bool                        m_machineModelVisible{false};
    QSet<ProjectWorkspaceId>    m_machineVisibleWorkspaceIds;
    QSet<QString>               m_visibleMachineEntries;
    bool                        m_machineVisibilityInitialized{false};
    bool                        m_clearingToolpath{false};
    lcnc::cam::ContourId        m_activeContourId{0};

    // ── Lead-in picking preview ────────────────────────────────────────
    int    m_previewLeadInContour{-1};
    int    m_previewLeadInPointIndex{-1};
    gp_Pnt m_previewLeadInPoint;
    double m_previewLeadInParam{0.0};
    bool   m_previewLeadInValid{false};

    // ── Pose state + 局部刷新 coalescer ─────────────────────────────────
    /// 当前姿态显式状态对象；CAM 持有，跨模块共享读/写。
    std::unique_ptr<lcnc::MachinePose> m_pose;
    /// 16ms 单次定时器：合并连续 poseChanged，60Hz 上限。
    QTimer* m_refreshCoalescer{nullptr};
    /// 待刷新的轴名集合（dirty 集合）；空集合表示需要做整体刷新。
    QSet<QString> m_pendingDirtyAxes;
    /// pose→cam 自更新过程中的 reentry 防护。
    bool m_inPoseSelfUpdate{false};

    /// 上一帧 OCC 视图中已选的 CAM contourId 集合，用于把"新增/移除"差分推给 SelectionService。
    QSet<std::uint64_t> m_lastCamSelectionContourIds;
    std::vector<lcnc::SubscriptionId> m_eventSubscriptions;
};
