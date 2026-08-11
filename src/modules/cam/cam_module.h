
#pragma once

#include <QObject>
#include <QColor>
#include <QList>
#include <QMap>
#include <QSet>
#include <QString>
#include <QVector>

#include <memory>

#include "core/task/module_task_scope.h"

#include "core/project/cam/cam_data_contracts.h"
#include "core/project/cam/cam_data_manager.h"
#include "modules/cam/settings/cam_config.h"
#include "modules/cam/i_cam_facade.h"
#include "modules/cam/contracts/i_cam_project_explorer_projection.h"
#include "modules/cam/services/machining_face_pipeline_service.h"
#include "modules/cam/i_cam_toolpath_provider.h"
#include "core/algorithms/cam/laser_toolpath.h"
#include "core/kernel/i_module.h"
#include "core/kernel/i_service.h"
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
     * 由 @ref DialogAxisCalibrationWizard 在用户依次拾取 A 轴参考面、
     * C 轴参考面、切割头下端面并填入物理 AC 中心坐标后整合而成。
     * 仅在 VERTICAL_AC_TABLE 构型下有效。
     */
    struct AxisCalibrationInputs {
        gp_Pnt aFaceCenter{0.0, 0.0, 0.0};       ///< A 轴参考面中心（模型坐标）
        gp_Pnt cFaceCenter{0.0, 0.0, 0.0};       ///< C 轴参考面中心（模型坐标）
        gp_Pnt cutterHeadFaceCenter{0.0, 0.0, 0.0}; ///< 切割头下端面中心（模型坐标）
        gp_Pnt physicalAcCenter{0.0, 0.0, 0.0};  ///< 兼容字段：新流程中目标中心来自机台构型配置。
        bool   hasPhysicalCenter{true};          ///< 是否需要执行整机平移对齐
        double physicalAAngle{0.0};              ///< 标定位对应的物理 A 角度（度）
        double physicalCAngle{0.0};              ///< 标定位对应的物理 C 角度（度）
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

    /// Remove all machine entities but keep the current machine configuration.
    void unloadMachine();

    /// Export machine model as STEP with LCNC_AXIS_* naming.
    void exportMachine(const QString& filePath);

    /// Auto-detect axis assignments by shape name heuristics.
    void autoDetectAxes();
    void applyAxisAssignments(const QMap<QString, QString>& entryToAxis);
    void assignShapesToAxis(const QStringList& entries, const QString& axisName);
    void unassignShape(const QString& entry);
    void clearAxisAssignments(const QString& axisName);
    QList<AxisOption> axisOptions(bool includeDetachOption = false) const;
    /// Return the preferred workpiece mount axis for the current machine preset.
    QString defaultWorkpieceMountAxis() const;
    gp_Pnt axisOrigin(const QString& axisName) const;
    void setAxisOrigin(const QString& axisName, const gp_Pnt& origin);
    bool setAxisLimits(const QString& axisName, double minVal, double maxVal);
    bool supportsAcCenterCalibration() const;
    bool currentAcRotationCenter(gp_Pnt& center) const;
    gp_Pnt cutterHeadModelPosition() const;
    gp_Pnt cutterHeadPhysicalPosition() const;
    void setCutterHeadModelPosition(const gp_Pnt& position);
    void setCutterHeadPhysicalPosition(const gp_Pnt& position);
    bool fillAxisOriginFromReferenceFace(WidgetOccView* occView,
                                         const QPoint& screenPos,
                                         const QString& axisName);
    bool setCutterHeadModelPositionFromReferenceFace(WidgetOccView* occView,
                                                     const QPoint& screenPos);
    /// 拾取一个平面参考面，返回其几何中心（已叠加 LocalTransformation）。
    /// 仅做查询、不修改任何模块状态，供标定向导使用。
    bool pickReferenceFaceCenter(WidgetOccView* occView,
                                 const QPoint& screenPos,
                                 gp_Pnt& center,
                                 QString* errorMessage = nullptr) const;
    bool alignMachineToPhysicalCenter(const gp_Pnt& physicalCenter);
    bool alignMachineToPhysicalCutterHead();
    /// 三段式模型对齐：用拾取到的模型参考交点平移机台几何，使其对齐到构型配置页
    /// 中手动填写的旋转中心。此流程不写入/修改 A/C 物理旋转中心。
    bool applyAxisCalibration(const AxisCalibrationInputs& inputs, QString* errorMessage = nullptr);
    // 中文翻译：机台标定位
    /// 进入"Machine mark positioning"：记录切割头模型点，并把 A=0/C=0、X/Y 调整为
    /// 让切割头世界 XY 与当前配置旋转中心 XY 对齐。不修改旋转中心、不持久化、不导出。
    /// 仅用于向导显示标定姿态下的当前 AC 中心 / 切割嘴位置。
    bool enterStandardCalibrationPose(const AxisCalibrationInputs& inputs,
                                      QString* errorMessage = nullptr);
    /// 切割头当前世界坐标（受当前 X/Y/Z 轴位置影响）。
    gp_Pnt cutterHeadWorldPosition() const;
    /// 当前生效的标定 AC 角度偏移（度）；未标定时返回 false。
    bool acAngleOffset(double& outA, double& outC) const;
    /// 当前机台已记录的物理 AC 中心 XYZ（mm）；未标定时返回 false。
    bool physicalAcCenter(gp_Pnt& outCenter) const;
    /// 当前机台是否已经完成至少一次三段式标定（cam.toml 里有完整记录）。
    /// 用于向导启动时回填 + 状态指示。
    bool isMachineCalibrated() const;
    QList<WorkpieceMountCandidate> mountableWorkpieces() const;
    bool autoInstallWorkpiece() const;
    void setAutoInstallWorkpiece(bool enabled);
    bool autoInstallCurrentWorkpiece();
    QStringList mountedWorkpieceEntriesForSourceEntries(const QStringList& sourceEntries) const;
    QStringList sourceWorkpieceEntriesForMountedEntries(const QStringList& mountedEntries) const;
    void setMountedWorkpieceEntriesVisible(const QStringList& sourceEntries, bool visible);
    void setSelectedMountedWorkpieceEntries(const QStringList& sourceEntries);
    bool supportsWorkpieceRotationAlignment() const;
    /// Sets the unified CAD-to-fixture setup origin to the configured workpiece
    /// rotation center.  It never moves or rewrites CAD geometry.
    bool alignWorkpieceSetupToRotationCenter();
    /// Legacy compatibility entry point.  New UI must use
    /// alignWorkpieceSetupToRotationCenter().
    bool alignWorkpieceInstallPositionToRotationCenter();

    // ── Workpiece Installation ───────────────────────────────────────────
    /// Mount the current Workpiece source document to an axis.  Placement is
    /// exclusively defined by WorkpieceSetupTransform and geometry is never moved.
    void mountWorkpiece(DocumentId sourceDocId, const QString& axisName, bool alignToInstallPosition = true);

    /// Clear workpiece-axis bindings without deleting Workpiece section geometry.
    void unmountAllWorkpieces();

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
    bool generateToolpath(double smoothAngle, bool useFaceClassification, double deflection = 0.1);
    TaskId generateToolpathAsync(double smoothAngle, bool useFaceClassification,
                                 double deflection = 0.1,
                                 AutoPipelineFaceMode mode = AutoPipelineFaceMode::AutoSeparate);
    void clearToolpath();

    // ── Explicit CAM pipeline ───────────────────────────────────────────
    /// Compute the first-stage face set.  Manual entries are retained and the
    /// result is the only face input intended for subsequent stages.
    bool separateMachiningFaces();
    /// Asynchronously identify automatic machining faces from the workpiece.
    /// Manual mode remains an explicit Apply action because it has no geometry
    /// recognition work to schedule.
    TaskId separateMachiningFacesAsync();
    /// Commit the current manually edited face set and invalidate all
    /// downstream stages.  It never regenerates faces behind the user's back.
    bool applyMachiningFaces();
    bool extractContoursFromMachiningFaces();
    bool discretizeCurrentContours();
    bool buildCurrentGeometricToolpath();
    bool solveCurrentGeometricToolpath();
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
    bool solveToolpathForOrder(const QVector<std::uint64_t>& orderedContourIds);
    lcnc::cam::ToolpathExportSnapshot exportToolpathSnapshot() const;
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
    void setActiveContourId(lcnc::cam::ContourId contourId);
    lcnc::cam::ContourId activeContourId() const { return m_activeContourId; }
    int activeContourIndex() const;
    bool setActiveContourLeadInLength(double mm);
    bool setActiveContourDeflection(double mm);
    ContourGenerationParams activeContourPendingParams() const;
    bool activeContourNeedsRecalculation() const;
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
    /// 删除图层；其下轮廓重挂到 reassignTo（0=自动选其余图层）。
    bool removeToolpathLayer(std::uint64_t layerId, std::uint64_t reassignTo = 0);
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

    /// Apply pending parameters and rebuild/solve only the active contour.
    bool recalcToolpath();
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
    /// Faces captured from the last automatic strategy extraction.
    void   setAutoMachiningFaces(const std::vector<TopoDS_Face>& faces,
                                 const QString& workpieceEntry);
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

    /// 取共享的 MachinePose 指针（CAM 持有所有权；UI/控制器只读/写值）。
    lcnc::MachinePose* machinePose() const;

    // ── Selection / Visibility ──────────────────────────────────────────
    void setEntityVisible(const QString& entry, bool visible);
    bool isEntityVisible(const QString& entry) const;
    QStringList visibleMachineEntries() const;
    void setMachineModelVisible(bool visible);
    bool isMachineModelVisible() const;
    void setRotaryAxisGuidesVisible(bool visible);
    bool rotaryAxisGuidesVisible() const;
    void setCutterHeadGuideVisible(bool visible);
    bool cutterHeadGuideVisible() const;
    /// 按 AppSettings 中的刀头外观（颜色/透明度/缩放）重建刀头锥指示器。
    void refreshCutterHeadAppearance();
    void setSelectedEntries(const QStringList& entries);
    QStringList selectedEntries() const;
    void syncSelectionFromView();

    // ── Travel path 虚线显示 ────────────────────────────────────────────
    // 中文翻译：切割路径显示
    /// 切换"Cutting path display"。OFF 时立即擦除；ON 时立刻按当前 plan 重绘。
    void setTravelPathVisible(bool on);
    bool isTravelPathVisible() const;
    /// 按当前 IProcessCuttingPlanProvider 提供的顺序刷新虚线（仅 visible=true 时）。
    void refreshTravelPath();

    // ── 切割链表序号标注显示 ────────────────────────────────────────────
    // 中文翻译：切割链表序号显示
    /// 切换"Cutting sequence number display"。OFF 时立即擦除；ON 时立刻按当前 plan 重绘。
    void setContourOrderLabelVisible(bool on);
    bool isContourOrderLabelVisible() const;
    /// 按当前 IProcessCuttingPlanProvider 提供的顺序刷新序号标注（仅 visible=true 时）。
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
    void workpieceMounted(const QString& entry);
    void workpieceUnmounted();
    void toolpathGenerated();
    void toolpathCleared();
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

private:
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

    /// Refresh axis guide AIS via MachineGuideRenderer.
    void displayAxisGuides();
    void eraseAxisGuideDisplay();
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
    bool translateMachineWorkspace(const gp_Vec& translation, const QString& operationTitle);
    bool translateMachineGeometryOnly(const gp_Vec& translation, const QString& operationTitle);
    void translateToolpathWorldData(const gp_Vec& translation);
    void autoDetectAxisOrigins();
    void applyStoredMachineProfile(const QString& machinePath);
    bool applyConfiguredMachineAxes(bool updateView);
    QVector<lcnc::cam::ContourId> defaultCuttingOrderByCAxis() const;
    void applyDefaultCuttingOrder();
    lcnc::cam::ToolpathExportSnapshot buildToolpathExportSnapshot(
        const std::vector<LaserContour>& contours,
        std::uint64_t revision,
        const QString& description) const;
    void updateToolpathMachineCoordinates();
    bool autoInstallCurrentWorkpieceInternal(bool alignToInstallPosition);
    bool clearMountedWorkpieceDisplay(bool refreshView);
    LcncDocument* workpieceDocument() const;
    DocumentId workpieceDocumentId() const;
    void refreshWorkpieceDisplay();
    void resetWorkpieceDisplayLocation();
    bool translateWorkpieceDocument(const gp_Vec& translation);
    void setCamContoursVisible(bool visible, bool updateView = true);
    void setCamContourVisible(int contourIndex, bool visible, bool updateView = true);
    void applyCamContourVisibility();
    void applyCamContourTransforms();
    void applyToolpathLayerColors(bool updateView = true);
    QList<int> selectedCamContourIndexes() const;

    void refreshMachineDisplay();
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
    /// 借用自 Kernel（独立机台参考资产，core 拥有）；本模块不负责其生命周期。
    lcnc::cam::MachineWorkspace*                         m_machineWorkspace{nullptr};

    TopoDS_Shape                m_workpieceShape;
    QMap<QString, QString>      m_mountedWorkpieceEntryBySourceEntry;
    mutable QList<Handle(AIS_Shape)> m_camContourAisCache;
    QString                     m_machineModelPath;
    lcnc::RenderQualityPreset   m_machineRenderQualityPreset{lcnc::RenderQualityPreset::Medium};
    gp_Pnt                      m_cutterHeadModelPosition{0.0, 0.0, 0.0};
    gp_Pnt                      m_cutterHeadPhysicalPosition{0.0, 0.0, 0.0};
    bool                        m_hasAcAngleOffset{false};
    double                      m_acAngleOffsetA{0.0};
    double                      m_acAngleOffsetC{0.0};
    bool                        m_hasPhysicalAcCenter{false};
    gp_Pnt                      m_physicalAcCenter{0.0, 0.0, 0.0};
    double                      m_smoothAngle{5.0};
    bool                        m_useFaceClassification{true};
    int                         m_extractionStrategy{0}; ///< LargestSmoothConnectedSurface

    using MachiningFaceEntry = lcnc::cam::MachiningFacePipelineService::Entry;
    std::unique_ptr<lcnc::cam::MachiningFacePipelineService> m_machiningFacePipeline;
    std::vector<MachiningFaceEntry>& m_machiningFaces;
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
};
