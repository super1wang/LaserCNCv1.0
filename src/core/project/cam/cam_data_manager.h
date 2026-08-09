#pragma once

#include "core/algorithms/cam/laser_toolpath.h"
#include "core/project/cam/cam_data_contracts.h"
#include "core/project/cam/layer_container.h"
#include "core/project/cam/layer_manager.h"
#include "core/kinematics/machine_topology.h"

#include <QHash>
#include <QList>
#include <QColor>

#include <array>
#include <cstdint>
#include <memory>

namespace lcnc::cam {

/// Ordered, persisted CAM pipeline boundaries.  A later stage may only be
/// consumed when all of its inputs are current.
enum class CamPipelineStage : int {
    FaceSeparation = 0,
    ContourExtraction,
    PointDiscretization,
    GeometricToolpath,
    MachineSolve,
    Count
};

struct CamPipelineStageState {
    bool available{false};
    bool dirty{true};
    std::uint64_t revision{0};
    std::uint64_t inputRevision{0};
    QString failureReason;
};

enum class MachiningFaceRole : int {
    MachiningSurface = 0,
    CrossSection = 1
};

/**
 * @brief Project-core CAM runtime data owner (core layer).
 *
 * Owned by LcncProjectManager (the project core), not by the CAM module. Holds
 * the dense CAM runtime data — contours, layers, lead-ins, machine coordinates,
 * and process parameters. The CAM module borrows this instance to run algorithms
 * and drive rendering. Sparse OCC geometry that should be visible/selectable in
 * the project tree is mirrored into the CAM document by CamModule; dense point
 * arrays intentionally stay here to avoid bloating OCAF.
 *
 * Persistence lives in core too: lcnc::cam::saveCamToolpath / loadCamToolpath
 * (cam_toolpath_io.h), invoked transactionally by LcncProjectManager alongside
 * the workpiece geometry.
 *
 * ID stability
 * ------------
 * Contour and layer IDs are paired with a deterministic ``signature``
 * (see LaserContour::signature / ToolpathLayer::signature) so that
 * re-running generateToolpath() preserves the same IDs across regenerations
 * and across sessions when the toolpath state is persisted.
 *
 * The two ``m_signatureTo*`` maps are the "historical mapping" – they remember
 * which ID was assigned to each signature from the previous generation.
 * ensureContourIds() / ensureToolpathLayers() consult these maps first; if a
 * signature is found, the old ID is reused.
 */
class CamDataManager
{
public:
    CamDataManager();
    ~CamDataManager();

    /// OCC-free record of a machining face for save/reload.
    /// After project load, CamModule rebinds the stored signatures to the
    /// workpiece's actual TopoDS_Face instances.
    struct MachiningFaceRecord {
        std::uint64_t faceId{0};
        std::uint64_t signature{0};
        QString workpieceEntry;
        bool manual{false};
        MachiningFaceRole role{MachiningFaceRole::MachiningSurface};
    };

    /// Set the machining-face records from the CAM module (called after
    /// generateToolpath or manual add/remove).
    void setMachiningFaceRecords(const std::vector<MachiningFaceRecord>& records)
    { m_machiningFaceRecords = records; }

    /// Machining-face records known at last save / current runtime.
    const std::vector<MachiningFaceRecord>& machiningFaceRecords() const
    { return m_machiningFaceRecords; }

    LaserToolpath& toolpath() { return m_toolpath; }
    const LaserToolpath& toolpath() const { return m_toolpath; }

    /// 工程级刀路生成参数（随工程持久化，保证重开可复现同一刀路）。
    /// 新建工程时由 CAM 模块用全局 cam.toml 默认值播种。
    struct GenerationParams {
        double leadInLength{5.0};
        double deflection{0.1};
        double smoothAngle{5.0};
        bool   useFaceClassification{true};
        int    extractionStrategy{0}; ///< ExtractionStrategy (Auto)
        double normalSampleStep{2.0};
    };
    GenerationParams&       generationParams()       { return m_generationParams; }
    const GenerationParams& generationParams() const { return m_generationParams; }
    GenerationParams&       appliedGenerationParams()       { return m_appliedGenerationParams; }
    const GenerationParams& appliedGenerationParams() const { return m_appliedGenerationParams; }
    bool generationParamsDirty() const { return m_generationParamsDirty; }
    void setGenerationParamsDirty(bool dirty) { m_generationParamsDirty = dirty; }

    MachiningMode machiningMode() const { return m_machiningMode; }
    void setMachiningMode(MachiningMode mode) { m_machiningMode = mode; }
    const MachineAxisLayout& machineAxisLayout() const { return m_machineAxisLayout; }
    void setMachineAxisLayout(const MachineAxisLayout& layout) { m_machineAxisLayout = layout; }
    const QString& solverId() const { return m_solverId; }
    void setSolverId(const QString& id) { m_solverId = id; }
    int solverVersion() const { return m_solverVersion; }
    void setSolverVersion(int version) { m_solverVersion = version; }
    const QString& solvedMachineConfigurationFingerprint() const { return m_solvedMachineConfigurationFingerprint; }
    void setSolvedMachineConfigurationFingerprint(const QString& fingerprint)
    { m_solvedMachineConfigurationFingerprint = fingerprint; }

    /// Pipeline stage state is project data, not UI state.  It deliberately
    /// preserves stale downstream snapshots for inspection while preventing
    /// Process from treating them as executable output.
    const CamPipelineStageState& pipelineStageState(CamPipelineStage stage) const;
    void commitPipelineStage(CamPipelineStage stage, std::uint64_t inputRevision = 0);
    void invalidatePipelineAfter(CamPipelineStage stage, const QString& reason);
    void failPipelineStage(CamPipelineStage stage, const QString& reason);
    void clearPipelineStages();
    void restorePipelineStageState(CamPipelineStage stage, const CamPipelineStageState& state);
    bool hasCompletePipelineChain() const;

    /// Phase A: 新引入的图层容器与 Qt 信号源。
    /// 现阶段是 LaserToolpath 上层的薄包装，Phase B 起逐步成为图层级状态的唯一权威。
    LayerContainer&       layerContainer()       { return m_layerContainer; }
    const LayerContainer& layerContainer() const { return m_layerContainer; }
    LayerManager*         layerManager()       { return m_layerManager.get(); }
    const LayerManager*   layerManager() const { return m_layerManager.get(); }

    bool hasToolpath() const { return m_toolpath.contourCount() > 0; }
    void clearToolpath(bool resetIds = true);

    ContourId contourIdAt(int contourIdx) const;
    int contourIndexById(ContourId contourId) const;
    void ensureContourIds();
    void ensureToolpathLayers();
    const std::vector<ToolpathLayer>& toolpathLayers() const { return m_toolpath.layers(); }
    ToolpathLayer* toolpathLayer(std::uint64_t layerId);
    const ToolpathLayer* toolpathLayer(std::uint64_t layerId) const;
    QList<int> contourIndexesInLayer(std::uint64_t layerId) const;

    // ── 三级容器：工程文档级图层增删改名（重命名走 updateToolpathLayer）─────────
    /// 新建一个空图层；返回分配的 layerId。
    std::uint64_t addLayer(const QString& name, const QColor& color = QColor());
    /// 删除图层；其下轮廓改挂到 reassignTo（为 0 或非法时挂到第一个其余图层）。
    /// 至少保留一个图层时才删除；成功返回 true。
    bool removeLayer(std::uint64_t layerId, std::uint64_t reassignTo = 0);
    /// 删除图层及其下所有轮廓（不重挂）。若为最后一个图层则保留空图层。
    bool removeLayerWithContours(std::uint64_t layerId);
    bool updateToolpathLayer(std::uint64_t layerId,
                             const QString& name,
                             const QColor& color,
                             const QString& toolName);
    bool setToolpathLayerEnabled(std::uint64_t layerId, bool enabled);
    bool assignContourToLayer(ContourId contourId, std::uint64_t layerId);
    /// 批量把多条轮廓移动到指定图层（仅一次 syncLayerContourIds + 一次通知）。
    bool assignContoursToLayer(const QList<ContourId>& contourIds, std::uint64_t layerId);
    bool reorderContours(const QList<int>& order);
    bool reorderContoursById(const QList<ContourId>& order);

    void markDirty(bool dirty = true) { m_dirty = dirty; }
    bool isDirty() const { return m_dirty; }

    /// --- ID stability helpers (see class doc) ---

    /// Flush the current signature→id mapping into persistent tables.
    /// Called by CamModule after a successful generateToolpath().
    void commitToolpathStates();

    /// Restore signature→id tables from a prior session (called on project load).
    void restoreSignatureTables(const QHash<std::uint64_t, std::uint64_t>& sigToContour,
                                const QHash<std::uint64_t, std::uint64_t>& sigToLayer,
                                ContourId nextContour,
                                std::uint64_t nextLayer);

    /// Replace the entire toolpath in one shot (used by loadToolpathFromDir).
    void replaceToolpath(LaserToolpath&& toolpath,
                         ContourId nextContour,
                         std::uint64_t nextLayer);

    /// Access the current signature mapping (for persistence).
    QHash<std::uint64_t, std::uint64_t> signatureToContourId() const { return m_signatureToContourId; }
    QHash<std::uint64_t, std::uint64_t> signatureToLayerId() const { return m_signatureToLayerId; }

private:
    ContourId nextContourId();
    std::uint64_t nextLayerId();
    void syncLayerContourIds();

    LaserToolpath m_toolpath;
    GenerationParams m_generationParams;
    GenerationParams m_appliedGenerationParams;
    bool m_generationParamsDirty{false};
    MachiningMode m_machiningMode{MachiningMode::Planar3Axis};
    MachineAxisLayout m_machineAxisLayout;
    QString m_solverId{QStringLiteral("Planar3Axis")};
    int m_solverVersion{machiningModeSolverVersion(MachiningMode::Planar3Axis)};
    QString m_solvedMachineConfigurationFingerprint;
    ContourId m_nextContourId{1};
    std::uint64_t m_nextLayerId{1};
    bool m_dirty{false};

    /// Machining-face records for save/reload (OCC-free, signature-based).
    std::vector<MachiningFaceRecord> m_machiningFaceRecords;

    std::array<CamPipelineStageState,
               static_cast<std::size_t>(CamPipelineStage::Count)> m_pipelineStages;

    /// Deterministic signature → allocated id maps (see ID stability doc above).
    QHash<std::uint64_t, std::uint64_t> m_signatureToContourId;
    QHash<std::uint64_t, std::uint64_t> m_signatureToLayerId;

    /// Phase A: 图层容器 + Qt 信号外壳。LayerManager 在 attach 时延迟构造，
    /// 以避免在 Q_OBJECT 头文件不可在该 TU 包含的场景（这里都 OK）。
    LayerContainer                  m_layerContainer;
    std::unique_ptr<LayerManager>   m_layerManager;
};

} // namespace lcnc::cam
