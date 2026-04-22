
#pragma once

#include <QObject>
#include <QList>
#include <QMap>

#include "base/cam_config.h"
#include "base/lcnc_application.h"
#include "base/laser_toolpath.h"
#include "base/machine_model_compressor.h"

#include <AIS_Shape.hxx>
#include <TopoDS_Shape.hxx>

class LcncDocument;
class GuiDocument;
class MachineKinematics;
class GraphicsScene;
class QTimer;
class QPoint;
class WidgetOccView;
class gp_Vec;
class gp_Ax1;
class gp_Pnt;

/**
 * @brief CAM module singleton — manages machine, toolpath, and simulation.
 *
 * Responsible for:
 *  - Machine model loading / unloading / export / axis configuration
 *  - Workpiece mounting onto machine axes
 *  - Toolpath generation, lead-in computation, preview display
 *  - Simulation (play / pause / stop / speed control)
 *  - Managing the "准备" (Prepare) tab page
 *  - Owns the machine document and its independent view
 *
 * For shape operations (move/rotate/delete) on machine entities, delegates
 * to ShapeService — sharing the same geometry code as the CAD module.
 */
class CamModule : public QObject
{
    Q_OBJECT
public:
    enum class MachineCompressionStrategy {
        FilledSolid,
        ExteriorShell,
        SewingShell,
        BoundingBoxProxy,
    };

    struct AxisOption {
        QString name;
        QString displayName;
    };

    struct WorkpieceMountCandidate {
        DocumentId documentId{kInvalidDocumentId};
        QString displayName;
        int workpieceCount{0};
    };

    static CamModule* instance();

    // ── Machine Document ─────────────────────────────────────────────────
    LcncDocument*      machineDocument() const;
    GuiDocument*       machineGuiDocument() const;
    DocumentId         machineDocumentId() const;
    MachineKinematics* kinematics() const;
    void               requestMachineView();
    QString            machineModelPath() const;
    void               setMachineModelPath(const QString& filePath);
    MachineRenderQuality machineRenderQuality() const;
    void               setMachineRenderQuality(MachineRenderQuality quality);

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
    bool alignMachineToPhysicalCenter(const gp_Pnt& physicalCenter);
    bool alignMachineToPhysicalCutterHead();
    /// Start machine compression asynchronously; returns false if startup validation fails.
    bool compressMachineModel(MachineCompressionStrategy strategy = MachineCompressionStrategy::FilledSolid);
    QList<WorkpieceMountCandidate> mountableWorkpieces() const;
    gp_Pnt workpieceInstallPosition() const;
    void setWorkpieceInstallPosition(const gp_Pnt& position);
    bool supportsWorkpieceRotationAlignment() const;
    bool alignWorkpieceInstallPositionToRotationCenter();

    // ── Workpiece Mounting ───────────────────────────────────────────────
    /// Mount workpiece from a source document onto a machine axis.
    void mountWorkpiece(DocumentId sourceDocId, const QString& axisName);

    /// Unmount all workpieces from the machine document.
    void unmountAllWorkpieces();

    // ── Shape Operations on Machine Doc (delegates to ShapeService) ──────
    bool moveShape(const QString& entry, const gp_Vec& translation);
    bool rotateShape(const QString& entry, const gp_Ax1& axis, double angleDeg);
    void deleteShape(const QString& entry);

    // ── Toolpath ─────────────────────────────────────────────────────────
    bool generateToolpath(double smoothAngle, bool useFaceClassification, double deflection = 0.1);
    void clearToolpath();
    const LaserToolpath& toolpath() const;
    LaserToolpath& toolpathRef();
    bool hasToolpath() const;

    /// Set lead-in entry point for a specific contour.
    void setLeadInEntry(int contourIdx, const gp_Pnt& entryPoint, double entryParam);
    void setLeadInLength(double mm);
    double leadInLength() const;
    void setNormalAngle(double deg);
    double normalAngle() const;
    void setDeflection(double mm);
    double deflection() const;
    void setContourEnabled(int contourIdx, bool enabled);
    void reorderContours(const QList<int>& order);

    /// Recalculate all lead-in lines and machine coordinates with current parameters.
    void recalcToolpath();

    /// Toggle toolpath display visibility.
    void setToolpathVisible(bool visible);
    bool isToolpathVisible() const;

    // ── Toolpath Parameters ──────────────────────────────────────────────
    double smoothAngle() const;
    void   setSmoothAngle(double deg);
    bool   useFaceClassification() const;
    void   setUseFaceClassification(bool on);
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

    // ── Simulation ───────────────────────────────────────────────────────
    void simulatePlay();
    void simulatePause();
    void simulateStop();
    void setSimulationSpeed(double factor);
    bool isSimulating() const;
    bool isSimPaused() const;

    // ── Axis Position ────────────────────────────────────────────────────
    void setAxisPosition(const QString& axisName, double value, bool refreshNow = true);
    void refreshMachineTransforms();

    // ── Selection / Visibility ──────────────────────────────────────────
    void setEntityVisible(const QString& entry, bool visible);
    void setSelectedEntries(const QStringList& entries);
    QStringList selectedEntries() const;
    void syncSelectionFromView();

signals:
    void machineViewRequested();
    void machineWorkspaceChanged();
    void operationFailed(const QString& title, const QString& message);
    void machineLoaded();
    void machineUnloaded();
    void workpieceMounted(const QString& entry);
    void workpieceUnmounted();
    void toolpathGenerated();
    void toolpathCleared();
    void toolpathVisibilityChanged(bool visible);
    void simulationTick(int contourIdx, int pointIdx, int totalPoints);
    void simulationStateChanged(bool playing);
    void simulationFinished();
    void selectionChanged(const QStringList& entries);
    void axisAssignmentsChanged();

private slots:
    void onSimTick();

private:
    explicit CamModule(QObject* parent = nullptr);

    struct WorkpieceShapeSource {
        QString workpieceEntry;
        TopoDS_Shape shape;
        int componentIndex{0};
    };

    /// Collect the workpiece compound shape from the machine document.
    TopoDS_Shape collectWorkpieceShape() const;
    QList<WorkpieceShapeSource> collectWorkpieceShapes() const;

    /// Display contour wires as green AIS shapes.
    void displayContours();
    /// Display lead-in edges as red AIS shapes.
    void displayLeadIns();
    /// Display sampled normal vectors as overlay lines.
    void displayNormals();
    void displayAxisGuides();
    void eraseAxisGuideDisplay();
    void updateAxisGuideTransforms();
    bool resolveLeadInHit(WidgetOccView* occView,
                          const QPoint& screenPos,
                          int& contourIdx,
                          gp_Pnt& entryPoint,
                          double& entryParam) const;
    bool resolveReferencePlaneCenter(WidgetOccView* occView,
                                     const QPoint& screenPos,
                                     gp_Pnt& center,
                                     QString* errorMessage) const;
    bool ensureAcCenterCalibrationAvailable(QString* errorMessage = nullptr) const;
    bool currentWorkpieceRotationCenter(gp_Pnt& center) const;
    bool translateMachineWorkspace(const gp_Vec& translation, const QString& operationTitle);
    void translateToolpathWorldData(const gp_Vec& translation);
    void autoDetectAxisOrigins();
    void applyStoredMachineProfile(const QString& machinePath);
    gp_Pnt defaultWorkpieceInstallPosition() const;
    void updateToolpathMachineCoordinates();

    void refreshMachineDisplay();

    static CamModule* s_instance;

    // ── Toolpath state (migrated from anonymous namespace in commands_cam.cpp) ──
    LaserToolpath               m_toolpath;
    QList<Handle(AIS_Shape)>    m_contourAis;
    QList<Handle(AIS_Shape)>    m_leadInAis;
    QList<Handle(AIS_Shape)>    m_normalAis;
    QMap<QString, Handle(AIS_Shape)> m_axisGuideAis;
    bool                        m_toolpathVisible{true};
    TopoDS_Shape                m_workpieceShape;
    QString                     m_machineModelPath;
    MachineRenderQuality        m_machineRenderQuality{MachineRenderQuality::Medium};
    gp_Pnt                      m_cutterHeadModelPosition{0.0, 0.0, 0.0};
    gp_Pnt                      m_cutterHeadPhysicalPosition{0.0, 0.0, 0.0};
    gp_Pnt                      m_workpieceInstallPosition{0.0, 0.0, 0.0};
    double                      m_smoothAngle{5.0};
    bool                        m_useFaceClassification{true};
    double                      m_deflection{0.1};

    // ── Normal overlay parameters ─────────────────────────────────────
    bool   m_showNormals{false};
    double m_normalSampleStep{2.0};

    // ── Lead-in picking preview ────────────────────────────────────────
    int    m_previewLeadInContour{-1};
    gp_Pnt m_previewLeadInPoint;
    double m_previewLeadInParam{0.0};
    bool   m_previewLeadInValid{false};

    // ── Simulation state ────────────────────────────────────────────────
    QTimer* m_simTimer{nullptr};
    int     m_simCurrentContour{0};
    int     m_simCurrentPoint{0};
    int     m_simTotalPoints{0};
    double  m_simSpeed{1.0};
    bool    m_simPlaying{false};
    bool    m_simPaused{false};
    bool    m_machineCompressionRunning{false};
};
