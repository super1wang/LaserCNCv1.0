#pragma once

#include <QObject>
#include <QList>
#include <QMap>
#include "base/lcnc_application.h"
#include "base/laser_toolpath.h"

#include <AIS_Shape.hxx>
#include <TopoDS_Shape.hxx>

class LcncDocument;
class GuiDocument;
class MachineKinematics;
class GraphicsScene;
class QTimer;
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

    // ── Machine Management ───────────────────────────────────────────────
    /// Load machine model from file with a kinematic preset.
    /// @p presetName: "VERTICAL_AC_TABLE", "VERTICAL_BC_TABLE", "AB_HEAD", "AC_HEAD"
    void loadMachine(const QString& filePath, const QString& presetName);

    /// Remove all machine entities and reset kinematics.
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
    QList<WorkpieceMountCandidate> mountableWorkpieces() const;

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
    void setAxisPosition(const QString& axisName, double value);

    // ── Selection / Visibility ──────────────────────────────────────────
    void setEntityVisible(const QString& entry, bool visible);
    void setSelectedEntries(const QStringList& entries);
    QStringList selectedEntries() const;
    void syncSelectionFromView();

signals:
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

    /// Collect the workpiece compound shape from the machine document.
    TopoDS_Shape collectWorkpieceShape() const;

    /// Display contour wires as green AIS shapes.
    void displayContours();
    /// Display lead-in edges as red AIS shapes.
    void displayLeadIns();

    void refreshMachineTransforms();
    void refreshMachineDisplay();

    static CamModule* s_instance;

    // ── Toolpath state (migrated from anonymous namespace in commands_cam.cpp) ──
    LaserToolpath               m_toolpath;
    QList<Handle(AIS_Shape)>    m_contourAis;
    QList<Handle(AIS_Shape)>    m_leadInAis;
    bool                        m_toolpathVisible{true};
    TopoDS_Shape                m_workpieceShape;
    double                      m_smoothAngle{5.0};
    bool                        m_useFaceClassification{true};

    // ── Simulation state ────────────────────────────────────────────────
    QTimer* m_simTimer{nullptr};
    int     m_simCurrentContour{0};
    int     m_simCurrentPoint{0};
    int     m_simTotalPoints{0};
    double  m_simSpeed{1.0};
    bool    m_simPlaying{false};
    bool    m_simPaused{false};
};
