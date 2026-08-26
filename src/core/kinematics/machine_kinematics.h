#pragma once

#include <QObject>
#include <QList>
#include <QMap>
#include <QString>

#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include "core/kinematics/machine_topology.h"

/**
 * @brief Definition of one kinematic axis of the machine.
 */
struct MachineAxisDef
{
    enum MotionType { Linear, Rotary };

    QString    name;                 ///< "BASE", "X", "Y", "Z", "A", "B", "C"
    MotionType motionType{Linear};
    gp_Dir     direction;            ///< translation direction (Linear) or rotation axis (Rotary)
    gp_Pnt     origin{0.0, 0.0, 0.0};///< pivot/origin point in machine coordinates
    double     minVal{-999.0};       ///< travel limit (mm or °)
    double     maxVal{ 999.0};
    QString    parentAxis;           ///< kinematic parent; empty = world
    lcnc::MachineAxisRole role{lcnc::MachineAxisRole::Unspecified}; ///< semantic role used by solver selection
    double     currentPos{0.0};      ///< current commanded position
};

/**
 * @brief Manages the machine's kinematic model.
 *
 * Responsibilities:
 *  - Stores axis definitions loaded from a preset configuration.
 *  - Maps document label entries (TDF label strings) to axes for both
 *    machine body shapes and workpieces.
 *  - Computes world transforms for shapes/workpieces at current axis positions.
 *  - Provides auto-detection of axis assignments by shape name.
 */
class MachineKinematics : public QObject
{
    Q_OBJECT
public:
    explicit MachineKinematics(QObject* parent = nullptr);

    // ── Preset configuration ──────────────────────────────────────────────────
    /// Load a standard kinematic preset.
    /// @param configType  "VERTICAL_AC_TABLE" | "VERTICAL_BC_TABLE" |
    ///                    "AB_HEAD" | "AC_HEAD"
    void    loadPreset(const QString& configType);
    void    setAxes(const QList<MachineAxisDef>& axes, const QString& configType = QString());
    QString configType() const { return m_configType; }    /// Reset all axes, assignments, and mounts.
    void    clear();
    // ── Axis definitions ──────────────────────────────────────────────────────
    const QList<MachineAxisDef>& axes() const { return m_axes; }
    MachineAxisDef*              findAxis(const QString& name);
    const MachineAxisDef*        findAxis(const QString& name) const;
    gp_Pnt                       axisOrigin(const QString& axisName) const;
    bool                         setAxisOrigin(const QString& axisName, const gp_Pnt& origin);
    bool                         setAxisLimits(const QString& axisName, double minVal, double maxVal);

    // ── Shape–axis assignments (machine body parts) ───────────────────────────
    void     assignShape(const QString& labelEntry, const QString& axisName);
    void     unassignShape(const QString& labelEntry);
    QString  axisForShape(const QString& labelEntry) const;
    QStringList shapesForAxis(const QString& axisName) const;
    const QMap<QString,QString>& shapeAssignments() const { return m_shapeToAxis; }

    // ── Workpiece mounting ────────────────────────────────────────────────────
    void     mountWorkpiece(const QString& wpcEntry, const QString& axisName);
    void     unmountWorkpiece(const QString& wpcEntry);
    QString  mountedAxis(const QString& wpcEntry) const;
    QStringList workpiecesOnAxis(const QString& axisName) const;
    const QMap<QString,QString>& wpcMounts() const { return m_wpcToAxis; }

    // ── Transform computation ─────────────────────────────────────────────────
    /// World transform for a machine body shape at current axis positions.
    gp_Trsf computeShapeTransform(const QString& labelEntry) const;
    /// World transform for a mounted workpiece at current axis positions.
    gp_Trsf computeWpcTransform(const QString& wpcEntry)     const;
    void setWorkpieceSetupTransform(const gp_Trsf& transform);
    /// Immutable CAD-to-fixture setup captured by read-only consumers such as
    /// the offline simulation sandbox.
    /// 中文翻译：供离线仿真等只读消费者捕获的 CAD 到夹具安装变换。
    const gp_Trsf& workpieceSetupTransform() const { return m_workpieceSetupTransform; }
    /// World transform for an axis node at current axis positions.
    gp_Trsf computeAxisTransform(const QString& axisName) const;
    /// Returns true when axisName is ancestorAxis itself or is carried by its
    /// descendant chain. Cyclic or incomplete parent chains fail closed.
    bool isAxisDescendantOf(const QString& axisName, const QString& ancestorAxis) const;
    /// Current controller X/Y/Z coordinates reconstructed in the machine-world
    /// frame. The result is independent of the imported machine CAD placement.
    gp_Pnt currentLinearPosition() const;

    /// Nominal laser beam direction in machine space at the home posture.
    /// All current presets carry the laser head on the machine Z axis, so at
    /// home the beam travels along -Z. Used to identify the machining face.
    gp_Dir  nominalBeamDirectionMachine() const;

    /// Workpiece->machine transform at the home posture (rotary axes treated
    /// as zero), independent of the live commanded positions. Used to derive
    /// the beam direction in workpiece coordinates for machining-face detection.
    gp_Trsf computeWpcTransformHome(const QString& wpcEntry) const;

    // ── Axis position control ─────────────────────────────────────────────────
    /// Clamp to [minVal, maxVal] and emit axisPositionChanged.
    void setAxisPosition(const QString& axisName, double pos);

    // ── Auto-detection ────────────────────────────────────────────────────────
    /// Heuristically assign shapes to axes by matching shape names to
    /// axis name keywords.
    /// @param entryToName  map from TDF label entry string to shape display name
    void autoDetect(const QMap<QString,QString>& entryToName);

signals:
    void axisPositionChanged(const QString& axisName, double pos);
    void assignmentsChanged();

private:
    gp_Trsf axisLocalTrsf(const MachineAxisDef& axis, bool home = false) const;
    gp_Trsf chainTrsf     (const QString& axisName, bool home = false) const;
    void    removeInvalidAssignments();

    QString               m_configType;
    QList<MachineAxisDef> m_axes;
    QMap<QString,QString> m_shapeToAxis;   ///< labelEntry → axisName (machine parts)
    QMap<QString,QString> m_wpcToAxis;     ///< wpcEntry   → axisName (workpieces)
    gp_Trsf               m_workpieceSetupTransform;
};
