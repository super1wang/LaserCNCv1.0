#pragma once

#include <QObject>
#include <QList>
#include <QMap>
#include <QString>

#include <gp_Dir.hxx>
#include <gp_Trsf.hxx>

/**
 * @brief Definition of one kinematic axis of the machine.
 */
struct MachineAxisDef
{
    enum MotionType { Linear, Rotary };

    QString    name;                 ///< "BASE", "X", "Y", "Z", "A", "B", "C"
    MotionType motionType{Linear};
    gp_Dir     direction;            ///< translation direction (Linear) or rotation axis (Rotary)
    double     minVal{-999.0};       ///< travel limit (mm or °)
    double     maxVal{ 999.0};
    QString    parentAxis;           ///< kinematic parent; empty = world
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
    QString configType() const { return m_configType; }

    // ── Axis definitions ──────────────────────────────────────────────────────
    const QList<MachineAxisDef>& axes() const { return m_axes; }
    MachineAxisDef*              findAxis(const QString& name);
    const MachineAxisDef*        findAxis(const QString& name) const;

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
    gp_Trsf axisLocalTrsf(const MachineAxisDef& axis) const;
    gp_Trsf chainTrsf     (const QString& axisName)   const;

    QString               m_configType;
    QList<MachineAxisDef> m_axes;
    QMap<QString,QString> m_shapeToAxis;   ///< labelEntry → axisName (machine parts)
    QMap<QString,QString> m_wpcToAxis;     ///< wpcEntry   → axisName (workpieces)
};
