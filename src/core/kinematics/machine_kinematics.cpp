#include "core/kinematics/machine_kinematics.h"
#include "core/math/numeric_constants.h"

#include <gp_Vec.hxx>
#include <gp_Ax1.hxx>
#include <gp_Pnt.hxx>

#include <QSet>

#include <cmath>
#include <utility>


// ── Constructor ───────────────────────────────────────────────────────────────

MachineKinematics::MachineKinematics(QObject* parent)
    : QObject(parent)
{}
// ── Clear ──────────────────────────────────────────────────────────────────────────
void MachineKinematics::clear()
{
    m_configType.clear();
    m_axes.clear();
    m_shapeToAxis.clear();
    m_wpcToAxis.clear();
    emit assignmentsChanged();
}
// ── Preset loading ─────────────────────────────────────────────────────────────

void MachineKinematics::loadPreset(const QString& configType)
{
    m_configType = configType;
    m_axes.clear();
    // NOTE: keep existing shape/wpc mappings so the user doesn't lose work
    // when switching config; they can re-run autoDetect afterwards.

    auto add = [this](const QString& name,
                      MachineAxisDef::MotionType mt,
                      const gp_Dir& dir,
                      double mn, double mx,
                      const QString& parent,
                      lcnc::MachineAxisRole role = lcnc::MachineAxisRole::Unspecified)
    {
        MachineAxisDef d;
        d.name = name;
        d.motionType = mt;
        d.direction  = dir;
        d.minVal     = mn;
        d.maxVal     = mx;
        d.parentAxis = parent;
        d.role       = role;
        m_axes.append(d);
    };

    // BASE is always the world-fixed machine bed
    add("BASE", MachineAxisDef::Linear, gp_Dir(0, 0, 1), 0.0, 0.0, QString());

    if (configType == "XYZ") {
        add("Y", MachineAxisDef::Linear, gp_Dir(0, 1, 0), -400.0,  400.0, "BASE", lcnc::MachineAxisRole::LinearY);
        add("X", MachineAxisDef::Linear, gp_Dir(1, 0, 0), -500.0,  500.0, "Y",    lcnc::MachineAxisRole::LinearX);
        add("Z", MachineAxisDef::Linear, gp_Dir(0, 0, 1), -300.0,  300.0, "X",    lcnc::MachineAxisRole::LinearZ);

    } else if (configType == "XYZA") {
        add("Y", MachineAxisDef::Linear, gp_Dir(0, 1, 0), -400.0,  400.0, "BASE", lcnc::MachineAxisRole::LinearY);
        add("X", MachineAxisDef::Linear, gp_Dir(1, 0, 0), -500.0,  500.0, "Y",    lcnc::MachineAxisRole::LinearX);
        add("Z", MachineAxisDef::Linear, gp_Dir(0, 0, 1), -300.0,  300.0, "X",    lcnc::MachineAxisRole::LinearZ);
        add("A", MachineAxisDef::Rotary, gp_Dir(1, 0, 0), -9999.0, 9999.0, "BASE", lcnc::MachineAxisRole::WorkpieceRotary);

    } else if (configType == "VERTICAL_AC_TABLE") {
        // Spindle chain: BASE → Y → X → Z (carries laser head)
        // Table  chain: BASE → A → C (carries workpiece)
        add("Y", MachineAxisDef::Linear, gp_Dir(0, 1, 0), -400.0,  400.0, "BASE", lcnc::MachineAxisRole::LinearY);
        add("X", MachineAxisDef::Linear, gp_Dir(1, 0, 0), -500.0,  500.0, "Y",    lcnc::MachineAxisRole::LinearX);
        add("Z", MachineAxisDef::Linear, gp_Dir(0, 0, 1), -300.0,  300.0, "X",    lcnc::MachineAxisRole::LinearZ);
        add("A", MachineAxisDef::Rotary, gp_Dir(1, 0, 0), -120.0,  120.0, "BASE", lcnc::MachineAxisRole::TableTilt);
        add("C", MachineAxisDef::Rotary, gp_Dir(0, 0, 1), -9999.0, 9999.0, "A", lcnc::MachineAxisRole::TableSpin);

    } else if (configType == "VERTICAL_BC_TABLE") {
        add("X", MachineAxisDef::Linear, gp_Dir(1, 0, 0), -500.0,  500.0, "BASE", lcnc::MachineAxisRole::LinearX);
        add("Y", MachineAxisDef::Linear, gp_Dir(0, 1, 0), -400.0,  400.0, "X",    lcnc::MachineAxisRole::LinearY);
        add("Z", MachineAxisDef::Linear, gp_Dir(0, 0, 1), -300.0,  300.0, "Y",    lcnc::MachineAxisRole::LinearZ);
        add("B", MachineAxisDef::Rotary, gp_Dir(0, 1, 0), -120.0,  120.0, "BASE", lcnc::MachineAxisRole::TableTilt);
        add("C", MachineAxisDef::Rotary, gp_Dir(0, 0, 1), -9999.0, 9999.0, "B", lcnc::MachineAxisRole::TableSpin);

    } else if (configType == "AB_HEAD") {
        // Gantry: X cross-beam, Y longitudinal, Z vertical; A/B tilt the head
        add("X", MachineAxisDef::Linear, gp_Dir(1, 0, 0), -500.0, 500.0, "BASE", lcnc::MachineAxisRole::LinearX);
        add("Y", MachineAxisDef::Linear, gp_Dir(0, 1, 0), -400.0, 400.0, "BASE", lcnc::MachineAxisRole::LinearY);
        add("Z", MachineAxisDef::Linear, gp_Dir(0, 0, 1), -300.0, 300.0, "X",    lcnc::MachineAxisRole::LinearZ);
        add("A", MachineAxisDef::Rotary, gp_Dir(1, 0, 0),  -90.0,  90.0, "Z",   lcnc::MachineAxisRole::HeadTiltPrimary);
        add("B", MachineAxisDef::Rotary, gp_Dir(0, 1, 0),  -45.0,  45.0, "A",   lcnc::MachineAxisRole::HeadTiltSecondary);

    } else if (configType == "AC_HEAD") {
        add("X", MachineAxisDef::Linear, gp_Dir(1, 0, 0), -500.0, 500.0, "BASE", lcnc::MachineAxisRole::LinearX);
        add("Y", MachineAxisDef::Linear, gp_Dir(0, 1, 0), -400.0, 400.0, "BASE", lcnc::MachineAxisRole::LinearY);
        add("Z", MachineAxisDef::Linear, gp_Dir(0, 0, 1), -300.0, 300.0, "X",    lcnc::MachineAxisRole::LinearZ);
        add("A", MachineAxisDef::Rotary, gp_Dir(1, 0, 0),  -90.0,  90.0, "Z",   lcnc::MachineAxisRole::HeadTiltPrimary);
        add("C", MachineAxisDef::Rotary, gp_Dir(0, 0, 1), -360.0, 360.0, "A",   lcnc::MachineAxisRole::HeadTiltSecondary);
    }

    removeInvalidAssignments();
}

void MachineKinematics::setAxes(const QList<MachineAxisDef>& axes, const QString& configType)
{
    if (!configType.trimmed().isEmpty())
        m_configType = configType.trimmed();
    m_axes = axes;
    bool hasBase = false;
    for (const MachineAxisDef& axis : std::as_const(m_axes)) {
        if (axis.name == QStringLiteral("BASE")) {
            hasBase = true;
            break;
        }
    }
    if (!hasBase) {
        MachineAxisDef base;
        base.name = QStringLiteral("BASE");
        base.motionType = MachineAxisDef::Linear;
        base.direction = gp_Dir(0, 0, 1);
        base.minVal = 0.0;
        base.maxVal = 0.0;
        m_axes.prepend(base);
    }
    removeInvalidAssignments();
    emit assignmentsChanged();
}

// ── Axis lookup ────────────────────────────────────────────────────────────────

MachineAxisDef* MachineKinematics::findAxis(const QString& name)
{
    for (auto& a : m_axes)
        if (a.name == name) return &a;
    return nullptr;
}

const MachineAxisDef* MachineKinematics::findAxis(const QString& name) const
{
    for (const auto& a : m_axes)
        if (a.name == name) return &a;
    return nullptr;
}

gp_Pnt MachineKinematics::axisOrigin(const QString& axisName) const
{
    if (const MachineAxisDef* axis = findAxis(axisName))
        return axis->origin;
    return gp_Pnt(0, 0, 0);
}

bool MachineKinematics::setAxisOrigin(const QString& axisName, const gp_Pnt& origin)
{
    MachineAxisDef* axis = findAxis(axisName);
    if (!axis)
        return false;

    axis->origin = origin;
    return true;
}

bool MachineKinematics::setAxisLimits(const QString& axisName, double minVal, double maxVal)
{
    MachineAxisDef* axis = findAxis(axisName);
    if (!axis)
        return false;

    if (minVal > maxVal)
        std::swap(minVal, maxVal);

    axis->minVal = minVal;
    axis->maxVal = maxVal;
    axis->currentPos = qBound(axis->minVal, axis->currentPos, axis->maxVal);
    return true;
}

// ── Shape assignments ──────────────────────────────────────────────────────────

void MachineKinematics::assignShape(const QString& entry, const QString& axisName)
{
    if (axisName.isEmpty())
        m_shapeToAxis.remove(entry);
    else
        m_shapeToAxis.insert(entry, axisName);
    emit assignmentsChanged();
}

void MachineKinematics::unassignShape(const QString& entry)
{
    m_shapeToAxis.remove(entry);
    emit assignmentsChanged();
}

QString MachineKinematics::axisForShape(const QString& entry) const
{
    return m_shapeToAxis.value(entry);
}

QStringList MachineKinematics::shapesForAxis(const QString& axisName) const
{
    QStringList result;
    for (auto it = m_shapeToAxis.cbegin(); it != m_shapeToAxis.cend(); ++it)
        if (it.value() == axisName) result.append(it.key());
    return result;
}

// ── Workpiece mounting ─────────────────────────────────────────────────────────

void MachineKinematics::mountWorkpiece(const QString& wpcEntry, const QString& axisName)
{
    if (axisName.isEmpty())
        m_wpcToAxis.remove(wpcEntry);
    else
        m_wpcToAxis.insert(wpcEntry, axisName);
    emit assignmentsChanged();
}

void MachineKinematics::unmountWorkpiece(const QString& wpcEntry)
{
    m_wpcToAxis.remove(wpcEntry);
    emit assignmentsChanged();
}

QString MachineKinematics::mountedAxis(const QString& wpcEntry) const
{
    return m_wpcToAxis.value(wpcEntry);
}

QStringList MachineKinematics::workpiecesOnAxis(const QString& axisName) const
{
    QStringList result;
    for (auto it = m_wpcToAxis.cbegin(); it != m_wpcToAxis.cend(); ++it)
        if (it.value() == axisName) result.append(it.key());
    return result;
}

// ── Transform computation ──────────────────────────────────────────────────────

gp_Trsf MachineKinematics::axisLocalTrsf(const MachineAxisDef& axis, bool home) const
{
    gp_Trsf t;  // identity
    if (axis.name == "BASE" || home || axis.currentPos == 0.0) return t;

    if (axis.motionType == MachineAxisDef::Linear) {
        t.SetTranslation(gp_Vec(axis.direction) * axis.currentPos);
    } else {
        t.SetRotation(gp_Ax1(axis.origin, axis.direction),
                      lcnc::math::degreesToRadians(axis.currentPos));
    }
    return t;
}

gp_Trsf MachineKinematics::chainTrsf(const QString& axisName, bool home) const
{
    // Build chain from ROOT to this axis: [ ..., parent, axisName ]
    QStringList chain;
    QString cur = axisName;
    while (!cur.isEmpty()) {
        chain.prepend(cur);
        const MachineAxisDef* def = findAxis(cur);
        if (!def) break;
        cur = def->parentAxis;
    }

    // Compose:  T_root * ... * T_parent * T_axis
    // (A.Multiplied(B)).Transform(P) = A.Transform(B.Transform(P))
    // so chain in order [root … axisName] gives the correct transform chain.
    gp_Trsf result;  // identity
    for (const QString& n : chain) {
        const MachineAxisDef* def = findAxis(n);
        if (def && def->name != "BASE")
            result = result.Multiplied(axisLocalTrsf(*def, home));
    }
    return result;
}

gp_Trsf MachineKinematics::computeShapeTransform(const QString& entry) const
{
    const QString axisName = m_shapeToAxis.value(entry);
    return axisName.isEmpty() ? gp_Trsf() : chainTrsf(axisName);
}

gp_Trsf MachineKinematics::computeWpcTransform(const QString& entry) const
{
    const QString axisName = m_wpcToAxis.value(entry);
    return axisName.isEmpty() ? m_workpieceSetupTransform
                              : chainTrsf(axisName).Multiplied(m_workpieceSetupTransform);
}

void MachineKinematics::setWorkpieceSetupTransform(const gp_Trsf& transform)
{
    m_workpieceSetupTransform = transform;
}

gp_Trsf MachineKinematics::computeAxisTransform(const QString& axisName) const
{
    return axisName.isEmpty() ? gp_Trsf() : chainTrsf(axisName);
}

bool MachineKinematics::isAxisDescendantOf(const QString& axisName,
                                           const QString& ancestorAxis) const
{
    const QString target = ancestorAxis.trimmed().toUpper();
    QString current = axisName.trimmed().toUpper();
    QSet<QString> visited;
    bool matched = false;
    while (!current.isEmpty()) {
        if (visited.contains(current))
            return false;
        visited.insert(current);
        matched = matched || current == target;
        const MachineAxisDef* axis = findAxis(current);
        if (!axis)
            return false;
        current = axis->parentAxis.trimmed().toUpper();
    }
    return matched;
}

gp_Pnt MachineKinematics::currentLinearPosition() const
{
    gp_Vec position(0.0, 0.0, 0.0);
    for (const QString& axisName : {QStringLiteral("X"),
                                    QStringLiteral("Y"),
                                    QStringLiteral("Z")}) {
        const MachineAxisDef* axis = findAxis(axisName);
        if (!axis || axis->motionType != MachineAxisDef::Linear)
            continue;
        position += gp_Vec(axis->direction) * axis->currentPos;
    }
    return gp_Pnt(position.X(), position.Y(), position.Z());
}

gp_Dir MachineKinematics::nominalBeamDirectionMachine() const
{
    // All current presets (XYZ, XYZA, VERTICAL_*_TABLE, AB_HEAD, AC_HEAD)
    // carry the laser head on the machine Z axis; at the home posture the
    // beam travels along -Z. Rotary axes (table or head) tilt the beam or
    // workpiece away from home during 5-axis motion, but face identification
    // is referenced to the setup (home) posture.
    return gp_Dir(0.0, 0.0, -1.0);
}

gp_Trsf MachineKinematics::computeWpcTransformHome(const QString& entry) const
{
    const QString axisName = m_wpcToAxis.value(entry);
    return axisName.isEmpty() ? m_workpieceSetupTransform
                              : chainTrsf(axisName, /*home=*/true).Multiplied(m_workpieceSetupTransform);
}

// ── Position control ───────────────────────────────────────────────────────────

void MachineKinematics::setAxisPosition(const QString& axisName, double pos)
{
    MachineAxisDef* def = findAxis(axisName);
    if (!def) return;
    // currentPos mirrors live machine/controller feedback for display.
    // Do not clamp it here: if the controller reports a value outside the
    // configured planning limits, the view must still show the actual pose.
    def->currentPos = pos;
    emit axisPositionChanged(axisName, def->currentPos);
}

// ── Auto-detection ─────────────────────────────────────────────────────────────

void MachineKinematics::autoDetect(const QMap<QString,QString>& entryToName)
{
    if (m_axes.isEmpty()) return;

    // Build keyword → axisName map
    // Priority order: longer/more-specific keywords override single chars
    using KwPair = QPair<QString,QString>;
    QList<KwPair> kw;  // keyword → axis, checked in order

    kw << KwPair(QStringLiteral("base"),    QStringLiteral("BASE"));
    kw << KwPair(QStringLiteral("bed"),     QStringLiteral("BASE"));
    kw << KwPair(QStringLiteral("frame"),   QStringLiteral("BASE"));
    kw << KwPair(QStringLiteral("body"),    QStringLiteral("BASE"));
    kw << KwPair(QStringLiteral("machine"), QStringLiteral("BASE"));
    // 中文翻译：固定
    kw << KwPair(QString::fromUtf8("Fixed"), QStringLiteral("BASE"));
    // 中文翻译：基座
    kw << KwPair(QString::fromUtf8("base"), QStringLiteral("BASE"));

    for (const auto& axis : m_axes) {
        if (axis.name == QStringLiteral("BASE")) continue;
        const QString al = axis.name.toLower();
        kw << KwPair(al + QStringLiteral("_axis"),  axis.name);
        kw << KwPair(al + QStringLiteral("-axis"),  axis.name);
        kw << KwPair(QStringLiteral("axis_") + al,  axis.name);
        // 中文翻译：_轴
        kw << KwPair(al + QString::fromUtf8("_axis"), axis.name);
        // 中文翻译：轴
        kw << KwPair(al + QString::fromUtf8("axis"),  axis.name);
        kw << KwPair(al + QStringLiteral("_slide"), axis.name);
        kw << KwPair(al + QStringLiteral("_table"), axis.name);
        kw << KwPair(al + QStringLiteral("_head"),  axis.name);
        kw << KwPair(al + QStringLiteral("_col"),   axis.name);
        kw << KwPair(al + QStringLiteral("_"),      axis.name);
        kw << KwPair(QStringLiteral("_") + al,      axis.name);
        kw << KwPair(QStringLiteral(" ") + al,      axis.name);
        kw << KwPair(al + QStringLiteral(" "),      axis.name);
    }

    for (auto it = entryToName.cbegin(); it != entryToName.cend(); ++it) {
        const QString lowerName = it.value().toLower();
        QString matched;

        // Priority 1: canonical LCNC_AXIS_<name> naming from machine export
        if (it.value().startsWith(QStringLiteral("LCNC_AXIS_"), Qt::CaseInsensitive)) {
            const QString axisFromName = it.value().mid(10).toUpper();
            if (findAxis(axisFromName))
                matched = axisFromName;
        }

        // Priority 2: keyword heuristics
        if (matched.isEmpty()) {
            for (const auto& pair : kw) {
                if (lowerName.contains(pair.first)) {
                    matched = pair.second;
                    break;
                }
            }
        }

        // Last resort: shape name is exactly the axis letter
        if (matched.isEmpty()) {
            for (const auto& axis : m_axes) {
                if (axis.name == "BASE") continue;
                if (lowerName == axis.name.toLower()) {
                    matched = axis.name;
                    break;
                }
            }
        }

        if (!matched.isEmpty())
            m_shapeToAxis.insert(it.key(), matched);
    }

    emit assignmentsChanged();
}

void MachineKinematics::removeInvalidAssignments()
{
    auto isValidAxis = [this](const QString& axisName) {
        return axisName.isEmpty() || findAxis(axisName) != nullptr;
    };

    for (auto it = m_shapeToAxis.begin(); it != m_shapeToAxis.end(); ) {
        if (!isValidAxis(it.value()))
            it = m_shapeToAxis.erase(it);
        else
            ++it;
    }

    for (auto it = m_wpcToAxis.begin(); it != m_wpcToAxis.end(); ) {
        if (!isValidAxis(it.value()))
            it = m_wpcToAxis.erase(it);
        else
            ++it;
    }
}
