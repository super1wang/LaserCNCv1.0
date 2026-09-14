#pragma once

#include <QHash>
#include <QList>
#include <QMap>
#include <QString>
#include <QStringList>

#include <array>
#include <cstdint>

class gp_Trsf;

namespace lcnc {

enum class MachineAxisRole
{
    Unspecified = 0,
    LinearX,
    LinearY,
    LinearZ,
    WorkpieceRotary,
    TableTilt,
    TableSpin,
    HeadTiltPrimary,
    HeadTiltSecondary
};

enum class MachiningMode
{
    Planar3Axis = 0,
    RotaryTube4Axis,
    SimultaneousTable5Axis,
    SimultaneousHead5Axis
};

QString machineAxisRoleName(MachineAxisRole role);
MachineAxisRole machineAxisRoleFromName(const QString& name);
QString machiningModeName(MachiningMode mode);
int machiningModeSolverVersion(MachiningMode mode);
MachiningMode machiningModeFromName(const QString& name,
                                    MachiningMode fallback = MachiningMode::Planar3Axis);

struct MachineAxisSlot
{
    QString name;
    MachineAxisRole role{MachineAxisRole::Unspecified};
};

struct MachineAxisLayout
{
    static constexpr int kMaxAxes = 5;

    std::array<MachineAxisSlot, kMaxAxes> axes{};
    std::uint8_t count{0};

    bool append(const QString& name, MachineAxisRole role);
    int indexOfName(const QString& name) const;
    int indexOfRole(MachineAxisRole role) const;
    QStringList axisNames() const;
    bool isValid(QString* errorMessage = nullptr) const;
    bool operator==(const MachineAxisLayout& other) const;
    bool operator!=(const MachineAxisLayout& other) const { return !(*this == other); }
};

struct SolvedMachinePose
{
    std::array<double, MachineAxisLayout::kMaxAxes> values{};
    std::uint8_t activeMask{0};
    bool valid{false};
    QString failureReason;

    bool isActive(int index) const;
    double value(int index) const;
    void setValue(int index, double axisValue, bool active = true);
};

struct WorkpieceSetupTransform
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double rotationXDeg{0.0};
    double rotationYDeg{0.0};
    double rotationZDeg{0.0};

    gp_Trsf toTransform() const;
    bool isIdentity(double tolerance = 1e-9) const;
};

struct HeadToolGeometry
{
    double zeroBeamX{0.0};
    double zeroBeamY{0.0};
    double zeroBeamZ{-1.0};
    double focusLength{0.0};
    double installationOffsetX{0.0};
    double installationOffsetY{0.0};
    double installationOffsetZ{0.0};
};

struct MachineModeDefinition
{
    MachiningMode mode{MachiningMode::Planar3Axis};
    MachineAxisLayout interpolatedAxes;
    QMap<QString, double> lockedAxisTargets;
    QString solverId;
    int solverVersion{0};

    bool isValid(QString* errorMessage = nullptr) const;
};

} // namespace lcnc
