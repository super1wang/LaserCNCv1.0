#pragma once

#include "core/kernel/i_service.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/kinematics/machine_topology.h"
#include "core/settings/toml_config.h"

#include <QObject>
#include <QString>
#include <QList>
#include <QVector>
#include <QHash>

namespace lcnc {

enum class MachineToolpathAlgorithm
{
    ThreeAxis,
    FiveAxisTable,
    FiveAxisHead
};

QString machineToolpathAlgorithmName(MachineToolpathAlgorithm algorithm);

struct MachineAxisRuntimeConfig
{
    // axis.direction is the controller-positive motion direction in the
    // parent zero-pose/world basis. axis.origin is persisted and edited in
    // controller-axis coordinates; axisDefinitions() converts it to the
    // right-handed OCC world point consumed by kinematics and collision code.
    // 中文翻译：direction 表示控制器正向；origin 以轴系坐标编辑/持久化，
    // 进入运动学与碰撞之前由 axisDefinitions() 转为 OCC 右手世界坐标。
    MachineAxisDef axis;
    int controllerIndex{-1};
    int homeIndex{-1};
    // Controller-facing conversion and motion values.  Keeping them here makes
    // MachineConfigurationService the one source of truth for every configured axis.
    double resolution{2000.0};
    double motionSpeed{10.0};
    double lowSpeed{1.0};
    double mediumSpeed{5.0};
    double highSpeed{20.0};
    double acceleration{200.0};
    double jerk{0.0};
    // Only meaningful for rotary tube axes.  It remains harmless for linear axes
    // and avoids a second, name-indexed Axis settings store.
    double pipeDiameter{0.0};
};

class MachineConfigurationService : public QObject, public IService, public TomlConfig
{
    Q_OBJECT
public:
    explicit MachineConfigurationService(QObject* parent = nullptr);

    bool loadDefault();
    bool saveDefault() const;

    QString presetName() const { return m_presetName; }
    void applyPreset(const QString& presetName);

    QVector<MachineAxisRuntimeConfig> axisConfigurations() const { return m_axisConfigs; }
    void setAxisConfigurations(const QVector<MachineAxisRuntimeConfig>& configs);
    void setMachineAxisDefinitions(const QString& presetName, const QList<MachineAxisDef>& axes);
    void setAxisHardwareConfigurations(const QVector<MachineAxisRuntimeConfig>& configs);
    /// Effective runtime definitions. Rotary/linear origins are converted from
    /// controller-axis input coordinates to right-handed OCC world points.
    QList<MachineAxisDef> axisDefinitions() const;

    /// Convert points between controller-axis coordinates (the values shown by
    /// teaching feedback/UI) and the internal right-handed OCC world frame.
    /// The common axis base zero is the origin in both representations; the
    /// configured orthogonal LinearX/Y/Z positive directions form the
    /// conversion basis; either handedness is supported.
    bool axisCoordinatesToWorld(const gp_Pnt& axisPoint, gp_Pnt* worldPoint) const;
    bool worldToAxisCoordinates(const gp_Pnt& worldPoint, gp_Pnt* axisPoint) const;

    MachineToolpathAlgorithm toolpathAlgorithm() const;
    QString toolpathAlgorithmText() const { return machineToolpathAlgorithmName(toolpathAlgorithm()); }
    QList<MachiningMode> supportedMachiningModes() const;
    MachiningMode defaultMachiningMode() const;
    MachineModeDefinition modeDefinition(MachiningMode mode) const;
    bool supportsMachiningMode(MachiningMode mode) const;
    /// Runtime world-space mounting posture consumed by CAM, collision and
    /// rendering. Its XYZ translation is derived from the axis-coordinate
    /// input below; Euler rotations remain right-handed geometric rotations.
    const WorkpieceSetupTransform& workpieceSetupTransform() const { return m_workpieceSetupWorld; }
    /// User-facing/persisted workpiece setup. XYZ is in controller-axis
    /// coordinates so taught values can be entered directly.
    const WorkpieceSetupTransform& workpieceSetupAxisCoordinates() const { return m_workpieceSetupAxis; }
    void setWorkpieceSetupAxisCoordinates(const WorkpieceSetupTransform& setup);
    /// Legacy/world-space import boundary. Converts XYZ to axis coordinates
    /// before storing it; new UI code should call setWorkpieceSetupAxisCoordinates().
    void setWorkpieceSetupTransform(const WorkpieceSetupTransform& worldSetup);
    bool validateConfiguration(QString* errorMessage = nullptr) const;
    bool validateCandidateConfiguration(const QString& presetName,
                                        const QList<MachineAxisDef>& axes,
                                        const HeadToolGeometry& headGeometry,
                                        QString* errorMessage = nullptr) const;
    const HeadToolGeometry& headToolGeometry() const { return m_headToolGeometry; }
    void setHeadToolGeometry(const HeadToolGeometry& geometry);
    /// Stable SHA-256 identity of the effective preset and axis runtime configuration.
    QString configurationFingerprint() const;

    void syncFromKinematics(const MachineKinematics* kinematics);

signals:
    void machineConfigurationChanged();

protected:
    void readFrom(const toml::value& root) override;
    void writeTo(toml::value& root) const override;
    const char* configName() const override { return "MachineConfiguration"; }

private:
    QVector<MachineAxisRuntimeConfig> mergedAxisConfigurations(
        const QList<MachineAxisDef>& axes) const;
    static QString defaultFilePath();
    static QVector<MachineAxisRuntimeConfig> defaultAxisConfigsForPreset(const QString& presetName);
    static MachineAxisRuntimeConfig defaultRuntimeConfig(const MachineAxisDef& axis, int index);
    static MachineAxisDef defaultAxisDefinition(const QString& name,
                                                MachineAxisDef::MotionType motionType,
                                                const QString& parentAxis);
    static MachineAxisRole defaultRoleForAxis(const QString& presetName,
                                              const QString& axisName,
                                              MachineAxisDef::MotionType motionType);
    void refreshCoordinateDerivedState();
    void setPresetDefaults(const QString& presetName);
    void notifyChanged();

    QString m_presetName{QStringLiteral("VERTICAL_AC_TABLE")};
    QVector<MachineAxisRuntimeConfig> m_axisConfigs;
    HeadToolGeometry m_headToolGeometry;
    WorkpieceSetupTransform m_workpieceSetupAxis;
    WorkpieceSetupTransform m_workpieceSetupWorld;
    MachiningMode m_configuredDefaultMode{MachiningMode::Planar3Axis};
    QHash<MachiningMode, QMap<QString, double>> m_lockedTargetOverrides;
};

} // namespace lcnc
