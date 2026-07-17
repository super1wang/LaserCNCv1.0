#pragma once

#include "core/kernel/i_service.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/settings/toml_config.h"

#include <QObject>
#include <QString>
#include <QList>
#include <QVector>

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
    QList<MachineAxisDef> axisDefinitions() const;

    MachineToolpathAlgorithm toolpathAlgorithm() const;
    QString toolpathAlgorithmText() const { return machineToolpathAlgorithmName(toolpathAlgorithm()); }

    void syncFromKinematics(const MachineKinematics* kinematics);

signals:
    void machineConfigurationChanged();

protected:
    void readFrom(const toml::value& root) override;
    void writeTo(toml::value& root) const override;
    const char* configName() const override { return "MachineConfiguration"; }

private:
    static QString defaultFilePath();
    static QVector<MachineAxisRuntimeConfig> defaultAxisConfigsForPreset(const QString& presetName);
    static MachineAxisRuntimeConfig defaultRuntimeConfig(const MachineAxisDef& axis, int index);
    static MachineAxisDef defaultAxisDefinition(const QString& name,
                                                MachineAxisDef::MotionType motionType,
                                                const QString& parentAxis);
    void setPresetDefaults(const QString& presetName);
    void notifyChanged();

    QString m_presetName{QStringLiteral("VERTICAL_AC_TABLE")};
    QVector<MachineAxisRuntimeConfig> m_axisConfigs;
};

} // namespace lcnc
