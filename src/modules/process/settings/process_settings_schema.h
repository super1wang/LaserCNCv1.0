#pragma once

#include <QList>
#include <QString>
#include <QVariant>

namespace lcnc { class ProcessSettings; }

namespace lcnc::process {

/**
 * @brief One typed setting definition used for validation and UI migration.
 */
struct ProcessSettingsSchemaField
{
    QString key;
    QString displayName;
    QString valueType;
    QVariant defaultValue;
    QVariant minValue;
    QVariant maxValue;
    QString unit;
    QString uiFieldId;
};

/**
 * @brief Motion controller and axis parameter snapshot for runtime preparation.
 */
struct ProcessDeviceSettings
{
    QString motionControllerName;
    QString controllerEndpoint;
    bool simulationMode{true};
    double axisTravelX{0.0};
    double axisTravelY{0.0};
    double axisTravelZ{0.0};
    double axisMaxVelocity{0.0};
    double axisAcceleration{0.0};
};

/**
 * @brief Laser and cutting tool defaults consumed by tool matching and planning.
 */
struct ProcessToolSettings
{
    QString laserDeviceName;
    double laserEnergy{0.0};
    double laserFrequency{0.0};
    double laserPulseWidth{0.0};
    double feedRate{0.0};
    double kerfWidth{0.0};
    int pierceDelayMs{0};
};

/**
 * @brief IO defaults used when workflow nodes omit explicit IO parameters.
 */
struct ProcessIoSettings
{
    QString defaultChannel;
    bool defaultValue{false};
};

/**
 * @brief Auxiliary process defaults for gas, water, monitor and loading position.
 */
struct ProcessAuxSettings
{
    QString assistGas;
    double gasPressure{0.0};
    bool waterCoolingEnabled{false};
    double waterMinFlow{0.0};
    bool monitorEnabled{false};
    int monitorIntervalMs{0};
    double loadingX{0.0};
    double loadingY{0.0};
    double loadingZ{0.0};
};

/**
 * @brief Complete typed Process settings snapshot.
 */
struct ProcessTypedSettingsSnapshot
{
    ProcessDeviceSettings device;
    ProcessToolSettings tool;
    ProcessIoSettings io;
    ProcessAuxSettings aux;
    int schemaVersion{1};
};

/**
 * @brief Builds typed settings snapshots and exposes schema metadata.
 */
class ProcessSettingsSchema
{
public:
    static int currentVersion();
    static QList<ProcessSettingsSchemaField> fields();
    static ProcessTypedSettingsSnapshot snapshotFrom(const lcnc::ProcessSettings& settings);
};

} // namespace lcnc::process