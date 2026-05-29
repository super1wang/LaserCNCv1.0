#include "modules/process/settings/process_settings_schema.h"

#include "modules/process/settings/process_settings.h"

#include <QObject>

namespace lcnc::process {

int ProcessSettingsSchema::currentVersion()
{
    return 1;
}

QList<ProcessSettingsSchemaField> ProcessSettingsSchema::fields()
{
    return {
        { QStringLiteral("device.motionController"), QObject::tr("运动控制器"), QStringLiteral("string"), QStringLiteral("PureSimulation"), {}, {}, {}, QStringLiteral("Setting_MotionControl") },
        { QStringLiteral("device.endpoint"), QObject::tr("控制器地址"), QStringLiteral("string"), {}, {}, {}, {}, QStringLiteral("Setting_MotionControl") },
        { QStringLiteral("device.simulationMode"), QObject::tr("仿真模式"), QStringLiteral("bool"), true, {}, {}, {}, QString() },
        { QStringLiteral("laser.device"), QObject::tr("激光器"), QStringLiteral("string"), QStringLiteral("Simulator"), {}, {}, {}, QStringLiteral("Setting_Laser") },
        { QStringLiteral("laser.energy"), QObject::tr("激光能量"), QStringLiteral("double"), 0.0, 0.0, 1000000.0, QStringLiteral("uJ"), QStringLiteral("Setting_Laser") },
        { QStringLiteral("tool.feedRate"), QObject::tr("进给速度"), QStringLiteral("double"), 10.0, 0.0, 1000000.0, QStringLiteral("mm/s"), QStringLiteral("Setting_Tool") },
        { QStringLiteral("tool.kerfWidth"), QObject::tr("割缝宽度"), QStringLiteral("double"), 0.05, 0.0, 1000000.0, QStringLiteral("mm"), QStringLiteral("Setting_Tool") },
        { QStringLiteral("io.defaultChannel"), QObject::tr("默认 IO 通道"), QStringLiteral("string"), QStringLiteral("DO0"), {}, {}, {}, QStringLiteral("Setting_IOIndex") },
        { QStringLiteral("aux.assistGas"), QObject::tr("辅助气体"), QStringLiteral("string"), QStringLiteral("Air"), {}, {}, {}, QStringLiteral("Setting_Gas") },
    };
}

ProcessTypedSettingsSnapshot ProcessSettingsSchema::snapshotFrom(const lcnc::ProcessSettings& settings)
{
    ProcessTypedSettingsSnapshot snapshot;
    snapshot.schemaVersion = currentVersion();
    snapshot.device.motionControllerName = settings.motionControllerName();
    snapshot.device.controllerEndpoint = settings.controllerEndpoint();
    snapshot.device.simulationMode = settings.simulationMode();
    snapshot.device.axisTravelX = settings.axisTravelX();
    snapshot.device.axisTravelY = settings.axisTravelY();
    snapshot.device.axisTravelZ = settings.axisTravelZ();
    snapshot.device.axisMaxVelocity = settings.axisMaxVelocity();
    snapshot.device.axisAcceleration = settings.axisAcceleration();

    snapshot.tool.laserDeviceName = settings.laserDeviceName();
    snapshot.tool.laserEnergy = settings.laserEnergy();
    snapshot.tool.laserFrequency = settings.laserFrequency();
    snapshot.tool.laserPulseWidth = settings.laserPulseWidth();
    snapshot.tool.feedRate = settings.toolFeedRate();
    snapshot.tool.kerfWidth = settings.toolKerfWidth();
    snapshot.tool.pierceDelayMs = settings.pierceDelayMs();

    snapshot.io.defaultChannel = settings.ioDefaultChannel();
    snapshot.io.defaultValue = settings.ioDefaultValue();

    snapshot.aux.assistGas = settings.assistGas();
    snapshot.aux.gasPressure = settings.gasPressure();
    snapshot.aux.waterCoolingEnabled = settings.waterCoolingEnabled();
    snapshot.aux.waterMinFlow = settings.waterMinFlow();
    snapshot.aux.monitorEnabled = settings.monitorEnabled();
    snapshot.aux.monitorIntervalMs = settings.monitorIntervalMs();
    snapshot.aux.loadingX = settings.loadingPositionX();
    snapshot.aux.loadingY = settings.loadingPositionY();
    snapshot.aux.loadingZ = settings.loadingPositionZ();
    return snapshot;
}

} // namespace lcnc::process