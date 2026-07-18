#include "modules/process/settings/process_parameter_registry.h"
#include "modules/process/settings/process_settings_service.h"

#include "core/kernel/kernel.h"
#include "core/kinematics/machine_configuration_service.h"

#include <QObject>
#include <limits>

namespace lcnc::process {
namespace {

ParameterDescriptor field(QString id, QString title, QString group,
                          ParameterValueType type, ProcessConfigArea area,
                          QString table, QString key, QVariant fallback = {},
                          QString unit = {}, double minimum = std::numeric_limits<double>::lowest(),
                          double maximum = std::numeric_limits<double>::max(), int decimals = 3)
{
    ParameterDescriptor result;
    result.id = std::move(id); result.title = std::move(title); result.group = std::move(group);
    result.type = type; result.area = area; result.tableName = std::move(table);
    result.key = std::move(key); result.defaultValue = std::move(fallback); result.unit = std::move(unit);
    result.minimum = minimum; result.maximum = maximum; result.decimals = decimals;
    return result;
}

ParameterDescriptor machineField(QString id, QString title, QString group, ParameterValueType type,
                                 QString key, QString unit = {}, double minimum = 0.0,
                                 double maximum = 1e12, int decimals = 3)
{
    auto result = field(std::move(id), std::move(title), std::move(group), type,
                        ProcessConfigArea::Devices, {}, std::move(key), {}, std::move(unit),
                        minimum, maximum, decimals);
    result.machineAxisField = true;
    return result;
}

QStringList toolNames(const ProcessSettingsService& settings)
{
    return settings.toolDisplayNames();
}

} // namespace

ProcessParameterRegistry::ProcessParameterRegistry(const ProcessSettingsService& settings)
    : m_settings(settings)
{
}

QVector<ParameterObjectDescriptor> ProcessParameterRegistry::buildObjects() const
{
    QVector<ParameterObjectDescriptor> objects;

    ParameterObjectDescriptor controller{QStringLiteral("controller"), QObject::tr("运动控制器"), QObject::tr("运动与轴系")};
#if defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS
    const QString defaultController = QStringLiteral("SimulatorCMHP");
#elif defined(LCNC_PROCESS_HAS_GTN) && LCNC_PROCESS_HAS_GTN
    const QString defaultController = QStringLiteral("GTN");
#else
    const QString defaultController = QStringLiteral("Simulator");
#endif
    auto controllerType = field("type", QObject::tr("控制器类型"), QObject::tr("连接"), ParameterValueType::Enum,
                                ProcessConfigArea::Devices, "MotionControl", "sType", defaultController);
    controllerType.enumValues = {QStringLiteral("Simulator")};
#if defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS
    controllerType.enumValues.append(QStringLiteral("SimulatorCMHP"));
    controllerType.enumValues.append(QStringLiteral("ACS"));
#endif
#if defined(LCNC_PROCESS_HAS_GTN) && LCNC_PROCESS_HAS_GTN
    controllerType.enumValues.append(QStringLiteral("GTN"));
#endif
    controller.fields = {controllerType};
    objects.append(controller);

    auto machine = lcnc::Kernel::current().services().getService<lcnc::MachineConfigurationService>();
    if (machine) {
        for (const auto& axis : machine->axisConfigurations()) {
            ParameterObjectDescriptor object;
            object.id = QStringLiteral("axis:%1").arg(axis.axis.name);
            object.title = axis.axis.name;
            object.category = QObject::tr("运动与轴系");
            object.fields = {
                machineField("controllerIndex", QObject::tr("控制器索引"), QObject::tr("基本"), ParameterValueType::Int, "controllerIndex", {}, 0, 128, 0),
                machineField("homeIndex", QObject::tr("回零索引"), QObject::tr("基本"), ParameterValueType::Int, "homeIndex", {}, 0, 128, 0),
                machineField("resolution", QObject::tr("分辨率"), QObject::tr("基本"), ParameterValueType::Double, "resolution", {}, 0.000001),
                machineField("motionSpeed", QObject::tr("运动速度"), QObject::tr("基本"), ParameterValueType::Double, "motionSpeed", axis.axis.motionType == MachineAxisDef::Rotary ? "°/s" : "mm/s", 0.0),
                machineField("min", QObject::tr("负限位"), QObject::tr("限位"), ParameterValueType::Double, "min", axis.axis.motionType == MachineAxisDef::Rotary ? "°" : "mm"),
                machineField("max", QObject::tr("正限位"), QObject::tr("限位"), ParameterValueType::Double, "max", axis.axis.motionType == MachineAxisDef::Rotary ? "°" : "mm"),
                machineField("lowSpeed", QObject::tr("低速"), QObject::tr("手动速度"), ParameterValueType::Double, "lowSpeed", axis.axis.motionType == MachineAxisDef::Rotary ? "°/s" : "mm/s", 0.0),
                machineField("mediumSpeed", QObject::tr("中速"), QObject::tr("手动速度"), ParameterValueType::Double, "mediumSpeed", axis.axis.motionType == MachineAxisDef::Rotary ? "°/s" : "mm/s", 0.0),
                machineField("highSpeed", QObject::tr("高速"), QObject::tr("手动速度"), ParameterValueType::Double, "highSpeed", axis.axis.motionType == MachineAxisDef::Rotary ? "°/s" : "mm/s", 0.0),
                machineField("acceleration", QObject::tr("加速度"), QObject::tr("运动学"), ParameterValueType::Double, "acceleration", {}, 0.0),
                machineField("jerk", QObject::tr("加加速度"), QObject::tr("运动学"), ParameterValueType::Double, "jerk", {}, 0.0)
            };
            if (axis.axis.motionType == MachineAxisDef::Rotary)
                object.fields.append(machineField("pipeDiameter", QObject::tr("管径"), QObject::tr("旋转轴"), ParameterValueType::Double, "pipeDiameter", "mm", 0.0));
            objects.append(object);
        }
    }

    ParameterObjectDescriptor laser{QStringLiteral("laser"), QObject::tr("激光器"), QObject::tr("激光与外设")};
    auto laserType = field("type", QObject::tr("设备类型"), QObject::tr("基本"), ParameterValueType::Enum, ProcessConfigArea::Devices, "Laser", "sType", "Simulator");
    laserType.enumValues = {"Simulator", "AnalogControl", "IPG", "Raycus", "Pharos", "ULTRON"};
    laser.fields = {laserType,
        field("resolution", QObject::tr("模拟分辨率"), QObject::tr("基本"), ParameterValueType::Double, ProcessConfigArea::Devices, "Laser", "fResolution", 0.0),
        field("serialPort", QObject::tr("串口"), QObject::tr("串口"), ParameterValueType::String, ProcessConfigArea::Devices, "ComSetting", "sPort", "COM1"),
        field("baud", QObject::tr("波特率"), QObject::tr("串口"), ParameterValueType::String, ProcessConfigArea::Devices, "ComSetting", "sBaudRate", "9600")};
    objects.append(laser);

    ParameterObjectDescriptor gas{QStringLiteral("gas"), QObject::tr("气体"), QObject::tr("介质与流程")};
    gas.fields = {field("enabled", QObject::tr("启用吹气"), QObject::tr("基本"), ParameterValueType::Bool, ProcessConfigArea::Operations, "Gas", "bBlow", false),
                  field("delay", QObject::tr("吹气延时"), QObject::tr("基本"), ParameterValueType::Double, ProcessConfigArea::Operations, "Gas", "fBlowDelay", 0.0, "s", 0.0),
                  field("pressure", QObject::tr("气压"), QObject::tr("基本"), ParameterValueType::Double, ProcessConfigArea::Operations, "Gas", "fPressure", 0.0, "MPa", 0.0)};
    objects.append(gas);

    ParameterObjectDescriptor water{QStringLiteral("water"), QObject::tr("水路与回水泵"), QObject::tr("介质与流程")};
    water.fields = {field("enabled", QObject::tr("启用水路"), QObject::tr("水路"), ParameterValueType::Bool, ProcessConfigArea::Operations, "Water", "bWater", false),
                    field("delay", QObject::tr("水路延时"), QObject::tr("水路"), ParameterValueType::Double, ProcessConfigArea::Operations, "Water", "fWaterDelay", 0.0, "s", 0.0),
                    field("pump", QObject::tr("启用回水泵"), QObject::tr("回水泵"), ParameterValueType::Bool, ProcessConfigArea::Operations, "Pump", "bPump", false)};
    objects.append(water);

    // IO channels are a collection editor, not property objects.  Keeping
    // exactly four nodes avoids a long duplicated object tree and lets each
    // node use a compact table with add/remove operations.
    const auto appendIoTable = [&objects](const QString& id, const QString& title, ProcessIoBucket bucket) {
        ParameterObjectDescriptor io;
        io.id = id;
        io.title = title;
        io.category = QObject::tr("I/O 与安全");
        io.ioTableObject = true;
        io.ioBucket = bucket;
        objects.append(io);
    };
    appendIoTable(QStringLiteral("io:digital-input"), QObject::tr("数字量输入"), ProcessIoBucket::DigitalInput);
    appendIoTable(QStringLiteral("io:digital-output"), QObject::tr("数字量输出"), ProcessIoBucket::DigitalOutput);
    appendIoTable(QStringLiteral("io:analog-input"), QObject::tr("模拟量输入"), ProcessIoBucket::AnalogInput);
    appendIoTable(QStringLiteral("io:analog-output"), QObject::tr("模拟量输出"), ProcessIoBucket::AnalogOutput);

    ParameterObjectDescriptor monitor{QStringLiteral("monitor"), QObject::tr("安全监控"), QObject::tr("I/O 与安全")};
    monitor.fields = {field("interlock", QObject::tr("互锁启用"), QObject::tr("基本"), ParameterValueType::Bool, ProcessConfigArea::Operations, "Cutting", "bInterLock", true),
                      field("curtain", QObject::tr("安全光幕"), QObject::tr("基本"), ParameterValueType::Bool, ProcessConfigArea::Operations, "Cutting", "bSafetyLightCurtain", false),
                      field("pressure", QObject::tr("气压监控"), QObject::tr("气体"), ParameterValueType::Bool, ProcessConfigArea::Operations, "Gas", "bPressureMonitor", false),
                      field("waterPressure", QObject::tr("水压监控"), QObject::tr("水路"), ParameterValueType::Bool, ProcessConfigArea::Operations, "Water", "bWaterPressureMonitor", false)};
    objects.append(monitor);

    ParameterObjectDescriptor camera{QStringLiteral("camera"), QObject::tr("相机"), QObject::tr("激光与外设")};
    camera.fields = {field("host", QObject::tr("主机"), QObject::tr("连接"), ParameterValueType::String, ProcessConfigArea::Devices, "CameraConnect", "sHost", "127.0.0.1"),
                     field("port", QObject::tr("端口"), QObject::tr("连接"), ParameterValueType::Int, ProcessConfigArea::Devices, "CameraConnect", "iPort", 0, {}, 0, 65535, 0),
                     field("accuracy", QObject::tr("像素精度"), QObject::tr("标定"), ParameterValueType::Double, ProcessConfigArea::Devices, "CameraCalibration", "fPixelAccuracy", 0.0, "mm", 0.0)};
    objects.append(camera);

    ParameterObjectDescriptor positions{QStringLiteral("positions"), QObject::tr("上下料位置"), QObject::tr("运动与轴系")};
    if (machine) for (const auto& axis : machine->axisConfigurations()) {
        const QString axisName = axis.axis.name;
        positions.fields.append(field(QStringLiteral("loading.%1").arg(axisName), QObject::tr("上料 %1").arg(axisName), QObject::tr("上料"), ParameterValueType::Double, ProcessConfigArea::Operations, "LoadingPos", QStringLiteral("fLoadingPos%1").arg(axisName), 0.0));
        positions.fields.append(field(QStringLiteral("blanking.%1").arg(axisName), QObject::tr("下料 %1").arg(axisName), QObject::tr("下料"), ParameterValueType::Double, ProcessConfigArea::Operations, "BlankingPos", QStringLiteral("fBlankingPos%1").arg(axisName), 0.0));
    }
    objects.append(positions);

    for (const QString& toolName : toolNames(m_settings)) {
        ParameterObjectDescriptor tool;
        tool.id = QStringLiteral("tool:%1").arg(toolName);
        tool.title = toolName;
        tool.category = QObject::tr("工具库");
        tool.toolObject = true;
        tool.fields = {
            field("lineVelocity", QObject::tr("切割速度"), QObject::tr("切割"), ParameterValueType::Double, ProcessConfigArea::Tools, toolName, "fLineVel", 10.0, "mm/s", 0.0),
            field("cutAcceleration", QObject::tr("切割加速度"), QObject::tr("切割"), ParameterValueType::Double, ProcessConfigArea::Tools, toolName, "fCutAcc", 100.0, {}, 0.0),
            field("cutJerk", QObject::tr("切割加加速度"), QObject::tr("切割"), ParameterValueType::Double, ProcessConfigArea::Tools, toolName, "fCutJerk", 1000.0, {}, 0.0),
            field("cuttingHeight", QObject::tr("切割高度增量"), QObject::tr("高度"), ParameterValueType::Double, ProcessConfigArea::Tools, toolName, "fCuttingHeight", 0.0, "mm"),
            field("idleHeight", QObject::tr("空程高度增量"), QObject::tr("高度"), ParameterValueType::Double, ProcessConfigArea::Tools, toolName, "fIdleHeight", 0.0, "mm"),
            field("energy", QObject::tr("能量"), QObject::tr("激光"), ParameterValueType::Double, ProcessConfigArea::Tools, toolName, "fEnergy", 20.0, "%", 0.0),
            field("frequency", QObject::tr("频率"), QObject::tr("激光"), ParameterValueType::Double, ProcessConfigArea::Tools, toolName, "fFrequency", 30.0, "kHz", 0.0),
            field("pulseWidth", QObject::tr("脉宽"), QObject::tr("激光"), ParameterValueType::Double, ProcessConfigArea::Tools, toolName, "fPluse", 20.0, "μs", 0.0),
            field("beforeOpen", QObject::tr("开光前延时"), QObject::tr("激光延时"), ParameterValueType::Double, ProcessConfigArea::Tools, toolName, "fBeforeOpenLaser", 0.0, "s", 0.0),
            field("afterClose", QObject::tr("关光后延时"), QObject::tr("激光延时"), ParameterValueType::Double, ProcessConfigArea::Tools, toolName, "fAfterCloseLaser", 0.0, "s", 0.0)
        };
        if (machine) for (const auto& axis : machine->axisConfigurations()) {
            const QString name = axis.axis.name;
            tool.fields.append(field(QStringLiteral("jump.%1").arg(name), QObject::tr("%1 空程速度").arg(name), QObject::tr("Jump"),
                                     ParameterValueType::Double, ProcessConfigArea::Tools, toolName,
                                     QStringLiteral("f%1Vel").arg(name), 10.0,
                                     axis.axis.motionType == MachineAxisDef::Rotary ? "°/s" : "mm/s", 0.0));
        }
        objects.append(tool);
    }
    return objects;
}

} // namespace lcnc::process
