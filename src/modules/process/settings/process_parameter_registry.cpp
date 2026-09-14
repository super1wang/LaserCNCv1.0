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
    const bool gtnController = m_settings.rawValue(
        ProcessConfigArea::Devices, QStringLiteral("MotionControl"),
        QStringLiteral("sType"), QStringLiteral("SimulatorCMHP"))
                                   .toString()
                                   .compare(QStringLiteral("GTN"), Qt::CaseInsensitive) == 0;

    // 中文翻译：运动控制器；运动与轴系
    ParameterObjectDescriptor controller{QStringLiteral("controller"), QObject::tr("motion controller"), QObject::tr("Movement and Axis Systems")};
#if defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS
    const QString defaultController = QStringLiteral("SimulatorCMHP");
#elif defined(LCNC_PROCESS_HAS_GTN) && LCNC_PROCESS_HAS_GTN
    const QString defaultController = QStringLiteral("GTN");
#else
    const QString defaultController = QStringLiteral("Simulator");
#endif
    // 中文翻译：控制器类型；连接
    auto controllerType = field("type", QObject::tr("Controller type"), QObject::tr("connect"), ParameterValueType::Enum,
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

    if (gtnController) {
        // 中文翻译：GTN 五轴组；运动与轴系。
        ParameterObjectDescriptor group{
            QStringLiteral("gtn-five-axis-group"), QObject::tr("GTN five-axis group"),
            QObject::tr("Movement and Axis Systems")};
        group.parentObjectId = QStringLiteral("controller");
        group.fields = {
            // 中文翻译：启用新架构 Group/CommandList；模式。
            field("enabled", QObject::tr("Enable Group/CommandList architecture"),
                  QObject::tr("Mode"), ParameterValueType::Bool,
                  ProcessConfigArea::Devices, "GTN", "bUseGroupArchitecture", false),
            // 中文翻译：启用控制器 RTCP；模式。
            field("rtcp", QObject::tr("Enable controller RTCP"), QObject::tr("Mode"),
                  ParameterValueType::Bool, ProcessConfigArea::Devices, "GTN",
                  "bEnableRtcp", false),
            // 中文翻译：允许使用构型派生参数进行 RTCP 加工；调试安全。
            field("configurationDerivedTrial",
                  QObject::tr("Allow configuration-derived RTCP machining"),
                  QObject::tr("Commissioning safety"), ParameterValueType::Bool,
                  ProcessConfigArea::Devices, "GTN",
                  "bAllowConfigurationDerivedRtcp", false),
            // 中文翻译：Group 编号；资源。
            field("group", QObject::tr("Group index"), QObject::tr("Resources"),
                  ParameterValueType::Int, ProcessConfigArea::Devices, "GTN",
                  "iFiveAxisGroup", 1, {}, 1, 2, 0),
            // 中文翻译：CommandList 编号；资源。
            field("list", QObject::tr("Command list index"), QObject::tr("Resources"),
                  ParameterValueType::Int, ProcessConfigArea::Devices, "GTN",
                  "iFiveAxisCommandList", 1, {}, 1, 4, 0),
            // 中文翻译：Group 平滑时间；轨迹规划。
            field("smoothTime", QObject::tr("Group smooth time"),
                  QObject::tr("Trajectory planning"), ParameterValueType::Double,
                  ProcessConfigArea::Devices, "GTN", "fGroupSmoothTime", 20.0,
                  "ms", 0.0, 60.0),
            // 中文翻译：Group 平滑系数；轨迹规划。
            field("smoothK", QObject::tr("Group smooth coefficient"),
                  QObject::tr("Trajectory planning"), ParameterValueType::Double,
                  ProcessConfigArea::Devices, "GTN", "fGroupSmoothK", 15.0,
                  {}, 0.0, 1000.0, 3),
            // 中文翻译：前瞻段数；轨迹规划。
            field("lookAheadNum", QObject::tr("Look-ahead segment count"),
                  QObject::tr("Trajectory planning"), ParameterValueType::Int,
                  ProcessConfigArea::Devices, "GTN", "iGroupLookAheadNum", 200,
                  {}, 1, 10000, 0),
            // 中文翻译：前瞻时间常数；轨迹规划。
            field("lookAheadTime", QObject::tr("Look-ahead time constant"),
                  QObject::tr("Trajectory planning"), ParameterValueType::Double,
                  ProcessConfigArea::Devices, "GTN", "fGroupLookAheadTime", 0.01,
                  {}, 0.000001, 1000.0, 6),
            // 中文翻译：前瞻曲率系数；轨迹规划。
            field("lookAheadRadius", QObject::tr("Look-ahead radius ratio"),
                  QObject::tr("Trajectory planning"), ParameterValueType::Double,
                  ProcessConfigArea::Devices, "GTN", "fGroupLookAheadRadiusRatio", 0.1,
                  {}, 0.0, 1000.0, 6),
            // 中文翻译：第一旋转轴速度参考比例；轨迹规划。
            field("primaryRotaryRatio", QObject::tr("Primary rotary velocity reference ratio"),
                  QObject::tr("Trajectory planning"), ParameterValueType::Double,
                  ProcessConfigArea::Devices, "GTN", "fGroupPrimaryRotaryVelRefRatio", 20.0,
                  {}, 0.000001, 1000000.0, 6),
            // 中文翻译：第二旋转轴速度参考比例；轨迹规划。
            field("slaveRotaryRatio", QObject::tr("Slave rotary velocity reference ratio"),
                  QObject::tr("Trajectory planning"), ParameterValueType::Double,
                  ProcessConfigArea::Devices, "GTN", "fGroupSlaveRotaryVelRefRatio", 20.0,
                  {}, 0.000001, 1000000.0, 6),
            // 中文翻译：RTCP 轴坐标一致性容差；RTCP 验证。
            field("rtcpAgreement", QObject::tr("RTCP axis agreement tolerance"),
                  QObject::tr("RTCP verification"), ParameterValueType::Double,
                  ProcessConfigArea::Devices, "GTN", "fGroupRtcpAxisAgreementTolerance", 0.05,
                  {}, 0.000001, 1000.0, 6),
            // 中文翻译：RTCP 转换抽检步长；RTCP 验证。
            field("rtcpValidationStride", QObject::tr("RTCP transform validation stride"),
                  QObject::tr("RTCP verification"), ParameterValueType::Int,
                  ProcessConfigArea::Devices, "GTN", "iGroupRtcpValidationStride", 100,
                  {}, 1, 1000000, 0)
        };
        objects.append(group);
    }

    lcnc::Kernel* kernel = lcnc::Kernel::tryCurrent();
    auto machine = kernel
        ? kernel->services().getService<lcnc::MachineConfigurationService>()
        : nullptr;

    // Ribbon homing is controller setup, not a Process workflow node. Keep it
    // as a child page below the selectable motion-controller root.
    // 中文翻译：回零配置；运动与轴系
    ParameterObjectDescriptor homing{QStringLiteral("homing"), QObject::tr("Homing settings"),
                                     QObject::tr("Movement and Axis Systems")};
    homing.parentObjectId = QStringLiteral("controller");
    if (machine) {
        QStringList defaultOrder{QStringLiteral("Z")};
        for (const auto& axis : machine->axisConfigurations()) {
            const QString axisName = axis.axis.name.trimmed().toUpper();
            if (!axisName.isEmpty() && axisName != QStringLiteral("BASE")
                && !defaultOrder.contains(axisName)) {
                defaultOrder.append(axisName);
            }
        }
        const QStringList supportedAxes{QStringLiteral("X"), QStringLiteral("Y"),
                                        QStringLiteral("Z"), QStringLiteral("A"),
                                        QStringLiteral("B"), QStringLiteral("C")};
        for (const auto& axis : machine->axisConfigurations()) {
            const QString axisName = axis.axis.name.trimmed().toUpper();
            if (!supportedAxes.contains(axisName))
                continue;
            // 中文翻译：%1 轴
            const QString group = QObject::tr("%1 axis").arg(axisName);
            auto method = field(
                QStringLiteral("method.%1").arg(axisName), QObject::tr("Homing method"), group,
                ParameterValueType::Enum, ProcessConfigArea::Devices, QStringLiteral("Homing"),
                QStringLiteral("sMethod%1").arg(axisName), QStringLiteral("ControllerHome"));
            method.enumValues = {QStringLiteral("Disabled"),
                                 QStringLiteral("ControllerHome"),
                                 QStringLiteral("SetCurrentPosition")};
            // 中文翻译：关闭（不执行）；控制器回零（运动）；当前位置置位（不运动）
            method.enumLabels = {QObject::tr("Disabled (no action)"),
                                 QObject::tr("Controller homing (motion)"),
                                 QObject::tr("Set current position (no motion)")};
            homing.fields.append(method);
            // 中文翻译：回零顺序
            homing.fields.append(field(
                QStringLiteral("order.%1").arg(axisName), QObject::tr("Homing order"), group,
                ParameterValueType::Int, ProcessConfigArea::Devices, QStringLiteral("Homing"),
                QStringLiteral("iOrder%1").arg(axisName), defaultOrder.indexOf(axisName) + 1,
                {}, 1, 128, 0));
            // 中文翻译：置位坐标
            homing.fields.append(field(
                QStringLiteral("position.%1").arg(axisName), QObject::tr("Set coordinate"), group,
                ParameterValueType::Double, ProcessConfigArea::Devices, QStringLiteral("Homing"),
                QStringLiteral("fPosition%1").arg(axisName), 0.0,
                axis.axis.motionType == MachineAxisDef::Rotary ? QStringLiteral("°")
                                                                : QStringLiteral("mm"),
                -1000000.0, 1000000.0, 6));
        }
    }
    objects.append(homing);

    if (machine) {
        for (const auto& axis : machine->axisConfigurations()) {
            ParameterObjectDescriptor object;
            object.id = QStringLiteral("axis:%1").arg(axis.axis.name);
            object.title = axis.axis.name;
            // 中文翻译：运动与轴系
            object.category = QObject::tr("Movement and Axis Systems");
            object.fields = {
                // 中文翻译：控制器索引；基本
                machineField("controllerIndex", QObject::tr("Controller index"), QObject::tr("Basic"), ParameterValueType::Int, "controllerIndex", {}, 0, 128, 0),
                // 中文翻译：回零索引；基本
                machineField("homeIndex", QObject::tr("Zero index"), QObject::tr("Basic"), ParameterValueType::Int, "homeIndex", {}, 0, 128, 0),
                // 中文翻译：分辨率；基本
                machineField("resolution", QObject::tr("resolution"), QObject::tr("Basic"), ParameterValueType::Double, "resolution", {}, 0.000001),
                // 中文翻译：运动速度；基本
                machineField("motionSpeed", QObject::tr("Movement speed"), QObject::tr("Basic"), ParameterValueType::Double, "motionSpeed", axis.axis.motionType == MachineAxisDef::Rotary ? "°/s" : "mm/s", 0.0),
                // 中文翻译：负限位；限位
                machineField("min", QObject::tr("Negative limit"), QObject::tr("Limit"), ParameterValueType::Double, "min", axis.axis.motionType == MachineAxisDef::Rotary ? "°" : "mm", -1e12, 1e12),
                // 中文翻译：正限位；限位
                machineField("max", QObject::tr("Positive limit"), QObject::tr("Limit"), ParameterValueType::Double, "max", axis.axis.motionType == MachineAxisDef::Rotary ? "°" : "mm", -1e12, 1e12),
                // 中文翻译：低速；手动速度
                machineField("lowSpeed", QObject::tr("low speed"), QObject::tr("manual speed"), ParameterValueType::Double, "lowSpeed", axis.axis.motionType == MachineAxisDef::Rotary ? "°/s" : "mm/s", 0.0),
                // 中文翻译：中速；手动速度
                machineField("mediumSpeed", QObject::tr("medium speed"), QObject::tr("manual speed"), ParameterValueType::Double, "mediumSpeed", axis.axis.motionType == MachineAxisDef::Rotary ? "°/s" : "mm/s", 0.0),
                // 中文翻译：高速；手动速度
                machineField("highSpeed", QObject::tr("high speed"), QObject::tr("manual speed"), ParameterValueType::Double, "highSpeed", axis.axis.motionType == MachineAxisDef::Rotary ? "°/s" : "mm/s", 0.0),
                // 中文翻译：加速度；运动学
                machineField("acceleration", QObject::tr("acceleration"), QObject::tr("Kinesiology"), ParameterValueType::Double, "acceleration", {}, 0.0)
            };
            if (gtnController) {
                const QString suffix = axis.axis.name.trimmed().toUpper();
                // 中文翻译：点位平滑时间；运动学
                object.fields.append(field(
                    QStringLiteral("trapSmoothTime"), QObject::tr("Point motion smooth time"),
                    QObject::tr("Kinesiology"), ParameterValueType::Int,
                    ProcessConfigArea::Devices, QStringLiteral("GTN"),
                    QStringLiteral("iTrapSmoothTime%1").arg(suffix), 10,
                    QStringLiteral("ms"), 0.0, 50.0, 0));
                // 中文翻译：Jog 平滑系数；运动学
                object.fields.append(field(
                    QStringLiteral("jogSmooth"), QObject::tr("Jog smooth coefficient"),
                    QObject::tr("Kinesiology"), ParameterValueType::Double,
                    ProcessConfigArea::Devices, QStringLiteral("GTN"),
                    QStringLiteral("fJogSmooth%1").arg(suffix), 0.5,
                    {}, 0.0, 0.999, 3));
            } else {
                // 中文翻译：加加速度；运动学
                object.fields.append(machineField(
                    "jerk", QObject::tr("Jerk"), QObject::tr("Kinesiology"),
                    ParameterValueType::Double, "jerk", {}, 0.0));
            }
            if (axis.axis.motionType == MachineAxisDef::Rotary)
                // 中文翻译：管径；旋转轴
                object.fields.append(machineField("pipeDiameter", QObject::tr("Pipe diameter"), QObject::tr("axis of rotation"), ParameterValueType::Double, "pipeDiameter", "mm", 0.0));
            objects.append(object);
        }
    }

    // 中文翻译：激光器；激光与外设
    ParameterObjectDescriptor laser{QStringLiteral("laser"), QObject::tr("laser"), QObject::tr("Lasers and Peripherals")};
    // 中文翻译：设备类型；基本
    auto laserType = field("type", QObject::tr("Device type"), QObject::tr("Basic"), ParameterValueType::Enum, ProcessConfigArea::Devices, "Laser", "sType", "Simulator");
    laserType.enumValues = {"Simulator", "AnalogControl", "IPG", "Raycus", "Pharos", "ULTRON"};
    laser.fields = {laserType,
        // 中文翻译：模拟分辨率；基本
        field("resolution", QObject::tr("Analog resolution"), QObject::tr("Basic"), ParameterValueType::Double, ProcessConfigArea::Devices, "Laser", "fResolution", 0.0),
        // 中文翻译：串口；串口
        field("serialPort", QObject::tr("serial port"), QObject::tr("serial port"), ParameterValueType::String, ProcessConfigArea::Devices, "ComSetting", "sPort", "COM1"),
        // 中文翻译：波特率；串口
        field("baud", QObject::tr("baud rate"), QObject::tr("serial port"), ParameterValueType::String, ProcessConfigArea::Devices, "ComSetting", "sBaudRate", "9600")};
    objects.append(laser);

    // 中文翻译：气体；介质与流程
    ParameterObjectDescriptor gas{QStringLiteral("gas"), QObject::tr("gas"), QObject::tr("Medium and process")};
    // 中文翻译：启用吹气；基本
    gas.fields = {field("enabled", QObject::tr("Enable blowing"), QObject::tr("Basic"), ParameterValueType::Bool, ProcessConfigArea::Operations, "Gas", "bBlow", false),
                  // 中文翻译：吹气延时；基本
                  field("delay", QObject::tr("blow delay"), QObject::tr("Basic"), ParameterValueType::Double, ProcessConfigArea::Operations, "Gas", "fBlowDelay", 0.0, "s", 0.0),
                  // 中文翻译：气压；基本
                  field("pressure", QObject::tr("air pressure"), QObject::tr("Basic"), ParameterValueType::Double, ProcessConfigArea::Operations, "Gas", "fPressure", 0.0, "MPa", 0.0)};
    objects.append(gas);

    // 中文翻译：水路与回水泵；介质与流程
    ParameterObjectDescriptor water{QStringLiteral("water"), QObject::tr("Waterway and return pump"), QObject::tr("Medium and process")};
    // 中文翻译：启用水路；水路
    water.fields = {field("enabled", QObject::tr("Activate waterways"), QObject::tr("waterway"), ParameterValueType::Bool, ProcessConfigArea::Operations, "Water", "bWater", false),
                    // 中文翻译：水路延时；水路
                    field("delay", QObject::tr("waterway delay"), QObject::tr("waterway"), ParameterValueType::Double, ProcessConfigArea::Operations, "Water", "fWaterDelay", 0.0, "s", 0.0),
                    // 中文翻译：启用回水泵；回水泵
                    field("pump", QObject::tr("Enable return water pump"), QObject::tr("Return water pump"), ParameterValueType::Bool, ProcessConfigArea::Operations, "Pump", "bPump", false)};
    objects.append(water);

    // Initial motion is a Process policy but CAM remains the sole owner of
    // automatic five-axis planning and collision validation.
    // 中文翻译：首段设置；加工起始运动。
    ParameterObjectDescriptor initialApproach{
        QStringLiteral("initial-approach"), QObject::tr("Initial segment settings"),
        QObject::tr("Medium and process")};
    // 中文翻译：规划模式；首段。
    auto initialMode = field(
        QStringLiteral("mode"), QObject::tr("Planning mode"), QObject::tr("Initial segment"),
        ParameterValueType::Enum, ProcessConfigArea::Workflow,
        QStringLiteral("InitialApproach"), QStringLiteral("sMode"),
        QStringLiteral("Automatic"));
    initialMode.enumValues = {QStringLiteral("Manual"), QStringLiteral("Automatic")};
    // 中文翻译：手动规划；自动规划。
    initialMode.enumLabels = {QObject::tr("Manual planning"), QObject::tr("Automatic planning")};
    initialApproach.fields = {
        initialMode,
        // 中文翻译：Z 安全坐标（控制器绝对坐标）；首段。
        field(QStringLiteral("safetyZ"), QObject::tr("Z safety coordinate (absolute motion)"),
              QObject::tr("Initial segment"), ParameterValueType::Double,
              ProcessConfigArea::Workflow, QStringLiteral("InitialApproach"),
              QStringLiteral("fSafetyZ"), 0.0, QStringLiteral("mm"),
              -1000000.0, 1000000.0, 3)
    };
    objects.append(initialApproach);

    // IO channels are a collection editor, not property objects.  Keeping
    // exactly four nodes avoids a long duplicated object tree and lets each
    // node use a compact table with add/remove operations.
    const auto appendIoTable = [&objects](const QString& id, const QString& title, ProcessIoBucket bucket) {
        ParameterObjectDescriptor io;
        io.id = id;
        io.title = title;
        // 中文翻译：I/O 与安全
        io.category = QObject::tr("I/O and security");
        io.ioTableObject = true;
        io.ioBucket = bucket;
        objects.append(io);
    };
    // 中文翻译：数字量输入
    appendIoTable(QStringLiteral("io:digital-input"), QObject::tr("digital input"), ProcessIoBucket::DigitalInput);
    // 中文翻译：数字量输出
    appendIoTable(QStringLiteral("io:digital-output"), QObject::tr("Digital output"), ProcessIoBucket::DigitalOutput);
    // 中文翻译：模拟量输入
    appendIoTable(QStringLiteral("io:analog-input"), QObject::tr("Analog input"), ProcessIoBucket::AnalogInput);
    // 中文翻译：模拟量输出
    appendIoTable(QStringLiteral("io:analog-output"), QObject::tr("Analog output"), ProcessIoBucket::AnalogOutput);

    // 中文翻译：安全监控；I/O 与安全
    ParameterObjectDescriptor monitor{QStringLiteral("monitor"), QObject::tr("security monitoring"), QObject::tr("I/O and security")};
    // 中文翻译：互锁启用；基本
    monitor.fields = {field("interlock", QObject::tr("Interlock enabled"), QObject::tr("Basic"), ParameterValueType::Bool, ProcessConfigArea::Operations, "Cutting", "bInterLock", true),
                      // 中文翻译：安全光幕；基本
                      field("curtain", QObject::tr("safety light curtain"), QObject::tr("Basic"), ParameterValueType::Bool, ProcessConfigArea::Operations, "Cutting", "bSafetyLightCurtain", false),
                      // 中文翻译：气压监控；气体
                      field("pressure", QObject::tr("Air pressure monitoring"), QObject::tr("gas"), ParameterValueType::Bool, ProcessConfigArea::Operations, "Gas", "bPressureMonitor", false),
                      // 中文翻译：水压监控；水路
                      field("waterPressure", QObject::tr("water pressure monitoring"), QObject::tr("waterway"), ParameterValueType::Bool, ProcessConfigArea::Operations, "Water", "bWaterPressureMonitor", false)};
    objects.append(monitor);

    // 中文翻译：相机；激光与外设
    ParameterObjectDescriptor camera{QStringLiteral("camera"), QObject::tr("camera"), QObject::tr("Lasers and Peripherals")};
    // 中文翻译：主机；连接
    camera.fields = {field("host", QObject::tr("Host"), QObject::tr("connect"), ParameterValueType::String, ProcessConfigArea::Devices, "CameraConnect", "sHost", "127.0.0.1"),
                     // 中文翻译：端口；连接
                     field("port", QObject::tr("port"), QObject::tr("connect"), ParameterValueType::Int, ProcessConfigArea::Devices, "CameraConnect", "iPort", 0, {}, 0, 65535, 0),
                     // 中文翻译：像素精度；标定
                     field("accuracy", QObject::tr("Pixel accuracy"), QObject::tr("Calibration"), ParameterValueType::Double, ProcessConfigArea::Devices, "CameraCalibration", "fPixelAccuracy", 0.0, "mm", 0.0)};
    objects.append(camera);

    // 中文翻译：上下料位置；运动与轴系
    ParameterObjectDescriptor positions{QStringLiteral("positions"), QObject::tr("Loading and unloading position"), QObject::tr("Movement and Axis Systems")};
    if (machine) for (const auto& axis : machine->axisConfigurations()) {
        const QString axisName = axis.axis.name;
        // 中文翻译：启用上料 %1；上料
        positions.fields.append(field(QStringLiteral("loading.enabled.%1").arg(axisName), QObject::tr("Enable loading %1").arg(axisName), QObject::tr("Feeding"), ParameterValueType::Bool, ProcessConfigArea::Operations, "LoadingPos", QStringLiteral("bLoadingPos%1").arg(axisName), false));
        // 中文翻译：上料 %1；上料
        positions.fields.append(field(QStringLiteral("loading.%1").arg(axisName), QObject::tr("Loading %1").arg(axisName), QObject::tr("Feeding"), ParameterValueType::Double, ProcessConfigArea::Operations, "LoadingPos", QStringLiteral("fLoadingPos%1").arg(axisName), 0.0));
        // 中文翻译：启用下料 %1；下料
        positions.fields.append(field(QStringLiteral("blanking.enabled.%1").arg(axisName), QObject::tr("Enable blanking %1").arg(axisName), QObject::tr("blanking"), ParameterValueType::Bool, ProcessConfigArea::Operations, "BlankingPos", QStringLiteral("bBlankingPos%1").arg(axisName), false));
        // 中文翻译：下料 %1；下料
        positions.fields.append(field(QStringLiteral("blanking.%1").arg(axisName), QObject::tr("Blanking %1").arg(axisName), QObject::tr("blanking"), ParameterValueType::Double, ProcessConfigArea::Operations, "BlankingPos", QStringLiteral("fBlankingPos%1").arg(axisName), 0.0));
    }
    objects.append(positions);

    for (const QString& toolName : toolNames(m_settings)) {
        ParameterObjectDescriptor tool;
        tool.id = QStringLiteral("tool:%1").arg(toolName);
        tool.title = toolName;
        // 中文翻译：工具库
        tool.category = QObject::tr("Tool library");
        tool.toolObject = true;
        tool.fields = {
            // 中文翻译：切割速度；切割
            field("lineVelocity", QObject::tr("cutting speed"), QObject::tr("cutting"), ParameterValueType::Double, ProcessConfigArea::Tools, toolName, "fLineVel", 10.0, "mm/s", 0.0),
            // 中文翻译：切割加速度；切割
            field("cutAcceleration", QObject::tr("Cutting acceleration"), QObject::tr("cutting"), ParameterValueType::Double, ProcessConfigArea::Tools, toolName, "fCutAcc", 100.0, {}, 0.0),
            // 中文翻译：切割高度增量；高度
            // 中文翻译：空程高度增量；高度
            // 中文翻译：空程加速度
            field("jumpAcceleration", QObject::tr("Idle acceleration"), QObject::tr("Jump"), ParameterValueType::Double, ProcessConfigArea::Tools, toolName, "fIdelAcc", 100.0, {}, 0.0),
            // 中文翻译：能量；激光
            field("energy", QObject::tr("energy"), QObject::tr("laser"), ParameterValueType::Double, ProcessConfigArea::Tools, toolName, "fEnergy", 20.0, "%", 0.0),
            // 中文翻译：频率；激光
            field("frequency", QObject::tr("Frequency"), QObject::tr("laser"), ParameterValueType::Double, ProcessConfigArea::Tools, toolName, "fFrequency", 30.0, "kHz", 0.0),
            // 中文翻译：脉宽；激光
            field("pulseWidth", QObject::tr("pulse width"), QObject::tr("laser"), ParameterValueType::Double, ProcessConfigArea::Tools, toolName, "fPluse", 20.0, "μs", 0.0),
            // 中文翻译：开光前延时；激光延时
            field("beforeOpen", QObject::tr("Delay before lighting"), QObject::tr("Laser delay"), ParameterValueType::Double, ProcessConfigArea::Tools, toolName, "fBeforeOpenLaser", 0.0, "s", 0.0),
            // 中文翻译：关光后延时；激光延时
            field("afterClose", QObject::tr("Delay after light off"), QObject::tr("Laser delay"), ParameterValueType::Double, ProcessConfigArea::Tools, toolName, "fAfterCloseLaser", 0.0, "s", 0.0)
        };
        if (gtnController) {
            // 中文翻译：插补平滑时间；切割
            tool.fields.append(field(
                "cutSmoothTime", QObject::tr("Interpolation smooth time"),
                QObject::tr("cutting"), ParameterValueType::Double,
                ProcessConfigArea::Tools, toolName, "fCutSmoothTime", 0.0,
                "ms", 0.0, 120.0));
            // 中文翻译：插补平滑系数；切割
            tool.fields.append(field(
                "cutSmoothCoefficient", QObject::tr("Interpolation smooth coefficient"),
                QObject::tr("cutting"), ParameterValueType::Double,
                ProcessConfigArea::Tools, toolName, "fCutSmoothK", 0.0,
                {}, 0.0, 99.999));
            // 中文翻译：轴平滑时间；切割
            tool.fields.append(field(
                "axisSmoothTime", QObject::tr("Axis smooth time"),
                QObject::tr("cutting"), ParameterValueType::Double,
                ProcessConfigArea::Tools, toolName, "fAxisSmoothTime", 0.0,
                "ms", 0.0, 60.0));
            // 中文翻译：轴平滑系数；切割
            tool.fields.append(field(
                "axisSmoothCoefficient", QObject::tr("Axis smooth coefficient"),
                QObject::tr("cutting"), ParameterValueType::Double,
                ProcessConfigArea::Tools, toolName, "fAxisSmoothK", 0.0,
                {}, 0.0, 99.999));
        } else {
            // 中文翻译：切割加加速度；切割
            tool.fields.append(field(
                "cutJerk", QObject::tr("Cutting jerk"), QObject::tr("cutting"),
                ParameterValueType::Double, ProcessConfigArea::Tools, toolName,
                "fCutJerk", 1000.0, {}, 0.0));
            // 中文翻译：空程加加速度；空程
            tool.fields.append(field(
                "jumpJerk", QObject::tr("Idle jerk"), QObject::tr("Jump"),
                ParameterValueType::Double, ProcessConfigArea::Tools, toolName,
                "fIdelJerk", 1000.0, {}, 0.0));
        }
        if (machine) for (const auto& axis : machine->axisConfigurations()) {
            const QString name = axis.axis.name;
            // 中文翻译：%1 空程速度
            tool.fields.append(field(QStringLiteral("jump.%1").arg(name), QObject::tr("%1 idle speed").arg(name), QObject::tr("Jump"),
                                     ParameterValueType::Double, ProcessConfigArea::Tools, toolName,
                                     QStringLiteral("f%1Vel").arg(name), 10.0,
                                     axis.axis.motionType == MachineAxisDef::Rotary ? "°/s" : "mm/s", 0.0));
        }
        objects.append(tool);
    }
    return objects;
}

} // namespace lcnc::process
