#include "modules/process/runtime/process_device_runtime.h"

#include "modules/process/device/process_io_types.h"
#include "modules/process/runtime/process_axis_types.h"
#include "modules/process/system/regex_patterns.h"
#include "core/logging/logger.h"
#include "modules/process/settings/process_settings_service.h"
#include "modules/process/runtime/process_runtime_configuration.h"
#include "modules/process/runtime/i_motion_command_sink.h"
#include "modules/process/runtime/process_cutting_safety.h"
#include "modules/process/device/motion_control/simulate_cmhp_motion_control.h"

#include "magic_enum.hpp"
#if defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS
#include "modules/process/device/motion_control/acs_motion_control.h"
#endif
#if defined(LCNC_PROCESS_HAS_GTN) && LCNC_PROCESS_HAS_GTN
#include "modules/process/device/motion_control/gtn_motion_control.h"
#endif

#include <array>
#include <cmath>
#include <set>

#include <QElapsedTimer>
#include <QThread>

using lcnc::process::AnalogIN;
using lcnc::process::AnalogOUT;
using lcnc::process::Axis;
using lcnc::process::DigitalIN;
using lcnc::process::DigitalOUT;
using std::string;
using std::vector;
using toml::table;

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::moveRelative(
    Axis axis, double distance, double velocity)
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected())
        // 中文翻译：未连接控制器，请先连接设备
        return {false, QObject::tr("The controller is not connected, please connect the device first")};
    if (!m_motionControl->IsMotorCreated(axis))
        // 中文翻译：轴未注册
        return {false, QObject::tr("Axis not registered")};
    const bool ok = m_motionControl->MoveRelative(axis, distance, velocity);
    // 中文翻译：相对运动命令失败
    return {ok, ok ? QString() : QObject::tr("Relative motion command failed")};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::moveAbsolute(
    Axis axis, double position, double velocity)
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected())
        // 中文翻译：未连接控制器，请先连接设备
        return {false, QObject::tr("The controller is not connected, please connect the device first")};
    if (!m_motionControl->IsMotorCreated(axis))
        // 中文翻译：轴未注册
        return {false, QObject::tr("Axis not registered")};
    const bool ok = m_motionControl->MoveAbsolute(axis, position, velocity);
    // 中文翻译：绝对运动命令失败
    return {ok, ok ? QString() : QObject::tr("Absolute motion command failed")};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::jog(
    Axis axis, bool positive, double velocity)
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected())
        // 中文翻译：未连接控制器，请先连接设备
        return {false, QObject::tr("The controller is not connected, please connect the device first")};
    if (!m_motionControl->IsMotorCreated(axis))
        // 中文翻译：轴未注册
        return {false, QObject::tr("Axis not registered")};
    const bool ok = m_motionControl->Jog(axis, positive, velocity);
    // 中文翻译：连续运动命令失败
    return {ok, ok ? QString() : QObject::tr("Continuous motion command failed")};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::stopAxis(Axis axis)
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected()
        || !m_motionControl->IsMotorCreated(axis)) {
        // 中文翻译：轴停止条件不满足
        return {false, QObject::tr("Axis stop conditions are not met")};
    }
    const bool ok = m_motionControl->StopMotion(axis);
    // 中文翻译：轴停止命令失败
    return {ok, ok ? QString() : QObject::tr("Axis stop command failed")};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::stopAllMotion()
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected())
        // 中文翻译：运动控制器未连接
        return {false, QObject::tr("Motion controller not connected")};
    const bool ok = m_motionControl->StopMotion() && m_motionControl->StopAllBuffer();
    // 中文翻译：停止运动失败
    return {ok, ok ? QString() : QObject::tr("Stop motion failed")};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::moveAxes(
    const QVector<Axis>& axes, const QVector<double>& positions, double velocity, bool relative)
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected())
        // 中文翻译：运动控制器未连接
        return {false, QObject::tr("Motion controller not connected")};
    if (QString::fromStdString(m_motionControl->GetName()) == QStringLiteral("GTN"))
        // 中文翻译：GTN 控制器暂不支持同步多轴运动，请改为顺序执行
        return {false, QObject::tr("The GTN controller does not currently support synchronous multi-axis motion. Please execute it sequentially instead.")};
    if (axes.size() != positions.size() || axes.isEmpty())
        // 中文翻译：同步多轴运动失败
        return {false, QObject::tr("Synchronized multi-axis motion failed")};
    vector<Axis> nativeAxes;
    vector<double> nativePositions;
    nativeAxes.reserve(static_cast<std::size_t>(axes.size()));
    nativePositions.reserve(static_cast<std::size_t>(positions.size()));
    for (int index = 0; index < axes.size(); ++index) {
        if (!m_motionControl->IsMotorCreated(axes.at(index)))
            // 中文翻译：轴未注册
            return {false, QObject::tr("Axis not registered")};
        nativeAxes.push_back(axes.at(index));
        nativePositions.push_back(positions.at(index));
    }
    const bool ok = relative
        ? m_motionControl->MoveMRelative(nativeAxes, nativePositions, velocity)
        : m_motionControl->MoveMAbsolute(nativeAxes, nativePositions, velocity);
    // 中文翻译：同步多轴运动失败
    return {ok, ok ? QString() : QObject::tr("Synchronized multi-axis motion failed")};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::setAxisPositions(
    const QVector<QPair<Axis, double>>& targets)
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected())
        // 中文翻译：运动控制器未连接
        return {false, QObject::tr("Motion controller not connected")};
    if (targets.isEmpty())
        // 中文翻译：置位轴表为空
        return {false, QObject::tr("Axis position table is empty")};

    std::set<Axis> validatedAxes;
    for (const auto& target : targets) {
        if (!m_motionControl->IsMotorCreated(target.first))
            // 中文翻译：轴未注册
            return {false, QObject::tr("Axis not registered")};
        if (!std::isfinite(target.second))
            // 中文翻译：轴置位坐标必须是有限数值
            return {false, QObject::tr("Axis position must be a finite value")};
        if (!validatedAxes.insert(target.first).second)
            // 中文翻译：置位轴表包含重复轴
            return {false, QObject::tr("Axis position table contains duplicate axes")};
    }

    // 在写入任何控制器寄存器前先完成整表校验，避免后续行非法时留下半完成状态。
    for (const auto& target : targets) {
        if (!m_motionControl->SetFPosition(target.first, target.second))
            // 中文翻译：轴置位失败
            return {false, QObject::tr("Axis position setting failed")};
    }
    return {true, QString()};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::setAxisEnabled(Axis axis, bool enabled)
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected())
        // 中文翻译：运动控制器未连接
        return {false, QObject::tr("Motion controller not connected")};
    if (!m_motionControl->IsMotorCreated(axis))
        // 中文翻译：轴未注册
        return {false, QObject::tr("Axis not registered")};
    const bool ok = m_motionControl->SetAxisEnable(axis, enabled);
    // 中文翻译：轴使能切换失败
    return {ok, ok ? QString() : QObject::tr("Axis enable switching failed")};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::setDigitalOutput(
    DigitalOUT output, bool value)
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected())
        // 中文翻译：运动控制器未连接
        return {false, QObject::tr("Motion controller not connected")};
    if (!m_motionControl->m_mapDigitalOUT.count(output))
        // 中文翻译：IO 输出切换失败
        return {false, QObject::tr("IO output switching failed")};
    const bool ok = m_motionControl->DigitalOutputSet(output, value ? 1 : 0);
    // 中文翻译：IO 输出切换失败
    return {ok, ok ? QString() : QObject::tr("IO output switching failed")};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::setDigitalOutput(
    const QString& outputName, bool value)
{
    QString enumName = outputName.trimmed();
    if (enumName.startsWith(QLatin1Char('a')) && enumName.size() >= 2)
        enumName = enumName.mid(1);
    const auto output = magic_enum::enum_cast<DigitalOUT>(enumName.toStdString());
    if (!output.has_value())
        // 中文翻译：IO 输出切换失败
        return {false, QObject::tr("IO output switching failed")};
    return setDigitalOutput(output.value(), value);
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::setAnalogOutput(
    AnalogOUT output, double value, const QString& outputName)
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected())
        // 中文翻译：运动控制器未连接
        return {false, QObject::tr("Motion controller not connected")};
    if (!m_motionControl->m_mapAnalogOUT.count(output))
        // 中文翻译：模拟量输出 %1 未注册
        return {false, QObject::tr("Analog output %1 is not registered").arg(outputName)};
    const bool ok = m_motionControl->AnalogOutputSet(output, value);
    // 中文翻译：输出信号 %1 设置失败
    return {ok, ok ? QString() : QObject::tr("Output signal %1 setup failed").arg(outputName)};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::homeAxes(
    const QStringList& axes, bool connected)
{
    const auto lock = lockDeviceAccess();
    MotionControl* const motionControl = connected ? m_motionControl.get() : nullptr;
    for (const QString& axisName : axes) {
        if (!motionControl)
            continue;

        const auto axis = magic_enum::enum_cast<Axis>(axisName.toStdString());
        if (!axis.has_value() || !motionControl->IsMotorCreated(axis.value())) {
            LCNC_WARN(lcnc::LogCode::Generic,
                      "ProcessDeviceRuntime::homeAxes: skip hardware home for axis {} "
                      "(extension or not registered)", axisName.toStdString());
            continue;
        }
        if (!motionControl->Home(axis.value())) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "ProcessDeviceRuntime::homeAxes: hardware Home failed for axis {}",
                     axisName.toStdString());
            // 中文翻译：回零失败，请检查日志
            return {false, QObject::tr("Return to zero failed, please check the log")};
        }
    }
    return {};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::moveToPreset(
    const QMap<QString, double>& targets, double velocity, const QString& positionName)
{
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected())
        // 中文翻译：未连接控制器，请先连接设备
        return {false, QObject::tr("The controller is not connected, please connect the device first")};

    for (auto it = targets.cbegin(); it != targets.cend(); ++it) {
        const auto axis = magic_enum::enum_cast<Axis>(it.key().toStdString());
        if (!axis.has_value() || !m_motionControl->IsMotorCreated(axis.value()))
            // 中文翻译：%1 轴未在控制器中创建
            return {false, QObject::tr("%1 axis was not created in the controller").arg(it.key())};
        if (!m_motionControl->IsEnabled(axis.value()))
            // 中文翻译：%1 轴当前未使能
            return {false, QObject::tr("%1 axis is not currently enabled").arg(it.key())};
        if (!m_motionControl->IsHomed(axis.value()))
            // 中文翻译：%1 轴尚未回零
            return {false, QObject::tr("%1 axis has not returned to zero yet").arg(it.key())};
        if (m_motionControl->IsAxisMoving(axis.value()))
            // 中文翻译：%1 轴正在运动
            return {false, QObject::tr("%1 axis is moving").arg(it.key())};
    }
    for (auto it = targets.cbegin(); it != targets.cend(); ++it) {
        const auto axis = magic_enum::enum_cast<Axis>(it.key().toStdString());
        if (!m_motionControl->MoveAbsolute(axis.value(), it.value(), velocity)) {
            (void)m_motionControl->StopMotion();
            // 中文翻译：%1 轴移动至%2失败
            return {false, QObject::tr("%1 axis movement to %2 failed").arg(it.key(), positionName)};
        }
    }
    return {};
}

lcnc::process::DeviceStatusSnapshot ProcessDeviceRuntime::pollStatus(
    const QStringList& axisNames, const QVector<QPair<QString, QString>>& digitalOutputs)
{
    lcnc::process::DeviceStatusSnapshot snapshot;
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected())
        return snapshot;
    snapshot.connected = true;
    snapshot.axes.reserve(axisNames.size());
    for (const QString& name : axisNames) {
        lcnc::process::DeviceAxisStatusSample sample;
        sample.name = name;
        const auto axis = magic_enum::enum_cast<Axis>(name.toStdString());
        if (axis.has_value() && m_motionControl->IsMotorCreated(axis.value())) {
            double position = 0.0;
            if (m_motionControl->GetActualPos(axis.value(), position)) {
                sample.pos = position;
                sample.enabled = m_motionControl->IsEnabled(axis.value());
                sample.valid = true;
            }
        }
        snapshot.axes.push_back(sample);
    }
    snapshot.digitalOutputs.reserve(digitalOutputs.size());
    for (const auto& entry : digitalOutputs) {
        lcnc::process::DeviceDigitalOutputSample sample;
        sample.channel = entry.first;
        sample.displayName = entry.second;
        QString enumName = entry.first;
        if (enumName.startsWith(QLatin1Char('a')) && enumName.size() >= 2)
            enumName = enumName.mid(1);
        const auto output = magic_enum::enum_cast<DigitalOUT>(enumName.toStdString());
        int value = 0;
        if (output.has_value() && m_motionControl->m_mapDigitalOUT.count(output.value())
            && m_motionControl->DigitalOutputGet(output.value(), value)) {
            sample.value = value != 0;
            sample.valid = true;
        }
        snapshot.digitalOutputs.push_back(sample);
    }
    return snapshot;
}

lcnc::process::DevicePeripheralSnapshot ProcessDeviceRuntime::pollPeripheralStatus()
{
    lcnc::process::DevicePeripheralSnapshot snapshot;
    const auto lock = lockDeviceAccess();
    if (!m_pLaserDevice)
        return snapshot;
    snapshot.deviceName = QString::fromStdString(m_pLaserDevice->GetName());
    snapshot.connected = m_pLaserDevice->IsConnected();
    snapshot.initialized = m_pLaserDevice->IsInited();
    snapshot.valid = true;
    if (snapshot.connected && snapshot.deviceName != QStringLiteral("Simulator")
        && snapshot.deviceName != QStringLiteral("AnalogControl")) {
        snapshot.diagnostic = QString::fromStdString(m_pLaserDevice->GetTroubleshooting());
    }
    return snapshot;
}

bool ProcessDeviceRuntime::readDigitalChannel(const QString& channel,
                                              bool* value,
                                              QString* errorMessage)
{
    if (!value)
        return false;
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected()) {
        // 中文翻译：运动控制器未连接
        if (errorMessage) *errorMessage = QObject::tr("Motion controller not connected");
        return false;
    }

    QString enumName = channel.trimmed();
    if (enumName.startsWith(QLatin1Char('a')) && enumName.size() >= 2)
        enumName = enumName.mid(1);
    int raw = 0;
    if (auto input = magic_enum::enum_cast<DigitalIN>(enumName.toStdString()); input.has_value()
        && m_motionControl->m_mapDigitalIN.count(input.value())
        && m_motionControl->DigitalInputGet(input.value(), raw)) {
        *value = raw != 0;
        return true;
    }
    if (auto output = magic_enum::enum_cast<DigitalOUT>(enumName.toStdString()); output.has_value()
        && m_motionControl->m_mapDigitalOUT.count(output.value())
        && m_motionControl->DigitalOutputGet(output.value(), raw)) {
        *value = raw != 0;
        return true;
    }
    // 中文翻译：通道未配置或读取失败: %1
    if (errorMessage) *errorMessage = QObject::tr("Channel not configured or read failed: %1").arg(channel);
    return false;
}

bool ProcessDeviceRuntime::readAnalogChannel(const QString& channel,
                                             double* value,
                                             QString* errorMessage)
{
    if (!value)
        return false;
    const auto lock = lockDeviceAccess();
    if (!m_motionControl || !m_motionControl->IsConnected()) {
        // 中文翻译：运动控制器未连接
        if (errorMessage) *errorMessage = QObject::tr("Motion controller not connected");
        return false;
    }

    QString enumName = channel.trimmed();
    if (enumName.startsWith(QLatin1Char('a')) && enumName.size() >= 2)
        enumName = enumName.mid(1);
    if (auto input = magic_enum::enum_cast<AnalogIN>(enumName.toStdString()); input.has_value()
        && m_motionControl->m_mapAnalogIN.count(input.value())
        && m_motionControl->AnalogInputGet(input.value(), *value)) {
        return true;
    }
    // 中文翻译：模拟量通道未配置或读取失败: %1
    if (errorMessage) *errorMessage = QObject::tr("Analog channel is not configured or failed to read: %1").arg(channel);
    return false;
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::runPreflight(
    const lcnc::process::ProcessPreflightRequest& request,
    lcnc::process::ProcessPreflightReport* report)
{
    const auto fail = [](const QString& error) {
        return lcnc::process::DeviceCommandResult{false, error};
    };
    if (!report)
        // 中文翻译：预检报告不可用
        return fail(QObject::tr("Preflight report unavailable"));

    const auto lock = lockDeviceAccess();
    MotionControl* const mc = m_motionControl.get();
    if (!mc || !mc->IsConnected())
        // 中文翻译：运动控制器未连接或连接已断开
        return fail(QObject::tr("Motion controller not connected or disconnected"));
    if (mc->ErrorOccurred())
        // 中文翻译：运动控制器存在异常，请清除故障后再加工
        return fail(QObject::tr("There is an abnormality in the motion controller. Please clear the fault before processing."));

    int fault = 0;
    if (!mc->IsAxisStatusNormal(fault))
        // 中文翻译：运动控制器状态读取失败，请检查控制器连接
        return fail(QObject::tr("Motion controller status reading failed, please check the controller connection"));
    if (fault != 0)
        // 中文翻译：运动控制器故障码: %1，请清除故障后再加工
        return fail(QObject::tr("Motion controller fault code: %1, please clear the fault before processing").arg(fault));

    for (auto it = request.lockedAxisTargets.cbegin(); it != request.lockedAxisTargets.cend(); ++it) {
        const auto axis = magic_enum::enum_cast<Axis>(it.key().toStdString());
        if (!axis.has_value() || !mc->IsMotorCreated(*axis))
            // 中文翻译：锁定轴 %1 未在控制器中创建
            return fail(QObject::tr("Locked axis %1 was not created in the controller").arg(it.key()));
        // 降阶四轴的 A/B 是锁定姿态轴，而非回零目标轴：AC/BC 管材加工会先
        // 将其低速置于配置的 ±90° 姿态。回零状态不等同于该安全加工姿态，
        // 因而这里只要求可控使能，随后以实际位置、故障和限位检查确认到位。
        if (!mc->IsEnabled(*axis))
            // 中文翻译：锁定轴 %1 未使能
            return fail(QObject::tr("Locked axis %1 is not enabled").arg(it.key()));
        if (!mc->MoveAbsolute(*axis, it.value(), request.lockedAxisMoveVelocity)) {
            (void)mc->StopMotion();
            // 中文翻译：锁定轴 %1 移动到安全姿态失败
            return fail(QObject::tr("Locked axis %1 failed to move to the safe posture").arg(it.key()));
        }
    }
    if (!request.lockedAxisTargets.isEmpty()) {
        QElapsedTimer timer;
        timer.start();
        while (mc->IsAxisMoving()) {
            if (timer.elapsed() > 30000) {
                (void)mc->StopMotion();
                // 中文翻译：等待锁定轴到达安全姿态超时
                return fail(QObject::tr("Timed out waiting for locked axes to reach the safe posture"));
            }
            QThread::msleep(20);
        }
        int postMoveFault = 0;
        if (!mc->IsAxisStatusNormal(postMoveFault) || postMoveFault != 0)
            // 中文翻译：锁定轴置位后控制器状态异常
            return fail(QObject::tr("Controller status is abnormal after positioning locked axes"));
        for (auto it = request.lockedAxisTargets.cbegin(); it != request.lockedAxisTargets.cend(); ++it) {
            const auto axis = magic_enum::enum_cast<Axis>(it.key().toStdString());
            double actual = 0.0;
            if (!axis.has_value() || !mc->GetActualPos(*axis, actual)
                || std::abs(actual - it.value()) > 0.05)
                // 中文翻译：锁定轴 %1 未到达目标位置
                return fail(QObject::tr("Locked axis %1 did not reach its target position").arg(it.key()));
        }
    }

    QStringList disabledAxes;
    QStringList unregisteredAxes;
    for (const QString& axisName : request.axisNames) {
        const QString name = axisName.trimmed().toUpper();
        if (name.isEmpty() || name == QStringLiteral("BASE"))
            continue;
        const auto axis = magic_enum::enum_cast<Axis>(name.toStdString());
        if (!axis.has_value())
            continue;
        if (!mc->IsMotorCreated(axis.value())) {
            unregisteredAxes.append(name);
            continue;
        }
        double position = 0.0;
        if (mc->GetActualPos(axis.value(), position))
            report->axisPositions.insert(name, position);
        const bool enabled = mc->IsEnabled(axis.value());
        report->axisEnabled.insert(name, enabled);
        if (!enabled)
            disabledAxes.append(name);
    }
    if (!unregisteredAxes.isEmpty())
        // 中文翻译：运动控制器轴系未注册: %1
        return fail(QObject::tr("Motion controller axis is not registered: %1")
                        .arg(unregisteredAxes.join(QObject::tr("，"))));
    if (!disabledAxes.isEmpty())
        // 中文翻译：运动控制器轴系未使能: %1
        return fail(QObject::tr("Motion controller axis is not enabled: %1")
                        .arg(disabledAxes.join(QObject::tr("，"))));

    LaserDevice* const laser = m_pLaserDevice;
    if (!laser)
        // 中文翻译：激光器未创建，请先连接设备
        return fail(QObject::tr("The laser has not been created, please connect the device first"));
    const QString laserName = QString::fromStdString(laser->GetName());
    const bool analogLaser = laserName.compare(QStringLiteral("AnalogControl"), Qt::CaseInsensitive) == 0;
    if (!analogLaser && !laser->IsConnected())
        // 中文翻译：激光器未连接或连接已断开
        return fail(QObject::tr("Laser not connected or disconnected"));
    if (!laser->IsInited())
        // 中文翻译：激光器参数未初始化
        return fail(QObject::tr("Laser parameters not initialized"));
    if (!analogLaser) {
        const QString laserFault = QString::fromStdString(laser->GetTroubleshooting()).trimmed();
        if (!laserFault.isEmpty() && laserFault.compare(QStringLiteral("OK"), Qt::CaseInsensitive) != 0)
            // 中文翻译：激光器异常: %1
            return fail(QObject::tr("Laser exception: %1").arg(laserFault));
    }

    for (const auto& guard : request.digitalGuards) {
        if (!guard.enabled)
            continue;
        if (guard.channel.trimmed().isEmpty())
            // 中文翻译：%1监控通道未配置
            return fail(QObject::tr("%1 monitoring channel is not configured").arg(guard.title));
        QString enumName = guard.channel.trimmed();
        if (enumName.startsWith(QLatin1Char('a')) && enumName.size() >= 2)
            enumName = enumName.mid(1);
        const auto input = magic_enum::enum_cast<DigitalIN>(enumName.toStdString());
        if (!input.has_value() || !mc->m_mapDigitalIN.count(input.value()))
            // 中文翻译：%1状态获取失败: 通道未配置: %2
            return fail(QObject::tr("%1 status acquisition failed: Channel not configured: %2")
                            .arg(guard.title, guard.channel));
        int raw = 0;
        if (!mc->DigitalInputGet(input.value(), raw))
            // 中文翻译：%1状态获取失败: 通道读取失败: %2
            return fail(QObject::tr("%1 Status acquisition failed: Channel read failed: %2")
                            .arg(guard.title, guard.channel));
        if (raw != 0)
            // 中文翻译：%1异常
            return fail(QObject::tr("%1Exception").arg(guard.title));
    }

    for (const auto& guard : request.analogGuards) {
        if (!guard.enabled)
            continue;
        if (guard.channel.trimmed().isEmpty())
            // 中文翻译：%1通道未配置
            return fail(QObject::tr("%1 channel is not configured").arg(guard.title));
        QString enumName = guard.channel.trimmed();
        if (enumName.startsWith(QLatin1Char('a')) && enumName.size() >= 2)
            enumName = enumName.mid(1);
        const auto input = magic_enum::enum_cast<AnalogIN>(enumName.toStdString());
        if (!input.has_value() || !mc->m_mapAnalogIN.count(input.value()))
            // 中文翻译：%1通道未配置: %2
            return fail(QObject::tr("Channel %1 is not configured: %2").arg(guard.title, guard.channel));
        double value = 0.0;
        if (!mc->AnalogInputGet(input.value(), value))
            // 中文翻译：%1状态获取失败: %2
            return fail(QObject::tr("%1 status acquisition failed: %2").arg(guard.title, guard.channel));
        if (value < guard.threshold)
            // 中文翻译：%1异常: 当前值 %2 %3，阈值 %4 %3
            return fail(QObject::tr("%1Exception: Current value %2 %3, threshold %4 %3")
                            .arg(guard.title,
                                 QString::number(value, 'f', 3),
                                 guard.unit,
                                 QString::number(guard.threshold, 'f', 3)));
    }
    return {};
}

lcnc::process::DeviceCommandResult ProcessDeviceRuntime::validateContourBoundary()
{
    const auto lock = lockDeviceAccess();
    MotionControl* const mc = m_motionControl.get();
    lcnc::process::ContourBoundaryHealth health;
    health.connected = mc && mc->IsConnected();
    if (!health.connected)
        return lcnc::process::evaluateContourBoundaryHealth(health);
    int fault = 0;
    health.statusReadable = mc->IsAxisStatusNormal(fault);
    health.faultCode = fault;
    if (!health.statusReadable || health.faultCode != 0)
        return lcnc::process::evaluateContourBoundaryHealth(health);
    for (Axis axis : mc->m_vecMotors) {
        if (!mc->IsMotorCreated(axis))
            health.missingAxes.append(QString::fromLatin1(magic_enum::enum_name(axis).data()));
        else if (!mc->IsEnabled(axis))
            health.disabledAxes.append(QString::fromLatin1(magic_enum::enum_name(axis).data()));
    }
    return lcnc::process::evaluateContourBoundaryHealth(health);
}

std::unique_ptr<lcnc::process::IMotionCommandSink> ProcessDeviceRuntime::createMotionSink(
    bool simulationMode,
    lcnc::process::PureSimulationToolpathTicker* simTicker,
    const lcnc::process::MotionSinkCallbacks& callbacks,
    const lcnc::MachineAxisLayout& layout)
{
    const auto lock = lockDeviceAccess();
    return lcnc::process::MotionSinkFactory::create(
        simulationMode ? nullptr : m_motionControl.get(), simulationMode, simTicker,
        callbacks, layout);
}
