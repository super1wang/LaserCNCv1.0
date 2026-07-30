#include "modules/process/steps/services/legacy_process_services.h"

#include "modules/process/runtime/device_command_queue.h"
#include "modules/process/System/Service.h"

#include <QElapsedTimer>
#include <QThread>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <utility>

namespace lcnc::process {

namespace {

MotionControl* motionControl(Service* service, QString* errorMessage)
{
    MotionControl* mc = service ? service->GetMotionControl() : nullptr;
    if (!mc || !mc->IsConnected()) {
        if (errorMessage)
            // 中文翻译：运动控制器未连接
            *errorMessage = QObject::tr("Motion controller not connected");
        return nullptr;
    }
    return mc;
}

bool resolveAxis(MotionControl* mc, const QString& axis, Axis* out, QString* errorMessage)
{
    if (!out)
        return false;
    const QString key = axis.trimmed().toUpper();
    auto eAxis = enum_cast<Axis>(key.toStdString());
    if (!eAxis.has_value() || !mc->IsMotorCreated(eAxis.value())) {
        if (errorMessage)
            // 中文翻译：轴 %1 未注册
            *errorMessage = QObject::tr("Axis %1 is not registered").arg(key);
        return false;
    }
    *out = eAxis.value();
    return true;
}

bool isRelativeMode(const QString& mode)
{
    return mode.compare(QStringLiteral("relative"), Qt::CaseInsensitive) == 0
        // 中文翻译：相对
        || mode.compare(QStringLiteral("relatively"), Qt::CaseInsensitive) == 0;
}

QString stripIoKeyPrefix(const QString& tomlKey)
{
    // toml key 形如 "aLaser" / "aStart"，对应枚举名是 "Laser" / "Start"。
    QString trimmed = tomlKey.trimmed();
    if (trimmed.size() >= 2 && trimmed.startsWith(QLatin1Char('a')))
        return trimmed.mid(1);
    return trimmed;
}

template <typename Enum>
std::optional<Enum> ioEnumFromKey(const QString& tomlKey)
{
    return enum_cast<Enum>(stripIoKeyPrefix(tomlKey).toStdString());
}

template <typename Fn>
bool executeDeviceCommand(DeviceCommandQueue* queue,
                          TaskPriority priority,
                          int timeoutMs,
                          Fn&& command,
                          QString* errorMessage)
{
    if (!queue) {
        if (errorMessage)
            // 中文翻译：设备命令队列不可用
            *errorMessage = QObject::tr("Device command queue is unavailable");
        return false;
    }
    const DeviceCommandResult result = queue->executeAndWait(
        DeviceCommandQueue::ResultCommand(std::forward<Fn>(command)), priority, timeoutMs);
    if (!result.success && errorMessage)
        *errorMessage = result.error;
    return result.success;
}

} // namespace

LegacyProcessMotionService::LegacyProcessMotionService(Service* service,
                                                       DeviceCommandQueue* deviceQueue)
    : m_service(service)
    , m_deviceQueue(deviceQueue)
{
}

bool LegacyProcessMotionService::moveAxis(const QString& axis,
                                          const QString& mode,
                                          double target,
                                          double velocity,
                                          int timeoutMs,
                                          QString* errorMessage)
{
    Service* const service = m_service;
    return executeDeviceCommand(m_deviceQueue, TaskPriority::Workflow, timeoutMs,
        [service, axis, mode, target, velocity] {
            QString error;
            const auto deviceLock = service ? service->lockDeviceAccess() : Service::DeviceLock{};
            MotionControl* mc = motionControl(service, &error);
            if (!mc)
                return DeviceCommandResult{false, error};
            Axis eAxis;
            if (!resolveAxis(mc, axis, &eAxis, &error))
                return DeviceCommandResult{false, error};
            const bool ok = isRelativeMode(mode)
                ? mc->MoveRelative(eAxis, target, velocity)
                : mc->MoveAbsolute(eAxis, target, velocity);
            if (!ok)
                // 中文翻译：轴 %1 运动失败
                error = QObject::tr("Axis %1 movement failed").arg(axis);
            return DeviceCommandResult{ok, error};
        }, errorMessage);
}

bool LegacyProcessMotionService::moveAxes(const QVariantList& rows,
                                          const QString& mode,
                                          int timeoutMs,
                                          QString* errorMessage)
{
    const bool sync = mode.compare(QStringLiteral("sync"), Qt::CaseInsensitive) == 0
        || mode.compare(QStringLiteral("synchronous"), Qt::CaseInsensitive) == 0
        // 中文翻译：同步
        || mode.compare(QStringLiteral("sync"), Qt::CaseInsensitive) == 0;
    if (!sync) {
        for (const QVariant& item : rows) {
            const QVariantMap row = item.toMap();
            if (!moveAxis(row.value(QStringLiteral("axis")).toString(),
                          row.value(QStringLiteral("mode"), QStringLiteral("absolute")).toString(),
                          row.value(QStringLiteral("target"), 0.0).toDouble(),
                          row.value(QStringLiteral("velocity"), 5.0).toDouble(),
                          timeoutMs,
                          errorMessage)) {
                return false;
            }
        }
        return true;
    }

    Service* const service = m_service;
    return executeDeviceCommand(m_deviceQueue, TaskPriority::Workflow, timeoutMs,
        [service, rows] {
            QString error;
            const auto deviceLock = service ? service->lockDeviceAccess() : Service::DeviceLock{};
            MotionControl* mc = motionControl(service, &error);
            if (!mc)
                return DeviceCommandResult{false, error};
            if (QString::fromStdString(mc->GetName()) == QStringLiteral("GTN"))
                return DeviceCommandResult{false,
                    // 中文翻译：GTN 控制器暂不支持同步多轴运动，请改为顺序执行
                    QObject::tr("The GTN controller does not currently support synchronous multi-axis motion. Please execute it sequentially instead.")};
            vector<Axis> axes;
            vector<double> positions;
            double velocity = 5.0;
            bool relative = false;
            for (const QVariant& item : rows) {
                const QVariantMap row = item.toMap();
                Axis eAxis;
                if (!resolveAxis(mc, row.value(QStringLiteral("axis")).toString(), &eAxis, &error))
                    return DeviceCommandResult{false, error};
                axes.push_back(eAxis);
                positions.push_back(row.value(QStringLiteral("target"), 0.0).toDouble());
                velocity = row.value(QStringLiteral("velocity"), velocity).toDouble();
                relative = isRelativeMode(row.value(QStringLiteral("mode"), QStringLiteral("absolute")).toString());
            }
            const bool ok = relative ? mc->MoveMRelative(axes, positions, velocity)
                                     : mc->MoveMAbsolute(axes, positions, velocity);
            if (!ok)
                // 中文翻译：同步多轴运动失败
                error = QObject::tr("Synchronized multi-axis motion failed");
            return DeviceCommandResult{ok, error};
        }, errorMessage);
}

bool LegacyProcessMotionService::stopMotion(QString* errorMessage)
{
    Service* const service = m_service;
    return executeDeviceCommand(m_deviceQueue, TaskPriority::Stop, 5000,
        [service] {
            QString error;
            const auto deviceLock = service ? service->lockDeviceAccess() : Service::DeviceLock{};
            MotionControl* mc = motionControl(service, &error);
            if (!mc)
                return DeviceCommandResult{false, error};
            const bool ok = mc->StopMotion() && mc->StopAllBuffer();
            if (!ok)
                // 中文翻译：停止运动失败
                error = QObject::tr("Stop motion failed");
            return DeviceCommandResult{ok, error};
        }, errorMessage);
}

LegacyProcessIoService::LegacyProcessIoService(Service* service,
                                               DeviceCommandQueue* deviceQueue)
    : m_service(service)
    , m_deviceQueue(deviceQueue)
{
}

bool LegacyProcessIoService::setOutput(const QString& signalType,
                                       const QString& ioName,
                                       const QVariant& value,
                                       QString* errorMessage)
{
    Service* const service = m_service;
    return executeDeviceCommand(m_deviceQueue, TaskPriority::Workflow, 5000,
        [service, signalType, ioName, value] {
            QString error;
            const auto deviceLock = service ? service->lockDeviceAccess() : Service::DeviceLock{};
            MotionControl* mc = motionControl(service, &error);
            if (!mc)
                return DeviceCommandResult{false, error};
            const bool digital = signalType.compare(QStringLiteral("digital"), Qt::CaseInsensitive) == 0;
            bool ok = false;
            if (digital) {
                if (auto e = ioEnumFromKey<DigitalOUT>(ioName)) {
                    if (mc->m_mapDigitalOUT.count(e.value()))
                        ok = mc->DigitalOutputSet(e.value(), value.toBool() ? 1 : 0);
                    else
                        // 中文翻译：数字量输出 %1 未注册
                        error = QObject::tr("Digital output %1 is not registered").arg(ioName);
                }
            } else if (auto e = ioEnumFromKey<AnalogOUT>(ioName)) {
                if (mc->m_mapAnalogOUT.count(e.value()))
                    ok = mc->AnalogOutputSet(e.value(), value.toDouble());
                else
                    // 中文翻译：模拟量输出 %1 未注册
                    error = QObject::tr("Analog output %1 is not registered").arg(ioName);
            }
            if (!ok && error.isEmpty())
                // 中文翻译：输出信号 %1 设置失败
                error = QObject::tr("Output signal %1 setup failed").arg(ioName);
            return DeviceCommandResult{ok, error};
        }, errorMessage);
}

bool LegacyProcessIoService::waitInput(const QString& signalType,
                                       const QString& ioName,
                                       const QVariant& targetValue,
                                       int timeoutMs,
                                       int pollIntervalMs,
                                       QString* errorMessage)
{
    const bool analog = signalType.compare(QStringLiteral("analog"), Qt::CaseInsensitive) == 0;
    auto digitalEnum = analog ? std::optional<DigitalIN>{} : ioEnumFromKey<DigitalIN>(ioName);
    auto analogEnum = analog ? ioEnumFromKey<AnalogIN>(ioName) : std::optional<AnalogIN>{};
    if ((!analog && !digitalEnum.has_value()) || (analog && !analogEnum.has_value())) {
        if (errorMessage)
            // 中文翻译：输入信号 %1 未注册
            *errorMessage = QObject::tr("Input signal %1 is not registered").arg(ioName);
        return false;
    }
    QElapsedTimer timer;
    timer.start();
    const int interval = std::clamp(pollIntervalMs, 10, 1000);
    while (timeoutMs <= 0 || timer.elapsed() <= timeoutMs) {
        const auto matched = std::make_shared<bool>(false);
        Service* const service = m_service;
        if (!executeDeviceCommand(m_deviceQueue, TaskPriority::Workflow,
                                  std::max(1000, interval * 2),
            [service, analog, digitalEnum, analogEnum, targetValue, matched, ioName] {
                QString error;
                const auto deviceLock = service ? service->lockDeviceAccess() : Service::DeviceLock{};
                MotionControl* mc = motionControl(service, &error);
                if (!mc)
                    return DeviceCommandResult{false, error};
            if (analog) {
                if (!mc->m_mapAnalogIN.count(analogEnum.value()))
                    // 中文翻译：输入信号 %1 未注册
                    return DeviceCommandResult{false, QObject::tr("Input signal %1 is not registered").arg(ioName)};
                double value = 0.0;
                if (mc->AnalogInputGet(analogEnum.value(), value)
                    && std::abs(value - targetValue.toDouble()) < 1e-6) {
                    *matched = true;
                }
            } else {
                if (!mc->m_mapDigitalIN.count(digitalEnum.value()))
                    // 中文翻译：输入信号 %1 未注册
                    return DeviceCommandResult{false, QObject::tr("Input signal %1 is not registered").arg(ioName)};
                int value = 0;
                if (mc->DigitalInputGet(digitalEnum.value(), value)
                    && (value != 0) == targetValue.toBool()) {
                    *matched = true;
                }
            }
                return DeviceCommandResult{};
            }, errorMessage)) {
            return false;
        }
        if (*matched)
            return true;
        QThread::msleep(static_cast<unsigned long>(interval));
    }
    if (errorMessage)
        // 中文翻译：等待输入 %1 超时
        *errorMessage = QObject::tr("Timed out waiting for input %1").arg(ioName);
    return false;
}

void CallbackProcessCuttingService::setSnapshotProvider(std::function<ProcessToolpathSnapshot()> provider)
{
    m_snapshotProvider = std::move(provider);
}

void CallbackProcessCuttingService::setExecutor(ExecutorFn executor)
{
    m_executor = std::move(executor);
}

ProcessToolpathSnapshot CallbackProcessCuttingService::toolpathSnapshot() const
{
    return m_snapshotProvider ? m_snapshotProvider() : ProcessToolpathSnapshot{};
}

bool CallbackProcessCuttingService::executeNormalCutting(const QString& nodeId,
                                                          const QVariantMap& parameters,
                                                          ProcessInterruptContext* interrupt,
                                                          QString* errorMessage)
{
    if (!m_executor)
        return true;
    return m_executor(nodeId, parameters, interrupt, errorMessage);
}

} // namespace lcnc::process
