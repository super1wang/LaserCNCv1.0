#include "modules/process/steps/services/process_workflow_services.h"

#include "modules/process/runtime/device_command_queue.h"
#include "modules/process/runtime/process_device_runtime.h"

#include <QElapsedTimer>
#include <QPair>
#include <QThread>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <utility>

namespace lcnc::process {

namespace {

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

ProcessMotionWorkflowService::ProcessMotionWorkflowService(ProcessDeviceRuntime* service,
                                                           DeviceCommandQueue* deviceQueue)
    : m_service(service)
    , m_deviceQueue(deviceQueue)
{
}

bool ProcessMotionWorkflowService::moveAxis(const QString& axis,
                                          const QString& mode,
                                          double target,
                                          double velocity,
                                          int timeoutMs,
                                          QString* errorMessage)
{
    ProcessDeviceRuntime* const service = m_service;
    return executeDeviceCommand(m_deviceQueue, TaskPriority::Workflow, timeoutMs,
        [service, axis, mode, target, velocity] {
            const auto eAxis = enum_cast<Axis>(axis.trimmed().toUpper().toStdString());
            if (!eAxis.has_value())
                // 中文翻译：轴 %1 未注册
                return DeviceCommandResult{false, QObject::tr("Axis %1 is not registered").arg(axis)};
            if (!service)
                // 中文翻译：运动控制器未连接
                return DeviceCommandResult{false, QObject::tr("Motion controller not connected")};
            return isRelativeMode(mode)
                ? service->moveRelative(eAxis.value(), target, velocity)
                : service->moveAbsolute(eAxis.value(), target, velocity);
        }, errorMessage);
}

bool ProcessMotionWorkflowService::moveAxes(const QVariantList& rows,
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

    ProcessDeviceRuntime* const service = m_service;
    return executeDeviceCommand(m_deviceQueue, TaskPriority::Workflow, timeoutMs,
        [service, rows] {
            if (!service)
                // 中文翻译：运动控制器未连接
                return DeviceCommandResult{false, QObject::tr("Motion controller not connected")};
            QVector<Axis> axes;
            QVector<double> positions;
            double velocity = 5.0;
            bool relative = false;
            for (const QVariant& item : rows) {
                const QVariantMap row = item.toMap();
                const QString axisName = row.value(QStringLiteral("axis")).toString().trimmed().toUpper();
                const auto axis = enum_cast<Axis>(axisName.toStdString());
                if (!axis.has_value())
                    // 中文翻译：轴 %1 未注册
                    return DeviceCommandResult{false, QObject::tr("Axis %1 is not registered").arg(axisName)};
                axes.push_back(axis.value());
                positions.push_back(row.value(QStringLiteral("target"), 0.0).toDouble());
                velocity = row.value(QStringLiteral("velocity"), velocity).toDouble();
                relative = isRelativeMode(row.value(QStringLiteral("mode"), QStringLiteral("absolute")).toString());
            }
            return service->moveAxes(axes, positions, velocity, relative);
        }, errorMessage);
}

bool ProcessMotionWorkflowService::setAxisPosition(const QVariantList& axes,
                                                   int timeoutMs,
                                                   QString* errorMessage)
{
    ProcessDeviceRuntime* const service = m_service;
    return executeDeviceCommand(m_deviceQueue, TaskPriority::Workflow, timeoutMs,
        [service, axes] {
            if (!service)
                // 中文翻译：运动控制器未连接
                return DeviceCommandResult{false, QObject::tr("Motion controller not connected")};
            QVector<QPair<Axis, double>> targets;
            for (const QVariant& item : axes) {
                const QVariantMap row = item.toMap();
                const QString axisName = row.value(QStringLiteral("axis")).toString().trimmed().toUpper();
                const auto eAxis = enum_cast<Axis>(axisName.toStdString());
                if (!eAxis.has_value())
                    // 中文翻译：轴 %1 未注册
                    return DeviceCommandResult{false, QObject::tr("Axis %1 is not registered").arg(axisName)};
                targets.append({eAxis.value(), row.value(QStringLiteral("position"), 0.0).toDouble()});
            }
            return service->setAxisPositions(targets);
        }, errorMessage);
}

bool ProcessMotionWorkflowService::stopMotion(QString* errorMessage)
{
    ProcessDeviceRuntime* const service = m_service;
    return executeDeviceCommand(m_deviceQueue, TaskPriority::Stop, 5000,
        [service] {
            if (!service)
                // 中文翻译：运动控制器未连接
                return DeviceCommandResult{false, QObject::tr("Motion controller not connected")};
            return service->stopAllMotion();
        }, errorMessage);
}

ProcessIoWorkflowService::ProcessIoWorkflowService(ProcessDeviceRuntime* service,
                                                   DeviceCommandQueue* deviceQueue)
    : m_service(service)
    , m_deviceQueue(deviceQueue)
{
}

bool ProcessIoWorkflowService::setOutput(const QString& signalType,
                                       const QString& ioName,
                                       const QVariant& value,
                                       QString* errorMessage)
{
    ProcessDeviceRuntime* const service = m_service;
    return executeDeviceCommand(m_deviceQueue, TaskPriority::Workflow, 5000,
        [service, signalType, ioName, value] {
            if (!service)
                // 中文翻译：运动控制器未连接
                return DeviceCommandResult{false, QObject::tr("Motion controller not connected")};
            const bool digital = signalType.compare(QStringLiteral("digital"), Qt::CaseInsensitive) == 0;
            if (digital) {
                if (const auto output = ioEnumFromKey<DigitalOUT>(ioName))
                    return service->setDigitalOutput(output.value(), value.toBool());
            } else if (const auto output = ioEnumFromKey<AnalogOUT>(ioName)) {
                return service->setAnalogOutput(output.value(), value.toDouble(), ioName);
            }
            // 中文翻译：输出信号 %1 设置失败
            return DeviceCommandResult{false, QObject::tr("Output signal %1 setup failed").arg(ioName)};
        }, errorMessage);
}

bool ProcessIoWorkflowService::waitInput(const QString& signalType,
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
        ProcessDeviceRuntime* const service = m_service;
        if (!executeDeviceCommand(m_deviceQueue, TaskPriority::Workflow,
                                  std::max(1000, interval * 2),
            [service, analog, targetValue, matched, ioName] {
                QString error;
                if (!service)
                    // 中文翻译：运动控制器未连接
                    return DeviceCommandResult{false, QObject::tr("Motion controller not connected")};
                if (analog) {
                    double value = 0.0;
                    if (!service->readAnalogChannel(ioName, &value, &error))
                        return DeviceCommandResult{false, error};
                    if (std::abs(value - targetValue.toDouble()) < 1e-6)
                        *matched = true;
                } else {
                    bool value = false;
                    if (!service->readDigitalChannel(ioName, &value, &error))
                        return DeviceCommandResult{false, error};
                    if (value == targetValue.toBool())
                        *matched = true;
                }
                if (!error.isEmpty())
                    return DeviceCommandResult{false, error};
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

bool ProcessIoWorkflowService::readInput(const QString& signalType,
                                         const QString& ioName,
                                         QVariant* value,
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
    ProcessDeviceRuntime* const service = m_service;
    return executeDeviceCommand(m_deviceQueue, TaskPriority::Workflow, 5000,
        [service, analog, ioName, value] {
            QString error;
            if (!service)
                // 中文翻译：运动控制器未连接
                return DeviceCommandResult{false, QObject::tr("Motion controller not connected")};
            if (analog) {
                double channelValue = 0.0;
                if (!service->readAnalogChannel(ioName, &channelValue, &error))
                    return DeviceCommandResult{false, error};
                if (value)
                    *value = channelValue;
            } else {
                bool channelValue = false;
                if (!service->readDigitalChannel(ioName, &channelValue, &error))
                    return DeviceCommandResult{false, error};
                if (value)
                    *value = channelValue;
            }
            return DeviceCommandResult{};
        }, errorMessage);
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
