#include "modules/process/steps/services/legacy_process_services.h"

#include "modules/process/System/Service.h"

#include <QElapsedTimer>
#include <QThread>

#include <algorithm>
#include <cmath>
#include <optional>

namespace lcnc::process {

namespace {

MotionControl* motionControl(Service* service, QString* errorMessage)
{
    MotionControl* mc = service ? service->GetMotionControl() : nullptr;
    if (!mc || !mc->IsConnected()) {
        if (errorMessage)
            *errorMessage = QObject::tr("运动控制器未连接");
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
            *errorMessage = QObject::tr("轴 %1 未注册").arg(key);
        return false;
    }
    *out = eAxis.value();
    return true;
}

bool isRelativeMode(const QString& mode)
{
    return mode.compare(QStringLiteral("relative"), Qt::CaseInsensitive) == 0
        || mode.compare(QStringLiteral("相对"), Qt::CaseInsensitive) == 0;
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

} // namespace

LegacyProcessMotionService::LegacyProcessMotionService(Service* service)
    : m_service(service)
{
}

bool LegacyProcessMotionService::moveAxis(const QString& axis,
                                          const QString& mode,
                                          double target,
                                          double velocity,
                                          int timeoutMs,
                                          QString* errorMessage)
{
    Q_UNUSED(timeoutMs);
    MotionControl* mc = motionControl(m_service, errorMessage);
    if (!mc)
        return false;
    Axis eAxis;
    if (!resolveAxis(mc, axis, &eAxis, errorMessage))
        return false;
    const bool ok = isRelativeMode(mode)
        ? mc->MoveRelative(eAxis, target, velocity)
        : mc->MoveAbsolute(eAxis, target, velocity);
    if (!ok && errorMessage)
        *errorMessage = QObject::tr("轴 %1 运动失败").arg(axis);
    return ok;
}

bool LegacyProcessMotionService::moveAxes(const QVariantList& rows,
                                          const QString& mode,
                                          int timeoutMs,
                                          QString* errorMessage)
{
    Q_UNUSED(timeoutMs);
    MotionControl* mc = motionControl(m_service, errorMessage);
    if (!mc)
        return false;

    const bool sync = mode.compare(QStringLiteral("sync"), Qt::CaseInsensitive) == 0
        || mode.compare(QStringLiteral("synchronous"), Qt::CaseInsensitive) == 0
        || mode.compare(QStringLiteral("同步"), Qt::CaseInsensitive) == 0;
    if (sync && QString::fromStdString(mc->GetName()) == QStringLiteral("GTN")) {
        if (errorMessage)
            *errorMessage = QObject::tr("GTN 控制器暂不支持同步多轴运动，请改为顺序执行");
        return false;
    }
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

    vector<Axis> axes;
    vector<double> positions;
    double velocity = 5.0;
    bool relative = false;
    for (const QVariant& item : rows) {
        const QVariantMap row = item.toMap();
        Axis eAxis;
        if (!resolveAxis(mc, row.value(QStringLiteral("axis")).toString(), &eAxis, errorMessage))
            return false;
        axes.push_back(eAxis);
        positions.push_back(row.value(QStringLiteral("target"), 0.0).toDouble());
        velocity = row.value(QStringLiteral("velocity"), velocity).toDouble();
        relative = isRelativeMode(row.value(QStringLiteral("mode"), QStringLiteral("absolute")).toString());
    }

    const bool ok = relative ? mc->MoveMRelative(axes, positions, velocity)
                             : mc->MoveMAbsolute(axes, positions, velocity);
    if (!ok && errorMessage)
        *errorMessage = QObject::tr("同步多轴运动失败");
    return ok;
}

bool LegacyProcessMotionService::stopMotion(QString* errorMessage)
{
    MotionControl* mc = motionControl(m_service, errorMessage);
    if (!mc)
        return false;
    const bool ok = mc->StopMotion() && mc->StopAllBuffer();
    if (!ok && errorMessage)
        *errorMessage = QObject::tr("停止运动失败");
    return ok;
}

LegacyProcessIoService::LegacyProcessIoService(Service* service)
    : m_service(service)
{
}

bool LegacyProcessIoService::setOutput(const QString& signalType,
                                       const QString& ioName,
                                       const QVariant& value,
                                       QString* errorMessage)
{
    MotionControl* mc = motionControl(m_service, errorMessage);
    if (!mc)
        return false;
    const bool digital = signalType.compare(QStringLiteral("digital"), Qt::CaseInsensitive) == 0;
    bool ok = false;
    if (digital) {
        if (auto e = ioEnumFromKey<DigitalOUT>(ioName)) {
            if (mc->m_mapDigitalOUT.count(e.value()))
                ok = mc->DigitalOutputSet(e.value(), value.toBool() ? 1 : 0);
            else if (errorMessage)
                *errorMessage = QObject::tr("数字量输出 %1 未注册").arg(ioName);
        }
    } else {
        if (auto e = ioEnumFromKey<AnalogOUT>(ioName)) {
            if (mc->m_mapAnalogOUT.count(e.value()))
                ok = mc->AnalogOutputSet(e.value(), value.toDouble());
            else if (errorMessage)
                *errorMessage = QObject::tr("模拟量输出 %1 未注册").arg(ioName);
        }
    }
    if (!ok && errorMessage && errorMessage->isEmpty())
        *errorMessage = QObject::tr("输出信号 %1 设置失败").arg(ioName);
    return ok;
}

bool LegacyProcessIoService::waitInput(const QString& signalType,
                                       const QString& ioName,
                                       const QVariant& targetValue,
                                       int timeoutMs,
                                       int pollIntervalMs,
                                       QString* errorMessage)
{
    MotionControl* mc = motionControl(m_service, errorMessage);
    if (!mc)
        return false;
    const bool analog = signalType.compare(QStringLiteral("analog"), Qt::CaseInsensitive) == 0;
    auto digitalEnum = analog ? std::optional<DigitalIN>{} : ioEnumFromKey<DigitalIN>(ioName);
    auto analogEnum = analog ? ioEnumFromKey<AnalogIN>(ioName) : std::optional<AnalogIN>{};
    if ((!analog && (!digitalEnum.has_value() || !mc->m_mapDigitalIN.count(digitalEnum.value())))
        || (analog && (!analogEnum.has_value() || !mc->m_mapAnalogIN.count(analogEnum.value())))) {
        if (errorMessage)
            *errorMessage = QObject::tr("输入信号 %1 未注册").arg(ioName);
        return false;
    }
    QElapsedTimer timer;
    timer.start();
    const int interval = std::clamp(pollIntervalMs, 10, 1000);
    while (timeoutMs <= 0 || timer.elapsed() <= timeoutMs) {
        if (analog) {
            double value = 0.0;
            if (mc->AnalogInputGet(analogEnum.value(), value)
                && std::abs(value - targetValue.toDouble()) < 1e-6)
                return true;
        } else {
            int value = 0;
            if (mc->DigitalInputGet(digitalEnum.value(), value)
                && (value != 0) == targetValue.toBool())
                return true;
        }
        QThread::msleep(static_cast<unsigned long>(interval));
    }
    if (errorMessage)
        *errorMessage = QObject::tr("等待输入 %1 超时").arg(ioName);
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
