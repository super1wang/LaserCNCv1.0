#include "modules/process/monitor/process_monitor_service.h"

#include <QFutureWatcher>
#include <QMetaObject>
#include <QPointer>
#include <QTimer>
#include <QtConcurrent>

#include <algorithm>

namespace lcnc::process {

namespace {

constexpr char kPressureMonitorId[] = "pressureMonitor";

bool readDigitalValue(const ProcessMonitorPollContext& context,
                      const QString& channel,
                      bool* value,
                      QString* errorMessage)
{
    if (!value)
        return false;

    Q_UNUSED(context);
    Q_UNUSED(channel);
    // Simulation mode: always return default value.
    *value = false;
    if (errorMessage)
        *errorMessage = QObject::tr("仿真模式：IO 不可用");
    return false;
}

bool readAnalogValue(const ProcessMonitorPollContext& context,
                     const QString& channel,
                     double* value,
                     QString* errorMessage)
{
    if (!value)
        return false;

    Q_UNUSED(context);
    Q_UNUSED(channel);
    // Simulation mode: always return default value.
    *value = 0.0;
    if (errorMessage)
        *errorMessage = QObject::tr("仿真模式：模拟量不可用");
    return false;
}

QString channelText(const QString& channel)
{
    return channel.trimmed().isEmpty() ? QObject::tr("未配置") : channel.trimmed();
}

ProcessMonitorStateItem makeDigitalState(const ProcessMonitorPollContext& context,
                                         const QString& id,
                                         const QString& title,
                                         const QString& channel,
                                         bool enabled,
                                         bool gateEnabled,
                                         const QString& activeDetail,
                                         const QString& inactiveDetail)
{
    ProcessMonitorStateItem item;
    item.id = id;
    item.title = title;
    item.channel = channelText(channel);
    item.enabled = enabled;
    item.digital = true;
    if (!enabled)
        return item;

    bool value = false;
    QString errorMessage;
    item.available = readDigitalValue(context, channel, &value, &errorMessage);
    item.boolValue = value;
    item.alarm = item.available && gateEnabled && value;
    if (!item.available) {
        item.detail = errorMessage.isEmpty() ? QObject::tr("信号不可用") : errorMessage;
    } else {
        item.detail = item.alarm ? activeDetail : inactiveDetail;
    }
    return item;
}

ProcessMonitorStateItem makeAnalogState(const ProcessMonitorPollContext& context,
                                        const QString& id,
                                        const QString& title,
                                        const QString& channel,
                                        bool enabled,
                                        int conversion,
                                        double threshold,
                                        const QString& unit)
{
    ProcessMonitorStateItem item;
    item.id = id;
    item.title = title;
    item.channel = channelText(channel);
    item.enabled = enabled;
    item.digital = false;
    item.unit = unit;
    item.thresholdValue = threshold;
    if (!enabled)
        return item;

    QString errorMessage;
    item.available = readAnalogValue(context, channel, &item.numericValue, &errorMessage);
    if (!item.available) {
        item.detail = errorMessage.isEmpty() ? QObject::tr("信号不可用") : errorMessage;
        return item;
    }

    item.displayValue = item.numericValue * static_cast<double>(conversion);
    item.alarm = item.displayValue < threshold;
    item.detail = QObject::tr("当前值 %1 %2，阈值 %3 %2")
        .arg(QString::number(item.displayValue, 'f', 3),
             unit,
             QString::number(threshold, 'f', 3));
    return item;
}

QString alarmMessageForState(const ProcessMonitorStateItem& state)
{
    if (state.digital)
        return QObject::tr("%1 异常").arg(state.title);
    return QObject::tr("%1 异常: %2").arg(state.title, state.detail);
}

ProcessMonitorSnapshot buildSnapshot(const ProcessMonitorPollContext& context)
{
    ProcessMonitorSnapshot snapshot;
    snapshot.monitoringEnabled = context.monitoringEnabled;
    snapshot.connected = context.connected;
    snapshot.simulationMode = context.simulationMode;
    snapshot.intervalMs = context.intervalMs;
    snapshot.faultAction = context.settings.faultAction;
    snapshot.axisPositions = context.cachedAxisPositions;

    for (const ProcessMonitorOutputChannel& output : context.outputChannels) {
        bool value = false;
        QString errorMessage;
        if (!readDigitalValue(context, output.channel, &value, &errorMessage) && !errorMessage.isEmpty()) {
            snapshot.warnings.append(QObject::tr("%1 读取失败: %2").arg(output.name, errorMessage));
        }
        snapshot.digitalOutputs.insert(output.name, value);
    }

    const bool blowOn = snapshot.digitalOutputs.value(QObject::tr("吹气"), false);
    snapshot.states.append(makeDigitalState(context,
                                            QStringLiteral("interlock"),
                                            QObject::tr("门禁"),
                                            context.interlockChannel,
                                            context.settings.interLockEnabled,
                                            true,
                                            QObject::tr("门禁已触发"),
                                            QObject::tr("门禁正常")));
    snapshot.states.append(makeDigitalState(context,
                                            QStringLiteral("safetyLightCurtain"),
                                            QObject::tr("安全光栅"),
                                            context.safetyLightCurtainChannel,
                                            context.settings.safetyLightCurtainEnabled,
                                            true,
                                            QObject::tr("安全光栅已触发"),
                                            QObject::tr("安全光栅正常")));
    snapshot.states.append(makeDigitalState(context,
                                            QString::fromLatin1(kPressureMonitorId),
                                            QObject::tr("气压监控"),
                                            context.pressureMonitorChannel,
                                            context.settings.pressureMonitorEnabled,
                                            blowOn,
                                            QObject::tr("吹气开启且气压异常"),
                                            blowOn ? QObject::tr("吹气开启，气压正常") : QObject::tr("吹气关闭，跳过判定")));
    snapshot.states.append(makeDigitalState(context,
                                            QStringLiteral("waterLeakage"),
                                            QObject::tr("漏水监控"),
                                            context.waterLeakageChannel,
                                            context.settings.waterLeakageMonitorEnabled,
                                            true,
                                            QObject::tr("漏水监控已触发"),
                                            QObject::tr("漏水监控正常")));
    snapshot.states.append(makeDigitalState(context,
                                            QStringLiteral("waterTank"),
                                            QObject::tr("水箱监控"),
                                            context.waterTankChannel,
                                            context.settings.waterTankMonitorEnabled,
                                            true,
                                            QObject::tr("水箱监控已触发"),
                                            QObject::tr("水箱监控正常")));
    snapshot.states.append(makeAnalogState(context,
                                           QStringLiteral("waterPressure"),
                                           QObject::tr("水压监控"),
                                           context.waterPressureChannel,
                                           context.settings.waterPressureMonitorEnabled,
                                           context.settings.waterPressureConversions,
                                           context.settings.waterPressureLimitMpa,
                                           QStringLiteral("MPa")));
    snapshot.states.append(makeAnalogState(context,
                                           QStringLiteral("waterLevel"),
                                           QObject::tr("水位监控"),
                                           context.waterLevelChannel,
                                           context.settings.waterLevelMonitorEnabled,
                                           context.settings.waterLevelConversions,
                                           context.settings.waterLevelLimitMm,
                                           QStringLiteral("mm")));
    return snapshot;
}

} // namespace

ProcessMonitorService::ProcessMonitorService(QObject* parent)
    : QObject(parent)
    , m_timer(new QTimer(this))
{
    qRegisterMetaType<ProcessMonitorSnapshot>();
    qRegisterMetaType<ProcessMonitorAlarm>();
    qRegisterMetaType<ProcessMonitorStateList>();
    qRegisterMetaType<ProcessMonitorAlarmList>();

    m_timer->setInterval(500);
    connect(m_timer, &QTimer::timeout, this, &ProcessMonitorService::pollAsync);
}

void ProcessMonitorService::setContextProvider(std::function<ProcessMonitorPollContext()> provider)
{
    m_contextProvider = std::move(provider);
}

void ProcessMonitorService::start()
{
    if (!m_timer->isActive())
        m_timer->start();
    requestPoll();
}

void ProcessMonitorService::stop()
{
    m_timer->stop();
    m_inFlight = false;
    m_triggerCounts.clear();
    m_activeAlarms.clear();
}

void ProcessMonitorService::requestPoll()
{
    QMetaObject::invokeMethod(this, [this] {
        pollAsync();
    }, Qt::QueuedConnection);
}

void ProcessMonitorService::pollAsync()
{
    if (m_inFlight || !m_contextProvider)
        return;

    const ProcessMonitorPollContext context = m_contextProvider();
    const int intervalMs = std::max(50, context.intervalMs);
    if (m_timer->interval() != intervalMs)
        m_timer->setInterval(intervalMs);

    m_inFlight = true;
    QPointer<ProcessMonitorService> self(this);
    auto* watcher = new QFutureWatcher<ProcessMonitorSnapshot>(this);
    connect(watcher, &QFutureWatcher<ProcessMonitorSnapshot>::finished, this, [this, self, watcher]() {
        const ProcessMonitorSnapshot result = watcher->result();
        watcher->deleteLater();
        m_inFlight = false;
        if (!self)
            return;

        ProcessMonitorSnapshot snapshot = result;
        applyAlarmState(snapshot);
        m_lastSnapshot = snapshot;
        emit snapshotUpdated(m_lastSnapshot);
    });
    watcher->setFuture(QtConcurrent::run([context]() {
        return buildSnapshot(context);
    }));
}

void ProcessMonitorService::applyAlarmState(ProcessMonitorSnapshot& snapshot)
{
    QHash<QString, ProcessMonitorAlarm> nextAlarms;
    ProcessMonitorAlarmList activeAlarms;

    for (const ProcessMonitorStateItem& state : snapshot.states) {
        const bool eligible = snapshot.monitoringEnabled && state.enabled && state.available && state.alarm;
        if (!eligible) {
            m_triggerCounts.remove(state.id);
            continue;
        }

        const int requiredCount = state.id == QString::fromLatin1(kPressureMonitorId)
            ? std::max(1, (500 + std::max(1, snapshot.intervalMs) - 1) / std::max(1, snapshot.intervalMs))
            : 1;
        const int count = m_triggerCounts.value(state.id, 0) + 1;
        m_triggerCounts.insert(state.id, count);
        if (count < requiredCount)
            continue;

        ProcessMonitorAlarm alarm;
        alarm.id = state.id;
        alarm.title = state.title;
        alarm.message = alarmMessageForState(state);
        alarm.action = snapshot.faultAction;
        alarm.active = true;
        nextAlarms.insert(alarm.id, alarm);
        activeAlarms.append(alarm);
    }

    for (auto it = nextAlarms.cbegin(); it != nextAlarms.cend(); ++it) {
        if (!m_activeAlarms.contains(it.key()))
            emit alarmRaised(it.value());
    }
    for (auto it = m_activeAlarms.cbegin(); it != m_activeAlarms.cend(); ++it) {
        if (!nextAlarms.contains(it.key()))
            emit alarmCleared(it.value());
    }

    m_activeAlarms = nextAlarms;
    snapshot.activeAlarms = activeAlarms;
    if (snapshot.activeAlarms.isEmpty()) {
        snapshot.summary = QObject::tr("监控正常");
    } else {
        QStringList messages;
        for (const ProcessMonitorAlarm& alarm : snapshot.activeAlarms)
            messages.append(alarm.message);
        snapshot.summary = messages.join(QObject::tr("；"));
    }
}

} // namespace lcnc::process
