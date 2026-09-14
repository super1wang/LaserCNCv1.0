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

    if (context.readDigital)
        return context.readDigital(channel, value, errorMessage);

    Q_UNUSED(channel);
    *value = false;
    if (errorMessage)
        *errorMessage = context.simulationMode
            // 中文翻译：仿真模式：IO 不可用
            ? QObject::tr("Emulation mode: IO not available")
            // 中文翻译：数字量读取器未配置
            : QObject::tr("Digital reader not configured");
    return false;
}

bool readAnalogValue(const ProcessMonitorPollContext& context,
                     const QString& channel,
                     double* value,
                     QString* errorMessage)
{
    if (!value)
        return false;

    if (context.readAnalog)
        return context.readAnalog(channel, value, errorMessage);

    Q_UNUSED(channel);
    *value = 0.0;
    if (errorMessage)
        *errorMessage = context.simulationMode
            // 中文翻译：仿真模式：模拟量不可用
            ? QObject::tr("Simulation mode: Analog values are not available")
            // 中文翻译：模拟量读取器未配置
            : QObject::tr("Analog reader not configured");
    return false;
}

QString channelText(const QString& channel)
{
    // 中文翻译：未配置
    return channel.trimmed().isEmpty() ? QObject::tr("Not configured") : channel.trimmed();
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
        // 中文翻译：信号不可用
        item.detail = errorMessage.isEmpty() ? QObject::tr("Signal not available") : errorMessage;
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
        // 中文翻译：信号不可用
        item.detail = errorMessage.isEmpty() ? QObject::tr("Signal not available") : errorMessage;
        return item;
    }

    item.displayValue = item.numericValue * static_cast<double>(conversion);
    item.alarm = item.displayValue < threshold;
    // 中文翻译：当前值 %1 %2，阈值 %3 %2
    item.detail = QObject::tr("Current value %1 %2, threshold %3 %2")
        .arg(QString::number(item.displayValue, 'f', 3),
             unit,
             QString::number(threshold, 'f', 3));
    return item;
}

QString alarmMessageForState(const ProcessMonitorStateItem& state)
{
    if (state.digital)
        // 中文翻译：%1 异常
        return QObject::tr("%1 exception").arg(state.title);
    // 中文翻译：%1 异常: %2
    return QObject::tr("%1 Exception: %2").arg(state.title, state.detail);
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
            // 中文翻译：%1 读取失败: %2
            snapshot.warnings.append(QObject::tr("%1 read failed: %2").arg(output.name, errorMessage));
        }
        snapshot.digitalOutputs.insert(output.name, value);
    }

    // 中文翻译：吹气
    const bool blowOn = snapshot.digitalOutputs.value(QObject::tr("blow air"), false);
    snapshot.states.append(makeDigitalState(context,
                                            QStringLiteral("interlock"),
                                            // 中文翻译：门禁
                                            QObject::tr("access control"),
                                            context.interlockChannel,
                                            context.settings.interLockEnabled,
                                            true,
                                            // 中文翻译：门禁已触发
                                            QObject::tr("Access control has been triggered"),
                                            // 中文翻译：门禁正常
                                            QObject::tr("Access control is normal")));
    snapshot.states.append(makeDigitalState(context,
                                            QStringLiteral("safetyLightCurtain"),
                                            // 中文翻译：安全光栅
                                            QObject::tr("Safety grating"),
                                            context.safetyLightCurtainChannel,
                                            context.settings.safetyLightCurtainEnabled,
                                            true,
                                            // 中文翻译：安全光栅已触发
                                            QObject::tr("Safety light barrier triggered"),
                                            // 中文翻译：安全光栅正常
                                            QObject::tr("Safety grating normal")));
    snapshot.states.append(makeDigitalState(context,
                                            QString::fromLatin1(kPressureMonitorId),
                                            // 中文翻译：气压监控
                                            QObject::tr("Air pressure monitoring"),
                                            context.pressureMonitorChannel,
                                            context.settings.pressureMonitorEnabled,
                                            blowOn,
                                            // 中文翻译：吹气开启且气压异常
                                            QObject::tr("Blowing is on and the air pressure is abnormal"),
                                            // 中文翻译：吹气开启，气压正常；吹气关闭，跳过判定
                                            blowOn ? QObject::tr("Blowing is on and the air pressure is normal") : QObject::tr("Blowing off, skipping judgment")));
    snapshot.states.append(makeDigitalState(context,
                                            QStringLiteral("waterLeakage"),
                                            // 中文翻译：漏水监控
                                            QObject::tr("Water leakage monitoring"),
                                            context.waterLeakageChannel,
                                            context.settings.waterLeakageMonitorEnabled,
                                            true,
                                            // 中文翻译：漏水监控已触发
                                            QObject::tr("Water leakage monitoring has been triggered"),
                                            // 中文翻译：漏水监控正常
                                            QObject::tr("Water leakage monitoring is normal")));
    snapshot.states.append(makeDigitalState(context,
                                            QStringLiteral("waterTank"),
                                            // 中文翻译：水箱监控
                                            QObject::tr("Water tank monitoring"),
                                            context.waterTankChannel,
                                            context.settings.waterTankMonitorEnabled,
                                            true,
                                            // 中文翻译：水箱监控已触发
                                            QObject::tr("Water tank monitoring has been triggered"),
                                            // 中文翻译：水箱监控正常
                                            QObject::tr("Water tank monitoring is normal")));
    snapshot.states.append(makeAnalogState(context,
                                           QStringLiteral("waterPressure"),
                                           // 中文翻译：水压监控
                                           QObject::tr("water pressure monitoring"),
                                           context.waterPressureChannel,
                                           context.settings.waterPressureMonitorEnabled,
                                           context.settings.waterPressureConversions,
                                           context.settings.waterPressureLimitMpa,
                                           QStringLiteral("MPa")));
    snapshot.states.append(makeAnalogState(context,
                                           QStringLiteral("waterLevel"),
                                           // 中文翻译：水位监控
                                           QObject::tr("water level monitoring"),
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
    m_pollPool.setMaxThreadCount(1);
    m_pollPool.setExpiryTimeout(-1);
    connect(m_timer, &QTimer::timeout, this, &ProcessMonitorService::pollAsync);
}

ProcessMonitorService::~ProcessMonitorService()
{
    stop();
}

void ProcessMonitorService::setContextProvider(std::function<ProcessMonitorPollContext()> provider)
{
    m_contextProvider = std::move(provider);
}

void ProcessMonitorService::start()
{
    m_active = true;
    if (!m_timer->isActive())
        m_timer->start();
    requestPoll();
}

void ProcessMonitorService::stop()
{
    m_active = false;
    m_timer->stop();
    if (m_watcher) {
        m_watcher->waitForFinished();
        disconnect(m_watcher, nullptr, this, nullptr);
        delete m_watcher;
        m_watcher = nullptr;
    }
    m_inFlight = false;
    m_triggerCounts.clear();
    m_activeAlarms.clear();
}

void ProcessMonitorService::requestPoll()
{
    QMetaObject::invokeMethod(this, [this] {
        if (m_active)
            pollAsync();
    }, Qt::QueuedConnection);
}

void ProcessMonitorService::pollAsync()
{
    if (!m_active || m_inFlight || !m_contextProvider)
        return;

    const ProcessMonitorPollContext context = m_contextProvider();
    const int intervalMs = std::max(50, context.intervalMs);
    if (m_timer->interval() != intervalMs)
        m_timer->setInterval(intervalMs);

    m_inFlight = true;
    QPointer<ProcessMonitorService> self(this);
    auto* watcher = new QFutureWatcher<ProcessMonitorSnapshot>(this);
    m_watcher = watcher;
    connect(watcher, &QFutureWatcher<ProcessMonitorSnapshot>::finished, this, [this, self, watcher]() {
        const ProcessMonitorSnapshot result = watcher->result();
        if (m_watcher == watcher)
            m_watcher = nullptr;
        watcher->deleteLater();
        m_inFlight = false;
        if (!self || !m_active)
            return;

        ProcessMonitorSnapshot snapshot = result;
        applyAlarmState(snapshot);
        m_lastSnapshot = snapshot;
        emit snapshotUpdated(m_lastSnapshot);
    });
    watcher->setFuture(QtConcurrent::run(&m_pollPool, [context]() {
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
        // 中文翻译：监控正常
        snapshot.summary = QObject::tr("Monitoring is normal");
    } else {
        QStringList messages;
        for (const ProcessMonitorAlarm& alarm : snapshot.activeAlarms)
            messages.append(alarm.message);
        snapshot.summary = messages.join(QObject::tr("；"));
    }
}

} // namespace lcnc::process
