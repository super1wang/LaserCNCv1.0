#pragma once

#include "modules/process/monitor/process_monitor_types.h"

#include <QHash>
#include <QObject>

#include <functional>

class QTimer;

namespace lcnc::process {

struct ProcessMonitorOutputChannel
{
    QString name;
    QString channel;
};

struct ProcessMonitorPollContext
{
    bool connected{false};
    bool simulationMode{true};
    bool monitoringEnabled{true};
    int intervalMs{500};
    lcnc::ProcessMonitorSettings settings;
    QMap<QString, double> cachedAxisPositions;
    QList<ProcessMonitorOutputChannel> outputChannels;
    QString interlockChannel;
    QString safetyLightCurtainChannel;
    QString pressureMonitorChannel;
    QString waterLeakageChannel;
    QString waterTankChannel;
    QString waterPressureChannel;
    QString waterLevelChannel;
};

class ProcessMonitorService : public QObject
{
    Q_OBJECT
public:
    explicit ProcessMonitorService(QObject* parent = nullptr);

    void setContextProvider(std::function<ProcessMonitorPollContext()> provider);
    void start();
    void stop();
    void requestPoll();

    const ProcessMonitorSnapshot& lastSnapshot() const { return m_lastSnapshot; }

signals:
    void snapshotUpdated(const lcnc::process::ProcessMonitorSnapshot& snapshot);
    void alarmRaised(const lcnc::process::ProcessMonitorAlarm& alarm);
    void alarmCleared(const lcnc::process::ProcessMonitorAlarm& alarm);

private:
    void pollAsync();
    void applyAlarmState(ProcessMonitorSnapshot& snapshot);

    std::function<ProcessMonitorPollContext()> m_contextProvider;
    QTimer* m_timer{nullptr};
    bool m_inFlight{false};
    ProcessMonitorSnapshot m_lastSnapshot;
    QHash<QString, int> m_triggerCounts;
    QHash<QString, ProcessMonitorAlarm> m_activeAlarms;
};

} // namespace lcnc::process
