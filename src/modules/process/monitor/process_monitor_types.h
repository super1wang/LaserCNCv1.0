#pragma once

#include <QMetaType>
#include <QList>
#include <QMap>
#include <QString>
#include <QStringList>

namespace lcnc {

enum class ProcessMonitorFaultAction { Continue, Pause, Stop };

struct ProcessMonitorSettings {
    ProcessMonitorFaultAction faultAction{ProcessMonitorFaultAction::Pause};
    bool interLockEnabled{true};
    bool safetyLightCurtainEnabled{false};
    bool pressureMonitorEnabled{false};
    bool waterLeakageMonitorEnabled{false};
    bool waterTankMonitorEnabled{false};
    bool waterPressureMonitorEnabled{false};
    double waterPressureConversions{1};
    double waterPressureLimitMpa{1.6};
    bool waterLevelMonitorEnabled{false};
    double waterLevelConversions{1};
    double waterLevelLimitMm{10.0};
};

} // namespace lcnc

namespace lcnc::process {

struct ProcessMonitorStateItem
{
    QString id;
    QString title;
    QString channel;
    QString unit;
    bool enabled{false};
    bool available{false};
    bool alarm{false};
    bool digital{true};
    bool boolValue{false};
    double numericValue{0.0};
    double displayValue{0.0};
    double thresholdValue{0.0};
    QString detail;
};

struct ProcessMonitorAlarm
{
    QString id;
    QString title;
    QString message;
    lcnc::ProcessMonitorFaultAction action{lcnc::ProcessMonitorFaultAction::Pause};
    bool active{false};
};

using ProcessMonitorStateList = QList<ProcessMonitorStateItem>;
using ProcessMonitorAlarmList = QList<ProcessMonitorAlarm>;

struct ProcessMonitorSnapshot
{
    bool monitoringEnabled{false};
    bool connected{false};
    bool simulationMode{true};
    int intervalMs{500};
    lcnc::ProcessMonitorFaultAction faultAction{lcnc::ProcessMonitorFaultAction::Pause};
    QMap<QString, double> axisPositions;
    QMap<QString, bool> digitalOutputs;
    ProcessMonitorStateList states;
    ProcessMonitorAlarmList activeAlarms;
    QString summary;
    QStringList warnings;

    bool hasActiveAlarm() const { return !activeAlarms.isEmpty(); }
};

} // namespace lcnc::process

Q_DECLARE_METATYPE(lcnc::process::ProcessMonitorStateItem)
Q_DECLARE_METATYPE(lcnc::process::ProcessMonitorAlarm)
Q_DECLARE_METATYPE(lcnc::process::ProcessMonitorStateList)
Q_DECLARE_METATYPE(lcnc::process::ProcessMonitorAlarmList)
Q_DECLARE_METATYPE(lcnc::process::ProcessMonitorSnapshot)
