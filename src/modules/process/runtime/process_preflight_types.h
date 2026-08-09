#pragma once

#include <QMap>
#include <QString>
#include <QStringList>
#include <QVector>

namespace lcnc::process {

struct ProcessPreflightDigitalGuard {
    bool enabled{false};
    QString title;
    QString channel;
};

struct ProcessPreflightAnalogGuard {
    bool enabled{false};
    QString title;
    QString channel;
    double threshold{0.0};
    QString unit;
};

struct ProcessPreflightRequest {
    QStringList axisNames;
    QMap<QString, double> lockedAxisTargets;
    double lockedAxisMoveVelocity{5.0};
    QVector<ProcessPreflightDigitalGuard> digitalGuards;
    QVector<ProcessPreflightAnalogGuard> analogGuards;
};

struct ProcessPreflightReport {
    QMap<QString, double> axisPositions;
    QMap<QString, bool> axisEnabled;
};

} // namespace lcnc::process
