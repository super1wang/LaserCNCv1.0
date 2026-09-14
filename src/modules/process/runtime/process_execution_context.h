#pragma once

#include <QMap>
#include <QString>
#include <QVariant>

namespace lcnc::process {

/**
 * @brief Runtime snapshot shared by state, device and execution services.
 */
struct ProcessExecutionContext
{
    QMap<QString, double> axisPositions;
    QMap<QString, bool> digitalOutputs;
    QMap<QString, double> analogOutputs;
    double laserEnergy{0.0};
    double laserFrequency{0.0};
    double laserPulseWidth{0.0};
    double feedOverride{1.0};
    QString currentNodeId;
    QString activeMotionController;
    QString activeLaserDevice;
    QVariant userData;
};

} // namespace lcnc::process