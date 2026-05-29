#pragma once

#include "modules/process/runtime/process_execution_context.h"

#include <QObject>
#include <QString>

namespace lcnc { class IMotionController; }
namespace lcnc::process { class ProcessDeviceManager; }

namespace lcnc::process {

/**
 * @brief Coordinates active Process devices behind motion, laser and IO APIs.
 */
class ProcessDeviceCoordinator : public QObject
{
    Q_OBJECT
public:
    explicit ProcessDeviceCoordinator(ProcessDeviceManager& deviceManager, QObject* parent = nullptr);

    void setMotionController(lcnc::IMotionController* controller);
    const ProcessExecutionContext& context() const { return m_context; }

    bool moveAxisTo(const QString& axisName, double position, QString* errorMessage = nullptr);
    bool setLaserEnergy(double value, QString* errorMessage = nullptr);
    bool setDigitalOutput(const QString& channel, bool value, QString* errorMessage = nullptr);
    void emergencyStop();

signals:
    void axisPositionUpdated(const QString& axisName, double value);
    void laserEnergyUpdated(double value);
    void digitalOutputUpdated(const QString& channel, bool value);
    void deviceError(const QString& message);

private:
    ProcessDeviceManager& m_deviceManager;
    lcnc::IMotionController* m_motionController{nullptr};
    ProcessExecutionContext m_context;
};

} // namespace lcnc::process