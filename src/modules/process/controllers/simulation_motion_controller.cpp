#include "modules/process/controllers/simulation_motion_controller.h"

#include "core/kernel/kernel.h"
#include "core/kinematics/machine_pose.h"
#include "core/logging/logger.h"

#include <cmath>
#include <QMutexLocker>

namespace lcnc::process {

SimulationMotionController::SimulationMotionController(QObject* parent)
    : QObject(parent)
{
    LCNC_DEBUG(LogCode::Generic, "SimulationMotionController ctor");
}

SimulationMotionController::~SimulationMotionController()
{
    LCNC_DEBUG(LogCode::Generic, "SimulationMotionController dtor");
}

lcnc::MachinePose* SimulationMotionController::pose() const
{
    if (m_poseCache)
        return m_poseCache;
    auto* k = lcnc::Kernel::tryCurrent();
    m_poseCache = k ? k->service<lcnc::MachinePose>() : nullptr;
    if (!m_poseCache) {
        LCNC_WARN(LogCode::Generic,
                  "SimulationMotionController: MachinePose service not registered");
    }
    return m_poseCache;
}

bool SimulationMotionController::start()
{
    LCNC_DEBUG(LogCode::Generic, "SimulationMotionController::start");
    m_running = true;
    m_estop = false;
    LCNC_INFO(LogCode::Generic, "SimulationMotionController started");
    return true;
}

void SimulationMotionController::stop()
{
    LCNC_DEBUG(LogCode::Generic, "SimulationMotionController::stop");
    m_running = false;
    m_runningPrograms.clear();
    m_pausedPrograms.clear();
    LCNC_INFO(LogCode::Generic, "SimulationMotionController stopped");
}

bool SimulationMotionController::jog(const QString& axis, double delta)
{
    LCNC_DEBUG(LogCode::Generic,
               "SimulationMotionController::jog axis={} delta={}",
               axis.toStdString(), delta);
    if (m_estop) {
        LCNC_WARN(LogCode::Generic,
                  "SimulationMotionController::jog blocked (estop)");
        return false;
    }
    if (!axisEnabled(axis)) {
        LCNC_WARN(LogCode::Generic,
                  "SimulationMotionController::jog blocked (axis disabled): {}",
                  axis.toStdString());
        return false;
    }
    auto* p = pose();
    if (!p)
        return false;
    return p->setAxisValue(axis, p->axisValue(axis) + delta);
}

bool SimulationMotionController::moveTo(const QString& axis, double absolutePos)
{
    LCNC_DEBUG(LogCode::Generic,
               "SimulationMotionController::moveTo axis={} pos={}",
               axis.toStdString(), absolutePos);
    if (m_estop) {
        LCNC_WARN(LogCode::Generic,
                  "SimulationMotionController::moveTo blocked (estop)");
        return false;
    }
    if (!axisEnabled(axis)) {
        LCNC_WARN(LogCode::Generic,
                  "SimulationMotionController::moveTo blocked (axis disabled): {}",
                  axis.toStdString());
        return false;
    }
    auto* p = pose();
    if (!p)
        return false;
    return p->setAxisValue(axis, absolutePos);
}

bool SimulationMotionController::home(const QString& axis)
{
    LCNC_DEBUG(LogCode::Generic,
               "SimulationMotionController::home axis='{}'", axis.toStdString());
    if (m_estop) {
        LCNC_WARN(LogCode::Generic,
                  "SimulationMotionController::home blocked (estop)");
        return false;
    }
    auto* p = pose();
    if (!p)
        return false;
    if (axis.isEmpty()) {
        QHash<QString, double> zeros;
        for (const QString& a : p->supportedAxes())
            zeros.insert(a, 0.0);
        p->setAxisValuesBatch(zeros);
        LCNC_INFO(LogCode::Generic,
                  "SimulationMotionController homed all axes ({})",
                  zeros.size());
        return true;
    }
    return p->setAxisValue(axis, 0.0);
}

void SimulationMotionController::emergencyStop()
{
    LCNC_DEBUG(LogCode::Generic, "SimulationMotionController::emergencyStop");
    m_estop = true;
    m_runningPrograms.clear();
    m_pausedPrograms.clear();
    LCNC_WARN(LogCode::Generic,
              "SimulationMotionController: emergency stop activated");
}

QMap<QString, double> SimulationMotionController::axisPositions() const
{
    QMap<QString, double> values;
    auto* p = pose();
    if (!p)
        return values;
    for (const QString& axis : p->supportedAxes())
        values.insert(axis, p->axisValue(axis));
    return values;
}

bool SimulationMotionController::setAxisEnabled(const QString& axis, bool enabled)
{
    const QString key = axis.trimmed().toUpper();
    if (key.isEmpty())
        return false;
    if (enabled)
        m_disabledAxes.remove(key);
    else
        m_disabledAxes.insert(key);
    LCNC_INFO(LogCode::Generic,
              "SimulationMotionController axis '{}' enabled={}",
              key.toStdString(), enabled);
    return true;
}

bool SimulationMotionController::axisEnabled(const QString& axis) const
{
    const QString key = axis.trimmed().toUpper();
    return key.isEmpty() || !m_disabledAxes.contains(key);
}

bool SimulationMotionController::axisHomed(const QString& axis) const
{
    auto* p = pose();
    if (!p)
        return false;
    return std::abs(p->axisValue(axis.trimmed().toUpper())) < 1e-6;
}

bool SimulationMotionController::setDigitalOutput(const QString& channel, bool value, QString* errorMessage)
{
    Q_UNUSED(errorMessage);
    QMutexLocker locker(&m_ioMutex);
    m_digitalValues.insert(channel.trimmed(), value);
    return true;
}

bool SimulationMotionController::digitalInput(const QString& channel, bool* value, QString* errorMessage) const
{
    Q_UNUSED(errorMessage);
    if (!value)
        return false;
    QMutexLocker locker(&m_ioMutex);
    *value = m_digitalValues.value(channel.trimmed(), false);
    return true;
}

bool SimulationMotionController::setAnalogOutput(const QString& channel, double value, QString* errorMessage)
{
    Q_UNUSED(errorMessage);
    QMutexLocker locker(&m_ioMutex);
    m_analogValues.insert(channel.trimmed(), value);
    return true;
}

bool SimulationMotionController::analogInput(const QString& channel, double* value, QString* errorMessage) const
{
    Q_UNUSED(errorMessage);
    if (!value)
        return false;
    QMutexLocker locker(&m_ioMutex);
    *value = m_analogValues.value(channel.trimmed(), 0.0);
    return true;
}

bool SimulationMotionController::executeProgram(const QString& program,
                                                int bufferIndex,
                                                bool waitForFinish,
                                                int timeoutMs,
                                                QString* errorMessage)
{
    Q_UNUSED(waitForFinish);
    Q_UNUSED(timeoutMs);
    Q_UNUSED(errorMessage);
    m_loadedPrograms.insert(bufferIndex, program);
    m_runningPrograms.insert(bufferIndex);
    m_pausedPrograms.remove(bufferIndex);
    LCNC_INFO(LogCode::Generic,
              "SimulationMotionController program buffer={} lines={}",
              bufferIndex,
              program.count(QChar('\n')) + 1);
    return true;
}

bool SimulationMotionController::programRunning(int bufferIndex, bool* running, QString* errorMessage) const
{
    Q_UNUSED(errorMessage);
    if (running)
        *running = m_runningPrograms.contains(bufferIndex) && !m_pausedPrograms.contains(bufferIndex);
    return true;
}

bool SimulationMotionController::pauseProgram(int bufferIndex, QString* errorMessage)
{
    Q_UNUSED(errorMessage);
    if (!m_runningPrograms.contains(bufferIndex))
        return true;
    m_pausedPrograms.insert(bufferIndex);
    return true;
}

bool SimulationMotionController::resumeProgram(int bufferIndex, QString* errorMessage)
{
    Q_UNUSED(errorMessage);
    if (!m_runningPrograms.contains(bufferIndex))
        return true;
    m_pausedPrograms.remove(bufferIndex);
    return true;
}

} // namespace lcnc::process
