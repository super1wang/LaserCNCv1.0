#include "modules/process/controllers/simulation_motion_controller.h"

#include "core/kernel/kernel.h"
#include "core/kinematics/machine_pose.h"
#include "core/logging/logger.h"

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
    LCNC_WARN(LogCode::Generic,
              "SimulationMotionController: emergency stop activated");
}

} // namespace lcnc::process
