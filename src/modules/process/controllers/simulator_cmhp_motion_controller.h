#pragma once

#include "modules/process/controllers/simulation_motion_controller.h"

namespace lcnc::process {

class SimulatorCmhpMotionController final : public SimulationMotionController
{
public:
    explicit SimulatorCmhpMotionController(QObject* parent = nullptr)
        : SimulationMotionController(parent)
    {
    }

    QString id() const override { return QStringLiteral("SimulatorCMHP"); }
};

} // namespace lcnc::process