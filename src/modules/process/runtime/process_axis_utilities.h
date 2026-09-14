#pragma once

#include "core/kinematics/machine_configuration_service.h"

#include <QStringList>

namespace lcnc::process {

QStringList homeOrderForAxes(const QList<MachineAxisDef>& axes);
bool sameAxisDefinitions(const QList<MachineAxisDef>& lhs, const QList<MachineAxisDef>& rhs);
double simulatedAxisValue(const MachineAxisDef& axis, double phase, int linearIndex, int rotaryIndex);

} // namespace lcnc::process
