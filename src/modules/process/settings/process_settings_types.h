#pragma once

#include "modules/process/runtime/process_axis_types.h"

#include <QString>
#include <QVector>

namespace lcnc::process {

enum class ProcessConfigArea { Devices, DigitalIo, AnalogIo, Operations, Workflow, Tools };

enum class ProcessIoBucket { DigitalInput, DigitalOutput, AnalogInput, AnalogOutput };

enum class ProcessInitialApproachMode { Manual = 0, Automatic };

struct ProcessInitialApproachSettings
{
    ProcessInitialApproachMode mode{ProcessInitialApproachMode::Automatic};
    double safetyZ{0.0};
};

struct ProcessIoChannel
{
    QString id;
    QString name;
    QString hardwareIndex;
    bool activeHigh{true};
    bool enabled{true};
    bool showInMain{false};
    bool builtin{false};
};

} // namespace lcnc::process
