#pragma once

#include <QString>

namespace lcnc::process {

enum class ProcessConfigArea { Devices, DigitalIo, AnalogIo, Operations, Workflow, Tools };

enum class ProcessIoBucket { DigitalInput, DigitalOutput, AnalogInput, AnalogOutput };

enum class ProcessInitialApproachMode { Manual = 0, Automatic };

struct ProcessInitialApproachSettings
{
    ProcessInitialApproachMode mode{ProcessInitialApproachMode::Automatic};
    double safetyZ{0.0};
    bool collisionCheckEnabled{false};
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
