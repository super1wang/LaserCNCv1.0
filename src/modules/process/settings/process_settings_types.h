#pragma once

#include <QString>

namespace lcnc::process {

enum class ProcessConfigArea { Devices, DigitalIo, AnalogIo, Operations, Workflow, Tools };

enum class ProcessIoBucket { DigitalInput, DigitalOutput, AnalogInput, AnalogOutput };

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
