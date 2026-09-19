#pragma once

#include "modules/process/runtime/prepared_device_program.h"
#include "modules/process/runtime/device_command_queue.h"

namespace lcnc::process {
class ProcessInterruptContext;

// All three calls execute on DeviceCommandQueue. Prepare encodes/seals ONLY
// the indexed section. Start cannot enqueue or run any later section. Poll
// reports completion only after this section has drained and outputs are safe
// at its contour boundary. No legacy startProgram adapter is supplied.
class IExactSectionSink {
public:
    virtual ~IExactSectionSink() = default;
    virtual bool prepareExactSection(const PreparedDeviceProgram&, int, QString* error)
    { return unavailable(error); }
    virtual bool startExactSection(const PreparedDeviceProgram&, int, QString* error)
    { return unavailable(error); }
    virtual bool continueExactPreparation(const PreparedDeviceProgram&, int, bool& complete, QString*)
    { complete = true; return true; }
    virtual bool isExactSectionRunning(const PreparedDeviceProgram&, int, QString* error)
    { return unavailable(error); }
private:
    static bool unavailable(QString* error) {
        if (error) *error = QStringLiteral("Exact-section lowering is unavailable for this controller");
        return false;
    }
};

// Workflow-thread orchestration, also exercised with a fake section sink.
// Owner retains sink through completion and performs safe stop/teardown on
// every failure. Same live invocation resumes in place; re-entry is rejected.
bool executePreparedSections(const PreparedDeviceProgram& program, IExactSectionSink& sink,
    DeviceCommandQueue& queue, ProcessInterruptContext& interrupt, const QString& nodeId,
    const std::function<DeviceCommandResult()>& boundaryHealth, QString* error);
} // namespace lcnc::process
