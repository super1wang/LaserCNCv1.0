#pragma once

#include "modules/process/runtime/i_motion_command_sink.h"
#include "modules/process/runtime/process_device_coordinator.h"

namespace lcnc::process {

/**
 * Serializes every real-controller sink call with ProcessDeviceRuntime reads.
 * Long waits deliberately release the lease between short start/status calls,
 * allowing the independent 150 ms APOS poller to publish live coordinates.
 */
class CoordinatedMotionCommandSink final : public IMotionCommandSink
{
public:
    CoordinatedMotionCommandSink(std::unique_ptr<IMotionCommandSink> inner,
                                 ProcessDeviceCoordinator& coordinator);

    QString id() const override;
    bool supportsBatchProgram() const override;
    void setCancellation(ProcessInterruptContext* token) override;
    void resetProgram() override;
    bool startProgram(QString* errorMessage = nullptr) override;
    bool isProgramRunning(QString* errorMessage = nullptr) override;
    bool flush(QString* errorMessage = nullptr) override;
    bool executeRapidSegment(const lcnc::cam::RapidMoveSegment& segment,
                             const Tool& tool,
                             QString* errorMessage = nullptr) override;
    void startCuttingHead(const Tool& tool) override;
    void stopCuttingHead() override;
    void setShutterTimings(double beforeOn, double afterOn,
                           double beforeOff, double afterOff,
                           double blowDelay) override;
    bool lineTo(const MachinePose5& target, const Tool& tool,
                QString* errorMessage = nullptr) override;
    bool beginSegment(const MachinePose5& startPose, const Tool& tool,
                      QString* errorMessage = nullptr) override;
    void endSegment(const Tool& tool) override;
    void laserOn(const Tool& tool) override;
    void laserOff(const Tool& tool) override;
    void endProgram(const Tool& tool) override;
    void applyToolMotionParams(const Tool& tool, bool jump) override;

private:
    std::unique_ptr<IMotionCommandSink> m_inner;
    ProcessDeviceCoordinator& m_coordinator;
    ProcessInterruptContext* m_token{nullptr};
};

} // namespace lcnc::process
