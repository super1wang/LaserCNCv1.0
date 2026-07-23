#pragma once

#include "modules/process/runtime/i_motion_command_sink.h"
#include "modules/process/runtime/axis_map.h"

#include <QString>

class GTNMotionControl;

namespace lcnc::process {

/**
 * @brief GTN 控制器的缓存式指令汇。
 *
 * GTN 与 ACS 的"批量执行"语义一致，只是缓存机制不同：
 *   - ACS：拼成 ACSPL+ 文本 → acsc_LoadBuffer + acsc_RunBuffer 一次提交。
 *   - GTN：每次 GTN_BufXxx / GTN_LnXYZEx 入控制器的运动 FIFO → GTN_CrdDataEx 提交 →
 *          GTN_CrdStart 启动整段插补一次性执行。
 *
 * 因此 sink 在 lineTo / laserOn / laserOff 阶段 **只** 调用底层 OffsetLineTo /
 * ProLaserControl（这些方法已经是 buffered，写入 GTN_BufXxx FIFO）；mid-stream **绝不**
 * 调用 SendCommand。所有"提交+启动"动作都集中在 flush()：
 *   1. mc->SendCommand()    → GTN_CrdDataEx + GTN_CrdStart 一次下发整段。
 *   2. mc->PrfTrapAxis()    → 等待全部插补完成（与 ACS 的 acsc_WaitProgramEnd 对应）。
 */
class GtnBufferedCommandSink final : public IMotionCommandSink
{
public:
    GtnBufferedCommandSink(GTNMotionControl* gtn, AxisMap axisMap);
    ~GtnBufferedCommandSink() override = default;

    QString id() const override { return QStringLiteral("GTN"); }
    bool supportsBatchProgram() const override { return true; }
    void setCancellation(ProcessInterruptContext* token) override { m_token = token; }

    void resetProgram() override;
    bool startProgram(QString* errorMessage = nullptr) override;
    bool isProgramRunning(QString* errorMessage = nullptr) override;
    bool flush(QString* errorMessage = nullptr) override;

    void jumpToIdleZ(const MachinePose5& pose, const Tool& tool) override;
    void jumpToPose(const MachinePose5& pose, const Tool& tool) override;
    void jumpToCuttingZ(const MachinePose5& pose, const Tool& tool) override;
    void startCuttingHead(const Tool& tool) override;
    void stopCuttingHead() override;
    void setShutterTimings(double beforeOn, double afterOn,
                           double beforeOff, double afterOff,
                           double blowDelay) override;

    void lineTo(const MachinePose5& target, const Tool& tool) override;
    void beginSegment(const MachinePose5& startPose, const Tool& tool) override;
    void endSegment(const Tool& tool) override;

    void laserOn(const Tool& tool) override;
    void laserOff(const Tool& tool) override;
    void endProgram(const Tool& tool) override;

    void applyToolMotionParams(const Tool& tool, bool jump) override;

private:
    GTNMotionControl* m_gtn{nullptr};
    AxisMap           m_axisMap;
    ProcessInterruptContext* m_token{nullptr};
};

} // namespace lcnc::process
