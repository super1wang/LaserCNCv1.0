#pragma once

#include "modules/process/runtime/i_motion_command_sink.h"
#include "modules/process/runtime/axis_map.h"

#include <QString>
#include <QElapsedTimer>
#include <functional>
#include <string>

class ACSMotionControl;

namespace lcnc::process {

/**
 * @brief ACS / SimulatorCMHP 控制器的文本指令汇。
 *
 * 工作机理：
 *   1. resetProgram() 清空底层 m_strCommand（acs->ResetProgramCommand()）。
 *   2. 空程与切割都由 beginSegment / lineTo / endSegment 生成协调插补；激光与切割头
 *      控制仍转发到 ACSMotionControl。
 *   3. 分段方法由 sink 直接根据 AxisMap 生成
 *      "XSEG/VFJA (0,1,2,3,4)" / "LINE/V (...)" 文本，规避控制器内部硬编码的 (X,Y)。
 *   4. flush() 调 acs->SendCommand() = acsc_StopBuffer + LoadBuffer + CompileBuffer + RunBuffer + WaitEnd。
 */
class AcsTextCommandSink final : public IMotionCommandSink
{
public:
    using PositionObserver = std::function<void(const QString&, double)>;

    AcsTextCommandSink(ACSMotionControl* acs, AxisMap axisMap,
                       PositionObserver positionObserver = {});
    ~AcsTextCommandSink() override = default;

    QString id() const override;
    bool supportsBatchProgram() const override { return true; }
    void setCancellation(ProcessInterruptContext* token) override { m_token = token; }

    void resetProgram() override;
    bool startProgram(QString* errorMessage = nullptr) override;
    bool isProgramRunning(QString* errorMessage = nullptr) override;
    bool flush(QString* errorMessage = nullptr) override;

    bool executeRapidSegment(const lcnc::cam::RapidMoveSegment& segment,
                             const Tool& tool, QString* errorMessage = nullptr) override;
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
    /// 把 sink 拼出的 ACSPL+ 片段追加到底层 m_strCommand。
    /// 不能命名为 `emit` —— 那是 Qt 关键字（#define emit）。
    void appendText(const std::string& text);
    void publishControllerPositions();

    ACSMotionControl*  m_acs{nullptr};
    AxisMap            m_axisMap;
    PositionObserver   m_positionObserver;
    ProcessInterruptContext* m_token{nullptr};
    double m_lastR1{0.0};
    double m_lastR2{0.0};
    bool   m_hasLastR1{false};
    bool   m_hasLastR2{false};
    QElapsedTimer m_positionPublishTimer;
};

} // namespace lcnc::process
