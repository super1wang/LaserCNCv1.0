#pragma once

#include "modules/process/runtime/i_motion_command_sink.h"
#include "modules/process/runtime/axis_map.h"
#include "modules/process/runtime/gtn_exact_session.h"

#include <QString>

class GTNMotionControl;

namespace lcnc::process {
class GtnEncodedProgram;

/**
 * @brief GTN 控制器的缓存式指令汇。
 *
 * GTN 与 ACS 的"批量执行"语义一致，只是缓存机制不同：
 *   - ACS：拼成 ACSPL+ 文本 → acsc_LoadBuffer + acsc_RunBuffer 一次提交。
 *   - GTN legacy：GTN_BufXxx / GTN_LnXYZEx → GTN_CrdDataEx → GTN_CrdStart。
 *   - GTN Group：GTN_MoveLinearAbsolute / list IO → CommandListDataEnd →
 *                StartCommandList。RTCP 开关只改变 Group 的坐标描述，CAM
 *                求解轴坐标始终保留为独立输入。
 *
 * Group 模式由 sink 独占一个完整生命周期：beginSegment() 初始化并取得轴组，
 * lineTo / laserOn / laserOff 只构建 CommandList，startProgram() 提交并启动，
 * isProgramRunning() 在完成或故障后立即释放轴组。轴组未释放前禁止退回单轴点位模式。
 * Legacy 模式仍使用 GTN_BufXxx / GTN_LnXYZEx，并由 SendCommand() 一次提交。
 */
class GtnBufferedCommandSink final : public IMotionCommandSink
{
public:
    GtnBufferedCommandSink(GTNMotionControl* gtn, AxisMap axisMap);
    ~GtnBufferedCommandSink() override;

    QString id() const override { return QStringLiteral("GTN"); }
    bool supportsBatchProgram() const override { return true; }
    void setCancellation(ProcessInterruptContext* token) override { m_token = token; }

    bool prepareExactSection(const PreparedDeviceProgram&, int ordinal, QString* error) override;
    bool startExactSection(const PreparedDeviceProgram&, int ordinal, QString* error) override;
    bool continueExactPreparation(const PreparedDeviceProgram&, int, bool&, QString*) override;
    bool isExactSectionRunning(const PreparedDeviceProgram&, int, QString*) override;

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
    bool releaseActiveGroup(QString* errorMessage, const char* reason);

    GTNMotionControl* m_gtn{nullptr};
    AxisMap           m_axisMap;
    ProcessInterruptContext* m_token{nullptr};
    bool m_bufferCommandFailed{false};
    bool m_groupProgramActive{false};
    std::unique_ptr<GtnExactSession> m_exactSession;
};

} // namespace lcnc::process
