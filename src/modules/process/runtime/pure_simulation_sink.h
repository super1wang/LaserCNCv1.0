#pragma once

#include "modules/cam/contracts/toolpath_export_dto.h"
#include "modules/process/runtime/i_motion_command_sink.h"
#include "modules/process/runtime/axis_map.h"

#include <QString>
#include <QVector>

class ProcessModule;

namespace lcnc::process {

class PureSimulationToolpathTicker;

/**
 * @brief 仿真模式下的指令汇 —— 收集点序列然后驱动 PureSimulationToolpathTicker 回放。
 *
 * 与 ACS/GTN sink 的差别：
 *  - 不调用任何控制器；所有运动等价为视图层模型 transform。
 *  - 激光/吹气 IO 写日志，不操作真实 IO（仿真环境本来就没有）。
 *  - feed 速度来自 Tool::m_dLineVelocity；feedOverride 来自 ProcessModule。
 */
class PureSimulationSink final : public IMotionCommandSink
{
public:
    PureSimulationSink(PureSimulationToolpathTicker* ticker,
                       ProcessModule* processModule,
                       AxisMap axisMap);
    ~PureSimulationSink() override = default;

    QString id() const override { return QStringLiteral("PureSimulation"); }
    bool supportsBatchProgram() const override { return true; }
    void setCancellation(ProcessInterruptContext* token) override { m_token = token; }

    void resetProgram() override;
    bool flush(QString* errorMessage = nullptr) override;

    void jumpToIdleZ(const Tool& tool) override;
    void jumpToXY(double x, double y, const Tool& tool) override;
    void jumpToPose(const MachinePose5& pose, const Tool& tool) override;
    void jumpToCuttingZ(const Tool& tool) override;
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
    PureSimulationToolpathTicker* m_ticker{nullptr};
    ProcessModule*                m_processModule{nullptr};
    AxisMap                       m_axisMap;
    ProcessInterruptContext*      m_token{nullptr};

    QVector<lcnc::cam::ToolpathExportPoint> m_pending;
    double m_feedRate{600.0};
};

} // namespace lcnc::process
