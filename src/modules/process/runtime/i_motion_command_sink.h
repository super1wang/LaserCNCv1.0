#pragma once

/**
 * @file i_motion_command_sink.h
 * @brief 控制器无关的运动指令"汇"：编排层只产生语义化命令，由实现自决落地。
 *
 * 设计要点：
 *  - 编排层（NormalCuttingManager 等）不再 if-branch 控制器类型。
 *  - ACS sink 内部把指令拼成 ACSPL+ 文本，flush() 时一次性 Load+Run。
 *  - GTN sink 内部把指令累积到 GTN buffer（GTN_BufXxx / GTN_LnXYZEx 等），
 *    flush() 时调用 GTN_CrdStart 启动整段加工。
 *  - PureSimulation sink 仅驱动 PureSimulationToolpathTicker，无 IO 落地。
 *
 * 5 轴 / 3 轴 / 5 轴头由 AxisMap 在构造期决定，避免在每条 lineTo 里硬编码 (0,1)。
 */

#include "axis_map.h"
#include "core/project/cam/travel_plan_contracts.h"
#include "modules/process/tool/tool.h"
#include "modules/process/runtime/machine_pose5.h"
#include "modules/process/runtime/motion_params.h"
#include "modules/process/runtime/exact_section_execution.h"

#include <QString>
#include <functional>
#include <memory>

class MotionControl;

namespace lcnc::process {

class ProcessInterruptContext;
class PureSimulationToolpathTicker;
class PreparedDeviceProgram;

/// Narrow callbacks used by motion sinks to project state back to the UI.
/// They deliberately avoid exposing ProcessModule through runtime contracts.
struct MotionSinkCallbacks
{
    std::function<void(const QString&, double)> positionObserver;
    std::function<double()> feedOverrideProvider;
};

class IMotionCommandSink : public IExactSectionSink
{
public:
    virtual ~IMotionCommandSink() = default;

    /// 标识符，便于日志区分；不参与控制流。
    virtual QString id() const = 0;

    /// true=批量模型（ACS 文本 / GTN buffered），false=单步实时（暂时无此实现）。
    virtual bool supportsBatchProgram() const = 0;

    /// 注入中断令牌；sink 内部在长循环里轮询。
    virtual void setCancellation(ProcessInterruptContext* token) = 0;


    // —— 程序生命周期 ——
    virtual void resetProgram() = 0;
    /// 提交并启动：ACS=Load+Run；GTN=CrdData+CrdStart；不得在此等待完成。
    virtual bool startProgram(QString* errorMessage = nullptr) = 0;
    /// 短状态读取；true 表示控制器/仿真仍在执行。
    virtual bool isProgramRunning(QString* errorMessage = nullptr) = 0;
    /// 兼容入口：提交、启动并等待。新代码应使用 startProgram/isProgramRunning，
    /// 以便把等待轮询调度为可抢占的短设备任务。
    virtual bool flush(QString* errorMessage = nullptr) = 0;

    // —— 空程轨迹 ——
    /// Executes one CAM-planned rapid segment without changing its axis mask
    /// or ordering.  The laser must be off while this API is used.
    virtual bool executeRapidSegment(const lcnc::cam::RapidMoveSegment& segment,
                                     const Tool& tool,
                                     QString* errorMessage = nullptr) = 0;
    /// 启动 / 停止跟随头（仅切割头模式）。
    virtual void startCuttingHead(const Tool& tool) = 0;
    virtual void stopCuttingHead() = 0;
    /// 设置激光快门 5 个延时（开前/开后/关前/关后/吹气延时）。
    virtual void setShutterTimings(double beforeOn, double afterOn,
                                   double beforeOff, double afterOff,
                                   double blowDelay) = 0;

    // —— 切割主循环 ——
    /// 在 X/Y/Z/R1/R2 上插入一段直线（5 个机床坐标）。
    /// 哪些轴参与由 pose.mask 决定；轴索引由 sink 构造期注入的 AxisMap 解析。
    virtual bool lineTo(const MachinePose5& target, const Tool& tool,
                        QString* errorMessage = nullptr) = 0;

    /// 进入一段协调插补（XSEG/VFJA 或 GTN crd 开段）。
    virtual bool beginSegment(const MachinePose5& startPose, const Tool& tool,
                              QString* errorMessage = nullptr) = 0;
    /// 结束一段（ENDS / SPLIT / kill Z 等）。
    virtual void endSegment(const Tool& tool) = 0;

    // —— 激光 / 吹气 / IO ——
    /// 开激光：写吹气 ON + 等待 + 出光延时 + 激光 IO ON。
    virtual void laserOn(const Tool& tool) = 0;
    /// 关激光：写关光前等待 + 激光 IO OFF + 关光后等待（不写 ENDS/SPLIT，
    /// 段闭合由 endSegment 负责）。
    virtual void laserOff(const Tool& tool) = 0;

    // —— 程序末尾 ——
    /// 写 "kill Z" + 可选吹气 OFF + STOP。在 flush() 前调用。
    virtual void endProgram(const Tool& tool) = 0;

    // —— 工具变更：把工具的 acc/jerk 推到控制器侧（ACSPL+: ACCi/DECi/JERKi；
    //    GTN: GTN_PrfTrap 之类）。jump=true 时用空程参数，否则用切割参数。
    virtual void applyToolMotionParams(const Tool& tool, bool jump) = 0;
};

/// 工厂入口，按 MachineConfigurationService + ProcessSettings 选型。
class MotionSinkFactory
{
public:
    /// 当 mc==nullptr / 仿真模式开启时，返回 PureSimulationSink。
    /// 否则根据 mc->GetName() 选 AcsTextCommandSink 或 GtnBufferedCommandSink。
    /// callbacks only carry the UI projection callbacks needed by a sink.
    static std::unique_ptr<IMotionCommandSink> create(
        ::MotionControl* mc,
        bool simulationMode,
        PureSimulationToolpathTicker* simTicker,
        const MotionSinkCallbacks& callbacks,
        const lcnc::MachineAxisLayout& layout);
};

} // namespace lcnc::process
