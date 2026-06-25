#pragma once

#include "modules/cam/contracts/toolpath_export_dto.h"

#include <QObject>
#include <QVector>

class QTimer;
class ProcessModule;

namespace lcnc::process {

/**
 * @brief 纯仿真模式下沿真实刀路点驱动机床模型。
 *
 * 在 NormalCutting 运行期间，旁路 ProcessModule::onSimulationTick() 的
 * Lissajous 正弦波，由本类按 (feedRate * feedOverride / 60 mm/s) 的速度
 * 在连续两点之间插值，并把 (machineX, machineY, machineZ, machineR1,
 * machineR2) 喂入 ProcessModule::setAxisPosition() —— 进而经
 * MachinePose → MachineKinematics → AIS::SetLocalTransformation 路径
 * 让 3D 模型沿轨迹运动。
 *
 * 设计上不持有 QThread；定时器跑在创建者所在线程（即主线程）。
 * 调用方在外层循环里 QCoreApplication::processEvents() 抽水即可。
 */
class PureSimulationToolpathTicker : public QObject
{
    Q_OBJECT
public:
    explicit PureSimulationToolpathTicker(ProcessModule* processModule, QObject* parent = nullptr);
    ~PureSimulationToolpathTicker() override;

    /**
     * 开始一条轮廓的轨迹回放。会停掉上一条轨迹（若有）。
     * @param points     轮廓机床坐标点序列（来自 ToolpathExportPoint）
     * @param feedRate   工具进给速度 mm/s（来自 Tool::m_dLineVelocity）
     * @param feedOverride 全局倍率 (0..2)
     */
    void start(const QVector<lcnc::cam::ToolpathExportPoint>& points,
               double feedRate,
               double feedOverride);

    void pause();
    void resume();
    void stop();
    bool isRunning() const;
    bool isDone() const;

private slots:
    void onTick();

private:
    void emitPosition(const lcnc::cam::ToolpathExportPoint& p);

    ProcessModule* m_processModule{nullptr};
    QTimer* m_timer{nullptr};
    QVector<lcnc::cam::ToolpathExportPoint> m_points;
    int m_currentSegment{0};   // 0..m_points.size()-2
    double m_segmentProgress{0.0}; // 累计已走过的米长（mm）
    double m_speedMmPerSec{0.0};
    bool m_paused{false};
    bool m_done{true};
    qint64 m_lastTickMs{0};
};

} // namespace lcnc::process
