#pragma once

#include <QObject>
#include <QString>

#include <functional>

class QTimer;
class LaserToolpath;

namespace lcnc::cam {

/**
 * @brief 刀路播放仿真器（v2.3 从 CamModule 抽出）。
 *
 * 持有播放状态机、计时器与速度，遍历 LaserToolpath 中启用的轮廓与点。
 * 每个 tick 通过回调把当前点的机床坐标 (x/y/z + 两个旋转轴) 写回外部
 * （通常是 MachineKinematics + GuiDocument 刷新）。
 *
 * 调用方持有该对象作为成员，在 init 时调用 setApplyAxisFn / setRefreshFn
 * 注入对外副作用，仿真器自身只负责状态推进与 Qt 信号。
 */
class ToolpathSimulator : public QObject {
    Q_OBJECT
public:
    /// 把机床坐标的 X/Y/Z + 两个旋转轴写回外部。refreshNow 由仿真器决定。
    using ApplyAxisFn = std::function<void(const QString& axis, double value, bool refreshNow)>;
    /// 写完所有轴后触发可视化刷新。
    using RefreshFn = std::function<void()>;

    explicit ToolpathSimulator(QObject* parent = nullptr);
    ~ToolpathSimulator() override;

    void setApplyAxisFn(ApplyAxisFn fn) { m_applyAxis = std::move(fn); }
    void setRefreshFn(RefreshFn fn) { m_refresh = std::move(fn); }

    /// 绑定刀路数据源（通常是 CamModule 内部 m_toolpath）。
    void setToolpath(const LaserToolpath* toolpath) { m_toolpath = toolpath; }

    void play();
    void pause();
    void stop();

    void setSpeed(double factor);
    bool isPlaying() const { return m_playing; }
    bool isPaused() const { return m_paused; }

signals:
    /// 当前进度。totalPoints 是所有启用轮廓的点数总和。
    void simulationTick(int contourIdx, int pointIdx, int totalPoints);
    /// 播放/暂停状态切换。playing=false 表示停止或暂停。
    void simulationStateChanged(bool playing);
    /// 全部轮廓播放完毕（stop() 的情况下也会触发一次）。
    void simulationFinished();

private slots:
    void onTick();

private:
    int currentInterval() const;

    const LaserToolpath* m_toolpath{nullptr};
    QTimer* m_timer{nullptr};
    ApplyAxisFn m_applyAxis;
    RefreshFn m_refresh;

    int     m_currentContour{0};
    int     m_currentPoint{0};
    int     m_totalPoints{0};
    double  m_speed{1.0};
    bool    m_playing{false};
    bool    m_paused{false};
};

} // namespace lcnc::cam
