#pragma once

#include <QObject>
#include <QHash>
#include <QString>
#include <QStringList>

#include "core/kernel/i_service.h"

class MachineKinematics;

namespace lcnc {

/**
 * @brief 机台当前姿态（XYZ + 旋转轴角度）的进程级显式状态。
 *
 * 设计要点：
 *   - 持有一份"按轴名 → 当前位置/角度"的 QHash；轴集合由当前
 *     @ref MachineKinematics 的轴定义自适应（如 AC 转台 = X/Y/Z/A/C）；
 *   - 所有姿态变更（仿真 / jog / 控制器回报）统一经此对象的 setter，
 *     通过 @ref poseChanged 信号通知订阅者（CAM 渲染、UI 显示）；
 *   - 信号携带"本次发生变化的轴名集合"，便于下游做局部刷新；
 *   - 线程亲和：本对象只在 GUI 线程使用；硬件控制器若在子线程中
 *     更新，请用 Qt::QueuedConnection 转发到本对象的 setter。
 *
 * 该类位于 core/kinematics，零 Qt UI / 零 OCC 几何依赖；
 * 由 CamModule 持有并经 IService 注册到 Kernel，跨模块共享。
 */
class MachinePose : public QObject, public lcnc::IService
{
    Q_OBJECT
public:
    explicit MachinePose(QObject* parent = nullptr);
    ~MachinePose() override;

    /// 注入构型；本对象会按 kin 的轴定义重置支持的轴集合，
    /// 未列入的轴在 @ref setAxisValue 中会被拒绝并打 WARN。
    /// 传入 nullptr 表示"无构型"，setter 全拒绝。
    void setKinematics(MachineKinematics* kin);

    /// 当前持有的构型（弱引用，所有权在 LcncDocument）。
    MachineKinematics* kinematics() const { return m_kin; }

    /// 写入单轴值；改变才会 emit 信号。返回 true 表示值发生了变化。
    /// @param emitChanged false 时静默更新（用于批量后单次 emit 场景）。
    bool setAxisValue(const QString& axis, double value, bool emitChanged = true);

    /// 批量写入；在内部聚合 dirty 集合后单次 emit。
    void setAxisValuesBatch(const QHash<QString, double>& values);

    /// 读取单轴值；轴不存在返回 0 并打 DEBUG。
    double axisValue(const QString& axis) const;

    /// 当前支持的轴名（按 kinematics axes() 顺序）。
    QStringList supportedAxes() const { return m_supportedAxes; }

    /// 当前完整姿态快照。
    QHash<QString, double> snapshot() const { return m_values; }

signals:
    /// 姿态发生变化，参数为本次变化的轴名集合（已去重）。
    void poseChanged(const QStringList& dirtyAxes);

    /// 构型/支持轴集合变更（kinematics 切换、构型重置等）。
    void supportedAxesChanged(const QStringList& axes);

private:
    /// 重新从 m_kin 读取并重建 m_values + m_supportedAxes。
    void rebuildFromKinematics();

private:
    MachineKinematics*        m_kin{nullptr};
    QStringList               m_supportedAxes;
    QHash<QString, double>    m_values;
};

} // namespace lcnc
