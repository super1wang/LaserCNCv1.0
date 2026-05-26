#pragma once

#include "core/settings/toml_config.h"

#include <QString>

namespace lcnc {

/**
 * @brief Process（激光加工执行）模块持久化设置占位实现。
 *
 * Phase 6.6：建立模块自带 settings 的统一约定（CamConfig 已迁移；
 * AppSettings 由内核拥有；本类为 ProcessModule 预留入口）。
 *
 * 当前仅持久化两项：
 *   - controllerEndpoint：上次连接的控制器端点（"tcp://host:port"）
 *   - simulationMode：仿真模式默认开关
 *
 * 持久化文件：`<exeDir>/config/process.toml`，路径由 @ref defaultFilePath 提供。
 * 后续按需扩充字段；新增字段需同时更新 readFrom/writeTo。
 */
class ProcessSettings : public TomlConfig
{
public:
    ProcessSettings() = default;

    /// 从默认路径加载；文件不存在视作首次启动，使用内置默认值。
    bool loadDefault();
    /// 保存到默认路径。
    bool saveDefault() const;

    QString controllerEndpoint() const { return m_controllerEndpoint; }
    void    setControllerEndpoint(const QString& endpoint);

    bool simulationMode() const { return m_simulationMode; }
    void setSimulationMode(bool enabled);

    QString motionControllerName() const { return m_motionControllerName; }
    void setMotionControllerName(const QString& name);

    QString laserDeviceName() const { return m_laserDeviceName; }
    void setLaserDeviceName(const QString& name);

    double laserEnergy() const { return m_laserEnergy; }
    void setLaserEnergy(double value);

    double laserFrequency() const { return m_laserFrequency; }
    void setLaserFrequency(double value);

    double laserPulseWidth() const { return m_laserPulseWidth; }
    void setLaserPulseWidth(double value);

protected:
    void readFrom(const toml::value& root) override;
    void writeTo(toml::value& root) const override;
    const char* configName() const override { return "ProcessSettings"; }

private:
    static QString defaultFilePath();

    QString m_controllerEndpoint;
    bool    m_simulationMode{true};
    QString m_motionControllerName{QStringLiteral("SimulatorCMHP")};
    QString m_laserDeviceName{QStringLiteral("Simulator")};
    double  m_laserEnergy{0.0};
    double  m_laserFrequency{0.0};
    double  m_laserPulseWidth{0.0};
};

} // namespace lcnc
