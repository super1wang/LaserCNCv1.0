#pragma once

#include "core/kinematics/i_motion_controller.h"

#include <QObject>
#include <QString>

namespace lcnc::process {

class AcsMotionControllerAdapter : public QObject, public lcnc::IMotionController
{
    Q_OBJECT
public:
    AcsMotionControllerAdapter(QString endpoint, bool simulator, QObject* parent = nullptr);
    ~AcsMotionControllerAdapter() override;

    QString id() const override;
    bool start() override;
    void stop() override;
    bool isRunning() const override { return m_running; }
    bool jog(const QString& axis, double delta) override;
    bool moveTo(const QString& axis, double absolutePos) override;
    bool home(const QString& axis = QString()) override;
    void emergencyStop() override;

private:
    int axisIndex(const QString& axis) const;
    bool validHandle() const;

    QString m_endpoint;
    bool m_simulator{false};
    bool m_running{false};
    void* m_handle{nullptr};
    double m_defaultVelocity{10.0};
};

class AcsSimulatorCmhpControllerAdapter final : public AcsMotionControllerAdapter
{
    Q_OBJECT
public:
    explicit AcsSimulatorCmhpControllerAdapter(QString endpoint, QObject* parent = nullptr)
        : AcsMotionControllerAdapter(std::move(endpoint), true, parent)
    {
    }
};

} // namespace lcnc::process