#pragma once

#include "core/kinematics/i_motion_controller.h"

#include <QObject>
#include <QMap>
#include <QString>

#include <utility>

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
    QMap<QString, double> axisPositions() const override;
    bool setAxisEnabled(const QString& axis, bool enabled) override;
    bool axisEnabled(const QString& axis) const override;
    bool axisHomed(const QString& axis) const override;
    bool setDigitalOutput(const QString& channel, bool value, QString* errorMessage = nullptr) override;
    bool digitalInput(const QString& channel, bool* value, QString* errorMessage = nullptr) const override;
    bool setAnalogOutput(const QString& channel, double value, QString* errorMessage = nullptr) override;
    bool analogInput(const QString& channel, double* value, QString* errorMessage = nullptr) const override;
    bool executeProgram(const QString& program,
                        int bufferIndex,
                        bool waitForFinish,
                        int timeoutMs,
                        QString* errorMessage = nullptr) override;
    bool programRunning(int bufferIndex, bool* running, QString* errorMessage = nullptr) const override;
    bool supportsProgramPause() const override { return true; }
    bool pauseProgram(int bufferIndex, QString* errorMessage = nullptr) override;
    bool resumeProgram(int bufferIndex, QString* errorMessage = nullptr) override;

private:
    int axisIndex(const QString& axis) const;
    bool validHandle() const;
    void logLastError(const QString& operation, QString* errorMessage = nullptr) const;
    int homeBufferIndex(const QString& axis) const;
    bool runBufferToEnd(int bufferIndex, int timeoutMs, QString* errorMessage = nullptr);
    bool parseIoBit(const QString& channel, int* bit, QString* variableName = nullptr) const;

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