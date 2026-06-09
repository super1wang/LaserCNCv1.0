#pragma once

#include <QObject>
#include <QMap>
#include <QString>

#include <utility>

namespace lcnc::process {

class AcsMotionControllerAdapter : public QObject
{
    Q_OBJECT
public:
    AcsMotionControllerAdapter(QString endpoint, bool simulator, QObject* parent = nullptr);
    ~AcsMotionControllerAdapter() override;

    QString id() const;
    bool start();
    void stop();
    bool isRunning() const { return m_running; }
    bool jog(const QString& axis, double delta);
    bool moveTo(const QString& axis, double absolutePos);
    bool home(const QString& axis = QString());
    void emergencyStop();
    QMap<QString, double> axisPositions() const;
    bool setAxisEnabled(const QString& axis, bool enabled);
    bool axisEnabled(const QString& axis) const;
    bool axisHomed(const QString& axis) const;
    bool setDigitalOutput(const QString& channel, bool value, QString* errorMessage = nullptr);
    bool digitalInput(const QString& channel, bool* value, QString* errorMessage = nullptr) const;
    bool setAnalogOutput(const QString& channel, double value, QString* errorMessage = nullptr);
    bool analogInput(const QString& channel, double* value, QString* errorMessage = nullptr) const;
    bool executeProgram(const QString& program,
                        int bufferIndex,
                        bool waitForFinish,
                        int timeoutMs,
                        QString* errorMessage = nullptr);
    bool programRunning(int bufferIndex, bool* running, QString* errorMessage = nullptr) const;
    bool supportsProgramPause() const { return true; }
    bool pauseProgram(int bufferIndex, QString* errorMessage = nullptr);
    bool resumeProgram(int bufferIndex, QString* errorMessage = nullptr);

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