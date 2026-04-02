#pragma once

#include <QObject>
#include <QMap>
#include <QString>

/**
 * @brief Process module singleton — manages execution process, peripherals, and parameters.
 *
 * Phase 5 is still in progress, so this module currently provides a thin
 * placeholder API and state container. Ribbon/UI should depend on this module
 * instead of embedding process logic directly.
 */
class ProcessModule : public QObject
{
    Q_OBJECT
public:
    enum class State {
        Idle,
        Running,
        Paused,
        Error,
        EmergencyStop,
    };

    static ProcessModule* instance();

    bool connectController(const QString& endpoint);
    void disconnectController();
    bool isConnected() const;

    void setSimulationMode(bool on);
    bool simulationMode() const;

    void start();
    void pause();
    void stop();
    void emergencyStop();

    State state() const;
    QMap<QString, double> currentAxisPositions() const;
    void setAxisPosition(const QString& axisName, double value);

signals:
    void connectionChanged(bool connected);
    void simulationModeChanged(bool enabled);
    void stateChanged(State state);
    void axisPositionChanged(const QString& axisName, double value);

private:
    explicit ProcessModule(QObject* parent = nullptr);

    static ProcessModule* s_instance;

    bool m_connected{false};
    bool m_simulationMode{true};
    State m_state{State::Idle};
    QMap<QString, double> m_axisPositions;
};
