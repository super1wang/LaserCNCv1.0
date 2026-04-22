#pragma once

#include <QObject>
#include <QList>
#include <QMap>
#include <QString>

#include "base/machine_kinematics.h"

class QTimer;

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

    void setAxisDefinitions(const QList<MachineAxisDef>& axes);

    void jog(const QString& axisName, int direction, int speedLevel);
    void home();

    void start();
    void pause();
    void stop();
    void emergencyStop();

    State state() const;
    QMap<QString, double> currentAxisPositions() const;
    void setAxisPosition(const QString& axisName, double value);

    void setFeedOverride(double factor);
    double feedOverride() const;

    QString statusMessage() const;

signals:
    void connectionChanged(bool connected);
    void simulationModeChanged(bool enabled);
    void stateChanged(State state);
    void axisPositionChanged(const QString& axisName, double value);
    void feedOverrideChanged(double factor);
    void statusMessageChanged(const QString& message);

private slots:
    void onSimulationTick();

private:
    explicit ProcessModule(QObject* parent = nullptr);

    void initializeAxisPositions();
    void setState(State state, const QString& statusMessage);
    void setStatusMessage(const QString& message);

    static ProcessModule* s_instance;

    bool m_connected{false};
    bool m_simulationMode{true};
    State m_state{State::Idle};
    QList<MachineAxisDef> m_axisDefinitions;
    QMap<QString, double> m_axisPositions;
    QTimer* m_simTimer{nullptr};
    double m_feedOverride{1.0};
    double m_simPhase{0.0};
    QString m_statusMessage;
};
