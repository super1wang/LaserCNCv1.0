#include "modules/process_module.h"

ProcessModule* ProcessModule::s_instance = nullptr;

ProcessModule* ProcessModule::instance()
{
    if (!s_instance)
        s_instance = new ProcessModule();
    return s_instance;
}

ProcessModule::ProcessModule(QObject* parent)
    : QObject(parent)
{}

bool ProcessModule::connectController(const QString& endpoint)
{
    m_connected = !endpoint.trimmed().isEmpty();
    emit connectionChanged(m_connected);
    return m_connected;
}

void ProcessModule::disconnectController()
{
    m_connected = false;
    m_state = State::Idle;
    emit connectionChanged(false);
    emit stateChanged(m_state);
}

bool ProcessModule::isConnected() const
{
    return m_connected;
}

void ProcessModule::setSimulationMode(bool on)
{
    if (m_simulationMode == on) return;
    m_simulationMode = on;
    emit simulationModeChanged(on);
}

bool ProcessModule::simulationMode() const
{
    return m_simulationMode;
}

void ProcessModule::start()
{
    m_state = State::Running;
    emit stateChanged(m_state);
}

void ProcessModule::pause()
{
    m_state = State::Paused;
    emit stateChanged(m_state);
}

void ProcessModule::stop()
{
    m_state = State::Idle;
    emit stateChanged(m_state);
}

void ProcessModule::emergencyStop()
{
    m_state = State::EmergencyStop;
    emit stateChanged(m_state);
}

ProcessModule::State ProcessModule::state() const
{
    return m_state;
}

QMap<QString, double> ProcessModule::currentAxisPositions() const
{
    return m_axisPositions;
}

void ProcessModule::setAxisPosition(const QString& axisName, double value)
{
    m_axisPositions.insert(axisName, value);
    emit axisPositionChanged(axisName, value);
}
