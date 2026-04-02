#include "modules/process_module.h"

#include <QList>
#include <QTimer>

#include <algorithm>
#include <cmath>

namespace {

QString defaultStatusText(bool simulationMode, bool connected)
{
    if (simulationMode) {
        return connected
            ? QObject::tr("仿真模式 — 控制器已连接")
            : QObject::tr("仿真模式 — 未连接");
    }

    return connected
        ? QObject::tr("控制器模式 — 待机")
        : QObject::tr("控制器模式 — 未连接");
}

double jogStepForLevel(int speedLevel)
{
    switch (speedLevel) {
    case 0:
        return 0.5;
    case 2:
        return 10.0;
    case 1:
    default:
        return 2.0;
    }
}

} // namespace

ProcessModule* ProcessModule::s_instance = nullptr;

ProcessModule* ProcessModule::instance()
{
    if (!s_instance)
        s_instance = new ProcessModule();
    return s_instance;
}

ProcessModule::ProcessModule(QObject* parent)
    : QObject(parent)
{
    initializeAxisPositions();

    m_simTimer = new QTimer(this);
    m_simTimer->setInterval(100);
    connect(m_simTimer, &QTimer::timeout, this, &ProcessModule::onSimulationTick);

    setStatusMessage(defaultStatusText(m_simulationMode, m_connected));
}

bool ProcessModule::connectController(const QString& endpoint)
{
    if (endpoint.trimmed().isEmpty()) {
        setState(State::Error, tr("控制器地址不能为空"));
        return false;
    }

    const bool wasConnected = m_connected;
    m_connected = true;
    if (!wasConnected)
        emit connectionChanged(true);

    if (m_simulationMode) {
        m_simulationMode = false;
        emit simulationModeChanged(false);
    }

    if (m_state == State::Error) {
        m_state = State::Idle;
        emit stateChanged(m_state);
    }

    setStatusMessage(tr("控制器已连接: %1").arg(endpoint));
    return true;
}

void ProcessModule::disconnectController()
{
    const bool wasConnected = m_connected;
    m_connected = false;
    m_simTimer->stop();
    m_state = State::Idle;
    if (wasConnected)
        emit connectionChanged(false);
    emit stateChanged(m_state);
    setStatusMessage(defaultStatusText(m_simulationMode, m_connected));
}

bool ProcessModule::isConnected() const
{
    return m_connected;
}

void ProcessModule::setSimulationMode(bool on)
{
    if (m_simulationMode == on)
        return;

    m_simulationMode = on;
    emit simulationModeChanged(on);

    if (on && m_state == State::Error) {
        m_state = State::Idle;
        emit stateChanged(m_state);
    }

    setStatusMessage(defaultStatusText(m_simulationMode, m_connected));
}

bool ProcessModule::simulationMode() const
{
    return m_simulationMode;
}

void ProcessModule::jog(const QString& axisName, int direction, int speedLevel)
{
    if (axisName.trimmed().isEmpty() || direction == 0 || m_state == State::EmergencyStop)
        return;

    const double delta = jogStepForLevel(speedLevel) * (direction > 0 ? 1.0 : -1.0);
    setAxisPosition(axisName, m_axisPositions.value(axisName) + delta);
    setStatusMessage(tr("点动 %1 轴 %2").arg(
        axisName,
        direction > 0 ? tr("正向") : tr("负向")));
}

void ProcessModule::home()
{
    if (m_state == State::EmergencyStop)
        return;

    initializeAxisPositions();
    m_simPhase = 0.0;
    setStatusMessage(tr("回零完成"));
}

void ProcessModule::start()
{
    if (m_state == State::EmergencyStop) {
        setStatusMessage(tr("急停状态，复位后才能运行"));
        return;
    }

    if (!m_simulationMode && !m_connected) {
        setState(State::Error, tr("未连接控制器，无法运行"));
        return;
    }

    if (m_simulationMode) {
        const int intervalMs = std::max(30, static_cast<int>(100.0 / std::max(0.1, m_feedOverride)));
        m_simTimer->start(intervalMs);
    }

    setState(State::Running,
             m_simulationMode ? tr("仿真运行中") : tr("控制器运行中"));
}

void ProcessModule::pause()
{
    if (m_state != State::Running)
        return;

    m_simTimer->stop();
    setState(State::Paused, tr("运行已暂停"));
}

void ProcessModule::stop()
{
    m_simTimer->stop();
    setState(State::Idle,
             m_simulationMode ? tr("仿真已停止") : tr("运行已停止"));
}

void ProcessModule::emergencyStop()
{
    m_simTimer->stop();
    setState(State::EmergencyStop, tr("急停已触发"));
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
    const double oldValue = m_axisPositions.value(axisName, 0.0);
    if (m_axisPositions.contains(axisName) && std::abs(oldValue - value) < 1e-9)
        return;

    m_axisPositions.insert(axisName, value);
    emit axisPositionChanged(axisName, value);
}

void ProcessModule::setFeedOverride(double factor)
{
    const double clamped = std::clamp(factor, 0.0, 2.0);
    if (std::abs(m_feedOverride - clamped) < 1e-9)
        return;

    m_feedOverride = clamped;
    emit feedOverrideChanged(m_feedOverride);

    if (m_simulationMode && m_simTimer->isActive()) {
        const int intervalMs = std::max(30, static_cast<int>(100.0 / std::max(0.1, m_feedOverride)));
        m_simTimer->setInterval(intervalMs);
    }

    setStatusMessage(tr("进给倍率: %1%").arg(qRound(m_feedOverride * 100.0)));
}

double ProcessModule::feedOverride() const
{
    return m_feedOverride;
}

QString ProcessModule::statusMessage() const
{
    return m_statusMessage;
}

void ProcessModule::onSimulationTick()
{
    if (m_state != State::Running || !m_simulationMode)
        return;

    m_simPhase += 0.08 * std::max(0.1, m_feedOverride);

    setAxisPosition(QStringLiteral("X"), std::sin(m_simPhase) * 45.0);
    setAxisPosition(QStringLiteral("Y"), std::cos(m_simPhase * 0.75) * 30.0);
    setAxisPosition(QStringLiteral("Z"), 15.0 + std::sin(m_simPhase * 0.5) * 12.0);
    setAxisPosition(QStringLiteral("A"), std::sin(m_simPhase * 0.4) * 35.0);
    setAxisPosition(QStringLiteral("C"), std::cos(m_simPhase * 0.3) * 80.0);
}

void ProcessModule::initializeAxisPositions()
{
    const QList<QString> axisNames = {
        QStringLiteral("X"),
        QStringLiteral("Y"),
        QStringLiteral("Z"),
        QStringLiteral("A"),
        QStringLiteral("C"),
    };

    for (const QString& axisName : axisNames)
        setAxisPosition(axisName, 0.0);
}

void ProcessModule::setState(State state, const QString& statusMessage)
{
    if (m_state != state) {
        m_state = state;
        emit stateChanged(m_state);
    }

    setStatusMessage(statusMessage);
}

void ProcessModule::setStatusMessage(const QString& message)
{
    if (m_statusMessage == message)
        return;

    m_statusMessage = message;
    emit statusMessageChanged(m_statusMessage);
}
