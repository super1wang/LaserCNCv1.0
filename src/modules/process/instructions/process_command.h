#pragma once

#include <QString>
#include <QVector>

namespace lcnc::process {

/**
 * @brief Controller-neutral Process command type.
 */
enum class ProcessCommandType
{
    SetFeed,
    SetLaserPower,
    LaserOn,
    LaserOff,
    MoveLinear,
    SetDigitalOutput,
    Dwell,
    WaitSignal,
    Home,
    Stop
};

/**
 * @brief One controller-neutral command generated before translator dispatch.
 */
struct ProcessCommand
{
    ProcessCommandType type{ProcessCommandType::Dwell};
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double r1{0.0};
    double r2{0.0};
    double feedRate{0.0};
    double laserEnergy{0.0};
    double laserFrequency{0.0};
    double laserPulseWidth{0.0};
    int durationMs{0};
    QString channel;
    bool boolValue{false};
    QString description;
};

/**
 * @brief Ordered controller-neutral command buffer.
 */
class ProcessCommandBuffer
{
public:
    void clear() { m_commands.clear(); }
    void append(const ProcessCommand& command) { m_commands.append(command); }
    const QVector<ProcessCommand>& commands() const { return m_commands; }
    int size() const { return m_commands.size(); }
    bool isEmpty() const { return m_commands.isEmpty(); }
    QString summary() const;

private:
    QVector<ProcessCommand> m_commands;
};

} // namespace lcnc::process