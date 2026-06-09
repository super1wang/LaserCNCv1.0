#include "modules/process/controllers/acs_motion_controller_adapter.h"

#include "core/kernel/kernel.h"
#include "core/kinematics/machine_configuration_service.h"
#include "core/logging/logger.h"

#include <QByteArray>
#include <QHash>
#include <QRegularExpression>
#include <QUrl>

#include <algorithm>
#include <cmath>

#include "ACSC.h"

namespace lcnc::process {

namespace {

QString endpointHost(const QString& endpoint)
{
    const QUrl url(endpoint.trimmed());
    if (url.isValid() && !url.host().isEmpty())
        return url.host();
    const QString trimmed = endpoint.trimmed();
    return trimmed.isEmpty() ? QStringLiteral("10.0.0.100") : trimmed;
}

int endpointPort(const QString& endpoint)
{
    const QUrl url(endpoint.trimmed());
    if (url.isValid() && url.port() > 0)
        return url.port();
    return ACSC_SOCKET_STREAM_PORT;
}

QHash<QString, int> axisIndexMap()
{
    return {
        { QStringLiteral("X"), 0 },
        { QStringLiteral("Y"), 1 },
        { QStringLiteral("Z"), 2 },
        { QStringLiteral("A"), 3 },
        { QStringLiteral("C"), 4 },
        { QStringLiteral("X1"), 4 },
        { QStringLiteral("B"), 5 },
        { QStringLiteral("Y1"), 5 },
        { QStringLiteral("Z1"), 6 },
        { QStringLiteral("A1"), 7 },
    };
}

} // namespace

AcsMotionControllerAdapter::AcsMotionControllerAdapter(QString endpoint, bool simulator, QObject* parent)
    : QObject(parent)
    , m_endpoint(std::move(endpoint))
    , m_simulator(simulator)
    , m_handle(ACSC_INVALID)
{
}

AcsMotionControllerAdapter::~AcsMotionControllerAdapter()
{
    stop();
}

QString AcsMotionControllerAdapter::id() const
{
    return m_simulator ? QStringLiteral("SimulatorCMHP") : QStringLiteral("ACS");
}

bool AcsMotionControllerAdapter::start()
{
    if (m_running && validHandle())
        return true;

    if (m_simulator) {
        m_handle = acsc_OpenCommSimulator();
    } else {
        QByteArray host = endpointHost(m_endpoint).toLocal8Bit();
        m_handle = acsc_OpenCommEthernetTCP(host.data(), endpointPort(m_endpoint));
    }

    if (!validHandle()) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "process.motion: {} connect failed",
                 id().toStdString());
        m_running = false;
        return false;
    }

    acsc_StopBuffer(static_cast<HANDLE>(m_handle), ACSC_NONE, nullptr);
    m_running = true;
    LCNC_INFO(lcnc::LogCode::Generic,
              "process.motion: {} connected",
              id().toStdString());
    return true;
}

void AcsMotionControllerAdapter::stop()
{
    if (validHandle()) {
        acsc_StopBuffer(static_cast<HANDLE>(m_handle), ACSC_NONE, nullptr);
        acsc_CloseComm(static_cast<HANDLE>(m_handle));
        if (m_simulator)
            acsc_CloseSimulator();
    }
    m_handle = ACSC_INVALID;
    m_running = false;
}

bool AcsMotionControllerAdapter::jog(const QString& axis, double delta)
{
    if (!validHandle())
        return false;
    const int index = axisIndex(axis);
    if (index < 0)
        return false;
    return acsc_ExtToPoint(static_cast<HANDLE>(m_handle),
                           ACSC_AMF_RELATIVE | ACSC_AMF_VELOCITY,
                           index,
                           delta,
                           m_defaultVelocity,
                           0.0,
                           nullptr) != 0;
}

bool AcsMotionControllerAdapter::moveTo(const QString& axis, double absolutePos)
{
    if (!validHandle())
        return false;
    const int index = axisIndex(axis);
    if (index < 0)
        return false;
    return acsc_ExtToPoint(static_cast<HANDLE>(m_handle),
                           ACSC_AMF_VELOCITY,
                           index,
                           absolutePos,
                           m_defaultVelocity,
                           0.0,
                           nullptr) != 0;
}

bool AcsMotionControllerAdapter::home(const QString& axis)
{
    if (!validHandle())
        return false;
    if (axis.trimmed().isEmpty()) {
        bool ok = true;
        const QStringList order = { QStringLiteral("Z"), QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("A"), QStringLiteral("C") };
        for (const QString& axisName : order)
            ok = home(axisName) && ok;
        return ok;
    }
    const QString normalizedAxis = axis.trimmed().toUpper();
    const int homeBuffer = homeBufferIndex(normalizedAxis);
    if (homeBuffer >= 0)
        return runBufferToEnd(homeBuffer, 300000);

    const int index = axisIndex(normalizedAxis);
    if (index < 0)
        return false;
    return acsc_SetFPosition(static_cast<HANDLE>(m_handle), index, 0.0, nullptr) != 0;
}

void AcsMotionControllerAdapter::emergencyStop()
{
    if (validHandle())
        acsc_StopBuffer(static_cast<HANDLE>(m_handle), ACSC_NONE, nullptr);
}

QMap<QString, double> AcsMotionControllerAdapter::axisPositions() const
{
    QMap<QString, double> positions;
    if (!validHandle())
        return positions;
    const QHash<QString, int> indexes = axisIndexMap();
    for (auto it = indexes.cbegin(); it != indexes.cend(); ++it) {
        if (positions.contains(it.key()))
            continue;
        double value = 0.0;
        if (acsc_GetFPosition(static_cast<HANDLE>(m_handle), it.value(), &value, nullptr))
            positions.insert(it.key(), value);
    }
    return positions;
}

bool AcsMotionControllerAdapter::setAxisEnabled(const QString& axis, bool enabled)
{
    if (!validHandle())
        return false;
    const int index = axisIndex(axis);
    if (index < 0)
        return false;
    const int ok = enabled
        ? acsc_Enable(static_cast<HANDLE>(m_handle), index, nullptr)
        : acsc_Disable(static_cast<HANDLE>(m_handle), index, nullptr);
    if (!ok) {
        logLastError(QStringLiteral("setAxisEnabled"));
        return false;
    }
    return acsc_WaitMotorEnabled(static_cast<HANDLE>(m_handle), index, enabled ? 1 : 0, 3000) != 0;
}

bool AcsMotionControllerAdapter::axisEnabled(const QString& axis) const
{
    if (!validHandle())
        return false;
    const int index = axisIndex(axis);
    if (index < 0)
        return false;
    int state = 0;
    if (!acsc_GetMotorState(static_cast<HANDLE>(m_handle), index, &state, nullptr))
        return false;
    return (state & ACSC_MST_ENABLE) != 0;
}

bool AcsMotionControllerAdapter::axisHomed(const QString& axis) const
{
    if (!validHandle())
        return false;
    const QString homeVariable = axis.trimmed().toUpper() + QStringLiteral("Home");
    QByteArray name = homeVariable.toLocal8Bit();
    int homeStatus = 0;
    if (acsc_ReadInteger(static_cast<HANDLE>(m_handle), ACSC_NONE, name.data(),
                         0, 0, ACSC_NONE, ACSC_NONE, &homeStatus, nullptr)) {
        return homeStatus == 1;
    }
    const int index = axisIndex(axis);
    if (index < 0)
        return false;
    double position = 0.0;
    return acsc_GetFPosition(static_cast<HANDLE>(m_handle), index, &position, nullptr) && std::abs(position) < 1e-6;
}

bool AcsMotionControllerAdapter::setDigitalOutput(const QString& channel, bool value, QString* errorMessage)
{
    if (!validHandle() && !start())
        return false;
    int bit = -1;
    QString variableName;
    parseIoBit(channel, &bit, &variableName);
    if (bit >= 0) {
        if (acsc_SetOutput(static_cast<HANDLE>(m_handle), 0, bit, value ? 1 : 0, nullptr))
            return true;
        logLastError(QStringLiteral("setDigitalOutput"), errorMessage);
        return false;
    }
    QByteArray variable = variableName.toLocal8Bit();
    int intValue = value ? 1 : 0;
    if (acsc_WriteInteger(static_cast<HANDLE>(m_handle), ACSC_NONE, variable.data(),
                          ACSC_NONE, ACSC_NONE, ACSC_NONE, ACSC_NONE, &intValue, nullptr))
        return true;
    logLastError(QStringLiteral("setDigitalOutput"), errorMessage);
    return false;
}

bool AcsMotionControllerAdapter::digitalInput(const QString& channel, bool* value, QString* errorMessage) const
{
    if (!value)
        return false;
    if (!validHandle())
        return false;
    int bit = -1;
    QString variableName;
    const bool numeric = parseIoBit(channel, &bit, &variableName);
    int intValue = 0;
    if (numeric && bit >= 0) {
        const bool readOutput = channel.trimmed().startsWith(QStringLiteral("DO"), Qt::CaseInsensitive);
        const int ok = readOutput
            ? acsc_GetOutput(static_cast<HANDLE>(m_handle), 0, bit, &intValue, nullptr)
            : acsc_GetInput(static_cast<HANDLE>(m_handle), 0, bit, &intValue, nullptr);
        if (ok) {
            *value = intValue != 0;
            return true;
        }
        logLastError(QStringLiteral("digitalInput"), errorMessage);
        return false;
    }
    QByteArray variable = variableName.toLocal8Bit();
    if (acsc_ReadInteger(static_cast<HANDLE>(m_handle), ACSC_NONE, variable.data(),
                         ACSC_NONE, ACSC_NONE, ACSC_NONE, ACSC_NONE, &intValue, nullptr)) {
        *value = intValue != 0;
        return true;
    }
    logLastError(QStringLiteral("digitalInput"), errorMessage);
    return false;
}

bool AcsMotionControllerAdapter::setAnalogOutput(const QString& channel, double value, QString* errorMessage)
{
    if (!validHandle() && !start())
        return false;
    const QString variableName = channel.trimmed().isEmpty() ? QStringLiteral("AOUT0") : channel.trimmed();
    QByteArray variable = variableName.toLocal8Bit();
    if (acsc_WriteReal(static_cast<HANDLE>(m_handle), ACSC_NONE, variable.data(),
                       ACSC_NONE, ACSC_NONE, ACSC_NONE, ACSC_NONE, &value, nullptr))
        return true;
    logLastError(QStringLiteral("setAnalogOutput"), errorMessage);
    return false;
}

bool AcsMotionControllerAdapter::analogInput(const QString& channel, double* value, QString* errorMessage) const
{
    if (!value || !validHandle())
        return false;
    const QString variableName = channel.trimmed().isEmpty() ? QStringLiteral("AIN0") : channel.trimmed();
    QByteArray variable = variableName.toLocal8Bit();
    if (acsc_ReadReal(static_cast<HANDLE>(m_handle), ACSC_NONE, variable.data(),
                      ACSC_NONE, ACSC_NONE, ACSC_NONE, ACSC_NONE, value, nullptr))
        return true;
    logLastError(QStringLiteral("analogInput"), errorMessage);
    return false;
}

bool AcsMotionControllerAdapter::executeProgram(const QString& program,
                                                int bufferIndex,
                                                bool waitForFinish,
                                                int timeoutMs,
                                                QString* errorMessage)
{
    if (!validHandle() && !start())
        return false;
    QByteArray bytes = program.toLocal8Bit();
    if (!acsc_StopBuffer(static_cast<HANDLE>(m_handle), bufferIndex, ACSC_SYNCHRONOUS)) {
        logLastError(QStringLiteral("StopBuffer"), errorMessage);
        return false;
    }
    if (!acsc_LoadBuffer(static_cast<HANDLE>(m_handle), bufferIndex, bytes.data(), bytes.size(), ACSC_SYNCHRONOUS)) {
        logLastError(QStringLiteral("LoadBuffer"), errorMessage);
        return false;
    }
    if (!acsc_CompileBuffer(static_cast<HANDLE>(m_handle), bufferIndex, ACSC_SYNCHRONOUS)) {
        logLastError(QStringLiteral("CompileBuffer"), errorMessage);
        return false;
    }
    if (!acsc_RunBuffer(static_cast<HANDLE>(m_handle), bufferIndex, nullptr, ACSC_SYNCHRONOUS)) {
        logLastError(QStringLiteral("RunBuffer"), errorMessage);
        return false;
    }
    if (waitForFinish && !acsc_WaitProgramEnd(static_cast<HANDLE>(m_handle), bufferIndex, timeoutMs > 0 ? timeoutMs : 300000)) {
        logLastError(QStringLiteral("WaitProgramEnd"), errorMessage);
        return false;
    }
    return true;
}

bool AcsMotionControllerAdapter::programRunning(int bufferIndex, bool* running, QString* errorMessage) const
{
    if (!running || !validHandle())
        return false;
    int state = 0;
    if (!acsc_GetProgramState(static_cast<HANDLE>(m_handle), bufferIndex, &state, ACSC_SYNCHRONOUS)) {
        logLastError(QStringLiteral("GetProgramState"), errorMessage);
        return false;
    }
    *running = (state & ACSC_PST_RUN) != 0;
    return true;
}

bool AcsMotionControllerAdapter::pauseProgram(int bufferIndex, QString* errorMessage)
{
    if (!validHandle()) {
        if (errorMessage)
            *errorMessage = QStringLiteral("ACS controller is not connected");
        return false;
    }
    if (!acsc_SuspendBuffer(static_cast<HANDLE>(m_handle), bufferIndex, ACSC_SYNCHRONOUS)) {
        logLastError(QStringLiteral("SuspendBuffer"), errorMessage);
        return false;
    }
    return true;
}

bool AcsMotionControllerAdapter::resumeProgram(int bufferIndex, QString* errorMessage)
{
    if (!validHandle()) {
        if (errorMessage)
            *errorMessage = QStringLiteral("ACS controller is not connected");
        return false;
    }
    if (!acsc_RunBuffer(static_cast<HANDLE>(m_handle), bufferIndex, nullptr, ACSC_SYNCHRONOUS)) {
        logLastError(QStringLiteral("RunBuffer"), errorMessage);
        return false;
    }
    return true;
}

int AcsMotionControllerAdapter::axisIndex(const QString& axis) const
{
    static const QHash<QString, int> indexes = axisIndexMap();
    return indexes.value(axis.trimmed().toUpper(), -1);
}

bool AcsMotionControllerAdapter::validHandle() const
{
    return m_handle != ACSC_INVALID && m_handle != nullptr;
}

void AcsMotionControllerAdapter::logLastError(const QString& operation, QString* errorMessage) const
{
    char errorBuffer[256] = {};
    int received = 0;
    const int code = acsc_GetLastError();
    QString message = QStringLiteral("%1 failed: ACS error %2").arg(operation).arg(code);
    if (validHandle() && acsc_GetErrorString(static_cast<HANDLE>(m_handle), code, errorBuffer, 255, &received)) {
        errorBuffer[std::clamp(received, 0, 255)] = '\0';
        message = QStringLiteral("%1 failed: %2").arg(operation, QString::fromLocal8Bit(errorBuffer).trimmed());
    }
    if (errorMessage)
        *errorMessage = message;
    LCNC_ERR(lcnc::LogCode::Generic, "process.motion.acs: {}", message.toStdString());
}

int AcsMotionControllerAdapter::homeBufferIndex(const QString& axis) const
{
    auto* kernel = lcnc::Kernel::tryCurrent();
    auto* machineConfig = kernel ? kernel->service<lcnc::MachineConfigurationService>() : nullptr;
    if (machineConfig) {
        for (const auto& config : machineConfig->axisConfigurations()) {
            if (config.axis.name.compare(axis, Qt::CaseInsensitive) == 0 && config.homeIndex >= 0)
                return config.homeIndex;
        }
    }
    static const QHash<QString, int> fallback = {
        { QStringLiteral("X"), 0 },
        { QStringLiteral("Y"), 1 },
        { QStringLiteral("Z"), 2 },
        { QStringLiteral("A"), 3 },
        { QStringLiteral("C"), 4 }
    };
    return fallback.value(axis.trimmed().toUpper(), -1);
}

bool AcsMotionControllerAdapter::runBufferToEnd(int bufferIndex, int timeoutMs, QString* errorMessage)
{
    if (!acsc_RunBuffer(static_cast<HANDLE>(m_handle), bufferIndex, nullptr, ACSC_SYNCHRONOUS)) {
        logLastError(QStringLiteral("RunHomeBuffer"), errorMessage);
        return false;
    }
    if (!acsc_WaitProgramEnd(static_cast<HANDLE>(m_handle), bufferIndex, timeoutMs)) {
        logLastError(QStringLiteral("WaitHomeBuffer"), errorMessage);
        return false;
    }
    return true;
}

bool AcsMotionControllerAdapter::parseIoBit(const QString& channel, int* bit, QString* variableName) const
{
    const QString trimmed = channel.trimmed();
    if (variableName)
        *variableName = trimmed.isEmpty() ? QStringLiteral("DO0") : trimmed;
    if (bit)
        *bit = -1;
    const QStringList parts = trimmed.split(QRegularExpression(QStringLiteral("[^0-9]+")), Qt::SkipEmptyParts);
    if (parts.isEmpty())
        return false;
    bool ok = false;
    const int parsed = parts.constLast().toInt(&ok);
    if (ok && bit)
        *bit = parsed;
    return ok;
}

} // namespace lcnc::process