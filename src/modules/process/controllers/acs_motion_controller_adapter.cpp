#include "modules/process/controllers/acs_motion_controller_adapter.h"

#include "core/logging/logger.h"

#include <QByteArray>
#include <QHash>
#include <QUrl>

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
        { QStringLiteral("X1"), 4 },
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
    if (!validHandle() && !start())
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
    if (!validHandle() && !start())
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
    if (!validHandle() && !start())
        return false;
    if (axis.trimmed().isEmpty()) {
        bool ok = true;
        for (int index = 0; index < 8; ++index)
            ok = acsc_SetFPosition(static_cast<HANDLE>(m_handle), index, 0.0, nullptr) != 0 && ok;
        return ok;
    }
    const int index = axisIndex(axis);
    if (index < 0)
        return false;
    return acsc_SetFPosition(static_cast<HANDLE>(m_handle), index, 0.0, nullptr) != 0;
}

void AcsMotionControllerAdapter::emergencyStop()
{
    if (validHandle())
        acsc_StopBuffer(static_cast<HANDLE>(m_handle), ACSC_NONE, nullptr);
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

} // namespace lcnc::process