#include "modules/process/controllers/gtn_motion_controller_adapter.h"

#include "core/logging/logger.h"

#include <QHash>
#include <QtMath>

#include "gts.h"

namespace lcnc::process {

namespace {

QHash<QString, short> axisIndexMap()
{
    return {
        { QStringLiteral("X"), 1 },
        { QStringLiteral("Y"), 2 },
        { QStringLiteral("Z"), 3 },
        { QStringLiteral("A"), 4 },
        { QStringLiteral("X1"), 5 },
        { QStringLiteral("Y1"), 6 },
        { QStringLiteral("Z1"), 7 },
        { QStringLiteral("A1"), 8 },
    };
}

long axisMask(short axis)
{
    if (axis < 1 || axis > 32)
        return 0;
    return 1L << (axis - 1);
}

} // namespace

GtnMotionControllerAdapter::GtnMotionControllerAdapter(QObject* parent)
    : QObject(parent)
{
}

GtnMotionControllerAdapter::~GtnMotionControllerAdapter()
{
    stop();
}

bool GtnMotionControllerAdapter::start()
{
    if (m_running)
        return true;

    const short result = GTN_Open(5, 2);
    if (result != 0) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "process.motion: GTN_Open failed code={}",
                 result);
        return false;
    }
    m_running = true;
    LCNC_INFO(lcnc::LogCode::Generic, "process.motion: GTN connected");
    return true;
}

void GtnMotionControllerAdapter::stop()
{
    if (m_running) {
        GTN_Stop(m_core, -1, 1);
        GTN_Close();
    }
    m_running = false;
    m_axisPositions.clear();
}

bool GtnMotionControllerAdapter::jog(const QString& axis, double delta)
{
    const short index = axisIndex(axis);
    if (index < 1)
        return false;
    return moveAxis(index, m_axisPositions.value(index, 0.0) + delta);
}

bool GtnMotionControllerAdapter::moveTo(const QString& axis, double absolutePos)
{
    const short index = axisIndex(axis);
    if (index < 1)
        return false;
    return moveAxis(index, absolutePos);
}

bool GtnMotionControllerAdapter::home(const QString& axis)
{
    if (!m_running && !start())
        return false;
    if (axis.trimmed().isEmpty()) {
        bool ok = true;
        for (short index = 1; index <= 8; ++index) {
            ok = GTN_ZeroPos(m_core, index, 1) == 0 && ok;
            m_axisPositions.insert(index, 0.0);
        }
        return ok;
    }
    const short index = axisIndex(axis);
    if (index < 1)
        return false;
    const bool ok = GTN_ZeroPos(m_core, index, 1) == 0;
    if (ok)
        m_axisPositions.insert(index, 0.0);
    return ok;
}

void GtnMotionControllerAdapter::emergencyStop()
{
    if (m_running)
        GTN_Stop(m_core, -1, 1);
}

short GtnMotionControllerAdapter::axisIndex(const QString& axis) const
{
    static const QHash<QString, short> indexes = axisIndexMap();
    return indexes.value(axis.trimmed().toUpper(), -1);
}

bool GtnMotionControllerAdapter::moveAxis(short axis, double absolutePos)
{
    if (!m_running && !start())
        return false;

    const long mask = axisMask(axis);
    if (!mask)
        return false;

    if (GTN_ClrSts(m_core, axis, 1) != 0)
        return false;
    if (GTN_AxisOn(m_core, axis) != 0)
        return false;
    if (GTN_PrfTrap(m_core, axis) != 0)
        return false;
    if (GTN_SetPos(m_core, axis, static_cast<long>(qRound64(absolutePos))) != 0)
        return false;
    if (GTN_Update(m_core, mask) != 0)
        return false;

    m_axisPositions.insert(axis, absolutePos);
    return true;
}

} // namespace lcnc::process