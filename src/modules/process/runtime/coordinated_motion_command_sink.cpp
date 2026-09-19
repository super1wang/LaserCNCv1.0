#include "modules/process/runtime/coordinated_motion_command_sink.h"

#include "modules/process/runtime/process_interrupt_context.h"

#include <QObject>
#include <QThread>

namespace lcnc::process {

CoordinatedMotionCommandSink::CoordinatedMotionCommandSink(
    std::unique_ptr<IMotionCommandSink> inner,
    ProcessDeviceCoordinator& coordinator)
    : m_inner(std::move(inner))
    , m_coordinator(coordinator)
{
}

QString CoordinatedMotionCommandSink::id() const
{
    const auto lease = m_coordinator.acquire();
    return m_inner ? m_inner->id() : QStringLiteral("Unavailable");
}

CoordinatedMotionCommandSink::~CoordinatedMotionCommandSink()
{
    const auto lease = m_coordinator.acquire();
    m_inner.reset();
}

bool CoordinatedMotionCommandSink::prepareExactSection(const PreparedDeviceProgram& p, int i, QString* e)
{
    const auto lease = m_coordinator.acquire();
    return m_inner && m_inner->prepareExactSection(p, i, e);
}
bool CoordinatedMotionCommandSink::continueExactPreparation(const PreparedDeviceProgram& p, int i, bool& done, QString* e)
{
    const auto lease = m_coordinator.acquire();
    done = false;
    return m_inner && m_inner->continueExactPreparation(p, i, done, e);
}
bool CoordinatedMotionCommandSink::startExactSection(const PreparedDeviceProgram& p, int i, QString* e)
{
    const auto lease = m_coordinator.acquire();
    return m_inner && m_inner->startExactSection(p, i, e);
}
bool CoordinatedMotionCommandSink::isExactSectionRunning(const PreparedDeviceProgram& p, int i, QString* e)
{
    const auto lease = m_coordinator.acquire();
    if (!m_inner) {
        if (e) *e = QStringLiteral("Exact motion sink is unavailable");
        return false;
    }
    return m_inner && m_inner->isExactSectionRunning(p, i, e);
}

bool CoordinatedMotionCommandSink::supportsBatchProgram() const
{
    return m_inner && m_inner->supportsBatchProgram();
}

void CoordinatedMotionCommandSink::setCancellation(ProcessInterruptContext* token)
{
    m_token = token;
    if (m_inner)
        m_inner->setCancellation(token);
}

void CoordinatedMotionCommandSink::resetProgram()
{
    const auto lease = m_coordinator.acquire();
    if (m_inner)
        m_inner->resetProgram();
}

bool CoordinatedMotionCommandSink::startProgram(QString* errorMessage)
{
    const auto lease = m_coordinator.acquire();
    return m_inner && m_inner->startProgram(errorMessage);
}

bool CoordinatedMotionCommandSink::isProgramRunning(QString* errorMessage)
{
    const auto lease = m_coordinator.acquire();
    return m_inner && m_inner->isProgramRunning(errorMessage);
}

bool CoordinatedMotionCommandSink::flush(QString* errorMessage)
{
    if (!startProgram(errorMessage))
        return false;
    while (isProgramRunning(errorMessage)) {
        if (m_token) {
            while (m_token->isPaused() && !m_token->isStopping())
                QThread::msleep(10);
            if (m_token->isStopping()) {
                // 中文翻译：切割已被中断
                if (errorMessage)
                    *errorMessage = QObject::tr("Cutting has been interrupted");
                return false;
            }
        }
        // No device lease is held while sleeping. The status executor can
        // acquire it and publish APOS between controller-running checks.
        // 中文翻译：等待期间不持有设备租约，坐标线程可在状态检查之间读取 APOS。
        QThread::msleep(10);
    }
    return !errorMessage || errorMessage->isEmpty();
}

bool CoordinatedMotionCommandSink::executeRapidSegment(
    const lcnc::cam::RapidMoveSegment& segment,
    const Tool& tool,
    QString* errorMessage)
{
    const auto lease = m_coordinator.acquire();
    return m_inner && m_inner->executeRapidSegment(segment, tool, errorMessage);
}

void CoordinatedMotionCommandSink::startCuttingHead(const Tool& tool)
{
    const auto lease = m_coordinator.acquire();
    if (m_inner)
        m_inner->startCuttingHead(tool);
}

void CoordinatedMotionCommandSink::stopCuttingHead()
{
    const auto lease = m_coordinator.acquire();
    if (m_inner)
        m_inner->stopCuttingHead();
}

void CoordinatedMotionCommandSink::setShutterTimings(
    double beforeOn, double afterOn, double beforeOff, double afterOff,
    double blowDelay)
{
    const auto lease = m_coordinator.acquire();
    if (m_inner) {
        m_inner->setShutterTimings(beforeOn, afterOn, beforeOff, afterOff,
                                   blowDelay);
    }
}

bool CoordinatedMotionCommandSink::lineTo(
    const MachinePose5& target, const Tool& tool, QString* errorMessage)
{
    const auto lease = m_coordinator.acquire();
    return m_inner && m_inner->lineTo(target, tool, errorMessage);
}

bool CoordinatedMotionCommandSink::beginSegment(
    const MachinePose5& startPose, const Tool& tool, QString* errorMessage)
{
    const auto lease = m_coordinator.acquire();
    return m_inner && m_inner->beginSegment(startPose, tool, errorMessage);
}

void CoordinatedMotionCommandSink::endSegment(const Tool& tool)
{
    const auto lease = m_coordinator.acquire();
    if (m_inner)
        m_inner->endSegment(tool);
}

void CoordinatedMotionCommandSink::laserOn(const Tool& tool)
{
    const auto lease = m_coordinator.acquire();
    if (m_inner)
        m_inner->laserOn(tool);
}

void CoordinatedMotionCommandSink::laserOff(const Tool& tool)
{
    const auto lease = m_coordinator.acquire();
    if (m_inner)
        m_inner->laserOff(tool);
}

void CoordinatedMotionCommandSink::endProgram(const Tool& tool)
{
    const auto lease = m_coordinator.acquire();
    if (m_inner)
        m_inner->endProgram(tool);
}

void CoordinatedMotionCommandSink::applyToolMotionParams(
    const Tool& tool, bool jump)
{
    const auto lease = m_coordinator.acquire();
    if (m_inner)
        m_inner->applyToolMotionParams(tool, jump);
}

} // namespace lcnc::process
