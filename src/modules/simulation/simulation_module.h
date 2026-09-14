#pragma once

#include "core/kernel/i_module.h"
#include "core/kernel/i_service.h"

#include <QObject>

#include <memory>

class QWidget;

namespace lcnc::simulation {

class SimulationSession;

/**
 * @brief Offline, CAM-only machine simulation facade.
 *
 * No Process service, controller SDK, laser, or serial device is used here.
 * A session owns all of its view and geometry state and is discarded on exit.
 */
class SimulationModule final : public QObject,
                               public lcnc::IModule,
                               public lcnc::IService
{
    Q_OBJECT
public:
    explicit SimulationModule(QObject* parent = nullptr);
    ~SimulationModule() override;

    lcnc::ModuleInfo info() const override;
    bool init(lcnc::IKernel& kernel) override;
    bool start() override;
    void stop() override;

    QWidget* enterSimulation();
    void exitSimulation();
    bool hasActiveSession() const;
    void run();
    void pause();
    void stopPlayback();
    void setSpeedMultiplier(double multiplier);
    void jumpToCollision(int direction);

signals:
    void sessionOpened(QWidget* page);
    void sessionClosed();
    /// The frozen CAM/machine snapshot changed and must not be reused.
    void sessionInvalidated();
    void operationFailed(const QString& title, const QString& message);

private:
    std::unique_ptr<SimulationSession> m_session;
};

} // namespace lcnc::simulation
