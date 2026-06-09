#include "modules/process/device/process_device_manager.h"

#include "core/kinematics/i_motion_controller.h"
#include "core/logging/logger.h"
#include "modules/process/controllers/simulation_motion_controller.h"
#include "modules/process/controllers/simulator_cmhp_motion_controller.h"
#include "modules/process/device/simulator_laser_device.h"
#include "modules/process/device/simulator_process_io.h"
#include "modules/process/settings/process_settings.h"

#if LCNC_PROCESS_HAS_ACS
#include "modules/process/controllers/acs_motion_controller_adapter.h"
#endif
#if LCNC_PROCESS_HAS_GTN
#include "modules/process/controllers/gtn_motion_controller_adapter.h"
#endif

#include <algorithm>
#include <utility>

namespace lcnc::process {
namespace {

class MotionControllerProcessDevice final : public IProcessDevice
{
public:
    void setDescriptor(QString descriptorName, QString displayName, QStringList capabilities)
    {
        m_descriptorName = std::move(descriptorName);
        m_displayName = std::move(displayName);
        m_capabilities = std::move(capabilities);
    }

    void setController(lcnc::IMotionController* controller)
    {
        m_controller = controller;
        if (m_controller && m_controller->isRunning()) {
            m_state = ProcessDeviceConnectionState::Connected;
            m_lastError.clear();
        } else if (m_state == ProcessDeviceConnectionState::Connected) {
            m_state = ProcessDeviceConnectionState::Disconnected;
        }
    }

    QString deviceId() const override { return m_descriptorName; }
    QString displayName() const override { return m_displayName.isEmpty() ? m_descriptorName : m_displayName; }
    ProcessDeviceKind kind() const override { return ProcessDeviceKind::MotionController; }
    QStringList capabilities() const override { return m_capabilities; }

    bool connectDevice(QString* errorMessage = nullptr) override
    {
        if (!m_controller) {
            m_state = ProcessDeviceConnectionState::Error;
            m_lastError = QStringLiteral("未创建活动运动控制器");
            if (errorMessage)
                *errorMessage = m_lastError;
            return false;
        }
        m_state = ProcessDeviceConnectionState::Connecting;
        if (!m_controller->start()) {
            m_state = ProcessDeviceConnectionState::Error;
            m_lastError = QStringLiteral("运动控制器启动失败");
            if (errorMessage)
                *errorMessage = m_lastError;
            return false;
        }
        m_state = ProcessDeviceConnectionState::Connected;
        m_lastError.clear();
        return true;
    }

    void disconnectDevice() override
    {
        m_state = ProcessDeviceConnectionState::Disconnecting;
        if (m_controller)
            m_controller->stop();
        m_state = ProcessDeviceConnectionState::Disconnected;
    }

    bool isConnected() const override { return m_controller && m_controller->isRunning(); }
    ProcessDeviceConnectionState connectionState() const override
    {
        if (m_state == ProcessDeviceConnectionState::Error)
            return m_state;
        return isConnected() ? ProcessDeviceConnectionState::Connected : ProcessDeviceConnectionState::Disconnected;
    }
    QString lastError() const override { return m_lastError; }
    QList<ProcessDeviceStatusItem> statusItems() const override
    {
        return {
            { QStringLiteral("ControllerId"), m_controller ? m_controller->id() : QStringLiteral("<none>") },
            { QStringLiteral("Running"), isConnected() ? QStringLiteral("true") : QStringLiteral("false") }
        };
    }

private:
    QString m_descriptorName{QStringLiteral("PureSimulation")};
    QString m_displayName{QStringLiteral("Pure Software Simulation")};
    QStringList m_capabilities;
    lcnc::IMotionController* m_controller{nullptr};
    ProcessDeviceConnectionState m_state{ProcessDeviceConnectionState::Disconnected};
    QString m_lastError;
};

ProcessDeviceRole defaultRoleFor(ProcessDeviceKind kind)
{
    switch (kind) {
    case ProcessDeviceKind::MotionController: return ProcessDeviceRole::ActiveMotion;
    case ProcessDeviceKind::Laser: return ProcessDeviceRole::ActiveLaser;
    case ProcessDeviceKind::Io: return ProcessDeviceRole::ActiveIo;
    case ProcessDeviceKind::Aux: return ProcessDeviceRole::None;
    }
    return ProcessDeviceRole::None;
}

} // namespace

ProcessDeviceManager::ProcessDeviceManager()
    : m_activeMotionController(QStringLiteral("PureSimulation"))
    , m_activeLaserDevice(QStringLiteral("Simulator"))
{
    registerBuiltInDevices();
    m_laserDevice = std::make_unique<SimulatorLaserDevice>();
    m_processIo = std::make_unique<SimulatorProcessIo>();
    m_motionDevice = std::make_unique<MotionControllerProcessDevice>();
    updateMotionDeviceDescriptor();
    updateLaserDeviceDescriptor();
    registerDefaultSessions();
}

ProcessDeviceManager::~ProcessDeviceManager() = default;

QVector<ProcessDeviceDescriptor> ProcessDeviceManager::descriptors(ProcessDeviceKind kind) const
{
    QVector<ProcessDeviceDescriptor> result;
    for (const auto& descriptor : m_descriptors) {
        if (descriptor.kind == kind)
            result.append(descriptor);
    }
    return result;
}

QStringList ProcessDeviceManager::availableMotionControllers() const
{
    return names(ProcessDeviceKind::MotionController);
}

QStringList ProcessDeviceManager::availableLaserDevices() const
{
    return names(ProcessDeviceKind::Laser);
}

void ProcessDeviceManager::setMotionController(lcnc::IMotionController* controller)
{
    m_motionController = controller;
    if (auto* motionDevice = dynamic_cast<MotionControllerProcessDevice*>(m_motionDevice.get()))
        motionDevice->setController(controller);
    for (ProcessDeviceSession& session : m_sessions) {
        if (session.kind == ProcessDeviceKind::MotionController) {
            session.connectionState = m_motionDevice ? m_motionDevice->connectionState() : ProcessDeviceConnectionState::Disconnected;
            session.lastError = m_motionDevice ? m_motionDevice->lastError() : QString();
        }
    }
}

void ProcessDeviceManager::syncFromSettings(const lcnc::ProcessSettings& settings)
{
    const QString motion = settings.motionControllerName().trimmed();
    m_activeMotionController = isDeviceUsable(ProcessDeviceKind::MotionController, motion)
        ? motion
        : QStringLiteral("PureSimulation");

    const QString laser = settings.laserDeviceName().trimmed();
    m_activeLaserDevice = isLaserDeviceAvailable(laser)
        ? laser
        : QStringLiteral("Simulator");

    if (!isDeviceUsable(ProcessDeviceKind::MotionController, m_activeMotionController)) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "process.device: motion controller '{}' is registered but unavailable",
                  m_activeMotionController.toStdString());
    }
    if (!isDeviceUsable(ProcessDeviceKind::Laser, m_activeLaserDevice)) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "process.device: laser device '{}' is registered but unavailable",
                  m_activeLaserDevice.toStdString());
    }

    updateMotionDeviceDescriptor();
    updateLaserDeviceDescriptor();

    if (auto* laser = laserDevice()) {
        laser->setEnergy(settings.laserEnergy());
        laser->setFrequency(settings.laserFrequency());
        laser->setPulseWidth(settings.laserPulseWidth());
    }
}

IProcessDevice* ProcessDeviceManager::processDevice(const QString& instanceId) const
{
    const auto* session = sessionById(instanceId);
    return session ? runtimeDeviceForSession(*session) : nullptr;
}

QString ProcessDeviceManager::addDevice(ProcessDeviceKind kind, const QString& descriptorName, QString* errorMessage)
{
    const ProcessDeviceDescriptor* item = descriptor(kind, descriptorName);
    if (!item) {
        if (errorMessage)
            *errorMessage = QStringLiteral("未注册外设: %1").arg(descriptorName);
        return QString();
    }

    QString instanceId = makeInstanceId(kind, item->name);
    int suffix = 2;
    while (sessionById(instanceId))
        instanceId = QStringLiteral("%1#%2").arg(makeInstanceId(kind, item->name)).arg(suffix++);

    ProcessDeviceSession session;
    session.instanceId = instanceId;
    session.kind = kind;
    session.role = defaultRoleFor(kind);
    session.descriptorName = item->name;
    session.displayName = item->displayName;
    session.connectionState = ProcessDeviceConnectionState::Disconnected;
    m_sessions.append(session);
    LCNC_INFO(lcnc::LogCode::Generic,
              "process.device: added instance='{}' descriptor='{}'",
              instanceId.toStdString(),
              item->name.toStdString());
    return instanceId;
}

bool ProcessDeviceManager::removeDevice(const QString& instanceId, QString* errorMessage)
{
    for (int index = 0; index < m_sessions.size(); ++index) {
        ProcessDeviceSession& session = m_sessions[index];
        if (session.instanceId != instanceId)
            continue;
        if (session.role == ProcessDeviceRole::ActiveMotion || session.role == ProcessDeviceRole::ActiveLaser) {
            if (errorMessage)
                *errorMessage = QStringLiteral("活动运动控制器或激光器不能删除");
            return false;
        }
        disconnectDevice(instanceId);
        m_sessions.removeAt(index);
        LCNC_INFO(lcnc::LogCode::Generic,
                  "process.device: removed instance='{}'",
                  instanceId.toStdString());
        return true;
    }

    if (errorMessage)
        *errorMessage = QStringLiteral("未找到外设实例: %1").arg(instanceId);
    return false;
}

bool ProcessDeviceManager::connectDevice(const QString& instanceId, QString* errorMessage)
{
    ProcessDeviceSession* session = sessionById(instanceId);
    if (!session) {
        if (errorMessage)
            *errorMessage = QStringLiteral("未找到外设实例: %1").arg(instanceId);
        return false;
    }
    if (!session->enabled)
        return true;

    const ProcessDeviceDescriptor* item = descriptor(session->kind, session->descriptorName);
    if (!item || item->availability == ProcessDeviceAvailability::Unavailable) {
        session->connectionState = ProcessDeviceConnectionState::Error;
        session->lastError = item && !item->unavailableReason.isEmpty()
            ? item->unavailableReason
            : QStringLiteral("外设在当前构建中不可用: %1").arg(session->descriptorName);
        if (errorMessage)
            *errorMessage = session->lastError;
        return false;
    }

    IProcessDevice* device = runtimeDeviceForSession(*session);
    if (!device) {
        session->connectionState = ProcessDeviceConnectionState::Error;
        session->lastError = QStringLiteral("外设尚未实现运行时适配器: %1").arg(session->descriptorName);
        if (errorMessage)
            *errorMessage = session->lastError;
        return false;
    }

    session->connectionState = ProcessDeviceConnectionState::Connecting;
    QString localError;
    if (!device->connectDevice(&localError)) {
        session->connectionState = ProcessDeviceConnectionState::Error;
        session->lastError = localError.isEmpty() ? device->lastError() : localError;
        if (errorMessage)
            *errorMessage = session->lastError;
        LCNC_WARN(lcnc::LogCode::Generic,
                  "process.device: connect failed instance='{}' error='{}'",
                  instanceId.toStdString(),
                  session->lastError.toStdString());
        return false;
    }

    if (session->kind == ProcessDeviceKind::MotionController && m_processIo) {
        QString ioError;
        if (!m_processIo->connectDevice(&ioError)) {
            device->disconnectDevice();
            session->connectionState = ProcessDeviceConnectionState::Error;
            session->lastError = ioError.isEmpty() ? m_processIo->lastError() : ioError;
            if (session->lastError.isEmpty())
                session->lastError = QStringLiteral("运动控制器 IO 连接失败");
            if (errorMessage)
                *errorMessage = session->lastError;
            LCNC_WARN(lcnc::LogCode::Generic,
                      "process.device: motion io connect failed instance='{}' error='{}'",
                      instanceId.toStdString(),
                      session->lastError.toStdString());
            return false;
        }
        LCNC_INFO(lcnc::LogCode::Generic,
                  "process.device: motion io connected with instance='{}'",
                  instanceId.toStdString());
    }

    session->connectionState = device->connectionState();
    session->lastError.clear();
    LCNC_INFO(lcnc::LogCode::Generic,
              "process.device: connected instance='{}'",
              instanceId.toStdString());
    return true;
}

void ProcessDeviceManager::disconnectDevice(const QString& instanceId)
{
    ProcessDeviceSession* session = sessionById(instanceId);
    if (!session)
        return;
    IProcessDevice* device = runtimeDeviceForSession(*session);
    session->connectionState = ProcessDeviceConnectionState::Disconnecting;
    if (session->kind == ProcessDeviceKind::MotionController && m_processIo)
        m_processIo->disconnectDevice();
    if (device)
        device->disconnectDevice();
    session->connectionState = ProcessDeviceConnectionState::Disconnected;
    session->lastError.clear();
    LCNC_INFO(lcnc::LogCode::Generic,
              "process.device: disconnected instance='{}'",
              instanceId.toStdString());
}

bool ProcessDeviceManager::connectAllDevices(QStringList* errorMessages, ProcessDeviceConnectProgress progress)
{
    const int total = std::count_if(m_sessions.cbegin(), m_sessions.cend(), [](const ProcessDeviceSession& session) {
        return session.enabled;
    });
    int current = 0;
    bool allConnected = true;
    for (const ProcessDeviceSession& session : std::as_const(m_sessions)) {
        if (!session.enabled)
            continue;
        ++current;
        if (progress)
            progress(current, total, QStringLiteral("连接外设: %1").arg(session.displayName));
        QString error;
        if (!connectDevice(session.instanceId, &error)) {
            allConnected = false;
            if (errorMessages)
                errorMessages->append(QStringLiteral("%1: %2").arg(session.displayName, error));
        }
    }
    return allConnected;
}

void ProcessDeviceManager::disconnectAllDevices()
{
    for (const ProcessDeviceSession& session : std::as_const(m_sessions)) {
        if (session.enabled)
            disconnectDevice(session.instanceId);
    }
}

std::unique_ptr<lcnc::IMotionController> ProcessDeviceManager::createMotionController(
    const lcnc::ProcessSettings& settings,
    QObject* parent,
    QString* errorMessage) const
{
    const QString active = normalizedName(m_activeMotionController);
    if (active == normalizedName(QStringLiteral("PureSimulation")) || active == normalizedName(QStringLiteral("sim")))
        return std::make_unique<SimulationMotionController>(parent);

    if (active == normalizedName(QStringLiteral("SimulatorCMHP"))) {
#if LCNC_PROCESS_HAS_ACS
        return std::make_unique<AcsSimulatorCmhpControllerAdapter>(settings.controllerEndpoint(), parent);
#else
        return std::make_unique<SimulatorCmhpMotionController>(parent);
#endif
    }
#if LCNC_PROCESS_HAS_ACS
    if (active == normalizedName(QStringLiteral("ACS")))
        return std::make_unique<AcsMotionControllerAdapter>(settings.controllerEndpoint(), false, parent);
#endif

#if LCNC_PROCESS_HAS_GTN
    if (active == normalizedName(QStringLiteral("GTN")))
        return std::make_unique<GtnMotionControllerAdapter>(parent);
#endif

    if (errorMessage) {
        const auto* item = descriptor(ProcessDeviceKind::MotionController, m_activeMotionController);
        *errorMessage = item && item->availability == ProcessDeviceAvailability::Unavailable
            ? QStringLiteral("Motion controller '%1' is unavailable in this build").arg(m_activeMotionController)
            : QStringLiteral("Unknown motion controller '%1'").arg(m_activeMotionController);
    }
    return nullptr;
}

bool ProcessDeviceManager::isMotionControllerAvailable(const QString& name) const
{
    return descriptor(ProcessDeviceKind::MotionController, name) != nullptr;
}

bool ProcessDeviceManager::isLaserDeviceAvailable(const QString& name) const
{
    return descriptor(ProcessDeviceKind::Laser, name) != nullptr;
}

bool ProcessDeviceManager::isDeviceUsable(ProcessDeviceKind kind, const QString& name) const
{
    const auto* item = descriptor(kind, name);
    return item && item->availability != ProcessDeviceAvailability::Unavailable;
}

const ProcessDeviceDescriptor* ProcessDeviceManager::descriptor(ProcessDeviceKind kind, const QString& name) const
{
    const QString expected = normalizedName(name);
    for (const auto& item : m_descriptors) {
        if (item.kind == kind && normalizedName(item.name) == expected)
            return &item;
    }
    return nullptr;
}

void ProcessDeviceManager::registerBuiltInDevices()
{
    m_descriptors = {
                { ProcessDeviceKind::MotionController,
                    QStringLiteral("PureSimulation"),
                    QStringLiteral("Pure Software Simulation"),
                    ProcessDeviceAvailability::Simulation,
                    { QStringLiteral("axis"), QStringLiteral("jog"), QStringLiteral("home"), QStringLiteral("no-sdk") },
                    {},
                    QStringLiteral("Setting_MotionControl") },
                { ProcessDeviceKind::MotionController,
                    QStringLiteral("SimulatorCMHP"),
                    QStringLiteral("CMHP Simulator"),
                    ProcessDeviceAvailability::Simulation,
                                        { QStringLiteral("axis"), QStringLiteral("jog"), QStringLiteral("home"), QStringLiteral("io"), QStringLiteral("program-buffer"), QStringLiteral("simulation-mode"),
#if LCNC_PROCESS_HAS_ACS
                                          QStringLiteral("acs-simulator")
#else
                                          QStringLiteral("software-simulator")
#endif
                                        },
                                        {},
                                        QStringLiteral("Setting_MotionControl") },
        { ProcessDeviceKind::MotionController,
          QStringLiteral("ACS"),
          QStringLiteral("ACS Motion Controller"),
#if LCNC_PROCESS_HAS_ACS
                    ProcessDeviceAvailability::Available,
#else
          ProcessDeviceAvailability::Unavailable,
#endif
                    { QStringLiteral("axis"), QStringLiteral("io"), QStringLiteral("laser-table") },
#if LCNC_PROCESS_HAS_ACS
                    {},
#else
                    QStringLiteral("ACS SDK 未启用"),
#endif
                    QStringLiteral("Setting_MotionControl") },
        { ProcessDeviceKind::MotionController,
          QStringLiteral("GTN"),
          QStringLiteral("GTN Motion Controller"),
#if LCNC_PROCESS_HAS_GTN
                    ProcessDeviceAvailability::Available,
#else
          ProcessDeviceAvailability::Unavailable,
#endif
          { QStringLiteral("axis"), QStringLiteral("io") },
#if LCNC_PROCESS_HAS_GTN
          {},
#else
          QStringLiteral("GTN SDK 未启用"),
#endif
          QStringLiteral("Setting_MotionControl") },
        { ProcessDeviceKind::Laser,
          QStringLiteral("Simulator"),
          QStringLiteral("Laser Simulator"),
          ProcessDeviceAvailability::Simulation,
          { QStringLiteral("energy"), QStringLiteral("frequency"), QStringLiteral("pulse-width") },
          {},
          QStringLiteral("Setting_Laser") },
        { ProcessDeviceKind::Laser,
          QStringLiteral("IPG"),
          QStringLiteral("IPG Laser"),
          ProcessDeviceAvailability::Unavailable,
          { QStringLiteral("energy"), QStringLiteral("monitor") },
          QStringLiteral("真实激光器适配器尚未启用"),
          QStringLiteral("Setting_Laser") },
        { ProcessDeviceKind::Laser,
          QStringLiteral("Raycus"),
          QStringLiteral("Raycus Laser"),
          ProcessDeviceAvailability::Unavailable,
          { QStringLiteral("energy"), QStringLiteral("frequency"), QStringLiteral("pulse-width") },
          QStringLiteral("真实激光器适配器尚未启用"),
          QStringLiteral("Setting_Laser") },
        { ProcessDeviceKind::Laser,
          QStringLiteral("RaycusQCW"),
          QStringLiteral("Raycus QCW Laser"),
          ProcessDeviceAvailability::Unavailable,
          { QStringLiteral("energy"), QStringLiteral("frequency"), QStringLiteral("pulse-width") },
          QStringLiteral("真实激光器适配器尚未启用"),
          QStringLiteral("Setting_Laser") },
        { ProcessDeviceKind::Laser,
          QStringLiteral("Pharos"),
          QStringLiteral("Pharos Laser"),
          ProcessDeviceAvailability::Unavailable,
          { QStringLiteral("divider"), QStringLiteral("energy") },
          QStringLiteral("真实激光器适配器尚未启用"),
          QStringLiteral("Setting_Laser") },
        { ProcessDeviceKind::Laser,
          QStringLiteral("Analog"),
          QStringLiteral("Analog Laser"),
          ProcessDeviceAvailability::Unavailable,
          { QStringLiteral("analog-output"), QStringLiteral("energy") },
          QStringLiteral("真实激光器适配器尚未启用"),
          QStringLiteral("Setting_Laser") },
        { ProcessDeviceKind::Laser,
          QStringLiteral("ULTRON"),
          QStringLiteral("ULTRON Laser"),
          ProcessDeviceAvailability::Unavailable,
                    { QStringLiteral("energy"), QStringLiteral("frequency") },
                    QStringLiteral("真实激光器适配器尚未启用"),
                    QStringLiteral("Setting_Laser") },
                { ProcessDeviceKind::Io,
                    QStringLiteral("SimulatorIO"),
                    QStringLiteral("Simulator IO"),
                    ProcessDeviceAvailability::Simulation,
                    { QStringLiteral("digital-in"), QStringLiteral("digital-out"), QStringLiteral("analog-in"), QStringLiteral("analog-out") },
                    {},
                    QStringLiteral("Setting_IOIndex") },
                { ProcessDeviceKind::Io,
                    QStringLiteral("BDAQ"),
                    QStringLiteral("BDAQ IO"),
                    ProcessDeviceAvailability::Unavailable,
                    { QStringLiteral("digital-in"), QStringLiteral("digital-out"), QStringLiteral("analog-in"), QStringLiteral("analog-out") },
                    QStringLiteral("BDAQ SDK 未启用"),
                    QStringLiteral("Setting_IOIndex") }
    };
}

void ProcessDeviceManager::registerDefaultSessions()
{
    addDevice(ProcessDeviceKind::MotionController, QStringLiteral("PureSimulation"));
    addDevice(ProcessDeviceKind::Laser, QStringLiteral("Simulator"));
}

void ProcessDeviceManager::updateMotionDeviceDescriptor()
{
    const auto* item = descriptor(ProcessDeviceKind::MotionController, m_activeMotionController);
    if (auto* motionDevice = dynamic_cast<MotionControllerProcessDevice*>(m_motionDevice.get())) {
        motionDevice->setDescriptor(m_activeMotionController,
                                    item ? item->displayName : m_activeMotionController,
                                    item ? item->capabilities : QStringList{});
        motionDevice->setController(m_motionController);
    }
    for (ProcessDeviceSession& session : m_sessions) {
        if (session.kind != ProcessDeviceKind::MotionController)
            continue;
        session.descriptorName = m_activeMotionController;
        session.displayName = item ? item->displayName : m_activeMotionController;
        session.instanceId = makeInstanceId(ProcessDeviceKind::MotionController, m_activeMotionController);
    }
}

void ProcessDeviceManager::updateLaserDeviceDescriptor()
{
    const auto* item = descriptor(ProcessDeviceKind::Laser, m_activeLaserDevice);
    for (ProcessDeviceSession& session : m_sessions) {
        if (session.kind != ProcessDeviceKind::Laser || session.role != ProcessDeviceRole::ActiveLaser)
            continue;
        session.descriptorName = m_activeLaserDevice;
        session.displayName = item ? item->displayName : m_activeLaserDevice;
        session.instanceId = makeInstanceId(ProcessDeviceKind::Laser, m_activeLaserDevice);
        if (auto* laser = m_laserDevice.get()) {
            session.connectionState = laser->connectionState();
            session.lastError = laser->lastError();
        }
    }
}

ProcessDeviceSession* ProcessDeviceManager::sessionById(const QString& instanceId)
{
    for (ProcessDeviceSession& session : m_sessions) {
        if (session.instanceId == instanceId)
            return &session;
    }
    return nullptr;
}

const ProcessDeviceSession* ProcessDeviceManager::sessionById(const QString& instanceId) const
{
    for (const ProcessDeviceSession& session : m_sessions) {
        if (session.instanceId == instanceId)
            return &session;
    }
    return nullptr;
}

IProcessDevice* ProcessDeviceManager::runtimeDeviceForSession(const ProcessDeviceSession& session) const
{
    switch (session.kind) {
    case ProcessDeviceKind::MotionController:
        return m_motionDevice.get();
    case ProcessDeviceKind::Laser:
        return normalizedName(session.descriptorName) == normalizedName(QStringLiteral("Simulator"))
            ? m_laserDevice.get()
            : nullptr;
    case ProcessDeviceKind::Io:
        return normalizedName(session.descriptorName) == normalizedName(QStringLiteral("SimulatorIO"))
            ? m_processIo.get()
            : nullptr;
    case ProcessDeviceKind::Aux:
        return nullptr;
    }
    return nullptr;
}

QStringList ProcessDeviceManager::names(ProcessDeviceKind kind) const
{
    QStringList result;
    for (const auto& item : m_descriptors) {
        if (item.kind == kind)
            result.append(item.name);
    }
    return result;
}

QString ProcessDeviceManager::normalizedName(const QString& name) const
{
    return name.trimmed().toCaseFolded();
}

QString ProcessDeviceManager::makeInstanceId(ProcessDeviceKind kind, const QString& descriptorName) const
{
    return QStringLiteral("%1:%2").arg(processDeviceKindName(kind), descriptorName.trimmed());
}

} // namespace lcnc::process
