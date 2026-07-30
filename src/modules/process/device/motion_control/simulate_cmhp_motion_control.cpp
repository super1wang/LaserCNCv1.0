#include "simulate_cmhp_motion_control.h"

#include "core/logging/logger.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>

#if defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS
#include "ACSC.h"
#endif

#include <cmath>

#if defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS

SimulateCMHPMotionControl::SimulateCMHPMotionControl(
    lcnc::process::ProcessSettingsService& settings,
    lcnc::process::ProcessRuntimeConfiguration& runtimeConfiguration)
    : ACSMotionControl(settings, runtimeConfiguration)
{
    m_strName = "SimulatorCMHP";
}

SimulateCMHPMotionControl::~SimulateCMHPMotionControl()
{
    (void)Disconnect();
}

bool SimulateCMHPMotionControl::Connect()
{
    if (m_hHandle != ACSC_INVALID) {
        m_bConnectFlag = true;
        return true;
    }

    DeleteOtherConnections();
    m_hHandle = acsc_OpenCommSimulator();
    if (m_hHandle == ACSC_INVALID) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "SimulatorCMHP: acsc_OpenCommSimulator failed");
        return false;
    }

    const QString simulatorProgram = QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("Simulator.prg"));
    QByteArray simulatorProgramPath = QFile::encodeName(
        QDir::toNativeSeparators(simulatorProgram));
    if (!QFileInfo::exists(simulatorProgram)
        || !acsc_StopBuffer(m_hHandle, ACSC_NONE, nullptr)
        || !acsc_LoadBuffersFromFile(m_hHandle, simulatorProgramPath.data(), nullptr)
        || !AfterOpenComm()) {
        LCNC_ERR(lcnc::LogCode::Generic,
                 "SimulatorCMHP: initialization or program load failed: {}",
                 simulatorProgram.toStdString());
        (void)acsc_CloseComm(m_hHandle);
        (void)acsc_CloseSimulator();
        m_hHandle = ACSC_INVALID;
        m_bConnectFlag = false;
        return false;
    }

    LCNC_INFO(lcnc::LogCode::Generic,
              "SimulatorCMHP: ACS Simulator connected; ACSPL+ command sink is available");
    return true;
}

bool SimulateCMHPMotionControl::Disconnect()
{
    if (m_hHandle == ACSC_INVALID) {
        m_bConnectFlag = false;
        return true;
    }

    // ACS Simulator reports an invalid-parameter error when there is no
    // active buffer for ACSC_NONE.  Closing the simulator communication still
    // safely aborts that empty state, so do not turn a completed disconnect
    // into a false failure solely because there was nothing to stop.
    const int stopResult = acsc_StopBuffer(m_hHandle, ACSC_NONE, nullptr);
    const int closeResult = acsc_CloseComm(m_hHandle);
    const int closeSimulatorResult = acsc_CloseSimulator();
    const bool stopped = stopResult != 0;
    const bool closed = closeResult != 0;
    const bool simulatorClosed = closeSimulatorResult != 0;
    m_hHandle = ACSC_INVALID;
    m_bConnectFlag = false;
    if (!stopped) {
        LCNC_WARN(lcnc::LogCode::Generic,
                  "SimulatorCMHP: no active ACS buffer while disconnecting; communication was still closed");
    }
    if (!simulatorClosed) {
        LCNC_INFO(lcnc::LogCode::Generic,
                  "SimulatorCMHP: simulator process was already released by CloseComm (result={})",
                  closeSimulatorResult);
    }
    return closed;
}

#else

namespace {
constexpr double kPositionTolerance = 1e-9;
}

SimulateCMHPMotionControl::SimulateCMHPMotionControl(lcnc::process::ProcessSettingsService& settings,
                                                       lcnc::process::ProcessRuntimeConfiguration& runtimeConfiguration)
    : MotionControl(settings, runtimeConfiguration)
{
    m_name = "Simulator";
}

void SimulateCMHPMotionControl::LogError()
{
    LCNC_WARN(lcnc::LogCode::Generic, "SimulatorCMHP operation rejected because the controller is disconnected");
}

SimulateCMHPMotionControl::AxisState& SimulateCMHPMotionControl::axis(Axis value) { return m_axes[value]; }
const SimulateCMHPMotionControl::AxisState* SimulateCMHPMotionControl::findAxis(Axis value) const { const auto it = m_axes.find(value); return it == m_axes.end() ? nullptr : &it->second; }
bool SimulateCMHPMotionControl::ready() const { return m_connected; }

void SimulateCMHPMotionControl::CreateMotor(Axis value, const table&) { if (!IsMotorCreated(value)) m_vecMotors.push_back(value); (void)axis(value); }
bool SimulateCMHPMotionControl::Connect()
{
#if defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS
    if (!m_acsSimulatorHandle) {
        m_acsSimulatorHandle = acsc_OpenCommSimulator();
        if (m_acsSimulatorHandle == ACSC_INVALID) {
            LCNC_ERR(lcnc::LogCode::Generic, "SimulatorCMHP: acsc_OpenCommSimulator failed");
            m_acsSimulatorHandle = nullptr;
            return false;
        }
        if (!acsc_StopBuffer(static_cast<HANDLE>(m_acsSimulatorHandle), ACSC_NONE, nullptr)) {
            acsc_CloseComm(static_cast<HANDLE>(m_acsSimulatorHandle));
            acsc_CloseSimulator();
            m_acsSimulatorHandle = nullptr;
            return false;
        }
    }
#endif
    m_connected = true;
    rebuildAxes();
    return true;
}
bool SimulateCMHPMotionControl::Reboot() { return m_connected; }
bool SimulateCMHPMotionControl::Disconnect()
{
    m_bufferRunning = false;
#if defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS
    if (m_acsSimulatorHandle) {
        (void)acsc_StopBuffer(static_cast<HANDLE>(m_acsSimulatorHandle), ACSC_NONE, nullptr);
        (void)acsc_CloseComm(static_cast<HANDLE>(m_acsSimulatorHandle));
        (void)acsc_CloseSimulator();
        m_acsSimulatorHandle = nullptr;
    }
#endif
    m_connected = false;
    return true;
}
bool SimulateCMHPMotionControl::IsConnected() { return m_connected; }
bool SimulateCMHPMotionControl::Home(Axis value) { if (!ready()) return false; auto& s=axis(value); s.position=0; s.homed=true; return true; }
bool SimulateCMHPMotionControl::Home() { if (!ready()) return false; for (const auto value:m_vecMotors) Home(value); return true; }
bool SimulateCMHPMotionControl::StopHome() { return ready(); }
bool SimulateCMHPMotionControl::IsHomed() { for (const auto value:m_vecMotors) if (!IsHomed(value)) return false; return ready(); }
bool SimulateCMHPMotionControl::IsHomed(Axis value) { const auto* s=findAxis(value); return ready() && s && s->homed; }
bool SimulateCMHPMotionControl::Enable(Axis value) { if (!ready()) return false; axis(value).enabled=true; return true; }
bool SimulateCMHPMotionControl::Enable() { if (!ready()) return false; for (const auto value:m_vecMotors) Enable(value); return true; }
bool SimulateCMHPMotionControl::Disable(Axis value) { if (!ready()) return false; axis(value).enabled=false; return true; }
bool SimulateCMHPMotionControl::Disable() { if (!ready()) return false; for (const auto value:m_vecMotors) Disable(value); return true; }
bool SimulateCMHPMotionControl::IsEnabled(Axis value) { const auto* s=findAxis(value); return ready() && s && s->enabled; }
bool SimulateCMHPMotionControl::IsEnabled() { for (const auto value:m_vecMotors) if (!IsEnabled(value)) return false; return ready(); }
bool SimulateCMHPMotionControl::SetAxisEnable(Axis value,bool enabled) { return enabled ? Enable(value) : Disable(value); }
bool SimulateCMHPMotionControl::Jog(Axis value,bool direction,double velocity) { return MoveRelative(value, direction ? velocity : -velocity, velocity); }
bool SimulateCMHPMotionControl::MoveRelative(Axis value,double position,double) { if (!ready()) return false; axis(value).position += position; return true; }
bool SimulateCMHPMotionControl::MoveMRelative(vector<Axis> axes,vector<double> positions,double velocity) { if (!ready() || axes.size()!=positions.size()) return false; for(size_t i=0;i<axes.size();++i) MoveRelative(axes[i],positions[i],velocity); return true; }
bool SimulateCMHPMotionControl::MoveAbsolute(Axis value,double position,double) { if (!ready()) return false; axis(value).position=position; return true; }
bool SimulateCMHPMotionControl::MoveMAbsolute(vector<Axis> axes,vector<double> positions,double velocity) { if (!ready() || axes.size()!=positions.size()) return false; for(size_t i=0;i<axes.size();++i) MoveAbsolute(axes[i],positions[i],velocity); return true; }
bool SimulateCMHPMotionControl::StopMotion(Axis) { return ready(); } bool SimulateCMHPMotionControl::StopMotion() { return ready(); } bool SimulateCMHPMotionControl::HaltMotor(Axis value) { return StopMotion(value); }
bool SimulateCMHPMotionControl::IsAxisMoving(Axis) { return false; } bool SimulateCMHPMotionControl::IsAxisMoving() { return false; }
bool SimulateCMHPMotionControl::GetActualPos(Axis value,double& position) { const auto* s=findAxis(value); if(!ready()||!s) return false; position=s->position; return true; } bool SimulateCMHPMotionControl::GetFeedbackPos(Axis value,double& position) { return GetActualPos(value,position); }
bool SimulateCMHPMotionControl::IsReachPos(Axis value,bool relative,double position) { double actual{}; return GetActualPos(value,actual) && (relative ? std::abs(position) <= kPositionTolerance : std::abs(actual-position) <= kPositionTolerance); }
bool SimulateCMHPMotionControl::IsAxisStatusNormal(int& fault) { fault=0; return ready(); } bool SimulateCMHPMotionControl::ErrorOccurred() const { return false; } bool SimulateCMHPMotionControl::IsQueueActive() { return m_bufferRunning; }
#define SIM_SET(name, member, type) bool SimulateCMHPMotionControl::name(Axis value,type input){ axis(value).member=input; return ready(); }
#define SIM_GET(name, member, type) bool SimulateCMHPMotionControl::name(Axis value,type& output){ const auto* s=findAxis(value); if(!ready()||!s) return false; output=s->member; return true; }
SIM_SET(SetAxisIndex,index,int) SIM_SET(SetAxisHomeBufferIndex,homeIndex,int) SIM_SET(SetAxisIsRotary,rotary,bool) SIM_SET(SetAxisResolution,resolution,int) SIM_SET(SetAxisTubeDiamater,tubeDiameter,double) SIM_SET(SetAxisVel,velocity,double) SIM_SET(SetAxisAcc,acceleration,double) SIM_SET(SetAxisDec,deceleration,double) SIM_SET(SetAxisJerk,jerk,double) SIM_SET(SetAxisNegLimit,negativeLimit,double) SIM_SET(SetAxisPosLimit,positiveLimit,double)
SIM_GET(GetAxisIndex,index,int) SIM_GET(GetAxisHomeBufferIndex,homeIndex,int) SIM_GET(GetAxisIsRotary,rotary,bool) SIM_GET(GetAxisResolution,resolution,int) SIM_GET(GetAxisTubeDiamater,tubeDiameter,double) SIM_GET(GetAxisVel,velocity,double) SIM_GET(GetAxisAcc,acceleration,double) SIM_GET(GetAxisDec,deceleration,double) SIM_GET(GetAxisJerk,jerk,double) SIM_GET(GetAxisNegLimit,negativeLimit,double) SIM_GET(GetAxisPosLimit,positiveLimit,double)
#undef SIM_SET
#undef SIM_GET
bool SimulateCMHPMotionControl::SetAxisVelAccDecJerk(Axis value,double vel,double acc,double dec,double jerk) { auto& s=axis(value); s.velocity=vel;s.acceleration=acc;s.deceleration=dec;s.jerk=jerk;return ready(); }
bool SimulateCMHPMotionControl::SetAxisSoftLimit(Axis value,double neg,double pos) { auto& s=axis(value);s.negativeLimit=neg;s.positiveLimit=pos;return ready(); }
bool SimulateCMHPMotionControl::GetAxisVelAccDecJerk(Axis value,double& vel,double& acc,double& dec,double& jerk) { const auto* s=findAxis(value);if(!ready()||!s)return false;vel=s->velocity;acc=s->acceleration;dec=s->deceleration;jerk=s->jerk;return true; }
bool SimulateCMHPMotionControl::GetAxisSoftLimit(Axis value,double& neg,double& pos) { const auto* s=findAxis(value);if(!ready()||!s)return false;neg=s->negativeLimit;pos=s->positiveLimit;return true; }
void SimulateCMHPMotionControl::ReadAxisSoftLimit(Axis value,double& neg,double& pos) { if(!GetAxisSoftLimit(value,neg,pos)){neg=0;pos=0;} }
bool SimulateCMHPMotionControl::DigitalOutputSet(DigitalIOData& io,int value,bool) { if(!ready())return false;m_digitalOutputs[io.qstrID]=value;return true; }
bool SimulateCMHPMotionControl::DigitalOutputGet(DigitalIOData& io,int& value,bool) { if(!ready())return false;value=m_digitalOutputs[io.qstrID];return true; }
bool SimulateCMHPMotionControl::DigitalInputGet(DigitalIOData& io,int& value,bool) { return DigitalOutputGet(io,value,false); }
bool SimulateCMHPMotionControl::AnalogOutputSet(AnalogIOData& io,double value,bool) { if(!ready())return false;m_analogOutputs[io.qstrID]=value;return true; }
bool SimulateCMHPMotionControl::AnalogOutputGet(AnalogIOData& io,double& value,bool) { if(!ready())return false;value=m_analogOutputs[io.qstrID];return true; }
bool SimulateCMHPMotionControl::AnalogInputGet(AnalogIOData& io,double& value,bool) { return AnalogOutputGet(io,value,false); }
int SimulateCMHPMotionControl::GetPressureState() { return m_pressureState ? 1 : 0; } bool SimulateCMHPMotionControl::GetIsPressureState() { return m_pressureState; } void SimulateCMHPMotionControl::SetIsPressureState(bool value) { m_pressureState=value; }
bool SimulateCMHPMotionControl::IsBufferRunning(int) { return m_bufferRunning; } bool SimulateCMHPMotionControl::StopBuffer(int) { m_bufferRunning=false;return ready(); } bool SimulateCMHPMotionControl::StopAllBuffer() { m_bufferRunning=false;return ready(); } bool SimulateCMHPMotionControl::PauseBuffer(int) { return StopBuffer(0); }
const string& SimulateCMHPMotionControl::GetName() const { return m_name; }

#endif
