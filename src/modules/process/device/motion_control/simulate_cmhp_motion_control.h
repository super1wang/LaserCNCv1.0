#pragma once

#if defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS

#include "acs_motion_control.h"

/// ACS Simulator controller.  It keeps the historical ACS command-program
/// surface so normal cutting can generate and execute ACSPL+ against the ACS
/// Simulator; failure is reported instead of changing the selected backend.
class SimulateCMHPMotionControl final : public ACSMotionControl
{
public:
    SimulateCMHPMotionControl(lcnc::process::ProcessSettingsService& settings,
                              lcnc::process::ProcessRuntimeConfiguration& runtimeConfiguration);
    ~SimulateCMHPMotionControl() override;

    bool Connect() override;
    bool Disconnect() override;
};

#else

#include "motion_control.h"

#include <map>

/// SDK-free local-state controller used by all-off builds.
class SimulateCMHPMotionControl final : public MotionControl
{
public:
    SimulateCMHPMotionControl(lcnc::process::ProcessSettingsService& settings,
                              lcnc::process::ProcessRuntimeConfiguration& runtimeConfiguration);
    ~SimulateCMHPMotionControl() override = default;

    void LogError() override;
    void CreateMotor(Axis axis, const table& axisTable) override;
    bool Connect() override; bool Reboot() override; bool Disconnect() override; bool IsConnected() override;
    bool Home(Axis axis) override; bool Home() override; bool StopHome() override; bool IsHomed() override; bool IsHomed(Axis axis) override;
    bool Enable(Axis axis) override; bool Enable() override; bool Disable(Axis axis) override; bool Disable() override; bool IsEnabled(Axis axis) override; bool IsEnabled() override; bool SetAxisEnable(Axis axis, bool enabled) override;
    bool Jog(Axis axis, bool direction, double velocity) override;
    bool MoveRelative(Axis axis, double position, double velocity) override;
    bool MoveMRelative(vector<Axis> axes, vector<double> positions, double velocity) override;
    bool MoveAbsolute(Axis axis, double position, double velocity) override;
    bool MoveMAbsolute(vector<Axis> axes, vector<double> positions, double velocity) override;
    bool StopMotion(Axis axis) override; bool StopMotion() override; bool HaltMotor(Axis axis) override;
    bool IsAxisMoving(Axis axis) override; bool IsAxisMoving() override;
    bool GetActualPos(Axis axis, double& position) override; bool GetFeedbackPos(Axis axis, double& position) override;
    bool SetFPosition(Axis axis, double position) override;
    bool IsReachPos(Axis axis, bool relative, double position) override;
    bool IsAxisStatusNormal(int& fault) override; bool ErrorOccurred() const override; bool IsQueueActive() override;
    bool SetAxisIndex(Axis axis, int value) override; bool SetAxisHomeBufferIndex(Axis axis, int value) override; bool SetAxisIsRotary(Axis axis, bool value) override; bool SetAxisResolution(Axis axis, int value) override; bool SetAxisTubeDiamater(Axis axis, double value) override; bool SetAxisVel(Axis axis, double value) override; bool SetAxisAcc(Axis axis, double value) override; bool SetAxisDec(Axis axis, double value) override; bool SetAxisJerk(Axis axis, double value) override; bool SetAxisNegLimit(Axis axis, double value) override; bool SetAxisPosLimit(Axis axis, double value) override; bool SetAxisVelAccDecJerk(Axis axis, double vel, double acc, double dec, double jerk) override; bool SetAxisSoftLimit(Axis axis, double neg, double pos) override;
    bool GetAxisIndex(Axis axis, int& value) override; bool GetAxisHomeBufferIndex(Axis axis, int& value) override; bool GetAxisIsRotary(Axis axis, bool& value) override; bool GetAxisResolution(Axis axis, int& value) override; bool GetAxisTubeDiamater(Axis axis, double& value) override; bool GetAxisVel(Axis axis, double& value) override; bool GetAxisAcc(Axis axis, double& value) override; bool GetAxisDec(Axis axis, double& value) override; bool GetAxisJerk(Axis axis, double& value) override; bool GetAxisNegLimit(Axis axis, double& value) override; bool GetAxisPosLimit(Axis axis, double& value) override; bool GetAxisVelAccDecJerk(Axis axis, double& vel, double& acc, double& dec, double& jerk) override; bool GetAxisSoftLimit(Axis axis, double& neg, double& pos) override; void ReadAxisSoftLimit(Axis axis, double& neg, double& pos) override;
    bool DigitalOutputSet(DigitalIOData& io, int value, bool logError = false) override; bool DigitalOutputGet(DigitalIOData& io, int& value, bool logError = false) override; bool DigitalInputGet(DigitalIOData& io, int& value, bool logError = false) override; bool AnalogOutputSet(AnalogIOData& io, double value, bool logError = false) override; bool AnalogOutputGet(AnalogIOData& io, double& value, bool logError = false) override; bool AnalogInputGet(AnalogIOData& io, double& value, bool logError = false) override;
    int GetPressureState() override; bool GetIsPressureState() override; void SetIsPressureState(bool value) override;
    bool IsBufferRunning(int index) override; bool StopBuffer(int index) override; bool StopAllBuffer() override; bool PauseBuffer(int index) override;
    const string& GetName() const override;

private:
    struct AxisState { int index{}, homeIndex{}, resolution{}; bool rotary{}, enabled{}, homed{}; double position{}, tubeDiameter{}, velocity{}, acceleration{}, deceleration{}, jerk{}, negativeLimit{}, positiveLimit{}; };
    AxisState& axis(Axis value);
    const AxisState* findAxis(Axis value) const;
    bool ready() const;
    bool m_connected{false};
    bool m_pressureState{false};
    bool m_bufferRunning{false};
    string m_name{"SimulatorCMHP"};
    std::map<Axis, AxisState> m_axes;
    std::map<QString, int> m_digitalOutputs;
    std::map<QString, double> m_analogOutputs;
};

#endif
