#pragma once

#if defined(LCNC_PROCESS_HAS_ACS) && LCNC_PROCESS_HAS_ACS

#include "acs_motion_control.h"

/// ACS Simulator controller.  It keeps the historical ACS command-program
/// surface so normal cutting can generate and execute ACSPL+ against the ACS
/// Simulator; failure is reported instead of changing the selected backend.
class SimulateCMHPMotionControl final : public ACSMotionControl {
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
class SimulateCMHPMotionControl final : public MotionControl {
  public:
    SimulateCMHPMotionControl(lcnc::process::ProcessSettingsService& settings,
                              lcnc::process::ProcessRuntimeConfiguration& runtimeConfiguration);
    ~SimulateCMHPMotionControl() override = default;

    void LogError() override;
    void CreateMotor(lcnc::process::Axis axis, const toml::table& axisTable) override;
    bool Connect() override;
    bool Reboot() override;
    bool Disconnect() override;
    bool IsConnected() override;
    bool Home(lcnc::process::Axis axis) override;
    bool Home() override;
    bool StopHome() override;
    bool IsHomed() override;
    bool IsHomed(lcnc::process::Axis axis) override;
    bool Enable(lcnc::process::Axis axis) override;
    bool Enable() override;
    bool Disable(lcnc::process::Axis axis) override;
    bool Disable() override;
    bool IsEnabled(lcnc::process::Axis axis) override;
    bool IsEnabled() override;
    bool SetAxisEnable(lcnc::process::Axis axis, bool enabled) override;
    bool Jog(lcnc::process::Axis axis, bool direction, double velocity) override;
    bool MoveRelative(lcnc::process::Axis axis, double position, double velocity) override;
    bool MoveMRelative(std::vector<lcnc::process::Axis> axes, std::vector<double> positions,
                       double velocity) override;
    bool MoveAbsolute(lcnc::process::Axis axis, double position, double velocity) override;
    bool MoveMAbsolute(std::vector<lcnc::process::Axis> axes, std::vector<double> positions,
                       double velocity) override;
    bool StopMotion(lcnc::process::Axis axis) override;
    bool StopMotion() override;
    bool HaltMotor(lcnc::process::Axis axis) override;
    bool IsAxisMoving(lcnc::process::Axis axis) override;
    bool IsAxisMoving() override;
    bool GetActualPos(lcnc::process::Axis axis, double& position) override;
    bool GetFeedbackPos(lcnc::process::Axis axis, double& position) override;
    bool SetFPosition(lcnc::process::Axis axis, double position) override;
    bool IsReachPos(lcnc::process::Axis axis, bool relative, double position) override;
    bool IsAxisStatusNormal(int& fault) override;
    bool ErrorOccurred() const override;
    bool IsQueueActive() override;
    bool SetAxisIndex(lcnc::process::Axis axis, int value) override;
    bool SetAxisHomeBufferIndex(lcnc::process::Axis axis, int value) override;
    bool SetAxisIsRotary(lcnc::process::Axis axis, bool value) override;
    bool SetAxisResolution(lcnc::process::Axis axis, int value) override;
    bool SetAxisTubeDiamater(lcnc::process::Axis axis, double value) override;
    bool SetAxisVel(lcnc::process::Axis axis, double value) override;
    bool SetAxisAcc(lcnc::process::Axis axis, double value) override;
    bool SetAxisDec(lcnc::process::Axis axis, double value) override;
    bool SetAxisJerk(lcnc::process::Axis axis, double value) override;
    bool SetAxisNegLimit(lcnc::process::Axis axis, double value) override;
    bool SetAxisPosLimit(lcnc::process::Axis axis, double value) override;
    bool SetAxisVelAccDecJerk(lcnc::process::Axis axis, double vel, double acc, double dec,
                              double jerk) override;
    bool SetAxisSoftLimit(lcnc::process::Axis axis, double neg, double pos) override;
    bool GetAxisIndex(lcnc::process::Axis axis, int& value) override;
    bool GetAxisHomeBufferIndex(lcnc::process::Axis axis, int& value) override;
    bool GetAxisIsRotary(lcnc::process::Axis axis, bool& value) override;
    bool GetAxisResolution(lcnc::process::Axis axis, int& value) override;
    bool GetAxisTubeDiamater(lcnc::process::Axis axis, double& value) override;
    bool GetAxisVel(lcnc::process::Axis axis, double& value) override;
    bool GetAxisAcc(lcnc::process::Axis axis, double& value) override;
    bool GetAxisDec(lcnc::process::Axis axis, double& value) override;
    bool GetAxisJerk(lcnc::process::Axis axis, double& value) override;
    bool GetAxisNegLimit(lcnc::process::Axis axis, double& value) override;
    bool GetAxisPosLimit(lcnc::process::Axis axis, double& value) override;
    bool GetAxisVelAccDecJerk(lcnc::process::Axis axis, double& vel, double& acc, double& dec,
                              double& jerk) override;
    bool GetAxisSoftLimit(lcnc::process::Axis axis, double& neg, double& pos) override;
    void ReadAxisSoftLimit(lcnc::process::Axis axis, double& neg, double& pos) override;
    bool DigitalOutputSet(DigitalIOData& io, int value, bool logError = false) override;
    bool DigitalOutputGet(DigitalIOData& io, int& value, bool logError = false) override;
    bool DigitalInputGet(DigitalIOData& io, int& value, bool logError = false) override;
    bool AnalogOutputSet(AnalogIOData& io, double value, bool logError = false) override;
    bool AnalogOutputGet(AnalogIOData& io, double& value, bool logError = false) override;
    bool AnalogInputGet(AnalogIOData& io, double& value, bool logError = false) override;
    int GetPressureState() override;
    bool GetIsPressureState() override;
    void SetIsPressureState(bool value) override;
    bool IsBufferRunning(int index) override;
    bool StopBuffer(int index) override;
    bool StopAllBuffer() override;
    bool PauseBuffer(int index) override;
    const std::string& GetName() const override;

  private:
    struct AxisState {
        int index{}, homeIndex{}, resolution{};
        bool rotary{}, enabled{}, homed{};
        double position{}, tubeDiameter{}, velocity{}, acceleration{}, deceleration{}, jerk{},
            negativeLimit{}, positiveLimit{};
    };
    AxisState& axis(lcnc::process::Axis value);
    const AxisState* findAxis(lcnc::process::Axis value) const;
    bool ready() const;
    bool m_connected{false};
    bool m_pressureState{false};
    bool m_bufferRunning{false};
    std::string m_name{"SimulatorCMHP"};
    std::map<lcnc::process::Axis, AxisState> m_axes;
    std::map<QString, int> m_digitalOutputs;
    std::map<QString, double> m_analogOutputs;
};

#endif
