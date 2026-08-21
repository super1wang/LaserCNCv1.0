#pragma once

#include "modules/process/device/laser/laser_device.h"

#include <cstdint>
#include <string>

/**
 * @brief Minimal, fail-closed ULTRON serial adapter.
 *
 * Protocol framing lives in protocol/ultron_protocol.* so it can be tested
 * without a serial port or physical laser.
 */
class ULTRONLaserDevice final : public LaserDevice {
  public:
    explicit ULTRONLaserDevice(lcnc::process::ProcessSettingsService& settings);

    ErrorCode setLaserTable(const toml::table& laserTable = {}) override;
    const std::string& GetName() override;
    bool IsInited() override;
    bool IsAvailableData(const QByteArray& data) override;
    bool StartLaser() override;
    bool StopLaser() override;

    bool SetEnergy(double energy) override;
    bool SetFrequency(double frequency) override;
    bool SetLaserParameter(const LaserParameter& parameter) override;
    bool SetPulseWidth(double pulseWidth) override;

    std::string GetEnergy() override;
    std::string GetFrequency() override;
    std::string GetPulseWidth() override;

    bool StartAimingBeam() override {
        return false;
    }
    bool StopAimingBeam() override {
        return false;
    }
    double GetAveragePower() override {
        return 0.0;
    }
    std::string GetTemperature() override {
        return {};
    }
    std::string GetTroubleshooting() override {
        return {};
    }

  private:
    bool initLaser();
    bool writeRegister(std::uint16_t address, std::uint16_t value);
    bool readRegister(std::uint16_t address, std::uint16_t* value);
    static bool scaledRegisterValue(double value, double scale,
                                    std::uint16_t* registerValue) noexcept;

    std::string m_name{"ULTRON"};
    bool m_initialized{false};
};
