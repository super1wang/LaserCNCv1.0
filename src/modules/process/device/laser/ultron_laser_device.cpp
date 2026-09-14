#include "modules/process/device/laser/ultron_laser_device.h"

#include "core/logging/logger.h"
#include "modules/process/device/laser/protocol/ultron_protocol.h"
#include "modules/process/device/process_device_log.h"

#include <QThread>
#include <cmath>
#include <limits>

namespace {

constexpr std::uint16_t kEnergyRegister = 0x0606;
constexpr std::uint16_t kAcoustoOpticRegister = 0x0607;
constexpr std::uint16_t kLaserEnableRegister = 0x0608;
constexpr std::uint16_t kExternalControlRegister = 0x060C;
constexpr std::uint16_t kLaserReadyRegister = 0x060D;
constexpr std::uint16_t kFrequencyRegister = 0x05DF;
constexpr std::uint16_t kPulseWidthRegister = 0x05E1;
constexpr std::uint16_t kLaserReadyValue = 100;
constexpr int kReadyPollCount = 30;
constexpr unsigned long kReadyPollIntervalMs = 100;

} // namespace

ULTRONLaserDevice::ULTRONLaserDevice(lcnc::process::ProcessSettingsService& settings)
    : LaserDevice(settings) {}

ErrorCode ULTRONLaserDevice::setLaserTable(const toml::table& laserTable) {
    bool connectionChanged = false;
    if (laserTable.count("ComSetting") != 0U) {
        const auto communication = processLaserTable(QStringLiteral("ComSetting"));
        if (SetComTable(communication, connectionChanged) != ErrorCode::ERROR_NONE) {
            m_initialized = false;
            return ErrorCode::ERROR_LASER_CONNECTIONFAILED;
        }
    }

    if (!initLaser()) {
        m_initialized = false;
        return ErrorCode::ERROR_LASER_CONNECTIONFAILED;
    }

    ErrorCode result = ErrorCode::ERROR_NONE;
    if (laserTable.count("Laser") != 0U || connectionChanged) {
        const toml::table parameters = connectionChanged
                                           ? processLaserTable(QStringLiteral("Laser"))
                                           : laserTable.at("Laser").as_table();
        double energy = m_dEnergy;
        double frequency = m_dFrequency;
        double pulseWidth = m_dPulseWidth;
        bool changed = false;
        if (parameters.count("fEnergy") != 0U) {
            energy = parameters.at("fEnergy").as_floating();
            changed = true;
        }
        if (parameters.count("fFrequency") != 0U) {
            frequency = parameters.at("fFrequency").as_floating();
            changed = true;
        }
        if (parameters.count("fPulseWidth") != 0U) {
            pulseWidth = parameters.at("fPulseWidth").as_floating();
            changed = true;
        }
        if (changed && !SetLaserParameter(LaserParameter(energy, frequency, pulseWidth)))
            result = ErrorCode::ERROR_LASER_SETTINGFAILED;
    }

    m_initialized = result == ErrorCode::ERROR_NONE;
    return result;
}

const std::string& ULTRONLaserDevice::GetName() {
    return m_name;
}

bool ULTRONLaserDevice::IsInited() {
    return m_initialized;
}

bool ULTRONLaserDevice::IsAvailableData(const QByteArray& data) {
    if (data.endsWith('\r'))
        return true;
    if (data.size() >= 7 && static_cast<std::uint8_t>(data.at(0)) == 0x01U &&
        static_cast<std::uint8_t>(data.at(1)) == 0x03U) {
        return lcnc::process::ultron::hasValidCrc(data);
    }
    if (data.size() == 8 && static_cast<std::uint8_t>(data.at(0)) == 0x01U &&
        static_cast<std::uint8_t>(data.at(1)) == 0x06U) {
        return lcnc::process::ultron::hasValidCrc(data);
    }
    return false;
}

bool ULTRONLaserDevice::StartLaser() {
    return writeRegister(kAcoustoOpticRegister, 1U);
}

bool ULTRONLaserDevice::StopLaser() {
    return writeRegister(kAcoustoOpticRegister, 0U);
}

bool ULTRONLaserDevice::SetEnergy(double energy) {
    std::uint16_t value = 0;
    if (energy < 0.0 || energy > 100.0 || !scaledRegisterValue(energy, 10.0, &value)) {
        lcnc::process::logDeviceError(
            ErrorCode::ERROR_LASER_SETTINGFAILED,
            QObject::tr("The laser energy %1 does not meet the requirement of 0 to 100.")
                .arg(energy)
                .toUtf8()
                .constData());
        return false;
    }
    if (!writeRegister(kEnergyRegister, value)) {
        lcnc::process::logDeviceError(
            ErrorCode::ERROR_LASER_SETTINGFAILED,
            QObject::tr("Set laser energy %1 failed.").arg(energy).toUtf8().constData());
        return false;
    }
    LaserDevice::UpdateEnergy(energy);
    return true;
}

bool ULTRONLaserDevice::SetFrequency(double frequency) {
    std::uint16_t value = 0;
    if (!scaledRegisterValue(frequency, 1000.0, &value) ||
        !writeRegister(kFrequencyRegister, value)) {
        lcnc::process::logDeviceError(
            ErrorCode::ERROR_LASER_SETTINGFAILED,
            QObject::tr("Set laser frequency %1 failed.").arg(frequency).toUtf8().constData());
        return false;
    }
    LaserDevice::UpdateFrequency(frequency);
    return true;
}

bool ULTRONLaserDevice::SetLaserParameter(const LaserParameter& parameter) {
    if (!SetPulseWidth(parameter.dPulseWidth) || !SetFrequency(parameter.dFrequency) ||
        !SetEnergy(parameter.dEnergy)) {
        return false;
    }
    LaserDevice::UpdateLaserParameter(parameter);
    return true;
}

bool ULTRONLaserDevice::SetPulseWidth(double pulseWidth) {
    std::uint16_t value = 0;
    if (!scaledRegisterValue(pulseWidth, 1.0, &value) ||
        !writeRegister(kPulseWidthRegister, value)) {
        lcnc::process::logDeviceError(ErrorCode::ERROR_LASER_SETTINGFAILED,
                                      QObject::tr("Set laser PulsePickerDivider %1 failed.")
                                          .arg(pulseWidth)
                                          .toUtf8()
                                          .constData());
        return false;
    }
    LaserDevice::UpdatePulseWidth(pulseWidth);
    return true;
}

std::string ULTRONLaserDevice::GetEnergy() {
    std::uint16_t value = 0;
    return readRegister(kEnergyRegister, &value) ? std::to_string(value)
                                                 : std::string("NotConnected");
}

std::string ULTRONLaserDevice::GetFrequency() {
    return "NotConnected";
}

std::string ULTRONLaserDevice::GetPulseWidth() {
    return "NotConnected";
}

bool ULTRONLaserDevice::initLaser() {
    if (!writeRegister(kLaserEnableRegister, 1U))
        return false;

    bool ready = false;
    for (int attempt = 0; attempt < kReadyPollCount; ++attempt) {
        std::uint16_t status = 0;
        if (!readRegister(kLaserReadyRegister, &status))
            return false;
        if (status == kLaserReadyValue) {
            ready = true;
            break;
        }
        QThread::msleep(kReadyPollIntervalMs);
    }
    if (!ready) {
        LCNC_ERR(lcnc::LogCode::Generic, "ULTRON laser did not become ready within {} ms",
                 kReadyPollCount * kReadyPollIntervalMs);
        return false;
    }
    return writeRegister(kExternalControlRegister, 1U);
}

bool ULTRONLaserDevice::writeRegister(std::uint16_t address, std::uint16_t value) {
    return OnceData(lcnc::process::ultron::makeWriteRegister(address, value));
}

bool ULTRONLaserDevice::readRegister(std::uint16_t address, std::uint16_t* value) {
    if (!value)
        return false;
    QByteArray response;
    return OnceData(lcnc::process::ultron::makeReadRegister(address), response) &&
           lcnc::process::ultron::parseRegisterValue(response, value);
}

bool ULTRONLaserDevice::scaledRegisterValue(double value, double scale,
                                            std::uint16_t* registerValue) noexcept {
    if (!registerValue || !std::isfinite(value) || !std::isfinite(scale) || value < 0.0 ||
        scale <= 0.0) {
        return false;
    }
    const double scaled = value * scale;
    if (!std::isfinite(scaled) ||
        scaled > static_cast<double>(std::numeric_limits<std::uint16_t>::max())) {
        return false;
    }
    *registerValue = static_cast<std::uint16_t>(std::lround(scaled));
    return true;
}
