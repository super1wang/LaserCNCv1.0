/************************************************************************/
/*                            锐科激光器实现类                           */
/************************************************************************/

#include "raycus_laser_device.h"

#include "modules/process/device/process_device_log.h"

#include <cmath>
#include <cstdint>
#include <stdlib.h>
#include <string>
// #include <cstringt.h>
#include <boost/lexical_cast.hpp>
#include <sstream>
using std::ostringstream;
using namespace std;
using toml::table;

RaycusLaserDevice::RaycusLaserDevice(lcnc::process::ProcessSettingsService& settings)
    : LaserDevice(settings), m_strName("Raycus"), m_bIsInited(false), m_dMaxCurrent(0),
      m_dSimmerCurrent(0), m_iWaveShape(0) {}

ErrorCode RaycusLaserDevice::setLaserTable(const table& tableLaser) {
    bool bConnectChange = false;

    // 串口
    ErrorCode eCode = ErrorCode::ERROR_NONE;
    if (tableLaser.count("ComSetting")) {
        table tCom = processLaserTable(QStringLiteral("ComSetting"));
        eCode = SetComTable(tCom, bConnectChange);
        if (eCode != ErrorCode::ERROR_NONE)
            return ErrorCode::ERROR_LASER_CONNECTIONFAILED;
    }

    // 参数
    if (tableLaser.count("Laser") || bConnectChange) {
        bool bChange = false;
        double dEnergy = m_dEnergy;
        double dFrequency = m_dFrequency;
        double dPulseWidth = m_dPulseWidth;

        table tLaser;
        if (bConnectChange)
            tLaser = processLaserTable(QStringLiteral("Laser"));
        else
            tLaser = tableLaser.at("Laser").as_table();

        if (tLaser.count("fEnergy")) {
            bChange = true;
            dEnergy = tLaser["fEnergy"].as_floating();
        }
        if (tLaser.count("fFrequency")) {
            bChange = true;
            dFrequency = tLaser["fFrequency"].as_floating();
        }
        if (tLaser.count("fPulseWidth")) {
            bChange = true;
            dPulseWidth = tLaser["fPulseWidth"].as_floating();
        }
        if (bChange) {
            if (!SetLaserParameter(LaserParameter(dEnergy, dFrequency, dPulseWidth))) {
                eCode = ErrorCode::ERROR_LASER_SETTINGFAILED;
            }
        }
    }
    if (eCode == ErrorCode::ERROR_NONE)
        m_bIsInited = true;
    else
        m_bIsInited = false;
    return eCode;
}

const string& RaycusLaserDevice::GetName() {
    return m_strName;
}

bool RaycusLaserDevice::IsInited() {
    return m_bIsInited;
}

bool RaycusLaserDevice::IsAvailableData(const QByteArray& data) {
    if (data.isEmpty())
        return false;

    // 检查是否以 \x0D 结尾
    if (data.endsWith('\x0D'))
        return true;

    return false;
}

bool RaycusLaserDevice::StartLaser() {
    return OnceData("\x1B\x4F\x0D");
}

bool RaycusLaserDevice::StopLaser() {
    return OnceData("\x1B\x53\x0D");
}

bool RaycusLaserDevice::StartAimingBeam() {
    return true;
}

bool RaycusLaserDevice::StopAimingBeam() {
    return true;
}

bool RaycusLaserDevice::SetEnergy(double dEnergy) {
    if (std::isfinite(dEnergy) && dEnergy > 0.0 && dEnergy <= 255.0) {
        return SetLaserParameter(LaserParameter(dEnergy, m_dFrequency, m_dPulseWidth));
    } else {
        lcnc::process::logDeviceError(
            ErrorCode::ERROR_LASER_SETTINGFAILED,
            QObject::tr("The laser energy %1 does not meet the minimum requirement of 0.")
                .arg(dEnergy)
                .toUtf8()
                .data());
        return false;
    }
}

bool RaycusLaserDevice::SetFrequency(double dFrequency) {
    if (std::isfinite(dFrequency) && dFrequency >= 50.0 && dFrequency <= 65535.0) {
        return SetLaserParameter(LaserParameter(m_dEnergy, dFrequency, m_dPulseWidth));
    } else {
        lcnc::process::logDeviceError(
            ErrorCode::ERROR_LASER_SETTINGFAILED,
            QObject::tr("The laser frequency %1 does not meet the minimum requirement of 50.")
                .arg(dFrequency)
                .toUtf8()
                .data());
        return false;
    }
}

bool RaycusLaserDevice::SetPulseWidth(double dPulseWidth) {
    if (std::isfinite(dPulseWidth) && dPulseWidth > 0.0 && std::isfinite(m_dFrequency) &&
        m_dFrequency >= 50.0 && dPulseWidth * m_dFrequency / 10000.0 <= 255.0) {
        return SetLaserParameter(LaserParameter(m_dEnergy, m_dFrequency, dPulseWidth));
    } else {
        lcnc::process::logDeviceError(
            ErrorCode::ERROR_LASER_SETTINGFAILED,
            QObject::tr("The laser pulse width %1 does not meet the requirement.")
                .arg(dPulseWidth)
                .toUtf8()
                .data());
        return false;
    }
}

bool RaycusLaserDevice::SetLaserParameter(const LaserParameter& parameter) {
    if (!std::isfinite(parameter.dEnergy) || !std::isfinite(parameter.dFrequency) ||
        !std::isfinite(parameter.dPulseWidth) || parameter.dEnergy <= 0.0 ||
        parameter.dEnergy > 255.0 || parameter.dFrequency < 50.0 ||
        parameter.dFrequency > 65535.0 || parameter.dPulseWidth <= 0.0) {
        lcnc::process::logDeviceError(
            ErrorCode::ERROR_LASER_SETTINGFAILED,
            QObject::tr("Raycus laser parameters are outside the protocol range.")
                .toUtf8()
                .constData());
        return false;
    }

    const auto frequency = static_cast<std::uint16_t>(parameter.dFrequency);
    const double dutyValue = (parameter.dPulseWidth * parameter.dFrequency + 5000.0) / 10000.0;
    if (!std::isfinite(dutyValue) || dutyValue < 1.0 || dutyValue > 255.0)
        return false;
    const auto duty = static_cast<std::uint8_t>(dutyValue);
    const auto energy = static_cast<std::uint8_t>(parameter.dEnergy);

    QByteArray command;
    command.reserve(10);
    command.append(char(0x1B));
    command.append(char(0x46));
    if (frequency > 0xffU)
        command.append(static_cast<char>((frequency >> 8U) & 0xffU));
    command.append(static_cast<char>(frequency & 0xffU));
    command.append(char(0x44));
    command.append(static_cast<char>(duty));
    command.append(char(0x50));
    command.append(static_cast<char>(energy));
    command.append(char(0x0D));
    const bool success = OnceData(command);
    Sleep(100);
    if (!success)
        lcnc::process::logDeviceError(
            ErrorCode::ERROR_LASER_SETTINGFAILED,
            QObject::tr("Set laser energy %1 frequency %2 pulse width %3 failed.")
                .arg(parameter.dEnergy)
                .arg(parameter.dFrequency)
                .arg(parameter.dPulseWidth)
                .toUtf8()
                .data());
    else
        LaserDevice::UpdateLaserParameter(parameter);
    return success;
}

string RaycusLaserDevice::GetEnergy() {
    string strEnergy = std::to_string(m_dEnergy);
    return strEnergy;
}

string RaycusLaserDevice::GetFrequency() {
    string strFrequency = std::to_string(m_dFrequency);
    return strFrequency;
}

string RaycusLaserDevice::GetPulseWidth() {
    string strPulseWidth = std::to_string(m_dPulseWidth);
    return strPulseWidth;
}
