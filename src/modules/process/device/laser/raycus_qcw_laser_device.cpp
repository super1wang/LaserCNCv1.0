/************************************************************************/
/*                            锐科QCW激光器实现类                           */
/************************************************************************/

#include "raycus_qcw_laser_device.h"

#include "modules/process/device/process_device_log.h"

#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <stdlib.h>
#include <string>
using std::ostringstream;
using namespace std;
using toml::table;

RaycusQCWLaserDevice::RaycusQCWLaserDevice(lcnc::process::ProcessSettingsService& settings)
    : LaserDevice(settings), m_strName("RaycusQCW"), m_bIsInited(false), m_dMaxCurrent(0),
      m_dSimmerCurrent(0), m_iWaveShape(0) {}

ErrorCode RaycusQCWLaserDevice::setLaserTable(const table& tableLaser) {
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

const string& RaycusQCWLaserDevice::GetName() {
    return m_strName;
}

bool RaycusQCWLaserDevice::IsInited() {
    return m_bIsInited;
}

bool RaycusQCWLaserDevice::IsAvailableData(const QByteArray& data) {
    if (data.isEmpty())
        return false;

    // 检查是否以 \x0D 结尾
    if (data.endsWith(QByteArray("\x55\xAA", 2)))
        return true;

    return false;
}

bool RaycusQCWLaserDevice::StartLaser() {
    return OnceData(QByteArray::fromHex("AA55000600FFE90001E955AA"));
}

bool RaycusQCWLaserDevice::StopLaser() {
    return OnceData(QByteArray::fromHex("AA55000600FFEA0001E955AA"));
}

bool RaycusQCWLaserDevice::StartAimingBeam() {
    return true;
}

bool RaycusQCWLaserDevice::StopAimingBeam() {
    return true;
}

bool RaycusQCWLaserDevice::SetEnergy(double dEnergy) {
    if (std::isfinite(dEnergy) && dEnergy >= 0.0 && dEnergy <= 100.0) {
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

bool RaycusQCWLaserDevice::SetFrequency(double dFrequency) {
    if (std::isfinite(dFrequency) && dFrequency >= 1.0 && dFrequency <= 5000.0) {
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

bool RaycusQCWLaserDevice::SetPulseWidth(double dPulseWidth) {
    if (std::isfinite(dPulseWidth) && dPulseWidth >= 1.0 && dPulseWidth <= 65535.0 &&
        std::isfinite(m_dFrequency) && m_dFrequency >= 1.0 && m_dFrequency <= 5000.0) {
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

bool RaycusQCWLaserDevice::SetLaserParameter(const LaserParameter& parameter) {
    if (!std::isfinite(parameter.dEnergy) || !std::isfinite(parameter.dFrequency) ||
        !std::isfinite(parameter.dPulseWidth) || parameter.dEnergy < 0.0 ||
        parameter.dEnergy > 100.0 || parameter.dFrequency < 1.0 || parameter.dFrequency > 5000.0 ||
        parameter.dPulseWidth < 1.0 || parameter.dPulseWidth > 65535.0) {
        lcnc::process::logDeviceError(
            ErrorCode::ERROR_LASER_SETTINGFAILED,
            QObject::tr("Raycus QCW laser parameters are outside the protocol range.")
                .toUtf8()
                .constData());
        return false;
    }

    const auto energy = static_cast<std::uint16_t>(parameter.dEnergy);
    const auto frequency = static_cast<std::uint16_t>(parameter.dFrequency);
    const auto pulseWidth = static_cast<std::uint16_t>(parameter.dPulseWidth);
    const auto dutyCycle = static_cast<std::uint8_t>(std::clamp(
        static_cast<int>(parameter.dPulseWidth * parameter.dFrequency / 10000.0), 1, 50));
    const auto checksum =
        static_cast<std::uint16_t>(753U + energy + frequency + dutyCycle + pulseWidth);

    QByteArray data = QByteArray::fromHex("AA55000D00FFE20004");
    data.append(static_cast<char>(energy & 0xffU));
    data.append(static_cast<char>((frequency >> 8U) & 0xffU));
    data.append(static_cast<char>(frequency & 0xffU));
    data.append(static_cast<char>(dutyCycle));
    data.append(static_cast<char>((pulseWidth >> 8U) & 0xffU));
    data.append(static_cast<char>(pulseWidth & 0xffU));
    data.append(static_cast<char>((checksum >> 8U) & 0xffU));
    data.append(static_cast<char>(checksum & 0xffU));
    data.append(QByteArray::fromHex("55AA"));
    const bool parameterWritten = OnceData(data);
    Sleep(100);
    const bool success = parameterWritten && StartLaser();
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

string RaycusQCWLaserDevice::GetEnergy() {
    string strEnergy = std::to_string(m_dEnergy);
    return strEnergy;
}

string RaycusQCWLaserDevice::GetFrequency() {
    string strFrequency = std::to_string(m_dFrequency);
    return strFrequency;
}

string RaycusQCWLaserDevice::GetPulseWidth() {
    string strPulseWidth = std::to_string(m_dPulseWidth);
    return strPulseWidth;
}
