#include "modules/process/device/laser/protocol/ultron_protocol.h"

namespace lcnc::process::ultron {
namespace {

constexpr char kDeviceAddress = 0x01;
constexpr char kReadHoldingRegister = 0x03;
constexpr char kWriteSingleRegister = 0x06;

std::uint16_t modbusCrc(const QByteArray& data) noexcept {
    std::uint16_t crc = 0xFFFF;
    for (const char rawByte : data) {
        crc ^= static_cast<std::uint8_t>(rawByte);
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc & 0x0001U) != 0U ? (crc >> 1U) ^ 0xA001U : crc >> 1U;
    }
    return crc;
}

void appendCrc(QByteArray& frame) {
    const std::uint16_t crc = modbusCrc(frame);
    frame.append(static_cast<char>(crc & 0x00FFU));
    frame.append(static_cast<char>((crc >> 8U) & 0x00FFU));
}

void appendBigEndian(QByteArray& frame, std::uint16_t value) {
    frame.append(static_cast<char>((value >> 8U) & 0x00FFU));
    frame.append(static_cast<char>(value & 0x00FFU));
}

} // namespace

QByteArray makeReadRegister(std::uint16_t address) {
    QByteArray frame;
    frame.reserve(8);
    frame.append(kDeviceAddress);
    frame.append(kReadHoldingRegister);
    appendBigEndian(frame, address);
    appendBigEndian(frame, 1U);
    appendCrc(frame);
    return frame;
}

QByteArray makeWriteRegister(std::uint16_t address, std::uint16_t value) {
    QByteArray frame;
    frame.reserve(8);
    frame.append(kDeviceAddress);
    frame.append(kWriteSingleRegister);
    appendBigEndian(frame, address);
    appendBigEndian(frame, value);
    appendCrc(frame);
    return frame;
}

bool hasValidCrc(const QByteArray& frame) noexcept {
    if (frame.size() < 4)
        return false;
    const std::uint16_t expected = modbusCrc(frame.first(frame.size() - 2));
    const auto low = static_cast<std::uint8_t>(frame.at(frame.size() - 2));
    const auto high = static_cast<std::uint8_t>(frame.at(frame.size() - 1));
    const std::uint16_t actual =
        static_cast<std::uint16_t>(low) | static_cast<std::uint16_t>(high << 8U);
    return actual == expected;
}

bool parseRegisterValue(const QByteArray& response, std::uint16_t* value) noexcept {
    if (!value)
        return false;

    if (response.size() >= 7 && static_cast<std::uint8_t>(response.at(0)) == 0x01U &&
        static_cast<std::uint8_t>(response.at(1)) == 0x03U &&
        static_cast<std::uint8_t>(response.at(2)) == 0x02U && hasValidCrc(response.first(7))) {
        *value = static_cast<std::uint16_t>(static_cast<std::uint8_t>(response.at(3)) << 8U) |
                 static_cast<std::uint8_t>(response.at(4));
        return true;
    }

    const QByteArray normalized = response.trimmed().toUpper();
    const qsizetype prefix = normalized.indexOf("010302");
    if (prefix < 0 || normalized.size() < prefix + 10)
        return false;
    bool ok = false;
    const auto parsed = normalized.mid(prefix + 6, 4).toUShort(&ok, 16);
    if (!ok)
        return false;
    *value = parsed;
    return true;
}

} // namespace lcnc::process::ultron
