#pragma once

#include <QByteArray>
#include <cstdint>

namespace lcnc::process::ultron {

QByteArray makeReadRegister(std::uint16_t address);
QByteArray makeWriteRegister(std::uint16_t address, std::uint16_t value);
bool parseRegisterValue(const QByteArray& response, std::uint16_t* value) noexcept;
bool hasValidCrc(const QByteArray& frame) noexcept;

} // namespace lcnc::process::ultron
