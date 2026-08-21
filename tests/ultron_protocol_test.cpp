#include "modules/process/device/laser/protocol/ultron_protocol.h"

#include <iostream>

int main() {
    const QByteArray write = lcnc::process::ultron::makeWriteRegister(0x0607, 1);
    if (write.toHex().toUpper() != QByteArray("010606070001F943")) {
        std::cerr << "unexpected write-register frame: " << write.toHex().constData() << '\n';
        return 1;
    }

    const QByteArray read = lcnc::process::ultron::makeReadRegister(0x060D);
    if (read.toHex().toUpper() != QByteArray("0103060D00011541")) {
        std::cerr << "unexpected read-register frame: " << read.toHex().constData() << '\n';
        return 2;
    }

    QByteArray binary = QByteArray::fromHex("0103020064B9AF");
    std::uint16_t value = 0;
    if (!lcnc::process::ultron::parseRegisterValue(binary, &value) || value != 100) {
        std::cerr << "binary response was not parsed\n";
        return 3;
    }
    if (!lcnc::process::ultron::parseRegisterValue(QByteArray("0103020064\r"), &value) ||
        value != 100) {
        std::cerr << "ASCII response was not parsed\n";
        return 4;
    }
    binary[3] = static_cast<char>(0x01);
    if (lcnc::process::ultron::parseRegisterValue(binary, &value) ||
        lcnc::process::ultron::parseRegisterValue({}, &value) ||
        lcnc::process::ultron::parseRegisterValue(QByteArray("0103020064"), nullptr)) {
        std::cerr << "invalid response was accepted\n";
        return 5;
    }
    return 0;
}
