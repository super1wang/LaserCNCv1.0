#pragma once

#include <QByteArray>
#include <QList>
#include <QString>
#include <QStringList>

#include <array>
#include <cstdint>

namespace lcnc::cam_algo {

struct MachineSafetyRuntimeAxis
{
    QString name;
    std::uint8_t motionType{0};
    std::array<double, 3> direction{};
    std::array<double, 3> origin{};
    double minimum{0.0};
    double maximum{0.0};
    QString parentAxis;
};

/**
 * Builds the machine-only runtime fingerprint stored in a .lmsp package.
 *
 * partAxisSequence is ordered by the machine parts in the packaged STEP file.
 * It deliberately contains no XCAF entry ids: those ids are document-local and
 * change whenever the same immutable STEP model is reloaded.
 */
QByteArray machineSafetyRuntimeFingerprint(
    const QString& configurationType,
    const QList<MachineSafetyRuntimeAxis>& axes,
    const QStringList& partAxisSequence,
    int packageFormatVersion);

} // namespace lcnc::cam_algo
