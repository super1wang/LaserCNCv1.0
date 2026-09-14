#include "core/algorithms/cam/machine_safety_fingerprint.h"

#include <QCryptographicHash>
#include <QDataStream>
#include <QIODevice>

namespace lcnc::cam_algo {

QByteArray machineSafetyRuntimeFingerprint(
    const QString& configurationType,
    const QList<MachineSafetyRuntimeAxis>& axes,
    const QStringList& partAxisSequence,
    int packageFormatVersion)
{
    QByteArray canonical;
    QDataStream stream(&canonical, QIODevice::WriteOnly);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setFloatingPointPrecision(QDataStream::DoublePrecision);
    stream << quint32(5)
           << QStringLiteral("lmsi-machine-domain-v2")
           << QStringLiteral("boundary-unknown-fail-closed")
           << QStringLiteral("complete-head-is-z-axis-machine-geometry")
           << QStringLiteral("workpiece-is-the-only-runtime-geometry-variable")
           << QStringLiteral("required-pairs:Z-A,Z-C,X-A,X-C")
           << qint32(packageFormatVersion)
           << configurationType;
    stream << quint32(axes.size());
    for (const MachineSafetyRuntimeAxis& axis : axes) {
        stream << axis.name << quint8(axis.motionType)
               << axis.direction[0] << axis.direction[1] << axis.direction[2]
               << axis.origin[0] << axis.origin[1] << axis.origin[2]
               << axis.minimum << axis.maximum << axis.parentAxis;
    }
    stream << quint32(partAxisSequence.size());
    for (const QString& axisName : partAxisSequence)
        stream << axisName;
    return QCryptographicHash::hash(canonical, QCryptographicHash::Sha256);
}

} // namespace lcnc::cam_algo
