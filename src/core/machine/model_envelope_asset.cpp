#include "core/machine/model_envelope_asset.h"

#include "core/logging/logger.h"

#include <QCryptographicHash>
#include <QDataStream>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSet>

#include <cmath>
#include <limits>

namespace {

QByteArray fileSha256(const QString& path, QString* errorMessage)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to read model-envelope resource: %1")
                                .arg(path);
        return {};
    }
    QCryptographicHash hash(QCryptographicHash::Sha256);
    if (!hash.addData(&file)) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to hash model-envelope resource: %1")
                                .arg(path);
        return {};
    }
    return hash.result();
}

bool isSafeRelativePath(const QString& path)
{
    const QString normalized = QDir::cleanPath(path.trimmed()).replace('\\', '/');
    return !normalized.isEmpty() && normalized != QStringLiteral(".")
        && !QFileInfo(normalized).isAbsolute()
        && normalized != QStringLiteral("..")
        && !normalized.startsWith(QStringLiteral("../"))
        && !normalized.contains(QStringLiteral("/../"));
}

bool exactHexSha256(const QString& text, QByteArray* digest)
{
    const QByteArray encoded = text.trimmed().toLatin1().toLower();
    const QByteArray decoded = QByteArray::fromHex(encoded);
    if (encoded.size() != 64 || decoded.size() != 32
        || decoded.toHex() != encoded) {
        return false;
    }
    *digest = decoded;
    return true;
}

bool finitePositive(double value)
{
    return std::isfinite(value) && value > 0.0;
}

bool finiteNonNegative(double value)
{
    return std::isfinite(value) && value >= 0.0;
}

} // namespace

namespace lcnc {

bool ModelEnvelopeAsset::loadAndValidate(const QString& manifestPath,
                                         const QString& expectedSourceModelPath,
                                         ModelEnvelopeAsset* asset,
                                         QString* errorMessage)
{
    if (!asset) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Model-envelope output is null");
        return false;
    }
    *asset = {};
    QFile file(manifestPath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to read model-envelope manifest: %1")
                                .arg(manifestPath);
        return false;
    }
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject()) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Invalid model-envelope manifest: %1")
                                .arg(parseError.errorString());
        return false;
    }
    const QJsonObject root = document.object();
    ModelEnvelopeAsset parsed;
    parsed.schema = root.value(QStringLiteral("schema")).toString();
    parsed.formatVersion = root.value(QStringLiteral("format_version")).toInt();
    parsed.generator = root.value(QStringLiteral("backend")).toString();
    parsed.generatorVersion = root.value(QStringLiteral("backend_version")).toString();
    parsed.deflectionMm = root.value(QStringLiteral("deflection_mm")).toDouble(
        std::numeric_limits<double>::quiet_NaN());
    parsed.angleRad = root.value(QStringLiteral("angle_rad")).toDouble(
        std::numeric_limits<double>::quiet_NaN());
    parsed.alphaMm = root.value(QStringLiteral("alpha_mm")).toDouble(
        std::numeric_limits<double>::quiet_NaN());
    parsed.offsetMm = root.value(QStringLiteral("offset_mm")).toDouble(
        std::numeric_limits<double>::quiet_NaN());
    parsed.collisionConservative =
        root.value(QStringLiteral("collision_conservative")).toBool(false)
        && root.value(QStringLiteral("all_valid")).toBool(false);
    if (!exactHexSha256(root.value(QStringLiteral("source_model_sha256")).toString(),
                        &parsed.sourceModelSha256)) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Model-envelope source SHA-256 is invalid");
        return false;
    }
    if (parsed.schema != QStringLiteral("lcnc.model-envelope/v1")
        || parsed.formatVersion != kCurrentFormatVersion
        || parsed.generator.isEmpty() || parsed.generatorVersion.isEmpty()
        || !finitePositive(parsed.deflectionMm)
        || !finitePositive(parsed.angleRad)
        || !finitePositive(parsed.alphaMm)
        || !finitePositive(parsed.offsetMm)
        || !parsed.collisionConservative) {
        if (errorMessage)
            *errorMessage = QStringLiteral(
                "Model-envelope manifest is incomplete, unsupported, or not conservative");
        return false;
    }
    if (!expectedSourceModelPath.isEmpty()) {
        QString hashError;
        const QByteArray expected = fileSha256(expectedSourceModelPath, &hashError);
        if (expected.isEmpty() || expected != parsed.sourceModelSha256) {
            if (errorMessage) {
                *errorMessage = expected.isEmpty() ? hashError
                    : QStringLiteral("Model-envelope source model fingerprint mismatch");
            }
            return false;
        }
    }
    const QJsonArray bodies = root.value(QStringLiteral("bodies")).toArray();
    if (bodies.isEmpty() || bodies.size() > 64
        || root.value(QStringLiteral("body_count")).toInt() != bodies.size()) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Model-envelope body count is invalid");
        return false;
    }
    const QDir manifestDirectory(QFileInfo(manifestPath).absolutePath());
    const QString simplifiedStepValue = QDir::cleanPath(
        root.value(QStringLiteral("simplified_step_path")).toString()).replace('\\', '/');
    const QString simplifiedStepShaValue =
        root.value(QStringLiteral("simplified_step_sha256")).toString();
    if (!simplifiedStepValue.isEmpty() || !simplifiedStepShaValue.isEmpty()) {
        if (!isSafeRelativePath(simplifiedStepValue)
            || !exactHexSha256(simplifiedStepShaValue,
                               &parsed.simplifiedStepSha256)) {
            if (errorMessage)
                *errorMessage = QStringLiteral("Model-envelope simplified STEP binding is invalid");
            return false;
        }
        parsed.simplifiedStepPath = simplifiedStepValue;
        QString hashError;
        if (fileSha256(manifestDirectory.filePath(parsed.simplifiedStepPath),
                       &hashError) != parsed.simplifiedStepSha256) {
            if (errorMessage) {
                *errorMessage = hashError.isEmpty()
                    ? QStringLiteral("Model-envelope simplified STEP fingerprint mismatch")
                    : hashError;
            }
            return false;
        }
    }
    QSet<QString> axes;
    QSet<QString> meshPaths;
    for (const QJsonValue& value : bodies) {
        const QJsonObject object = value.toObject();
        ModelEnvelopeBodyResource body;
        body.name = object.value(QStringLiteral("name")).toString().trimmed();
        body.axisName = object.value(QStringLiteral("axis")).toString().trimmed().toUpper();
        body.parentAxis = object.value(QStringLiteral("parent_axis")).toString()
                              .trimmed().toUpper();
        body.meshPath = QDir::cleanPath(
            object.value(QStringLiteral("mesh_path")).toString()).replace('\\', '/');
        body.alphaMm = object.value(QStringLiteral("alpha_mm")).toDouble(
            parsed.alphaMm);
        body.offsetMm = object.value(QStringLiteral("offset_mm")).toDouble(
            parsed.offsetMm);
        body.vertexCount = static_cast<std::uint64_t>(
            object.value(QStringLiteral("output_vertices")).toInteger());
        body.triangleCount = static_cast<std::uint64_t>(
            object.value(QStringLiteral("output_triangles")).toInteger());
        const QJsonObject validation = object.value(QStringLiteral("validation")).toObject();
        const bool conservative = object.value(QStringLiteral("valid")).toBool(false)
            && validation.value(QStringLiteral("non_empty")).toBool(false)
            && validation.value(QStringLiteral("finite")).toBool(false)
            && validation.value(QStringLiteral("watertight")).toBool(false)
            && validation.value(QStringLiteral("two_manifold")).toBool(false)
            && validation.value(QStringLiteral("outside_source_points")).toInteger(-1) == 0;
        if (!exactHexSha256(object.value(QStringLiteral("mesh_sha256")).toString(),
                            &body.meshSha256)
            || body.name.isEmpty() || body.axisName.isEmpty()
            || !isSafeRelativePath(body.meshPath)
            || !finitePositive(body.alphaMm)
            || !finitePositive(body.offsetMm)
            || body.offsetMm >= body.alphaMm
            || body.vertexCount == 0 || body.triangleCount == 0
            || axes.contains(body.axisName)
            || meshPaths.contains(body.meshPath.toCaseFolded())
            || !conservative) {
            if (errorMessage)
                *errorMessage = QStringLiteral("Invalid model-envelope body resource: %1")
                                    .arg(body.axisName);
            return false;
        }
        const QString resolvedMesh = manifestDirectory.filePath(body.meshPath);
        QString hashError;
        if (fileSha256(resolvedMesh, &hashError) != body.meshSha256) {
            if (errorMessage)
                *errorMessage = hashError.isEmpty()
                    ? QStringLiteral("Model-envelope mesh fingerprint mismatch: %1")
                          .arg(body.meshPath)
                    : hashError;
            return false;
        }
        axes.insert(body.axisName);
        meshPaths.insert(body.meshPath.toCaseFolded());
        parsed.bodies.append(std::move(body));
    }
    parsed.manifestPath = QFileInfo(manifestPath).absoluteFilePath();
    QString manifestHashError;
    parsed.manifestSha256 = fileSha256(parsed.manifestPath, &manifestHashError);
    if (parsed.manifestSha256.size() != 32) {
        if (errorMessage)
            *errorMessage = manifestHashError;
        return false;
    }
    *asset = std::move(parsed);
    return true;
}

bool ModelEnvelopeAsset::loadBodyMesh(const ModelEnvelopeAsset& asset,
                                      const ModelEnvelopeBodyResource& body,
                                      cam_algo::SurfaceTriangleSoup* soup,
                                      QString* errorMessage)
{
    if (!soup) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Model-envelope mesh output is null");
        return false;
    }
    *soup = {};
    const QString path = QDir(QFileInfo(asset.manifestPath).absolutePath())
                             .filePath(body.meshPath);
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to open model-envelope PLY: %1").arg(path);
        return false;
    }
    bool binaryLittleEndian = false;
    std::uint64_t vertexCount = 0;
    std::uint64_t faceCount = 0;
    qint64 headerBytes = 0;
    for (int lineNumber = 0; lineNumber < 64; ++lineNumber) {
        const QByteArray line = file.readLine(4096);
        headerBytes += line.size();
        if (line.isEmpty() || headerBytes > 64 * 1024) {
            if (errorMessage)
                *errorMessage = QStringLiteral("Model-envelope PLY header is incomplete");
            return false;
        }
        const QList<QByteArray> fields = line.trimmed().split(' ');
        if (fields.size() == 3 && fields.at(0) == "format")
            binaryLittleEndian = fields.at(1) == "binary_little_endian";
        if (fields.size() == 3 && fields.at(0) == "element") {
            bool ok = false;
            const qulonglong count = fields.at(2).toULongLong(&ok);
            if (!ok)
                return false;
            if (fields.at(1) == "vertex")
                vertexCount = count;
            else if (fields.at(1) == "face")
                faceCount = count;
        }
        if (line.trimmed() == "end_header")
            break;
    }
    if (!binaryLittleEndian || vertexCount != body.vertexCount
        || faceCount != body.triangleCount || vertexCount > 5'000'000
        || faceCount > 5'000'000) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Model-envelope PLY layout or counts are invalid");
        return false;
    }
    QDataStream stream(&file);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
    soup->vertices.reserve(static_cast<std::size_t>(vertexCount));
    soup->triangles.reserve(static_cast<std::size_t>(faceCount));
    for (std::uint64_t index = 0; index < vertexCount; ++index) {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        stream >> x >> y >> z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            if (errorMessage)
                *errorMessage = QStringLiteral("Model-envelope PLY contains a non-finite vertex");
            return false;
        }
        soup->vertices.push_back({x, y, z});
    }
    for (std::uint64_t index = 0; index < faceCount; ++index) {
        quint8 count = 0;
        quint32 a = 0;
        quint32 b = 0;
        quint32 c = 0;
        stream >> count >> a >> b >> c;
        if (count != 3 || a >= vertexCount || b >= vertexCount || c >= vertexCount) {
            if (errorMessage)
                *errorMessage = QStringLiteral("Model-envelope PLY contains an invalid face");
            return false;
        }
        soup->triangles.push_back({a, b, c});
    }
    if (stream.status() != QDataStream::Ok || !file.atEnd()) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Model-envelope PLY payload is truncated or extended");
        return false;
    }
    return true;
}

} // namespace lcnc
