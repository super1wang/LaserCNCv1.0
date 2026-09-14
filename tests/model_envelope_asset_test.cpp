#include "core/algorithms/cam/machine_safety_index.h"
#include "core/machine/machine_safety_package.h"
#include "core/machine/model_envelope_asset.h"

#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDataStream>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTemporaryDir>

#include <JlCompress.h>

#include <array>
#include <iostream>

namespace {

bool require(bool condition, const QString& message)
{
    if (!condition)
        std::cerr << message.toStdString() << '\n';
    return condition;
}

QByteArray fileSha256(const QString& path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
        return {};
    QCryptographicHash hash(QCryptographicHash::Sha256);
    return hash.addData(&file) ? hash.result() : QByteArray{};
}

bool writeTetrahedronPly(const QString& path)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
        return false;
    static constexpr char header[] =
        "ply\n"
        "format binary_little_endian 1.0\n"
        "element vertex 4\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "element face 4\n"
        "property list uchar uint vertex_indices\n"
        "end_header\n";
    if (file.write(header, sizeof(header) - 1) != sizeof(header) - 1)
        return false;
    QDataStream stream(&file);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
    const std::array<std::array<float, 3>, 4> vertices{{
        {{0.0f, 0.0f, 0.0f}}, {{1.0f, 0.0f, 0.0f}},
        {{0.0f, 1.0f, 0.0f}}, {{0.0f, 0.0f, 1.0f}}}};
    const std::array<std::array<quint32, 3>, 4> triangles{{
        {{0, 2, 1}}, {{0, 1, 3}}, {{0, 3, 2}}, {{1, 2, 3}}}};
    for (const auto& vertex : vertices)
        stream << vertex[0] << vertex[1] << vertex[2];
    for (const auto& triangle : triangles)
        stream << quint8{3} << triangle[0] << triangle[1] << triangle[2];
    return stream.status() == QDataStream::Ok;
}

QString createEnvelopeFixture(const QString& root, const QString& sourceModel)
{
    const QDir directory(root);
    if (!directory.mkpath(QStringLiteral("meshes")))
        return {};
    const std::array<std::pair<const char*, const char*>, 6> topology{{
        {"BASE", ""}, {"X", "Y"}, {"Y", "BASE"},
        {"Z", "X"}, {"A", "BASE"}, {"C", "A"}}};
    QJsonArray bodies;
    for (const auto& [axisText, parentText] : topology) {
        const QString axis = QString::fromLatin1(axisText);
        const QString relativePath = QStringLiteral("meshes/%1.ply").arg(axis);
        const QString absolutePath = directory.filePath(relativePath);
        if (!writeTetrahedronPly(absolutePath))
            return {};
        bodies.append(QJsonObject{
            {QStringLiteral("name"), QStringLiteral("LCNC_AXIS_%1").arg(axis)},
            {QStringLiteral("axis"), axis},
            {QStringLiteral("parent_axis"), QString::fromLatin1(parentText)},
            {QStringLiteral("mesh_path"), relativePath},
            {QStringLiteral("mesh_sha256"),
             QString::fromLatin1(fileSha256(absolutePath).toHex())},
            {QStringLiteral("alpha_mm"),
             axis == QStringLiteral("Z") ? 3.0 : 10.0},
            {QStringLiteral("offset_mm"),
             axis == QStringLiteral("Z") ? 0.5 : 2.5},
            {QStringLiteral("output_vertices"), 4},
            {QStringLiteral("output_triangles"), 4},
            {QStringLiteral("valid"), true},
            {QStringLiteral("validation"), QJsonObject{
                 {QStringLiteral("non_empty"), true},
                 {QStringLiteral("finite"), true},
                 {QStringLiteral("watertight"), true},
                 {QStringLiteral("two_manifold"), true},
                 {QStringLiteral("outside_source_points"), 0}}}});
    }
    const QString simplifiedStepPath = QStringLiteral("simplified_machine.step");
    QFile simplifiedStep(directory.filePath(simplifiedStepPath));
    static constexpr char stepFixture[] = "ISO-10303-21;\nEND-ISO-10303-21;\n";
    if (!simplifiedStep.open(QIODevice::WriteOnly | QIODevice::Truncate)
        || simplifiedStep.write(stepFixture, sizeof(stepFixture) - 1)
            != sizeof(stepFixture) - 1) {
        return {};
    }
    simplifiedStep.close();
    const QJsonObject manifest{
        {QStringLiteral("schema"), QStringLiteral("lcnc.model-envelope/v1")},
        {QStringLiteral("format_version"), 1},
        {QStringLiteral("backend"), QStringLiteral("test-envelope")},
        {QStringLiteral("backend_version"), QStringLiteral("1")},
        {QStringLiteral("source_model_sha256"),
         QString::fromLatin1(fileSha256(sourceModel).toHex())},
        {QStringLiteral("deflection_mm"), 2.0},
        {QStringLiteral("angle_rad"), 0.35},
        {QStringLiteral("alpha_mm"), 10.0},
        {QStringLiteral("offset_mm"), 2.5},
        {QStringLiteral("collision_conservative"), true},
        {QStringLiteral("all_valid"), true},
        {QStringLiteral("body_count"), static_cast<int>(topology.size())},
        {QStringLiteral("simplified_step_path"), simplifiedStepPath},
        {QStringLiteral("simplified_step_sha256"),
         QString::fromLatin1(fileSha256(directory.filePath(simplifiedStepPath)).toHex())},
        {QStringLiteral("bodies"), bodies}};
    const QString path = directory.filePath(QStringLiteral("model_envelope.json"));
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)
        || file.write(QJsonDocument(manifest).toJson(QJsonDocument::Indented)) < 0) {
        return {};
    }
    return path;
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    const QString sourceModel = QString::fromUtf8(LCNC_AC_TABLE_MODEL_PATH);
    QTemporaryDir temporary;
    if (!require(temporary.isValid(), QStringLiteral("Unable to create temporary directory")))
        return 1;
    const QString manifestPath = createEnvelopeFixture(temporary.path(), sourceModel);
    lcnc::ModelEnvelopeAsset asset;
    QString error;
    if (!require(!manifestPath.isEmpty()
                 && lcnc::ModelEnvelopeAsset::loadAndValidate(
                     manifestPath, sourceModel, &asset, &error),
                 QStringLiteral("Unable to validate model-envelope fixture: %1").arg(error))
        || !require(asset.bodies.size() == 6 && asset.manifestSha256.size() == 32
                        && asset.simplifiedStepPath == QStringLiteral("simplified_machine.step")
                        && asset.simplifiedStepSha256.size() == 32
                        && asset.bodies[3].axisName == QStringLiteral("Z")
                        && asset.bodies[3].alphaMm == 3.0
                        && asset.bodies[3].offsetMm == 0.5,
                    QStringLiteral("Validated model-envelope metadata is incomplete"))) {
        return 1;
    }
    lcnc::cam_algo::SurfaceTriangleSoup soup;
    if (!require(lcnc::ModelEnvelopeAsset::loadBodyMesh(
                     asset, asset.bodies.front(), &soup, &error)
                 && soup.vertices.size() == 4 && soup.triangles.size() == 4,
                 QStringLiteral("Unable to load model-envelope PLY: %1").arg(error))) {
        return 1;
    }

    lcnc::cam_algo::MachineSafetyIndexCompiler compiler;
    if (!require(compiler.loadMachine(sourceModel, &error)
                 && compiler.applyModelEnvelope(asset, &error),
                 QStringLiteral("Unable to bind model envelope to machine: %1").arg(error))) {
        return 1;
    }
    auto options = lcnc::cam_algo::defaultAcTableSafetyBuildOptions();
    for (auto& axis : options.axes)
        axis.step = axis.maximum - axis.minimum;
    options.maximumCells = 64;
    options.maximumExactQueries = 0;
    options.refinementLevels = 0;
    options.useSurfaceBvhCertification = false;
    options.useLeafBvhCertification = false;
    lcnc::cam_algo::MachineSafetyIndex index;
    if (!require(compiler.build(options, &index, nullptr, &error),
                 QStringLiteral("Unable to build bound safety index: %1").arg(error))
        || !require(index.envelopeManifestSha256() == asset.manifestSha256,
                    QStringLiteral("Safety index lost the envelope binding"))) {
        return 1;
    }
    const QString indexPath = temporary.filePath(QStringLiteral("machine.lmsi"));
    const QString packagePath = temporary.filePath(QStringLiteral("machine.lmsp"));
    const QByteArray runtimeConfigurationSha256(32, '\x35');
    if (!require(index.save(indexPath, &error)
                 && lcnc::MachineSafetyPackage::createWithEnvelope(
                     sourceModel, indexPath, manifestPath, packagePath,
                     runtimeConfigurationSha256, &error),
                 QStringLiteral("Unable to create envelope safety package: %1").arg(error))) {
        return 1;
    }

    QTemporaryDir extraction;
    lcnc::MachineSafetyPackageLoadResult package;
    lcnc::cam_algo::MachineSafetyIndex extractedIndex;
    if (!require(lcnc::MachineSafetyPackage::extractAndValidate(
                     packagePath, extraction.path(), &package, &error),
                 QStringLiteral("Unable to validate envelope safety package: %1").arg(error))
        || !require(extractedIndex.load(package.safetyIndexPath, &error),
                    QStringLiteral("Unable to load packaged safety index: %1").arg(error))
        || !require(package.manifest.formatVersion == 2
                    && package.manifest.envelopeManifestSha256 == asset.manifestSha256
                    && package.envelopeAsset.bodies.size() == 6
                    && QFileInfo(QDir(QFileInfo(package.envelopeManifestPath).absolutePath())
                                     .filePath(package.envelopeAsset.simplifiedStepPath)).isFile()
                    && extractedIndex.envelopeManifestSha256() == asset.manifestSha256,
                    QStringLiteral("Envelope safety package binding is incomplete"))) {
        return 1;
    }

    const QString embeddedMesh = QDir(QFileInfo(package.envelopeManifestPath).absolutePath())
                                     .filePath(package.envelopeAsset.bodies.front().meshPath);
    QFile tamperedMesh(embeddedMesh);
    if (!require(tamperedMesh.open(QIODevice::Append)
                 && tamperedMesh.write("tamper", 6) == 6,
                 QStringLiteral("Unable to create tampered envelope mesh fixture"))) {
        return 1;
    }
    tamperedMesh.close();
    const QString tamperedPackage = temporary.filePath(QStringLiteral("tampered.lmsp"));
    if (!require(JlCompress::compressDir(tamperedPackage, extraction.path(), true),
                 QStringLiteral("Unable to archive tampered envelope package"))) {
        return 1;
    }
    QTemporaryDir tamperedExtraction;
    QString tamperError;
    if (!require(!lcnc::MachineSafetyPackage::extractAndValidate(
                     tamperedPackage, tamperedExtraction.path(), nullptr, &tamperError),
                 QStringLiteral("Package with a modified envelope mesh was accepted"))) {
        return 1;
    }
    return 0;
}
