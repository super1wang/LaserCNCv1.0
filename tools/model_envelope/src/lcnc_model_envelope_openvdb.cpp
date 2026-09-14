#include "model_envelope_common.h"

#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/TopologyToLevelSet.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/version.h>

#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <exception>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

using lcnc::tools::model_envelope::MachineBodyMesh;
using lcnc::tools::model_envelope::TriangleMesh;

TriangleMesh buildEnvelope(const TriangleMesh& source,
                           double voxelSizeMm,
                           double offsetMm,
                           int closingSteps,
                           double adaptivity)
{
    std::vector<openvdb::Vec3s> points;
    std::vector<openvdb::Vec3I> triangles;
    points.reserve(source.vertices.size());
    triangles.reserve(source.triangles.size());
    for (const auto& point : source.vertices) {
        points.emplace_back(static_cast<float>(point.x),
                            static_cast<float>(point.y),
                            static_cast<float>(point.z));
    }
    for (const auto& triangle : source.triangles) {
        if (triangle[0] > static_cast<std::uint32_t>(std::numeric_limits<int>::max())
            || triangle[1] > static_cast<std::uint32_t>(std::numeric_limits<int>::max())
            || triangle[2] > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
            throw std::overflow_error("OpenVDB triangle index exceeds signed integer range");
        }
        triangles.emplace_back(static_cast<int>(triangle[0]),
                               static_cast<int>(triangle[1]),
                               static_cast<int>(triangle[2]));
    }

    const openvdb::math::Transform::Ptr transform =
        openvdb::math::Transform::createLinearTransform(voxelSizeMm);
    openvdb::FloatGrid::Ptr levelSet =
        openvdb::tools::meshToLevelSet<openvdb::FloatGrid>(
            *transform, points, triangles, 3.0f);
    if (!levelSet)
        throw std::runtime_error("OpenVDB mesh conversion returned no level set");

    // The enclosed-region mask contains both the signed interior and closed
    // cavities. Rebuilding from its topology discards the original B-Rep
    // topology and internal shells. Dilation is rounded upward to voxels so
    // the collision proxy remains one-sided and conservative.
    const openvdb::BoolGrid::Ptr solid =
        openvdb::tools::extractEnclosedRegion(*levelSet);
    if (!solid || solid->tree().empty())
        throw std::runtime_error("OpenVDB enclosed-region extraction returned no volume");
    const int dilationVoxels = static_cast<int>(std::ceil(offsetMm / voxelSizeMm)) + 1;
    levelSet = openvdb::tools::topologyToLevelSet(
        *solid, 3, closingSteps, dilationVoxels, 0);

    openvdb::tools::VolumeToMesh mesher(0.0, adaptivity, true);
    mesher(*levelSet);
    TriangleMesh result;
    result.vertices.reserve(mesher.pointListSize());
    for (std::size_t index = 0; index < mesher.pointListSize(); ++index) {
        const openvdb::Vec3s& point = mesher.pointList()[index];
        result.vertices.push_back({point.x(), point.y(), point.z()});
    }

    for (std::size_t poolIndex = 0;
         poolIndex < mesher.polygonPoolListSize(); ++poolIndex) {
        const openvdb::tools::PolygonPool& pool =
            mesher.polygonPoolList()[poolIndex];
        for (std::size_t index = 0; index < pool.numTriangles(); ++index) {
            const openvdb::Vec3I& triangle = pool.triangle(index);
            result.triangles.push_back({
                static_cast<std::uint32_t>(triangle[0]),
                static_cast<std::uint32_t>(triangle[1]),
                static_cast<std::uint32_t>(triangle[2])});
        }
        for (std::size_t index = 0; index < pool.numQuads(); ++index) {
            const openvdb::Vec4I& quad = pool.quad(index);
            result.triangles.push_back({
                static_cast<std::uint32_t>(quad[0]),
                static_cast<std::uint32_t>(quad[1]),
                static_cast<std::uint32_t>(quad[2])});
            result.triangles.push_back({
                static_cast<std::uint32_t>(quad[0]),
                static_cast<std::uint32_t>(quad[2]),
                static_cast<std::uint32_t>(quad[3])});
        }
    }
    return result;
}

bool positiveFinite(const QString& text, double* value)
{
    bool ok = false;
    const double parsed = text.toDouble(&ok);
    if (!ok || !std::isfinite(parsed) || parsed <= 0.0)
        return false;
    *value = parsed;
    return true;
}

bool nonNegativeFinite(const QString& text, double* value)
{
    bool ok = false;
    const double parsed = text.toDouble(&ok);
    if (!ok || !std::isfinite(parsed) || parsed < 0.0)
        return false;
    *value = parsed;
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    QCoreApplication::setApplicationName(QStringLiteral("lcnc_model_envelope_openvdb"));
    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral(
        "Apache-2.0 OpenVDB enclosed-region solidification benchmark and envelope generator"));
    const QCommandLineOption helpOption = parser.addHelpOption();
    QCommandLineOption machineOption({QStringLiteral("m"), QStringLiteral("machine")},
        QStringLiteral("LCNC_AXIS_* named STEP machine model"), QStringLiteral("path"));
    QCommandLineOption outputDirectoryOption(QStringLiteral("output-dir"),
        QStringLiteral("Optional directory for per-body binary PLY files"),
        QStringLiteral("path"));
    QCommandLineOption reportOption(QStringLiteral("report"),
        QStringLiteral("JSON benchmark report path"), QStringLiteral("path"));
    QCommandLineOption deflectionOption(QStringLiteral("deflection"),
        QStringLiteral("OCC linear tessellation deflection in millimeters"),
        QStringLiteral("mm"), QStringLiteral("2.0"));
    QCommandLineOption angleOption(QStringLiteral("angle"),
        QStringLiteral("OCC angular tessellation deflection in radians"),
        QStringLiteral("rad"), QStringLiteral("0.35"));
    QCommandLineOption voxelOption(QStringLiteral("voxel-size"),
        QStringLiteral("OpenVDB finest voxel size in millimeters"),
        QStringLiteral("mm"), QStringLiteral("2.0"));
    QCommandLineOption offsetOption(QStringLiteral("offset"),
        QStringLiteral("Conservative outward level-set offset in millimeters"),
        QStringLiteral("mm"), QStringLiteral("2.5"));
    QCommandLineOption adaptivityOption(QStringLiteral("adaptivity"),
        QStringLiteral("VolumeToMesh adaptivity in the range 0 through 1"),
        QStringLiteral("value"), QStringLiteral("0.01"));
    QCommandLineOption closingStepsOption(QStringLiteral("closing-steps"),
        QStringLiteral("Morphological closing steps used to seal voxel-scale openings"),
        QStringLiteral("count"), QStringLiteral("2"));
    QCommandLineOption samplesOption(QStringLiteral("samples"),
        QStringLiteral("Maximum source vertices sampled per rigid body"),
        QStringLiteral("count"), QStringLiteral("20000"));
    parser.addOptions({machineOption, outputDirectoryOption, reportOption,
                       deflectionOption, angleOption, voxelOption,
                       offsetOption, adaptivityOption, closingStepsOption,
                       samplesOption});
    if (!parser.parse(QCoreApplication::arguments())) {
        QTextStream(stderr) << parser.errorText() << '\n';
        return 2;
    }
    if (parser.isSet(helpOption)) {
        QTextStream(stdout) << parser.helpText();
        return 0;
    }

    QTextStream err(stderr);
    double deflection = 0.0;
    double angle = 0.0;
    double voxelSize = 0.0;
    double offset = 0.0;
    double adaptivity = 0.0;
    bool samplesOk = false;
    bool closingStepsOk = false;
    const int samples = parser.value(samplesOption).toInt(&samplesOk);
    const int closingSteps = parser.value(closingStepsOption).toInt(&closingStepsOk);
    if (!parser.isSet(machineOption)
        || !positiveFinite(parser.value(deflectionOption), &deflection)
        || !positiveFinite(parser.value(angleOption), &angle)
        || !positiveFinite(parser.value(voxelOption), &voxelSize)
        || !nonNegativeFinite(parser.value(offsetOption), &offset)
        || !nonNegativeFinite(parser.value(adaptivityOption), &adaptivity)
        || adaptivity > 1.0 || !closingStepsOk || closingSteps < 0
        || closingSteps > 64 || !samplesOk || samples < 1) {
        err << "Invalid envelope benchmark arguments\n";
        return 2;
    }

    openvdb::initialize();
    QVector<MachineBodyMesh> bodies;
    lcnc::tools::model_envelope::ImportMetrics importMetrics;
    QString error;
    QElapsedTimer totalTimer;
    totalTimer.start();
    if (!lcnc::tools::model_envelope::loadMachineBodyMeshes(
            parser.value(machineOption), deflection, angle,
            &bodies, &importMetrics, &error)) {
        if (parser.isSet(reportOption)) {
            QFile failedReport(parser.value(reportOption));
            if (QDir().mkpath(QFileInfo(failedReport.fileName()).absolutePath())
                && failedReport.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
                failedReport.write(QJsonDocument(QJsonObject{
                    {QStringLiteral("backend"), QStringLiteral("openvdb-enclosed-region-solidify")},
                    {QStringLiteral("all_valid"), false},
                    {QStringLiteral("error"), error}}).toJson());
            }
        }
        std::fprintf(stderr, "%s\n", error.toLocal8Bit().constData());
        std::fflush(stderr);
        openvdb::uninitialize();
        return 3;
    }

    QJsonArray bodyReports;
    std::uint64_t outputVertices = 0;
    std::uint64_t outputTriangles = 0;
    qint64 totalEnvelopeMs = 0;
    bool allValid = true;
    const QString outputDirectory = parser.value(outputDirectoryOption);
    try {
        for (const MachineBodyMesh& body : std::as_const(bodies)) {
            QElapsedTimer bodyTimer;
            bodyTimer.start();
            TriangleMesh envelope = buildEnvelope(
                body.mesh, voxelSize, offset, closingSteps, adaptivity);
            const qint64 envelopeMs = bodyTimer.elapsed();
            totalEnvelopeMs += envelopeMs;
            outputVertices += envelope.vertices.size();
            outputTriangles += envelope.triangles.size();
            const auto validation = lcnc::tools::model_envelope::validateEnvelope(
                body.mesh, envelope, static_cast<std::size_t>(samples));
            const bool valid = validation.nonEmpty && validation.finite
                && validation.watertight && validation.twoManifold
                && validation.outsideSourcePoints == 0;
            allValid = allValid && valid;

            QString outputPath;
            if (!outputDirectory.isEmpty()) {
                outputPath = QDir(outputDirectory).filePath(
                    lcnc::tools::model_envelope::sanitizedBodyFileName(
                        body.axisName, body.name) + QStringLiteral(".ply"));
                if (!lcnc::tools::model_envelope::writeBinaryPly(
                        outputPath, envelope, &error)) {
                    err << error << '\n';
                    openvdb::uninitialize();
                    return 5;
                }
            }

            QJsonObject report{
                {QStringLiteral("name"), body.name},
                {QStringLiteral("axis"), body.axisName},
                {QStringLiteral("source_vertices"),
                 static_cast<qint64>(body.mesh.vertices.size())},
                {QStringLiteral("source_triangles"),
                 static_cast<qint64>(body.mesh.triangles.size())},
                {QStringLiteral("output_vertices"),
                 static_cast<qint64>(envelope.vertices.size())},
                {QStringLiteral("output_triangles"),
                 static_cast<qint64>(envelope.triangles.size())},
                {QStringLiteral("envelope_ms"), envelopeMs},
                {QStringLiteral("valid"), valid},
                {QStringLiteral("output_path"), outputPath},
                {QStringLiteral("validation"),
                 lcnc::tools::model_envelope::validationMetricsJson(validation)}};
            bodyReports.append(report);
        }
    } catch (const std::exception& failure) {
        err << failure.what() << '\n';
        openvdb::uninitialize();
        return 4;
    } catch (...) {
        err << "Unknown OpenVDB envelope failure\n";
        openvdb::uninitialize();
        return 4;
    }

    QJsonObject root{
        {QStringLiteral("schema"), QStringLiteral("lcnc.model-envelope-benchmark/v1")},
        {QStringLiteral("backend"), QStringLiteral("openvdb-enclosed-region-solidify")},
        {QStringLiteral("backend_version"), QStringLiteral(OPENVDB_LIBRARY_VERSION_STRING)},
        {QStringLiteral("machine"), parser.value(machineOption)},
        {QStringLiteral("body_count"), bodies.size()},
        {QStringLiteral("deflection_mm"), deflection},
        {QStringLiteral("angle_rad"), angle},
        {QStringLiteral("voxel_size_mm"), voxelSize},
        {QStringLiteral("offset_mm"), offset},
        {QStringLiteral("adaptivity"), adaptivity},
        {QStringLiteral("closing_steps"), closingSteps},
        {QStringLiteral("import"),
         lcnc::tools::model_envelope::importMetricsJson(importMetrics)},
        {QStringLiteral("envelope_ms"), totalEnvelopeMs},
        {QStringLiteral("total_ms"), totalTimer.elapsed()},
        {QStringLiteral("output_vertices"), static_cast<qint64>(outputVertices)},
        {QStringLiteral("output_triangles"), static_cast<qint64>(outputTriangles)},
        {QStringLiteral("peak_working_set_bytes"),
         static_cast<qint64>(lcnc::tools::model_envelope::peakWorkingSetBytes())},
        {QStringLiteral("peak_private_bytes"),
         static_cast<qint64>(lcnc::tools::model_envelope::peakPrivateBytes())},
        {QStringLiteral("all_valid"), allValid},
        {QStringLiteral("bodies"), bodyReports}};
    const QByteArray json = QJsonDocument(root).toJson(QJsonDocument::Indented);
    if (parser.isSet(reportOption)) {
        QFile reportFile(parser.value(reportOption));
        if (!QDir().mkpath(QFileInfo(reportFile.fileName()).absolutePath())
            || !reportFile.open(QIODevice::WriteOnly | QIODevice::Truncate)
            || reportFile.write(json) != json.size()) {
            err << "Unable to write JSON report\n";
            openvdb::uninitialize();
            return 5;
        }
    }
    QTextStream(stdout) << json;
    openvdb::uninitialize();
    return allValid ? 0 : 6;
}
