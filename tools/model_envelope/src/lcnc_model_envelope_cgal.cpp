#include "model_envelope_common.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/alpha_wrap_3.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/version.h>

#include <BRep_Builder.hxx>
#include <DESTEP_Parameters.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopoDS_Face.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QSaveFile>
#include <QSet>
#include <QTextStream>

#include <cmath>
#include <cstdio>
#include <cstdint>
#include <exception>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using CgalPoint = Kernel::Point_3;
using CgalMesh = CGAL::Surface_mesh<CgalPoint>;
using lcnc::tools::model_envelope::MachineBodyMesh;
using lcnc::tools::model_envelope::TriangleMesh;

TriangleMesh buildEnvelope(const TriangleMesh& source,
                           double alphaMm,
                           double offsetMm)
{
    std::vector<CgalPoint> points;
    points.reserve(source.vertices.size());
    for (const auto& point : source.vertices)
        points.emplace_back(point.x, point.y, point.z);
    std::vector<std::array<std::uint32_t, 3>> faces = source.triangles;

    CgalMesh wrapped;
    CGAL::alpha_wrap_3(points, faces, alphaMm, offsetMm, wrapped);

    TriangleMesh result;
    result.vertices.resize(wrapped.number_of_vertices());
    for (const auto vertex : wrapped.vertices()) {
        const CgalPoint& point = wrapped.point(vertex);
        result.vertices[vertex.idx()] = {point.x(), point.y(), point.z()};
    }
    result.triangles.reserve(wrapped.number_of_faces());
    for (const auto face : wrapped.faces()) {
        std::array<std::uint32_t, 3> triangle{};
        int count = 0;
        for (const auto vertex : CGAL::vertices_around_face(wrapped.halfedge(face), wrapped)) {
            if (count < 3)
                triangle[count] = static_cast<std::uint32_t>(vertex.idx());
            ++count;
        }
        if (count != 3)
            throw std::runtime_error("CGAL alpha wrap returned a non-triangle face");
        result.triangles.push_back(triangle);
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

QByteArray fileSha256(const QString& path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
        return {};
    QCryptographicHash hash(QCryptographicHash::Sha256);
    return hash.addData(&file) ? hash.result() : QByteArray{};
}

QString parentAxis(const QString& axis)
{
    if (axis == QStringLiteral("Y") || axis == QStringLiteral("A"))
        return QStringLiteral("BASE");
    if (axis == QStringLiteral("X"))
        return QStringLiteral("Y");
    if (axis == QStringLiteral("Z"))
        return QStringLiteral("X");
    if (axis == QStringLiteral("C"))
        return QStringLiteral("A");
    return {};
}

bool writeTessellatedStep(const QString& path,
                          const QVector<MachineBodyMesh>& bodies,
                          QString* errorMessage)
{
    if (path.isEmpty() || bodies.isEmpty()) {
        *errorMessage = QStringLiteral("Simplified STEP output is incomplete");
        return false;
    }
    Handle(TDocStd_Document) document =
        new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
    XCAFDoc_DocumentTool::Set(document->Main());
    const Handle(XCAFDoc_ShapeTool) shapes =
        XCAFDoc_DocumentTool::ShapeTool(document->Main());
    BRep_Builder builder;
    for (const MachineBodyMesh& body : bodies) {
        if (body.mesh.vertices.empty() || body.mesh.triangles.empty()
            || body.mesh.vertices.size() > static_cast<std::size_t>(
                std::numeric_limits<int>::max())
            || body.mesh.triangles.size() > static_cast<std::size_t>(
                std::numeric_limits<int>::max())) {
            *errorMessage = QStringLiteral("Simplified STEP body is empty or too large: %1")
                                .arg(body.axisName);
            return false;
        }
        Handle(Poly_Triangulation) triangulation = new Poly_Triangulation(
            static_cast<int>(body.mesh.vertices.size()),
            static_cast<int>(body.mesh.triangles.size()), false, false);
        for (std::size_t index = 0; index < body.mesh.vertices.size(); ++index) {
            const auto& point = body.mesh.vertices[index];
            triangulation->SetNode(static_cast<int>(index + 1),
                                   gp_Pnt(point.x, point.y, point.z));
        }
        for (std::size_t index = 0; index < body.mesh.triangles.size(); ++index) {
            const auto& triangle = body.mesh.triangles[index];
            triangulation->SetTriangle(
                static_cast<int>(index + 1),
                Poly_Triangle(static_cast<int>(triangle[0] + 1),
                              static_cast<int>(triangle[1] + 1),
                              static_cast<int>(triangle[2] + 1)));
        }
        TopoDS_Face face;
        builder.MakeFace(face, triangulation);
        const TDF_Label label = shapes->AddShape(face, false);
        const QString name = QStringLiteral("LCNC_AXIS_%1").arg(body.axisName);
        TDataStd_Name::Set(label, TCollection_ExtendedString(
            name.toStdU16String().c_str()));
    }
    if (!QDir().mkpath(QFileInfo(path).absolutePath())) {
        *errorMessage = QStringLiteral("Unable to create simplified STEP directory");
        return false;
    }
    DESTEP_Parameters parameters;
    parameters.WriteSchema = DESTEP_Parameters::WriteMode_StepSchema_AP242DIS;
    parameters.WriteTessellated = DESTEP_Parameters::RWMode_Tessellated_OnNoBRep;
    parameters.WriteName = true;
    parameters.WriteSubshapeNames = true;
    STEPCAFControl_Writer writer;
    writer.SetNameMode(true);
    if (!writer.Transfer(document, parameters)
        || writer.Write(QFileInfo(path).absoluteFilePath().toUtf8().constData())
            != IFSelect_RetDone) {
        *errorMessage = QStringLiteral("Unable to write tessellated AP242 STEP output");
        return false;
    }
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    QCoreApplication::setApplicationName(QStringLiteral("lcnc_model_envelope_cgal"));
    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral(
        "Licensed CGAL Alpha Wrap model-envelope generator running as an independent tool"));
    const QCommandLineOption helpOption = parser.addHelpOption();
    QCommandLineOption machineOption({QStringLiteral("m"), QStringLiteral("machine")},
        QStringLiteral("LCNC_AXIS_* named STEP machine model"), QStringLiteral("path"));
    QCommandLineOption outputDirectoryOption(QStringLiteral("output-dir"),
        QStringLiteral("Optional directory for per-body binary PLY files"),
        QStringLiteral("path"));
    QCommandLineOption reportOption(QStringLiteral("report"),
        QStringLiteral("JSON benchmark report path"), QStringLiteral("path"));
    QCommandLineOption stepOutputOption(QStringLiteral("step-output"),
        QStringLiteral("Optional AP242 tessellated STEP output retaining LCNC_AXIS_* bodies"),
        QStringLiteral("path"));
    QCommandLineOption deflectionOption(QStringLiteral("deflection"),
        QStringLiteral("OCC linear tessellation deflection in millimeters"),
        QStringLiteral("mm"), QStringLiteral("3.0"));
    QCommandLineOption angleOption(QStringLiteral("angle"),
        QStringLiteral("OCC angular tessellation deflection in radians"),
        QStringLiteral("rad"), QStringLiteral("0.35"));
    QCommandLineOption alphaOption(QStringLiteral("alpha"),
        QStringLiteral("CGAL Alpha Wrap alpha in millimeters"),
        QStringLiteral("mm"), QStringLiteral("10.0"));
    QCommandLineOption offsetOption(QStringLiteral("offset"),
        QStringLiteral("CGAL outward offset in millimeters"),
        QStringLiteral("mm"), QStringLiteral("2.5"));
    QCommandLineOption detailAxesOption(QStringLiteral("detail-axes"),
        QStringLiteral("Comma-separated axes using the high-fidelity Alpha Wrap profile"),
        QStringLiteral("axes"));
    QCommandLineOption detailAlphaOption(QStringLiteral("detail-alpha"),
        QStringLiteral("High-fidelity CGAL Alpha Wrap feature threshold in millimeters"),
        QStringLiteral("mm"), QStringLiteral("2.0"));
    QCommandLineOption detailOffsetOption(QStringLiteral("detail-offset"),
        QStringLiteral("High-fidelity outward offset in millimeters"),
        QStringLiteral("mm"), QStringLiteral("0.5"));
    QCommandLineOption samplesOption(QStringLiteral("samples"),
        QStringLiteral("Maximum source vertices sampled per rigid body"),
        QStringLiteral("count"), QStringLiteral("20000"));
    parser.addOptions({machineOption, outputDirectoryOption, reportOption,
                       stepOutputOption,
                       deflectionOption, angleOption, alphaOption,
                       offsetOption, detailAxesOption, detailAlphaOption,
                       detailOffsetOption, samplesOption});
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
    double alpha = 0.0;
    double offset = 0.0;
    double detailAlpha = 0.0;
    double detailOffset = 0.0;
    QSet<QString> detailAxes;
    const QSet<QString> supportedAxes{
        QStringLiteral("BASE"), QStringLiteral("X"), QStringLiteral("Y"),
        QStringLiteral("Z"), QStringLiteral("A"), QStringLiteral("B"),
        QStringLiteral("C")};
    for (const QString& value : parser.value(detailAxesOption).split(
             ',', Qt::SkipEmptyParts)) {
        const QString axis = value.trimmed().toUpper();
        if (!supportedAxes.contains(axis)) {
            err << "Unsupported detail axis: " << axis << '\n';
            return 2;
        }
        detailAxes.insert(axis);
    }
    bool samplesOk = false;
    const int samples = parser.value(samplesOption).toInt(&samplesOk);
    if (!parser.isSet(machineOption)
        || !positiveFinite(parser.value(deflectionOption), &deflection)
        || !positiveFinite(parser.value(angleOption), &angle)
        || !positiveFinite(parser.value(alphaOption), &alpha)
        || !positiveFinite(parser.value(offsetOption), &offset)
        || !positiveFinite(parser.value(detailAlphaOption), &detailAlpha)
        || !positiveFinite(parser.value(detailOffsetOption), &detailOffset)
        || offset >= alpha || detailOffset >= detailAlpha
        || !samplesOk || samples < 1) {
        err << "Invalid envelope benchmark arguments\n";
        return 2;
    }

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
                    {QStringLiteral("backend"), QStringLiteral("cgal-alpha-wrap")},
                    {QStringLiteral("all_valid"), false},
                    {QStringLiteral("error"), error}}).toJson());
            }
        }
        std::fprintf(stderr, "%s\n", error.toLocal8Bit().constData());
        std::fflush(stderr);
        return 3;
    }

    QJsonArray bodyReports;
    std::uint64_t outputVertices = 0;
    std::uint64_t outputTriangles = 0;
    qint64 totalEnvelopeMs = 0;
    bool allValid = true;
    QVector<MachineBodyMesh> envelopeBodies;
    envelopeBodies.reserve(bodies.size());
    const QString outputDirectory = parser.value(outputDirectoryOption);
    try {
        for (const MachineBodyMesh& body : std::as_const(bodies)) {
            const bool highFidelity = detailAxes.contains(body.axisName);
            const double bodyAlpha = highFidelity ? detailAlpha : alpha;
            const double bodyOffset = highFidelity ? detailOffset : offset;
            std::fprintf(stderr, "CGAL wrapping %s: %zu vertices, %zu triangles\n",
                         body.name.toLatin1().constData(), body.mesh.vertices.size(),
                         body.mesh.triangles.size());
            std::fflush(stderr);
            QElapsedTimer bodyTimer;
            bodyTimer.start();
            std::fprintf(stderr, "  profile=%s alpha=%.6g offset=%.6g\n",
                         highFidelity ? "high-fidelity" : "standard",
                         bodyAlpha, bodyOffset);
            std::fflush(stderr);
            TriangleMesh envelope = buildEnvelope(body.mesh, bodyAlpha, bodyOffset);
            std::fprintf(stderr, "CGAL wrap complete: %zu vertices, %zu triangles\n",
                         envelope.vertices.size(), envelope.triangles.size());
            std::fflush(stderr);
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
            QString meshPath;
            QByteArray meshSha256;
            if (!outputDirectory.isEmpty()) {
                outputPath = QDir(outputDirectory).absoluteFilePath(
                    lcnc::tools::model_envelope::sanitizedBodyFileName(
                        body.axisName, body.name) + QStringLiteral(".ply"));
                if (!lcnc::tools::model_envelope::writeBinaryPly(
                        outputPath, envelope, &error)) {
                    err << error << '\n';
                    return 5;
                }
                meshSha256 = fileSha256(outputPath);
                const QString manifestDirectory = parser.isSet(reportOption)
                    ? QFileInfo(parser.value(reportOption)).absolutePath()
                    : QFileInfo(outputPath).absolutePath();
                meshPath = QDir(manifestDirectory).relativeFilePath(outputPath)
                               .replace('\\', '/');
                if (meshSha256.size() != 32) {
                    err << "Unable to fingerprint PLY output\n";
                    return 5;
                }
            }

            QJsonObject report{
                {QStringLiteral("name"), body.name},
                {QStringLiteral("axis"), body.axisName},
                {QStringLiteral("parent_axis"), parentAxis(body.axisName)},
                {QStringLiteral("profile"), highFidelity
                    ? QStringLiteral("high-fidelity") : QStringLiteral("standard")},
                {QStringLiteral("alpha_mm"), bodyAlpha},
                {QStringLiteral("offset_mm"), bodyOffset},
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
                {QStringLiteral("mesh_path"), meshPath},
                {QStringLiteral("mesh_sha256"),
                 QString::fromLatin1(meshSha256.toHex())},
                {QStringLiteral("validation"),
                 lcnc::tools::model_envelope::validationMetricsJson(validation)}};
            bodyReports.append(report);
            envelopeBodies.append({body.name, body.axisName, std::move(envelope)});
        }
    } catch (const std::exception& failure) {
        err << failure.what() << '\n';
        return 4;
    } catch (...) {
        err << "Unknown CGAL envelope failure\n";
        return 4;
    }

    QString simplifiedStepPath;
    QString simplifiedStepRelativePath;
    QByteArray simplifiedStepSha256;
    if (allValid && parser.isSet(stepOutputOption)) {
        simplifiedStepPath = QFileInfo(parser.value(stepOutputOption)).absoluteFilePath();
        if (!writeTessellatedStep(simplifiedStepPath, envelopeBodies, &error)) {
            err << error << '\n';
            return 7;
        }
        simplifiedStepSha256 = fileSha256(simplifiedStepPath);
        const QString manifestDirectory = parser.isSet(reportOption)
            ? QFileInfo(parser.value(reportOption)).absolutePath()
            : QFileInfo(simplifiedStepPath).absolutePath();
        simplifiedStepRelativePath = QDir(manifestDirectory)
            .relativeFilePath(simplifiedStepPath).replace('\\', '/');
        if (simplifiedStepSha256.size() != 32) {
            err << "Unable to fingerprint simplified STEP output\n";
            return 7;
        }
    }

    QStringList detailAxisList = detailAxes.values();
    detailAxisList.sort();
    QJsonObject root{
        {QStringLiteral("schema"), QStringLiteral("lcnc.model-envelope/v1")},
        {QStringLiteral("format_version"), 1},
        {QStringLiteral("backend"), QStringLiteral("cgal-alpha-wrap")},
        {QStringLiteral("backend_version"), QStringLiteral(CGAL_VERSION_STR)},
        {QStringLiteral("machine"), parser.value(machineOption)},
        {QStringLiteral("source_model_sha256"),
         QString::fromLatin1(fileSha256(parser.value(machineOption)).toHex())},
        {QStringLiteral("body_count"), bodies.size()},
        {QStringLiteral("deflection_mm"), deflection},
        {QStringLiteral("angle_rad"), angle},
        {QStringLiteral("alpha_mm"), alpha},
        {QStringLiteral("offset_mm"), offset},
        {QStringLiteral("detail_axes"), QJsonArray::fromStringList(detailAxisList)},
        {QStringLiteral("detail_alpha_mm"), detailAlpha},
        {QStringLiteral("detail_offset_mm"), detailOffset},
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
        {QStringLiteral("collision_conservative"), allValid},
        {QStringLiteral("simplified_step_path"), simplifiedStepRelativePath},
        {QStringLiteral("simplified_step_sha256"),
         QString::fromLatin1(simplifiedStepSha256.toHex())},
        {QStringLiteral("bodies"), bodyReports}};
    const QByteArray json = QJsonDocument(root).toJson(QJsonDocument::Indented);
    if (parser.isSet(reportOption)) {
        QSaveFile reportFile(parser.value(reportOption));
        if (!QDir().mkpath(QFileInfo(reportFile.fileName()).absolutePath())
            || !reportFile.open(QIODevice::WriteOnly | QIODevice::Truncate)
            || reportFile.write(json) != json.size()
            || !reportFile.commit()) {
            err << "Unable to write JSON report\n";
            return 5;
        }
    }
    QTextStream(stdout) << json;
    return allValid ? 0 : 6;
}
