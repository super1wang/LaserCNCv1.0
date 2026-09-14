#include "model_envelope_common.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QTemporaryDir>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

struct BackendRun
{
    QString name;
    QString program;
    QString reportPath;
    QString outputPath;
    QString simplifiedStepPath;
    QStringList arguments;
    QJsonObject report;
    QByteArray standardOutput;
    QByteArray standardError;
    int exitCode{-1};
};

bool runBackend(BackendRun* run, QString* error)
{
    QProcess process;
    process.setProgram(run->program);
    process.setArguments(run->arguments);
    process.start();
    if (!process.waitForStarted(30'000)) {
        *error = QStringLiteral("%1 did not start: %2")
            .arg(run->name, process.errorString());
        return false;
    }
    if (!process.waitForFinished(30 * 60 * 1000)) {
        process.kill();
        process.waitForFinished(30'000);
        *error = QStringLiteral("%1 exceeded the 30 minute benchmark budget")
            .arg(run->name);
        return false;
    }
    run->standardOutput = process.readAllStandardOutput();
    run->standardError = process.readAllStandardError();
    run->exitCode = process.exitCode();
    QFile reportFile(run->reportPath);
    if (!reportFile.open(QIODevice::ReadOnly)) {
        *error = QStringLiteral("%1 did not produce a benchmark report; exit=%2; stderr=%3")
            .arg(run->name)
            .arg(run->exitCode)
            .arg(QString::fromUtf8(run->standardError));
        return false;
    }
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(
        reportFile.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject()) {
        *error = QStringLiteral("%1 report is invalid JSON: %2")
            .arg(run->name, parseError.errorString());
        return false;
    }
    run->report = document.object();
    return true;
}

bool hasExpectedBodies(const QJsonObject& report)
{
    const QJsonArray bodies = report.value(QStringLiteral("bodies")).toArray();
    if (bodies.size() != 6)
        return false;
    QStringList axes;
    for (const QJsonValue& value : bodies)
        axes.append(value.toObject().value(QStringLiteral("axis")).toString());
    axes.sort();
    QStringList expected{QStringLiteral("A"), QStringLiteral("BASE"),
                         QStringLiteral("C"), QStringLiteral("X"),
                         QStringLiteral("Y"), QStringLiteral("Z")};
    expected.sort();
    return axes == expected;
}

double numeric(const QJsonObject& report, const QString& key)
{
    return report.value(key).toDouble(std::numeric_limits<double>::infinity());
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    QTextStream out(stdout);
    QTextStream err(stderr);
    QTemporaryDir temporary;
    if (!temporary.isValid()) {
        err << "Unable to create benchmark staging directory\n";
        return 2;
    }

    const QString machinePath = QString::fromUtf8(LCNC_AC_TABLE_MODEL_PATH);
    BackendRun cgal{
        QStringLiteral("CGAL Alpha Wrap"),
        QString::fromUtf8(LCNC_CGAL_ENVELOPE_TOOL_PATH),
        temporary.filePath(QStringLiteral("cgal.json")),
        temporary.filePath(QStringLiteral("cgal")),
        temporary.filePath(QStringLiteral("cgal-envelope.step"))};
    cgal.arguments = {
        QStringLiteral("--machine"), machinePath,
        QStringLiteral("--deflection"), QStringLiteral("2.0"),
        QStringLiteral("--angle"), QStringLiteral("0.35"),
        QStringLiteral("--alpha"), QStringLiteral("10.0"),
        QStringLiteral("--offset"), QStringLiteral("2.5"),
        QStringLiteral("--samples"), QStringLiteral("20000"),
        QStringLiteral("--output-dir"), cgal.outputPath,
        QStringLiteral("--step-output"), cgal.simplifiedStepPath,
        QStringLiteral("--report"), cgal.reportPath};

    BackendRun openvdb{
        QStringLiteral("OpenVDB Enclosed Region Solidify"),
        QString::fromUtf8(LCNC_OPENVDB_ENVELOPE_TOOL_PATH),
        temporary.filePath(QStringLiteral("openvdb.json")),
        temporary.filePath(QStringLiteral("openvdb")),
        QString{}};
    openvdb.arguments = {
        QStringLiteral("--machine"), machinePath,
        QStringLiteral("--deflection"), QStringLiteral("2.0"),
        QStringLiteral("--angle"), QStringLiteral("0.35"),
        QStringLiteral("--voxel-size"), QStringLiteral("2.0"),
        QStringLiteral("--offset"), QStringLiteral("2.5"),
        QStringLiteral("--adaptivity"), QStringLiteral("0.01"),
        QStringLiteral("--closing-steps"), QStringLiteral("2"),
        QStringLiteral("--samples"), QStringLiteral("20000"),
        QStringLiteral("--output-dir"), openvdb.outputPath,
        QStringLiteral("--report"), openvdb.reportPath};

    QString error;
    if (!runBackend(&cgal, &error) || !runBackend(&openvdb, &error)) {
        err << error << '\n';
        return 3;
    }
    if (!hasExpectedBodies(cgal.report) || !hasExpectedBodies(openvdb.report)) {
        err << "Envelope loaders did not preserve BASE/X/Y/Z/A/C rigid-body grouping\n";
        return 4;
    }
    QVector<lcnc::tools::model_envelope::MachineBodyMesh> reloadedBodies;
    lcnc::tools::model_envelope::ImportMetrics reloadMetrics;
    QString reloadError;
    if (!QFileInfo::exists(cgal.simplifiedStepPath)
        || cgal.report.value(QStringLiteral("simplified_step_path")).toString().isEmpty()
        || cgal.report.value(QStringLiteral("simplified_step_sha256")).toString().size() != 64
        || !lcnc::tools::model_envelope::loadMachineBodyMeshes(
            cgal.simplifiedStepPath, 2.0, 0.35,
            &reloadedBodies, &reloadMetrics, &reloadError)
        || reloadedBodies.size() != 6) {
        err << "CGAL AP242 STEP output did not reload with all six axis bodies: "
            << reloadError << '\n';
        return 7;
    }
    const bool cgalValid = cgal.report.value(QStringLiteral("all_valid")).toBool();
    const bool openvdbValid = openvdb.report.value(QStringLiteral("all_valid")).toBool();
    if (!cgalValid && !openvdbValid) {
        err << "Neither envelope backend passed conservative mesh validation\n";
        return 5;
    }

    const double cgalTime = cgalValid
        ? numeric(cgal.report, QStringLiteral("envelope_ms"))
        : std::numeric_limits<double>::infinity();
    const double openvdbTime = openvdbValid
        ? numeric(openvdb.report, QStringLiteral("envelope_ms"))
        : std::numeric_limits<double>::infinity();
    const double cgalMemory = cgalValid
        ? numeric(cgal.report, QStringLiteral("peak_private_bytes"))
        : std::numeric_limits<double>::infinity();
    const double openvdbMemory = openvdbValid
        ? numeric(openvdb.report, QStringLiteral("peak_private_bytes"))
        : std::numeric_limits<double>::infinity();
    const double cgalTriangles = cgalValid
        ? numeric(cgal.report, QStringLiteral("output_triangles"))
        : std::numeric_limits<double>::infinity();
    const double openvdbTriangles = openvdbValid
        ? numeric(openvdb.report, QStringLiteral("output_triangles"))
        : std::numeric_limits<double>::infinity();
    const double fastestTime = std::min(cgalTime, openvdbTime);
    const double lowestMemory = std::min(cgalMemory, openvdbMemory);
    const double lowestTriangles = std::min(cgalTriangles, openvdbTriangles);
    const auto score = [&](const QJsonObject& report, bool valid) {
        if (!valid)
            return std::numeric_limits<double>::infinity();
        return 0.5 * numeric(report, QStringLiteral("envelope_ms")) / fastestTime
            + 0.3 * numeric(report, QStringLiteral("peak_private_bytes")) / lowestMemory
            + 0.2 * numeric(report, QStringLiteral("output_triangles")) / lowestTriangles;
    };
    const double cgalScore = score(cgal.report, cgalValid);
    const double openvdbScore = score(openvdb.report, openvdbValid);
    const QString winner = cgalScore < openvdbScore
        ? QStringLiteral("cgal-alpha-wrap")
        : QStringLiteral("openvdb-enclosed-region-solidify");

    QJsonObject comparison{
        {QStringLiteral("schema"), QStringLiteral("lcnc.model-envelope-comparison/v1")},
        {QStringLiteral("machine"), machinePath},
        {QStringLiteral("selection_rule"),
         QStringLiteral("reject invalid backends, then 50% envelope time + 30% peak private bytes + 20% output triangles")},
        {QStringLiteral("winner"), winner},
        {QStringLiteral("cgal_score"), cgalScore},
        {QStringLiteral("openvdb_score"), openvdbScore},
        {QStringLiteral("cgal"), cgal.report},
        {QStringLiteral("openvdb"), openvdb.report}};
    const QByteArray comparisonJson =
        QJsonDocument(comparison).toJson(QJsonDocument::Indented);
    const QString retainedReport = QDir::current().filePath(
        QStringLiteral("model_envelope_backend_comparison.json"));
    QFile retained(retainedReport);
    if (!retained.open(QIODevice::WriteOnly | QIODevice::Truncate)
        || retained.write(comparisonJson) != comparisonJson.size()) {
        err << "Unable to retain comparison report\n";
        return 6;
    }
    out << comparisonJson;
    return 0;
}
