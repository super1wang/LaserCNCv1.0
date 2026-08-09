#include <QCoreApplication>
#include <QDataStream>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>
#include <QTemporaryDir>
#include <QTextStream>

#include <JlCompress.h>
#include <toml.hpp>

#include <fstream>

namespace {

constexpr quint64 kPointsMagic = 0x315450434E434C00ull;
constexpr quint32 kPointsV5 = 5;

bool copyRequired(const QString& source, const QString& target, QString* error)
{
    if (!QFileInfo::exists(source)) {
        if (error) *error = QStringLiteral("Required source file does not exist: %1").arg(source);
        return false;
    }
    QFile::remove(target);
    if (!QFile::copy(source, target)) {
        if (error) *error = QStringLiteral("Unable to copy %1 to %2").arg(source, target);
        return false;
    }
    return true;
}

void writeInvalidPose(QDataStream& output)
{
    for (int index = 0; index < 5; ++index) output << 0.0;
    output << quint8(0) << quint8(0)
           << QStringLiteral("Migrated project requires machine-coordinate recomputation");
}

bool copyPoint(QDataStream& input, QDataStream& output, QStringList* rotaryNames)
{
    double values[13]{};
    quint8 crossValid = 0, machineValid = 0;
    qint32 edgeIndex = -1;
    QString r1Name, r2Name;
    for (int index = 0; index < 6; ++index) input >> values[index];
    for (int index = 6; index < 9; ++index) input >> values[index];
    input >> crossValid;
    for (int index = 9; index < 13; ++index) input >> values[index];
    input >> edgeIndex;
    double machine[5]{};
    for (double& value : machine) input >> value;
    input >> r1Name >> r2Name >> machineValid;

    for (int index = 0; index < 6; ++index) output << values[index];
    for (int index = 6; index < 9; ++index) output << values[index];
    output << crossValid;
    for (int index = 9; index < 13; ++index) output << values[index];
    output << edgeIndex;
    writeInvalidPose(output);
    // Axis names belong to the old snapshot layout even when its machine
    // coordinates were invalid.  Preserve that topology hint while clearing
    // every legacy coordinate value.
    for (const QString& name : {r1Name, r2Name})
        if (!name.trimmed().isEmpty() && !rotaryNames->contains(name.trimmed().toUpper()))
            rotaryNames->append(name.trimmed().toUpper());
    return input.status() == QDataStream::Ok && output.status() == QDataStream::Ok;
}

bool upgradePoints(const QString& path, QStringList* rotaryNames, QString* error)
{
    QFile inputFile(path);
    if (!inputFile.open(QIODevice::ReadOnly)) {
        if (error) *error = QStringLiteral("Unable to read legacy point set: %1").arg(path);
        return false;
    }
    QDataStream input(&inputFile);
    input.setByteOrder(QDataStream::LittleEndian);
    input.setFloatingPointPrecision(QDataStream::DoublePrecision);
    input.setVersion(QDataStream::Qt_6_5);
    quint64 magic = 0;
    quint32 version = 0, contourCount = 0;
    input >> magic >> version >> contourCount;
    if (magic != kPointsMagic || version < 1 || version > 4) {
        if (version == kPointsV5) return true;
        if (error) *error = QStringLiteral("Unsupported legacy point-set version: %1").arg(version);
        return false;
    }

    const QString outputPath = path + QStringLiteral(".v5");
    QFile outputFile(outputPath);
    if (!outputFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (error) *error = QStringLiteral("Unable to create v5 point set: %1").arg(outputPath);
        return false;
    }
    QDataStream output(&outputFile);
    output.setByteOrder(QDataStream::LittleEndian);
    output.setFloatingPointPrecision(QDataStream::DoublePrecision);
    output.setVersion(QDataStream::Qt_6_5);
    output << kPointsMagic << kPointsV5 << contourCount;
    for (quint32 contour = 0; contour < contourCount; ++contour) {
        quint64 contourId = 0;
        quint32 pointCount = 0;
        input >> contourId >> pointCount;
        output << contourId << pointCount;
        for (quint32 point = 0; point < pointCount; ++point)
            if (!copyPoint(input, output, rotaryNames)) return false;
        quint8 leadValid = 0;
        input >> leadValid;
        output << leadValid;
        if (leadValid && !copyPoint(input, output, rotaryNames)) return false;
    }
    inputFile.close();
    outputFile.close();
    if (input.status() != QDataStream::Ok || output.status() != QDataStream::Ok) {
        if (error) *error = QStringLiteral("Legacy point set is truncated or corrupt");
        QFile::remove(outputPath);
        return false;
    }
    QFile::remove(path);
    if (!QFile::rename(outputPath, path)) {
        if (error) *error = QStringLiteral("Unable to replace migrated point set");
        return false;
    }
    return true;
}

toml::value axisEntry(const char* name, const char* role)
{
    toml::value entry(toml::table{});
    entry["name"] = std::string(name);
    entry["role"] = std::string(role);
    return entry;
}

bool upgradeCam(const QString& rootPath, QString* error)
{
    const QString tomlPath = QDir(rootPath).filePath(QStringLiteral("cam/cache/cam_toolpath.toml"));
    const QString pointsPath = QDir(rootPath).filePath(QStringLiteral("cam/cache/cam_toolpath_points.bin"));
    if (!QFileInfo::exists(tomlPath) && !QFileInfo::exists(pointsPath)) return true;
    if (!QFileInfo::exists(tomlPath) || !QFileInfo::exists(pointsPath)) {
        if (error) *error = QStringLiteral("Legacy CAM cache is incomplete");
        return false;
    }
    QStringList rotaryNames;
    if (!upgradePoints(pointsPath, &rotaryNames, error)) return false;
    toml::value root;
    try { root = toml::parse(tomlPath.toStdString()); }
    catch (const std::exception& exception) {
        if (error) *error = QString::fromLocal8Bit(exception.what());
        return false;
    }
    root["schemaVersion"] = 5;
    QString mode = QStringLiteral("Planar3Axis");
    toml::array layout;
    layout.push_back(axisEntry("X", "LinearX"));
    layout.push_back(axisEntry("Y", "LinearY"));
    layout.push_back(axisEntry("Z", "LinearZ"));
    if (rotaryNames.size() == 1) {
        mode = QStringLiteral("RotaryTube4Axis");
        const QByteArray axis = rotaryNames.front().toLatin1();
        layout.push_back(axisEntry(axis.constData(),
            rotaryNames.front() == QStringLiteral("A") ? "WorkpieceRotary" : "TableSpin"));
    } else if (rotaryNames.size() >= 2) {
        mode = QStringLiteral("SimultaneousTable5Axis");
        QString tilt = rotaryNames.contains(QStringLiteral("A")) ? QStringLiteral("A") : QStringLiteral("B");
        QString spin = rotaryNames.contains(QStringLiteral("C")) ? QStringLiteral("C") : rotaryNames.back();
        const QByteArray tiltBytes = tilt.toLatin1(), spinBytes = spin.toLatin1();
        layout.push_back(axisEntry(tiltBytes.constData(), "TableTilt"));
        layout.push_back(axisEntry(spinBytes.constData(), "TableSpin"));
    }
    root["machiningMode"] = mode.toStdString();
    root["solverId"] = mode.toStdString();
    root["solverVersion"] = mode == QStringLiteral("Planar3Axis") ? 3 : 2;
    root["machineConfigurationFingerprint"] = std::string();
    root["machineAxisLayout"] = layout;
    // Workpiece mounting is machine-model state in v5 and must not be carried
    // into the upgraded project package.
    // 中文翻译：v5 的工件装夹属于机床模型状态，升级后的项目包不得保留该字段。
    root.as_table().erase("workpieceSetup");
    if (root.contains("pipelineStages") && root.at("pipelineStages").is_array()) {
        for (toml::value& stage : root["pipelineStages"].as_array()) {
            if (stage.is_table() && stage.contains("stage") && stage.at("stage").as_integer() == 4) {
                stage["available"] = false;
                stage["dirty"] = true;
                stage["failureReason"] = std::string("Migrated project requires machine-coordinate recomputation");
            }
        }
    }
    std::ofstream output(tomlPath.toStdString(), std::ios::binary | std::ios::trunc);
    output << toml::format(root);
    return output.good();
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    const QStringList args = app.arguments();
    QTextStream err(stderr);
    if (args.size() < 3) {
        err << "Usage: lcnc_project_upgrade <input.lcnc> <output.lcnc> [--tools <tools.toml>]\n";
        return 2;
    }
    const QString input = QFileInfo(args[1]).absoluteFilePath();
    const QString output = QFileInfo(args[2]).absoluteFilePath();
    if (QDir::cleanPath(input).compare(QDir::cleanPath(output), Qt::CaseInsensitive) == 0) {
        err << "Input and output paths must be different.\n";
        return 2;
    }
    if (QFileInfo::exists(output)) {
        err << "Output already exists; refusing to overwrite it.\n";
        return 2;
    }
    QString toolsPath;
    const int toolsIndex = args.indexOf(QStringLiteral("--tools"));
    if (toolsIndex >= 0 && toolsIndex + 1 < args.size())
        toolsPath = QFileInfo(args[toolsIndex + 1]).absoluteFilePath();

    QTemporaryDir staging;
    QString error;
    if (!staging.isValid() || JlCompress::extractDir(input, staging.path()).isEmpty()) {
        err << "Unable to extract input package.\n";
        return 1;
    }
    const QString manifestPath = QDir(staging.path()).filePath(QStringLiteral("project.toml"));
    toml::value manifest;
    try { manifest = toml::parse(manifestPath.toStdString()); }
    catch (const std::exception& exception) { err << exception.what() << '\n'; return 1; }
    const int version = manifest.contains("formatVersion")
        ? static_cast<int>(manifest.at("formatVersion").as_integer()) : 0;
    if (version < 1 || version > 4) {
        err << "Only project versions 1 through 4 can be upgraded.\n";
        return 1;
    }
    const QString legacyXbf = QDir(staging.path()).filePath(QStringLiteral("project.xbf"));
    const QString currentXbf = QDir(staging.path()).filePath(QStringLiteral("workpiece.xbf"));
    if (!QFileInfo::exists(currentXbf) && QFileInfo::exists(legacyXbf)
        && !QFile::rename(legacyXbf, currentXbf)) {
        err << "Unable to rename legacy XCAF resource.\n";
        return 1;
    }
    const QString snapshotPath = QDir(staging.path()).filePath(QStringLiteral("tools.toml"));
    if (!QFileInfo::exists(snapshotPath)
        && (toolsPath.isEmpty() || !copyRequired(toolsPath, snapshotPath, &error))) {
        err << "Historical package has no tools snapshot; supply --tools <tools.toml>.\n";
        return 1;
    }
    if (!upgradeCam(staging.path(), &error)) { err << error << '\n'; return 1; }
    manifest["formatVersion"] = 5;
    manifest["toolpathAlgorithmVersion"] = std::string("unsolved:0");
    manifest["configurationSchemaVersion"] = std::string("2");
    toml::value resources = manifest.contains("resources") && manifest.at("resources").is_table()
        ? manifest.at("resources") : toml::value(toml::table{});
    resources["workpieceXcaf"] = std::string("workpiece.xbf");
    resources["camCacheDirectory"] = std::string("cam/cache");
    resources["toolSnapshot"] = std::string("tools.toml");
    if (resources.contains("projectXcaf")) resources.as_table().erase("projectXcaf");
    manifest["resources"] = resources;
    std::ofstream manifestOutput(manifestPath.toStdString(), std::ios::binary | std::ios::trunc);
    manifestOutput << toml::format(manifest);
    manifestOutput.close();
    if (!JlCompress::compressDir(output, staging.path(), true)) {
        QFile::remove(output);
        err << "Unable to create upgraded package.\n";
        return 1;
    }
    return 0;
}
