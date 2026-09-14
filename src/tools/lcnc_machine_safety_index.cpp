#include "core/algorithms/cam/machine_safety_index.h"
#include "core/machine/machine_safety_package.h"
#include "core/machine/model_envelope_asset.h"

#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QSet>
#include <QTemporaryDir>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <memory>
#include <mutex>

namespace {

void emitBuildProgress(int percent, const QString& stage)
{
    static std::mutex outputMutex;
    const QByteArray token = stage.toUtf8();
    std::lock_guard<std::mutex> lock(outputMutex);
    std::fprintf(stdout, "LCNC_PROGRESS|%d|%s\n",
                 std::clamp(percent, 0, 100), token.constData());
    std::fflush(stdout);
}

QString formatPose(const lcnc::cam_algo::MachineSafetyIndex& index,
                   const lcnc::cam_algo::MachineSafetyPose& pose)
{
    QStringList fields;
    for (int axis = 0; axis < pose.count && axis < index.axes().size(); ++axis) {
        fields.append(QStringLiteral("%1=%2")
            .arg(index.axes().at(axis).name)
            .arg(pose.values[axis], 0, 'g', 15));
    }
    return fields.join(QLatin1Char(','));
}

bool parsePose(const lcnc::cam_algo::MachineSafetyIndex& index,
               const QString& text,
               lcnc::cam_algo::MachineSafetyPose* pose)
{
    if (!pose)
        return false;
    *pose = {};
    pose->count = static_cast<std::uint8_t>(index.axes().size());
    QSet<QString> assigned;
    for (const QString& field : text.split(QLatin1Char(','), Qt::SkipEmptyParts)) {
        const QStringList parts = field.split(QLatin1Char('='));
        if (parts.size() != 2)
            return false;
        bool ok = false;
        const double value = parts.at(1).trimmed().toDouble(&ok);
        if (!ok)
            return false;
        const QString name = parts.at(0).trimmed().toUpper();
        int axisIndex = -1;
        for (int indexValue = 0; indexValue < index.axes().size(); ++indexValue) {
            if (index.axes().at(indexValue).name.compare(name, Qt::CaseInsensitive) == 0) {
                axisIndex = indexValue;
                break;
            }
        }
        if (axisIndex < 0 || assigned.contains(name))
            return false;
        assigned.insert(name);
        pose->values[axisIndex] = value;
    }
    return assigned.size() == index.axes().size();
}

bool parsePoseForAxes(
    const QVector<lcnc::cam_algo::MachineSafetyAxisGrid>& axes,
    const QString& text,
    lcnc::cam_algo::MachineSafetyPose* pose)
{
    if (!pose)
        return false;
    *pose = {};
    pose->count = static_cast<std::uint8_t>(axes.size());
    QSet<QString> assigned;
    for (const QString& field : text.split(QLatin1Char(','), Qt::SkipEmptyParts)) {
        const QStringList parts = field.split(QLatin1Char('='));
        if (parts.size() != 2)
            return false;
        bool ok = false;
        const double value = parts.at(1).trimmed().toDouble(&ok);
        if (!ok)
            return false;
        const QString name = parts.at(0).trimmed().toUpper();
        int axisIndex = -1;
        for (int index = 0; index < axes.size(); ++index) {
            if (axes.at(index).name.compare(name, Qt::CaseInsensitive) == 0) {
                axisIndex = index;
                break;
            }
        }
        if (axisIndex < 0 || assigned.contains(name))
            return false;
        assigned.insert(name);
        pose->values[axisIndex] = value;
    }
    return assigned.size() == axes.size();
}

bool loadHotPoses(
    const QString& filePath,
    const QVector<lcnc::cam_algo::MachineSafetyAxisGrid>& axes,
    QVector<lcnc::cam_algo::MachineSafetyPose>* poses,
    QString* error)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        *error = QStringLiteral("Unable to open hot APOS feedback: %1")
            .arg(file.errorString());
        return false;
    }
    QTextStream stream(&file);
    int lineNumber = 0;
    while (!stream.atEnd()) {
        ++lineNumber;
        const QString line = stream.readLine().trimmed();
        if (line.isEmpty() || line.startsWith(QLatin1Char('#')))
            continue;
        lcnc::cam_algo::MachineSafetyPose pose;
        if (!parsePoseForAxes(axes, line, &pose)) {
            *error = QStringLiteral("Invalid hot APOS feedback at line %1")
                .arg(lineNumber);
            return false;
        }
        poses->append(pose);
    }
    if (poses->isEmpty()) {
        *error = QStringLiteral("Hot APOS feedback file contains no poses");
        return false;
    }
    return true;
}

void printSummary(QTextStream& out,
                  const lcnc::cam_algo::MachineSafetyIndex& index,
                  qint64 fileBytes)
{
    const auto effectiveSafe = index.effectiveStateCount(
        lcnc::cam_algo::MachineSafetyIndexState::CertifiedSafe);
    const auto collision = index.effectiveStateCount(
        lcnc::cam_algo::MachineSafetyIndexState::CollisionSample);
    const auto unknown = index.effectiveStateCount(
        lcnc::cam_algo::MachineSafetyIndexState::Unknown);
    out << "bodies=" << index.bodies().size()
        << " pairs=" << index.pairs().size()
        << " base_cells=" << index.cellCount()
        << " effective_cells=" << index.effectiveCellCount()
        << " refinement_levels=" << index.maximumRefinementLevel()
        << " safe_cells=" << effectiveSafe
        << " collision_samples=" << collision
        << " unknown_cells=" << unknown
        << " coverage_pct="
        << QString::number(index.decisionCoverage() * 100.0, 'f', 3)
        << " file_bytes=" << fileBytes << '\n';
    const auto safePose = index.firstPose(
        lcnc::cam_algo::MachineSafetyIndexState::CertifiedSafe);
    if (safePose.count > 0)
        out << "safe_apos=" << formatPose(index, safePose) << '\n';
    const auto collisionPose = index.firstPose(
        lcnc::cam_algo::MachineSafetyIndexState::CollisionSample);
    if (collisionPose.count > 0)
        out << "collision_apos=" << formatPose(index, collisionPose) << '\n';
}

struct UnknownAuditMetrics
{
    int requested{0};
    int collected{0};
    int attempts{0};
    int exactSafeFar{0};
    int exactSafeNear{0};
    int clearanceViolation{0};
    int collision{0};
    int failures{0};
    qint64 exactMs{0};
    QVector<int> blockingPairCounts;
};

bool auditUnknownPoses(
    QTextStream& out,
    const lcnc::cam_algo::MachineSafetyIndex& index,
    const QString& machinePath,
    int requested,
    const QString& csvPath,
    QString* errorMessage)
{
    lcnc::cam_algo::MachineSafetyIndexCompiler oracle;
    if (!oracle.loadMachine(machinePath, errorMessage))
        return false;

    QFile csvFile;
    std::unique_ptr<QTextStream> csv;
    if (!csvPath.isEmpty()) {
        csvFile.setFileName(csvPath);
        if (!csvFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
            if (errorMessage) {
                *errorMessage = QStringLiteral("Unable to open Unknown audit CSV: %1")
                    .arg(csvFile.errorString());
            }
            return false;
        }
        csv = std::make_unique<QTextStream>(&csvFile);
        *csv << "sample,cell,refinement_level,index_state,exact_state,"
                "minimum_distance_mm,blocking_pair,apos\n";
    }

    UnknownAuditMetrics metrics;
    metrics.requested = requested;
    metrics.blockingPairCounts.fill(0, index.pairs().size());
    std::uint64_t random = 0x243f6a8885a308d3ull;
    const int maximumAttempts = std::max(10'000, requested * 100);
    const double farThresholdMm = index.clearanceMm()
        + std::max(5.0, index.clearanceMm() * 10.0);
    QElapsedTimer exactTimer;
    exactTimer.start();
    while (metrics.collected < requested && metrics.attempts < maximumAttempts) {
        ++metrics.attempts;
        lcnc::cam_algo::MachineSafetyPose pose;
        pose.count = static_cast<std::uint8_t>(index.axes().size());
        for (int axisIndex = 0; axisIndex < index.axes().size(); ++axisIndex) {
            random = random * 6364136223846793005ull + 1442695040888963407ull;
            const double unit = static_cast<double>(random >> 11)
                / static_cast<double>(std::uint64_t{1} << 53);
            const auto& axis = index.axes().at(axisIndex);
            pose.values[axisIndex] = axis.minimum
                + unit * (axis.maximum - axis.minimum);
        }
        const auto indexed = index.query(pose);
        if (!indexed.inRange
            || indexed.state != lcnc::cam_algo::MachineSafetyIndexState::Unknown) {
            continue;
        }

        const auto exact = oracle.validatePoseExact(index, pose);
        ++metrics.collected;
        QString exactClass;
        if (!exact.failureReason.isEmpty()) {
            ++metrics.failures;
            exactClass = QStringLiteral("ExactFailure");
        } else if (exact.state
                   == lcnc::cam_algo::MachineSafetyIndexState::CertifiedSafe) {
            if (exact.minimumDistanceMm > farThresholdMm) {
                ++metrics.exactSafeFar;
                exactClass = QStringLiteral("ExactSafeFar");
            } else {
                ++metrics.exactSafeNear;
                exactClass = QStringLiteral("ExactSafeNear");
            }
        } else if (exact.minimumDistanceMm <= 1.0e-6) {
            ++metrics.collision;
            exactClass = QStringLiteral("Collision");
        } else {
            ++metrics.clearanceViolation;
            exactClass = QStringLiteral("ClearanceViolation");
        }
        if (exact.blockingPair >= 0
            && exact.blockingPair < metrics.blockingPairCounts.size()) {
            ++metrics.blockingPairCounts[exact.blockingPair];
        }
        if (csv) {
            *csv << metrics.collected << ','
                 << indexed.cellIndex << ','
                 << indexed.refinementLevel << ','
                 << "BoundaryUnknown," << exactClass << ','
                 << QString::number(exact.minimumDistanceMm, 'g', 15) << ','
                 << exact.blockingPair << ','
                 << '"' << formatPose(index, pose) << '"' << '\n';
            csv->flush();
        }
        if (metrics.collected % 16 == 0 || metrics.collected == requested) {
            out << "unknown_audit_progress=" << metrics.collected
                << '/' << requested << '\n';
            out.flush();
        }
    }
    metrics.exactMs = exactTimer.elapsed();
    if (metrics.collected != requested) {
        if (errorMessage) {
            *errorMessage = QStringLiteral(
                "Unable to collect requested Unknown audit poses: %1/%2 after %3 attempts")
                    .arg(metrics.collected).arg(requested).arg(metrics.attempts);
        }
        return false;
    }

    out << "unknown_audit_ok=1"
        << " requested=" << metrics.requested
        << " collected=" << metrics.collected
        << " attempts=" << metrics.attempts
        << " exact_safe_far=" << metrics.exactSafeFar
        << " exact_safe_near=" << metrics.exactSafeNear
        << " clearance_violation=" << metrics.clearanceViolation
        << " collision=" << metrics.collision
        << " exact_failures=" << metrics.failures
        << " exact_ms=" << metrics.exactMs
        << " far_threshold_mm=" << farThresholdMm;
    for (int pair = 0; pair < metrics.blockingPairCounts.size(); ++pair)
        out << " pair_" << pair << '=' << metrics.blockingPairCounts.at(pair);
    out << '\n';
    return true;
}

bool auditCertifiedSafePoses(
    QTextStream& out,
    const lcnc::cam_algo::MachineSafetyIndex& index,
    const QString& machinePath,
    int requested,
    QString* errorMessage)
{
    lcnc::cam_algo::MachineSafetyIndexCompiler oracle;
    if (!oracle.loadMachine(machinePath, errorMessage))
        return false;
    std::uint64_t random = 0xd1b54a32d192ed03ull;
    const int maximumAttempts = std::max(10'000, requested * 100);
    int attempts = 0;
    int collected = 0;
    QElapsedTimer exactTimer;
    exactTimer.start();
    while (collected < requested && attempts < maximumAttempts) {
        ++attempts;
        lcnc::cam_algo::MachineSafetyPose pose;
        pose.count = static_cast<std::uint8_t>(index.axes().size());
        for (int axisIndex = 0; axisIndex < index.axes().size(); ++axisIndex) {
            random = random * 6364136223846793005ull + 1442695040888963407ull;
            const double unit = static_cast<double>(random >> 11)
                / static_cast<double>(std::uint64_t{1} << 53);
            const auto& axis = index.axes().at(axisIndex);
            pose.values[axisIndex] = axis.minimum
                + unit * (axis.maximum - axis.minimum);
        }
        if (index.query(pose).state
            != lcnc::cam_algo::MachineSafetyIndexState::CertifiedSafe) {
            continue;
        }
        const auto exact = oracle.validatePoseExact(index, pose);
        if (exact.state
            != lcnc::cam_algo::MachineSafetyIndexState::CertifiedSafe) {
            if (errorMessage) {
                *errorMessage = QStringLiteral(
                    "CertifiedSafe audit found a false-safe APOS: %1; exact=%2; reason=%3")
                    .arg(formatPose(index, pose),
                         lcnc::cam_algo::machineSafetyIndexStateName(exact.state),
                         exact.failureReason);
            }
            return false;
        }
        ++collected;
    }
    if (collected != requested) {
        if (errorMessage) {
            *errorMessage = QStringLiteral(
                "Unable to collect requested CertifiedSafe audit poses: %1/%2")
                    .arg(collected).arg(requested);
        }
        return false;
    }
    out << "safe_audit_ok=1"
        << " requested=" << requested
        << " collected=" << collected
        << " attempts=" << attempts
        << " false_safe=0"
        << " exact_ms=" << exactTimer.elapsed() << '\n';
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    QCoreApplication::setApplicationName(QStringLiteral("lcnc_machine_safety_index"));
    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral(
        "Build and query a conservative LaserCNC machine safety-domain index"));
    parser.addHelpOption();
    QCommandLineOption machineOption({QStringLiteral("m"), QStringLiteral("machine")},
        QStringLiteral("LCNC_AXIS_* named STEP machine model"), QStringLiteral("path"));
    QCommandLineOption outputOption({QStringLiteral("o"), QStringLiteral("output")},
        QStringLiteral("Output machine safety index"), QStringLiteral("path"));
    QCommandLineOption packageOutputOption(QStringLiteral("package-output"),
        QStringLiteral("Output .lmsp package containing the model and one safety index"),
        QStringLiteral("path"));
    QCommandLineOption runtimeConfigurationOption(
        QStringLiteral("runtime-configuration-sha256"),
        QStringLiteral("Runtime kinematics/policy fingerprint stored in the package"),
        QStringLiteral("hex"));
    QCommandLineOption envelopeManifestOption(
        QStringLiteral("envelope-manifest"),
        QStringLiteral("Validated lcnc.model-envelope/v1 JSON used as the persisted collision mesh source"),
        QStringLiteral("path"));
    QCommandLineOption indexOption({QStringLiteral("i"), QStringLiteral("index")},
        QStringLiteral("Existing machine safety index"), QStringLiteral("path"));
    QCommandLineOption poseOption(QStringLiteral("apos"),
        QStringLiteral("Physical APOS, for example X=0,Y=0,Z=0,A=0,C=0"),
        QStringLiteral("values"));
    QCommandLineOption exactBudgetOption(QStringLiteral("exact-budget"),
        QStringLiteral("Maximum OCC exact center probes during generation"),
        QStringLiteral("count"), QStringLiteral("1"));
    QCommandLineOption exactModeOption(QStringLiteral("exact-mode"),
        QStringLiteral("Exact implementation: leaf or whole"),
        QStringLiteral("mode"), QStringLiteral("leaf"));
    QCommandLineOption motionBoundOption(QStringLiteral("motion-bound"),
        QStringLiteral("Cell motion bound: directional or scalar"),
        QStringLiteral("mode"), QStringLiteral("directional"));
    QCommandLineOption refinementOption(QStringLiteral("refine-levels"),
        QStringLiteral("Sparse refinement levels: 0 through 3; levels 2-3 require hot APOS feedback"),
        QStringLiteral("count"), QStringLiteral("1"));
    QCommandLineOption hotAposFileOption(QStringLiteral("hot-apos-file"),
        QStringLiteral("Text file with one X=...,Y=...,Z=...,A=...,C=... hot pose per line"),
        QStringLiteral("path"));
    QCommandLineOption hotRadiusOption(QStringLiteral("hot-radius-cells"),
        QStringLiteral("Normalized radius for level 2-3 hot refinement"),
        QStringLiteral("radius"), QStringLiteral("2.0"));
    QCommandLineOption checkpointOption(QStringLiteral("checkpoint"),
        QStringLiteral("Atomically replaced valid .lmsi checkpoint path"),
        QStringLiteral("path"));
    QCommandLineOption noResumeOption(QStringLiteral("no-resume"),
        QStringLiteral("Ignore an existing checkpoint and rebuild from the base grid"));
    QCommandLineOption gridProfileOption(QStringLiteral("grid-profile"),
        QStringLiteral("Grid profile: base, ac, or acz"),
        QStringLiteral("name"), QStringLiteral("base"));
    QCommandLineOption surfaceBvhOption(QStringLiteral("surface-bvh"),
        QStringLiteral("Surface BVH certification: on or off"),
        QStringLiteral("mode"), QStringLiteral("off"));
    QCommandLineOption surfaceMeshDeflectionOption(
        QStringLiteral("surface-mesh-deflection"),
        QStringLiteral("Persisted surface mesh linear deflection in millimeters"),
        QStringLiteral("mm"), QStringLiteral("0.5"));
    QCommandLineOption leafBvhOption(QStringLiteral("leaf-bvh"),
        QStringLiteral("Swept CAD leaf BVH certification: on or off"),
        QStringLiteral("mode"), QStringLiteral("off"));
    QCommandLineOption dependencyCacheOption(QStringLiteral("dependency-cache"),
        QStringLiteral("Pair/body dependency-axis cache: on or off"),
        QStringLiteral("mode"), QStringLiteral("on"));
    QCommandLineOption refinementThreadsOption(QStringLiteral("refine-threads"),
        QStringLiteral("Sparse refinement worker count"),
        QStringLiteral("count"), QStringLiteral("8"));
    QCommandLineOption auditUnknownOption(QStringLiteral("audit-unknown"),
        QStringLiteral("Exact-oracle audit count for random BoundaryUnknown APOS"),
        QStringLiteral("count"), QStringLiteral("0"));
    QCommandLineOption auditCsvOption(QStringLiteral("audit-csv"),
        QStringLiteral("Optional CSV output for BoundaryUnknown exact audit"),
        QStringLiteral("path"));
    QCommandLineOption auditSafeOption(QStringLiteral("audit-safe"),
        QStringLiteral("Exact-oracle audit count for random CertifiedSafe APOS"),
        QStringLiteral("count"), QStringLiteral("0"));
    parser.addOptions({machineOption, outputOption, packageOutputOption,
                       runtimeConfigurationOption,
                       envelopeManifestOption,
                       indexOption, poseOption,
                       exactBudgetOption, exactModeOption, motionBoundOption,
                       refinementOption, gridProfileOption, surfaceBvhOption,
                       surfaceMeshDeflectionOption,
                       leafBvhOption, dependencyCacheOption, refinementThreadsOption,
                       hotAposFileOption, hotRadiusOption, checkpointOption,
                       noResumeOption,
                       auditUnknownOption, auditCsvOption, auditSafeOption});
    parser.process(app);
    QTextStream out(stdout);
    QTextStream err(stderr);

    QString resolvedMachinePath = parser.value(machineOption);
    QTemporaryDir extractedMachinePackage;
    lcnc::MachineSafetyPackageLoadResult inputPackage;
    const bool generationRequested = parser.isSet(machineOption)
        && (parser.isSet(outputOption) || parser.isSet(packageOutputOption));
    if (generationRequested)
        emitBuildProgress(1, QStringLiteral("starting"));
    if (parser.isSet(machineOption)
        && lcnc::MachineSafetyPackage::isPackagePath(resolvedMachinePath)) {
        if (generationRequested)
            emitBuildProgress(2, QStringLiteral("extracting_package"));
        QString packageError;
        if (!extractedMachinePackage.isValid()
            || !lcnc::MachineSafetyPackage::extractAndValidate(
                resolvedMachinePath, extractedMachinePackage.path(),
                &inputPackage, &packageError)) {
            err << packageError << '\n';
            return 3;
        }
        resolvedMachinePath = inputPackage.modelPath;
    }

    if (generationRequested) {
        QElapsedTimer productionTimer;
        productionTimer.start();
        bool budgetOk = false;
        const int budget = parser.value(exactBudgetOption).toInt(&budgetOk);
        if (!budgetOk || budget < 0) {
            err << "Invalid --exact-budget\n";
            return 2;
        }
        lcnc::cam_algo::MachineSafetyIndexCompiler compiler;
        QString error;
        emitBuildProgress(3, QStringLiteral("loading_machine"));
        if (!compiler.loadMachine(resolvedMachinePath, &error)) {
            err << error << '\n';
            return 3;
        }
        lcnc::ModelEnvelopeAsset envelopeAsset;
        if (parser.isSet(envelopeManifestOption)) {
            emitBuildProgress(8, QStringLiteral("loading_envelope"));
            if (!lcnc::ModelEnvelopeAsset::loadAndValidate(
                    parser.value(envelopeManifestOption), resolvedMachinePath,
                    &envelopeAsset, &error)
                || !compiler.applyModelEnvelope(envelopeAsset, &error)) {
                err << error << '\n';
                return 13;
            }
        }
        emitBuildProgress(10, QStringLiteral("preparing_index"));
        auto options = lcnc::cam_algo::defaultAcTableSafetyBuildOptions();
        const QString gridProfile = parser.value(gridProfileOption).trimmed().toLower();
        if (gridProfile == QStringLiteral("ac")
            || gridProfile == QStringLiteral("acz")) {
            for (auto& axis : options.axes) {
                if (axis.name == QStringLiteral("A")
                    || axis.name == QStringLiteral("C")
                    || (gridProfile == QStringLiteral("acz")
                        && axis.name == QStringLiteral("Z"))) {
                    axis.step *= 0.5;
                }
            }
            options.maximumCells = 4'000'000;
        } else if (gridProfile != QStringLiteral("base")) {
            err << "Invalid --grid-profile; expected base, ac, or acz\n";
            return 2;
        }
        options.maximumExactQueries = budget;
        const QString exactMode = parser.value(exactModeOption).trimmed().toLower();
        if (exactMode != QStringLiteral("leaf")
            && exactMode != QStringLiteral("whole")) {
            err << "Invalid --exact-mode; expected leaf or whole\n";
            return 2;
        }
        options.useLeafExact = exactMode == QStringLiteral("leaf");
        const QString motionBound = parser.value(motionBoundOption).trimmed().toLower();
        if (motionBound != QStringLiteral("directional")
            && motionBound != QStringLiteral("scalar")) {
            err << "Invalid --motion-bound; expected directional or scalar\n";
            return 2;
        }
        options.useDirectionalMotionBounds =
            motionBound == QStringLiteral("directional");
        const QString surfaceBvh = parser.value(surfaceBvhOption).trimmed().toLower();
        if (surfaceBvh != QStringLiteral("on")
            && surfaceBvh != QStringLiteral("off")) {
            err << "Invalid --surface-bvh; expected on or off\n";
            return 2;
        }
        options.useSurfaceBvhCertification = surfaceBvh == QStringLiteral("on");
        bool surfaceMeshDeflectionOk = false;
        options.surfaceMeshDeflectionMm = parser.value(surfaceMeshDeflectionOption)
            .toDouble(&surfaceMeshDeflectionOk);
        if (!surfaceMeshDeflectionOk
            || !std::isfinite(options.surfaceMeshDeflectionMm)
            || options.surfaceMeshDeflectionMm <= 0.0) {
            err << "Invalid --surface-mesh-deflection\n";
            return 2;
        }
        const QString leafBvh = parser.value(leafBvhOption).trimmed().toLower();
        if (leafBvh != QStringLiteral("on")
            && leafBvh != QStringLiteral("off")) {
            err << "Invalid --leaf-bvh; expected on or off\n";
            return 2;
        }
        options.useLeafBvhCertification = leafBvh == QStringLiteral("on");
        const QString dependencyCache = parser.value(dependencyCacheOption)
            .trimmed().toLower();
        if (dependencyCache != QStringLiteral("on")
            && dependencyCache != QStringLiteral("off")) {
            err << "Invalid --dependency-cache; expected on or off\n";
            return 2;
        }
        options.useDependencyCache = dependencyCache == QStringLiteral("on");
        bool refinementOk = false;
        options.refinementLevels = parser.value(refinementOption).toInt(&refinementOk);
        if (!refinementOk || options.refinementLevels < 0
            || options.refinementLevels > 3) {
            err << "Invalid --refine-levels; expected 0 through 3\n";
            return 2;
        }
        bool hotRadiusOk = false;
        options.hotRefinementRadiusCells = parser.value(hotRadiusOption)
            .toDouble(&hotRadiusOk);
        if (!hotRadiusOk || options.hotRefinementRadiusCells < 0.0) {
            err << "Invalid --hot-radius-cells\n";
            return 2;
        }
        if (parser.isSet(hotAposFileOption)
            && !loadHotPoses(parser.value(hotAposFileOption), options.axes,
                             &options.hotPoses, &error)) {
            err << error << '\n';
            return 2;
        }
        if (options.refinementLevels > 1 && options.hotPoses.isEmpty()) {
            err << "--refine-levels 2 or 3 requires --hot-apos-file\n";
            return 2;
        }
        if (parser.isSet(checkpointOption))
            options.checkpointPath = QFileInfo(parser.value(checkpointOption))
                .absoluteFilePath();
        options.resumeCheckpoint = !parser.isSet(noResumeOption);
        bool refinementThreadsOk = false;
        options.refinementThreads = parser.value(refinementThreadsOption)
            .toInt(&refinementThreadsOk);
        if (!refinementThreadsOk || options.refinementThreads < 1
            || options.refinementThreads > 32) {
            err << "Invalid --refine-threads; expected 1 through 32\n";
            return 2;
        }
        options.progressCallback = [](int percent, const QString& stage) {
            emitBuildProgress(10 + percent * 80 / 100, stage);
        };
        lcnc::cam_algo::MachineSafetyIndex index;
        lcnc::cam_algo::MachineSafetyBuildMetrics metrics;
        if (!compiler.build(options, &index, &metrics, &error)) {
            err << error << '\n';
            return 4;
        }
        QTemporaryDir packageStaging;
        const QString indexOutputPath = parser.isSet(outputOption)
            ? QFileInfo(parser.value(outputOption)).absoluteFilePath()
            : packageStaging.filePath(QStringLiteral("machine.lmsi"));
        if (!parser.isSet(outputOption) && !packageStaging.isValid()) {
            err << "Unable to create package index staging directory\n";
            return 5;
        }
        QElapsedTimer saveTimer;
        saveTimer.start();
        emitBuildProgress(92, QStringLiteral("saving_index"));
        if (!index.save(indexOutputPath, &error)) {
            err << error << '\n';
            return 5;
        }
        metrics.saveMs = saveTimer.elapsed();
        metrics.fileBytes = QFileInfo(indexOutputPath).size();
        bool generationSafeAuditOk = false;
        const int generationSafeAuditCount = parser.value(auditSafeOption)
            .toInt(&generationSafeAuditOk);
        if (!generationSafeAuditOk || generationSafeAuditCount < 0) {
            err << "Invalid --audit-safe\n";
            return 2;
        }
        if (generationSafeAuditCount > 0) {
            emitBuildProgress(95, QStringLiteral("auditing_safe"));
            if (!auditCertifiedSafePoses(out, index, resolvedMachinePath,
                                         generationSafeAuditCount, &error)) {
                err << error << '\n';
                return 10;
            }
        }
        const qint64 measuredBuildMs = metrics.importMs + metrics.gridMs
            + metrics.refinementMs + metrics.surfaceBvhCompileMs
            + metrics.exactMs + metrics.saveMs;
        const double exactShare = measuredBuildMs > 0
            ? 100.0 * static_cast<double>(metrics.exactMs) / measuredBuildMs
            : 0.0;
        constexpr qint64 kMaximumProductionIndexBytes = 100ll * 1024ll * 1024ll;
        if (parser.isSet(packageOutputOption)
            && (productionTimer.elapsed() > 15ll * 60ll * 1000ll
                || exactShare > 10.0
                || metrics.fileBytes > kMaximumProductionIndexBytes)) {
            err << "Production machine safety package gates failed: build_ms="
                << productionTimer.elapsed() << " exact_share_pct="
                << QString::number(exactShare, 'f', 3)
                << " index_bytes=" << metrics.fileBytes << '\n';
            return 12;
        }
        if (parser.isSet(packageOutputOption)) {
            emitBuildProgress(98, QStringLiteral("packaging"));
            QByteArray runtimeConfigurationSha256;
            if (parser.isSet(runtimeConfigurationOption)) {
                const QByteArray encoded = parser.value(runtimeConfigurationOption)
                    .trimmed().toLatin1();
                runtimeConfigurationSha256 = QByteArray::fromHex(encoded);
                if (encoded.size() != 64 || runtimeConfigurationSha256.size() != 32
                    || runtimeConfigurationSha256.toHex() != encoded.toLower()) {
                    err << "Invalid --runtime-configuration-sha256\n";
                    return 2;
                }
            }
            const QString packagePath =
                lcnc::MachineSafetyPackage::normalizedPackagePath(
                    parser.value(packageOutputOption));
            const bool packageCreated = parser.isSet(envelopeManifestOption)
                ? lcnc::MachineSafetyPackage::createWithEnvelope(
                    resolvedMachinePath, indexOutputPath,
                    parser.value(envelopeManifestOption), packagePath,
                    runtimeConfigurationSha256, &error)
                : lcnc::MachineSafetyPackage::create(
                    resolvedMachinePath, indexOutputPath, packagePath,
                    runtimeConfigurationSha256, &error);
            if (!packageCreated) {
                err << error << '\n';
                return 11;
            }
            out << "package_ok=1 package_path=" << packagePath
                << " package_bytes=" << QFileInfo(packagePath).size() << '\n';
        }
        out << "generate_ok=1"
            << " resumed_from_checkpoint="
            << (metrics.resumedFromCheckpoint ? 1 : 0)
            << " checkpoint_level=" << metrics.checkpointRefinementLevel
            << " import_ms=" << metrics.importMs
            << " grid_ms=" << metrics.gridMs
            << " refinement_ms=" << metrics.refinementMs
            << " surface_bvh_compile_ms=" << metrics.surfaceBvhCompileMs
            << " surface_bvh_query_ms=" << metrics.surfaceBvhQueryMs
            << " surface_bvh_queries=" << metrics.surfaceBvhQueries
            << " surface_bvh_certified_pairs="
            << metrics.surfaceBvhCertifiedPairs
            << " leaf_bvh_query_ms=" << metrics.leafBvhQueryMs
            << " leaf_bvh_queries=" << metrics.leafBvhQueries
            << " leaf_bvh_certified_pairs=" << metrics.leafBvhCertifiedPairs
            << " exact_ms=" << metrics.exactMs
            << " exact_queries=" << metrics.exactQueries
            << " exact_failures=" << metrics.exactFailures
            << " geometry_leaves=" << metrics.geometryLeaves
            << " leaf_candidates=" << metrics.leafCandidatePairs
            << " leaf_exact_queries=" << metrics.leafExactQueries
            << " whole_exact_queries=" << metrics.wholeShapeExactQueries
            << " refined_parents=" << metrics.refinedParentCells
            << " refined_cells=" << metrics.refinedCells
            << " refined_safe_cells=" << metrics.refinedSafeCells
            << " refined_unknown_cells=" << metrics.refinedUnknownCells
            << " refinement_body_evaluations="
            << metrics.refinementBodyEvaluations
            << " refinement_body_cache_hits="
            << metrics.refinementBodyCacheHits
            << " refinement_pair_evaluations="
            << metrics.refinementPairEvaluations
            << " refinement_pair_cache_hits="
            << metrics.refinementPairCacheHits
            << " save_ms=" << metrics.saveMs
            << " measured_build_ms=" << measuredBuildMs
            << " exact_share_pct=" << QString::number(exactShare, 'f', 3)
            << '\n';
        printSummary(out, index, metrics.fileBytes);
        out.flush();
        emitBuildProgress(100, QStringLiteral("complete"));
        return 0;
    }

    if (parser.isSet(indexOption)) {
        QElapsedTimer loadTimer;
        loadTimer.start();
        lcnc::cam_algo::MachineSafetyIndex index;
        QString error;
        if (!index.load(parser.value(indexOption), &error)) {
            err << error << '\n';
            return 6;
        }
        out << "load_ok=1 load_ms=" << loadTimer.elapsed() << '\n';
        printSummary(out, index, QFileInfo(parser.value(indexOption)).size());
        bool auditOk = false;
        const int auditCount = parser.value(auditUnknownOption).toInt(&auditOk);
        if (!auditOk || auditCount < 0) {
            err << "Invalid --audit-unknown\n";
            return 2;
        }
        if (auditCount > 0) {
            if (!parser.isSet(machineOption)) {
                err << "--audit-unknown requires --machine\n";
                return 2;
            }
            if (!auditUnknownPoses(out, index, resolvedMachinePath,
                                   auditCount, parser.value(auditCsvOption),
                                   &error)) {
                err << error << '\n';
                return 9;
            }
        }
        bool safeAuditOk = false;
        const int safeAuditCount = parser.value(auditSafeOption).toInt(&safeAuditOk);
        if (!safeAuditOk || safeAuditCount < 0) {
            err << "Invalid --audit-safe\n";
            return 2;
        }
        if (safeAuditCount > 0) {
            if (!parser.isSet(machineOption)) {
                err << "--audit-safe requires --machine\n";
                return 2;
            }
            if (!auditCertifiedSafePoses(out, index,
                                         resolvedMachinePath,
                                         safeAuditCount, &error)) {
                err << error << '\n';
                return 10;
            }
        }
        if (parser.isSet(poseOption)) {
            lcnc::cam_algo::MachineSafetyPose pose;
            if (!parsePose(index, parser.value(poseOption), &pose)) {
                err << "Invalid --apos; every index axis must be provided exactly once\n";
                return 7;
            }
            const auto query = index.query(pose);
            out << "query_state="
                << lcnc::cam_algo::machineSafetyIndexStateName(query.state)
                << " in_range=" << (query.inRange ? 1 : 0)
                << " exact_pose_certificate=" << (query.exactPoseCertificate ? 1 : 0)
                << " cell=" << query.cellIndex
                << " refinement_level=" << query.refinementLevel
                << " clearance_lower_bound_mm=" << query.clearanceLowerBoundMm
                << " blocking_pair=" << query.blockingPair << '\n';
            return query.inRange ? 0 : 8;
        }
        return 0;
    }

    parser.showHelp(1);
}
