#include "core/algorithms/cam/machine_safety_index.h"
#include "core/algorithms/cam/machine_motion_certificate.h"
#include "core/algorithms/cam/machine_safety_fingerprint.h"
#include "core/machine/machine_safety_package.h"

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QProcess>
#include <QTemporaryDir>
#include <QTextStream>

#include <JlCompress.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>

namespace {

bool require(bool condition, const QString& message)
{
    if (!condition)
        std::cerr << message.toStdString() << '\n';
    return condition;
}

lcnc::cam_algo::MachineSafetyPose arbitraryPoseInsideSafeCell(
    const lcnc::cam_algo::MachineSafetyIndex& index,
    lcnc::cam_algo::MachineSafetyPose pose)
{
    if (pose.count == 0)
        return pose;
    const auto& axis = index.axes().front();
    const double candidate = pose.values[0] + axis.step * 0.17;
    if (candidate < axis.maximum)
        pose.values[0] = candidate;
    return pose;
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    lcnc::cam_algo::MachineSafetyRuntimeAxis testAxis;
    testAxis.name = QStringLiteral("A");
    testAxis.motionType = 1;
    testAxis.direction = {1.0, 0.0, 0.0};
    testAxis.minimum = -120.0;
    testAxis.maximum = 120.0;
    testAxis.parentAxis = QStringLiteral("BASE");
    const QList<lcnc::cam_algo::MachineSafetyRuntimeAxis> testAxes{testAxis};
    const QByteArray stableFingerprint =
        lcnc::cam_algo::machineSafetyRuntimeFingerprint(
            QStringLiteral("VERTICAL_AC_TABLE"), testAxes,
            {QStringLiteral("BASE"), QStringLiteral("A"), QStringLiteral("A")}, 1);
    if (!require(stableFingerprint ==
                     lcnc::cam_algo::machineSafetyRuntimeFingerprint(
                         QStringLiteral("VERTICAL_AC_TABLE"), testAxes,
                         {QStringLiteral("BASE"), QStringLiteral("A"), QStringLiteral("A")}, 1),
                 QStringLiteral("Equivalent ordered part assignments changed the runtime fingerprint"))
        || !require(stableFingerprint !=
                        lcnc::cam_algo::machineSafetyRuntimeFingerprint(
                            QStringLiteral("VERTICAL_AC_TABLE"), testAxes,
                            {QStringLiteral("BASE"), QStringLiteral("A"), QStringLiteral("BASE")}, 1),
                    QStringLiteral("A changed part-axis assignment did not invalidate the runtime fingerprint"))) {
        return 1;
    }
    QTemporaryDir temporary;
    if (!require(temporary.isValid(), QStringLiteral("Unable to create temporary directory")))
        return 1;
    const QString outputPath = temporary.filePath(QStringLiteral("ac-table.lmsi"));
    const QString packagePath = temporary.filePath(QStringLiteral("ac-table.lmsp"));
    const QByteArray runtimeConfigurationSha256(32, '\x5a');
    QProcess generator;
    generator.setProgram(QString::fromUtf8(LCNC_MACHINE_SAFETY_INDEX_TOOL_PATH));
    generator.setArguments({QStringLiteral("--machine"),
                            QString::fromUtf8(LCNC_AC_TABLE_MODEL_PATH),
                            QStringLiteral("--output"), outputPath,
                            QStringLiteral("--package-output"), packagePath,
                            QStringLiteral("--runtime-configuration-sha256"),
                            QString::fromLatin1(runtimeConfigurationSha256.toHex()),
                            QStringLiteral("--exact-budget"), QStringLiteral("1")});
    QElapsedTimer generationTimer;
    generationTimer.start();
    generator.start();
    if (!require(generator.waitForStarted(10'000),
                 QStringLiteral("Machine safety generator did not start"))
        || !require(generator.waitForFinished(900'000),
                    QStringLiteral("Machine safety generator timed out"))) {
        return 1;
    }
    const QByteArray generatorOutput = generator.readAllStandardOutput();
    const QByteArray generatorError = generator.readAllStandardError();
    if (!require(generator.exitStatus() == QProcess::NormalExit
                 && generator.exitCode() == 0,
                 QStringLiteral("Machine safety generator failed: %1")
                    .arg(QString::fromUtf8(generatorError)))) {
        return 1;
    }
    const qint64 generationWallMs = generationTimer.elapsed();
    if (!require(generatorOutput.contains("leaf_exact_queries=2")
                 && generatorOutput.contains("whole_exact_queries=0"),
                 QStringLiteral("Generator did not use the expected leaf exact path"))) {
        return 1;
    }
    if (!require(generatorOutput.contains("LCNC_PROGRESS|3|loading_machine")
                 && generatorOutput.contains("|base_grid")
                 && generatorOutput.contains("|refinement_level_1")
                 && generatorOutput.contains("LCNC_PROGRESS|100|complete"),
                 QStringLiteral("Generator did not stream complete build progress"))) {
        return 1;
    }

    QElapsedTimer loadTimer;
    loadTimer.start();
    lcnc::cam_algo::MachineSafetyIndex index;
    QString error;
    if (!require(index.load(outputPath, &error),
                 QStringLiteral("Unable to load generated index: %1").arg(error))) {
        return 1;
    }
    const qint64 loadMs = loadTimer.elapsed();
    QTemporaryDir extractedPackage;
    lcnc::MachineSafetyPackageLoadResult package;
    if (!require(extractedPackage.isValid()
                 && lcnc::MachineSafetyPackage::extractAndValidate(
                     packagePath, extractedPackage.path(), &package, &error),
                 QStringLiteral("Unable to validate generated .lmsp package: %1")
                     .arg(error))
        || !require(QFileInfo(package.modelPath).isFile()
                    && QFileInfo(package.safetyIndexPath).isFile()
                    && package.manifest.runtimeConfigurationSha256
                        == runtimeConfigurationSha256,
                    QStringLiteral("Generated .lmsp resources are missing"))) {
        return 1;
    }
    const QString duplicateIndex = QDir(extractedPackage.path()).filePath(
        QStringLiteral("safety/duplicate.lmsi"));
    const QString duplicatePackage = temporary.filePath(QStringLiteral("duplicate.lmsp"));
    if (!require(QFile::copy(package.safetyIndexPath, duplicateIndex)
                 && JlCompress::compressDir(duplicatePackage,
                                             extractedPackage.path(), true),
                 QStringLiteral("Unable to create duplicate-index package fixture"))) {
        return 1;
    }
    QTemporaryDir duplicateExtraction;
    QString duplicateError;
    if (!require(!lcnc::MachineSafetyPackage::extractAndValidate(
                     duplicatePackage, duplicateExtraction.path(), nullptr,
                     &duplicateError),
                 QStringLiteral("Package with multiple .lmsi files was accepted"))) {
        return 1;
    }
    QFile::remove(duplicateIndex);

    QFile modifiedModel(package.modelPath);
    if (!require(modifiedModel.open(QIODevice::Append)
                 && modifiedModel.write("modified", 8) == 8,
                 QStringLiteral("Unable to modify package model fixture"))) {
        return 1;
    }
    modifiedModel.close();
    const QString modifiedPackage = temporary.filePath(QStringLiteral("modified.lmsp"));
    if (!require(JlCompress::compressDir(modifiedPackage,
                                         extractedPackage.path(), true),
                 QStringLiteral("Unable to create modified-model package fixture"))) {
        return 1;
    }
    QTemporaryDir modifiedExtraction;
    QString modifiedError;
    if (!require(!lcnc::MachineSafetyPackage::extractAndValidate(
                     modifiedPackage, modifiedExtraction.path(), nullptr,
                     &modifiedError),
                 QStringLiteral("Package with a modified machine model was accepted"))) {
        return 1;
    }
    const qint64 fileBytes = QFileInfo(outputPath).size();
    const auto effectiveSafeCells = index.effectiveStateCount(
        lcnc::cam_algo::MachineSafetyIndexState::CertifiedSafe);
    const auto collisionCells = index.effectiveStateCount(
        lcnc::cam_algo::MachineSafetyIndexState::CollisionSample);
    const auto unknownCells = index.effectiveStateCount(
        lcnc::cam_algo::MachineSafetyIndexState::Unknown);

    const QString scalarOutputPath = temporary.filePath(
        QStringLiteral("ac-table-scalar.lmsi"));
    QProcess scalarGenerator;
    scalarGenerator.setProgram(QString::fromUtf8(LCNC_MACHINE_SAFETY_INDEX_TOOL_PATH));
    scalarGenerator.setArguments({QStringLiteral("--machine"),
                                  QString::fromUtf8(LCNC_AC_TABLE_MODEL_PATH),
                                  QStringLiteral("--output"), scalarOutputPath,
                                  QStringLiteral("--exact-budget"), QStringLiteral("0"),
                                  QStringLiteral("--motion-bound"), QStringLiteral("scalar"),
                                  QStringLiteral("--refine-levels"), QStringLiteral("0"),
                                  QStringLiteral("--surface-bvh"), QStringLiteral("off"),
                                  QStringLiteral("--leaf-bvh"), QStringLiteral("off")});
    QElapsedTimer scalarGenerationTimer;
    scalarGenerationTimer.start();
    scalarGenerator.start();
    if (!require(scalarGenerator.waitForStarted(10'000),
                 QStringLiteral("Scalar baseline generator did not start"))
        || !require(scalarGenerator.waitForFinished(900'000),
                    QStringLiteral("Scalar baseline generator timed out"))
        || !require(scalarGenerator.exitStatus() == QProcess::NormalExit
                    && scalarGenerator.exitCode() == 0,
                    QStringLiteral("Scalar baseline generator failed: %1")
                        .arg(QString::fromUtf8(scalarGenerator.readAllStandardError())))) {
        return 1;
    }
    const qint64 scalarGenerationWallMs = scalarGenerationTimer.elapsed();
    lcnc::cam_algo::MachineSafetyIndex scalarIndex;
    if (!require(scalarIndex.load(scalarOutputPath, &error),
                 QStringLiteral("Unable to load scalar baseline index: %1").arg(error))) {
        return 1;
    }
    const auto scalarSafeCells = scalarIndex.effectiveStateCount(
        lcnc::cam_algo::MachineSafetyIndexState::CertifiedSafe);
    if (!require(index.bodies().size() >= 6, QStringLiteral("Expected six named axis bodies"))
        || !require(index.pairs().size() >= 4, QStringLiteral("Expected AC cross-branch pairs"))
        || !require(index.cellCount() == 400'000, QStringLiteral("Safety grid resolution changed"))
        || !require(effectiveSafeCells > 0, QStringLiteral("Index contains no certified safe cell"))
        || !require(index.decisionCoverage()
                        > scalarIndex.decisionCoverage() + 0.30,
                    QStringLiteral("Adaptive bounds did not improve coverage by 30 points"))
        || !require(index.decisionCoverage() >= 0.85,
                    QStringLiteral("Adaptive index regressed below 85% decision coverage"))
        || !require(collisionCells > 0, QStringLiteral("Index contains no exact collision sample"))
        || !require(fileBytes > 0 && fileBytes < 100 * 1024 * 1024,
                    QStringLiteral("Index file size is outside the MVP budget"))
        || !require(generationWallMs < 900'000,
                    QStringLiteral("Index generation exceeded 15 minutes"))
        || !require(loadMs < 5'000, QStringLiteral("Index cold load exceeded 5 seconds"))) {
        return 1;
    }

    const auto safeCenter = index.firstPose(
        lcnc::cam_algo::MachineSafetyIndexState::CertifiedSafe);
    const auto arbitrarySafe = arbitraryPoseInsideSafeCell(index, safeCenter);
    const auto safeQuery = index.query(arbitrarySafe);
    const auto collisionPose = index.firstPose(
        lcnc::cam_algo::MachineSafetyIndexState::CollisionSample);
    const auto collisionQuery = index.query(collisionPose);
    const auto arbitraryCollisionCellPose = arbitraryPoseInsideSafeCell(
        index, collisionPose);
    const auto arbitraryCollisionCellQuery = index.query(
        arbitraryCollisionCellPose);
    lcnc::cam_algo::MachineMotionCertificateKey motionKey;
    motionKey.machineSourceSha256 = index.sourceSha256();
    motionKey.safetyIndexSha256 = index.contentSha256();
    motionKey.safetyPolicySha256 = QByteArrayLiteral("unit-safety-policy");
    motionKey.motionProfileSha256 = QByteArrayLiteral("unit-linear-profile");
    motionKey.pathRevision = 7;
    motionKey.edgeId = 3;
    const auto safeEdge = lcnc::cam_algo::certifyLinearMotionEdgeWithIndex(
        index, motionKey, safeCenter, arbitrarySafe);
    const auto collisionEdge = lcnc::cam_algo::certifyLinearMotionEdgeWithIndex(
        index, motionKey, collisionPose, collisionPose);
    lcnc::cam_algo::MachineSafetyPose safeMidpoint = safeCenter;
    for (int axis = 0; axis < safeMidpoint.count; ++axis)
        safeMidpoint.values[axis] = (safeCenter.values[axis]
                                    + arbitrarySafe.values[axis]) * 0.5;
    auto staleMotionKey = motionKey;
    ++staleMotionKey.pathRevision;
    auto deviatedMidpoint = safeMidpoint;
    deviatedMidpoint.values[0] += 0.001;
    if (!require(safeQuery.inRange
                 && safeQuery.state == lcnc::cam_algo::MachineSafetyIndexState::CertifiedSafe,
                 QStringLiteral("Arbitrary APOS in certified cell did not query as safe"))
        || !require(collisionQuery.inRange && collisionQuery.exactPoseCertificate
                 && collisionQuery.state
                    == lcnc::cam_algo::MachineSafetyIndexState::CollisionSample,
                 QStringLiteral("Exact collision APOS certificate did not query as collision"))
        || !require(arbitraryCollisionCellQuery.inRange
                 && !arbitraryCollisionCellQuery.exactPoseCertificate
                 && arbitraryCollisionCellQuery.state
                    == lcnc::cam_algo::MachineSafetyIndexState::Unknown,
                 QStringLiteral("Exact collision sample leaked into volumetric coverage"))
        || !require(safeEdge.state
                        == lcnc::cam_algo::MachineMotionCertificateState::CertifiedSafe,
                    QStringLiteral("Safe LMSI cell did not certify a contained motion edge"))
        || !require(collisionEdge.state
                        != lcnc::cam_algo::MachineMotionCertificateState::CertifiedSafe,
                    QStringLiteral("Collision pose produced a false-safe motion edge"))
        || !require(lcnc::cam_algo::machinePoseMatchesLinearMotionCertificate(
                        safeEdge, motionKey, 0.5, safeMidpoint),
                    QStringLiteral("Valid APOS did not match its motion certificate"))
        || !require(!lcnc::cam_algo::machinePoseMatchesLinearMotionCertificate(
                        safeEdge, staleMotionKey, 0.5, safeMidpoint)
                    && !lcnc::cam_algo::machinePoseMatchesLinearMotionCertificate(
                        safeEdge, motionKey, 0.5, deviatedMidpoint),
                    QStringLiteral("Stale key or APOS deviation bypassed the certificate"))) {
        return 1;
    }

    const QString corruptedPath = temporary.filePath(QStringLiteral("corrupted.lmsi"));
    if (!require(QFile::copy(outputPath, corruptedPath),
                 QStringLiteral("Unable to copy index for checksum test"))) {
        return 1;
    }
    QFile corrupted(corruptedPath);
    if (!require(corrupted.open(QIODevice::ReadWrite) && corrupted.size() > 0,
                 QStringLiteral("Unable to open checksum test index"))) {
        return 1;
    }
    corrupted.seek(corrupted.size() - 1);
    QByteArray finalByte = corrupted.read(1);
    finalByte[0] = static_cast<char>(finalByte.at(0) ^ 0x5a);
    corrupted.seek(corrupted.size() - 1);
    corrupted.write(finalByte);
    corrupted.close();
    lcnc::cam_algo::MachineSafetyIndex rejectedIndex;
    QString checksumError;
    if (!require(!rejectedIndex.load(corruptedPath, &checksumError)
                 && checksumError.contains(QStringLiteral("checksum"),
                                           Qt::CaseInsensitive),
                 QStringLiteral("Corrupted safety index was not rejected"))) {
        return 1;
    }

    lcnc::cam_algo::MachineSafetyIndexCompiler oracle;
    if (!require(oracle.loadMachine(QString::fromUtf8(LCNC_AC_TABLE_MODEL_PATH), &error),
                 QStringLiteral("Unable to reload source model for exact oracle: %1").arg(error))) {
        return 1;
    }
    auto cacheOptions = lcnc::cam_algo::defaultAcTableSafetyBuildOptions();
    for (auto& axis : cacheOptions.axes) {
        axis.cellCount = 2;
        axis.step = (axis.maximum - axis.minimum) / axis.cellCount;
    }
    cacheOptions.maximumCells = 64;
    cacheOptions.maximumRefinedCells = 2'048;
    cacheOptions.maximumExactQueries = 0;
    cacheOptions.refinementThreads = 1;
    lcnc::cam_algo::MachineSafetyIndex cacheOnIndex;
    lcnc::cam_algo::MachineSafetyIndex cacheOffIndex;
    cacheOptions.useDependencyCache = true;
    if (!require(oracle.build(cacheOptions, &cacheOnIndex, nullptr, &error),
                 QStringLiteral("Unable to build dependency-cache test index: %1")
                     .arg(error))) {
        return 1;
    }
    cacheOptions.useDependencyCache = false;
    if (!require(oracle.build(cacheOptions, &cacheOffIndex, nullptr, &error),
                 QStringLiteral("Unable to build cache-off test index: %1").arg(error))
        || !require(cacheOnIndex.effectiveStateCount(
                        lcnc::cam_algo::MachineSafetyIndexState::CertifiedSafe)
                        == cacheOffIndex.effectiveStateCount(
                            lcnc::cam_algo::MachineSafetyIndexState::CertifiedSafe)
                    && cacheOnIndex.effectiveStateCount(
                        lcnc::cam_algo::MachineSafetyIndexState::Unknown)
                        == cacheOffIndex.effectiveStateCount(
                            lcnc::cam_algo::MachineSafetyIndexState::Unknown),
                    QStringLiteral("Dependency cache changed safety classification"))) {
        return 1;
    }
    for (std::uint64_t cell = 0; cell < cacheOnIndex.cellCount(); ++cell) {
        const auto pose = cacheOnIndex.cellCenter(cell);
        if (!require(cacheOnIndex.query(pose).state
                         == cacheOffIndex.query(pose).state,
                     QStringLiteral("Dependency cache changed a grid-cell query"))) {
            return 1;
        }
    }
    const auto hotUnknown = cacheOnIndex.firstPose(
        lcnc::cam_algo::MachineSafetyIndexState::Unknown);
    if (hotUnknown.count > 0) {
        auto deepOptions = cacheOptions;
        deepOptions.useDependencyCache = true;
        deepOptions.refinementLevels = 2;
        deepOptions.maximumRefinedCells = 100'000;
        deepOptions.hotPoses = {hotUnknown};
        deepOptions.hotRefinementRadiusCells = 0.01;
        deepOptions.checkpointPath = temporary.filePath(
            QStringLiteral("multi-level.checkpoint.lmsi"));
        lcnc::cam_algo::MachineSafetyIndex deepIndex;
        if (!require(oracle.build(deepOptions, &deepIndex, nullptr, &error),
                     QStringLiteral("Unable to build multi-level hot index: %1")
                         .arg(error))
            || !require(deepIndex.maximumRefinementLevel() == 2,
                        QStringLiteral("Hot APOS did not receive level-2 refinement"))
            || !require(deepIndex.query(hotUnknown).refinementLevel == 2,
                        QStringLiteral("Level-2 query did not descend the sparse tree"))) {
            return 1;
        }
        lcnc::cam_algo::MachineSafetyIndex checkpointIndex;
        if (!require(checkpointIndex.load(deepOptions.checkpointPath, &error),
                     QStringLiteral("Unable to load multi-level checkpoint: %1")
                         .arg(error))
            || !require(checkpointIndex.maximumRefinementLevel() == 2,
                        QStringLiteral("Checkpoint lost multi-level refinement"))) {
            return 1;
        }
        lcnc::cam_algo::MachineSafetyIndex resumedIndex;
        lcnc::cam_algo::MachineSafetyBuildMetrics resumedMetrics;
        if (!require(oracle.build(deepOptions, &resumedIndex,
                                  &resumedMetrics, &error),
                     QStringLiteral("Unable to resume multi-level checkpoint: %1")
                         .arg(error))
            || !require(resumedMetrics.resumedFromCheckpoint
                            && resumedMetrics.checkpointRefinementLevel == 2,
                        QStringLiteral("Compiler rebuilt instead of resuming the checkpoint"))
            || !require(resumedIndex.maximumRefinementLevel() == 2
                            && resumedIndex.query(hotUnknown).state
                                == deepIndex.query(hotUnknown).state,
                        QStringLiteral("Resumed checkpoint changed query semantics"))) {
            return 1;
        }
    }
    const auto safeExact = oracle.validatePoseExact(index, arbitrarySafe);
    const auto collisionExact = oracle.validatePoseExact(index, collisionPose);
    if (!require(safeExact.state
                 == lcnc::cam_algo::MachineSafetyIndexState::CertifiedSafe,
                 QStringLiteral("Certified-safe APOS failed exact reverse validation"))
        || !require(collisionExact.state
                 == lcnc::cam_algo::MachineSafetyIndexState::CollisionSample,
                 QStringLiteral("Collision APOS failed exact reverse validation"))) {
        return 1;
    }

    constexpr int kSafeAuditPoses = 256;
    std::uint64_t auditRandom = 0xd1b54a32d192ed03ull;
    int auditedSafePoses = 0;
    for (int attempt = 0; attempt < 100'000
         && auditedSafePoses < kSafeAuditPoses; ++attempt) {
        lcnc::cam_algo::MachineSafetyPose pose;
        pose.count = static_cast<std::uint8_t>(index.axes().size());
        for (int axisIndex = 0; axisIndex < index.axes().size(); ++axisIndex) {
            auditRandom = auditRandom * 6364136223846793005ull
                + 1442695040888963407ull;
            const double unit = static_cast<double>(auditRandom >> 11)
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
        if (!require(exact.state
                     == lcnc::cam_algo::MachineSafetyIndexState::CertifiedSafe,
                     QStringLiteral("Directional safe cell produced a false-safe audit pose"))) {
            return 1;
        }
        ++auditedSafePoses;
    }
    if (!require(auditedSafePoses == kSafeAuditPoses,
                 QStringLiteral("Unable to collect enough safe audit poses"))) {
        return 1;
    }

    constexpr int kQueries = 500'000;
    std::uint64_t state = 0x9e3779b97f4a7c15ull;
    std::uint64_t answered = 0;
    QVector<qint64> queryLatenciesNs;
    queryLatenciesNs.reserve(100'000);
    QElapsedTimer queryTimer;
    queryTimer.start();
    for (int queryIndex = 0; queryIndex < kQueries; ++queryIndex) {
        lcnc::cam_algo::MachineSafetyPose pose;
        pose.count = static_cast<std::uint8_t>(index.axes().size());
        for (int axisIndex = 0; axisIndex < index.axes().size(); ++axisIndex) {
            state = state * 6364136223846793005ull + 1442695040888963407ull;
            const double unit = static_cast<double>(state >> 11)
                / static_cast<double>(std::uint64_t{1} << 53);
            const auto& axis = index.axes().at(axisIndex);
            pose.values[axisIndex] = axis.minimum
                + unit * (axis.maximum - axis.minimum);
        }
        const auto queryStarted = std::chrono::steady_clock::now();
        const auto query = index.query(pose);
        if (queryIndex < 100'000) {
            queryLatenciesNs.append(std::chrono::duration_cast<
                std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now() - queryStarted).count());
        }
        if (query.state != lcnc::cam_algo::MachineSafetyIndexState::Unknown)
            ++answered;
    }
    const qint64 queryMs = queryTimer.elapsed();
    std::sort(queryLatenciesNs.begin(), queryLatenciesNs.end());
    const qint64 queryP99Ns = queryLatenciesNs.at(
        static_cast<int>(queryLatenciesNs.size() * 99 / 100));
    const double successRate = 100.0 * static_cast<double>(answered) / kQueries;
    if (!require(queryMs < 2'000, QStringLiteral("500,000 index queries exceeded 2 seconds"))
        || !require(queryP99Ns <= 10'000,
                    QStringLiteral("Index query P99 exceeded 10 microseconds"))
        || !require(answered > 0, QStringLiteral("Random APOS query success rate is zero"))) {
        return 1;
    }

    QTextStream(stdout)
        << generatorOutput
        << "test_generation_wall_ms=" << generationWallMs
        << " scalar_generation_wall_ms=" << scalarGenerationWallMs
        << " load_ms=" << loadMs
        << " file_bytes=" << fileBytes
        << " safe_cells=" << effectiveSafeCells
        << " scalar_safe_cells=" << scalarSafeCells
        << " collision_samples=" << collisionCells
        << " unknown_cells=" << unknownCells
        << " query_count=" << kQueries
        << " query_ms=" << queryMs
        << " query_p99_ns=" << queryP99Ns
        << " query_success_pct=" << QString::number(successRate, 'f', 3)
        << " exact_verified_poses=2"
        << " safe_audit_poses=" << auditedSafePoses
        << " exact_verified_accuracy_pct=100.000"
        << " checksum_rejection=1\n";
    return 0;
}
