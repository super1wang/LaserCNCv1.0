#include "modules/cad/services/model_envelope_service.h"

#include "core/machine/model_envelope_asset.h"
#include "core/task/task_progress.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QSaveFile>
#include <QTemporaryDir>

namespace {

bool copyFileAtomically(const QString& sourcePath,
                        const QString& targetPath,
                        QString* errorMessage)
{
    QFile source(sourcePath);
    QSaveFile target(targetPath);
    if (!source.open(QIODevice::ReadOnly)
        || !QDir().mkpath(QFileInfo(targetPath).absolutePath())
        || !target.open(QIODevice::WriteOnly)) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to open simplified STEP output: %1")
                                .arg(targetPath);
        return false;
    }
    QByteArray buffer(1024 * 1024, Qt::Uninitialized);
    while (!source.atEnd()) {
        const qint64 count = source.read(buffer.data(), buffer.size());
        if (count <= 0 || target.write(buffer.constData(), count) != count) {
            if (errorMessage)
                *errorMessage = QStringLiteral("Unable to write simplified STEP output: %1")
                                    .arg(targetPath);
            return false;
        }
    }
    if (!target.commit()) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to publish simplified STEP output: %1")
                                .arg(targetPath);
        return false;
    }
    return true;
}

} // namespace

namespace lcnc::cad {

bool ModelEnvelopeService::generate(const CadModelEnvelopeRequest& request,
                                    TaskProgress* progress,
                                    CadModelEnvelopeResult* result,
                                    QString* errorMessage)
{
    if (!result || !QFileInfo(request.sourceModelPath).isFile()
        || !QFileInfo(request.generatorPath).isFile()
        || request.assetRootDirectory.trimmed().isEmpty()) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Model-envelope request is incomplete");
        return false;
    }
    *result = {};
    const int progressMinimum = qBound(0, request.progressMinimum, 100);
    const int progressMaximum = qBound(progressMinimum, request.progressMaximum, 100);
    const auto setProgress = [progress, progressMinimum, progressMaximum](
                                 int percent, const QString& step) {
        if (!progress)
            return;
        progress->setStepName(step);
        progress->setValue(progressMinimum
            + qBound(0, percent, 100) * (progressMaximum - progressMinimum) / 100);
    };
    const QString assetRoot = QFileInfo(request.assetRootDirectory).absoluteFilePath();
    if (!QDir().mkpath(assetRoot)
        || !QDir().mkpath(QDir(assetRoot).filePath(QStringLiteral("generations")))) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to create model-envelope asset directory");
        return false;
    }
    QTemporaryDir staging(QDir(assetRoot).filePath(
        QStringLiteral(".staging-XXXXXX")));
    if (!staging.isValid()) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to create model-envelope staging directory");
        return false;
    }
    const QString meshDirectory = staging.filePath(QStringLiteral("meshes"));
    const QString manifestPath = staging.filePath(
        QStringLiteral("model_envelope.json"));
    const QString simplifiedStepPath = request.simplifiedStepOutputPath.isEmpty()
        ? QString{} : staging.filePath(QStringLiteral("simplified_machine.step"));
    if (progress) {
        progress->setRange(0, 100);
    }
    // 中文翻译：正在生成保守模型包络...
    setProgress(2, QObject::tr("Generating conservative model envelope..."));
    QProcess process;
    process.setProgram(request.generatorPath);
    QStringList arguments{
        QStringLiteral("--machine"), request.sourceModelPath,
        QStringLiteral("--deflection"), QString::number(request.deflectionMm, 'g', 17),
        QStringLiteral("--angle"), QString::number(request.angleRad, 'g', 17),
        QStringLiteral("--alpha"), QString::number(request.alphaMm, 'g', 17),
        QStringLiteral("--offset"), QString::number(request.offsetMm, 'g', 17),
        QStringLiteral("--samples"),
        QString::number(request.maximumValidationSamples),
        QStringLiteral("--output-dir"), meshDirectory,
        QStringLiteral("--report"), manifestPath};
    if (!simplifiedStepPath.isEmpty())
        arguments.append({QStringLiteral("--step-output"), simplifiedStepPath});
    if (!request.detailAxes.isEmpty()) {
        arguments.append({
            QStringLiteral("--detail-axes"), request.detailAxes.join(','),
            QStringLiteral("--detail-alpha"),
            QString::number(request.detailAlphaMm, 'g', 17),
            QStringLiteral("--detail-offset"),
            QString::number(request.detailOffsetMm, 'g', 17)});
    }
    process.setArguments(arguments);
    process.start();
    if (!process.waitForStarted(10'000)) {
        if (errorMessage)
            *errorMessage = process.errorString();
        return false;
    }
    while (!process.waitForFinished(100)) {
        if (progress && progress->isAbortRequested()) {
            process.terminate();
            if (!process.waitForFinished(2'000)) {
                process.kill();
                process.waitForFinished(10'000);
            }
            if (errorMessage)
                *errorMessage = QStringLiteral("Model-envelope generation cancelled");
            return false;
        }
    }
    const QByteArray standardError = process.readAllStandardError();
    if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0) {
        if (errorMessage) {
            *errorMessage = standardError.isEmpty()
                ? QStringLiteral("Model-envelope generator failed with exit code %1")
                      .arg(process.exitCode())
                : QString::fromUtf8(standardError).trimmed();
        }
        return false;
    }
    // 中文翻译：正在验证模型包络...
    setProgress(92, QObject::tr("Validating model envelope..."));
    ModelEnvelopeAsset asset;
    if (!ModelEnvelopeAsset::loadAndValidate(
            manifestPath, request.sourceModelPath, &asset, errorMessage)) {
        return false;
    }
    const QString generationName = QString::fromLatin1(asset.manifestSha256.toHex());
    const QString generationDirectory = QDir(assetRoot).filePath(
        QStringLiteral("generations/%1").arg(generationName));
    QString publishedManifest = QDir(generationDirectory).filePath(
        QStringLiteral("model_envelope.json"));
    if (!QFileInfo(generationDirectory).isDir()) {
        staging.setAutoRemove(false);
        if (!QDir().rename(staging.path(), generationDirectory)) {
            staging.setAutoRemove(true);
            if (errorMessage)
                *errorMessage = QStringLiteral("Unable to publish model-envelope generation");
            return false;
        }
    }
    ModelEnvelopeAsset publishedAsset;
    if (!ModelEnvelopeAsset::loadAndValidate(
            publishedManifest, request.sourceModelPath, &publishedAsset,
            errorMessage)
        || publishedAsset.manifestSha256 != asset.manifestSha256) {
        return false;
    }
    if (!request.simplifiedStepOutputPath.isEmpty()) {
        if (publishedAsset.simplifiedStepPath.isEmpty()
            || !copyFileAtomically(
                QDir(QFileInfo(publishedManifest).absolutePath()).filePath(
                    publishedAsset.simplifiedStepPath),
                QFileInfo(request.simplifiedStepOutputPath).absoluteFilePath(),
                errorMessage)) {
            return false;
        }
        result->simplifiedStepOutputPath =
            QFileInfo(request.simplifiedStepOutputPath).absoluteFilePath();
    }
    QJsonObject pointer{
        {QStringLiteral("schema"), QStringLiteral("lcnc.model-envelope-current/v1")},
        {QStringLiteral("manifest"),
         QDir(assetRoot).relativeFilePath(publishedManifest).replace('\\', '/')},
        {QStringLiteral("manifest_sha256"), generationName}};
    QSaveFile current(QDir(assetRoot).filePath(QStringLiteral("current.json")));
    const QByteArray pointerBytes = QJsonDocument(pointer).toJson(QJsonDocument::Indented);
    if (!current.open(QIODevice::WriteOnly)
        || current.write(pointerBytes) != pointerBytes.size()
        || !current.commit()) {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unable to atomically publish model-envelope pointer");
        return false;
    }
    result->manifestPath = publishedManifest;
    result->manifestSha256 = publishedAsset.manifestSha256;
    result->bodyCount = publishedAsset.bodies.size();
    // 中文翻译：模型包络已就绪
    setProgress(100, QObject::tr("Model envelope ready"));
    return true;
}

} // namespace lcnc::cad
