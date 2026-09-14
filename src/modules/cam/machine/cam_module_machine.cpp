
#include "core/algorithms/cam/collision_safety_domain.h"
#include "core/algorithms/cam/collision_scan_policy.h"
#include "core/algorithms/cam/face_classifier.h"
#include "core/algorithms/cam/initial_approach_axis_planner.h"
#include "core/algorithms/cam/laser_toolpath.h"
#include "core/algorithms/cam/machine_safety_index.h"
#include "core/algorithms/cam/surface_collision_prefilter.h"
#include "core/algorithms/cam/travel_path_planner.h"
#include "core/algorithms/occt_exact_operation_lock.h"
#include "core/document/lcnc_document.h"
#include "core/document/xcaf_utils.h"
#include "core/kernel/i_kernel.h"
#include "core/kernel/kernel.h"
#include "core/kernel/service_registry.h"
#include "core/kinematics/machine_calibration.h"
#include "core/kinematics/machine_configuration_service.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/kinematics/machine_pose.h"
#include "core/logging/logger.h"
#include "core/machine/machine_safety_package.h"
#include "core/machine/machine_workspace.h"
#include "core/project/cam/cam_data_manager.h"
#include "core/project/cam/layer_container.h"
#include "core/project/cam/layer_manager.h"
#include "core/project/lcnc_project_manager.h"
#include "core/services/selection_service.h"
#include "core/settings/app_settings.h"
#include "core/task/task_manager.h"
#include "modules/cad/i_cad_facade.h"
#include "modules/cad/services/shape_service.h"
#include "modules/cam/cam_module.h"
#include "modules/cam/collision/collision_geometry_cache.h"
#include "modules/cam/collision/cutter_collision_geometry.h"
#include "modules/cam/contracts/cam_events.h"
#include "modules/cam/display/cam_display_projection_service.h"
#include "modules/cam/integration/cam_service_adapters.h"
#include "modules/cam/interaction/reference_pick.h"
#include "modules/cam/internal/cam_module_support.h"
#include "modules/cam/machine/machine_axis_detector.h"
#include "modules/cam/machine/machine_io.h"
#include "modules/cam/settings/cam_config.h"
#include "modules/cam/toolpath/toolpath_generation_service.h"
#include "modules/cam/toolpath/toolpath_sequence_service.h"
#include "modules/cam/toolpath/toolpath_solve_service.h"
#include "view/contour_order_label_renderer.h"
#include "view/graphics_scene.h"
#include "view/gui_application.h"
#include "view/gui_document.h"
#include "view/machine_guide_renderer.h"
#include "view/toolpath_renderer.h"
#include "view/travel_path_renderer.h"
#include "view/widget_occ_view.h"

#include <AIS_DisplayMode.hxx>
#include <Aspect_PolygonOffsetMode.hxx>
#include <Aspect_TypeOfLine.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <Bnd_Box.hxx>
#include <Bnd_OBB.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Graphic3d_ZLayerId.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <NCollection_Map.hxx>
#include <NCollection_Sequence.hxx>
#include <OSD_Parallel.hxx>
#include <OSD_ThreadPool.hxx>
#include <Precision.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <QByteArray>
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QMutex>
#include <QMutexLocker>
#include <QPoint>
#include <QPointer>
#include <QProcess>
#include <QScopeGuard>
#include <QSignalBlocker>
#include <QTemporaryDir>
#include <QThread>
#include <QTimer>
#include <Quantity_Color.hxx>
#include <Quantity_NameOfColor.hxx>
#include <STEPControl_Reader.hxx>
#include <SelectMgr_EntityOwner.hxx>
#include <SelectMgr_SelectableObject.hxx>
#include <StdSelect_BRepOwner.hxx>
#include <StlAPI_Reader.hxx>
#include <TDF_Label.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {
using CamTravelCollisionBody = lcnc::cam::TravelCollisionBody;
using CamTravelCollisionLeaf = lcnc::cam::TravelCollisionLeaf;
using CamTravelCollisionGeometryCache = lcnc::cam::TravelCollisionGeometryCache;
using lcnc::cam::buildCollisionGeometry;
using lcnc::cam::collisionAxisSourceId;
using lcnc::cam::collisionGeometryKey;
using lcnc::cam::transformCollisionAabb;
using lcnc::cam::transformCollisionObb;

QByteArray sha256File(const QString& path)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
        return {};
    QCryptographicHash hash(QCryptographicHash::Sha256);
    return hash.addData(&file) ? hash.result() : QByteArray{};
}

QString modelEnvelopeGeneratorPath()
{
    const QString configured = qEnvironmentVariable("LCNC_MODEL_ENVELOPE_CGAL").trimmed();
    if (!configured.isEmpty() && QFileInfo(configured).isFile())
        return QFileInfo(configured).absoluteFilePath();
    const QString executable =
#ifdef Q_OS_WIN
        QStringLiteral("lcnc_model_envelope_cgal.exe");
#else
        QStringLiteral("lcnc_model_envelope_cgal");
#endif
    const QString appDirectory = QCoreApplication::applicationDirPath();
    const QStringList candidates{
        QDir(appDirectory).filePath(executable),
        QDir::cleanPath(QDir(appDirectory).filePath(
            QStringLiteral("../../../x64/envelope-tools/cgal/Release/")
            + executable))};
    for (const QString& candidate : candidates) {
        if (QFileInfo(candidate).isFile())
            return QFileInfo(candidate).absoluteFilePath();
    }
    return candidates.constLast();
}

QString machineSafetyBuildStageText(const QByteArray& token)
{
    // 中文翻译：启动机台安全包生成器；解压已有机台安全包；加载并分析机台 STEP 模型；
    // 准备安全索引；构建持久化表面 BVH；构建基础安全网格；复用基础网格断点。
    if (token == "starting")
        return QObject::tr("Starting machine safety package generator");
    if (token == "extracting_package")
        return QObject::tr("Extracting existing machine safety package");
    if (token == "loading_machine")
        return QObject::tr("Loading and analyzing the machine STEP model");
    if (token == "preparing_index")
        return QObject::tr("Preparing the machine safety index");
    if (token == "surface_bvh")
        return QObject::tr("Building persistent surface BVH data");
    if (token == "base_grid")
        return QObject::tr("Building the base machine safety grid");
    if (token == "base_grid_reused" || token == "resuming_checkpoint")
        return QObject::tr("Reusing the machine safety checkpoint");

    // 中文翻译：抽样精确碰撞候选；保存安全索引断点；构建一级细化；构建热区细化；
    // 完成安全索引；保存索引；审计安全姿态；打包；生成完成。
    if (token == "exact_sampling")
        return QObject::tr("Sampling exact collision candidates");
    if (token == "saving_checkpoint")
        return QObject::tr("Saving the machine safety checkpoint");
    if (token == "refinement_level_1")
        return QObject::tr("Building level-1 machine safety refinement");
    if (token.startsWith("hot_refinement_"))
        return QObject::tr("Building hot-zone machine safety refinement");
    if (token == "finalizing_index" || token == "index_complete")
        return QObject::tr("Finalizing the machine safety index");
    if (token == "saving_index")
        return QObject::tr("Saving the machine safety index");
    if (token == "auditing_safe")
        return QObject::tr("Auditing CertifiedSafe poses");
    if (token == "packaging")
        return QObject::tr("Packaging the machine model and safety index");
    if (token == "complete")
        return QObject::tr("Machine safety package generation completed");
    return QObject::tr("Building the machine safety package");
}
using lcnc::cam::detail::entityEntries;
using lcnc::cam::detail::faceBelongsToSource;
using lcnc::cam::detail::shapeCenter;
using lcnc::cam::detail::translatedShapeCopy;
using lcnc::cam::detail::watchTask;
} // namespace

void CamModule::configureMachine(const QString& presetName)
{
    LcncDocument* doc = machineDocument();
    if (!doc || presetName.isEmpty())
        return;

    MachineKinematics* kin = doc->machineKinematics();
    if (!kin)
        return;

    const bool sameConfig = (kin->configType() == presetName);
    invalidateMachineSafetyPackage();
    invalidateMachineEnvironment();
    kin->loadPreset(presetName);
    m_config.setMachinePreset(presetName);
    if (m_machineConfig)
        m_machineConfig->syncFromKinematics(kin);
    if (!activeMachineProfilePath().isEmpty())
        applyStoredMachineProfile(activeMachineProfilePath());

    if (!sameConfig)
        clearToolpath();

    displayAxisGuides();
    refreshMachineTransforms();
    emit axisAssignmentsChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
}

void CamModule::loadMachine(const QString& filePath)
{
    LcncDocument* doc = machineDocument();
    if (!doc || filePath.isEmpty()) {
        // 中文翻译：加载机台；机台工作区或模型路径不可用。
        emit operationFailed(tr("Loading machine"),
                             tr("The machine workspace or model path is unavailable."));
        return;
    }

    QFileInfo fi(filePath);
    if (!fi.exists() || !fi.isFile()) {
        // 中文翻译：加载机台；机台模型文件不存在。
        emit operationFailed(tr("Loading machine"),
                             tr("The machine model file does not exist."));
        return;
    }
    auto* taskManager = lcnc::Kernel::current().taskManager();
    if (!taskManager) {
        // 中文翻译：加载机台；后台任务服务不可用。
        emit operationFailed(tr("Loading machine"),
                             tr("The background task service is unavailable."));
        return;
    }

    const QString normalizedPath = fi.absoluteFilePath();
    const bool packageInput = lcnc::MachineSafetyPackage::isPackagePath(normalizedPath);
    const auto packageExtraction = packageInput
        ? std::make_shared<QTemporaryDir>() : std::shared_ptr<QTemporaryDir>{};
    if (packageExtraction && !packageExtraction->isValid()) {
        emit operationFailed(tr("Loading machine"),
                             tr("Unable to create the machine safety package extraction directory."));
        return;
    }
    const auto packageLoad = std::make_shared<lcnc::MachineSafetyPackageLoadResult>();
    const auto packageIndex = std::make_shared<lcnc::cam_algo::MachineSafetyIndex>();
    if (packageInput)
        m_machineSafetyPackageManager.beginLoad(normalizedPath);
    else
        m_machineSafetyPackageManager.clear();
    emit machineSafetyPackageChanged();
    const auto stageTimer = std::make_shared<QElapsedTimer>();
    stageTimer->start();
    LCNC_INFO(lcnc::LogCode::Generic,
              "stage=machine.load event=begin path='{}'",
              normalizedPath.toStdString());

    // The worker must not touch the live XCAF document.  The document is also
    // read by the renderer, selection, calibration and collision code on the
    // GUI thread.  Parse into detached TopoDS shapes first, then replace the
    // document in one GUI-thread commit after checking this generation token.
    const std::uint64_t loadGeneration = ++m_machineLoadGeneration;
    const auto result = std::make_shared<lcnc::cam::machine_io::MachineImportResult>();
    m_machineLoadPending.store(true);
    invalidateMachineEnvironment();

    // 中文翻译：加载机台: %1
    TaskId taskId = taskManager->run(tr("Loading machine: %1").arg(fi.fileName()),
        [normalizedPath, packageInput, packageExtraction, packageLoad,
         packageIndex, result](TaskProgress* prog) {
            if (prog->isAbortRequested())
                throw std::runtime_error("machine load cancelled");
            QString modelPath = normalizedPath;
            if (packageInput) {
                QString packageError;
                if (!lcnc::MachineSafetyPackage::extractAndValidate(
                        normalizedPath, packageExtraction->path(), packageLoad.get(),
                        &packageError)
                    || !packageIndex->load(packageLoad->safetyIndexPath, &packageError)) {
                    result->error = packageError;
                    throw std::runtime_error(packageError.toStdString());
                }
                modelPath = packageLoad->modelPath;
            }
            if (!lcnc::cam::machine_io::readMachineFile(modelPath, prog, result.get())) {
                throw std::runtime_error(result->error.isEmpty()
                    ? "machine model parse failed" : result->error.toStdString());
            }
            if (prog->isAbortRequested())
                throw std::runtime_error("machine load cancelled");
        });

    m_taskScope.track(taskId);
    watchTask(this, taskId, [this, taskId, normalizedPath, loadGeneration, result,
                             packageInput, packageExtraction, packageLoad,
                             packageIndex, stageTimer](bool ok) {
        bool committed = false;
        const auto stageLog = qScopeGuard([&] {
            LCNC_INFO(lcnc::LogCode::Generic,
                      "stage=machine.load event=end result={} generation={} parts={} elapsed_ms={} path='{}'",
                      committed ? "success" : "failed", loadGeneration,
                      result->parts.size(), stageTimer->elapsed(),
                      normalizedPath.toStdString());
        });
        m_taskScope.release(taskId);
        if (loadGeneration != m_machineLoadGeneration) {
            LCNC_INFO(lcnc::LogCode::Generic,
                      "cam.machine: discarded stale machine load generation {}",
                      loadGeneration);
            return;
        }
        m_machineLoadPending.store(false);
        if (!ok || !result->isValid()) {
            if (packageInput) {
                m_machineSafetyPackageManager.markInvalid(
                    result->error.isEmpty()
                        ? tr("Unable to read the machine safety package")
                        : result->error);
                emit machineSafetyPackageChanged();
            }
            LCNC_ERR(lcnc::LogCode::Generic,
                     "cam.machine: loading '{}' failed: {}",
                     normalizedPath.toStdString(), result->error.toStdString());
            // 中文翻译：加载机台；无法读取机台模型。
            emit operationFailed(tr("Loading machine"), result->error.isEmpty()
                ? tr("Unable to read the machine model.") : result->error);
            return;
        }

        LcncDocument* liveDocument = machineDocument();
        if (!liveDocument) {
            // 中文翻译：加载机台；机台工作区在提交模型前已关闭。
            emit operationFailed(tr("Loading machine"),
                                 tr("The machine workspace was closed before the model could be committed."));
            return;
        }

        // Commit is deliberately late: a failed/cancelled/newer load leaves
        // the previously displayed, collision-capable machine intact.
        liveDocument->clearEntityKind(LcncDocument::EntityKind::Machine);
        for (const auto& part : std::as_const(result->parts)) {
            liveDocument->addShapeEntity(part.shape, part.name,
                                         LcncDocument::EntityKind::Machine);
        }
        m_machineVisibilityInitialized = false;
        invalidateMachineEnvironment();

        m_machineModelPath = normalizedPath;
        m_loadedMachineModelPath = normalizedPath;
        if (m_machineWorkspace)
            m_machineWorkspace->setModelFilePath(normalizedPath);
        m_config.setMachineModelPath(m_machineModelPath);
        applyConfiguredMachineAxes(false);
        autoDetectAxes();
        applyStoredMachineProfile(activeMachineProfilePath());
        const QByteArray actualRuntimeFingerprint =
            machineSafetyConfigurationFingerprint();
        if (packageInput
            && packageIndex
            && packageIndex->bodies().size() == result->parts.size()
            && packageLoad->manifest.runtimeConfigurationSha256.size() == 32
            && packageLoad->manifest.runtimeConfigurationSha256
                == actualRuntimeFingerprint) {
            m_machineSafetyPackageManager.publish(
                *packageLoad, packageIndex, packageExtraction);
            const auto publishedStatus =
                m_machineSafetyPackageManager.status();
            if (!publishedStatus.executionEligible()) {
                setCollisionDetectionEnabled(false);
                emit operationWarning(
                    tr("Machine safety package"),
                    publishedStatus.reason.isEmpty()
                        ? tr("The machine safety package is incomplete and collision detection remains disabled.")
                        : publishedStatus.reason);
            }
        } else if (packageInput) {
            LCNC_WARN(lcnc::LogCode::Generic,
                      "Machine safety runtime fingerprint mismatch package='{}' expected={} actual={} parts={} assignments={}",
                      normalizedPath.toStdString(),
                      packageLoad->manifest.runtimeConfigurationSha256.toHex().toStdString(),
                      actualRuntimeFingerprint.toHex().toStdString(),
                      result->parts.size(),
                      liveDocument->machineKinematics()->shapeAssignments().size());
            m_machineSafetyPackageManager.markInvalid(
                tr("The current machine kinematics or part assignments do not match the machine safety package"));
            setCollisionDetectionEnabled(false);
            // 中文翻译：机台安全包；当前机台运动学或零件归属与安全包不一致，包内安全文件已失效。
            emit operationWarning(tr("Machine safety package"),
                tr("The current machine kinematics or part assignments do not match the machine safety package. The embedded safety index is disabled."));
        }
        refreshMachineDisplay();
        scheduleWorkpieceSafetyOverlayPreparation();
        emit machineSafetyPackageChanged();
        emit machineLoaded();
        committed = true;
    });
}

bool CamModule::buildOrUpdateMachineSafetyPackage(QString* errorMessage)
{
    if (m_machineSafetyPackageBuildTask != kInvalidTaskId) {
        if (errorMessage)
            *errorMessage = tr("A machine safety package is already being generated");
        return false;
    }
    const QString sourcePath = activeMachineProfilePath();
    if (sourcePath.isEmpty() || !QFileInfo(sourcePath).isFile()) {
        if (errorMessage)
            *errorMessage = tr("Load a machine model before generating its safety package");
        return false;
    }
    auto* tasks = lcnc::Kernel::current().taskManager();
    if (!tasks) {
        if (errorMessage)
            *errorMessage = tr("The background task service is unavailable");
        return false;
    }
    const QFileInfo sourceInfo(sourcePath);
    const QString outputPath = lcnc::MachineSafetyPackage::isPackagePath(sourcePath)
        ? sourceInfo.absoluteFilePath()
        : sourceInfo.absoluteDir().filePath(sourceInfo.completeBaseName()
                                            + QStringLiteral(".lmsp"));
    const QString program = QDir(QCoreApplication::applicationDirPath()).filePath(
#ifdef Q_OS_WIN
        QStringLiteral("lcnc_machine_safety_index.exe"));
#else
        QStringLiteral("lcnc_machine_safety_index"));
#endif
    if (!QFileInfo(program).isFile()) {
        if (errorMessage)
            *errorMessage = tr("The machine safety package generator is unavailable: %1")
                                .arg(program);
        return false;
    }
    const QString envelopeGenerator = modelEnvelopeGeneratorPath();
    auto* cadFacade = lcnc::Kernel::current().service<lcnc::ICadFacade>();
    if (!cadFacade || !QFileInfo(envelopeGenerator).isFile()) {
        if (errorMessage) {
            // 中文翻译：已授权的模型包络生成器不可用：%1
            *errorMessage = tr("The licensed model-envelope generator is unavailable: %1")
                                .arg(envelopeGenerator);
        }
        return false;
    }
    const QByteArray runtimeConfigurationSha256 =
        machineSafetyConfigurationFingerprint();
    if (runtimeConfigurationSha256.size() != 32) {
        if (errorMessage)
            *errorMessage = tr("The current machine safety configuration is incomplete");
        return false;
    }
    const auto currentPackage = m_machineSafetyPackageManager.status();
    const QByteArray sourceModelSha256 =
        lcnc::MachineSafetyPackage::isPackagePath(sourcePath)
            && currentPackage.packagePath == sourcePath
            && currentPackage.modelSha256.size() == 32
        ? currentPackage.modelSha256 : sha256File(sourcePath);
    if (sourceModelSha256.size() != 32) {
        if (errorMessage)
            *errorMessage = tr("Unable to fingerprint the machine model");
        return false;
    }
    // A rebuild is monitored by the main process, but collision detection is
    // an explicit opt-in again only after the resulting package is reloaded and
    // validated. The non-collision machining workflow remains available.
    // 中文翻译：重建期间关闭碰撞检测；新包重新加载并验证后由操作者再次显式启用。
    setCollisionDetectionEnabled(false);
    m_machineSafetyPackageManager.beginBuild(
        sourceModelSha256, runtimeConfigurationSha256);
    emit machineSafetyPackageChanged();

    struct BuildResult {
        QString error;
        QByteArray output;
    };
    const auto buildResult = std::make_shared<BuildResult>();
    const TaskId taskId = tasks->run(
        tr("Generate machine safety package: %1").arg(sourceInfo.fileName()),
        [sourcePath, outputPath, program, envelopeGenerator, cadFacade,
         runtimeConfigurationSha256, buildResult](TaskProgress* progress) {
            QTemporaryDir extractedSource;
            QString envelopeSourcePath = sourcePath;
            if (lcnc::MachineSafetyPackage::isPackagePath(sourcePath)) {
                lcnc::MachineSafetyPackageLoadResult existingPackage;
                if (!extractedSource.isValid()
                    || !lcnc::MachineSafetyPackage::extractAndValidate(
                        sourcePath, extractedSource.path(), &existingPackage,
                        &buildResult->error)) {
                    throw std::runtime_error(
                        "unable to extract the source machine safety package");
                }
                envelopeSourcePath = existingPackage.modelPath;
            }
            QTemporaryDir envelopeAssets;
            if (!envelopeAssets.isValid()) {
                // 中文翻译：无法创建模型包络暂存目录
                buildResult->error = QObject::tr(
                    "Unable to create model-envelope staging directory");
                throw std::runtime_error("unable to create model-envelope staging directory");
            }
            lcnc::CadModelEnvelopeRequest envelopeRequest;
            envelopeRequest.sourceModelPath = envelopeSourcePath;
            envelopeRequest.generatorPath = envelopeGenerator;
            envelopeRequest.assetRootDirectory = envelopeAssets.path();
            envelopeRequest.progressMinimum = 0;
            envelopeRequest.progressMaximum = 20;
            lcnc::CadModelEnvelopeResult envelopeResult;
            if (!cadFacade->runModelEnvelopeGenerator(
                    envelopeRequest, progress, &envelopeResult,
                    &buildResult->error)) {
                throw std::runtime_error(buildResult->error.isEmpty()
                    ? "model-envelope generation failed"
                    : buildResult->error.toStdString());
            }
            QProcess process;
            progress->setRange(0, 100);
            progress->setStepName(
                machineSafetyBuildStageText(QByteArrayLiteral("starting")));
            progress->setValue(0);
            process.setProgram(program);
            QStringList arguments{
                QStringLiteral("--machine"), sourcePath,
                QStringLiteral("--package-output"), outputPath,
                QStringLiteral("--runtime-configuration-sha256"),
                QString::fromLatin1(runtimeConfigurationSha256.toHex()),
                QStringLiteral("--envelope-manifest"),
                envelopeResult.manifestPath,
                QStringLiteral("--exact-budget"), QStringLiteral("1"),
                QStringLiteral("--audit-safe"), QStringLiteral("32"),
                // The production grid uses conservative envelope AABBs. The
                // persisted envelope Surface-BVH resolves Unknown at runtime;
                // a full 400k-cell Surface-BVH sweep remains an explicit offline
                // profile. Original STEP leaves cannot certify the envelope.
                // 中文翻译：生产网格使用包络 AABB 保守认证，持久包络 Surface-BVH 在运行时
                // 消解 Unknown；全网格 Surface-BVH 扫描保留为显式离线档。
                QStringLiteral("--surface-bvh"), QStringLiteral("off"),
                QStringLiteral("--leaf-bvh"), QStringLiteral("off"),
                QStringLiteral("--dependency-cache"), QStringLiteral("on"),
                QStringLiteral("--refine-threads"), QStringLiteral("8"),
                QStringLiteral("--checkpoint"),
                outputPath + QStringLiteral(".checkpoint.lmsi")};
            const QString hotFeedbackPath = outputPath
                + QStringLiteral(".hot_apos.txt");
            if (QFileInfo::exists(hotFeedbackPath)) {
                arguments.append({QStringLiteral("--hot-apos-file"),
                                  hotFeedbackPath});
            }
            arguments.append({QStringLiteral("--refine-levels"),
                              QFileInfo::exists(hotFeedbackPath)
                                  ? QStringLiteral("2")
                                  : QStringLiteral("1")});
            process.setArguments(arguments);
            process.start();
            if (!process.waitForStarted(10'000)) {
                buildResult->error = process.errorString();
                throw std::runtime_error("machine safety package generator did not start");
            }

            QByteArray pendingOutput;
            const auto consumeProcessOutput = [&] {
                const QByteArray standardOutput = process.readAllStandardOutput();
                if (!standardOutput.isEmpty()) {
                    buildResult->output.append(standardOutput);
                    pendingOutput.append(standardOutput);
                }
                while (true) {
                    const qsizetype newline = pendingOutput.indexOf('\n');
                    if (newline < 0)
                        break;
                    const QByteArray line = pendingOutput.left(newline).trimmed();
                    pendingOutput.remove(0, newline + 1);
                    if (!line.startsWith("LCNC_PROGRESS|"))
                        continue;
                    const QList<QByteArray> fields = line.split('|');
                    bool percentOk = false;
                    const int percent = fields.size() >= 2
                        ? fields.at(1).toInt(&percentOk) : 0;
                    if (!percentOk || fields.size() < 3)
                        continue;
                    progress->setStepName(
                        machineSafetyBuildStageText(fields.at(2)));
                    progress->setValue(20 + qBound(0, percent, 100) * 80 / 100);
                }
                const QByteArray standardError = process.readAllStandardError();
                if (!standardError.isEmpty())
                    buildResult->error.append(QString::fromUtf8(standardError));
            };
            while (!process.waitForFinished(250)) {
                consumeProcessOutput();
                if (progress->isAbortRequested()) {
                    process.kill();
                    process.waitForFinished(10'000);
                    consumeProcessOutput();
                    throw std::runtime_error("machine safety package generation cancelled");
                }
            }
            consumeProcessOutput();
            if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0)
                throw std::runtime_error(buildResult->error.isEmpty()
                    ? "machine safety package generator failed"
                    : buildResult->error.toStdString());
        });
    m_machineSafetyPackageBuildTask = taskId;
    m_taskScope.track(taskId);
    watchTask(this, taskId, [this, taskId, sourcePath, outputPath, buildResult](bool ok) {
        m_taskScope.release(taskId);
        m_machineSafetyPackageBuildTask = kInvalidTaskId;
        if (!ok || !QFileInfo(outputPath).isFile()) {
            m_machineSafetyPackageManager.finishBuildFailure(
                buildResult->error.isEmpty()
                    ? tr("Machine safety package generation failed")
                    : buildResult->error);
            emit machineSafetyPackageChanged();
            emit operationFailed(tr("Generate machine safety package"),
                buildResult->error.isEmpty()
                    ? tr("Machine safety package generation failed")
                    : buildResult->error);
            return;
        }
        m_config.copyMachineProfile(sourcePath, outputPath);
        setMachineModelPath(outputPath);
        loadMachine(outputPath);
    });
    return true;
}

void CamModule::unloadMachine()
{
    LcncDocument* doc = machineDocument();
    if (!doc) return;

    // Remove machine entities
    NCollection_Sequence<TDF_Label> labels = doc->entityLabels(LcncDocument::EntityKind::Machine);
    QStringList entries;
    for (int i = 1; i <= labels.Length(); ++i)
        entries << XcafUtils::entry(labels.Value(i));
    for (const QString& e : entries)
        ShapeService::deleteShape(doc, e);

    ++m_machineLoadGeneration;
    m_machineLoadPending.store(false);
    m_machineSafetyPackageManager.clear();
    invalidateMachineEnvironment();
    m_machineModelPath.clear();
    m_loadedMachineModelPath.clear();
    if (m_machineWorkspace)
        m_machineWorkspace->setModelFilePath(QString());
    m_config.setMachineModelPath(QString());
    refreshMachineDisplay();
    emit machineSafetyPackageChanged();
    emit machineUnloaded();
}

void CamModule::exportMachine(const QString& filePath)
{
    LcncDocument* machDoc = machineDocument();
    if (!machDoc || filePath.isEmpty()) {
        // 中文翻译：导出机台；机台工作区或导出路径不可用。
        emit operationFailed(tr("Export machine"),
                             tr("The machine workspace or export path is unavailable."));
        return;
    }

    if (!lcnc::cam::machine_io::exportMachineToFile(
            machDoc, machDoc->machineKinematics(), filePath)) {
        // 中文翻译：导出机台；写入机台模型失败。
        emit operationFailed(tr("Export machine"), tr("Failed to write the machine model."));
    }
}

bool CamModule::exportSimplifiedMachine(const QString& filePath,
                                        QString* errorMessage)
{
    if (m_modelEnvelopeExportTask != kInvalidTaskId) {
        if (errorMessage)
            // 中文翻译：模型包络导出任务已在运行
            *errorMessage = tr("A model-envelope export is already running");
        return false;
    }
    LcncDocument* document = machineDocument();
    if (!document || filePath.trimmed().isEmpty()) {
        if (errorMessage)
            // 中文翻译：请先加载机台模型，再导出包络
            *errorMessage = tr("Load a machine model before exporting its envelope");
        return false;
    }
    const QFileInfo outputInfo(filePath);
    const QString suffix = outputInfo.suffix().toLower();
    if (suffix != QStringLiteral("stp") && suffix != QStringLiteral("step")) {
        if (errorMessage)
            // 中文翻译：精简机台输出必须为 STEP 文件
            *errorMessage = tr("The simplified machine output must be a STEP file");
        return false;
    }
    MachineKinematics* machine = document->machineKinematics();
    const QSet<QString> supportedAxes{
        QStringLiteral("BASE"), QStringLiteral("X"), QStringLiteral("Y"),
        QStringLiteral("Z"), QStringLiteral("A"), QStringLiteral("B"),
        QStringLiteral("C")};
    const NCollection_Sequence<TDF_Label> labels =
        document->entityLabels(LcncDocument::EntityKind::Machine);
    for (int index = 1; index <= labels.Length(); ++index) {
        const QString entry = XcafUtils::entry(labels.Value(index));
        const QString axis = machine->axisForShape(entry).trimmed().toUpper();
        if (axis.isEmpty()) {
            if (errorMessage) {
                // 中文翻译：导出包络前必须为每个机台零件分配轴。未分配零件：%1
                *errorMessage = tr("Every machine part must be assigned to an axis before envelope export. Unassigned part: %1")
                                    .arg(XcafUtils::name(labels.Value(index)));
            }
            return false;
        }
        if (!supportedAxes.contains(axis)) {
            if (errorMessage)
                // 中文翻译：模型包络生成器不支持 %1 轴
                *errorMessage = tr("The model-envelope generator does not support axis %1").arg(axis);
            return false;
        }
    }
    auto* tasks = lcnc::Kernel::current().taskManager();
    auto* cadFacade = lcnc::Kernel::current().service<lcnc::ICadFacade>();
    const QString generator = modelEnvelopeGeneratorPath();
    if (!tasks || !cadFacade || !QFileInfo(generator).isFile()) {
        if (errorMessage) {
            // 中文翻译：已授权的模型包络生成器不可用：%1
            *errorMessage = tr("The licensed model-envelope generator is unavailable: %1")
                                .arg(generator);
        }
        return false;
    }
    const auto staging = std::make_shared<QTemporaryDir>();
    if (!staging->isValid()) {
        if (errorMessage)
            // 中文翻译：无法创建模型包络暂存目录
            *errorMessage = tr("Unable to create model-envelope staging directory");
        return false;
    }
    const QString normalizedOutput = outputInfo.absoluteFilePath();
    const QString stagedSource = staging->filePath(QStringLiteral("marked_machine.step"));
    if (!lcnc::cam::machine_io::exportMachineToFile(document, machine, stagedSource)) {
        if (errorMessage)
            // 中文翻译：无法暂存已标轴机台模型以生成包络
            *errorMessage = tr("Unable to stage the marked machine model for envelope generation");
        return false;
    }

    struct ExportResult {
        QString error;
        QString outputPath;
    };
    const auto result = std::make_shared<ExportResult>();
    const TaskId taskId = tasks->run(
        // 中文翻译：生成模型包络：%1
        tr("Generate model envelope: %1").arg(outputInfo.fileName()),
        [staging, stagedSource, normalizedOutput, generator, cadFacade,
         result](TaskProgress* progress) {
            lcnc::CadModelEnvelopeRequest request;
            request.sourceModelPath = stagedSource;
            request.generatorPath = generator;
            request.assetRootDirectory = staging->filePath(QStringLiteral("asset"));
            request.simplifiedStepOutputPath = normalizedOutput;
            // Z and rotary-axis bodies carry the most recognizable machine
            // silhouette. Preserve smaller exterior features there while the
            // coarse profile remains available for collision-only packages.
            request.detailAxes = {QStringLiteral("Z"), QStringLiteral("A"),
                                  QStringLiteral("B"), QStringLiteral("C")};
            request.detailAlphaMm = 3.0;
            request.detailOffsetMm = 0.5;
            lcnc::CadModelEnvelopeResult generated;
            if (!cadFacade->runModelEnvelopeGenerator(
                    request, progress, &generated, &result->error)) {
                throw std::runtime_error(result->error.isEmpty()
                    ? "model-envelope export failed"
                    : result->error.toStdString());
            }
            result->outputPath = generated.simplifiedStepOutputPath;
        });
    m_modelEnvelopeExportTask = taskId;
    m_taskScope.track(taskId);
    emit modelEnvelopeExportStateChanged();
    watchTask(this, taskId, [this, taskId, normalizedOutput, result](bool ok) {
        m_taskScope.release(taskId);
        m_modelEnvelopeExportTask = kInvalidTaskId;
        emit modelEnvelopeExportStateChanged();
        if (!ok || result->outputPath.isEmpty()
            || !QFileInfo(normalizedOutput).isFile()) {
            emit operationFailed(
                // 中文翻译：导出精简机台
                tr("Export simplified machine"),
                result->error.isEmpty()
                    // 中文翻译：模型包络导出失败
                    ? tr("Model-envelope export failed") : result->error);
            return;
        }
        emit modelEnvelopeExported(normalizedOutput);
    });
    return true;
}

void CamModule::autoDetectAxes()
{
    LcncDocument* doc = machineDocument();
    if (!doc) return;

    lcnc::cam::machine_axis_detector::autoDetectAxisNames(doc, doc->machineKinematics());
    applyStoredMachineProfile(activeMachineProfilePath());
    if (m_machineConfig)
        m_machineConfig->syncFromKinematics(doc->machineKinematics());
    if (auto* gd = activeGuiDocument())
        gd->applyMachineDisplayStyle();
    refreshMachineTransforms();
    emit axisAssignmentsChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
}

void CamModule::applyAxisAssignments(const QMap<QString, QString>& entryToAxis)
{
    LcncDocument* doc = machineDocument();
    if (!doc || entryToAxis.isEmpty())
        return;

    MachineKinematics* kin = doc->machineKinematics();
    for (auto it = entryToAxis.cbegin(); it != entryToAxis.cend(); ++it) {
        if (it.value().isEmpty())
            kin->unassignShape(it.key());
        else
            kin->assignShape(it.key(), it.value());
    }

    invalidateMachineSafetyPackage();
    invalidateMachineEnvironment();
    if (auto* gd = activeGuiDocument())
        gd->applyMachineDisplayStyle();
    refreshMachineTransforms();
    emit axisAssignmentsChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
}

void CamModule::assignShapesToAxis(const QStringList& entries, const QString& axisName)
{
    if (entries.isEmpty() || axisName.isEmpty())
        return;

    QMap<QString, QString> entryToAxis;
    for (const QString& entry : entries)
        entryToAxis.insert(entry, axisName);
    applyAxisAssignments(entryToAxis);
}

void CamModule::unassignShape(const QString& entry)
{
    if (entry.isEmpty())
        return;

    QMap<QString, QString> entryToAxis;
    entryToAxis.insert(entry, QString());
    applyAxisAssignments(entryToAxis);
}

QList<CamModule::AxisOption> CamModule::axisOptions(bool includeDetachOption) const
{
    QList<AxisOption> result;
    MachineKinematics* kin = kinematics();
    if (!kin)
        return result;

    if (includeDetachOption)
        // 中文翻译：— 解除已有挂载 —
        result.append({QString(), tr("— Uninstall existing mounts —")});

    for (const MachineAxisDef& axis : kin->axes()) {
        QString displayName;
        if (axis.name == QStringLiteral("BASE")) {
            // 中文翻译：BASE（固定基座）
            displayName = tr("BASE (fixed base)");
        } else if (axis.motionType == MachineAxisDef::Rotary) {
            // 中文翻译：%1 轴（旋转）
            displayName = tr("%1 axis (rotation)").arg(axis.name);
        } else {
            // 中文翻译：%1 轴（线性）
            displayName = tr("%1 axis (linear)").arg(axis.name);
        }
        result.append({axis.name, displayName});
    }

    return result;
}

QString CamModule::defaultWorkpieceMountAxis() const
{
    const MachineKinematics* kin = kinematics();
    if (!kin || kin->axes().isEmpty())
        return QString();

    const auto hasAxis = [kin](const QString& axisName) {
        return kin->findAxis(axisName) != nullptr;
    };

    const QString configType = kin->configType();
    if (configType == QStringLiteral("VERTICAL_AC_TABLE")
        || configType == QStringLiteral("VERTICAL_BC_TABLE")) {
        if (hasAxis(QStringLiteral("C")))
            return QStringLiteral("C");
    } else if (configType == QStringLiteral("XYZA")) {
        if (hasAxis(QStringLiteral("A")))
            return QStringLiteral("A");
    }

    if (hasAxis(QStringLiteral("BASE")))
        return QStringLiteral("BASE");

    return kin->axes().isEmpty() ? QString() : kin->axes().first().name;
}

gp_Pnt CamModule::axisOrigin(const QString& axisName) const
{
    if (MachineKinematics* kin = kinematics())
        return kin->axisOrigin(axisName);
    return gp_Pnt(0, 0, 0);
}

void CamModule::setAxisOrigin(const QString& axisName, const gp_Pnt& origin)
{
    MachineKinematics* kin = kinematics();
    if (!kin)
        return;

    const gp_Pnt current = kin->axisOrigin(axisName);
    if (current.SquareDistance(origin) < 1e-12)
        return;

    if (!kin->setAxisOrigin(axisName, origin))
        return;

    // Runtime calibration works in OCC world coordinates. Persist the result
    // through the single machine configuration authority, which converts it
    // back to controller-axis coordinates (for example, world Z=-300 becomes
    // taught Z=+300 when the configured controller Z direction is downward).
    if (m_machineConfig)
        m_machineConfig->syncFromKinematics(kin);

    displayAxisGuides();
    refreshMachineTransforms();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
}

bool CamModule::setAxisLimits(const QString& axisName, double minVal, double maxVal)
{
    MachineKinematics* kin = kinematics();
    if (!kin)
        return false;

    if (!kin->setAxisLimits(axisName, minVal, maxVal))
        return false;

    displayAxisGuides();
    refreshMachineTransforms();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
    return true;
}

bool CamModule::currentAcRotationCenter(gp_Pnt& center) const
{
    if (!ensureAcCenterCalibrationAvailable())
        return false;

    const MachineKinematics* kin = kinematics();
    if (!kin)
        return false;

    const MachineAxisDef* tiltAxis = nullptr;
    const MachineAxisDef* spinAxis = nullptr;
    for (const MachineAxisDef& axis : kin->axes()) {
        if (axis.role == lcnc::MachineAxisRole::TableTilt)
            tiltAxis = &axis;
        else if (axis.role == lcnc::MachineAxisRole::TableSpin)
            spinAxis = &axis;
    }
    double axisSeparation = 0.0;
    return tiltAxis && spinAxis
        && lcnc::kinematics::tableRotationCenterFromReferences(
            *tiltAxis, tiltAxis->origin, *spinAxis, spinAxis->origin,
            center, &axisSeparation)
        && axisSeparation <= 0.1;
}

bool CamModule::currentAcRotationCenterAxisCoordinates(gp_Pnt& center) const
{
    gp_Pnt worldCenter;
    if (!currentAcRotationCenter(worldCenter) || !m_machineConfig)
        return false;
    return m_machineConfig->worldToAxisCoordinates(worldCenter, &center);
}

bool CamModule::currentWorkpieceRotationCenter(gp_Pnt& center) const
{
    const MachineKinematics* kin = kinematics();
    if (!kin)
        return false;

    bool hasTableTilt = false;
    bool hasTableSpin = false;
    for (const MachineAxisDef& axis : kin->axes()) {
        hasTableTilt = hasTableTilt || axis.role == lcnc::MachineAxisRole::TableTilt;
        hasTableSpin = hasTableSpin || axis.role == lcnc::MachineAxisRole::TableSpin;
    }
    if (hasTableTilt && hasTableSpin)
        return currentAcRotationCenter(center);

    if (kin->configType() == QStringLiteral("XYZA")) {
        if (!kin->findAxis(QStringLiteral("A")))
            return false;

        center = kin->axisOrigin(QStringLiteral("A"));
        return true;
    }

    return false;
}

gp_Pnt CamModule::cutterHeadModelPosition() const
{
    return m_cutterHeadModelPosition;
}

gp_Pnt CamModule::cutterHeadPhysicalPosition() const
{
    return m_cutterHeadPhysicalPosition;
}

bool CamModule::pickReferenceFaceCenter(WidgetOccView* occView,
                                        const QPoint& screenPos,
                                        gp_Pnt& center,
                                        QString* errorMessage) const
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CamModule::pickReferenceFaceCenter begin");
    const bool ok = resolveReferencePlaneCenter(occView, screenPos, center, errorMessage);
    LCNC_DEBUG(lcnc::LogCode::Generic,
               "CamModule::pickReferenceFaceCenter end ok={}", ok);
    return ok;
}

bool CamModule::enterStandardCalibrationPose(const AxisCalibrationInputs& inputs,
                                             QString* errorMessage)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CamModule::enterStandardCalibrationPose begin");

    auto fail = [&](const QString& msg) {
        if (errorMessage)
            *errorMessage = msg;
        LCNC_ERR(lcnc::LogCode::Generic,
                 "CamModule::enterStandardCalibrationPose failed: {}",
                 msg.toStdString());
        // 中文翻译：绝对标定目标
        emit operationFailed(tr("Absolute calibration target"), msg);
        return false;
    };

    QString reason;
    if (!ensureAcCenterCalibrationAvailable(&reason))
        return fail(reason);

    MachineKinematics* kin = kinematics();
    if (!kin)
        // 中文翻译：找不到机台轴系配置。
        return fail(tr("The machine axis system configuration cannot be found."));
    for (const MachineAxisDef& axis : kin->axes()) {
        if ((axis.role == lcnc::MachineAxisRole::TableTilt
             || axis.role == lcnc::MachineAxisRole::TableSpin)
            && std::abs(axis.currentPos) > 1e-6) {
            // 中文翻译：绝对几何标定要求实际转台旋转轴先回到 0；向导不会替机台修改实时轴坐标。
            return fail(tr("Absolute geometry calibration requires the physical rotary-table axes to be at 0 first. The wizard will not change live axis coordinates."));
        }
    }

    try {
        gp_Pnt configuredCenter;
        if (!currentAcRotationCenter(configuredCenter))
            // 中文翻译：请先在应用程序选项的机台构型页配置转台旋转轴原点。
            return fail(tr("Please configure the rotary-table axis origins on the machine configuration page of the application options first."));

        LCNC_INFO(lcnc::LogCode::Generic,
                  "Calibration absolute targets: configuredCenter=({:.3f},{:.3f},{:.3f}) "
                  "tcp=({:.3f},{:.3f},{:.3f}) pickedHead=({:.3f},{:.3f},{:.3f})",
                  configuredCenter.X(), configuredCenter.Y(), configuredCenter.Z(),
                  cutterHeadWorldPosition().X(), cutterHeadWorldPosition().Y(),
                  cutterHeadWorldPosition().Z(), inputs.cutterHeadFaceCenter.X(),
                  inputs.cutterHeadFaceCenter.Y(), inputs.cutterHeadFaceCenter.Z());
    } catch (const Standard_Failure& f) {
        // 中文翻译：OCC 异常：%1
        return fail(tr("OCC exception: %1").arg(QString::fromUtf8(f.what())));
    } catch (const std::exception& e) {
        // 中文翻译：异常：%1
        return fail(tr("Exception: %1").arg(QString::fromUtf8(e.what())));
    } catch (...) {
        // 中文翻译：发生未知异常。
        return fail(tr("An unknown exception occurred."));
    }

    return true;
}

gp_Pnt CamModule::cutterHeadWorldPosition() const
{
    const MachineKinematics* kin = kinematics();
    if (!kin)
        return gp_Pnt(0.0, 0.0, 0.0);
    return kin->currentLinearPosition();
}

gp_Pnt CamModule::cutterHeadAxisPosition() const
{
    const gp_Pnt world = cutterHeadWorldPosition();
    gp_Pnt axis;
    return m_machineConfig
            && m_machineConfig->worldToAxisCoordinates(world, &axis)
        ? axis : world;
}

bool CamModule::applyAxisCalibration(const AxisCalibrationInputs& inputs,
                                     QString* errorMessage)
{
    LCNC_DEBUG(lcnc::LogCode::Generic, "CamModule::applyAxisCalibration begin");

    auto fail = [&](const QString& msg) {
        if (errorMessage)
            *errorMessage = msg;
        LCNC_ERR(lcnc::LogCode::Generic,
                 "CamModule::applyAxisCalibration failed: {}",
                 msg.toStdString());
        // 中文翻译：机台坐标系标定
        emit operationFailed(tr("Machine coordinate system calibration"), msg);
        return false;
    };

    // Validate only. Absolute targets must never move the live controller pose.
    QString reason;
    if (!enterStandardCalibrationPose(inputs, &reason)) {
        // enterStandardCalibrationPose() has already reported this validation
        // failure. Avoid emitting a duplicate operationFailed signal here.
        if (errorMessage)
            *errorMessage = reason;
        LCNC_ERR(lcnc::LogCode::Generic,
                 "CamModule::applyAxisCalibration rejected before geometry update: {}",
                 reason.toStdString());
        return false;
    }

    try {
        gp_Pnt configuredCenter;
        if (!currentAcRotationCenter(configuredCenter))
            // 中文翻译：请先在应用程序选项的机台构型页配置转台旋转轴原点。
            return fail(tr("Please configure the rotary-table axis origins on the machine configuration page of the application options first."));

        LcncDocument* doc = machineDocument();
        MachineKinematics* kin = kinematics();
        if (!doc || !kin)
            // 中文翻译：找不到机台项目文档。
            return fail(tr("The machine project document cannot be found."));

        const gp_Pnt absoluteTcp = cutterHeadWorldPosition();
        const gp_Pnt cutterHeadSnapshot = m_cutterHeadModelPosition;
        const NCollection_Sequence<TDF_Label> machineLabels =
            doc->entityLabels(LcncDocument::EntityKind::Machine);
        struct GeometryMove {
            TDF_Label label;
            gp_Vec translation;
        };
        std::vector<GeometryMove> completedMoves;
        auto rollback = [&] {
            for (auto it = completedMoves.rbegin(); it != completedMoves.rend(); ++it) {
                const gp_Vec reverse(-it->translation.X(), -it->translation.Y(),
                                     -it->translation.Z());
                ShapeService::moveShape(doc, it->label, reverse);
            }
            completedMoves.clear();
            m_cutterHeadModelPosition = cutterHeadSnapshot;
        };
        bool geometryCommitted = false;
        auto rollbackGuard = qScopeGuard([&] {
            if (!geometryCommitted)
                rollback();
        });
        Q_UNUSED(rollbackGuard);
        auto moveLabel = [&](const TDF_Label& label, const gp_Vec& translation) {
            if (translation.SquareMagnitude() < 1e-12)
                return true;
            if (!ShapeService::moveShape(doc, label, translation))
                return false;
            completedMoves.push_back({label, translation});
            return true;
        };

        lcnc::kinematics::TableCalibrationPlan calibrationPlan;
        lcnc::kinematics::CalibrationPlanError planError =
            lcnc::kinematics::CalibrationPlanError::None;
        if (!lcnc::kinematics::buildTableCalibrationPlan(
                *kin, inputs.tiltFaceCenter, inputs.spinFaceCenter,
                inputs.cutterHeadFaceCenter, configuredCenter, absoluteTcp,
                calibrationPlan, &planError)) {
            if (planError == lcnc::kinematics::CalibrationPlanError::ParallelRotaryAxes) {
                // 中文翻译：TableTilt 与 TableSpin 旋转轴平行，无法定义唯一的转台中心。
                return fail(tr("The TableTilt and TableSpin axes are parallel and do not define a unique rotary-table center."));
            }
            if (planError == lcnc::kinematics::CalibrationPlanError::SkewRotaryReferences) {
                // 中文翻译：拾取的两条旋转轴线不相交，请检查参考面和轴方向配置。
                return fail(tr("The two picked rotary-axis lines do not intersect. Check the reference faces and configured axis directions."));
            }
            if (planError == lcnc::kinematics::CalibrationPlanError::UnreachableToolCorrection) {
                // 中文翻译：所需切割头校正包含刀头承载链无法产生的方向，请检查模型装配和父轴链路。
                return fail(tr("The required cutter correction contains a direction that the configured tool-carrier chain cannot produce. Check the model assembly and parent-axis chain."));
            }
            // 中文翻译：切割头承载链不完整或其直线轴运动退化，无法执行标定。
            return fail(tr("The tool-carrier chain is incomplete or its linear-axis motion is degenerate, so calibration cannot continue."));
        }

        // Stage 1: derive the model center from the configured rotary-axis
        // lines, then translate the complete machine to the physical center.
        const gp_Vec& acTranslation = calibrationPlan.wholeMachineTranslation;
        for (int index = 1; index <= machineLabels.Length(); ++index) {
            const TDF_Label label = machineLabels.Value(index);
            if (!label.IsNull() && !moveLabel(label, acTranslation)) {
                rollback();
                // 中文翻译：转台中心对齐失败，机台模型已恢复标定前位置。
                return fail(tr("Rotary-table center alignment failed and the machine model has been restored to its pre-calibration position."));
            }
        }
        gp_Pnt alignedCutterFaceWorld = inputs.cutterHeadFaceCenter.Translated(
            acTranslation);

        // Stage 2: propagate corrections only through linear axes on the real
        // tool-carrier chain. A workpiece-side linear axis is never counted as
        // cutter motion.
        QMap<QString, int> directPartCounts;
        QMap<QString, int> affectedPartCounts;
        for (int index = 1; index <= machineLabels.Length(); ++index) {
            const TDF_Label label = machineLabels.Value(index);
            if (label.IsNull())
                continue;
            const QString entry = XcafUtils::entry(label);
            const QString assignedAxis = kin->axisForShape(entry);
            directPartCounts[assignedAxis.toUpper()] += 1;
            for (const auto& correction : calibrationPlan.carrierCorrections) {
                if (!kin->isAxisDescendantOf(assignedAxis, correction.axisName))
                    continue;
                if (correction.translation.SquareMagnitude() >= 1e-12)
                    affectedPartCounts[correction.axisName] += 1;
            }
            const gp_Vec partTranslation =
                lcnc::kinematics::inheritedCarrierCorrection(
                    *kin, assignedAxis, calibrationPlan.carrierCorrections);
            if (!moveLabel(label, partTranslation)) {
                rollback();
                // 中文翻译：切割头 XYZ 对齐失败，机台模型已恢复标定前位置。
                return fail(tr("Cutter-head XYZ alignment failed and the machine model has been restored to its pre-calibration position."));
            }
        }
        for (const auto& correction : calibrationPlan.carrierCorrections) {
            if (correction.translation.SquareMagnitude() < 1e-12)
                continue;
            LCNC_INFO(lcnc::LogCode::Generic,
                      "Calibration tool-carrier axis={} correction=({:.3f},{:.3f},{:.3f}) "
                      "directParts={} affectedParts={}",
                      correction.axisName.toStdString(), correction.translation.X(),
                      correction.translation.Y(), correction.translation.Z(),
                      directPartCounts.value(correction.axisName),
                      affectedPartCounts.value(correction.axisName));
            if (directPartCounts.value(correction.axisName) == 0) {
                rollback();
                // 中文翻译：标定需要移动刀头承载链中的 %1 轴，但没有机台模型直接归属于该轴。请先标记该轴模型。
                return fail(tr("Calibration requires moving the %1-axis on the tool-carrier chain, but no machine model is directly assigned to that axis. Please mark that axis model first.")
                                .arg(correction.axisName));
            }
            if (affectedPartCounts.value(correction.axisName) == 0) {
                rollback();
                // 中文翻译：标定需要移动 %1 轴，但未找到归属于该轴子树的机台部件。请先完成轴归属标记。
                return fail(tr("Calibration requires moving the %1-axis, but no machine parts assigned to that axis subtree were found. Please complete the axis assignments first.")
                                .arg(correction.axisName));
            }
        }
        alignedCutterFaceWorld.Translate(calibrationPlan.cutterTranslation);
        // Persist the cutter reference in its carrier-local frame. Picks are
        // world-space points with the live local transformation already
        // applied, so storing the world point directly would double-apply the
        // current XYZ feedback in collision/simulation consumers.
        m_cutterHeadModelPosition = alignedCutterFaceWorld.Transformed(
            kin->computeAxisTransform(calibrationPlan.toolCarrierAxisName).Inverted());

        if (!activeMachineProfilePath().isEmpty())
            m_config.setCutterHeadModelPositionForMachine(activeMachineProfilePath(),
                                                          m_cutterHeadModelPosition);

        invalidateMachineSafetyPackage();
        invalidateMachineEnvironment();

        // 自动 STEP 回写：对齐后的模型作为下次启动的初始模型。
        if (!activeMachineProfilePath().isEmpty()) {
            LCNC_INFO(lcnc::LogCode::Generic,
                      "Saving aligned machine model back to: {}",
                      activeMachineProfilePath().toStdString());
            if (doc && !lcnc::cam::machine_io::exportMachineToFile(
                            doc, doc->machineKinematics(), activeMachineProfilePath())) {
                LCNC_WARN(lcnc::LogCode::Generic,
                          "Auto-save of machine model failed: {}",
                          activeMachineProfilePath().toStdString());
                // 仅警告，不中断标定流程
            }
        }
        geometryCommitted = true;
    } catch (const Standard_Failure& f) {
        // 中文翻译：OCC 异常：%1
        return fail(tr("OCC exception: %1").arg(QString::fromUtf8(f.what())));
    } catch (const std::exception& e) {
        // 中文翻译：异常：%1
        return fail(tr("Exception: %1").arg(QString::fromUtf8(e.what())));
    } catch (...) {
        // 中文翻译：发生未知异常。
        return fail(tr("An unknown exception occurred."));
    }

    displayAxisGuides();
    refreshMachineDisplay();

    LCNC_INFO(lcnc::LogCode::Generic,
              "CamModule::applyAxisCalibration done (absolute AC center and TCP XYZ kept fixed; machine geometry aligned)");
    emit machineWorkspaceChanged();
    return true;
}

QList<CamModule::WorkpieceMountCandidate> CamModule::mountableWorkpieces() const
{
    QList<WorkpieceMountCandidate> result;
    LcncDocument* doc = lcnc::Kernel::current().projectManager()->workpieceDocument();
    if (!doc)
        return result;

    const int workpieceCount = doc->entityLabels(LcncDocument::EntityKind::Workpiece).Length();
    if (workpieceCount <= 0)
        return result;

    const QString stateName = lcnc::Kernel::current().projectManager()->session().workpiece().displayName.trimmed();
    // 中文翻译：当前工件
    const QString displayName = stateName.isEmpty() ? tr("current workpiece") : stateName;

    // 中文翻译：%1  (%2 形体)
    result.append({doc->id(), tr("%1 (%2 shape)").arg(displayName).arg(workpieceCount), workpieceCount});

    return result;
}

bool CamModule::autoInstallWorkpiece() const
{
    return m_config.autoInstallWorkpiece();
}

void CamModule::setAutoInstallWorkpiece(bool enabled)
{
    m_config.setAutoInstallWorkpiece(enabled);
    autoInstallCurrentWorkpieceInternal(enabled);
    emit machineWorkspaceChanged();
}

bool CamModule::autoInstallCurrentWorkpiece()
{
    return autoInstallCurrentWorkpieceInternal(m_config.autoInstallWorkpiece());
}

QStringList CamModule::sourceWorkpieceEntriesForMountedEntries(const QStringList& mountedEntries) const
{
    QStringList result;
    GuiDocument* gd = activeGuiDocument();
    LcncDocument* doc = workpieceDocument();
    if (!gd || !doc)
        return result;

    const QStringList selectedSourceEntries = gd->selectedEntries(doc->id());
    if (selectedSourceEntries.isEmpty())
        return result;

    for (const QString& mountedEntry : mountedEntries) {
        if (!mountedEntry.isEmpty()
            && selectedSourceEntries.contains(mountedEntry)
            && !result.contains(mountedEntry))
            result.append(mountedEntry);
    }
    return result;
}

bool CamModule::alignWorkpieceSetupToRotationCenter()
{
    gp_Pnt center;
    if (!currentWorkpieceRotationCenter(center)) {
        // 中文翻译：工件安装姿态；当前构型没有可用于对齐的工件旋转中心。
        emit operationFailed(tr("Workpiece setup"), tr("There is no workpiece rotation center available for alignment in the current configuration."));
        return false;
    }

    lcnc::WorkpieceSetupTransform setup = workpieceSetupTransform();
    gp_Pnt axisCenter;
    if (!m_machineConfig
        || !m_machineConfig->worldToAxisCoordinates(center, &axisCenter)) {
        // 中文翻译：工件安装姿态；无法将转台中心转换为轴系坐标。
        emit operationFailed(tr("Workpiece setup"),
                             tr("The rotary-table center cannot be converted to controller-axis coordinates."));
        return false;
    }
    setup.x = axisCenter.X();
    setup.y = axisCenter.Y();
    setup.z = axisCenter.Z();
    return setWorkpieceSetupTransform(setup);
}

void CamModule::autoDetectAxisOrigins()
{
    MachineKinematics* kin = kinematics();
    lcnc::cam::machine_axis_detector::autoDetectAxisOrigins(machineDocument(), kin);
    if (kin && m_machineConfig)
        m_machineConfig->syncFromKinematics(kin);
}

void CamModule::applyStoredMachineProfile(const QString& machinePath)
{
    MachineKinematics* kin = kinematics();
    if (!kin || machinePath.isEmpty())
        return;

    const auto profile = lcnc::cam::machine_axis_detector::applyStoredMachineProfile(
        kin, m_config, machinePath);

    if (profile.hasCutterHeadModel)
        m_cutterHeadModelPosition = profile.cutterHeadModelPosition;
    if (profile.hasCutterHeadPhysical)
        m_cutterHeadPhysicalPosition = profile.cutterHeadPhysicalPosition;
    if (profile.hasWorkpieceInstall && m_machineConfig
        && m_machineConfig->workpieceSetupTransform().isIdentity()) {
        lcnc::WorkpieceSetupTransform migrated;
        migrated.x = profile.workpieceInstallPosition.X();
        migrated.y = profile.workpieceInstallPosition.Y();
        migrated.z = profile.workpieceInstallPosition.Z();
        m_machineConfig->setWorkpieceSetupTransform(migrated);
        if (!m_machineConfig->saveDefault()) {
            LCNC_WARN(lcnc::LogCode::Generic,
                      "cam.machine: failed to persist migrated legacy workpiece installation setup");
        }
        kin->setWorkpieceSetupTransform(
            m_machineConfig->workpieceSetupTransform().toTransform());
        LCNC_INFO(lcnc::LogCode::Generic,
                  "cam.machine: migrated legacy workpiece installation XYZ into unified workpiece setup");
    }
    if (profile.hasWorkpieceInstall)
        m_config.clearLegacyWorkpieceInstallPositionForMachine(machinePath);

}

bool CamModule::applyConfiguredMachineAxes(bool updateView)
{
    MachineKinematics* kin = kinematics();
    if (!kin || !m_machineConfig)
        return false;

    QList<MachineAxisDef> axes = m_machineConfig->axisDefinitions();
    if (axes.isEmpty())
        return false;

    // MachineConfigurationService owns the static topology and limits, while
    // the existing kinematics instance mirrors live controller feedback.  A
    // workspace/file switch must replace only the former; copying the raw
    // configured currentPos values would fabricate a home pose until the next
    // device polling update arrives.
    // 中文翻译：切换工程只更新轴拓扑和限位；保留已有运动学中的实时反馈，不能把配置默认零位当作控制器反馈。
    for (MachineAxisDef& axis : axes) {
        if (axis.name == QStringLiteral("BASE"))
            continue;
        if (const MachineAxisDef* liveAxis = kin->findAxis(axis.name))
            axis.currentPos = liveAxis->currentPos;
    }

    kin->setAxes(axes, m_machineConfig->presetName());
    kin->setWorkpieceSetupTransform(m_machineConfig->workpieceSetupTransform().toTransform());
    if (m_camData && !m_camData->machineAxisLayout().isValid()) {
        const lcnc::MachiningMode mode = m_machineConfig->defaultMachiningMode();
        const lcnc::MachineModeDefinition definition = m_machineConfig->modeDefinition(mode);
        m_camData->setMachiningMode(mode);
        m_camData->setMachineAxisLayout(definition.interpolatedAxes);
        m_camData->setSolverId(definition.solverId);
        m_camData->setSolverVersion(definition.solverVersion);
        m_camData->setSolvedMachineConfigurationFingerprint(QString());
    }
    m_config.setMachinePreset(m_machineConfig->presetName());
    // setAxes() updates this kinematics object in-place, so refresh the pose
    // even when the pointer is unchanged; otherwise newly configured C/B axes
    // are rejected and the workpiece cannot follow the rotary table.
    if (m_pose)
        m_pose->setKinematics(kin);
    if (!updateView)
        return true;

    if (m_camData && hasToolpath()) {
        // A machine/configuration change invalidates only the solved machine
        // coordinate stage.  Re-solving remains an explicit CAM operation so
        // stale coordinates cannot silently become executable.
        // 中文翻译：机床或安装姿态变化只使机床坐标阶段失效；重新求解必须由 CAM 显式执行，旧坐标不得静默变为可加工状态。
        m_camData->setSolvedMachineConfigurationFingerprint(QString());
        m_camData->invalidatePipelineAfter(
            lcnc::cam::CamPipelineStage::GeometricToolpath,
            QStringLiteral("Machine configuration changed; machine coordinates must be solved again"));
        for (LaserContour& contour : toolpathRef().contours()) {
            for (ToolpathPoint& point : contour.points)
                point.machineCoord = {};
            if (contour.leadInSolution.valid)
                contour.leadInSolution.point.machineCoord = {};
        }
        m_camData->markDirty(true);
        if (m_travelPathRenderer)
            m_travelPathRenderer->erase(activeGuiDocument());
        emit pipelineStageChanged(lcnc::cam::CamPipelineStage::MachineSolve);
        lcnc::Kernel::current().projectManager()->notifyDomainChanged(
            lcnc::ProjectDomain::Cam);
    }

    displayAxisGuides();
    if (hasToolpath())
        refreshToolpathDisplay();
    refreshMachineTransforms();
    emit axisAssignmentsChanged();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
    return true;
}

// ── Workpiece Installation ───────────────────────────────────────────────────

void CamModule::mountWorkpiece(DocumentId sourceDocId, const QString& axisName, bool alignToInstallPosition)
{
    auto* project = lcnc::Kernel::current().projectManager();
    LcncDocument* srcDoc = project->domainDocumentById(sourceDocId);
    if (!srcDoc || !project->isDomainDocument(sourceDocId, lcnc::ProjectDomain::Workpiece))
        return;

    MachineKinematics* kin = kinematics();
    const QString targetAxis = axisName.trimmed().isEmpty()
        ? defaultWorkpieceMountAxis()
        : axisName.trimmed();

    NCollection_Sequence<TDF_Label> sourceLabels =
        srcDoc->entityLabels(LcncDocument::EntityKind::Workpiece);
    if (sourceLabels.Length() == 0)
        return;

    Q_UNUSED(alignToInstallPosition);
    bool mountChanged = false;
    QString firstEntry;
    QMap<QString, QString> mountedEntries;
    for (int i = 1; i <= sourceLabels.Length(); ++i) {
        const QString entry = XcafUtils::entry(sourceLabels.Value(i));
        if (entry.isEmpty())
            continue;
        if (firstEntry.isEmpty())
            firstEntry = entry;
        mountedEntries.insert(entry, entry);
        if (!kin)
            continue;

        const QString currentAxis = kin->mountedAxis(entry);
        if (currentAxis == targetAxis)
            continue;

        if (targetAxis.isEmpty())
            kin->unmountWorkpiece(entry);
        else
            kin->mountWorkpiece(entry, targetAxis);
        mountChanged = true;
    }

    if (!mountChanged)
        return;
    m_mountedWorkpieceEntryBySourceEntry = mountedEntries;
    refreshMachineTransforms();
    emit machineWorkspaceChanged();
    emit workpieceMounted(firstEntry);
}

bool CamModule::autoInstallCurrentWorkpieceInternal(bool alignToInstallPosition)
{
    if (!alignToInstallPosition)
        return false;

    auto* project = lcnc::Kernel::current().projectManager();
    const DocumentId sourceDocId = project->workpieceDocumentId();
    LcncDocument* sourceDoc = project->domainDocumentById(sourceDocId);
    if (!sourceDoc || sourceDoc->entityLabels(LcncDocument::EntityKind::Workpiece).Length() == 0) {
        clearMountedWorkpieceDisplay(false);
        return false;
    }

    mountWorkpiece(sourceDocId, QString(), true);
    return true;
}

bool CamModule::clearMountedWorkpieceDisplay(bool refreshView)
{
    LcncDocument* doc = machineDocument();
    if (!doc)
        return false;

    MachineKinematics* kin = doc->machineKinematics();
    const QStringList workpieceEntries = entityEntries(doc, LcncDocument::EntityKind::Workpiece);
    const bool hadEntries = !workpieceEntries.isEmpty() || !m_mountedWorkpieceEntryBySourceEntry.isEmpty();
    for (const QString& entry : workpieceEntries) {
        if (kin)
            kin->unmountWorkpiece(entry);
    }
    if (kin) {
        for (const QString& entry : m_mountedWorkpieceEntryBySourceEntry.keys())
            kin->unmountWorkpiece(entry);
        if (LcncDocument* srcDoc = workpieceDocument()) {
            const NCollection_Sequence<TDF_Label> sourceLabels =
                srcDoc->entityLabels(LcncDocument::EntityKind::Workpiece);
            for (int i = 1; i <= sourceLabels.Length(); ++i)
                kin->unmountWorkpiece(XcafUtils::entry(sourceLabels.Value(i)));
        }
    }
    doc->clearEntityKind(LcncDocument::EntityKind::Workpiece);
    m_mountedWorkpieceEntryBySourceEntry.clear();

    if (!hadEntries)
        return false;

    if (refreshView)
        refreshMachineDisplay();
    emit workpieceUnmounted();
    emit machineWorkspaceChanged();
    return true;
}

bool CamModule::moveShape(const QString& entry, const gp_Vec& translation)
{
    LcncDocument* doc = machineDocument();
    if (!doc || entry.isEmpty()) return false;

    NCollection_Sequence<TDF_Label> labels = doc->entityLabels(LcncDocument::EntityKind::Machine);
    for (int i = 1; i <= labels.Length(); ++i) {
        if (XcafUtils::entry(labels.Value(i)) == entry) {
            bool ok = ShapeService::moveShape(doc, labels.Value(i), translation);
            if (ok) {
                invalidateMachineSafetyPackage();
                invalidateMachineEnvironment();
                refreshMachineDisplay();
            }
            return ok;
        }
    }

    NCollection_Sequence<TDF_Label> wpcLabels =
        doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    for (int i = 1; i <= wpcLabels.Length(); ++i) {
        if (XcafUtils::entry(wpcLabels.Value(i)) == entry) {
            bool ok = ShapeService::moveShape(doc, wpcLabels.Value(i), translation);
            if (ok) {
                invalidateMachineEnvironment();
                refreshMachineDisplay();
            }
            return ok;
        }
    }
    return false;
}

bool CamModule::rotateShape(const QString& entry, const gp_Ax1& axis, double angleDeg)
{
    LcncDocument* doc = machineDocument();
    if (!doc || entry.isEmpty()) return false;

    NCollection_Sequence<TDF_Label> labels = doc->entityLabels(LcncDocument::EntityKind::Machine);
    for (int i = 1; i <= labels.Length(); ++i) {
        if (XcafUtils::entry(labels.Value(i)) == entry) {
            bool ok = ShapeService::rotateShape(doc, labels.Value(i), axis, angleDeg);
            if (ok) {
                invalidateMachineSafetyPackage();
                invalidateMachineEnvironment();
                refreshMachineDisplay();
            }
            return ok;
        }
    }

    NCollection_Sequence<TDF_Label> wpcLabels =
        doc->entityLabels(LcncDocument::EntityKind::Workpiece);
    for (int i = 1; i <= wpcLabels.Length(); ++i) {
        if (XcafUtils::entry(wpcLabels.Value(i)) == entry) {
            bool ok = ShapeService::rotateShape(doc, wpcLabels.Value(i), axis, angleDeg);
            if (ok) {
                invalidateMachineEnvironment();
                refreshMachineDisplay();
            }
            return ok;
        }
    }
    return false;
}

void CamModule::deleteShape(const QString& entry)
{
    LcncDocument* doc = machineDocument();
    if (!doc || entry.isEmpty()) return;

    bool deletingMachineShape = false;
    const NCollection_Sequence<TDF_Label> machineLabels =
        doc->entityLabels(LcncDocument::EntityKind::Machine);
    for (int index = 1; index <= machineLabels.Length(); ++index) {
        if (XcafUtils::entry(machineLabels.Value(index)) == entry) {
            deletingMachineShape = true;
            break;
        }
    }

    if (auto* gd = activeGuiDocument())
        gd->eraseEntity(doc->id(), entry);

    ShapeService::deleteShape(doc, entry);
    if (deletingMachineShape)
        invalidateMachineSafetyPackage();
    invalidateMachineEnvironment();
    refreshMachineTransforms();
    lcnc::Kernel::current().projectManager()->notifyDomainChanged(lcnc::ProjectDomain::Machine);
}

// ── Toolpath ──────────────────────────────────────────────────────────────────
