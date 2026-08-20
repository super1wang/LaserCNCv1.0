#include "modules/process/cutting/normal_cutting_manager.h"

#include "core/logging/logger.h"
#include "core/kernel/kernel.h"
#include "modules/cam/contracts/i_cam_initial_approach_planner.h"
#include "modules/cam/contracts/i_cam_toolpath_provider.h"
#include "modules/process/runtime/process_device_runtime.h"
#include "modules/process/tool/tool.h"
#include "modules/process/tool/tool_factory.h"
#include "modules/process/cutting/process_cutting_plan_service.h"
#include "modules/process/cutting/pure_simulation_toolpath_ticker.h"
#include "modules/process/device/motion_control/motion_control.h"
#include "modules/process/runtime/i_motion_command_sink.h"
#include "modules/process/runtime/device_command_queue.h"
#include "modules/process/runtime/machine_pose5.h"
#include "modules/process/runtime/process_interrupt_context.h"
#include "modules/process/runtime/rapid_motion_utilities.h"
#include "modules/process/settings/process_settings_service.h"
#include "modules/process/runtime/process_cutting_safety.h"

#include <QString>
#include <QStringList>
#include <QElapsedTimer>
#include <QScopeGuard>
#include <QVariantMap>
#include <QThread>

#include <algorithm>
#include <cmath>

namespace lcnc::process {

namespace {

constexpr char kStartNumber[]       = "startNumber";
constexpr char kEndNumber[]         = "endNumber";
constexpr char kCompensationIndex[] = "compensationIndex";

constexpr int kTokenPollEvery     = 64;   // sink.lineTo 期间每多少段轮询一次 token

constexpr char kEnvContourIndex[] = "contourIndex";
constexpr char kEnvContourTotal[] = "contourTotal";
constexpr char kEnvBackend[]      = "backend";
constexpr char kEnvPhase[]        = "phase";

QString makeLabel(const QString& phase, int idx, int total)
{
    return QStringLiteral("normalCutting/%1/%2-of-%3").arg(phase).arg(idx).arg(total);
}

MachinePose5 toPose5(const lcnc::cam::ToolpathExportPoint& p,
                      double ox, double oy)
{
    MachinePose5 pose;
    // machineAxes follows the configurable physical layout (which may be
    // Y/X/Z/A/C).  Motion sinks consume this fixed semantic representation.
    pose.x  = p.machineX + ox;
    pose.y  = p.machineY + oy;
    pose.z  = p.machineZ;
    pose.r1 = p.machineR1;
    pose.r2 = p.machineR2;
    pose.r1Name = p.rotaryAxis1Name;
    pose.r2Name = p.rotaryAxis2Name;
    pose.mask = MachinePose5::Bx | MachinePose5::By | MachinePose5::Bz;
    if (!p.rotaryAxis1Name.isEmpty()) pose.mask |= MachinePose5::Br1;
    if (!p.rotaryAxis2Name.isEmpty()) pose.mask |= MachinePose5::Br2;
    return pose;
}

bool sameCacheDouble(double a, double b)
{
    return std::abs(a - b) <= 1e-9;
}

} // namespace

NormalCuttingManager::NormalCuttingManager(ProcessDeviceRuntime* service,
                                           std::shared_ptr<lcnc::cam::ICamToolpathProvider> toolpathProvider,
                                           NormalCuttingCallbacks callbacks,
                                           DeviceCommandQueue* deviceQueue,
                                           ProcessSettingsService* settings,
                                           QObject* parent)
    : QObject(parent)
    , m_service(service)
    , m_toolpathProvider(std::move(toolpathProvider))
    , m_callbacks(std::move(callbacks))
    , m_deviceQueue(deviceQueue)
    , m_settings(settings)
    , m_toolpathService(std::make_unique<ProcessToolpathService>(m_toolpathProvider))
    , m_simTicker(std::make_unique<PureSimulationToolpathTicker>(
          m_callbacks.motionSink.positionObserver))
{
    // 兜底 Tool —— 当 ToolFactory 找不到匹配工具时 resolveTool() 返回这个。
    // 由 Fix #1 (Tool 类内默认初始化) 保证非赋值字段不再是 0xCD…。这里仅显式覆盖几个最关键的。
    m_sanitizedDefaultTool.m_strName        = "__fallback__";
    m_sanitizedDefaultTool.m_dLineVelocity  = 600.0;
}

NormalCuttingManager::~NormalCuttingManager() = default;

void NormalCuttingManager::setCuttingPlanService(ProcessCuttingPlanService* service)
{
    if (m_planChangedConnection)
        QObject::disconnect(m_planChangedConnection);

    m_planService = service;
    clearCuttingListCache();

    if (m_planService) {
        m_planChangedConnection = QObject::connect(
            m_planService,
            &ProcessCuttingPlanService::planChanged,
            this,
            [this]() { clearCuttingListCache(); });
    } else {
        m_planChangedConnection = {};
    }
}

bool NormalCuttingManager::run(const QString& nodeId,
                                const QVariantMap& parameters,
                                ProcessInterruptContext* interrupt,
                                QString* errorMessage)
{
    ProcessInterruptContext localFallback;
    ProcessInterruptContext& ic = interrupt ? *interrupt : localFallback;

    const int startNumber   = parameters.value(QString::fromLatin1(kStartNumber), 1).toInt();
    const int endNumber     = parameters.value(QString::fromLatin1(kEndNumber), 0).toInt();
    const QString compIndex = parameters.value(QString::fromLatin1(kCompensationIndex)).toString();

    double compOffsetX = 0.0;
    double compOffsetY = 0.0;
    if (!compIndex.isEmpty()) {
        LCNC_INFO(lcnc::LogCode::Generic,
                  "normal-cutting: compensation index '{}' specified, but CompDevice not wired yet",
                  compIndex.toStdString());
    }

    if (!m_toolpathProvider) {
        // 中文翻译：CAM 刀路提供者未注册
        if (errorMessage) *errorMessage = tr("CAM tool path provider is not registered");
        return false;
    }

    QVector<CuttingRow> cuttingList;
    lcnc::cam::ToolpathExportSnapshot executionSnapshot;
    const CuttingListCacheKey directCacheKey{
        m_toolpathProvider->toolpathRevision(),
        m_planService ? m_planService->planRevision() : 0ull,
        startNumber,
        endNumber,
        compOffsetX,
        compOffsetY
    };
    if (cacheKeyMatches(directCacheKey)) {
        LCNC_INFO(lcnc::LogCode::Generic,
                  "normal-cutting: using cached planned cutting list contours={} snapshotRev={} planRev={}",
                  m_cuttingListCacheRows.size(),
                  directCacheKey.snapshotRevision,
                  directCacheKey.planRevision);
        cuttingList = cachedCuttingListCopy();
        // A catalog refresh for preflight/UI may have occurred since the
        // rows were cached.  Always reacquire the committed CAM execution
        // snapshot; never execute whichever snapshot happened to be read last.
        executionSnapshot = m_toolpathService->refreshCommittedExecutionSnapshot();
    } else {
        // CAM alone commits the ordered/solved execution snapshot.  The
        // Process range filter is applied only to its derived execution rows
        // below; it must never request a sliced or reordered CAM path.
        executionSnapshot = m_toolpathService->refreshCommittedExecutionSnapshot();
        if (!executionSnapshot.hasEnabledContours()) {
            // 中文翻译：CAM 中没有可执行的启用轮廓
            if (errorMessage) *errorMessage = tr("There is no executable enable profile in CAM");
            return false;
        }
        cuttingList = buildCuttingList(executionSnapshot, startNumber, endNumber,
                                       compOffsetX, compOffsetY, errorMessage);
    }

    QString layoutError;
    if (!executionSnapshot.machineAxisLayout.isValid(&layoutError)) {
        // 中文翻译：刀路快照轴布局无效：%1
        if (errorMessage) *errorMessage = tr("The toolpath snapshot axis layout is invalid: %1").arg(layoutError);
        return false;
    }

    if (cuttingList.isEmpty()) {
        if (errorMessage && errorMessage->isEmpty())
            // 中文翻译：筛选后的切割链表为空
            *errorMessage = tr("The filtered cutting list is empty");
        return false;
    }

    // 选 sink —— 硬件 sink 必须在设备执行线程创建、使用和销毁。
    const bool simMode = m_callbacks.simulationModeProvider
        && m_callbacks.simulationModeProvider();
    const MotionSinkCallbacks& sinkCallbacks = m_callbacks.motionSink;
    std::shared_ptr<IMotionCommandSink> sink;
    QString backendLabel;
    if (simMode) {
        auto created = m_service
            ? m_service->createMotionSink(true, m_simTicker.get(), sinkCallbacks,
                                          executionSnapshot.machineAxisLayout)
            : nullptr;
        sink = std::shared_ptr<IMotionCommandSink>(std::move(created));
        if (sink) {
            sink->setCancellation(&ic);
            backendLabel = sink->id();
        }
    } else if (m_deviceQueue && m_service) {
        const DeviceCommandResult creation = m_deviceQueue->executeAndWait(
            DeviceCommandQueue::ResultCommand([this, &sink, &backendLabel, &ic,
                                               sinkCallbacks,
                                               layout = executionSnapshot.machineAxisLayout] {
                auto created = m_service->createMotionSink(
                    false, m_simTicker.get(), sinkCallbacks, layout);
                if (!created) {
                    return DeviceCommandResult{
                        false,
                        QObject::tr("Motion instruction set construction failed (configuration/controller mismatch)")};
                }
                created->setCancellation(&ic);
                backendLabel = created->id();
                DeviceCommandQueue* const queue = m_deviceQueue;
                sink = std::shared_ptr<IMotionCommandSink>(
                    created.release(),
                    [queue](IMotionCommandSink* pointer) {
                        if (!pointer)
                            return;
                        if (queue && queue->isWorkerThread()) {
                            delete pointer;
                            return;
                        }
                        if (queue) {
                            const auto result = queue->executeAndWait(
                                DeviceCommandQueue::ResultCommand([pointer] {
                                    delete pointer;
                                    return DeviceCommandResult{};
                                }),
                                TaskPriority::Stop,
                                5000);
                            if (result.completion != DeviceCommandCompletion::Shutdown
                                && result.completion != DeviceCommandCompletion::Cancelled) {
                                return;
                            }
                        }
                        // An SDK-backed sink must not be destroyed on the wrong
                        // thread after shutdown; retain it for operator recovery.
                        LCNC_ERR(lcnc::LogCode::Generic,
                                 "NormalCuttingManager retained a device sink because executor teardown rejected destruction");
                    });
                return DeviceCommandResult{};
            }),
            TaskPriority::Workflow,
            5000);
        if (!creation.success && errorMessage)
            *errorMessage = creation.error;
    }
    if (!sink) {
        // 中文翻译：运动指令汇构造失败（构型/控制器不匹配）
        if (errorMessage && errorMessage->isEmpty())
            *errorMessage = tr("Motion instruction set construction failed (configuration/controller mismatch)");
        return false;
    }

    // 通知外层暂停普通的仿真轴驱动，避免与刀路回放写入竞争。
    if (m_callbacks.normalCuttingActivityObserver)
        m_callbacks.normalCuttingActivityObserver(true);
    const auto restoreAxisDriver = qScopeGuard([this]() {
        if (m_callbacks.normalCuttingActivityObserver)
            m_callbacks.normalCuttingActivityObserver(false);
    });

    // 断点续跑。
    int startContourIndex = 0;
    if (ic.hasResumePoint(nodeId)) {
        const auto rp = ic.resumePoint(nodeId);
        const int last        = rp.env.value(QString::fromLatin1(kEnvContourIndex), -1).toInt();
        const int oldTotal    = rp.env.value(QString::fromLatin1(kEnvContourTotal), -1).toInt();
        const QString oldBack = rp.env.value(QString::fromLatin1(kEnvBackend)).toString();
        if (last >= 0 && oldTotal == cuttingList.size() && oldBack == backendLabel) {
            const QString phase = rp.env.value(QString::fromLatin1(kEnvPhase)).toString();
            startContourIndex = (phase == QLatin1String("afterContour")) ? (last + 1) : last;
            startContourIndex = std::clamp(startContourIndex, 0, static_cast<int>(cuttingList.size()));
            // 中文翻译：从断点续跑：跳过前 %1 条轮廓
            emit logMessage(tr("Resume from breakpoint: skip first %1 contours").arg(startContourIndex));
        } else {
            ic.clearResumePoint(nodeId);
        }
    }

    // 中文翻译：普通切割开始：%1 条轮廓（起始 %2），后端=%3
    emit logMessage(tr("Normal cutting start: %1 contours (start %2), backend=%3")
                        .arg(cuttingList.size())
                        .arg(startContourIndex + 1)
                        .arg(backendLabel));

    const int total = cuttingList.size();
    for (int i = startContourIndex; i < total; ++i) {
        QVariantMap env;
        env.insert(QString::fromLatin1(kEnvContourIndex), i);
        env.insert(QString::fromLatin1(kEnvContourTotal), total);
        env.insert(QString::fromLatin1(kEnvBackend), backendLabel);
        env.insert(QString::fromLatin1(kEnvPhase), QStringLiteral("beforeContour"));
        if (!ic.checkpoint(nodeId, makeLabel(QStringLiteral("beforeContour"), i, total), env)) {
            // 中文翻译：普通切割已被中断
            if (errorMessage) *errorMessage = tr("Normal cutting has been interrupted");
            return false;
        }

        const CuttingRow& row = cuttingList[i];
        emit contourStarted(i + 1, total,
                            // 中文翻译：轮廓 #%1 (%2)
                            tr("Outline #%1 (%2)")
                                .arg(row.data.contour.contourId)
                                .arg(row.data.contour.contourName));

        bool ok = false;
        try {
            ok = executeContour(sink, simMode, row, ic, nodeId, i, total, errorMessage);
        } catch (const std::bad_optional_access& ex) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "normal-cutting: bad_optional_access on contour {} ({}), tool='{}'",
                     row.data.contour.contourId, ex.what(),
                     row.tool ? row.tool->m_strName : std::string("<null>"));
            if (errorMessage)
                // 中文翻译：轮廓 %1 执行异常：工具方向字段无效
                *errorMessage = tr("Contour %1 execution exception: Tool direction field is invalid").arg(row.data.contour.contourId);
            return false;
        } catch (const std::exception& ex) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "normal-cutting: exception on contour {}: {}",
                     row.data.contour.contourId, ex.what());
            if (errorMessage)
                // 中文翻译：轮廓 %1 执行异常：%2
                *errorMessage = tr("Contour %1 execution exception: %2")
                                    .arg(row.data.contour.contourId)
                                    .arg(QString::fromUtf8(ex.what()));
            return false;
        }

        if (!ok) {
            if (ic.isStopping())
                return false;
            if (errorMessage && errorMessage->isEmpty())
                // 中文翻译：轮廓 %1 执行中断
                *errorMessage = tr("Execution of contour %1 interrupted").arg(row.data.contour.contourId);
            return false;
        }

        env[QString::fromLatin1(kEnvPhase)] = QStringLiteral("afterContour");
        if (!ic.checkpoint(nodeId, makeLabel(QStringLiteral("afterContour"), i, total), env)) {
            // 中文翻译：普通切割已被中断
            if (errorMessage) *errorMessage = tr("Normal cutting has been interrupted");
            return false;
        }

        emit contourFinished(i + 1, total);
    }

    ic.clearResumePoint(nodeId);
    // 中文翻译：普通切割完成
    emit logMessage(tr("Ordinary cutting completed"));
    return true;
}

bool NormalCuttingManager::prepareInitialApproach(
    const CuttingRow& row,
    ProcessInterruptContext& interrupt,
    lcnc::cam::InitialApproachSnapshot* approach,
    QString* errorMessage)
{
    if (!approach) {
        if (errorMessage)
            *errorMessage = tr("CAM initial approach planner or execution snapshot is unavailable");
        return false;
    }

    auto planner = lcnc::Kernel::current().services()
        .getService<lcnc::cam::ICamInitialApproachPlanner>();
    const auto execution = m_toolpathProvider
        ? m_toolpathProvider->exportCommittedExecutionSnapshot()
        : lcnc::cam::ToolpathExportSnapshot{};
    if (!planner || !m_service || !m_deviceQueue
        || execution.revision == 0 || execution.machineConfigurationFingerprint.isEmpty()) {
        // 中文翻译：CAM 首段规划器或执行快照不可用
        if (errorMessage)
            *errorMessage = tr("CAM initial approach planner or execution snapshot is unavailable");
        return false;
    }

    const QStringList axisNames = execution.machineAxisLayout.axisNames();
    const auto readApos = [this, &axisNames](QMap<QString, double>* positions,
                                             QString* error) {
        const auto captured = std::make_shared<QMap<QString, double>>();
        const DeviceCommandResult read = m_deviceQueue->executeAndWait(
            DeviceCommandQueue::ResultCommand([this, axisNames, captured] {
                return m_service->readAxisPositions(axisNames, captured.get());
            }), TaskPriority::Workflow, 5000);
        if (read.success && positions)
            *positions = *captured;
        if (!read.success && error)
            *error = read.error;
        return read.success;
    };
    const auto poseDrifted = [&execution](const QMap<QString, double>& before,
                                          const QMap<QString, double>& after) {
        for (int index = 0; index < execution.machineAxisLayout.count; ++index) {
            const auto& axis = execution.machineAxisLayout.axes[index];
            const double tolerance = (axis.role == lcnc::MachineAxisRole::LinearX
                || axis.role == lcnc::MachineAxisRole::LinearY
                || axis.role == lcnc::MachineAxisRole::LinearZ) ? 0.01 : 0.01;
            if (!before.contains(axis.name) || !after.contains(axis.name)
                || std::abs(before.value(axis.name) - after.value(axis.name)) > tolerance) {
                return true;
            }
        }
        return false;
    };

    const ProcessInitialApproachSettings initialSettings = m_settings
        ? m_settings->initialApproachSettings()
        : ProcessInitialApproachSettings{};
    for (int attempt = 0; attempt < 2; ++attempt) {
        if (interrupt.isStopping()) {
            if (errorMessage)
                *errorMessage = tr("Normal cutting has been interrupted");
            return false;
        }
        QMap<QString, double> before;
        QString approachError;
        if (!readApos(&before, &approachError)) {
            if (errorMessage)
                *errorMessage = approachError;
            return false;
        }
        QStringList aposFields;
        for (auto it = before.cbegin(); it != before.cend(); ++it)
            aposFields.append(QStringLiteral("%1=%2").arg(it.key()).arg(it.value(), 0, 'f', 4));
        LCNC_INFO(lcnc::LogCode::Generic,
                  "stage=process.initial_approach.apos event=captured attempt={} positions='{}'",
                  attempt + 1, aposFields.join(QStringLiteral(",")).toStdString());

        lcnc::cam::InitialApproachRequest request;
        request.toolpathRevision = execution.revision;
        request.targetContourId = row.data.contour.contourId;
        request.machineConfigurationFingerprint = execution.machineConfigurationFingerprint;
        request.axisPositions = before;
        request.planningMode = initialSettings.mode == ProcessInitialApproachMode::Manual
            ? lcnc::cam::InitialApproachPlanningMode::Manual
            : lcnc::cam::InitialApproachPlanningMode::Automatic;
        request.safetyAxisZ = initialSettings.safetyZ;
        request.collisionCheckEnabled = initialSettings.collisionCheckEnabled;

        QElapsedTimer planningElapsed;
        planningElapsed.start();
        *approach = planner->planInitialApproach(request, &interrupt.stopRequested);
        LCNC_INFO(lcnc::LogCode::Generic,
                  "stage=process.initial_approach.plan event=end result={} mode={} segments={} elapsed_ms={}",
                  approach->isExecutable(approach->collision.blockWarning)
                      ? "success" : "failed",
                  initialSettings.mode == ProcessInitialApproachMode::Manual
                      ? "manual" : "automatic",
                  approach->transition.segments.size(), planningElapsed.elapsed());
        if (!approach->isExecutable(approach->collision.blockWarning)) {
            // 中文翻译：CAM 首段未通过碰撞校验
            if (errorMessage) {
                *errorMessage = approach->failureReason.isEmpty()
                    ? tr("CAM initial approach is not collision-verified")
                    : approach->failureReason;
            }
            return false;
        }

        QMap<QString, double> after;
        if (!readApos(&after, &approachError)) {
            if (errorMessage)
                *errorMessage = approachError;
            return false;
        }
        if (!poseDrifted(before, after))
            return true;
        if (attempt == 1) {
            // 中文翻译：首段规划期间控制器位置发生变化
            if (errorMessage)
                *errorMessage = tr("Controller position changed while planning the initial approach");
            return false;
        }
    }
    return false;
}

bool NormalCuttingManager::executeContour(const std::shared_ptr<IMotionCommandSink>& sink,
                                           bool pureSimulation,
                                           const CuttingRow& row,
                                           ProcessInterruptContext& interrupt,
                                           const QString& nodeId,
                                           int contourIndex,
                                           int total,
                                           QString* errorMessage)
{
    bool skippedEmptyContour = false;
    const bool needsInitialApproach = !row.hasEntryTransition;
    QElapsedTimer initialApproachElapsed;
    bool initialApproachSucceeded = false;
    const auto initialApproachLog = qScopeGuard([&] {
        if (!needsInitialApproach)
            return;
        LCNC_INFO(lcnc::LogCode::Generic,
                  "stage=process.initial_approach event=end result={} contour={} elapsed_ms={}",
                  initialApproachSucceeded ? "success" : "failed",
                  row.data.contour.contourId,
                  initialApproachElapsed.elapsed());
    });
    lcnc::cam::InitialApproachSnapshot initialApproach;
    if (needsInitialApproach) {
        initialApproachElapsed.start();
        LCNC_INFO(lcnc::LogCode::Generic,
                  "stage=process.initial_approach event=begin contour={} pure_simulation={}",
                  row.data.contour.contourId, pureSimulation);
        if (!prepareInitialApproach(row, interrupt, &initialApproach, errorMessage))
            return false;
    }

    auto constructAndStart = [&](IMotionCommandSink& commandSink, QString* startError) {
        // Construct and start one complete contour on the device executor.
        if (!row.tool) {
        // 中文翻译：轮廓 %1 没有绑定工具
            if (startError)
                *startError = tr("Profile %1 has no binding tools").arg(row.data.contour.contourId);
            return false;
        }

    // 运行中轴使能可能在启动预检之后被人为撤销或被驱动器切断。控制器的
    // 指令构建层会跳过失能轴，若这里不阻断，就会把该轮廓视为完成并继续
    // 下发下一轮廓。每次下发前直接读取硬件状态，将其作为不可恢复的步骤失败。
    if (!pureSimulation) {
        const DeviceCommandResult health = m_service
            ? m_service->validateContourBoundary()
            : DeviceCommandResult{false, tr("The motion controller is not connected during processing")};
        if (!health.success) {
            if (startError)
                *startError = health.error;
            return false;
        }
    }

    const Tool& tool = *row.tool;
    const auto& pts = row.data.points;
    if (pts.size() < 2) {
        skippedEmptyContour = true;
        return true;  // 空轮廓静默跳过
    }

    const double ox = row.compensationOffsetX;
    const double oy = row.compensationOffsetY;
    const auto& p0 = pts.front();
    const auto& lead = row.data.contour.leadInPoint;
    const MachinePose5 leadPose = toPose5(lead, ox, oy);
    const MachinePose5 contourStartPose = toPose5(p0, ox, oy);

    // ——— 程序起始（与遗留 buildContourACS / executeContourGTN 等价的语义序列）———
    commandSink.resetProgram();
    commandSink.applyToolMotionParams(tool, /*jump=*/true);
    QString commandError;

    Tool rapidTool = tool;
    rapidTool.m_dLineVelocity = tool.m_dIdleXVelocity > 0
        ? tool.m_dIdleXVelocity : 10.0;
    rapidTool.m_dLineAcc = tool.m_dIdleXYAccDec > 0
        ? tool.m_dIdleXYAccDec : tool.m_dLineAcc;
    rapidTool.m_dLineJerk = tool.m_dIdleXYJerk > 0
        ? tool.m_dIdleXYJerk : tool.m_dLineJerk;

    const auto submitCoordinatedRapid = [&](const QVector<MachinePose5>& poses) {
        if (poses.isEmpty())
            return true;
        if (!commandSink.beginSegment(poses.front(), rapidTool, &commandError))
            return false;
        for (const MachinePose5& pose : poses) {
            if (!commandSink.lineTo(pose, rapidTool, &commandError))
                return false;
        }
        commandSink.endSegment(rapidTool);
        if (!commandSink.flush(&commandError))
            return false;
        commandSink.resetProgram();
        commandSink.applyToolMotionParams(tool, /*jump=*/false);
        return true;
    };

    if (row.hasEntryTransition) {
        // The continuously solved CAM curve is the sole authority for
        // inter-contour rapid motion.  Do not rebuild a fixed XY/AC jump here:
        // it would break pose continuity and can reintroduce a rotary sweep.
        commandSink.stopCuttingHead();
        const int rapidCount = row.entryTransition.segments.size();
        QVector<MachinePose5> rapidPoses;
        rapidPoses.reserve(rapidCount);
        for (int rapidIndex = 0; rapidIndex < rapidCount; ++rapidIndex) {
            // CAM has already applied the configured rapid offset to geometry
            // and solved the complete five-axis sequence. Never alter one
            // semantic axis here: doing so leaves the certified TCP curve and
            // can drive the head through the workpiece.
            // 中文翻译：空程偏置和连续五轴求解已由 CAM 完成，Process 必须原样执行。
            const lcnc::cam::RapidMoveSegment& rapid =
                row.entryTransition.segments.at(rapidIndex);
            if (pureSimulation) {
                if (!commandSink.executeRapidSegment(rapid, tool, &commandError)) {
                    if (startError) *startError = commandError.isEmpty()
                        ? tr("Planned rapid command generation failed") : commandError;
                    return false;
                }
            } else {
                rapidPoses.append(solvedRapidPose(rapid));
            }
        }
        // A rapid must finish before the head/laser/cutting program is built.
        // Keeping them in one ACS buffer made the temporal boundary implicit
        // and allowed legacy PTP-looking behaviour around START 6.  Submit
        // the continuously solved XSEG/LINE path now, wait for it, then build
        // a fresh cutting program from the reached lead-in pose.
        if (!pureSimulation && !submitCoordinatedRapid(rapidPoses)) {
            if (startError) *startError = commandError.isEmpty()
                ? tr("Planned rapid execution failed") : commandError;
            return false;
        }
        if (tool.m_bCuttingHead)
            commandSink.startCuttingHead(tool);
    } else {
        // There is no predecessor contour for the first row of every new run,
        // including a range run after Stop. Submit measured APOS to CAM and
        // execute only its immutable entry plan; Process must never reuse the
        // previous run or rebuild a Z-up/XY/Z-down motion locally.
        // 中文翻译：每次全新加工（含停止后的范围加工）的首轮廓都没有前序轮廓，
        // 必须由 CAM 根据控制器 APOS 重新生成首段，不能复用上次运行的过渡段。
        if (tool.m_bCuttingHead && tool.m_bCrossBridge)
            commandSink.stopCuttingHead();
        const lcnc::cam::InitialApproachSnapshot& approach = initialApproach;
        if (pureSimulation) {
            for (const auto& segment : approach.transition.segments) {
                if (!commandSink.executeRapidSegment(segment, tool, &commandError)) {
                    // 中文翻译：CAM 首段执行失败
                    if (startError) *startError = commandError.isEmpty()
                        ? tr("Initial CAM approach execution failed") : commandError;
                    return false;
                }
            }
        } else {
            // Preserve CAM's safe-zone phase boundaries. Retract/approach are
            // issued as real controller absolute-Z commands and verified at
            // the requested signed coordinate. SafeXY and SafeAC remain
            // coordinated axis-space lines, submitted as separate programs so
            // the controller cannot blend across the safe-zone boundary.
            // 中文翻译：严格保留 CAM 安全区阶段边界；退回/接近调用控制器 Z 轴
            // 绝对运动并校验有符号目标，SafeXY 与 SafeAC 分别提交，禁止跨阶段圆滑。
            int segmentIndex = 0;
            while (segmentIndex < approach.transition.segments.size()) {
                const auto phase = approach.transition.segments.at(segmentIndex).phase;
                int phaseEnd = segmentIndex + 1;
                while (phaseEnd < approach.transition.segments.size()
                       && approach.transition.segments.at(phaseEnd).phase == phase) {
                    ++phaseEnd;
                }
                const bool absoluteZPhase = phase == lcnc::cam::RapidSegmentPhase::Retract
                    || phase == lcnc::cam::RapidSegmentPhase::Approach;
                QElapsedTimer phaseElapsed;
                phaseElapsed.start();
                if (absoluteZPhase) {
                    for (int index = segmentIndex; index < phaseEnd; ++index) {
                        if ((approach.transition.segments.at(index).movingAxisMask & ~0x04u) != 0) {
                            // 中文翻译：CAM 首段 Z 阶段包含非 Z 轴运动
                            if (startError) *startError = tr(
                                "CAM initial Z phase contains a non-Z axis movement");
                            return false;
                        }
                    }
                    const double targetZ = approach.transition.segments.at(phaseEnd - 1)
                        .target.axes[2];
                    const double velocity = tool.m_dIdleZVelocity > 0.0
                        ? tool.m_dIdleZVelocity : rapidTool.m_dLineVelocity;
                    const DeviceCommandResult move = m_deviceQueue->executeAndWait(
                        DeviceCommandQueue::ResultCommand(
                            [this, targetZ, velocity] {
                                return m_service->moveAbsoluteAndWait(
                                    Axis::Z, targetZ, velocity, 30000, 0.05);
                            }),
                        TaskPriority::Workflow, 35000);
                    if (!move.success) {
                        // 中文翻译：CAM 首段 Z 轴绝对运动失败
                        if (startError) *startError = move.error.isEmpty()
                            ? tr("Initial CAM absolute Z movement failed") : move.error;
                        return false;
                    }
                } else {
                    QVector<MachinePose5> rapidPoses;
                    rapidPoses.reserve(phaseEnd - segmentIndex);
                    for (int index = segmentIndex; index < phaseEnd; ++index)
                        rapidPoses.append(solvedRapidPose(
                            approach.transition.segments.at(index)));
                    if (!submitCoordinatedRapid(rapidPoses)) {
                        // 中文翻译：CAM 首段执行失败
                        if (startError) *startError = commandError.isEmpty()
                            ? tr("Initial CAM approach execution failed") : commandError;
                        return false;
                    }
                }
                LCNC_INFO(lcnc::LogCode::Generic,
                          "stage=process.initial_approach.execute_phase event=end phase={} segments={} elapsed_ms={}",
                          static_cast<int>(phase), phaseEnd - segmentIndex,
                          phaseElapsed.elapsed());
                segmentIndex = phaseEnd;
            }
            // Direct absolute-Z phases bypass the buffered sink. Re-establish
            // cutting parameters even when SafeXY/SafeAC contained no motion.
            // 中文翻译：Z 轴绝对运动绕过缓存指令汇；即使 SafeXY/SafeAC 无位移，
            // 也必须在切割前重新建立切割参数。
            commandSink.resetProgram();
            commandSink.applyToolMotionParams(tool, /*jump=*/false);
        }
        if (tool.m_bCuttingHead)
            commandSink.startCuttingHead(tool);
        initialApproachSucceeded = true;
    }

    commandSink.setShutterTimings(tool.m_dBeforeOn, tool.m_dAfterOn,
                            tool.m_dBeforeOff, tool.m_dAfterOff, tool.m_dBlowDelay);
    commandSink.laserOn(tool);

    // ——— 协调插补段：beginSegment → lineTo*  → endSegment ———
    if (!commandSink.beginSegment(leadPose, tool, &commandError)
        || !commandSink.lineTo(contourStartPose, tool, &commandError)) {
        if (startError) *startError = commandError.isEmpty()
            ? tr("Controller command generation failed")
            : commandError;
        return false;
    }

    for (int j = 1; j < pts.size(); ++j) {
        if ((j % kTokenPollEvery) == 0) {
            QVariantMap env;
            env.insert(QString::fromLatin1(kEnvContourIndex), contourIndex);
            env.insert(QString::fromLatin1(kEnvContourTotal), total);
            env.insert(QString::fromLatin1(kEnvBackend), commandSink.id());
            env.insert(QString::fromLatin1(kEnvPhase), QStringLiteral("segment"));
            env.insert(QStringLiteral("segmentIndex"), j);
            if (!interrupt.noteCheckpoint(nodeId,
                                          makeLabel(QStringLiteral("segment"), contourIndex, total),
                                          env)) {
                return false;
            }
        }
        if (!commandSink.lineTo(toPose5(pts[j], ox, oy), tool, &commandError)) {
            if (startError) *startError = commandError.isEmpty()
                ? tr("Controller command generation failed")
                : commandError;
            return false;
        }
    }

    commandSink.endSegment(tool);
    commandSink.laserOff(tool);
    commandSink.endProgram(tool);

    // ——— 一次性提交并启动（不在持锁区等待控制器完成）———
    QString flushErr;
    if (!commandSink.startProgram(&flushErr)) {
        if (startError) *startError = flushErr.isEmpty()
            // 中文翻译：控制器执行失败
            ? tr("Controller execution failed")
            : flushErr;
        return false;
    }
        return true;
    };

    if (pureSimulation) {
        if (!constructAndStart(*sink, errorMessage))
            return false;
    } else {
        const auto result = m_deviceQueue->executeAndWait(
            DeviceCommandQueue::ResultCommand([&] {
                QString startError;
                const bool success = constructAndStart(*sink, &startError);
                return DeviceCommandResult{success, startError};
            }),
            TaskPriority::Workflow,
            -1);
        if (!result.success) {
            if (errorMessage)
                *errorMessage = result.error;
            return false;
        }
    }

    if (skippedEmptyContour)
        return true;

    QString pollErrorText;
    while (true) {
        // “暂停”只在轮廓边界生效；当前已下发的轮廓自然完成。停止/急停
        // 则由 Stop 优先级命令落到控制器，随后本轮询立即观察到完成。
        if (interrupt.isStopping()) {
            if (errorMessage)
                // 中文翻译：普通切割已被中断
                *errorMessage = tr("Normal cutting has been interrupted");
            return false;
        }

        bool running = false;
        if (pureSimulation || !m_deviceQueue) {
            running = sink->isProgramRunning(&pollErrorText);
        } else {
            const auto state = std::make_shared<bool>(false);
            const DeviceCommandResult result = m_deviceQueue->executeAndWait(
                DeviceCommandQueue::ResultCommand([this, sink, state] {
                    QString pollError;
                    *state = sink->isProgramRunning(&pollError);
                    return DeviceCommandResult{pollError.isEmpty(), pollError};
                }), TaskPriority::Workflow, 1000);
            if (!result.success) {
                if (errorMessage)
                    // 中文翻译：控制器状态读取失败
                    *errorMessage = result.error.isEmpty() ? tr("Controller status read failed") : result.error;
                return false;
            }
            running = *state;
        }
        if (!pollErrorText.isEmpty()) {
            if (errorMessage)
                *errorMessage = pollErrorText;
            return false;
        }
        if (!running)
            return true;
        QThread::msleep(10);
    }
}

bool NormalCuttingManager::cacheKeyMatches(const CuttingListCacheKey& key) const
{
    return m_hasCuttingListCache
        && m_cuttingListCacheKey.snapshotRevision == key.snapshotRevision
        && m_cuttingListCacheKey.planRevision == key.planRevision
        && m_cuttingListCacheKey.startNumber == key.startNumber
        && m_cuttingListCacheKey.endNumber == key.endNumber
        && sameCacheDouble(m_cuttingListCacheKey.offsetX, key.offsetX)
        && sameCacheDouble(m_cuttingListCacheKey.offsetY, key.offsetY);
}

QVector<NormalCuttingManager::CuttingRow> NormalCuttingManager::cachedCuttingListCopy()
{
    QVector<CuttingRow> rows = m_cuttingListCacheRows;
    for (CuttingRow& row : rows) {
        QStringList warnings;
        row.tool = resolveTool(row.data.contour.toolName,
                               row.data.contour.layerName,
                               &warnings);
        for (const auto& w : warnings)
            LCNC_WARN(lcnc::LogCode::Generic, "normal-cutting: {}", w.toStdString());
    }
    return rows;
}

void NormalCuttingManager::storeCuttingListCache(const CuttingListCacheKey& key,
                                                const QVector<CuttingRow>& rows)
{
    m_cuttingListCacheKey = key;
    m_cuttingListCacheRows = rows;
    for (CuttingRow& row : m_cuttingListCacheRows)
        row.tool = nullptr;
    m_hasCuttingListCache = true;

    LCNC_INFO(lcnc::LogCode::Generic,
              "normal-cutting: cached planned cutting list contours={} snapshotRev={} planRev={}",
              rows.size(),
              key.snapshotRevision,
              key.planRevision);
}

void NormalCuttingManager::clearCuttingListCache()
{
    if (m_hasCuttingListCache) {
        LCNC_INFO(lcnc::LogCode::Generic,
                  "normal-cutting: invalidated planned cutting list cache");
    }
    m_hasCuttingListCache = false;
    m_cuttingListCacheKey = {};
    m_cuttingListCacheRows.clear();
}

QVector<NormalCuttingManager::CuttingRow>
NormalCuttingManager::buildCuttingList(const lcnc::cam::ToolpathExportSnapshot& snapshot,
                                       int startNumber,
                                       int endNumber,
                                       double offsetX,
                                       double offsetY,
                                       QString* errorMessage)
{
    const CuttingListCacheKey cacheKey{
        snapshot.revision,
        m_planService ? m_planService->planRevision() : 0ull,
        startNumber,
        endNumber,
        offsetX,
        offsetY
    };

    if (cacheKeyMatches(cacheKey)) {
        LCNC_INFO(lcnc::LogCode::Generic,
                  "normal-cutting: using cached planned cutting list contours={} snapshotRev={} planRev={}",
                  m_cuttingListCacheRows.size(),
                  cacheKey.snapshotRevision,
                  cacheKey.planRevision);
        return cachedCuttingListCopy();
    }

    QVector<CuttingRow> out;

    const QString camBlockReason = camExecutionBlockReason(snapshot);
    if (!camBlockReason.isEmpty()) {
        if (errorMessage) {
            *errorMessage = tr("Machining cannot start: %1").arg(camBlockReason);
        }
        LCNC_ERR(lcnc::LogCode::Generic,
                 "normal-cutting: rejected unsafe CAM execution snapshot: {}",
                 camBlockReason.toStdString());
        return {};
    }

    QHash<std::uint64_t, int> indexById;
    indexById.reserve(snapshot.contours.size());
    for (int i = 0; i < snapshot.contours.size(); ++i)
        indexById.insert(snapshot.contours.at(i).contourId, i);

    if (m_planService) {
        ProcessCuttingPlanService::CuttingListFilter filter;
        filter.startSequence = startNumber;
        filter.endSequence   = endNumber;
        const auto plan = m_planService->buildCuttingList(filter);
        std::uint64_t previousSelectedContourId = 0;
        for (const auto& e : plan) {
            const int srcIdx = indexById.value(e.contourId, -1);
            if (srcIdx < 0) continue;
            const auto& contour = snapshot.contours.at(srcIdx);
            if (contour.needsRecalculation) {
                if (errorMessage) {
                    const QString reason = contour.recalculationReason.trimmed().isEmpty()
                        // 中文翻译：刀路尚未重新计算
                        ? tr("Toolpath has not been recalculated") : contour.recalculationReason;
                    // 中文翻译：轮廓 "%1" 无法加工：%2
                    *errorMessage = tr("Contour \"%1\" cannot be machined: %2")
                                        .arg(contour.contourName, reason);
                }
                LCNC_ERR(lcnc::LogCode::Generic,
                         "normal-cutting: contour {} requires CAM recalculation",
                         contour.contourId);
                return {};
            }
            if (!contour.hasLeadIn || !contour.leadInPoint.machineCoordValid) {
                if (errorMessage) {
                    const QString reason = contour.leadInError.trimmed().isEmpty()
                        // 中文翻译：下刀位姿无效
                        ? tr("The cutting position is invalid")
                        : contour.leadInError;
                    // 中文翻译：轮廓 "%1" 无法加工：%2
                    *errorMessage = tr("Contour \"%1\" cannot be machined: %2")
                                        .arg(contour.contourName, reason);
                }
                LCNC_ERR(lcnc::LogCode::ToolpathLeadInInvalid,
                         "normal-cutting: contour {} has invalid lead-in: {}",
                         contour.contourId,
                         contour.leadInError.toStdString());
                return {};
            }
            CuttingRow row;
            row.data.contour = contour;
            row.data.points  = snapshot.pointsByContourId.value(contour.contourId);
            if (const auto* transition = snapshot.travelPlan.transitionTo(contour.contourId);
                transition
                && canReuseCommittedTransition(previousSelectedContourId, *transition)) {
                row.entryTransition = *transition;
                row.hasEntryTransition = true;
            }
            row.compensationOffsetX = offsetX;
            row.compensationOffsetY = offsetY;
            if (row.data.points.size() < 2) {
                LCNC_WARN(lcnc::LogCode::Generic,
                          "normal-cutting: contour {} skipped (only {} point(s))",
                          contour.contourId, row.data.points.size());
                continue;
            }
            for (const auto& point : row.data.points) {
                if (point.machineCoordValid)
                    continue;
                if (errorMessage) {
                    // 中文翻译：轮廓 "%1" 无法加工：存在未求解的五轴刀路点
                    *errorMessage = tr("Contour \"%1\" cannot be machined: there are unresolved five-axis toolpath points")
                                        .arg(contour.contourName);
                }
                LCNC_ERR(lcnc::LogCode::Generic,
                         "normal-cutting: contour {} contains invalid machine coordinates",
                         contour.contourId);
                return {};
            }
            QStringList warnings;
            row.tool = resolveTool(e.toolName, e.layerName, &warnings);
            for (const auto& w : warnings)
                LCNC_WARN(lcnc::LogCode::Generic, "normal-cutting: {}", w.toStdString());
            out.append(std::move(row));
            previousSelectedContourId = contour.contourId;
        }
        if (!out.isEmpty())
        {
            // CAM v5 owns canonical and cross-contour continuous rotary angles.
            // Process executes the ordered snapshot without geometry-specific re-solving.
            storeCuttingListCache(cacheKey, out);
            return out;
        }
        // Phase F：掉落的 fallback 路径已删除。plan service 失败时直接返回空结果。
        if (errorMessage)
            // 中文翻译：无法生成切割链表: 加工链表服务未返回数据
            *errorMessage = tr("Unable to generate cutting list: Processing list service did not return data");
        LCNC_ERR(lcnc::LogCode::Generic,
                 "normal-cutting: cutting plan service returned empty list, giving up");
    }

    return out;
}

Tool* NormalCuttingManager::resolveTool(const QString& toolName,
                                         const QString& layerName,
                                         QStringList* warnings)
{
    auto match = [](Tool* t, const QString& expected) {
        return t && !expected.trimmed().isEmpty()
            && QString::fromStdString(t->m_strName) == expected.trimmed();
    };

    Tool* t = ToolFactory::GetTool(toolName);
    if (match(t, toolName))
        return t;

    Tool* fallback = ToolFactory::GetTool(layerName);
    if (match(fallback, layerName)) {
        if (warnings)
            // 中文翻译：工具 "%1" 未注册，已回退到图层名 "%2"
            warnings->append(tr("Tool \"%1\" is not registered and has fallen back to layer name \"%2\"").arg(toolName, layerName));
        return fallback;
    }

    if (warnings)
        // 中文翻译：工具 "%1"/图层 "%2" 均未注册，使用默认工具参数
        warnings->append(tr("Tool \"%1\"/layer \"%2\" are not registered, using default tool parameters").arg(toolName, layerName));
    return &m_sanitizedDefaultTool;
}

} // namespace lcnc::process
