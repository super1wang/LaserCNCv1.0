#include "modules/process/cutting/normal_cutting_manager.h"

#include "core/logging/logger.h"
#include "modules/cam/i_cam_toolpath_provider.h"
#include "modules/process/runtime/process_device_runtime.h"
#include "modules/process/tool/tool.h"
#include "modules/process/tool/tool_factory.h"
#include "modules/process/cutting/process_cutting_plan_service.h"
#include "modules/process/cutting/pure_simulation_toolpath_ticker.h"
#include "modules/process/device/motion_control/motion_control.h"
#include "modules/process/process_module.h"
#include "modules/process/runtime/i_motion_command_sink.h"
#include "modules/process/runtime/device_command_queue.h"
#include "modules/process/runtime/machine_pose5.h"
#include "modules/process/runtime/process_interrupt_context.h"

#include <QString>
#include <QStringList>
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
    pose.x  = p.machineAxes[0] + ox;
    pose.y  = p.machineAxes[1] + oy;
    pose.z  = p.machineAxes[2];
    pose.r1 = p.machineAxes[3];
    pose.r2 = p.machineAxes[4];
    pose.r1Name = p.rotaryAxis1Name;
    pose.r2Name = p.rotaryAxis2Name;
    // 默认 X+Y 参与；下游 sink 还会与构型实际拥有的轴 & 即可。
    pose.mask = p.machineAxisMask;
    return pose;
}

bool sameCacheDouble(double a, double b)
{
    return std::abs(a - b) <= 1e-9;
}

} // namespace

NormalCuttingManager::NormalCuttingManager(ProcessDeviceRuntime* service,
                                           std::shared_ptr<lcnc::cam::ICamToolpathProvider> toolpathProvider,
                                           ProcessModule* processModule,
                                           DeviceCommandQueue* deviceQueue,
                                           QObject* parent)
    : QObject(parent)
    , m_service(service)
    , m_toolpathProvider(std::move(toolpathProvider))
    , m_processModule(processModule)
    , m_deviceQueue(deviceQueue)
    , m_toolpathService(std::make_unique<ProcessToolpathService>(m_toolpathProvider))
    , m_simTicker(std::make_unique<PureSimulationToolpathTicker>(processModule))
{
    // 兜底 Tool —— 当 ToolFactory 找不到匹配工具时 resolveTool() 返回这个。
    // 由 Fix #1 (Tool 类内默认初始化) 保证非赋值字段不再是 0xCD…。这里仅显式覆盖几个最关键的。
    m_sanitizedDefaultTool.m_strName        = "__fallback__";
    m_sanitizedDefaultTool.m_dLineVelocity  = 600.0;
    m_sanitizedDefaultTool.m_dIdleZHeight   = 5.0;
    m_sanitizedDefaultTool.m_dCuttingHeight = 0.0;
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
        executionSnapshot = m_toolpathService->currentSnapshot();
    } else {
        if (m_planService) {
            ProcessCuttingPlanService::CuttingListFilter filter;
            filter.startSequence = startNumber;
            filter.endSequence = endNumber;
            const auto plan = m_planService->buildCuttingList(filter);
            QVector<std::uint64_t> orderedContourIds;
            orderedContourIds.reserve(plan.size());
            for (const auto& entry : plan)
                orderedContourIds.append(entry.contourId);
            executionSnapshot = m_toolpathService->refreshSnapshotForOrder(orderedContourIds);
        } else {
            executionSnapshot = m_toolpathService->refreshSnapshot();
        }
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
    const bool simMode = m_processModule && m_processModule->simulationMode();
    std::shared_ptr<IMotionCommandSink> sink;
    QString backendLabel;
    if (simMode) {
        auto created = m_service
            ? m_service->createMotionSink(true, m_simTicker.get(), m_processModule,
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
                                               layout = executionSnapshot.machineAxisLayout] {
                auto created = m_service->createMotionSink(
                    false, m_simTicker.get(), m_processModule, layout);
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

    // 旁路 ProcessModule 的 Lissajous 正弦波。
    if (m_processModule)
        m_processModule->setNormalCuttingActive(true);
    auto restoreAxisDriver = [this]() {
        if (m_processModule) m_processModule->setNormalCuttingActive(false);
    };

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
            restoreAxisDriver();
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
            restoreAxisDriver();
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
            restoreAxisDriver();
            return false;
        }

        if (!ok) {
            restoreAxisDriver();
            if (ic.isStopping())
                return false;
            if (errorMessage && errorMessage->isEmpty())
                // 中文翻译：轮廓 %1 执行中断
                *errorMessage = tr("Execution of contour %1 interrupted").arg(row.data.contour.contourId);
            return false;
        }

        env[QString::fromLatin1(kEnvPhase)] = QStringLiteral("afterContour");
        if (!ic.checkpoint(nodeId, makeLabel(QStringLiteral("afterContour"), i, total), env)) {
            restoreAxisDriver();
            // 中文翻译：普通切割已被中断
            if (errorMessage) *errorMessage = tr("Normal cutting has been interrupted");
            return false;
        }

        emit contourFinished(i + 1, total);
    }

    ic.clearResumePoint(nodeId);
    restoreAxisDriver();
    // 中文翻译：普通切割完成
    emit logMessage(tr("Ordinary cutting completed"));
    return true;
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

    // JumpToSetAFPos 在原实现里写若干 SET 与 PTP，等价为 sink 内"准备阶段"。
    // 在 sink 抽象下我们只暴露关键 jump 动作；联机硬件保持原 MotionControl 行为。
    if (tool.m_bCuttingHead) {
        if (tool.m_bCrossBridge) {
            commandSink.stopCuttingHead();
            commandSink.jumpToIdleZ(leadPose, tool);
            commandSink.jumpToPose(leadPose, tool);
            commandSink.startCuttingHead(tool);
        } else {
            commandSink.jumpToIdleZ(leadPose, tool);
            commandSink.jumpToPose(leadPose, tool);
            commandSink.startCuttingHead(tool);
        }
    } else {
        commandSink.jumpToIdleZ(leadPose, tool);
        commandSink.jumpToPose(leadPose, tool);
    }

    commandSink.jumpToCuttingZ(leadPose, tool);

    commandSink.setShutterTimings(tool.m_dBeforeOn, tool.m_dAfterOn,
                            tool.m_dBeforeOff, tool.m_dAfterOff, tool.m_dBlowDelay);
    commandSink.laserOn(tool);

    // ——— 协调插补段：beginSegment → lineTo*  → endSegment ———
    QString commandError;
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
            30000);
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

    QHash<std::uint64_t, int> indexById;
    indexById.reserve(snapshot.contours.size());
    for (int i = 0; i < snapshot.contours.size(); ++i)
        indexById.insert(snapshot.contours.at(i).contourId, i);

    if (m_planService) {
        ProcessCuttingPlanService::CuttingListFilter filter;
        filter.startSequence = startNumber;
        filter.endSequence   = endNumber;
        const auto plan = m_planService->buildCuttingList(filter);
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
