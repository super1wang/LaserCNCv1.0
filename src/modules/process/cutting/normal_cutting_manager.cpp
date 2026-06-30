#include "modules/process/cutting/normal_cutting_manager.h"

#include "core/logging/logger.h"
#include "modules/cam/i_cam_toolpath_provider.h"
#include "modules/process/System/Service.h"
#include "modules/process/Tool/Tool.h"
#include "modules/process/Tool/ToolFactory.h"
#include "modules/process/cutting/process_cutting_plan_service.h"
#include "modules/process/cutting/pure_simulation_toolpath_ticker.h"
#include "modules/process/device/MotionControl/MotionControl.h"
#include "modules/process/process_module.h"
#include "modules/process/runtime/i_motion_command_sink.h"
#include "modules/process/runtime/machine_pose5.h"
#include "modules/process/runtime/process_interrupt_context.h"

#include <QCoreApplication>
#include <QString>
#include <QStringList>
#include <QVariantMap>

#include <algorithm>
#include <cmath>

namespace lcnc::process {

namespace {

constexpr char kStartNumber[]       = "startNumber";
constexpr char kEndNumber[]         = "endNumber";
constexpr char kCompensationIndex[] = "compensationIndex";

constexpr int kPollSliceMs        = 20;
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
    pose.x  = p.machineX + ox;
    pose.y  = p.machineY + oy;
    pose.z  = p.machineZ;
    pose.r1 = p.machineR1;
    pose.r2 = p.machineR2;
    // 默认 X+Y 参与；下游 sink 还会与构型实际拥有的轴 & 即可。
    pose.mask = MachinePose5::Bx | MachinePose5::By
              | MachinePose5::Bz
              | MachinePose5::Br1
              | MachinePose5::Br2;
    return pose;
}

} // namespace

NormalCuttingManager::NormalCuttingManager(Service* service,
                                           std::shared_ptr<lcnc::cam::ICamToolpathProvider> toolpathProvider,
                                           ProcessModule* processModule,
                                           QObject* parent)
    : QObject(parent)
    , m_service(service)
    , m_toolpathProvider(std::move(toolpathProvider))
    , m_processModule(processModule)
    , m_toolpathService(std::make_unique<ProcessToolpathService>(m_toolpathProvider))
    , m_simTicker(std::make_unique<PureSimulationToolpathTicker>(processModule))
{
    // 兜底 Tool —— 当 ToolFactory 找不到匹配工具时 resolveTool() 返回这个。
    // 由 Fix #1 (Tool 类内默认初始化) 保证非赋值字段不再是 0xCD…。这里仅显式覆盖几个最关键的。
    m_sanitizedDefaultTool.m_strDirectionX  = "X";
    m_sanitizedDefaultTool.m_strDirectionY  = "Y";
    m_sanitizedDefaultTool.m_strName        = "__fallback__";
    m_sanitizedDefaultTool.m_dLineVelocity  = 600.0;
    m_sanitizedDefaultTool.m_dIdleZHeight   = 5.0;
    m_sanitizedDefaultTool.m_dCuttingHeight = 0.0;
}

NormalCuttingManager::~NormalCuttingManager() = default;

void NormalCuttingManager::setCuttingPlanService(ProcessCuttingPlanService* service)
{
    m_planService = service;
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
        if (errorMessage) *errorMessage = tr("CAM 刀路提供者未注册");
        return false;
    }
    const auto snapshot = m_toolpathService->refreshSnapshot();
    if (!snapshot.hasEnabledContours()) {
        if (errorMessage) *errorMessage = tr("CAM 中没有可执行的启用轮廓");
        return false;
    }

    auto cuttingList = buildCuttingList(snapshot, startNumber, endNumber, compOffsetX, compOffsetY, errorMessage);
    if (cuttingList.isEmpty()) {
        if (errorMessage && errorMessage->isEmpty())
            *errorMessage = tr("筛选后的切割链表为空");
        return false;
    }

    // 选 sink —— 工厂内自决 ACS / GTN / PureSim。
    MotionControl* mc = m_service ? m_service->GetMotionControl() : nullptr;
    const bool simMode = m_processModule && m_processModule->simulationMode();
    auto sink = MotionSinkFactory::create(mc, simMode, m_simTicker.get(), m_processModule);
    if (!sink) {
        if (errorMessage) *errorMessage = tr("运动指令汇构造失败（构型/控制器不匹配）");
        return false;
    }
    sink->setCancellation(&ic);
    const QString backendLabel = sink->id();

    // 联机硬件路径强依赖 Tool::m_strDirectionX/Y（ACS 坐标系轴名）。空字段补回 X/Y，避免崩。
    if (backendLabel != QStringLiteral("PureSimulation")) {
        for (auto& row : cuttingList) {
            if (!row.tool) continue;
            if (row.tool->m_strDirectionX.empty()) row.tool->m_strDirectionX = "X";
            if (row.tool->m_strDirectionY.empty()) row.tool->m_strDirectionY = "Y";
        }
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
            emit logMessage(tr("从断点续跑：跳过前 %1 条轮廓").arg(startContourIndex));
        } else {
            ic.clearResumePoint(nodeId);
        }
    }

    emit logMessage(tr("普通切割开始：%1 条轮廓（起始 %2），后端=%3")
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
            if (errorMessage) *errorMessage = tr("普通切割已被中断");
            return false;
        }

        const CuttingRow& row = cuttingList[i];
        emit contourStarted(i + 1, total,
                            tr("轮廓 #%1 (%2)")
                                .arg(row.data.contour.contourId)
                                .arg(row.data.contour.contourName));

        bool ok = false;
        try {
            ok = executeContour(*sink, row, ic, nodeId, i, total, errorMessage);
        } catch (const std::bad_optional_access& ex) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "normal-cutting: bad_optional_access on contour {} ({}), tool='{}'",
                     row.data.contour.contourId, ex.what(),
                     row.tool ? row.tool->m_strName : std::string("<null>"));
            if (errorMessage)
                *errorMessage = tr("轮廓 %1 执行异常：工具方向字段无效").arg(row.data.contour.contourId);
            restoreAxisDriver();
            return false;
        } catch (const std::exception& ex) {
            LCNC_ERR(lcnc::LogCode::Generic,
                     "normal-cutting: exception on contour {}: {}",
                     row.data.contour.contourId, ex.what());
            if (errorMessage)
                *errorMessage = tr("轮廓 %1 执行异常：%2")
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
                *errorMessage = tr("轮廓 %1 执行中断").arg(row.data.contour.contourId);
            return false;
        }

        env[QString::fromLatin1(kEnvPhase)] = QStringLiteral("afterContour");
        if (!ic.checkpoint(nodeId, makeLabel(QStringLiteral("afterContour"), i, total), env)) {
            restoreAxisDriver();
            if (errorMessage) *errorMessage = tr("普通切割已被中断");
            return false;
        }

        emit contourFinished(i + 1, total);
    }

    ic.clearResumePoint(nodeId);
    restoreAxisDriver();
    emit logMessage(tr("普通切割完成"));
    return true;
}

bool NormalCuttingManager::executeContour(IMotionCommandSink& sink,
                                           const CuttingRow& row,
                                           ProcessInterruptContext& interrupt,
                                           const QString& nodeId,
                                           int contourIndex,
                                           int total,
                                           QString* errorMessage)
{
    if (!row.tool) {
        if (errorMessage) *errorMessage = tr("轮廓 %1 没有绑定工具").arg(row.data.contour.contourId);
        return false;
    }
    const Tool& tool = *row.tool;
    const auto& pts = row.data.points;
    if (pts.size() < 2)
        return true;  // 空轮廓静默跳过

    const double ox = row.compensationOffsetX;
    const double oy = row.compensationOffsetY;
    const auto& p0 = pts.front();
    const double p0x = p0.machineX + ox;
    const double p0y = p0.machineY + oy;
    const MachinePose5 startPose = toPose5(p0, ox, oy);

    // ——— 程序起始（与遗留 buildContourACS / executeContourGTN 等价的语义序列）———
    sink.resetProgram();
    sink.applyToolMotionParams(tool, /*jump=*/true);

    // JumpToSetAFPos 在原实现里写若干 SET 与 PTP，等价为 sink 内"准备阶段"。
    // 在 sink 抽象下我们只暴露关键 jump 动作；联机硬件保持原 MotionControl 行为。
    if (tool.m_bCuttingHead) {
        if (tool.m_bCrossBridge) {
            sink.stopCuttingHead();
            sink.jumpToIdleZ(tool);
            sink.jumpToXY(p0x, p0y, tool);
            sink.jumpToPose(startPose, tool);
            sink.startCuttingHead(tool);
        } else {
            sink.jumpToXY(p0x, p0y, tool);
            sink.jumpToPose(startPose, tool);
            sink.startCuttingHead(tool);
        }
    } else {
        sink.jumpToIdleZ(tool);
        sink.jumpToXY(p0x, p0y, tool);
        sink.jumpToPose(startPose, tool);
    }

    sink.setShutterTimings(tool.m_dBeforeOn, tool.m_dAfterOn,
                            tool.m_dBeforeOff, tool.m_dAfterOff, tool.m_dBlowDelay);
    sink.laserOn(tool);

    // ——— 协调插补段：beginSegment → lineTo*  → endSegment ———
    sink.beginSegment(startPose, tool);

    for (int j = 1; j < pts.size(); ++j) {
        if ((j % kTokenPollEvery) == 0) {
            QVariantMap env;
            env.insert(QString::fromLatin1(kEnvContourIndex), contourIndex);
            env.insert(QString::fromLatin1(kEnvContourTotal), total);
            env.insert(QString::fromLatin1(kEnvBackend), sink.id());
            env.insert(QString::fromLatin1(kEnvPhase), QStringLiteral("segment"));
            env.insert(QStringLiteral("segmentIndex"), j);
            if (!interrupt.noteCheckpoint(nodeId,
                                          makeLabel(QStringLiteral("segment"), contourIndex, total),
                                          env)) {
                return false;
            }
        }
        sink.lineTo(toPose5(pts[j], ox, oy), tool);
    }

    sink.endSegment(tool);
    sink.laserOff(tool);
    sink.endProgram(tool);

    // ——— 一次性下发 + 等待完成（ACS: LoadBuffer+RunBuffer+WaitEnd；
    //                          GTN: CrdDataEx+CrdStart+PrfTrapAxis；
    //                          PureSim: 启动 ticker + 等回放完成）———
    QString flushErr;
    if (!sink.flush(&flushErr)) {
        if (errorMessage) *errorMessage = flushErr.isEmpty()
            ? tr("控制器执行失败")
            : flushErr;
        return false;
    }

    QCoreApplication::processEvents(QEventLoop::AllEvents, kPollSliceMs);
    return true;
}

QVector<NormalCuttingManager::CuttingRow>
NormalCuttingManager::buildCuttingList(const lcnc::cam::ToolpathExportSnapshot& snapshot,
                                       int startNumber,
                                       int endNumber,
                                       double offsetX,
                                       double offsetY,
                                       QString* errorMessage)
{
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
            QStringList warnings;
            row.tool = resolveTool(e.toolName, e.layerName, &warnings);
            for (const auto& w : warnings)
                LCNC_WARN(lcnc::LogCode::Generic, "normal-cutting: {}", w.toStdString());
            out.append(std::move(row));
        }
        if (!out.isEmpty())
            return out;
        // Phase F：掉落的 fallback 路径已删除。plan service 失败时直接返回空结果。
        if (errorMessage)
            *errorMessage = tr("无法生成切割链表: 加工链表服务未返回数据");
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
            warnings->append(tr("工具 \"%1\" 未注册，已回退到图层名 \"%2\"").arg(toolName, layerName));
        return fallback;
    }

    if (warnings)
        warnings->append(tr("工具 \"%1\"/图层 \"%2\" 均未注册，使用默认工具参数").arg(toolName, layerName));
    return &m_sanitizedDefaultTool;
}

} // namespace lcnc::process
