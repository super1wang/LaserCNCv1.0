#include "modules/process/toolpath/process_toolpath_service.h"

#include "core/logging/logger.h"
#include "modules/cam/i_cam_toolpath_provider.h"
// process_settings.h removed - using simplified types

#include <QObject>
#include <QReadLocker>
#include <QWriteLocker>

namespace lcnc::process {

ProcessToolpathService::ProcessToolpathService(std::shared_ptr<lcnc::cam::ICamToolpathProvider> provider)
    : m_provider(std::move(provider))
{
}

void ProcessToolpathService::setProvider(std::shared_ptr<lcnc::cam::ICamToolpathProvider> provider)
{
    QWriteLocker locker(&m_snapshotLock);
    m_provider = std::move(provider);
}

lcnc::cam::ToolpathExportSnapshot ProcessToolpathService::refreshSnapshot()
{
    std::shared_ptr<lcnc::cam::ICamToolpathProvider> provider;
    {
        QReadLocker locker(&m_snapshotLock);
        provider = m_provider;
    }
    lcnc::cam::ToolpathExportSnapshot snapshot;
    if (!provider) {
        // 中文翻译：未连接 CAM 刀路提供者
        snapshot.description = QObject::tr("CAM toolpath provider not connected");
    } else {
        snapshot = provider->exportToolpathCatalogSnapshot();
    }
    LCNC_INFO(lcnc::LogCode::Generic,
              "process.toolpath: snapshot revision={} contours={} points={}",
              snapshot.revision,
              snapshot.contours.size(),
              snapshot.totalPointCount());
    {
        QWriteLocker locker(&m_snapshotLock);
        m_snapshot = snapshot;
    }
    return snapshot;
}

lcnc::cam::ToolpathExportSnapshot ProcessToolpathService::refreshCommittedExecutionSnapshot()
{
    std::shared_ptr<lcnc::cam::ICamToolpathProvider> provider;
    {
        QReadLocker locker(&m_snapshotLock);
        provider = m_provider;
    }
    lcnc::cam::ToolpathExportSnapshot snapshot;
    if (!provider) {
        // 中文翻译：未连接 CAM 刀路提供者
        snapshot.description = QObject::tr("CAM toolpath provider not connected");
    } else {
        // Process only receives CAM's current committed execution result.  It
        // cannot pass a different contour order, trigger a re-solve, or edit
        // the exported path from its workflow thread.
        // 中文翻译：Process 只能读取 CAM 已提交的执行结果，不能传入顺序、触发重算或改写刀路。
        snapshot = provider->exportCommittedExecutionSnapshot();
    }
    LCNC_INFO(lcnc::LogCode::Generic,
              "process.toolpath: committed execution snapshot revision={} contours={} points={}",
              snapshot.revision,
              snapshot.contours.size(),
              snapshot.totalPointCount());
    {
        QWriteLocker locker(&m_snapshotLock);
        m_snapshot = snapshot;
    }
    return snapshot;
}

 lcnc::cam::ToolpathExportSnapshot ProcessToolpathService::currentSnapshot() const
{
    QReadLocker locker(&m_snapshotLock);
    return m_snapshot;
}

ProcessJobPlan ProcessToolpathService::buildJobPlan() const
{
    const lcnc::cam::ToolpathExportSnapshot snapshot = currentSnapshot();
    ProcessJobPlan plan;
    plan.revision = snapshot.revision;
    // 工具参数的真实解析在 NormalCuttingManager::resolveTool() 中按 layer/toolName 落地；
    // 这里只填默认占位，保证 ProcessJobContour 字段一致。

    for (const auto& contour : snapshot.contours) {
        if (!contour.enabled || !contour.layerEnabled)
            continue;

        ProcessJobContour jobContour;
        jobContour.contour = contour;
        jobContour.points = snapshot.pointsByContourId.value(contour.contourId);
        jobContour.toolSettings = ProcessToolSettings{};
        if (jobContour.points.isEmpty())
            // 中文翻译：轮廓 %1 没有刀路点
            jobContour.warnings.append(QObject::tr("Profile %1 has no toolpath points").arg(QString::number(contour.contourId)));

        bool hasInvalidMachinePoint = false;
        for (const auto& point : jobContour.points) {
            if (!point.machineCoordValid) {
                hasInvalidMachinePoint = true;
                break;
            }
        }
        if (hasInvalidMachinePoint)
            // 中文翻译：轮廓 %1 包含无效机床坐标点
            jobContour.warnings.append(QObject::tr("Contour %1 contains invalid machine coordinate points").arg(QString::number(contour.contourId)));

        plan.totalPointCount += jobContour.points.size();
        plan.warnings.append(jobContour.warnings);
        plan.contours.append(jobContour);
    }

    plan.valid = !plan.contours.isEmpty() && plan.totalPointCount > 0;
    if (!plan.valid)
        // 中文翻译：没有可执行的启用刀路轮廓
        plan.warnings.append(QObject::tr("No executable enabled toolpath profile"));
    return plan;
}

} // namespace lcnc::process
