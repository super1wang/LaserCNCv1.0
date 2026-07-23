#include "modules/process/toolpath/process_toolpath_service.h"

#include "core/logging/logger.h"
#include "modules/cam/i_cam_toolpath_provider.h"
// process_settings.h removed - using simplified types

#include <QObject>

namespace lcnc::process {

ProcessToolpathService::ProcessToolpathService(std::shared_ptr<lcnc::cam::ICamToolpathProvider> provider)
    : m_provider(std::move(provider))
{
}

void ProcessToolpathService::setProvider(std::shared_ptr<lcnc::cam::ICamToolpathProvider> provider)
{
    m_provider = std::move(provider);
}

lcnc::cam::ToolpathExportSnapshot ProcessToolpathService::refreshSnapshot()
{
    if (!m_provider) {
        m_snapshot = {};
        m_snapshot.description = QObject::tr("未连接 CAM 刀路提供者");
        return m_snapshot;
    }
    m_snapshot = m_provider->exportToolpathSnapshot();
    LCNC_INFO(lcnc::LogCode::Generic,
              "process.toolpath: snapshot revision={} contours={} points={}",
              m_snapshot.revision,
              m_snapshot.contours.size(),
              m_snapshot.totalPointCount());
    return m_snapshot;
}

lcnc::cam::ToolpathExportSnapshot ProcessToolpathService::refreshSnapshotForOrder(
    const QVector<std::uint64_t>& orderedContourIds)
{
    if (!m_provider) {
        m_snapshot = {};
        m_snapshot.description = QObject::tr("未连接 CAM 刀路提供者");
        return m_snapshot;
    }
    // Cutting-plan mutations resolve the authoritative CAM order on the GUI
    // thread before publishing planChanged. Process consumes the immutable
    // cached snapshot here and must never mutate CAM from its workflow thread.
    m_snapshot = m_provider->exportToolpathSnapshotForOrder(orderedContourIds);
    LCNC_INFO(lcnc::LogCode::Generic,
              "process.toolpath: ordered snapshot revision={} contours={} points={} orderSize={}",
              m_snapshot.revision,
              m_snapshot.contours.size(),
              m_snapshot.totalPointCount(),
              orderedContourIds.size());
    return m_snapshot;
}

ProcessJobPlan ProcessToolpathService::buildJobPlan() const
{
    ProcessJobPlan plan;
    plan.revision = m_snapshot.revision;
    // 工具参数的真实解析在 NormalCuttingManager::resolveTool() 中按 layer/toolName 落地；
    // 这里只填默认占位，保证 ProcessJobContour 字段一致。

    for (const auto& contour : m_snapshot.contours) {
        if (!contour.enabled || !contour.layerEnabled)
            continue;

        ProcessJobContour jobContour;
        jobContour.contour = contour;
        jobContour.points = m_snapshot.pointsByContourId.value(contour.contourId);
        jobContour.toolSettings = ProcessToolSettings{};
        if (jobContour.points.isEmpty())
            jobContour.warnings.append(QObject::tr("轮廓 %1 没有刀路点").arg(QString::number(contour.contourId)));

        bool hasInvalidMachinePoint = false;
        for (const auto& point : jobContour.points) {
            if (!point.machineCoordValid) {
                hasInvalidMachinePoint = true;
                break;
            }
        }
        if (hasInvalidMachinePoint)
            jobContour.warnings.append(QObject::tr("轮廓 %1 包含无效机床坐标点").arg(QString::number(contour.contourId)));

        plan.totalPointCount += jobContour.points.size();
        plan.warnings.append(jobContour.warnings);
        plan.contours.append(jobContour);
    }

    plan.valid = !plan.contours.isEmpty() && plan.totalPointCount > 0;
    if (!plan.valid)
        plan.warnings.append(QObject::tr("没有可执行的启用刀路轮廓"));
    return plan;
}

} // namespace lcnc::process
