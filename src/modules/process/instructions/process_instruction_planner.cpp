#include "modules/process/instructions/process_instruction_planner.h"

#include "core/logging/logger.h"

#include <QObject>

namespace lcnc::process {

ProcessCommandBuffer ProcessInstructionPlanner::planJob(const ProcessJobPlan& jobPlan) const
{
    ProcessCommandBuffer buffer;
    if (!jobPlan.valid)
        return buffer;

    for (const ProcessJobContour& contour : jobPlan.contours) {
        if (contour.points.isEmpty())
            continue;

        ProcessCommand feed;
        feed.type = ProcessCommandType::SetFeed;
        feed.feedRate = contour.toolSettings.feedRate;
        feed.description = QObject::tr("设置进给: %1").arg(feed.feedRate);
        buffer.append(feed);

        ProcessCommand laserPower;
        laserPower.type = ProcessCommandType::SetLaserPower;
        laserPower.laserEnergy = contour.toolSettings.laserEnergy;
        laserPower.laserFrequency = contour.toolSettings.laserFrequency;
        laserPower.laserPulseWidth = contour.toolSettings.laserPulseWidth;
        laserPower.description = QObject::tr("设置激光参数");
        buffer.append(laserPower);

        const auto& firstPoint = contour.points.first();
        ProcessCommand rapid;
        rapid.type = ProcessCommandType::MoveLinear;
        rapid.x = firstPoint.machineX;
        rapid.y = firstPoint.machineY;
        rapid.z = firstPoint.machineZ;
        rapid.r1 = firstPoint.machineR1;
        rapid.r2 = firstPoint.machineR2;
        rapid.feedRate = contour.toolSettings.feedRate;
        const QString contourIdText = QString::number(contour.contour.contourId);
        rapid.description = QObject::tr("移动到轮廓 %1 起点").arg(contourIdText);
        buffer.append(rapid);

        ProcessCommand laserOn;
        laserOn.type = ProcessCommandType::LaserOn;
        laserOn.description = QObject::tr("轮廓 %1 激光开").arg(contourIdText);
        buffer.append(laserOn);

        for (const auto& point : contour.points) {
            ProcessCommand cut;
            cut.type = ProcessCommandType::MoveLinear;
            cut.x = point.machineX;
            cut.y = point.machineY;
            cut.z = point.machineZ;
            cut.r1 = point.machineR1;
            cut.r2 = point.machineR2;
            cut.feedRate = contour.toolSettings.feedRate;
            cut.description = QObject::tr("切割轮廓 %1").arg(contourIdText);
            buffer.append(cut);
        }

        ProcessCommand laserOff;
        laserOff.type = ProcessCommandType::LaserOff;
        laserOff.description = QObject::tr("轮廓 %1 激光关").arg(contourIdText);
        buffer.append(laserOff);
    }

    LCNC_INFO(lcnc::LogCode::Generic,
              "process.instructions: planned {} commands for {} contours",
              buffer.size(),
              jobPlan.contours.size());
    return buffer;
}

} // namespace lcnc::process