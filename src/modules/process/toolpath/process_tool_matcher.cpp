#include "modules/process/toolpath/process_tool_matcher.h"

// process_settings.h removed - using simplified types

#include <QObject>

namespace lcnc::process {

ProcessToolSettings ProcessToolMatcher::match(const lcnc::cam::ToolpathExportContour& contour,
                                              const lcnc::ProcessSettings& settings,
                                              QStringList* warnings)
{
    ProcessToolSettings tool = ProcessSettingsSchema::snapshotFrom(settings).tool;
    if (contour.toolName.trimmed().isEmpty() && warnings) {
        warnings->append(QObject::tr("轮廓 %1 未绑定 CAM 工具，使用 Process 默认工具参数")
                             .arg(QString::number(contour.contourId)));
    }
    return tool;
}

} // namespace lcnc::process