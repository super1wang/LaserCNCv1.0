#include "modules/process/toolpath/process_tool_matcher.h"

#include <QObject>

namespace lcnc::process {

ProcessToolSettings ProcessToolMatcher::match(const lcnc::cam::ToolpathExportContour& contour,
                                              QStringList* warnings)
{
    ProcessToolSettings tool;
    if (contour.toolName.trimmed().isEmpty() && warnings) {
        warnings->append(QObject::tr("轮廓 %1 未绑定 CAM 工具，使用 Process 默认工具参数")
                             .arg(QString::number(contour.contourId)));
    }
    return tool;
}

} // namespace lcnc::process
