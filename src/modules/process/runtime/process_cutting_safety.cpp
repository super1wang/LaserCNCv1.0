#include "modules/process/runtime/process_cutting_safety.h"

#include <QCoreApplication>

namespace lcnc::process {

DeviceCommandResult evaluateContourBoundaryHealth(const ContourBoundaryHealth& health)
{
    const auto cuttingTr = [](const char* source) {
        return QCoreApplication::translate("lcnc::process::NormalCuttingManager", source);
    };
    const auto fail = [](const QString& error) {
        return DeviceCommandResult{false, error};
    };
    if (!health.connected)
        // 中文翻译：加工过程中运动控制器未连接
        return fail(cuttingTr("The motion controller is not connected during processing"));
    if (!health.statusReadable)
        // 中文翻译：加工过程中无法读取运动控制器状态
        return fail(cuttingTr("Unable to read motion controller status during processing"));
    if (health.faultCode != 0)
        // 中文翻译：加工过程中运动控制器故障码: %1
        return fail(cuttingTr("Motion controller fault code during processing: %1")
                        .arg(health.faultCode));
    const QStringList unsafeAxes = health.missingAxes + health.disabledAxes;
    if (!unsafeAxes.isEmpty())
        // 中文翻译：加工过程中轴系未使能: %1
        return fail(cuttingTr("The axis system is not enabled during machining: %1")
                        .arg(unsafeAxes.join(cuttingTr("，"))));
    return {};
}

} // namespace lcnc::process
