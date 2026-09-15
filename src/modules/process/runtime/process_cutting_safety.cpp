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

QString camExecutionBlockReason(const lcnc::cam::ToolpathExportSnapshot& snapshot,
                                bool realMachineExecution)
{
    const auto cuttingTr = [](const char* source) {
        return QCoreApplication::translate("lcnc::process::NormalCuttingManager", source);
    };
    const auto& collision = snapshot.motionPlan.collision;
    const auto& safety = snapshot.collisionSafety;
    const bool collisionRequired = safety.effectiveVerificationMode()
        == lcnc::cam::CollisionVerificationMode::Required;
    if (realMachineExecution && collisionRequired && safety.machinePackageRequired
        && (!safety.machinePackageReady || safety.packageBuildInProgress)) {
        if (!safety.failureReason.isEmpty())
            return safety.failureReason;
        // 中文翻译：机台安全包尚未就绪或正在构建；真实机台运动已失败关闭
        return cuttingTr("The machine safety package is not ready or is being built; real-machine motion is fail-closed");
    }
    if (realMachineExecution && collisionRequired && safety.jobOverlayRequired
        && (!safety.jobOverlayReady || safety.jobOverlayBuildInProgress)) {
        if (!safety.jobOverlayFailureReason.isEmpty())
            return safety.jobOverlayFailureReason;
        // 中文翻译：工件碰撞叠加缓存尚未就绪或正在构建；真实机台运动已失败关闭
        return cuttingTr("The workpiece collision overlay is not ready or is being built; real-machine motion is fail-closed");
    }
    if (collisionRequired && (!collision.complete
        || collision.state == lcnc::cam::CollisionValidationState::Pending)) {
        // 中文翻译：CAM 全路径碰撞校验尚未完成
        return cuttingTr("CAM full-path collision validation is incomplete");
    }
    if (collisionRequired && collision.blocksExecution(collision.blockWarning)) {
        if (!collision.failureReason.isEmpty())
            return collision.failureReason;
        // 中文翻译：CAM 全路径碰撞校验未确认路径安全
        return cuttingTr("CAM full-path collision validation did not confirm a safe path");
    }
    if (snapshot.contours.size() > 1 && !snapshot.travelPlan.isPathReady()) {
        // 中文翻译：空程规划丢失或已过期
        return snapshot.travelPlan.failureReason.isEmpty()
            ? cuttingTr("The rapid travel plan is missing or out of date")
            : snapshot.travelPlan.failureReason;
    }
    if (realMachineExecution && collisionRequired
        && snapshot.motionPlan.nodes.size() > 1) {
        const int expectedEdges = snapshot.motionPlan.nodes.size() - 1;
        if (snapshot.motionPlan.edgeCertificates.size() != expectedEdges) {
            // 中文翻译：CAM 连续运动证书缺失或数量不完整；真实机台运动已失败关闭
            return cuttingTr("CAM continuous-motion certificates are missing or incomplete; real-machine motion is fail-closed");
        }
        for (const auto& certificate : snapshot.motionPlan.edgeCertificates) {
            if (certificate.state
                != lcnc::cam::CamMotionCertificateState::CertifiedSafe) {
                if (!certificate.reason.isEmpty())
                    return certificate.reason;
                // 中文翻译：CAM 连续运动证书未确认整段路径安全
                return cuttingTr("A CAM continuous-motion certificate did not confirm the complete edge as safe");
            }
        }
    }
    return {};
}

} // namespace lcnc::process
