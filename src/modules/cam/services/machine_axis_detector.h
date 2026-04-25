#pragma once

#include <gp_Pnt.hxx>

#include <QMap>
#include <QString>

class LcncDocument;
class MachineKinematics;
class CamConfig;

namespace lcnc::cam {

/**
 * @brief 机床轴自动识别 + 已存配置回填（v2.3 从 CamModule 抽出）。
 *
 * 这些函数都是无状态算法，被 CamModule 在 loadMachine / autoDetectAxes 流程中调用。
 * 它们只读 LcncDocument 的标签集合 + MachineKinematics 状态，写回 MachineKinematics
 * 与 CamConfig（持久化层），不触碰 GUI。
 */
namespace machine_axis_detector {

/// 把 doc 内 EntityKind::Machine 的 entry→name 映射喂给 kin->autoDetect。
void autoDetectAxisNames(LcncDocument* doc, MachineKinematics* kin);

/// 对每个 Rotary 轴，按其挂载形体的 bbox 中心作为旋转原点。
/// 已经设置过有效原点的轴若希望仍被覆盖，调用方应自行过滤。
void autoDetectAxisOrigins(LcncDocument* doc, MachineKinematics* kin);

/// 从 CamConfig 中读取 machinePath 对应的轴原点 / 切割头位置 / 工件安装位置，
/// 写回 kin 与传出参数。machinePath 为空直接返回。
struct StoredProfile {
    gp_Pnt cutterHeadModelPosition{0.0, 0.0, 0.0};
    gp_Pnt cutterHeadPhysicalPosition{0.0, 0.0, 0.0};
    gp_Pnt workpieceInstallPosition{0.0, 0.0, 0.0};
    bool   hasCutterHeadModel{false};
    bool   hasCutterHeadPhysical{false};
    bool   hasWorkpieceInstall{false};
};
StoredProfile applyStoredMachineProfile(MachineKinematics* kin,
                                        ::CamConfig& config,
                                        const QString& machinePath);

} // namespace machine_axis_detector
} // namespace lcnc::cam
