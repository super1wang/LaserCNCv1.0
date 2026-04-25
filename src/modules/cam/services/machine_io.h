#pragma once

class LcncDocument;
class MachineKinematics;
class TaskProgress;
class QString;

namespace lcnc::cam::machine_io {

/**
 * @brief 机台文件 I/O 纯服务（v2.3 从 CamModule 抽出）。
 *
 * 仅做 STEP/STL/BREP 读写 + XCAF 装配，不触发任务、不 emit 信号、
 * 不写模块状态。任务调度与信号转发留在 CamModule。
 */

/// 把 STEP/STL/BREP 文件读入 LcncDocument 的 Machine 实体集合。
/// 由调用方在 TaskProgress 任务内调用，进度区间假定 [0,100]。
/// 失败返回 false（文件不存在/格式不支持/读取失败）。
bool loadMachineFromFile(LcncDocument* doc,
                         const QString& filePath,
                         TaskProgress* progress);

/// 把当前机台 LcncDocument 按轴分组导出为 STEP（含 LCNC_AXIS_* 命名）。
/// 失败返回 false。
bool exportMachineToFile(LcncDocument* doc,
                         MachineKinematics* kin,
                         const QString& filePath);

} // namespace lcnc::cam::machine_io
