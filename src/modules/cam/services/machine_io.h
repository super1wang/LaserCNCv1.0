#pragma once

class LcncDocument;
class MachineKinematics;
class TaskProgress;
class QString;
class TopoDS_Shape;

#include <functional>

#include <QString>
#include <QVector>

#include <TopoDS_Shape.hxx>

namespace lcnc::cam::machine_io {

/**
 * @brief 机台文件 I/O 纯服务（v2.3 从 CamModule 抽出）。
 *
 * 仅做 STEP/STL/BREP 读写 + XCAF 装配，不触发任务、不 emit 信号、
 * 不写模块状态。任务调度与信号转发留在 CamModule。
 */

/// Detached result of reading a machine model. It deliberately contains no
/// LcncDocument/XCAF handle, so parsing can run on a worker without racing the
/// document currently used by the UI, calibration, or collision planner.
struct MachineImportResult {
    struct Part {
        QString      name;
        TopoDS_Shape shape;
    };

    QVector<Part> parts;
    QString       error;

    bool isValid() const { return !parts.isEmpty(); }
};

/// Read STEP/STL/BREP into detached OCC shapes. This is the only machine-file
/// operation allowed on a worker thread; callers commit the result to the live
/// machine document on its owning/UI thread after checking their load token.
bool readMachineFile(const QString& filePath,
                     TaskProgress* progress,
                     MachineImportResult* result);

/// 把 STEP/STL/BREP 文件读入 LcncDocument 的 Machine 实体集合。
/// 由调用方在 TaskProgress 任务内调用，进度区间假定 [0,100]。
/// 失败返回 false（文件不存在/格式不支持/读取失败）。
bool loadMachineFromFile(LcncDocument* doc,
                         const QString& filePath,
                         TaskProgress* progress,
                         const std::function<void(const QString& entry, const TopoDS_Shape& shape)>& onShapeLoaded = {});

/// 把当前机台 LcncDocument 按轴分组导出为 STEP（含 LCNC_AXIS_* 命名）。
/// 失败返回 false。
bool exportMachineToFile(LcncDocument* doc,
                         MachineKinematics* kin,
                         const QString& filePath);

} // namespace lcnc::cam::machine_io
