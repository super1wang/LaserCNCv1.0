#pragma once

#include "core/project/project_types.h"

#include <QObject>
#include <QString>
#include <QStringList>

#include <memory>

class LcncDocument;
class MachineKinematics;

namespace lcnc::cam {

/**
 * @brief CAM 模块拥有的"机台工作台"——把机台几何 + 运动学 + 工件挂载从工程包里抽出来。
 *
 * 设计要点
 * ---------
 *  - **拥有机台 LcncDocument**：本工作台是机台几何的唯一所有者（参考资产）。
 *    内部以 XCAF 容器存储 STEP/STL/BREP 导入结果；并把该 doc 的一个 **非拥有引用**
 *    登记到 LcncProjectManager（attachMachineDocument），仅供视图/选择的域路由
 *    （`gd->domainForDocument` / `rebuildDomain(Machine, ...)`）识别机台文档。
 *  - **持有 MachineKinematics**：当前仍由机台 LcncDocument 持有，本工作台提供快捷访问；
 *    运动学描述当前机床构型和物理旋转中心，不随参考模型加载/卸载而重置。
 *  - **机台模型路径**：全局来自 `CamConfig::machineModelPath()`，**不随 .lcnc 项目存档**；
 *    `loadModel(path)` 在用户切换机台或 CamModule::init 时调用一次。
 *  - **独立于工程**：机台几何不属于工程数据 —— 不进 .lcnc、不计入工程脏标记、
 *    新建/打开工程时不被清空（见 LcncProjectManager::resetProjectDocuments）。
 *  - **生命周期**：由 Kernel 持有，晚于业务模块创建、早于 ProjectManager 销毁。
 */
class MachineWorkspace : public QObject
{
    Q_OBJECT
public:
    explicit MachineWorkspace(QObject* parent = nullptr);
    ~MachineWorkspace() override;

    /// 持有的机台 LcncDocument；构造时即创建，存在期内不为空。
    LcncDocument*       document();
    const LcncDocument* document() const;

    /// 运动学对象 —— 当前仍由 LcncDocument 持有，工作台仅提供快捷访问。
    /// 后续阶段计划将 MachineKinematics 值持有在本类中。
    MachineKinematics*       kinematics();
    const MachineKinematics* kinematics() const;

    /// 当前已加载机台模型文件路径（来自 CamConfig，全局共享）；可为空。
    QString modelFilePath() const { return m_modelFilePath; }
    void    setModelFilePath(const QString& path);

    /// 仅清空机台参考几何；保留工件、刀路与 MachineKinematics 的轴定义。
    void clearMachineGeometry();

signals:
    /// 机台模型加载/卸载完成，UI 与渲染层据此重建。
    void machineModelChanged();

private:
    std::unique_ptr<LcncDocument> m_document;
    QString                       m_modelFilePath;
};

} // namespace lcnc::cam
