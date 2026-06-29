#include "modules/cam/workspace/machine_workspace.h"

#include "core/document/lcnc_document.h"
#include "core/kinematics/machine_kinematics.h"
#include "core/project/lcnc_project_manager.h"
#include "core/kernel/kernel.h"

namespace lcnc::cam {

namespace {
// 机台工作台用一个特殊 DocumentId — 这里不能直接用 LcncProjectManager 的 id 序列
// （会与未来的工件 doc 冲突），用一个高位常量代替：DocumentId 是 int，所以 -2
// 作为"工作台专属"标记够用（kInvalidDocumentId=-1）。
constexpr int kMachineWorkspaceDocId = -2;
} // namespace

MachineWorkspace::MachineWorkspace(QObject* parent)
    : QObject(parent)
{
    // LcncDocument 构造为 private，由 LcncProjectManager 代建。
    auto* pm = lcnc::Kernel::current().projectManager();
    if (pm) {
        m_document.reset(pm->createMachineDocument());
    }
    // 预触发生成 MachineKinematics（LcncDocument 的 lazy 创建），此后
    // kinematics() 直接返回该实例。
    if (m_document)
        (void)m_document->machineKinematics();
}

MachineWorkspace::~MachineWorkspace() = default;

LcncDocument*       MachineWorkspace::document()       { return m_document.get(); }
const LcncDocument* MachineWorkspace::document() const { return m_document.get(); }

MachineKinematics*       MachineWorkspace::kinematics()       { return m_document ? m_document->machineKinematics() : nullptr; }
const MachineKinematics* MachineWorkspace::kinematics() const { return m_document ? m_document->machineKinematics() : nullptr; }

void MachineWorkspace::setModelFilePath(const QString& path)
{
    if (m_modelFilePath == path)
        return;
    m_modelFilePath = path;
    emit machineModelChanged();
}

void MachineWorkspace::clearMachineGeometry()
{
    if (!m_document) return;
    m_document->clearEntityKind(LcncDocument::EntityKind::Machine);
    m_document->clearEntityKind(LcncDocument::EntityKind::Workpiece);
    if (auto* kin = m_document->machineKinematics())
        kin->clear();
    emit machineModelChanged();
}

} // namespace lcnc::cam
