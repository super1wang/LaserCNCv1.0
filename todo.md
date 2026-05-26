# LaserCNC Todo 与审阅记录

审阅日期：2026-05-08  
审阅重点：框架结构优化、项目运行稳定性、单项目三域数据边界、唯一 workspace view。

## 当前基线

- 单项目模式已经落地：`LcncProjectManager` 拥有 Workpiece、Machine、CAM 三份 `LcncDocument`。
- `GuiApplication` 只拥有一个 workspace `GuiDocument`。
- `GuiDocument` 已使用 `DocumentId + entry` 注册 AIS 对象，并支持 `rebuildDomain()` 局部刷新。
- CAD/CAM 显隐、选择和刷新路径已基本按 document/domain 限定。
- `LcncDocument` 已完成首轮瘦身：工件显示名和源文件路径迁入 `LcncProjectSession::workpiece()`。
- 阶段 1 稳定性收口已完成首轮实现：`.lcnc` 保存 manifest 时使用 session workpiece source path，工程包读写错误日志更明确，`GuiDocument` domain rebuild 增加 display registry 计数诊断。
- 运行时显示修复已完成：Workpiece 源模型直接进入公共视窗，安装仅平移源 Workpiece document；CAM 轮廓主体由 CAM document 的 AIS registry 唯一显示，`ToolpathRenderer` 不再创建第二份轮廓 AIS。
- `LaserCNC` Debug 构建已通过；当前仅见第三方对象缺 PDB 的非致命链接警告。

## 审阅发现的问题

| 优先级 | 问题 | 影响 | 建议 |
| --- | --- | --- | --- |
| P0（已完成） | `LcncProjectPackage::save()` 过去把 `manifest.sourceFilePath` 写成 `workpieceDocument.filePath()`。 | 已改为由 `LcncProjectManager` 传入 session manifest metadata，避免保存后工件源路径变成工程包路径。 | 后续在 package roundtrip 测试中持续校验 `session().workpiece().sourceFilePath`。 |
| P0（已完成） | 导入工件后曾存在源模型、挂载模型以及 CAM sparse/runtime 轮廓重复显示。 | 已取消 Machine document 工件副本：Workpiece 源 AIS 直接显示，安装只平移源数据；CAM 轮廓主体由 CAM document 唯一显示，`ToolpathRenderer` 仅保留引入线/法线/预览覆盖物。 | 手工 smoke 中继续验证 STEP 导入、节点显隐和刀路显隐的一致性。 |
| P0 | 缺少运行时 smoke 验证。 | 机台加载、工件导入、工程树勾选、CAM 轮廓和 `.lcnc` 打开保存仍主要依赖手测。 | 建立最小 smoke checklist 和可脚本化验证，覆盖三域显示组合与保存/打开。 |
| P1 | `LcncDocument` 仍持有 `MachineKinematics`、三份 domain tree 和 category group labels。 | document 仍承担部分 machine/domain 业务，后续会阻碍 MachineModelManager 独立。 | 第二轮瘦身：迁出 `MachineKinematics`，将 tree/cache 归入对应 domain manager 或 snapshot builder。 |
| P1 | `GuiDocument` 仍保留 `sourceDocument()` 和裸 entry 兼容 API。 | 新旧路径并存，后续修改显示逻辑时容易重新引入跨 document entry 冲突。 | 将 CAD/CAM/app 调用全部迁到 document/domain-aware API 后删除兼容 wrapper。 |
| P1 | `RenderingManager` 仍依赖 `GuiDocument::sourceDocument()`。 | 多域 workspace 下样式应用仍有兼容痕迹。 | 改为按 display registry 的 domain/document 分组应用样式。 |
| P1 | `.lcnc` package backend 依赖 PowerShell `Compress-Archive` / `Expand-Archive`。 | 对运行环境、错误恢复、编码和大文件稳定性不够可控。 | 本轮暂保留 PowerShell；阶段 4 先抽象 archive backend 和原子保存，后续再替换为 QuaZip。 |
| P1 | `CadModule`、`CamModule` 实现文件过重。 | I/O、显示同步、业务流程、任务回调混在 facade 中，维护风险高。 | 继续拆 service：WorkpieceImportService、WorkspaceDisplaySyncService、MachineModelManager、ToolpathWorkflowService。 |
| P2 | `GuiApplication` 保留 `s_instance` 单例守卫。 | 虽然没有公开 `instance()`，但仍是旧单例思路的残留。 | 在确认无外部依赖后改为普通对象守卫或移除静态状态，仅由 Kernel 注入访问。 |
| P2 | Process 目前以仿真控制器为主。 | 真实控制器接入、掉线恢复、错误状态和安全互锁不足。 | 强化 `IMotionController` 状态机、错误恢复、急停互锁和日志诊断。 |
| P2 | 第三方库和历史文档变更较多，工作树噪声大。 | 后续审阅 diff 容易混入无关文件。 | 建议建立 vendor 更新规则和文档归档规则，构建输出保持忽略。 |

## 下一阶段开发规划

### 阶段 1：运行稳定性收口（2026-05-09 已完成首轮实现）

- [x] 修复 `.lcnc` 保存 metadata：`manifest.sourceFilePath` 使用 session workpiece source path。
- [x] `LcncProjectPackage::save()` 新增 session-driven manifest 保存通道，并把实际写出的 manifest 回写到 `LcncProjectSession`。
- [x] 保留 PowerShell zip 后端，同时补充压缩、解压、manifest、XCAF 缺失等失败路径日志。
- [x] 为 `GuiDocument::rebuildDomain()` / `eraseDomain()` 增加 display registry 计数日志，便于确认 domain rebuild 不清空其它 domain。
- [x] 修复运行时重复显示：取消 Machine document 工件挂载副本，安装改为平移 Workpiece 源形体；CAM 轮廓主体改由 CAM document AIS 唯一显示。
- [x] VS Code CMake Tools 构建 `LaserCNC` 通过；仅保留既有 `vc143.pdb` 非致命链接警告。
- [ ] 运行时 smoke 仍需手工执行：启动、加载机台、导入 STEP、反向顺序加载、三域节点显隐、CAM 轮廓、保存/打开。

### 阶段 2：框架结构继续瘦身

- 从 `LcncDocument` 迁出 `MachineKinematics`，建立 `MachineModelManager` 或纳入 `MachineProjectState`/CAM service 管理。
- 把 `LcncDocument::EntityKind` 降级为 document-local storage tag，避免承载项目域所有权语义。
- 合并或迁移 `m_workpieceTree/m_machineTree/m_camTree`，让 ProjectExplorer snapshot 从 domain manager 或 document storage adapter 构建。
- 删除 `GuiDocument` 的裸 entry API 和 `sourceDocument()` 兼容路径。
- 将 `RenderingManager` 改为按 `ProjectDomain` 和 `DocumentId` 应用样式。

### 阶段 3：模块服务化

- CAD：拆出导入导出、建模流程、选择同步、显示刷新服务，减少 `CadModule` facade 体积。
- CAM：拆出机台模型管理、挂载管理、刀路 workflow、CAM runtime 同步服务。
- Process：完善控制器 contract，区分仿真控制器、真实控制器和 UI 状态同步。
- ProjectExplorer：推进真正 model-view 适配，节点 id 稳定化，减少 `QTreeWidgetItem*` 语义泄漏。

### 阶段 4：工程包和配置稳定性

- 抽象 zip backend，替换 PowerShell 实现。
- `.lcnc` 保存采用临时文件 + 原子替换；失败时保留旧包。
- 增加 package schema version 校验和向后兼容策略。
- 对 `CamConfig`、`ProcessSettings`、`AppSettings` 增加配置值范围校验和默认值恢复。

### 阶段 5：测试与诊断

- 增加核心算法单元测试：CAD primitive/boolean/transform、CAM face classifier、toolpath id reorder。
- 增加 project package roundtrip 测试：新建、导入、保存、打开、metadata 校验。
- 增加 view/display 集成测试钩子：统计每个 domain 的 AIS 数量和选择结果。
- 增加日志分层：启动、配置、project IO、display registry、machine kinematics、controller state。

## 手工 smoke 清单

- 启动程序后无崩溃，日志目录创建成功。
- 加载机台模型后 workspace 显示机台。
- 再打开 STEP/IGES/STL/BREP 工件后，机台不应短暂清空，且 Workpiece 源模型直接在公共视窗中显示。
- 反向顺序也成立：先打开工件，再加载机台。
- Machine 节点勾选只影响机台 AIS，不影响工件 AIS。
- Workpiece 节点勾选只影响 Workpiece 源 AIS，不影响 Machine/CAM domain。
- CAM 轮廓刷新不清空 Workpiece/Machine 显示，工程树隐藏轮廓后不应残留另一份轮廓。
- 保存 `.lcnc` 后重新打开，三域根节点、工件源路径、机台路径和 CAM runtime 状态符合预期。
- Process 执行 tab 独立可用，不出现在 ProjectExplorer 树中。
