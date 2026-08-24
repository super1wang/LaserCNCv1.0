# LaserCNC 当前交付状态

更新日期：2026-08-24

产品版本：`1.6.0`（由 CMake 单一版本源生成）

## 当前结论

本次完成固定机台安全包、工件 Job Overlay、Surface-BVH/Coal 保守回退、连续运动证书和 Process 许可证的软件闭环，并对全部工作区修改做了文件级审阅。真实加工只消费与包键、环境代际和运动端点一致的不可变证书；缺失、构建中、失效、BoundaryUnknown 或不完整状态失败关闭。该结果可作为生产架构基线，但正式夹具、扩大 exact 审计、GUI/长稳和物理设备验证仍是实体机发布门禁。

## 本次交付

- `.lmsp`：QuaZip 原子打包机台模型、唯一 `.lmsi` 和多层 SHA-256/运行时配置指纹；模型或必要机台配置变化使索引失效。
- `.lmsi`：多级稀疏细化、断点续建、热 APOS 反馈、叶级 BVH 和 schema 5 持久表面数据；混合部件的完整距离 BVH 与封闭实体包含 BVH 分离。
- Job Overlay：工件作为唯一运行时几何变量后台构建局部保守场，发布不可变快照并按环境代际失效。
- 统一查询：Rapid、LeadIn、Cutting、Traverse 和首刀规划统一生成连续边证书；自动生成不再重复逐节点 OCCT，显式诊断仍保留 OCCT 审计后端。
- Process：真实加工 O(1) 校验证书；相对/绝对点动、预设位置和流程运动申请固定许可证，连续点动周期续签。
- 显示/仿真：模拟锥头和喷嘴只用于示意；空程按 Safe、Pending、BoundaryUnknown/Warning、Collision 分桶显示。
- 工程审查补强：修复重建期碰撞意图被错误折叠、混合几何包含漏判、同步多轴模式混用以及遗留 workpiece-proxy 诊断分支。

## 验证

| 验证 | 结果 |
| --- | --- |
| `git diff --check` | 通过；仅有 Git 行尾转换提示。 |
| `scripts/check_architecture.ps1 -Root .` | 通过。 |
| 日常 ACS+GTN Debug 构建 | 通过，`x64/ninja/Debug/LaserCNC.exe` 链接成功。 |
| 日常完整 CTest | 41/41 通过，167.58 秒。 |
| 真实 AC 转台索引回归 | 生成、QuaZip 打包、加载、50 万次查询、校验和破坏拒绝均通过；单项 104.12 秒。 |
| 混合几何回归 | 开放面保留、实体内部包含及 schema 5 持久化往返均通过。 |
| ACS+GTN Release | 构建通过；Surface-BVH、Coal、运动证书契约和 Process 失败关闭 4/4 通过，5.51 秒。 |
| ASan | 全量构建通过；TaskManager、碰撞、证书、Process 安全和快照并发关键回归 6/6 通过，15.94 秒。 |
| real-laser 变体 | 构建/链接通过；该结果只证明编译边界，不是物理激光器放行。 |

## 已知边界

1. 任意未规划 APOS 的 `.lmsi + Coal` 仍适合后台预测而非硬实时；硬实时边界是预先构建的 O(1) 证书/许可证。
2. 正式夹具模型、挂接关系和指纹 DTO 尚未接入 Job Overlay。
3. 自动绕障/重规划尚未实现；碰撞和 Unknown 会安全阻断，不会改写 CAM 权威路径。
4. 点动许可证仍需实体机量化 APOS 新鲜度、跟随误差和最坏制动距离。
5. 未完成 GUI 人工验收、8 小时资源趋势、Application Verifier 或真实 ACS/GTN/激光/IO 低速加工验证。

自动化、SDK 仿真、GUI 和物理机是四类独立证据。完整审计见 [AUDIT.md](AUDIT.md)，专项指标见 [docs/collision_detection_todo.md](docs/collision_detection_todo.md)，剩余工作见 [todo.md](todo.md)。
