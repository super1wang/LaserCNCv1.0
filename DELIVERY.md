# LaserCNC 当前交付状态

更新日期：2026-08-26

产品版本：`1.6.1`（由 CMake 单一版本源生成）

## 当前结论

本次在 v1.6.0 碰撞安全基线上完成三项设置与坐标系修正，并对全部工作区修改做了文件级审阅：CAM 自动排序方向改由软件配置持有；Process 设置对话框按草稿事务提交或恢复；机台标定以绝对 AC 中心和控制器 XYZ 对应的绝对模拟 TCP 为目标，只牵引正确的机台运动子树。该结果可作为继续开发和自动化回归基线，但标定向导 GUI 操作、机台 STEP 回写和物理轴位置仍需在目标机台上验收。

## 本次交付

- CAM 配置：`toolpath.autoSortAxis` 保存到软件 `config/cam.toml`；工程包不再写入旧 `lastAutoSortAxis`，切换工程不会改变软件偏好或制造项目 dirty。
- Process 设置：Apply 保持当前页，OK 为“应用成功后关闭”，Cancel/Esc/标题栏关闭恢复最后提交状态；配置变更改用 TOML 结构比较。
- 绝对标定：A 面提供 Y/Z、C 面提供 X，先牵引整机对齐绝对 AC 中心；再按 Y→Y/X/Z、X→X/Z、Z→Z 的轴子树分别牵引模型，使所选刀嘴面 XYZ 对齐当前控制器 XYZ 所代表的绝对模拟 TCP。
- 坐标隔离：标定不改实时 A/C/XYZ，不移动固定 XY 横梁、工件或刀路；模拟锥头和离线仿真 TCP 不再依赖 STEP 初始放置。
- 回归覆盖：补充轴祖先链、零点/实时 XYZ TCP、工程包移除旧排序字段及 Process 取消恢复检查。

v1.6.0 碰撞安全基线继续保留：

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
| 日常 ACS+GTN Debug 构建 | v1.6.1 全量构建通过，`x64/ninja/Debug/LaserCNC.exe` 链接成功。 |
| 日常完整 CTest | v1.6.1 通过 41/41，147.69 秒。 |
| 真实 AC 转台索引回归 | 生成、QuaZip 打包、加载、50 万次查询、校验和破坏拒绝均通过；本轮单项 99.99 秒。 |
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
6. 本轮绝对标定尚未完成目标机台 GUI 拾取、STEP 自动回写重载和物理 A/C/XYZ 联动验收；自动化只证明坐标与拓扑契约。

自动化、SDK 仿真、GUI 和物理机是四类独立证据。完整审计见 [AUDIT.md](AUDIT.md)，专项指标见 [docs/collision_detection_todo.md](docs/collision_detection_todo.md)，剩余工作见 [todo.md](todo.md)。
