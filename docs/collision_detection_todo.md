# 碰撞检测与安全域专项待办

更新日期：2026-08-21

上层审计与优先级：[AUDIT.md](../AUDIT.md)、[todo.md](../todo.md)

本文档记录碰撞检测、安全域缓存、空程规划和首段规划的后续工作。当前代码已经形成可阶段性交付的 C0 几何缓存与稀疏精确位姿证书，但尚未完成最终设计中的 C1～C3 稠密/分块安全域和两层路径搜索。除非本文件中的验收项通过，不得把当前实现描述为“任意姿态 O(1) 安全域规划”或“完整三级安全域”。

## 当前已实现基线

- CAM 拥有碰撞几何和 OCC 运算；Process 只提交无 OCC 的实时 APOS、读取不可变首段结果并原样执行已求解轴坐标。
- `ICamCollisionSafetyDomain` 为刀路、轮廓间空程和首段提供统一的无 OCC 路径校验接口。
- `CamTravelCollisionGeometryCache` 缓存独立碰撞 BRep、AABB、OBB、表面网格/BVH、轴绑定、安装变换和环境版本。
- `CollisionSafetyDomainCache` 按完整物理轴位姿、TCP/法向、运动阶段、碰撞源对和间隙缓存稀疏精确证书；只复用 `Safe`、`Warning`、`Collision`，不复用 `Pending`、`Indeterminate` 或失效环境。
- 机台、工件、刀具代理、轴构型、碰撞源或间隙变化通过 `environmentRevision` 使当前几何/证书失效；显示开关不参与失效。
- 全刀路校验会复用并填充稀疏证书；任意 APOS 首段查询在缓存未命中时执行 AABB/OBB、网格预筛和受全局锁保护的精确 OCC 距离复核。
- 手动首段使用控制器有符号绝对 Z 坐标；自动兼容路径按候选安全 Z、`SafeXY`、`SafeAC`、`Approach` 构造并离散校验。未加载机台模型时只跳过首段的机台碰撞源校验。
- 停止后再次开始会清空流程断点，重新读取 APOS 并重新规划首段；Process 不复用来源轮廓为 0 的旧首段过渡。

## 与最终设计的差距

| 层级 | 目标 | 当前状态 |
|---|---|---|
| C0 碰撞场景编译 | 私有网格/BVH、包围体、轴绑定、变换和版本指纹 | 部分完成；已有运行期内存缓存，尚无持久化、分块编译产物和完整刀具包络版本 |
| C1 机台固有安全域 | All-AC XYZ 场、固定 AC PoseField、机台禁区、Z 安全退回域 | 未实现；当前只有精确位姿稀疏证书和按 Z 候选逐条扫描 |
| C2 当前任务安全域 | Machine/Workpiece/Fixture 分场及保守 OR 合并 | 未实现；当前几何在同一环境缓存中组合，没有可独立更新的体素场、夹具场和 `validMask` |
| C3 首点与路径缓存 | GoalPose、GoalNavigation、ApproachCorridor、CurrentPose LRU、AC Sweep LRU | 未实现；当前首段在开始加工时按确定性三阶段路线生成并扫描 |
| S1 安全退回 | 当前 AC PoseField 上 DDA/supercover 找到最近安全退回域 | 未实现；当前按物理 Z 方向枚举候选绝对坐标 |
| S2 过渡换姿 | 两层 A*/Theta* 选择 XYZ 路径和 AC 换姿门户 | 未实现；当前固定为安全 Z 上的 XY 后 AC 顺序 |
| S3 接近首点 | GoalNavigation 到入口，再沿认证 ApproachCorridor 接近 | 部分完成；保留首点既定 IK 分支和 Approach 边界，但没有导航场/通道证书 |

## P0：安全语义和失败关闭

- [ ] 将自动规划开关拆成 `safetyDomainPlanningEnabled` 与 `postCollisionValidationEnabled`。自动模式必须依赖完整、有效且未过期的安全域；精确复核开关只能控制最终追加复核，不能绕过安全域。
- [ ] 为所有场增加 `validMask`/`Unknown`；未知、构建中、取消、版本不匹配或精确复核失败在真实加工中一律阻止执行。
- [ ] 定义不可变 `SafetyDomainKey`，至少包含机台几何、运动学拓扑与轴方向、限位、碰撞源配置、刀具/喷嘴包络、工件/夹具装夹、间隙/体素规格、刀路顺序/首点分支和算法版本。
- [ ] 增加 `CertifiedMotionEdge`：保存阶段、已求解路点、最小间隙、阻塞区间、碰撞源摘要、域键和证书状态；Process 执行插补必须与该边完全一致。
- [ ] 将当前离散点校验升级为连续段保守细分或扫掠校验，覆盖采样点之间的碰撞；边界区域必须回退到现有精确 OCC 算法。
- [ ] 给角度分桶和体素离散加入保守膨胀：物理间隙、XYZ 半对角线误差、A/C 分桶误差、网格误差、装配/标定误差和控制器跟随误差全部计入。

## P1：C1 机台固有安全域

- [ ] 实现分块、粗细两级的三维 `MachineAllAcSafeField`/`MachineAllAcForbiddenXYZ`。它表示允许的全部 AC 联合区域内始终安全的 XYZ 交集/禁区并集，不是完整 `X×Y×Z×A×C` 五维稠密数组。
- [ ] 实现 `MachinePoseField(A,C)`，按联合 AC 单元和实际 IK 分支分桶并使用 LRU；禁止把独立 A 范围与 C 范围做笛卡尔积。
- [ ] 实现 `MachineSafeRetractDomain`，按当前 AC、刀具包络和机台部件场提供可认证的 Z 退回柱/区域，而不是固定数值。
- [ ] 按运动部件拆分并保守合并：切割头/Z 滑枕三维场、X 横梁二维或分层场、Y 轴约束、AC 转台联合姿态场。
- [ ] 机台模型或稳定的头部/刀具包络加载后在 CAM 后台构建 C1；显示/隐藏机台不得使其失效。

## P1：C2 当前任务安全域

- [ ] 为工件、夹具、装夹变换和刀具包络建立独立分块场，使用 `JobBlocked = MachineBlocked OR WorkpieceBlocked OR FixtureBlocked` 组合，避免工件变化后重扫整个机台。
- [ ] 每个 tile 至少保存 `blockedMask`、`validMask`、量化 `clearance` 和可选碰撞源摘要；支持并发只读快照和后台增量替换。
- [ ] 工件/夹具加载或安装姿态稳定后在 CAM 阶段构建/更新 C2；构建取消或项目切换不得发布半成品。
- [ ] 明确夹具数据来源、轴挂接和版本指纹；当前实现只有机台与工件，夹具安全域尚无正式数据契约。

## P1：C3 首点和在线路径缓存

- [ ] 刀路生成、排序或首点 IK 分支改变后构建 `GoalPoseField`、反向 `GoalNavigationField` 和完整 `GoalApproachCorridor`。
- [ ] 实现 `CurrentPoseField` LRU；实时 APOS 只触发查询或缺失姿态分桶构建，不使 C1/C2 静态缓存整体失效。
- [ ] 实现 `AcSweepField` LRU。键必须包含起止 A/C、全部 waypoints、插补模式、C±360 解绕方式和运动曲线指纹，不能只按端点缓存。
- [ ] 对刀路、轮廓间空程和首段统一复用 C0～C3 查询与证书构建接口，移除各调用方重复的路径采样策略。

## P1：S1～S3 规划算法

- [ ] 在当前 AC PoseField 上实现 Z 方向 DDA/supercover，寻找最近安全退回门户；把世界安全点通过运动学反算为控制器有符号绝对 Z 坐标。
- [ ] 实现直线 DDA 快速通道、Theta* 减折点和 A* 绕障，并对每条边返回最小间隙与阻塞区间。
- [ ] 实现当前 AC 层与目标 AC 层的两层图搜索：同层固定 AC 移动 XYZ，跨层仅允许在对应 `AcSweepField` 安全的 XYZ 门户执行实际 AC 曲线。
- [ ] S3 使用 GoalNavigation 到接近入口并沿 GoalApproachCorridor 到达 CAM 已提交首点；不得重新选择“等价”AC 分支。
- [ ] 仅在体素/图搜索仍无路时引入稀疏五维 RRT-Connect 兜底，并对结果做连续碰撞校验；该兜底不作为近期阶段完成条件。
- [ ] 将 `CamModule::planInitialApproach()` 中的状态捕获、候选生成、离散化和碰撞扫描拆入 `InitialApproachPlanningService`；保留 `ICamInitialApproachPlanner` 作为 Process 公共边界。

## P2：缓存工程化

- [ ] 实现 tile 压缩、内存预算、LRU 淘汰、统计和背压，避免为每个 AC 姿态常驻完整三维场。
- [ ] 评估 C0/C1 按机台指纹持久化；磁盘格式必须带 schema、算法版本、单位、坐标系、体素规格和完整校验和，加载失败时安全回退到重建。
- [ ] 增加 TaskManager 取消、项目/机台切换的代际发布和模块停止等待；禁止工作线程持有文档、GUI 或可变 OCC 对象。
- [ ] 日志统一记录 C0～C3 构建/命中/失效原因、tile 数、内存、Unknown 比例、DDA/Theta*/A* 扩展节点、精确回退次数和耗时。

## 验收门禁

- [ ] 覆盖 Z 正反方向和控制器有符号绝对坐标；验证 S1 的世界高度判定与最终轴指令一致。
- [ ] 覆盖 A/C 联合安全区、A 先动/C 先动/同步运动以及 C、C+360、C-360 的不同扫掠体。
- [ ] 覆盖首点上方、转台内部、机台禁区、工件/夹具局部障碍、Unknown、过期缓存和任务取消的失败关闭。
- [ ] 随机抽取安全域判定为安全的位姿/边，使用现有精确 OCC 碰撞算法反向验证；任何假安全都视为发布阻断。
- [ ] 验证停止后从头运行会读取新 APOS、重查 CurrentPose/Sweep，并且不会复用旧首段证书。
- [ ] 验证 CAM 证书中的全部路点、阶段和 AC 插补顺序与 ACS/GTN/PureSimulation 实际消费一致。
- [ ] 分别记录自动化、GUI、SDK 模拟器和无激光低速实体机证据；自动化通过不能替代夹具、限位、安全 IO、急停恢复和真实跟随误差验收。
