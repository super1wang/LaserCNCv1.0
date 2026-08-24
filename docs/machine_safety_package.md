# 机台安全包（`.lmsp`）

更新日期：2026-08-24

`.lmsp` 是 QuaZip 归档格式，用于把机台模型、唯一的 `.lmsi` 安全索引和强绑定清单作为一个不可变机台资产交付。它不是工程数据，不写入 `.lcnc`，生命周期跟随全局 `MachineWorkspace`。

## 包内布局

```text
machine_safety.toml
machine/
  machine.step               # 也可保留 stp/stl/brep 后缀
safety/
  machine.lmsi               # 整个归档只能存在这一个 .lmsi
```

`machine_safety.toml` 保存：

- schema、格式版本、软件版本和创建时间；
- 模型路径和安全索引路径；
- 模型文件 SHA-256、索引文件 SHA-256；
- `.lmsi` 内部内容指纹和安全配置指纹；
- 运行时机台运动学和机台零件归属指纹；
- 由上述字段合成的包键。

加载时先枚举 ZIP 条目并拒绝绝对路径、`..` 路径、重复清单或多个 `.lmsi`，随后解压到受控临时目录并逐层验证全部指纹。嵌入模型的哈希还必须与 `.lmsi::sourceSha256` 一致。因此修改归档中的模型、替换索引、修改清单或加入第二个索引都会使包失效。

## 失效与执行资格

- 资源和运行时配置全部匹配时，包内索引才会激活。
- 后台重建期间自动关闭全局碰撞检测；新包只在完整生成并通过加载校验后发布，随后由操作者再次显式启用。
- 模型、轴类型/方向/原点/限位/父子关系或机台零件轴归属改变时，立即释放已加载索引。指纹使用包内 STEP 零件顺序与对应轴名，不使用重载后会变化的 XCAF 文档条目号。
- 归档、清单、资源或任一指纹校验失败时，包不具备执行资格。

加载一个机台运动学或零件归属不匹配的 `.lmsp` 时，只提交其模型供查看和重新构建，不激活包内 `.lmsi`。完整切割头必须是 Z 轴机台实体并由模型哈希绑定；模拟锥头/喷嘴代理只用于显示。工件、碰撞开关、流程阻断策略和在线证书版本变化不使作为机器型号资产的机台包失效。

未加载机台、只加载 STEP 但尚未生成安全包、安全包失效或安全包正在构建时，全局碰撞检测开关不可激活。开关关闭时不要求机台包、Job Overlay 或连续运动证书，非碰撞检测加工流程仍可使用。

## 生成与加载

离线工具兼容原始索引和安全包输出：

```powershell
lcnc_machine_safety_index.exe `
  --machine "model/精简ac转台.stp" `
  --package-output "model/精简ac转台.lmsp" `
  --runtime-configuration-sha256 <64位十六进制指纹>
```

应用 Ribbon 的“生成安全包”调用同一个独立工具。输出先写临时归档，完成后原子替换；成功后复制原 STEP 机台 profile，将配置路径自动切换为 `.lmsp` 并重新加载。若当前路径已经是 `.lmsp`，则从包内模型重建并原位更新。

生产档还会传入：

```powershell
--leaf-bvh on --surface-bvh off --dependency-cache on `
--refine-levels 1 --refine-threads 8 --exact-budget 1 --audit-safe 32 `
--checkpoint <输出包>.checkpoint.lmsi
```

真实 AC 转台模型的 0.5 mm OCCT 表面三角化超过 15 分钟，因此一键档默认使用保守叶级 BVH；
`--surface-bvh on` 保留给显式高精离线档。它只提高 Unknown 消解率，不改变失败关闭语义。

`.lmsi` schema 5 支持最多三级递归稀疏细化。二、三级必须通过 `--hot-apos-file`
限定到 AC 轴附近或真实路径产生的 BoundaryUnknown 热区。基网格及每个完整级别都会原子写入
检查点；再次执行默认验证模型哈希、轴网格、碰撞对和间隙后续建，`--no-resume` 强制重建。

schema 5 可保存完整碰撞三角面片、网格误差、闭合体标志和混合部件的独立封闭实体包含网格。
加载后重建内部/Coal BVH，避免绑定 Coal 私有节点 ABI，同时避免重复 STEP 三角化。schema 3/4
若含旧持久网格会被拒绝并要求重建，防止开放面干扰实体包含判定。

机台加载入口同时接受 STEP/STL/BREP 和 `.lmsp`。`autoLoadMachineModel=true` 时，配置中保存的 `.lmsp` 会沿用现有启动自动加载流程。

## 碰撞查询边界

有效包已接入 `MachineOnly` 快速通道：连续边先使用 `.lmsi` supercover 证书；全圈旋转轴将物理
连续角映射到规范圈并在周期缝两侧分别认证。`BoundaryUnknown` 只允许进入有预算的
Surface-BVH/Coal，生产连续证书的 OCCT exact 预算固定为 0。
`FullEnvironment` 使用严格与关系：机台 LMSI 和工件局部场必须同时证明安全，任一 Unknown 都不能
被另一侧覆盖。工件是当前唯一 Job 几何变量，有效包与工件同时存在后即后台构建 Overlay。

规划完成后，每条 Rapid、LeadIn、Cutting 和 Traverse 边都持有与包键、环境代际和端点绑定的
连续运动证书。真实 Process 只消费完整证书，缺失、Unknown、过期或构建中状态一律阻止执行。
手动固定运动和连续点动使用同一查询器签发短时许可证。

`.lmsi + Coal + Motion Certificate` 是唯一软件碰撞判定入口，不替代轴限位、驱动保护、STO、急停、安全 IO、门禁和跟随误差监视。
