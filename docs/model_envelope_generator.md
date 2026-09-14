# 外部模型包络生成器

更新日期：2026-08-28

## 结论

当前 `model/精简ac转台.stp` 的生产后端是 **CGAL Alpha Wrap**。项目已经取得所需 CGAL 许可，允许进入产品接入阶段；实现仍保持独立构建、独立进程和文件协议边界，不成为 `LaserCNC`、`lcnc_core`、CAD/CAM 模块或安全包运行库的链接依赖。

OpenVDB 12.0.1 的稳定接口链可显著缩短离线生成时间，但当前真实模型的 C 轴体含有其 `meshToLevelSet` 无法可靠实体化的开放或混合面，因此不能作为碰撞模型生产后端。后续若升级为支持任意 polygon soup 的 shrink-wrap 实现，仍必须重新通过同一有效性门禁，不能沿用本次性能结论直接切换。

## 目标和边界

包络生成只保留碰撞需要的保守外形，不保留原始 B-Rep 拓扑、孔洞、内部空腔、紧固件层级或装配约束。机台不能整体合成一个固定外壳，必须按运动刚体分别处理：

```text
STEP/XCAF LCNC_AXIS_* 分组
    -> BASE / X / Y / Z / A / C 三角网格
    -> 每个刚体独立生成保守闭合包络
    -> 有效性门禁
    -> PLY/JSON 中立资产
    -> .lmsi schema 6 持久碰撞网格
    -> .lmsp v2 强绑定安全包
```

分轴是硬约束。A、C 等旋转体若与 BASE 合并，静态姿态看似正确，但运动后会产生错误的碰撞几何和过度阻塞。

通用工具位于 CAD 几何处理边界，Process 不得包含或消费 OCC 类型。CAM/安全包只消费经校验的、无 OCC 类型的索引三角网格。

## 两个隔离后端

### CGAL Alpha Wrap

- 目标：`lcnc_model_envelope_cgal`
- 构建：仅由 `tools/model_envelope/CMakeLists.txt` 独立生成
- 算法：CGAL 6.0.1 Alpha Wrap，EPICK 稳健谓词内核
- 标准参数：OCC deflection 2.0 mm、angle 0.35 rad、alpha 10.0 mm、offset 2.5 mm
- 展示型分轴参数：Z/A/B/C 使用 alpha 3.0 mm、offset 0.5 mm；BASE/X/Y 仍使用标准参数
- 输出：每轴体 binary little-endian PLY、JSON 指标和校验结果；按需输出保留 `LCNC_AXIS_*` 名称的 AP242 tessellated STEP
- 部署：独立运行目录；主程序和主库不链接 CGAL/GMP/MPFR

Alpha Wrap 对任意三角汤更适合当前机台输入，能够直接给出闭合二流形包络。它的生成速度不是在线路径指标；生成完成后只把包络三角网格交给碰撞索引。

`alpha` 是包络可跨越的局部特征尺度，`offset` 是包络相对源外表面的保守外扩量。展示型导出只对 Z 和旋转轴缩小这两个阈值：大于约 3 mm 的外观特征尽量保留，内腔以及小于阈值的窄缝、小孔和螺丝孔仍被跨越或封闭。它没有恢复原始内部拓扑，也不会把螺栓等内部零件重新带入碰撞模型。

AP242 STEP 是每个轴体一个带三角化的 tessellated face，不重新构造数十万 B-Rep 三角面，也不承诺可编辑的解析曲面、孔或原装配拓扑。它用于碰撞模型交换、重新加载和可视化；精确加工几何仍使用原始 STEP。

### OpenVDB

- 目标：`lcnc_model_envelope_openvdb`
- 构建：仅由 `tools/model_envelope/CMakeLists.txt` 独立生成
- 版本：OpenVDB 12.0.1，Apache-2.0
- 稳定接口链：`meshToLevelSet -> extractEnclosedRegion -> topologyToLevelSet -> VolumeToMesh`
- 标准参数：voxel 2.0 mm、offset 2.5 mm、closing steps 2、adaptivity 0.01
- 部署：独立运行目录，避免与主程序的 OCCT/TBB 运行时闭包混用

该链能够填充闭合空腔并做体素闭运算，但不是任意 polygon soup shrink-wrap。对开放/混合面必须失败关闭，不能把“输出了闭合网格”等同于“完整包住了源模型”。

## 真实模型基准

基准目标 `lcnc_model_envelope_backend_benchmark_test` 只注册在独立子工程中。推荐使用主工程约定内的独立生成树：

```powershell
cmake -S tools/model_envelope -B build-cmake/model-envelope `
  -G "Ninja Multi-Config"
cmake --build build-cmake/model-envelope --config Release --parallel 16
ctest --test-dir build-cmake/model-envelope -C Release --output-on-failure -V
```

根 `CMakeLists.txt` 不声明 CGAL/OpenVDB 包、目标或开关，因此独立子工程的 vcpkg Boost/Eigen/TBB 不参与 LaserCNC/Coal 的依赖解析。

测试输入 SHA-256 为 `3CFC60AD01963E2DE72C7AF63C5FDC06A5EC761F44C8261A132ED635B38B85AD`，文件大小为 65,862,437 bytes。两后端使用相同 STEP 导入、相同串行 OCC 三角化和相同六轴体分组。包络阶段单独计时。

门禁顺序：

1. 必须识别且只输出 `BASE/X/Y/Z/A/C` 六个刚体；
2. 每个输出必须非空、有限、闭合、二流形；
3. 每个刚体最多抽样 20,000 个源顶点，全部必须位于包络内或表面上；
4. 淘汰未通过者后，才按包络耗时 50%、峰值私有内存 30%、输出三角面 20% 评分。

2026-08-28 Release 实测：

| 后端 | 包络耗时 | 总耗时 | 峰值私有内存 | 输出三角面 | 有效性 |
|---|---:|---:|---:|---:|---|
| CGAL Alpha Wrap | 15.421 s | 27.220 s | 494,039,040 B | 196,576 | 六轴体通过 |
| OpenVDB | 3.283 s | 18.275 s | 616,689,664 B | 1,695,508 | C 轴失败 |

OpenVDB C 轴抽样 6,196 个源顶点，其中 221 个在包络外，最深漏包约 6.721 mm。将 adaptivity 调到 0.2 或 0.5 仍然失败，因此这不是单纯的输出网格密度问题。CGAL 被选中不是因为它生成更快，而是因为它是唯一通过碰撞保守性门禁的后端，同时输出面数约为 OpenVDB 的 11.6%。

基准汇总保存在构建树的 `model_envelope_backend_comparison.json`。这是算法选型证据，不等同于 GUI、整机运动仿真或物理机验收。

## 已完成的三阶段接入

### 第一阶段：CAD 外部生成服务

CAD facade 已提供通用包络生成服务：创建暂存目录、启动授权 CGAL 进程、轮询取消、校验输出，并按清单 SHA-256 发布到 `generations/<sha256>/`。最后使用 `QSaveFile` 原子更新 `current.json`。工具异常、取消、缺轴、输出缺失或校验失败时不会替换当前资产。

### 第二阶段：中立包络资产

`lcnc.model-envelope/v1` 清单记录：

- schema 和生成器版本；
- 源模型 SHA-256；
- BASE/X/Y/Z/A/C 轴名、父轴和局部坐标约定；
- 每体 PLY 路径、顶点/三角面数和 SHA-256；
- alpha、offset、三角化精度和完整校验指标；
- `collision_conservative=true` 的门禁结论。

加载器拒绝绝对路径和 `..` 路径、重复轴/网格路径、非有限参数、非保守结果、源模型哈希不符、网格哈希不符，以及超过资源上限或结构异常的 binary little-endian PLY。清单和 PLY 不包含 OCC 类型，可由 CAD、CAM 和离线安全工具共同消费。

### 第三阶段：安全索引与安全包

原始 STEP 继续作为显示、重建、精确离线计算和源指纹；包络 PLY 直接转换为现有 `SurfaceTriangleSoup`/`SurfaceCollisionModel`，其 AABB 和 Surface-BVH 作为 `.lmsi` schema 6 的认证与持久碰撞几何。绑定包络时禁止原 STEP 叶级 BVH 为更大的外扩体证明分离。运行时从索引重建内部/Coal BVH，不再为了在线碰撞重新三角化 STEP。

`.lmsp` v2 同时保存原始模型、唯一 `.lmsi`、包络清单和六个轴体 PLY。包键绑定原始模型哈希、索引哈希、包络清单哈希以及由轴名、父轴、相对路径和网格哈希组成的 mesh-set 指纹。任一不匹配都会使资产加载失败；旧 `.lmsp` v1 仍可读取，但新的一键生成流程固定产出 v2。

CAM 的“生成安全包”任务先调用 CAD facade 完成 0–20% 包络阶段，再调用安全索引工具完成 20–100% 编译与封装。若输入已经是 `.lmsp`，先校验并提取其原始模型，再生成新包。CGAL 工具解析顺序为环境变量 `LCNC_MODEL_ENVELOPE_CGAL`、主程序同目录、仓库独立工具输出目录。

## 主动包络导出入口

CAM 页“机台”Ribbon 现在提供两条主动入口：

1. “标记轴系”：打开原有轴归属/旋转中心对话框；勾选“应用轴标记后生成包络并导出精简 STEP”，确认后选择输出位置；
2. “生成包络”：对当前已经打开并完成标轴的机台直接选择输出 STEP。

两条入口都先把内存中的当前机台文档按 `LCNC_AXIS_*` 分组写入临时 STEP，再交给独立 CGAL 进程。因此尚未保存回原文件的轴归属也会生效。只要有一个机台零件未分配轴，或者轴名不属于当前外部协议支持的 `BASE/X/Y/Z/A/B/C`，任务就失败关闭，不会静默丢弃该零件。

生成过程运行在 TaskManager 后台任务中，模块停止时参与统一取消/等待。CAD 服务先校验 PLY、清单、源指纹及可选 STEP 指纹，再用 `QSaveFile` 原子发布用户指定的 `.stp/.step`；失败或取消不会用半成品覆盖原文件。成功输出可再次由机台加载流程识别 `LCNC_AXIS_*` 并恢复轴归属。

主动 STEP 导出采用分轴展示型配置：BASE/X/Y 保持 alpha 10.0 mm、offset 2.5 mm，以控制整机体积和面数；Z/A/B/C 自动提高到 alpha 3.0 mm、offset 0.5 mm，以保留主轴箱、转台和回转体的可识别轮廓。安全包生成仍统一使用标准碰撞配置，避免把展示精度的额外三角面带入在线碰撞数据。

真实 AC 转台模型的展示型回归结果如下；两档都通过闭合、二流形和源模型包容性门禁，平衡档作为产品默认：

| 配置 | Z/A/C 三角面 | 总三角面 | 包络耗时 | 总耗时 | 表面外扩均值 |
|---|---:|---:|---:|---:|---:|
| 高精度 2.0 / 0.35 mm | 242,974 | 432,212 | 29.343 s | 45.738 s | 约 0.35 mm |
| 平衡 3.0 / 0.5 mm | 97,236 | 286,474 | 21.250 s | 36.208 s | 约 0.50 mm |

如果分轴 Alpha Wrap 仍不能满足最终展示要求，再进入 OCC 混合路线：保留选定轴体的原始外表面并执行特征识别/补洞。但该路线必须额外解决装配内件剔除、内外壳判定和失败关闭，不能直接用原始 OCC 复合体替换包络，否则会重新引入当前功能刻意删除的内部拓扑。

机台加载默认仍显示原始 CAD。需要快速显示或无 CAD 运行环境时，可显式加载包络 PLY，但不得因此改变轴体归属、运动学指纹或碰撞安全资格。

## 许可证与部署边界

CGAL 许可已经取得。独立工具边界仍保留，以隔离依赖、升级节奏和运行时闭包：

- 主程序不链接 CGAL/GMP/MPFR；
- 外部工具与主程序只通过命令行、PLY 和 JSON 通信；
- 工具应作为独立运行时闭包构建和部署，可用 `LCNC_MODEL_ENVELOPE_CGAL` 指定已授权版本；
- 发布包仍需携带与已购许可证一致的版本和归属记录。

## 第三阶段后的验收边界

- 视觉检查六个 PLY 与原始模型的叠加，重点检查 C 轴和大开孔；
- 对源三角形面内采样，而不仅是源顶点采样；
- 自相交检查和组件数量策略；
- GUI、整机全姿态仿真、低速实机和安全系统验收。

第三阶段自动化与真实模型回归证明的是资产生成、哈希绑定、篡改拒绝、索引编译和软件碰撞数据链正确；不等同于 GUI 操作验收、全姿态覆盖或物理机安全批准。
