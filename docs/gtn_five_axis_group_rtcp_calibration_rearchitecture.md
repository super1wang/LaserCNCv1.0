# GTN 新五轴 Group、RTCP 与实测标定重构方案

状态：软件重构已实施；实体控制卡与实机验收待完成

日期：2026-09-02
适用工程：LaserCNC 现有五轴 CAM、Process 与固高 GSN/GTN 控制链路

### 2026-09-09 坐标契约修订

GTN MCS 模型几何统一采用 Z 向上右手机床零位参考系，不能等同于
控制器轴反馈坐标基。方向保持配置世界方向，配置旋转中心转换到世界系后下发。
标定记录 schema 2 使用该契约；schema 1 仅保留作证据，禁止激活或驱动 Group。
完整原因、迁移步骤及验证见 [v11 交付说明](versions/2026-09-09-gtn-right-handed-mcs-v11.md)。

### 2026-09-10 RTCP 指令参考系修正

上述模型几何约定不意味着“当前姿态世界 TCP”可以直接当作 RTCP 指令点。
双转台输入需先去除工件载体当前运动：`p_mcs = T_home * inverse(T_current) * p_world`。
CAM 同时保留世界 TCP（碰撞/显示）和零位参考 TCP（GTN 指令），Process 仅透传；
控制器变换结果仍须对比独立 CAM 轴解，不通过改变阈值或轴方向消除超差。
见 [v11.3 修复说明](versions/2026-09-10-gtn-rtcp-reference-v11.3.md)。

### 2026-09-11 轮廓间快移与正常 RTCP 加工

连续 IK 后，空程世界 TCP/法线必须从同一工件局部点按已求解姿态重建，不能保留规划姿态下的世界值。
缓存复用同时恢复后继轮廓的连续轴解，碰撞证明仍使用最新异步校验结果。
按用户要求取消构型派生 RTCP 的专用试运动限速、加速度上限及强制关光/关气分支；
改用正常刀具参数、机床轴限制和数字量工艺时序。保留显式来源许可、记录指纹及失配停机，
`ConfigurationDerived` 仍不等同于 `MachineVerified`。
本节取代历史试运动策略，详见 [v11.4 交付说明](versions/2026-09-11-gtn-rtcp-transition-v11.4.md)。

## 0. 2026-09-01 实施状态

本轮已完成以下软件范围：

- 增加不可变物理标定记录、样本/记录哈希、原子保存与活动标定切换；
- 增加父/子旋转轴多姿态轴线拟合、覆盖率和残差质量门槛及纯算法测试；
- 增加独立物理五轴标定向导，支持姿态模板、CSV 导入、当前控制器 APOS 辅助填充、人工实测点录入、拟合、候选保存和激活；
- CAM 导出同时携带 MCS TCP 姿态和软件 IK 预测轴坐标；
- GTN 增加 `Axis Group + CommandList` 执行链，RTCP 关闭时下发软件求解轴坐标，RTCP 开启时下发 MCS TCP 与旋转轴姿态；
- 激光、吹气和工艺延时写入同一 CommandList，避免运动与工艺输出分裂；
- RTCP 模式用 `GTN_GroupPosTransform` 抽检控制器转换轴与 CAM 预测轴，超差、奇异或缺少已实机验证标定时拒绝加工；
- Process 预检增加五轴快照、TCP 有效性和 `MachineVerified` 标定门槛；
- Process 设置增加 Group/List、RTCP、前瞻、平滑、旋转轴速度参考和转换一致性参数。

尚未完成、不得视为生产验收的范围：

- 实体卡 DLL/MC/DSP/固件、五轴及 RTCP 授权能力读取和现场确认；
- 自动驱动机台完成姿态序列和自动测量；当前向导仅辅助采集，运动和计量由操作员控制；
- 固定 TCP、低速空程、Stop/急停/报警/断线、激光和 PSO 的实体机验证；
- 多份历史运行目录配置的清理与生产 MachineProfile 唯一来源迁移。

SDK 与参考资料处理：本轮核对了用户提供的 2025-04 x64 `gts.h/gts.lib/gts.dll`、CommandList Demo 和《新架构功能编程手册》。工程内 `3rd/gts` 的接口版本更新且已包含本轮使用的 Group、CommandList、位置转换及列表 IO API，因此没有用较旧 DLL 覆盖当前工程依赖；初始化顺序和参数语义按资料逐项复核。

软件验证结果：

- `acs-gtn-debug` 全目标 Ninja Debug 构建通过；
- 架构、翻译上下文、Process 当前设置、运行时轴映射、CAM 双坐标合同、标定求解及快照并发共 8 项相关测试通过；
- ASan Debug 的 `lcnc_startup_smoke_test` 通过；
- 常规 Debug 启动冒烟仍在既有 CAD 任务面板 `CadTaskPanelController::refreshPanelState()` 阶段崩溃，发生在本轮新增物理标定信号接线之前；该问题不属于 GTN 调用链，但在修复前仍阻塞常规 Debug 整机发布；
- 全量 CTest 中机台安全索引测试在 180 秒超时，ACS Simulator SDK 集成测试长时间无输出后终止。本轮相关测试已独立通过，但上述两个既有环境/性能门禁仍为 Open。

### 0.1 无探针时的构型派生 TCP 与 RTCP 加工

机台面板提供“从当前构型生成 RTCP 参数”入口。按钮将当前构型中的父/子旋转轴线上点、方向向量、轴号、当量与操作员输入的 TCP 生成不可变 `ConfigurationDerived` 记录，激活后同时打开 Group、RTCP 和构型派生参数许可。

粗测 TCP 填写规则：

- 填写机床零位姿态下，刀尖/激光焦点在 MCS 中的绝对 XYZ；
- 如果 MCS 原点就建在刀尖/焦点，填 `0, 0, 0`；
- 不能填焦距、喷嘴长度或旋转中心；
- 界面默认载入当前 `headTcp.installationOffsetX/Y/Z`，必须根据现场 MCS 定义确认后才能生成。

`ConfigurationDerived` 不等于 `MachineVerified`。v11.4 起，显式允许该来源后按正常 RTCP 工艺执行：Group 初始化保持输出关闭，运行时按正常 CommandList 激光/吹气时序控制，采用配置的刀具速度、加速度和机床轴约束。旧的 5 mm/s、2 deg/s 专用试运动上限不再生效。软件允许加工不代表物理标定精度或整机验收通过。

## 1. 结论

现有构型配置已经具备五轴软件运动学的基本结构字段，包括轴角色、父子链、轴线上一点、轴方向、控制器轴号、当量、限位、工件安装变换和刀头几何。但是，当前配置只能证明“可以表达一个理论五轴模型”，不能证明“已经完成实机五轴标定”，也不能直接作为固高 RTCP 的生产配置。

当前主要结论如下：

1. RTCP 关闭时，继续由 CAM 完整求解并下发 `Xaxis/Yaxis/Zaxis/R1/R2` 机床轴坐标。
2. RTCP 开启时，CAM 下发 `TCP_X/TCP_Y/TCP_Z + R1/R2`，固高控制器根据五轴运动学模型计算直线轴补偿。
3. CAM 在两种模式下都必须保留预测机床轴坐标，用于连续姿态、碰撞、安全证书和控制器转换结果对比；RTCP 模式下预测轴坐标不得作为运动命令下发。
4. 新控制链统一使用 `Axis Group + CommandList`。RTCP 不是运行中切换的单个 API 开关，切换模式必须在空闲状态重新配置 Group。
5. 当前三段式标定向导只对齐 STEP 机台几何，不采集真实轴姿态，也不拟合旋转轴线。它不能作为物理五轴标定证明，应明确改名为“机台几何对齐向导”。
6. 新增独立的“物理五轴运动学标定向导”，采集多姿态实测值，自动拟合父/子旋转轴轴线，生成 `CP/CS/VP/VS`、质量指标和不可变标定记录。
7. 实测标定不需要将 STEP/STL 发送给控制器。STEP 继续服务于 OCC 显示、碰撞和数字孪生；控制器只消费数值运动学模型。
8. 当前各运行输出目录存在互相矛盾的 `machine.toml`。重构前必须建立唯一、可追溯的活动机台配置来源。

## 2. 当前配置审阅

### 2.1 已有数据

`MachineConfigurationService` 当前保存：

- 机型预设和加工模式；
- 每根轴的名称、类型、语义角色和父轴；
- `direction[3]`：控制器正方向在零位父坐标中的方向；
- `origin[3]`：以控制器轴坐标编辑和持久化的轴线上一点；
- 控制器轴号、回零号、当量、限位、速度、加速度和通用 jerk；
- `workpieceSetup`；
- `headTcp` 的名义光束方向、焦距和安装偏置；
- 包含上述字段的综合配置指纹。

这些字段可支持当前 CAM 软件 IK，也可以作为生成固高 `TFiveAxisKinematicParameter` 的候选输入。

### 2.2 当前部署配置不一致

本次只读检查发现：

| 运行目录 | 预设 | 旋转轴 | 旋转中心候选 | 其它差异 |
|---|---|---|---|---|
| `x64/ninja/Debug/config/machine.toml` | `VERTICAL_BC_TABLE` | B/C | B、C 均为 `(0,0,150)` | `workpieceSetup.z=60` |
| `x64/vs/Debug/config/machine.toml` | `VERTICAL_AC_TABLE` | A/C | A、C 均为 `(0,0,-150)` | 缺少当前坐标约定标记 |
| `x64/vs/Release/config/machine.toml` | `VERTICAL_BC_TABLE` | B/C | B、C 均为 `(0,0,0)` | X 正方向为 `(-1,0,0)` |

这些配置不能同时代表同一台实机。未确认上机使用的具体 EXE 和配置路径前，不能将其中任何一份认定为标定真值。连接日志必须输出规范化配置路径、配置哈希、机型、五轴角色和标定 ID。

### 2.3 已有参数的充分性

| 固高/五轴需求 | 当前字段 | 现状判断 |
|---|---|---|
| XYZ/R1/R2 语义与物理轴号 | role、parent、controllerIndex | 结构上具备，但多份部署配置不一致 |
| 轴当量 | resolution | 有单值，但需明确到 `TProfileScale alpha/beta` 的换算和实测来源 |
| 五轴机型 | preset + parent/role 推导 | 可推导，但未持久化固高模型枚举和推导版本 |
| Primary/Slave 轴线上一点 | rotary origin | 可表达，但目前多为理论/手填值，无实测证明 |
| Primary/Slave 轴方向 | rotary direction | 可表达，但目前为标准单位向量，无拟合质量 |
| XYZ 实际运动方向 | linear direction | 可表达理论方向；当前校验强制正交，不能记录真实非正交误差 |
| `dirMode`、`dir[5]`、`axisVector[5][3]` | 无控制器快照 | 缺失；只能临时推导，无法回读对比和审计 |
| ACS 理论零点与实际零点偏差 | homeIndex、origin | 语义不足，缺少独立 ACS kinematic offset 与标定来源 |
| 旋转轴周期/展开策略 | 软限位 | 缺少控制器旋转周期、连续轴和最短/指定方向策略 |
| ToolLocationPoint/TCS | headTcp | 只有上位机刀头几何，未形成固高 TCS/工具标定记录 |
| PCS | workpieceSetup | 有基础值，但缺少 PCS 所有权、测量来源和独立指纹 |
| 标定样本 | 无 | 缺失 |
| 拟合残差、覆盖范围、异常点 | 无 | 缺失 |
| 标定时间、设备、人员、版本 | 无 | 缺失 |
| RTCP 控制器正反解验证 | 无 | 缺失 |
| 控制器五轴/前瞻授权 | 无运行时能力快照 | 缺失 |

结论：当前参数足以运行理论软件 IK，但不满足实机五轴标定的可追溯性、质量证明和 RTCP 启用门槛。

### 2.4 当前校验逻辑的限制

当前 `validateConfiguration()` 主要检查：

- 轴名称、角色和控制器轴号唯一；
- 当量、限位和运动参数有效；
- 父链无缺失、无环；
- TableSpin 是 TableTilt 的后代；
- X/Y/Z 相对运动满秩；
- X/Y/Z 方向严格正交。

它没有检查：

- 旋转轴样本是否足够；
- 旋转轴拟合半径、角度覆盖、RMS、最大残差和条件数；
- 拟合轴向与控制器正方向是否一致；
- 固高模型类型与父子链是否一致；
- CAM 正解/逆解与 `GTN_GroupPosTransform` 是否一致；
- 当前配置是否来自一份已激活、未过期的实测标定。

严格正交校验还意味着当前结构不能直接保存 `dirMode=1` 所需的实际非正交轴向。重构时应把“控制器显示坐标基底”和“实测物理轴向”分开，或将坐标转换推广为可逆的非正交三维基底，而不是继续用一个字段承载两种语义。

## 3. 现有标定向导定位

当前 `DialogAxisCalibrationWizard` 执行：

1. 从 STEP/OCC 视图拾取父旋转轴参考面中心；
2. 拾取子旋转轴参考面中心；
3. 拾取切割头下端面中心；
4. 读取已经手工配置的旋转中心；
5. 平移机台 STEP，使模型轴心和模拟 TCP 对齐配置值；
6. 回写对齐后的 STEP。

现有算法从两条已配置方向的直线求最近点，并要求轴线间距不超过固定容差。它没有通过多个真实旋转姿态重新计算轴线，也不会修改物理旋转中心。

因此目标重构为：

- `MachineGeometryAlignmentWizard`：保留现有 STEP 选面和几何对齐功能；
- `PhysicalKinematicsCalibrationWizard`：新增实机多姿态采集、轴线拟合、工具标定和 RTCP 验证；
- 在应用层提供统一的“机台标定中心”入口，但在数据和权限上严格区分几何对齐与物理标定。

## 4. 目标配置模型

### 4.1 配置所有权

| 数据 | Owner | 生命周期 |
|---|---|---|
| STEP 机台装配和轴部件归属 | MachineWorkspace/CAD | 参考几何，可重新对齐 |
| 名义轴拓扑、轴号、当量、限位 | MachineConfigurationService | 机台级 |
| 原始实测样本和拟合结果 | MachineCalibrationService | 不可变标定记录 |
| 活动标定 ID | MachineProfileStore | 机台级原子切换 |
| RTCP 开关、Group/List、前瞻和平滑 | Process 设置 | 控制器运行配置 |
| 控制器版本、授权和 Group 状态 | Process session | 连接级，不持久化为标定真值 |
| PCS/工件安装 | Project/机台工件设置 | 每次工件或装夹可变化 |
| TCS/工具偏置 | Tool calibration store | 每把工具/切割头版本 |

Process 不得包含 OCC 类型。它只接收冻结的标量 `ControllerKinematicsSnapshot`。

### 4.2 标定记录

建议增加以下 OCC-free 数据：

```cpp
struct CalibrationSample
{
    QString sampleId;
    QString targetAxisName;
    std::array<double, 5> actualAxes; // 实际反馈，不是命令值
    std::array<double, 3> measuredReferencePointMcs;
    QString measurementSource;
    QString timestampUtc;
    bool accepted;
};

struct AxisLineFit
{
    QString axisName;
    std::array<double, 3> pointMcs;
    std::array<double, 3> unitVectorMcs;
    int sampleCount;
    double angularCoverageDeg;
    double fittedRadiusMm;
    double rmsResidualMm;
    double maxResidualMm;
    double conditionMetric;
    QVector<QString> rejectedSampleIds;
};

struct MachineCalibrationRecord
{
    int schemaVersion;
    QString calibrationId;
    QString machineIdentity;
    QString nominalConfigurationFingerprint;
    QString controllerModelType;
    AxisLineFit primaryAxis;
    AxisLineFit slaveAxis;
    ToolCalibrationSnapshot tool;
    QVector<CalibrationSample> samples;
    CalibrationVerificationResult verification;
    QString rawSamplesSha256;
    QString calibrationFingerprint;
    QString createdAtUtc;
    QString operatorName;
    QString measurementDevice;
    QString softwareVersion;
};
```

标定记录一经完成即不可原位覆盖。重新标定生成新 ID；激活新记录采用 staging、完整校验和原子替换。旧记录保留用于回滚和审计。

### 4.3 控制器快照

```cpp
struct ControllerKinematicsSnapshot
{
    short modelType; // RW_C_ON_A、RW_C_ON_B 等
    std::array<double, 3> primaryAxisPointMcs;
    std::array<double, 3> slaveAxisPointMcs;
    std::array<double, 3> toolLocationPointMcs;
    short directionMode;
    std::array<short, 5> directions;
    std::array<std::array<double, 3>, 5> axisVectorsMcs;
    std::array<short, 5> physicalAxisIndices;
    std::array<ProfileScale, 5> scales;
    QString machineKinematicsFingerprint;
    QString calibrationFingerprint;
    QString toolCalibrationFingerprint;
};
```

`modelType` 和 Group 内顺序由轴角色与父子链生成，不能仅依据轴名称猜测。例如：

- TableTilt=A，TableSpin=C，且 C 的父链包含 A：`RW_C_ON_A`；
- TableTilt=B，TableSpin=C，且 C 的父链包含 B：`RW_C_ON_B`。

任何不受支持、父链不一致或轴角色不唯一的组合均拒绝生成快照。

### 4.4 指纹拆分

当前综合 `configurationFingerprint()` 将工件安装、轴硬件、运动学和刀头几何混为一个哈希。重构后至少拆分：

- `axisHardwareFingerprint`：轴号、当量、零点、限位；
- `machineKinematicsFingerprint`：拓扑和实测轴线；
- `toolCalibrationFingerprint`：TCP/TCS；
- `workpieceSetupFingerprint`：PCS/装夹；
- `controllerExecutionFingerprint`：以上内容加 RTCP 模式、Group 配置、SDK/固件能力。

保留综合指纹用于兼容，但预检应报告具体是哪一层发生变化。

## 5. 双执行模式

### 5.1 RTCP 关闭

CAM 完整求解：

```text
工件局部点和法向
  -> 切割/空程法向偏置
  -> 工件安装变换
  -> 连续 R1/R2 姿态选择
  -> 根据工具侧/工件侧父链求解 X/Y/Z
  -> 轴限位、奇异点、正解残差和碰撞检查
  -> Xaxis/Yaxis/Zaxis/R1/R2
```

Group 配置：

```text
Command coordinate = ACS
Orientation         = ORI_MODE_NONE
Profile coordinate = ACS
```

`GTN_MoveLinearAbsolute` 输入为完整机床轴坐标。Process 不做二次求解或符号翻转。

### 5.2 RTCP 开启

CAM 输出：

- `TCP_X/TCP_Y/TCP_Z`；
- 连续求解的 `R1/R2`；
- 仅供碰撞和一致性检查的预测 `Xaxis/Yaxis/Zaxis/R1/R2`。

第一阶段固定使用 MCS 输入，避免应用 `workpieceSetup` 和控制器 PCS 重复生效：

```text
Command coordinate = MCS
Orientation         = ORI_MODE_ROTATE_AXIS_POS
Profile coordinate = PCS
```

`GTN_MoveLinearAbsolute` 输入：

```text
TCP_MCS_X, TCP_MCS_Y, TCP_MCS_Z, R1, R2
```

PCS 直接输入作为后续独立阶段；只有当 PCS 所有权、旋转轴姿态语义和实体卡验证完成后才能开放。

### 5.3 执行快照

CAM 的每个最终运动节点同时冻结：

- TCP 意图；
- 预测机床轴位姿；
- 轮廓/空程/下刀阶段；
- segment number 和 user tag；
- 机床、标定、工具和工件指纹。

快照顶层明确指定 `SoftwareSolvedAxes` 或 `ControllerRtcp`。Motion sink 不得依据数组内容猜测模式。

PureSimulation 始终使用预测机床轴位姿。ACS 仅接受软件求解轴模式。GTN 新 Group sink 根据快照模式选择唯一合法的命令表示。

## 6. 物理五轴标定向导

### 6.1 模块边界

物理向导不能从 CAM 直接调用 Process，因为模块依赖只允许上层协调下层。当前实现边界：

- `core/algorithms/kinematics/axis_line_calibration_solver.*`：纯数学拟合；
- `core/kinematics/machine_calibration_record.h`：OCC-free 标定 DTO；
- `core/kinematics/machine_calibration_service.*`：样本、候选记录、活动指针和原子激活；
- `modules/cam/ui/machine/dialog_physical_kinematics_calibration_wizard.*`：只处理表单、CSV、纯算法和 Core 标定服务，不持有 Process 或控制器；
- `MainWindow`：应用层收到“读取当前反馈”请求后调用 Process 的异步状态刷新，再把轴位置快照回送给向导；
- 现有 CAM 向导改名为几何对齐子流程，不拥有实机运动和标定记录。

当前版本不提供自动运动，因此尚未新增 `ICalibrationMotionService`。后续接入自动姿态运动时，必须按本方案新增 Process 合同和 DeviceCommandQueue 实现，不能把控制器指针注入 CAM 向导。

### 6.2 数据采集来源

向导支持三个来源：

1. CSV 导入：导入固高或计量系统的 `X,Y,Z,R1,R2` 实测文件；
2. 辅助示教：操作员低速移动、测得标准球中心后点击“采集当前点”，软件自动读取实际 APOS；
3. 自动测量：未来接入探针、相机或激光测高仪，通过 `ICalibrationMeasurementSource` 自动返回参考点。

仅记录多个旋转角度而没有每个姿态对应的实测参考点，无法拟合旋转轴线。自动填充至少需要：

- 实际反馈 X/Y/Z/R1/R2；
- 标准球/测点在 MCS 中的位置，或能从探针接触与探针标定计算该位置；
- 测量源、时间和探针/球半径补偿。

### 6.3 向导步骤

#### 步骤 0：预检

- 选择唯一活动机台配置；
- 确认控制器版本、五轴和前瞻授权；
- 确认轴号、当量、正方向和限位；
- 五轴已回零、空闲、激光关闭；
- 安全 IO、停止和急停有效；
- 标准球、探针或其它测量源已完成自身标定；
- 保存标定前配置和活动标定 ID。

任一状态未知即禁止自动运动。

#### 步骤 1：选择机型和轴角色

向导根据 `TableTilt/TableSpin` 和父链自动显示：

```text
Primary = A 或 B
Slave   = C
Model   = RW_C_ON_A 或 RW_C_ON_B
Group   = X/Y/Z/Primary/Slave
```

操作员只确认，不直接编辑固高枚举。

#### 步骤 2：标定子旋转轴

- 父倾斜轴固定在标定零姿态；
- 子旋转轴选择覆盖充分的多个角度；
- 每个角度由操作员确认测量状态后采集实际 APOS 和参考点；
- 推荐生产采集 5~12 点，数学最低 3 点；
- 自动检查角度跨度、点间距离、拟合半径和重复点；
- 拟合 `CS + t*VS`。

#### 步骤 3：标定父旋转轴

- 子轴固定在记录的标定姿态；
- 父轴采集多个实际角度和参考点；
- 拟合 `CP + t*VP`；
- 所有结果转换到零位姿 MCS。

#### 步骤 4：方向和正负号

轴线拟合只能得到无方向直线。向导根据实际正向角度增加时参考点的运动方向，选择 `VP/VS` 的符号，使右手旋转预测与实测一致。不得仅依靠轴名 A/B/C 决定方向。

#### 步骤 5：TCP/TCS

- 双转台仍需记录真实激光焦点、喷嘴参考点和工具版本；
- 摆头机型还需完整刀长和旋转中心到 TCP 的偏置；
- 标定结果进入独立工具记录，不修改 A/C 或 B/C 轴心以补偿刀具误差。

#### 步骤 6：拟合报告和人工确认

显示：

- CP/VP、CS/VS；
- 采样数量和角度覆盖；
- 拟合半径；
- 每个样本残差；
- RMS、最大残差、条件指标；
- 被建议排除的异常点及原因；
- 与当前配置的中心和方向变化量；
- 模型类型、轴号、当量、工具和配置指纹。

异常点必须由操作员明确确认后才能排除，不能静默删除。

#### 步骤 7：影子验证

先不启用 RTCP 运动：

1. 用新候选模型在 CAM 计算预测 ACS；
2. 初始化固高 Group，但保持激光关闭；
3. 使用 `GTN_GroupPosTransform` 将 TCP 姿态转换为 ACS；
4. 对比 CAM 与固高在标定姿态、刀路首尾点、旋转极值和接近奇异点处的结果；
5. 输出最大、平均和 RMS 轴差；
6. 任一超出机台验收阈值则拒绝激活。

#### 步骤 8：固定 TCP 实机验证

- 低速、激光关闭；
- TCP 对准标准球固定点；
- 改变父/子旋转姿态；
- 控制器自动补偿 XYZ；
- 测量 TCP 漂移向量、最大误差和 RMS；
- 停止、急停和故障恢复必须同时验证。

只有完成该步骤，状态才允许从 `Computed` 进入 `MachineVerified`。

#### 步骤 9：原子激活

- 写入新的不可变标定记录；
- 校验文件、样本哈希和结果哈希；
- 原子切换 `activeCalibrationId`；
- 更新机床运动学、CAM 求解、碰撞和控制器快照；
- 使已有执行快照和安全包失效；
- 保留旧标定以便回滚；
- 记录审计日志。

### 6.4 拟合算法

每根旋转轴采用三维圆/轴线鲁棒拟合：

1. 对测量点做中心化和协方差分析；
2. 求最佳拟合平面，平面法向作为轴线无向向量；
3. 在平面内建立二维基底；
4. 最小二乘拟合圆心和半径；
5. 迭代鲁棒加权或受控异常点剔除；
6. 将圆心还原到 MCS，形成轴线上一点；
7. 根据实际正向运动确定轴向符号；
8. 计算 RMS、最大残差、角度覆盖和条件指标。

算法必须拒绝：

- 少于最低有效样本数；
- 角度覆盖过小或样本集中；
- 参考点太靠近旋转轴导致半径退化；
- 拟合平面/圆病态；
- 方向符号无法由测量确定；
- 父/子姿态未保持在该阶段要求的固定值；
- 实际反馈与记录目标漂移超过采集阈值；
- 结果超出机床物理包络。

具体生产阈值不得照搬 Demo，应由机台精度指标和测量系统不确定度配置，并写入标定记录。

当前第一版采用全体已接受样本的最小二乘拟合，并显示超过残差建议阈值的样本 ID，但不会自动删除样本。操作员需复核测量、修正或删除对应表格行后重新计算；自动鲁棒加权留待有标准数据集后加入，不能在没有审计记录的情况下静默剔除点。

### 6.5 自动运动约束

向导自动运动必须满足：

- 仅由应用层调用 Process 的标定运动服务；
- Process 标定运动服务使用 DeviceCommandQueue；
- 每段运动取得碰撞/安全许可并具有有限超时；
- 默认低速，激光、PSO、快门和吹气保持安全关闭；
- 每次运动后确认实际 APOS 到位才允许采样；
- 暂停、停止、急停和窗口关闭均能终止会话；
- 断线、报警、回零丢失或配置指纹变化立即作废当前会话；
- 向导不能直接调用 GTN SDK，也不能跨线程持有 `MotionControl*`。

完全自动的标准球搜索需要可靠探针/视觉测量源。在没有该硬件接口时，第一版应实现“安全移动建议 + 操作员对准 + 一键采集 APOS”，不能把轴反馈位置误称为已自动测得球心。

## 7. GTN Group 初始化和 RTCP 开关

### 7.1 状态机

```text
Disconnected
  -> Connected
  -> CapabilityVerified
  -> Homed
  -> CalibrationVerified
  -> GroupConfigured(mode + fingerprints)
  -> CommandListReady
  -> Running
```

RTCP 开关只允许在控制器空闲且 CommandList 未执行时改变。改变后：

1. 安全停止并确认 Group/List 空闲；
2. Disable Group；
3. 清除旧关联；
4. 重新配置运动学、坐标定义和前瞻；
5. 回读验证；
6. 重新生成执行合同并完成预检。

运行中、暂停中、停止恢复中或故障未恢复时禁止切换。失败后不得自动退回另一种坐标语义。

### 7.2 通用初始化顺序

```text
1. Open/连接并查询 DLL、MC、DSP、固件和资源
2. 加载并记录规范化配置文件路径和哈希
3. 检查五轴、CommandList、前瞻和 RTCP 能力
4. 检查五轴回零、空闲、无报警
5. GTN_GroupDisable
6. GTN_UngroupAllAxes
7. GTN_SetAxisScale x 5
8. GTN_AddAxisToGroup x 5
9. GTN_SetGroupKinematicTransform
10. 设置 ACS offset、PCS/TCS（按当前模式）
11. 设置 CommandPosDefine/ProfileCoordinateSystem
12. 设置轴、Group、姿态、停止、平滑和前瞻参数
13. 绑定 Group 和 CommandList
14. GTN_GroupEnable
15. 回读模型、坐标定义、约束和关联
16. GTN_GroupPosTransform 一致性检查
```

所有步骤都写 `gtn.api:` 日志，并在首个非成功结果时停止初始化。不得像 Demo 一样记录错误后继续执行。

### 7.3 CommandList 执行

```text
GTN_ClearCommandListData
GTN_MoveLinearAbsolute x N
GTN_CommandListDataEnd
GTN_StartCommandList
```

仅文档明确的 `10700` 使用有界、可取消重试；`11700` 调用 `GTN_GetLastCommandError` 并记录 commandCode、errorCode、segmentNum 和 userTag。

## 8. 设置界面

### 8.1 机台配置页

显示：

- 唯一活动机台配置路径和哈希；
- 名义机型、轴角色和父子链；
- 当前活动标定 ID、时间、状态和质量；
- CP/VP、CS/VS；
- 标定残差和最近 RTCP 验证结果；
- “物理五轴标定向导”入口；
- “STEP 几何对齐向导”独立入口。

旋转轴实测结果不再通过普通表格静默手改。专家模式允许建立候选值，但必须经过同一验证和激活流程。

### 8.2 Process/GTN 页

```text
五轴控制架构：Group + CommandList
控制器 RTCP：关闭 / 开启
命令坐标系：MCS（首期只读）
Group/List：...
平滑参数：accTime / coefficient
前瞻参数：...
活动标定：ID / fingerprint / verified
控制器能力：版本 / FiveAxis / LookAhead / RTCP
```

规则：

- 旧 FIFO 模式不允许启用 RTCP；
- 非 GTN 控制器不显示 GTN RTCP；
- 未激活标定、标定过期或控制器转换验证失败时开关不可选；
- 连接或运行状态下只读；
- 切换后重新生成执行快照并预检，无需重新抽取 CAD 轮廓。

## 9. 代码重构范围

### 9.1 Core/Machine

- 新增 OCC-free 标定 DTO 和 MachineCalibrationService；
- 新增三维旋转轴线拟合算法和确定性单元测试；
- 拆分配置指纹；
- 建立唯一 MachineProfileStore；
- 支持从活动标定生成 MachineKinematics 和 ControllerKinematicsSnapshot；
- 明确非正交实测轴向与控制器显示坐标基底的关系。

### 9.2 CAM

- 执行节点同时保留 TCP 意图和预测轴位姿；
- RTCP 模式仍做影子完整 IK 和碰撞验证；
- 首段、空程、下刀和切割使用同一双表示节点；
- 配置/标定/工具/工件指纹进入提交快照；
- 当前向导改为 STEP 几何对齐，不再宣称完成物理五轴标定。

### 9.3 Process

- 新增 FiveAxisExecutionMode；
- 新增 ICalibrationMotionService；
- 新 Group/CommandList sink 接收明确的轴命令或 TCP 命令 variant；
- 增加能力查询、模型回读、转换对比和结构化日志；
- RTCP 开关纳入 Group 生命周期；
- 保持 SDK 调用全部经过设备租约和 DeviceCommandQueue。

### 9.4 App/UI

- 应用层统一物理标定向导；
- 采集 APOS、导入 CSV、显示样本和拟合质量；
- 编排 Machine、Process 和可选 CAM 几何对齐 facade；
- 不产生 CAM -> Process 依赖。

### 9.5 配置迁移

- 旧 `machine.toml` 只迁移为 `Nominal/Unverified`；
- 不因存在 origin/direction 自动升级为 `Calibrated`；
- 首次运行要求明确选择活动机台配置；
- 多个输出目录配置不再自动竞争；
- 未完成实测标定仍可使用 PureSimulation；是否允许非 RTCP 实机轴坐标加工由独立安全策略决定，RTCP 必须禁止。

## 10. 分阶段实施

### Phase 0：配置唯一性与审计

- 建立 MachineProfileStore；
- 日志输出活动配置路径、哈希和标定 ID；
- 检测并报告 AC/BC、中心、方向和坐标约定漂移；
- 不改变运动。

### Phase 1：数据合同与纯算法

- 增加标定记录、轴线拟合、质量门槛和测试；
- 增加控制器运动学快照生成器；
- CAM 节点输出双表示；
- 无实体运动。

### Phase 2：标定向导辅助采集

- CSV 导入；
- 手动/辅助示教采集实际 APOS；
- 自动拟合、报告、候选保存和原子激活；
- 不启用自动探测。

### Phase 3：新 Group、RTCP 关闭

- 用 Group + CommandList 执行 CAM 已求解轴坐标；
- 保持软件 IK；
- 完成低速、激光关闭的实体五轴验证。

### Phase 4：RTCP 影子验证

- 配置固高模型；
- 用 `GTN_GroupPosTransform` 对比 CAM 预测轴；
- 输出可审计误差报告；
- 不执行 RTCP 轨迹。

### Phase 5：RTCP 实体运动

- 固定 TCP 多姿态验证；
- 单点、短线、小闭合轮廓；
- 空程、停止、急停和断线恢复；
- 最后接入激光和 PSO。

### Phase 6：自动测量

- 接入探针、相机或激光测高设备；
- 自动搜索参考球/参考面；
- 自动运行姿态序列和重复性评估；
- 保留人工确认与停止权限。

## 11. 验证门槛

### 自动化

- 配置迁移和活动配置唯一性；
- AC、BC、双摆头和混合机型映射；
- 理想轴、带噪声轴、异常点、覆盖不足和退化样本拟合；
- 轴向符号判定；
- 非正交模型坐标转换；
- 标定记录哈希、原子激活和回滚；
- RTCP/非 RTCP 命令 variant 不可混用；
- Group 初始化顺序、错误传播、10700 重试、11700 详情和停止；
- 配置/标定漂移导致快照失效；
- Process 源码无 OCC 类型。

### GUI

- 向导中断、恢复、重开和历史记录；
- 样本表、残差图、异常点确认和候选对比；
- 连接/运行时开关只读；
- DPI、中文翻译和窗口关闭停止行为。

### 实机

- 轴正方向、当量、回零和限位；
- 多姿态采集重复性；
- 标准球拟合残差；
- CAM/GTN 转换一致性；
- 固定 TCP 漂移；
- 低速五轴轨迹；
- Stop、急停、伺服报警和断线；
- 激光关闭阶段通过后才允许激光/PSO。

构建和 CTest 只能证明软件门槛，不能替代上述实机验收。

## 12. 实机投产前仍需确认

1. 本次上机实际使用的 EXE、构建变体和 `machine.toml` 路径；
2. 实机是 AC 还是 BC，父/子旋转轴的真实名称和控制器物理轴号；
3. 固高当前 DLL、MC、DSP、固件及五轴/RTCP/前瞻授权；
4. 当前是否已有标准球、测头、相机或激光测高仪；
5. 实测数据由操作员重新找正球心后记录 XYZ，还是由外部计量设备直接输出球心；
6. 机台规定的标定误差、固定 TCP 漂移和重复性验收阈值；
7. TCS 的物理基准是喷嘴端、激光焦点还是独立测头中心；
8. 生产机台配置的规范存储位置、权限和备份策略。

上述软件实现不会自动把候选标定提升为实机已验证。只有操作员录入固定 TCP 实测结果且满足向导门槛，记录才可标记为 `MachineVerified`；即使如此，首次开放 RTCP 加工仍必须完成本章的实体卡与实机门槛。当前建议先启用 Group/CommandList、保持 RTCP 关闭并进行激光关闭的低速空程验证，再进入 RTCP 影子对比和实体运动阶段。
