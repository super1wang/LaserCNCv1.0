# LaserCNC

目标：开发一款三维五轴激光加工 CAM 软件（CAD + CAM + 激光控制）。

## 技术栈与环境

1. 语言：C++17
2. UI：Qt 6.9.1（路径：E:/Qt6.9.1/6.9.1/msvc2022_64）
3. 几何内核：OpenCASCADE 7.9.0（路径：G:/environment/occt7.9-vc144-64-combined/occt-vc144-64）
4. Ribbon：SARibbon（路径：F:/wangchao/SARibbon）
5. 构建：CMake + MSVC2022 x64

## 当前项目框架

### 1) base 层（核心数据与任务）

- `LcncApplication`：文档生命周期管理、活动文档切换、信号分发；内含**机台工作区**管理（`ensureMachineDocument` / `machineDocumentId` / `isMachineDocument`）
- `LcncDocument`：基于 XCAF 文档的数据封装，XDE free-shape 存储 + `TDataStd_Integer` 分类标签，支持 Undo/Redo
- `MachineKinematics`：运动学数据，支持 `clear()` 重置与 `autoDetect()` 识别 `LCNC_AXIS_*` 标准命名
- `XcafUtils`：XCAF 标签名称、Entry、Shape 获取等工具
- `TaskManager` / `TaskProgress`：统一异步任务与进度状态管理

### 2) graphics 层（三维显示）

- `GraphicsScene`：封装 `V3d_Viewer` + `AIS_InteractiveContext`
- `ShapeObjectDriver`：AIS 形体对象样式与材质封装

### 3) gui 层（文档显示桥接）

- `GuiApplication`：`LcncDocument` 与 `GuiDocument` 映射
- `GuiDocument`：**per-document V3d_View 架构**（Mayo 模式）：每个文档拥有独立的 `V3d_View`、`AIS_ViewCube`、`AIS_Trihedron` 和动画定时器，切换文档只切换渲染目标，摄像机状态保持

### 4) app 层（命令与界面）

- `MainWindow`：SARibbon 主窗体与三栏布局、信号连接顺序保证关闭文档无悬空指针；**准备标签始终显示机台文档视图**（`showMachineView` / `showWorkpieceView` 路由）
- `CommandBase` + `CommandContainer`：命令模式封装
- 文件命令：新建/打开（后台 Task + 进度条）/保存/导入导出（后台 Task + 进度条）
- 编辑命令：撤销/重做
- 显示命令：视角、显示模式
- 机台命令：`CmdLoadMachine`（复用唯一机台文档）/ `CmdMarkAxes` / `CmdMountWorkpiece` / **`CmdUnloadMachine`** / **`CmdExportMachine`**（轴系按 `LCNC_AXIS_*` 命名导出 STEP）
- 关键控件：
  - `WidgetOccView`：中间 3D 视窗，共享一个 `Aspect_NeutralWindow`，各文档视图复用
  - `WidgetModelTree`：准备页模型树（始终显示机台文档结构）
  - `WidgetMachinePanel`：机台管理面板（含加载/标记轴系/卸载/导出四个操作按钮）
  - `WidgetLaserControl`：激光控制面板
  - `DialogTaskManager`：任务进度窗口

## 阶段进展

### 已完成阶段

#### 阶段 1：框架搭建与基础交互（已完成）

**核心架构**
- 完成整体分层架构（base / graphics / gui / app）
- 完成 Ribbon 主界面和三栏布局（左侧文档/准备/执行，中间3D视窗，右侧机台/激光控制）
- 完成命令模式框架（CommandBase + CommandContainer + IAppContext）

**文档系统**
- 完成文档生命周期（新建、打开、关闭、活动文档切换）
- 完成 STEP / IGES / STL / BREP 文件导入（后台 TaskManager，带进度条）
- 完成 STEP 导出骨架
- 完成 Undo/Redo 命令框架（XCAF transaction 级别）
- 完成文档树（左侧"文档"标签，显示全部打开文档的装配树节点）

**3D 视窗**
- 完成中间视窗基础交互（旋转 / 平移 / 缩放 / 选择）
- 完成 per-document V3d_View 架构（Mayo 模式），切换文档无视图抖动
- 完成视图辅助元素：视图方块（右上，AIS_ViewCube，点击动画切换视角）和坐标轴（左下，RGB AIS_Trihedron）
- ViewCube 点击正常（直接调用 `StartAnimation`，绕开 `HandleClick` 内部保护标志）
- 每文档独立 RGB 坐标轴和 ViewCube，无需重建

**任务与进度**
- 完成异步任务框架（TaskManager + TaskProgress + DialogTaskManager）
- 打开文件 / 导入 STEP/IGES/STL/BREP 均为后台 Task，不阻塞 UI，显示进度条

**稳定性修复**
- 修复启动时 3D 背景不可见问题
- 修复新建后继续新建不崩溃
- 修复 `XCAFDoc_ShapeTool::AddComponent` 在非形状标签上调用导致的 XCAF 树损坏及关闭崩溃
- 修复 `OpenCommand/CommitCommand` 事务栈残留导致的析构崩溃
- 修复 `documentClosed` 信号时序问题（在 `onActiveDocumentChanged` else 分支提前 detach，避免悬空指针）
- 修复关闭文档时 `eraseAll` 误删 ViewCube/Trihedron gizmo 问题（改为 per-map 逐步 erase）
- 修复节点点击崩溃（在 `setActiveDocument` 之前先读取 item data）
- 修复 `closeEvent` 同步关闭文档导致的卡死（改为只 detach 视图，由析构链清理资源）

### 已完成阶段（续）

#### 阶段 2：CAD 功能实装（已完成核心部分，草图延后）

**已完成**
- [x] 基本体参数化建模（长方体/圆柱体/球体/圆锥体/圆环体）
  - 每个基本体弹出参数输入对话框（QDialog + QFormLayout + QDoubleSpinBox）
  - 使用 BRepPrimAPI_MakeBox/Cylinder/Sphere/Cone/Torus 创建 OCC 形体
  - 自动创建文档（如无活动文档）并添加至工件分类
  - 创建后自动刷新显示并自适应 FitAll
- [x] 平移操作（CmdMoveShape）
- [x] 旋转操作（CmdRotateShape）
- [x] 布尔操作（CmdBoolUnion/CmdBoolCut/CmdBoolCommon）
- [x] 测量工具（距离/角度/面积）
- [x] 所有 CAD 命令注册到 CommandContainer，Ribbon CAD 标签正确连接

**延后至后续迭代**
- [ ] 草图编辑与约束（线/圆/圆弧）— 需要专用草图框架，计划在阶段 2.5 独立实施
- [ ] 缩放操作（gp_Trsf::SetScale）

#### 阶段 3：机台独立工作区与运动学（已完成）

**机台工作区架构**（本期新增核心设计）
- `LcncApplication::ensureMachineDocument()`：软件启动时自动创建**唯一且永久**的机台文档，该文档不受工件文档打开/关闭影响，也不会出现在"文档"标签的树中
- `LcncApplication::isMachineDocument(id)`：用于在整个系统中区分机台文档与工件文档
- `LcncApplication::closeDocument(id)`：机台文档被保护，无法通过普通关闭流程删除
- `IAppContext::machineDocument()` / `machineGuiDocument()`：命令层统一访问机台文档的接口
- **准备标签视图路由**：`MainWindow::onLeftTabChanged()` 切换到"准备"标签时调用 `showMachineView()`，切换到其他标签时调用 `showWorkpieceView()`，保证视图与标签绑定
- 左侧模型树与右侧机台面板始终反映机台文档状态，不受工件文档活动状态影响

**运动学数据模型**（`src/base/machine_kinematics.h/.cpp`）
- `MachineAxisDef`：轴定义结构体（名称、运动类型 Linear/Rotary、方向向量、行程范围、父轴、当前位置）
- `MachineKinematics`：运动学数据管理类（QObject），支持：
  - `loadPreset(configType)`：内置 4 种常见机床构型
    - `VERTICAL_AC_TABLE`：立式主轴 + AC 双转台
    - `VERTICAL_BC_TABLE`：立式主轴 + BC 双转台
    - `AB_HEAD`：龙门 + AB 摆头
    - `AC_HEAD`：龙门 + AC 摆头
  - `clear()`：重置所有轴定义、形体分配与工件挂载（本期新增）
  - `assignAxis/unassignAxis`：形体-轴系手动绑定
  - `mountWorkpiece/unmountWorkpiece`：工件挂载到指定轴
  - `autoDetect()`：优先识别 `LCNC_AXIS_<name>` 标准前缀（本期新增），其次按形体名称关键字自动分配轴系
  - `computeShapeTransform(entry)`：按运动学链式积计算机台零件世界变换
  - `setAxisPosition(name, pos)`：设置轴位置并触发 `axisPositionChanged` 信号

**命令系统**（`src/app/commands_machine.h/.cpp`）
- `CmdLoadMachine`：选择机床构型 → 选择 STEP/STL/BREP 文件 → 清除旧机台实体 → 后台导入到**唯一机台文档** → 完成后自动运行 `autoDetect()`
- `CmdMarkAxes`：操作机台文档，打开 `DialogMarkAxes` 手动指定轴系
- `CmdMountWorkpiece`：操作机台文档的运动学，读取当前活动**工件文档**中的工件实体
- **`CmdUnloadMachine`**（新增）：确认后清除所有机台实体及轴系配置，重置 3D 显示
- **`CmdExportMachine`**（新增）：将机台模型导出为 STEP 文件，每个轴系的形体合并成一个整体 Compound，命名为 `LCNC_AXIS_<轴名>`（如 `LCNC_AXIS_X`、`LCNC_AXIS_C`），未分配形体归入 `LCNC_AXIS_UNASSIGNED`；重新导入时 `autoDetect()` 自动识别并恢复轴系

**机台面板**（`WidgetMachinePanel`）
- 四个操作按钮：加载机台模型 / 标记轴系 / 卸载机台 / 导出机台模型
- 轴系位置 SpinBox 联动 `MachineKinematics::setAxisPosition()` + `GuiDocument::updateAxisTransforms()`

**轴系标记对话框**（`src/app/dialog_mark_axes.h/.cpp`）
- 网格布局，每行显示形体名称 + 轴系下拉选择框
- "自动检测"按钮：调用 `autoDetect()` 并更新 UI

**准备页模型树**（`WidgetModelTree`）
- 始终显示机台文档结构：轴系视图（每轴加粗节点）、机台零件按轴分组、挂载工件标注

### 进行中阶段

#### 阶段 4：CAM 刀路计算

- [ ] 3+2 定位与 5 轴联动刀路算法
- [ ] 刀路可视化与干涉检查
- [ ] 后处理与 G 代码生成

#### 阶段 5：激光加工控制

- ACS 控制器接入
- 设备连接状态与报警处理
- 加工流程执行控制（启动/暂停/停止/急停）
- 运动仿真与实时状态反馈

#### 阶段 6：工程化与质量

- 单元测试与集成测试
- 崩溃保护与日志体系
- 项目配置管理与参数持久化
- 安装包与部署流程

## 参考

- Mayo 架构参考：F:/wangchao/Yunco3D-v1.0/src
- 流程模块参考：F:/wangchao/Yunco1.1/trunk/librecad/src/Process
