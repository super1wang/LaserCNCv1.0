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

- `LcncApplication`：文档生命周期管理、活动文档切换、信号分发
- `LcncDocument`：基于 XCAF 文档的数据封装，XDE free-shape 存储 + `TDataStd_Integer` 分类标签，支持 Undo/Redo
- `XcafUtils`：XCAF 标签名称、Entry、Shape 获取等工具
- `TaskManager` / `TaskProgress`：统一异步任务与进度状态管理

### 2) graphics 层（三维显示）

- `GraphicsScene`：封装 `V3d_Viewer` + `AIS_InteractiveContext`
- `ShapeObjectDriver`：AIS 形体对象样式与材质封装

### 3) gui 层（文档显示桥接）

- `GuiApplication`：`LcncDocument` 与 `GuiDocument` 映射
- `GuiDocument`：**per-document V3d_View 架构**（Mayo 模式）：每个文档拥有独立的 `V3d_View`、`AIS_ViewCube`、`AIS_Trihedron` 和动画定时器，切换文档只切换渲染目标，摄像机状态保持

### 4) app 层（命令与界面）

- `MainWindow`：SARibbon 主窗体与三栏布局、信号连接顺序保证关闭文档无悬空指针
- `CommandBase` + `CommandContainer`：命令模式封装
- 文件命令：新建/打开（后台 Task + 进度条）/保存/导入导出（后台 Task + 进度条）
- 编辑命令：撤销/重做
- 显示命令：视角、显示模式
- 关键控件：
  - `WidgetOccView`：中间 3D 视窗，共享一个 `Aspect_NeutralWindow`，各文档视图复用
  - `WidgetModelTree`：准备页模型树
  - `WidgetMachinePanel`：机台管理面板
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

### 进行中阶段

#### 阶段 2：CAD 功能实装（进行中）

**已完成**
- [x] 基本体参数化建模（长方体/圆柱体/球体/圆锥体/圆环体）
  - 每个基本体弹出参数输入对话框（QDialog + QFormLayout + QDoubleSpinBox）
  - 使用 BRepPrimAPI_MakeBox/Cylinder/Sphere/Cone/Torus 创建 OCC 形体
  - 自动创建文档（如无活动文档）并添加至工件分类
  - 创建后自动刷新显示并自适应 FitAll
- [x] 平移操作（CmdMoveShape）
  - 下拉选择文档中的形体 + XYZ 增量输入
  - 使用 BRepBuilderAPI_Transform + gp_Trsf::SetTranslation 变换
  - 原地更新 XCAF 标签中的形体（XCAFDoc_ShapeTool::SetShape）
- [x] 旋转操作（CmdRotateShape）
  - 选择形体 + 旋转轴方向向量 + 旋转角度输入
  - 使用 gp_Ax1 + gp_Trsf::SetRotation 变换
- [x] 布尔操作（CmdBoolUnion/CmdBoolCut/CmdBoolCommon）
  - 双下拉选择形体 A 和 B
  - 使用 BRepAlgoAPI_Fuse/Cut/Common 进行布尔运算
  - 结果添加为新的工件实体（保留原始形体）
- [x] 测量工具（距离/角度/面积）
  - 距离：BRepExtrema_DistShapeShape 计算最小距离，弹窗显示结果（mm）
  - 角度：获取两形体首面法向量，计算夹角，弹窗显示结果（度）
  - 面积：BRepGProp::SurfaceProperties 计算总表面积，弹窗显示结果（mm²）
- [x] 所有 CAD 命令注册到 CommandContainer，Ribbon CAD 标签正确连接

**待完成**
- [ ] 草图编辑与约束（线/圆/圆弧）— 需要专用草图框架，延至后续迭代
- [ ] 缩放操作（gp_Trsf::SetScale）— 可快速补充

### 未完成阶段

#### 阶段 3：CAM 刀路计算

- 机床运动学模型（AC/BC/AB 等）
- 3+2 定位与 5 轴联动刀路
- 刀路可视化与干涉检查
- 后处理与 G 代码生成

#### 阶段 4：激光加工控制

- ACS 控制器接入
- 设备连接状态与报警处理
- 加工流程执行控制（启动/暂停/停止/急停）
- 运动仿真与实时状态反馈

#### 阶段 5：工程化与质量

- 单元测试与集成测试
- 崩溃保护与日志体系
- 项目配置管理与参数持久化
- 安装包与部署流程

## 参考

- Mayo 架构参考：F:/wangchao/Yunco3D-v1.0/src
- 流程模块参考：F:/wangchao/Yunco1.1/trunk/librecad/src/Process
