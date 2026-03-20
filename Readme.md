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
- `LcncDocument`：基于 XCAF 文档的数据封装，支持实体分类和 Undo/Redo
- `XcafUtils`：XCAF 标签名称、Entry、Shape 获取等工具
- `TaskManager` / `TaskProgress`：统一异步任务与进度状态管理

### 2) graphics 层（三维显示）

- `GraphicsScene`：封装 `V3d_Viewer` + `AIS_InteractiveContext`
- `ShapeObjectDriver`：AIS 形体对象样式与材质封装

### 3) gui 层（文档显示桥接）

- `GuiApplication`：`LcncDocument` 与 `GuiDocument` 映射
- `GuiDocument`：按文档维护 3D 场景与 AIS 对象缓存

### 4) app 层（命令与界面）

- `MainWindow`：SARibbon 主窗体与三栏布局
- `CommandBase` + `CommandContainer`：命令模式封装
- 文件命令：新建/打开/保存/导入导出
- 编辑命令：撤销/重做
- 显示命令：视角、显示模式
- 关键控件：
  - `WidgetOccView`：中间 3D 视窗
  - `WidgetModelTree`：准备页模型树
  - `WidgetMachinePanel`：机台管理面板
  - `WidgetLaserControl`：激光控制面板
  - `DialogTaskManager`：任务进度窗口

## 阶段进展

### 已完成阶段

#### 阶段 1：框架搭建与基础交互（已完成）

- 完成整体分层架构（base/graphics/gui/app）
- 完成 Ribbon 主界面和三栏布局
- 完成文档系统（新建、打开、关闭、活动文档切换）
- 完成 STEP/IGES/STL 导入与 STEP 导出骨架
- 完成 Undo/Redo 命令框架
- 完成任务异步框架和进度显示
- 完成中间视窗基础交互（旋转/平移/缩放/选择）
- 完成视图辅助元素：视图方块（右上）和坐标轴（左下）
- 完成左侧新增“文档”标签：显示全部打开文档的装配树
- 修复稳定性问题：
  - 启动时 3D 背景可见
  - 新建后继续新建不崩溃
  - “打开文件”流程与视图切换不崩溃

### 未完成阶段

#### 阶段 2：CAD 功能实装

- 基本体参数化建模（长方体/圆柱/球/圆锥/圆环）
- 草图编辑与约束（线/圆/圆弧）
- 平移/旋转/缩放与布尔操作
- 测量工具（距离/角度/面积）

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
