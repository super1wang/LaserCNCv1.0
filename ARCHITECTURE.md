# LaserCNC 架构说明

本文描述 2026-07-17 源码基线的实际架构。若本文与阶段性设计记录冲突，以源码、`CMakeLists.txt` 和本文为准。

## 1. 系统定位

LaserCNC 是 Windows 单进程桌面应用，将 CAD、CAM 与五轴激光加工执行整合在同一工作区。技术栈为 C++17、Qt 6、OpenCASCADE/XCAF、SARibbon、toml11、spdlog 与 QuaZip。

核心设计目标：

- 数据所有权集中在 `core`，业务模块只借用数据并执行领域操作。
- OCC 几何止于 CAD/CAM/view 边界，Process 只消费无 OCC 类型的 DTO。
- 每个打开工程拥有独立数据和视图生命周期；机台模型是跨工程的独立参考资产。
- 异步任务、硬件 IO 和 GUI 对象必须有明确的停止、等待与反向析构顺序。

## 2. 分层与依赖

允许的主要依赖方向为：

```text
app / modules/*/ui / commands
            |
            v
modules facade / services
            |
            v
view --------+--------> core
                         ^
core/algorithms ----------+
```

硬约束：

- `core/**` 不得包含 `view/`、`modules/`、`app/`。
- `view/**` 不得包含 `modules/`、`app/`。
- `core/algorithms/**` 只使用 OCC、数学类型和轻量参数，不依赖 UI、文档所有者或 `Kernel`。
- `src/modules/process/**` 不得出现 `TopoDS_*`、`AIS_*`、`gp_*`、`Geom_*`、`BRep*`、`XCAF*`。
- 跨模块同步读取使用接口或服务；状态通知使用 Qt signal 或 `EventBus`。

CMake 将源码拆为 `lcnc_core`、`lcnc_view`、`lcnc_module_cad`、`lcnc_module_cam`、`lcnc_module_process`、`lcnc_app` 六个静态库，最终链接到 `LaserCNC`。

## 3. Kernel 与生命周期

`lcnc::Kernel` 是进程内唯一入口，由 `main()` 按值创建。它拥有：

| 对象 | 所有权与职责 |
| --- | --- |
| `ServiceRegistry` | 以接口类型注册服务；模块停止后清空。 |
| `EventBus` | 类型化同步事件总线。 |
| `ModuleRegistry` | 模块拓扑排序、`init/start/stop` 与反向停止。 |
| `AppSettings` | 应用级配置。 |
| `LcncProjectManager` | 多工作区及工程数据生命周期。 |
| `MachineWorkspace` | 跨工程常驻的机台参考资产。 |
| `TaskManager` | `QtConcurrent` 后台任务、进度和协作式取消。 |
| `MachineConfigurationService` | 机台轴与硬件配置事实源。 |

`GuiApplication` 由 `main()` 拥有后以非拥有指针注入 Kernel，`CommandContainer` 由 `MainWindow` 拥有并注入。正常关闭顺序为：

```text
停止业务模块
  -> 等待后台任务和硬件轮询退出
  -> 销毁 MainWindow
  -> 销毁 GuiApplication / GuiDocument
  -> 销毁 Kernel 数据服务
  -> 最后关闭 Logger 和 QApplication
```

模块依赖固定为 `cad -> cam -> process`。配置禁用上游模块时，下游模块不得启动。

## 4. 工程、文档与数据域

当前并非“三份独立工程文档”。一个 `ProjectWorkspace` 实际拥有：

- 一个统一的项目 `LcncDocument`：同时保存 Workpiece 与 CAM 的 XCAF 实体。
- 一个 `CamDataManager`：保存稠密刀路点、轮廓、图层、排序、引线、生成参数和运行时 CAM 状态。
- 一个 `LcncProjectSession`：保存工程名、路径、manifest、保存选项与 dirty flags。

逻辑数据域与物理存储关系如下：

| 逻辑域 | 物理所有者 | 说明 |
| --- | --- | --- |
| Workpiece | `ProjectWorkspace::m_projectDocument` | 源几何及 CAD 建模结果，实体标记为 Workpiece/Auxiliary。 |
| CAM 稀疏几何 | 同一 `m_projectDocument` | 轮廓 wire 以 CAM `EntityKind` 存入 XCAF。 |
| CAM 稠密数据 | `ProjectWorkspace::m_camData` | 不放入 OCC 文档，使用稳定 `ContourId` 与 `xcafEntry` 关联。 |
| Machine | Kernel 拥有的 `MachineWorkspace` | 独立参考资产，不写入工程包，不应使工程变脏。 |

`workpieceDocument()` 与 `camDocument()` 当前返回同一项目文档；这是统一存储的兼容 API，不代表两个独立实例。机台文档只以非拥有引用登记到 `LcncProjectManager`，用于域路由。

`LcncProjectManager` 维护多个 `ProjectWorkspace`。单文档模式打开新文件时替换全部旧工作区；多文档模式允许并存并切换活动工作区。

## 5. View 架构

`GuiApplication` 为每个 `ProjectWorkspaceId` 创建并拥有一个 `GuiDocument`。`MainWindow` 为工作区创建对应 `WidgetOccView` 并在活动工作区切换时切换视图。

每个 `GuiDocument` 包含：

- `GraphicsScene`、`AIS_InteractiveContext`、`V3d_Viewer/V3d_View`。
- `RenderingManager` 与渲染配置。
- 以 `{DocumentId, XCAF entry}` 为身份的 AIS 显示注册表。
- CAM 轮廓、辅助对象、机台和工件的逻辑域/实体类型标记。

域刷新必须是差量操作。纯工作区切换只恢复显示，不得把显示恢复误报成数据修改。`fitAll` 优先可见工件，其次可见 CAM，辅助轴线和坐标系不参与包围盒。

## 6. CAD 模块

`CadModule` 负责 Workpiece 域的导入、导出、建模、草图、特征、变换、删除和选择路由。复杂 OCC 运算应位于 `core/algorithms/cad`，文档写回和刷新由模块/service 协调。

异步导入通过 `TaskManager` 在 detached workspace 或明确的目标文档上计算，完成后再由主线程采纳结果；后台任务不得直接持有生命周期不受保护的 UI 对象。

## 7. CAM 模块

`CamModule` 借用项目的统一 XCAF 文档、`CamDataManager`、机台工作区和机台配置服务，负责轮廓提取、离散、刀路求解、图层、排序、引线与渲染协调。

业务边界为：

```text
生成：提取轮廓 + 离散点集
  -> 标记 CAM 数据 dirty
  -> 用户显式重新计算/排序
  -> 五轴刀路求解与顺序应用
```

Process 不读取 OCC。CAM 通过 `ICamToolpathProvider` 输出 `ToolpathExportSnapshot`，其中只包含控制器无关的点、姿态、轮廓和工艺引用。

## 8. Process 模块

当前 `ProcessModule` 仍是较大的 facade/coordinator，实际拥有或协调：

- typed settings 与参数/IO 模型。
- `Service` 及运动控制器、激光器、工具表。
- `ProcessCuttingPlanService`、`NormalCuttingManager` 与 motion sink。
- 流程文档、步骤插件注册表和 `ProcessWorkflowExecutor`。
- 加工前置检查、状态、连接/断开、回零、急停、轮询和监控。

执行链路为：

```text
CAM ToolpathExportSnapshot
  -> ProcessToolpathService / ProcessCuttingPlanService
  -> NormalCuttingManager
  -> MotionSinkFactory
  -> PureSimulation / ACS text / GTN buffered sink
  -> 运动控制器与激光/IO
```

`runStart()` 是加工硬门禁：流程、CAM dirty 状态、图层/工具映射、控制器/激光器、轴、IO 与监控条件必须正常才允许进入加工。任何 Error、EmergencyStop 或停止路径必须关闭激光与吹气等安全输出。

硬件 SDK 类型只能存在于私有实现。当前旧 `Service`、静态控制器实例和双日志系统仍是待收口的技术债，详见 `todo.md`。

## 9. 异步与内存安全规则

- C++ 所有权优先使用 `unique_ptr/shared_ptr`；OCC 使用 `Handle`；QObject 使用父子树。
- 非拥有裸指针必须由更长生命周期对象保证，并在异步边界前转换为快照、受控句柄或在关闭时等待。
- `TaskManager` 析构会先请求取消，再等待全部 worker 退出，最后释放任务实体。
- Process 硬件轮询和环境监控在停止时等待当前 future 完成，控制器对象不得先于 worker 销毁。
- Qt GUI 只能在主线程访问；供应商 SDK 是否线程安全不能假定，硬件调用最终应串行化到单一设备执行上下文。
- 所有 catch 必须记录错误；析构和停止路径不得向外抛异常。

静态审计和一次成功构建不能证明“绝无泄漏”。发布门槛必须包含 ASan/Application Verifier、长时间开关工程/连接设备/仿真循环和退出压力测试。

## 10. `.lcnc` 工程包

工程包由 QuaZip 在 staging 目录中事务式生成，当前格式版本为 3：

```text
project.toml
workpiece.xbf
cam_toolpath.toml
cam_toolpath_points.bin
```

- `workpiece.xbf` 同时含 Workpiece/Auxiliary 与 CAM 类型实体。
- `cam_toolpath.toml` 保存图层、轮廓、签名、引线、排序和生成参数等元数据。
- `cam_toolpath_points.bin` 保存稠密采样点。
- 机台模型不进入工程包。
- v1/v2 读取与旧 `process_cutting_plan.toml` 迁移暂时保留；移除前必须先确定产品支持窗口并提供离线升级工具。

## 11. 配置与可选硬件

Qt SerialPort 是当前 `LaserDevice -> SerialPort` 继承链的必需依赖。ACS、GTN、BDAQ 和真实激光器由 CMake 选项控制；可选源文件和供应商库必须在同一个开关下成对加入，禁止“选项关闭但实现仍无条件编译/链接”。

个人 SDK 路径只允许作为 CMake cache 默认值，长期应迁移到 `CMakePresets.json` 或本机 preset。每种硬件组合必须有独立配置/编译验证。

## 12. 维护准则

- 架构事实只维护在本文；阶段计划只维护在 `todo.md`。
- 不保留未接入运行时、仅用于演示的平行框架。
- 兼容读取必须有版本、测试样本、删除条件和截止版本；禁止无限期双写。
- 单文件持续超过约 1000 行时，应按业务服务拆分，禁止继续扩张 facade。
- 每次提交至少执行分层扫描、过时 API 扫描、Process OCC 扫描、`git diff --check` 和 Debug 构建。
