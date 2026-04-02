一、目标架构

项目按业务拆分为四层：

1、app
- 仅保留 MainWindow、Ribbon、QAction/Command 壳、对话框和各类 widget
- 不直接承载核心业务逻辑
- Ribbon 中尽量只调用模块接口

2、modules
- 新增业务模块层，负责承接 app 与 base/gui 之间的业务编排
- 当前规划并已开始实施以下模块：
	- CadModule：文档管理、建模操作、文档页管理
	- CamModule：机台管理、工件挂载、刀路、仿真、准备页管理
	- ProcessModule：加工流程、外设、参数、执行页管理
	- ShapeService：共享几何操作服务，供 CAD/CAM 共用

3、gui / graphics
- GuiApplication、GuiDocument、GraphicsScene 继续负责视图与显示桥接
- 每个文档维持独立 view，机台文档保留独立工作视图

4、base
- 保持纯数据和算法职责
- 包含 LcncApplication、LcncDocument、MachineKinematics、LaserToolpath、FaceClassifier、IKSolver、TaskManager 等

二、模块职责

1、CAD 模块
- 负责工件文档的新建、打开、保存、关闭、导入导出
- 负责工件的建模、编辑、布尔运算、删除、拆解等能力
- 负责文档 tab 页及其内容组织
- 面向 UI 暴露统一接口，命令层只采集参数并调用模块 API

2、CAM 模块
- 负责机台模型加载、轴系配置、工件挂载
- 负责刀路生成、引刀设置、刀路显示、运动仿真
- 负责准备 tab 页
- 拥有独立机台文档及对应 view
- CAM 内需要的平移/旋转/删除等常规几何操作统一复用 ShapeService，不重复实现

3、Process 模块
- 负责加工流程、外设、工艺参数及执行状态管理
- 负责执行 tab 页
- 当前先建立统一接口和状态容器，后续逐步接入实际设备与执行流程

4、共享子模块
- 日志、TaskManager、参数配置、后续设备管理等作为独立共用模块/服务存在

三、单例策略

以下模块采用单例是可行的：

1、CadModule
- 对应整个应用唯一的 CAD 文档管理入口

2、CamModule
- 对应整个应用唯一的机台工作区、刀路状态与仿真器

3、ProcessModule
- 对应整个应用唯一的执行状态和控制器连接状态

4、ShapeService
- 不做单例，采用无状态静态工具服务即可

四、CAM 复用 CAD 能力方案

CAM 中的工件和机台零件同样会用到平移、旋转、删除等常规几何操作。

处理方式：
- 将这些通用操作从 commands_cad.cpp 中抽离到 ShapeService
- ShapeService 直接操作 LcncDocument + TDF_Label，不关心当前来自 CAD 还是 CAM
- CadModule 和 CamModule 都通过 ShapeService 调用同一套实现

这样可以保证：
- 不重复实现几何编辑逻辑
- 不出现 CAD / CAM 同功能行为不一致的问题
- 后续如果增加缩放、镜像、阵列等，也可以继续统一沉到 ShapeService

五、当前已完成的首轮实施

1、新增 src/modules/
- shape_service.h / shape_service.cpp
- cad_module.h / cad_module.cpp
- cam_module.h / cam_module.cpp
- process_module.h / process_module.cpp

2、已接入的能力
- CMakeLists.txt 已纳入 modules 层编译
- IAppContext / AppContext 已可访问 CadModule、CamModule、ProcessModule
- CmdUndo / CmdRedo 已委托 CadModule
- 文件命令已开始通过 CadModule 统一承接打开、导入、保存、导出、关闭流程
- CAD 命令（创建、平移、旋转、删除、拆解）已继续收敛到 CadModule / ShapeService
- 机台命令已开始委托 CamModule
- CAM 命令已开始委托 CamModule
- 工具路径共享访问 CamToolpathAccess 已切换为从 CamModule 读取状态
- MainWindow 中机台轴位置和刀路参数/轮廓状态的直接写底层逻辑已开始改为通过 CamModule 路由
- CadModule 已补上文档级事务封装，避免创建/变换/删除/拆解丢失 Undo 语义

3、当前状态
- 工程已可成功编译
- 现阶段属于“第一轮模块落地 + 文件/CAD 命令继续内聚 + MainWindow 编排继续收口”

六、后续继续迁移项

1、继续瘦化 commands_machine / commands_cam，清理 commands_cam 中遗留的旧静态状态代码
2、将 MainWindow 中剩余的文档树、可见性、选择联动等业务编排进一步迁入 CadModule / CamModule / ProcessModule
3、补齐 ProcessModule 与执行页、外设控制、工艺参数的真实集成
4、补齐文档页、准备页、执行页的模块内聚管理
5、按模块继续梳理日志、任务、参数等共用子模块
