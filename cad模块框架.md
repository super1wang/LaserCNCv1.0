# CAD 模块框架

> 目标：重新梳理当前 CAD 模块代码状态，并给出后续可扩展框架。核心原则是：CAD 建模相关功能先抽到 `core/algorithms/cad/**`，再封装为命令；文档拥有独立草图管理器；TaskPanel 不再硬编码按钮，而是通过指令注册表和选择类型解析器动态展示可用工具。

## 1. 当前代码状态判断

### 1.1 已经合理的部分

- 算法层已有雏形：`src/core/algorithms/cad/` 已包含 `primitives`、`sketch`、`features`、`boolean_ops`、`measure`、`transform_ops`，基础体、草图 wire/face、拉伸/旋转、布尔、测量和形体变换已脱离 UI 和文档层。
- `CadModule` 已作为微内核业务模块存在，同时实现 `IModule` 和 `ICadFacade`，承担文档生命周期、建模编排、信号通知和视图刷新入口。
- 命令体系已存在：`commands_file`、`commands_edit`、`commands_cad` 已接入 `CommandBase + CommandContainer`，Ribbon 和部分 TaskPanel 按 command id 触发。
- 草图从临时 profile 迈向文档级数据：`SketchManagerRegistry` 按 `DocumentId` 管理 `SketchManager`，`SketchRecord` 保存草图 plane、elements、profileFace、visible、used-by-feature 状态。
- CAD 文档状态层已落地：`CadDocumentRegistry` 按 `DocumentId` 管理 `CadDocumentState`，每个状态聚合 `SketchManager`、选择上下文和展示状态。
- TaskPanel 首页已迁为 `CadToolRegistry + CadToolFilter` 驱动，基础体、草图特征、实体编辑、布尔、测量、删除均通过 descriptor 描述；未接通的面/边工具暂不暴露。
- MainWindow 的 CAD 文档树构造已迁到 `CadModelTreeAdapter`；MainWindow 只负责把 tree snapshot 渲染为 `QTreeWidgetItem`。
- TaskPanel 参数页的基础体和草图特征 preview/apply 已接入 `CadCommandDispatcher + ICadToolCommand`。
- 移动/旋转已收敛为 TaskPanel 一个“变换”入口：右侧 TransformPage 不再选择模式，只提供参考基准、XYZ 平移、XYZ 旋转、预览开关和应用/取消；默认参考为选中模型中心，参数变化只更新 preview，应用时才写回 XCAF 文档。
- View 层新增轻量 `TransformGizmoRenderer`，不依赖 modules；进入变换工具后在 OCC view 中显示 XYZ 平移轴和旋转环，拖拽 gizmo 会回写 TaskPanel 参数并触发预览刷新，旋转拖拽单次增量限制在 90 度以内。
- 草图 view overlay 已接入：`SketchOverlayRenderer` 负责 AIS overlay 显示、颜色、命中反查和可拖拽标记，`WidgetOccView` 发出 overlay 选择与拖拽信号，MainWindow 把 overlay key 转成 `CadSelectionContext`。编辑中草图支持整元素拖拽与手柄级拖拽；完成草图只参与点选，不再误触发拖拽。
- `commands_cad.cpp` 已完成按域拆分，草图命令、view 辅助命令、基础体入口、变换、布尔、测量、删除/拆解均迁到独立 cpp；旧文件仅作为兼容空壳保留，公共实体选择 helper 已下沉到 `command_helpers`。

### 1.2 当前主要问题

- `CadModule` 仍偏胖：文件导入导出、文档树快照、选择同步、ShapeService 调用、草图工作流、overlay snapshot、预览构建和视图刷新都集中在一个类中。短期可接受，长期应继续抽到 document/editor/selection/sketch controller。
- CAD 命令已按域拆分，TaskPanel 中移动/旋转已统一为“变换” TransformPage + gizmo 交互；Ribbon/旧 command 路径仍保留移动、旋转 `QDialog` 作为兼容入口。布尔、测量、删除、拆解仍保留旧 `QDialog/QMessageBox` 参数/结果界面，下一步应迁到 TaskPanel tool command 和参数 schema。
- TaskPanel 已具备 registry/filter，并且首页只显示当前上下文可用工具；参数页仍是固定 primitive/feature/sketch 页面。后续若要完全 descriptor/schema 化，需要继续引入 `cad_parameter_schema` 和 page factory。
- MainWindow 已移出文档树构造和参数页执行细节，但仍承担信号 wiring、TaskPanel 列表同步和 OccView preview 展示；后续可继续收敛到 `CadSelectionController` 和 preview renderer。
- 草图 overlay 已从整元素 MVP 推进到手柄级拖拽：支持线段端点、圆弧起点/中点/终点、圆心/半径、矩形中心/角点、多边形中心/半径等 handle；尚未支持约束联动、尺寸驱动求解和端点合并后的拓扑约束维护。
- 面/边/顶点的 selection domain 已准备好，但 `WidgetOccView` 还没有稳定回传 `SubShapeFace/SubShapeEdge/SubShapeVertex` 的 `CadSelectionItem`，因此 FaceTool/EdgeTool 暂不在 TaskPanel 暴露。
- `WidgetModelTree` 直接 include `MachineKinematics`，CAD UI 暴露 CAM/机台概念；这是历史耦合，后续应通过 adapter 或外部 provider 剥离。

## 2. 目标分层

CAD 模块建议收敛为下面的职责链：

```text
TaskPanel / Ribbon / ModelTree / OccView
        ↓
CadToolRegistry + CadSelectionTypeResolver
        ↓
CadToolCommand / CommandBase adapter
        ↓
CadModule / CadDocumentState / CadDocumentEditor
        ↓
core/algorithms/cad/**
        ↓
LcncDocument / GuiDocument / view renderer
```

关键规则：

1. **算法先行**：任何几何计算、拓扑构造、布尔、测量、草图约束、形体变换都先进入 `core/algorithms/cad/**`。
2. **命令封装**：所有建模动作都封装为 command/tool command，Ribbon、TaskPanel、快捷键、上下文菜单都只激活同一个命令。
3. **模块编排**：`CadModule` 只做文档准备、算法调用、写回 `LcncDocument`、刷新/通知，不直接承载 UI 表单逻辑。
4. **文档独立状态**：每个工件文档有独立 `CadDocumentState`，内部持有 `SketchManager`、后续 `FeatureManager`、选择状态和展示状态。
5. **数据和展示分离**：草图管理器是数据真源；模型树和 view 是投影，不应由管理器直接 new 树节点或 AIS 对象。

## 3. 建议目录结构

```text
src/core/algorithms/cad/
  primitives.{h,cpp}          基础体算法
  sketch.{h,cpp}              草图 wire/face 构造
  features.{h,cpp}            拉伸/旋转/扫掠等特征算法
  boolean_ops.{h,cpp}         布尔算法
  measure.{h,cpp}             测量算法
  transform_ops.{h,cpp}       移动/旋转/镜像/阵列等形体变换算法
  topology_ops.{h,cpp}        拆解、子形体提取、面/边查询
  sketch_constraints.{h,cpp}  草图约束求解与约束状态分析

src/modules/cad/
  cad_module.{h,cpp}          CAD facade + 文档/命令编排
  i_cad_facade.h              对外门面

  document/
    cad_document_state.{h,cpp}      每文档 CAD 状态
    cad_document_registry.{h,cpp}   DocumentId -> CadDocumentState
    cad_document_editor.{h,cpp}     open/commit/abort command + XCAF 写回封装

  sketch/
    sketch_types.h                  SketchRecord / SketchElement / ids
    sketch_manager.{h,cpp}          每文档草图数据管理
    sketch_serializer.{h,cpp}       草图持久化/undo redo 快照
    sketch_transaction.{h,cpp}      编辑草图、完成、取消、删除的事务对象

  selection/
    cad_selection.h                 统一选择对象 DTO
    cad_selection_resolver.{h,cpp}  view/modeltree selection -> CadSelectionContext
    cad_selection_controller.{h,cpp}树/view/TaskPanel 联动

  task/
    cad_tool_descriptor.h           工具元数据
    cad_tool_registry.{h,cpp}       工具注册表
    cad_selection_type_resolver.{h,cpp}
    cad_tool_filter.{h,cpp}         根据选择上下文过滤工具
    cad_parameter_schema.h          参数页描述模型

  commands/
    command_helpers.{h,cpp}
    commands_document.{h,cpp}
    commands_cad_primitives.cpp
    commands_cad_sketch.cpp
    commands_cad_view.cpp
    commands_features.{h,cpp}
    commands_transform.{h,cpp}
    commands_boolean.{h,cpp}
    commands_measure.{h,cpp}
    commands_delete.{h,cpp}

  ui/
    widget_cad_task_panel.{h,cpp}   只渲染 registry 解析后的工具和参数页
    cad_task_page_factory.{h,cpp}   descriptor/schema -> QWidget 参数页
    widget_model_tree.{h,cpp}       只显示 tree adapter 输出，不解析业务
    cad_model_tree_adapter.{h,cpp}  document/sketch/feature -> tree snapshot

src/view/
  cad_preview_renderer.{h,cpp}      临时 AIS 预览
  sketch_overlay_renderer.{h,cpp}   草图元素 AIS overlay、颜色、命中映射
  transform_gizmo_renderer.{h,cpp}  变换工具 AIS 轴/旋转环、命中映射
```

说明：`view/**` 不 include `modules/**`。`sketch_overlay_renderer` 使用 view 层自己的轻量 DTO，例如 `SketchOverlayItem`、`SketchVisualKey`；`modules/cad` 负责把 `SketchRecord` 转成这些 DTO。

## 4. 文档级草图管理

### 4.1 CadDocumentState

当前 `SketchManagerRegistry` 已经按文档管理草图，但建议明确升级为文档状态对象：

```cpp
class CadDocumentState {
public:
    DocumentId documentId() const;
    SketchManager& sketchManager();
    const SketchManager& sketchManager() const;
    CadSelectionState& selectionState();
    CadPresentationState& presentationState();
};
```

`CadModule` 持有 `CadDocumentRegistry`，在 `documentAdded/documentClosed` 时创建/释放 `CadDocumentState`。这样后续 FeatureManager、草图序列化、undo/redo、显示缓存都能挂到文档状态，而不是继续往 `CadModule` 塞成员变量。

### 4.2 SketchManager 的职责

`SketchManager` 建议只负责草图数据生命周期：

- 新增、删除、重命名草图。
- 维护 `SketchRecord`、`SketchElement`、约束、尺寸、plane、profileFace、usage、feature references。
- 提供稳定 id：`SketchId`、`SketchElementId`。
- 维护 visible/locked/construction/usedByFeature 等业务状态。
- 生成快照：`SketchSnapshot`、`SketchTreeSnapshot`、`SketchOverlaySnapshot`。
- 提供序列化和 undo/redo 所需的纯数据快照。

`SketchManager` 不建议直接负责：

- 创建 `QTreeWidgetItem`。
- 直接持有或显示 `AIS_Shape`。
- 直接操作 `GuiDocument` 或 `WidgetOccView`。
- 弹窗、按钮、TaskPanel 页面切换。

原因：草图管理器是文档数据层，生命周期和 undo/redo/持久化绑定；模型树和 view 是展示层，生命周期、刷新频率、选择模式都不同。让管理器统一管理树和 view 会让数据层依赖 UI，破坏分层，也会让测试和序列化变困难。

## 5. 模型树与 View 可视化如何协作

### 5.1 两种方案对比

| 方案 | 描述 | 优点 | 风险 |
| --- | --- | --- | --- |
| 管理器统一管理 tree + view | `SketchManager` 创建树节点、AIS overlay，并处理选择 | 状态看似集中 | 违反分层；数据层依赖 Qt widget/AIS；undo/redo 和持久化难；view 切换生命周期复杂 |
| tree/view 分别管理投影 | `SketchManager` 只发数据快照；`CadModelTreeAdapter` 和 `SketchOverlayRenderer` 分别渲染 | 分层清晰；易测试；树/view 可独立刷新；符合微内核边界 | 需要统一 selection key 和同步控制器 |

结论：采用第二种。`SketchManager` 是唯一数据真源；模型树和 view 分别持有自己的展示对象，并通过统一 selection key 联动。

### 5.2 推荐同步链路

```text
SketchManager 数据变化
      ↓ emits sketchChanged(docId, sketchId, reason)
CadModelTreeAdapter 生成树节点 snapshot
SketchOverlayController 生成 view overlay snapshot
      ↓
WidgetModelTree 更新节点
SketchOverlayRenderer 更新 AIS overlay
```

选择联动通过统一选择对象完成：

```cpp
enum class CadSelectionDomain {
    DocumentShape,
    SubShapeFace,
    SubShapeEdge,
    SubShapeVertex,
    Sketch,
    SketchElement,
    SketchConstraint,
    Feature
};

struct CadSelectionItem {
    DocumentId docId{kInvalidDocumentId};
    CadSelectionDomain domain{CadSelectionDomain::DocumentShape};
    QString entry;          // XCAF label entry, when selection comes from document shape
    int sketchId{0};        // when selection comes from sketch manager
    int sketchElementId{0}; // optional
    int subShapeIndex{-1};  // optional face/edge/vertex index or persistent naming key
};
```

模型树节点保存 `CadSelectionItem` 的序列化 key；view renderer 维护 `AIS object -> CadSelectionItem` 的反查表。树选中草图节点时，`CadSelectionController` 通知 renderer 高亮对应 overlay；view 选中 overlay 时，控制器通知模型树定位到对应节点。

### 5.3 草图数据类文档化管理

草图应按“文档内对象”管理，而不是 TaskPanel 临时数据：

- 每个 `SketchRecord` 拥有稳定 id、名称、plane、元素列表、约束列表、尺寸参数、派生 profileFace、显示状态。
- 每个草图元素拥有稳定 id 和几何类型；线、圆、圆弧、多边形等都能独立选择、约束、隐藏、删除。
- 草图被特征引用时，Feature 记录 source sketch id；草图 `usage=UsedByFeature`，默认隐藏但仍保留在文档状态中。
- 后续持久化时，把草图记录写入 XCAF 扩展属性或独立文档元数据；undo/redo 记录 `SketchManager` 快照差异。
- view 中草图不是直接写入 XCAF 的实体，而是由 `SketchOverlayRenderer` 根据 `SketchRecord` 构建的可选择 AIS overlay；只有应用特征后生成的实体才写入 XCAF。

## 6. TaskPanel 指令注册表

### 6.1 CadToolDescriptor

TaskPanel 不应硬编码按钮，而应显示工具注册表解析出的结果：

```cpp
enum class CadToolCategory {
    Document,
    BaseModeling,
    SketchCreate,
    SketchEdit,
    SketchFeature,
    FaceTool,
    Transform,
    Boolean,
    Measure,
    Delete
};

struct CadToolDescriptor {
    QString toolId;            // e.g. cad.feature.extrude
    QString commandId;         // existing CommandContainer id or tool command id
    QString title;
    QString icon;
    CadToolCategory category;
    QList<CadSelectionDomain> acceptedDomains;
    int minSelectionCount{0};
    int maxSelectionCount{1};  // -1 means unlimited
    bool requiresDocument{false};
    bool requiresSketchEditing{false};
    QString parameterPageId;   // empty means execute immediately
    int order{0};
};
```

所有建模命令在启动时注册 descriptor：

```text
CmdCreateBox        -> cad.primitive.box        BaseModeling, no selection
CmdBeginSketch      -> cad.sketch.begin         SketchCreate, document/no selection
CmdExtrudeSketch    -> cad.feature.extrude      SketchFeature, Sketch/ClosedProfile
CmdBooleanUnion     -> cad.boolean.union        Boolean, two DocumentShape
CmdTransformShape   -> cad.transform            Transform, DocumentShape/Sketch/SketchElement
CmdOffsetFace       -> cad.face.offset          FaceTool, SubShapeFace
CmdTrimSketch       -> cad.sketch.trim          SketchEdit, SketchElement
```

### 6.2 CadSelectionTypeResolver

类型解析器把当前选择转换为上下文：

```cpp
struct CadSelectionContext {
    DocumentId docId{kInvalidDocumentId};
    QList<CadSelectionItem> items;
    bool hasDocument{false};
    bool sketchEditing{false};
    bool hasClosedSketchProfile{false};
};
```

解析来源包括：

- `GuiDocument::selectedEntries()`：普通 XCAF 实体。
- `WidgetOccView` 子形体 pick：面、边、顶点。
- `SketchOverlayRenderer` 命中表：草图节点、草图元素、约束。
- `WidgetModelTree` 当前节点：文档、实体、草图、草图元素、特征。

解析器输出统一 `CadSelectionContext`，`CadToolFilter` 再根据 descriptor 过滤可用工具，TaskPanel 只负责显示。

### 6.3 TaskPanel 展示策略

TaskPanel 首页显示 `CadToolFilter::resolveTools(context)` 的结果，按 category 分组，类似截图中的“建模工具 / 面工具”：

- 无选择：文档、新建草图、基础体。
- 选中普通实体：变换、删除、拆解、测量；多选实体时显示布尔并/差/交。
- 选中实体面：在实体工具基础上追加面工具，如新建草图到面、局部拉伸、切除、抽壳、拔模、倒角、圆角、面积测量。
- 选中边：倒角、圆角、测距、投影到草图。
- 选中草图节点：编辑草图、拉伸、旋转、显示/隐藏、删除、复制、重命名。
- 选中草图元素：约束、尺寸、修剪、延伸、转换构造线、删除、隐藏。
- 选中草图特征：编辑参数、显示源草图、抑制/恢复、删除。

## 7. 命令封装方式

现有 `CommandBase::execute()` 不带参数，适合 Ribbon/快捷键“激活工具”。TaskPanel 参数化工具需要一个更通用的 tool command：

```cpp
struct CadCommandRequest {
    CadSelectionContext selection;
    QVariantMap params;
    bool preview{false};
};

class ICadToolCommand {
public:
    virtual CadToolDescriptor descriptor() const = 0;
    virtual bool preview(const CadCommandRequest& request, TopoDS_Shape* out, QString* err) = 0;
    virtual bool execute(const CadCommandRequest& request, QString* err) = 0;
};
```

推荐关系：

- `CommandBase` 仍用于 QAction、Ribbon、快捷键，职责是激活工具或执行无参数命令。
- `ICadToolCommand` 用于 TaskPanel 参数页的 preview/apply。
- 一个工具可以同时有 QAction adapter 和 TaskPanel descriptor，二者共享同一个 tool id。
- 参数页不直接调用 `CadModule::createPrimitive/applyFeature`，而是构造 `CadCommandRequest` 交给 `CadCommandDispatcher`。

执行链路：

```text
TaskPanel 点击工具
  -> CadToolRegistry 找到 descriptor
  -> 无参数：CommandContainer.execute(commandId)
  -> 有参数：打开参数页
  -> 参数变化：ICadToolCommand.preview(request)
  -> 应用：ICadToolCommand.execute(request)
  -> command 内调用 cad_algo + CadDocumentEditor 写回文档
```

## 8. 算法优先的落地规则

新增任何建模工具按以下顺序实现：

1. 在 `core/algorithms/cad/**` 增加自由函数或轻量参数结构，只处理 OCC/标量/错误输出。
2. 在 `modules/cad/commands/**` 增加 tool command，负责选择校验、参数校验、调用算法、写回文档。
3. 在 `CadToolRegistry` 注册 descriptor，声明适用选择类型和参数页。
4. TaskPanel 自动显示该工具；Ribbon 如需入口，只增加 QAction adapter。
5. view 预览只通过 `cad_preview_renderer`、`sketch_overlay_renderer` 或 `transform_gizmo_renderer`，不在 command 或 module 中手动持有 AIS 细节。

示例：变换已按下面方向落地，后续只需继续把 Ribbon 旧移动/旋转命令迁到同一 tool command：

```text
cad_algo::transformShape(shape, params) -> TopoDS_Shape
TransformPage + TransformGizmoRenderer -> 平移/旋转参数与拖拽输入
CadModule::buildTransformPreview/applyTransform -> 预览 compound / 提交替换 XCAF shape
```

布尔应拆成：

```text
cad_algo::fuse/cut/common -> TopoDS_Shape
BooleanToolCommand -> 解析两个实体，执行算法
CadDocumentEditor -> 写入结果，并按策略保留/隐藏/删除源实体
```

面工具应拆成：

```text
cad_algo::extractFace / offsetFace / extrudeFace / cutFromFace
FaceToolCommand -> 解析 SubShapeFace + 参数
CadDocumentEditor -> 写入新特征或替换实体
```

## 9. 关键类职责建议

| 类/模块 | 目标职责 | 不应承担 |
| --- | --- | --- |
| `CadModule` | facade、文档状态注册、命令调度、信号转发、文档写回入口 | UI 表单、树节点构造、大段 OCC 算法、AIS 细节 |
| `CadDocumentState` | 每文档 CAD 状态聚合：SketchManager、FeatureManager、selection/presentation state | QWidget/AIS 创建 |
| `SketchManager` | 草图数据、约束、可见性、usage、快照、序列化 | modeltree 节点、view 对象、弹窗 |
| `CadModelTreeAdapter` | 文档实体、草图、特征转 tree snapshot | 修改草图数据、执行命令 |
| `SketchOverlayRenderer` | 草图 overlay 显示、颜色、命中反查 | 保存权威草图数据 |
| `CadSelectionController` | 树/view/TaskPanel 选择联动，生成 `CadSelectionContext` | 几何算法 |
| `CadToolRegistry` | 注册工具元数据，按类别和 id 查找 | 执行几何算法 |
| `CadToolFilter` | 根据选择上下文过滤可用工具 | UI 构造 |
| `ICadToolCommand` | 参数校验、调用算法、写回文档、预览 | 弹出临时 QDialog |
| `WidgetCadTaskPanel` | 渲染工具列表和参数页，发出 tool request | 硬编码业务按钮、解释 OCC/文档类型 |

## 10. 迁移路线

### Phase C1：稳定选择模型

- 新增 `CadSelectionItem / CadSelectionContext`。
- MainWindow、ModelTree、WidgetOccView、SketchOverlayRenderer 都统一使用 selection item。
- 当前 `selectedEntries(QStringList)` 保留，但包装为 DocumentShape selection。

当前状态：已新增 `src/modules/cad/selection/cad_selection.h` 和 `cad_selection_resolver.{h,cpp}`，提供 `CadSelectionDomain`、`CadSelectionItem`、`CadSelectionContext`，并把 MainWindow 文档树点击和 view 普通实体选择包装为统一选择上下文。OccView 子形体 pick 与 SketchOverlay 命中表仍需后续接入。

### Phase C2：TaskPanel Registry

- 新增 `CadToolDescriptor / CadToolRegistry / CadToolFilter`。
- 先把当前首页硬编码按钮迁成 descriptor：新建文件、打开文件、新建草图、基础体、拉伸、旋转、变换、删除、拆解、测距。
- TaskPanel 改为根据 descriptor 动态建按钮；旧信号保留兼容一轮。

当前状态：已新增 `src/modules/cad/task/cad_tool_descriptor.h`、`cad_tool_registry.{h,cpp}`、`cad_tool_filter.{h,cpp}`，并将 `WidgetCadTaskPanel::buildHomePage()` 的文档、基础建模、草图特征、已选对象、布尔、测量、删除按钮迁为 registry 驱动。过滤逻辑已支持选择 domain、最小/最大选择数量、草图编辑和选中草图约束；草图编辑时首页只保留“继续草图”等可用入口；无文档时不再显示基础建模。未实现的 FaceTool/EdgeTool 和扫掠入口已从 TaskPanel 暂时隐藏。

### Phase C3：命令拆分与参数页命令化

- 拆分 `commands_cad.cpp` 为 primitives/sketch/features/transform/boolean/measure/delete。
- 增加 `ICadToolCommand` 和 `CadCommandDispatcher`。
- 基础体、草图特征的 preview/apply 改走 tool command request，MainWindow 不再直接调 `createPrimitive/applyFeature`。

当前状态：已新增 `src/modules/cad/task/cad_command_request.h`、`icad_tool_command.h`、`cad_command_dispatcher.{h,cpp}`。CadModule 持有 dispatcher，基础体和草图特征的 preview/apply 已由 MainWindow 通过 `previewTool/executeTool` 触发 tool command request，不再直接调用 `createPrimitive/applyFeature`。`commands_cad.cpp` 已拆为 `commands_cad_sketch.cpp`、`commands_cad_view.cpp`、`commands_cad_primitives.cpp`、`commands_cad_transform.cpp`、`commands_cad_boolean.cpp`、`commands_cad_measure.cpp`、`commands_cad_delete.cpp`，公共 helper 已迁入 `command_helpers.{h,cpp}`。移动/旋转在 TaskPanel 已统一为 `cad.transform` + `CadToolActivation::TransformPage`，参数变化调用 `CadModule::buildTransformPreview()`，应用调用 `CadModule::applyTransform()`；旧 `CmdMoveShape/CmdRotateShape` 仅作为 Ribbon/兼容入口保留。下一轮应把 boolean/measure/delete 中的临时 QDialog 参数采集迁到 TaskPanel tool command。

### Phase C4：草图 presentation

- 新增 `CadModelTreeAdapter`，把 MainWindow 中草图树节点构造迁出。
- 新增 `SketchOverlayRenderer` 和 `SketchOverlayController`，根据 `SketchManager` 快照显示草图 overlay。
- `setSketchVisible` 同步树 checkbox 与 overlay 显隐。

当前状态：已新增 `src/modules/cad/ui/cad_model_tree_adapter.{h,cpp}`，MainWindow 的文档实体、编辑中草图、完成草图和草图元素树节点构造已迁出；树点击通过 `CadSelectionResolver` 生成上下文，草图 checkbox 直接调用 `setSketchVisible`。`src/view/sketch_overlay_renderer.{h,cpp}` 已从 DTO 骨架升级为 AIS overlay renderer，支持点/线/圆弧/圆/矩形/多边形显示、颜色区分、命中 key 反查、对象清理和 draggable 标记。`WidgetOccView` 已接入 overlay 选择与拖拽信号，并且只允许 active sketch overlay 开始拖拽；`CadModule::sketchOverlaySnapshots()` 将编辑中草图、活动手柄和完成草图转换为 view-neutral snapshot。

补充状态：`src/view/transform_gizmo_renderer.{h,cpp}` 已加入 view 层，负责变换 gizmo 的 AIS 显示、颜色、命中反查和屏幕轴向投影；`WidgetOccView` 发出 `transformGizmoDragMoved(operation, axis, delta)`，MainWindow 把该 delta 写入 TransformPage 参数并触发 preview。旋转拖拽 delta 在 view 与参数层均按单次 90 度限幅；TransformPage 退出首页时统一发出清理信号，取消可可靠清除 preview 和 gizmo。该链路继续保持 view 不 include modules。

### Phase C5：草图约束与类文档化

- 新增 `sketch_constraints` 算法骨架。
- `SketchRecord` 增加 constraints/dimensions。
- 欠约束/完全约束/过约束颜色由 renderer 根据 constraint state 显示。
- 增加 `sketch_serializer`，接入 undo/redo 和后续文件持久化。

当前状态：已新增 `src/core/algorithms/cad/sketch_constraints.{h,cpp}`，提供不依赖 modules/view/app 的约束状态分析骨架；已新增 `src/modules/cad/sketch/sketch_serializer.{h,cpp}`，可将 `SketchRecord` 的标量元数据与元素参数转为 `QVariantMap`。`SketchRecord` constraints/dimensions 字段、颜色显示和 undo/redo 持久化接入尚未完成。

### Phase C6：面/边/子形体工具

- WidgetOccView 支持可稳定回传 face/edge/vertex selection item。
- ToolRegistry 增加 FaceTool/EdgeTool 分类。
- 新增局部拉伸、切除、倒角、圆角、抽壳、孔等工具时严格走算法层 + tool command。

当前状态：`CadToolCategory` 已扩展 `Boolean/Measure/Delete/FaceTool/EdgeTool`，布尔/测量/删除工具已通过 registry/filter 展示。由于 WidgetOccView 尚未稳定生成子形体 selection item，FaceTool/EdgeTool descriptor 暂不注册到默认 TaskPanel，避免显示不可执行入口。后续接通 face/edge/vertex pick 后再恢复面上草图、偏置面、圆角、倒角等工具。

## 11. 最终验收标准

当前实现状态：C1/C2/C3/C4/C5/C6 的框架骨架已进入代码并通过 Debug 构建。本轮已完成草图 AIS overlay、overlay 选择、编辑中草图元素整体拖拽、活动草图手柄拖拽、TaskPanel 可用工具收紧、扫掠/面边未实现入口隐藏、`commands_cad.cpp` 按域拆分，以及统一“变换” TransformPage + view gizmo + preview/apply。C5 的完整约束/undo/redo、C6 的子形体 pick 与具体面/边工具、C3 中 boolean/measure/delete 的旧 QDialog 参数页迁入 TaskPanel tool command 仍属于后续功能深化。

- 新增建模工具不需要改 `WidgetCadTaskPanel::buildHomePage()`，只注册 descriptor 和 command。
- TaskPanel、Ribbon、快捷键、模型树右键菜单都能复用同一 command/tool command。
- 选择实体、面、边、草图节点、草图元素时，TaskPanel 自动显示对应工具。
- 每个工件文档都有独立 `SketchManager`，草图数据随文档创建/关闭/undo/redo 生命周期变化。
- SketchManager 不直接依赖 QWidget/AIS；树和 view 只渲染 manager 快照。
- `core/algorithms/cad/**` 不 include `modules/`、`view/`、`app/`。
- `commands_cad.cpp` 被拆分，单文件职责清晰；命令中不再堆叠大段 OCC 算法和临时 QDialog 表单。
- MainWindow 只保留高层 wiring，CAD 细节迁入 `CadSelectionController`、`CadToolRegistry`、`CadModelTreeAdapter` 和 `CadCommandDispatcher`。
