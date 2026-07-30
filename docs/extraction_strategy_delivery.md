# 轮廓提取策略系统 — 交付文档

**版本**: 1.0
**日期**: 2026-07-27
**分支**: `main`

---

## 1. 概述

本次交付实现了一套完整的**轮廓提取策略系统**，用机台构型+工件装夹姿态驱动的自动识别替代了原有的单一外轮廓提取算法，解决了多孔平板类工件的加工面分离与内孔轮廓提取问题。

### 核心问题

原有算法（`BRepTools::OuterWire` per face）将"外表面"与"外轮廓"混淆，无法识别平面工作面（如多孔板）上的内孔边界（inner holes）。

### 解决方案

- **策略一（主要）**: 机台构型 + wpc 装夹姿态驱动：由 beam 方向（机床坐标系 -Z，经 wpc home 位姿变换到工件坐标系）自动识别面向激光的平面，取该平面**所有** Wire（外边界 + 每个内孔）。
- **策略二（同步）**: 手动选面作为补充（3D 视图中拾取工件面）。
- **策略三**: 策略枚举作为分发框架（Auto / Planar / Tube / Manual / Legacy）。

---

## 2. 新增/修改的文件清单

### 2.1 核心算法层 (`src/core/algorithms/cam/`)

| 文件 | 变更 | 说明 |
|------|------|------|
| `laser_toolpath.h` | 新增 | `ExtractionStrategy` 枚举、`ContourKind` 枚举、`ContourExtractionParams` 扩展、`selectMachiningFace`/`isPlanarFace`/`extractContoursFromFaces`/`computeFaceSignature` 声明 |
| `laser_toolpath.cpp` | 修改 | 策略分发、`selectMachiningFace`（beam 对齐+面积排序）、`extractContoursFromFaces`（bbox 对角线外边界检测、孔优先排序）、`computeFaceSignature`（确定性面哈希） |

### 2.2 运动学层 (`src/core/kinematics/`)

| 文件 | 变更 | 说明 |
|------|------|------|
| `machine_kinematics.h/.cpp` | 修改 | 新增 `nominalBeamDirectionMachine()`（gp_Dir(0,0,-1)）、`computeWpcTransformHome()`（回转轴归零的 wpc 变换）。`axisLocalTrsf`/`chainTrsf` 增加 `home` 参数。 |

### 2.3 项目核心数据层 (`src/core/project/cam/`)

| 文件 | 变更 | 说明 |
|------|------|------|
| `cam_data_manager.h` | 修改 | 新增 `MachiningFaceRecord`（OCC-free 面记录），加入 `GenerationParams::extractionStrategy` |
| `cam_toolpath_io.cpp` | 修改 | 保存/加载 `[machiningFaces]` 面签名记录、`generation.extractionStrategy` 参数 |

### 2.4 CAM 模块层 (`src/modules/cam/`)

| 文件 | 变更 | 说明 |
|------|------|------|
| `cam_module.h/.cpp` | 大幅修改 | `extractionStrategy()`/`setExtractionStrategy()`、`beamDirectionWpc()`、加工面管理 API（`addMachiningFace`/`removeMachiningFace`/`clearMachiningFaces`/`setAutoMachiningFaces`/`manualMachiningFaces`/`machiningFacesForTree`）、AIS 半透明高亮（`refreshMachiningFaceDisplay`）、面拾取（`pickMachiningFace`）、面记录推送（`pushMachiningFaceRecordsToCamData`）、加载时签名重绑（`rebindMachiningFacesFromRecords`）、`machiningFacesChanged` 信号 |
| `settings/cam_config.h/.cpp` | 修改 | 新增 `extractionStrategy` 持久化到 `cam.toml` |
| `ui/widget_toolpath_panel.h/.cpp` | 修改 | 4-option 下拉（Auto/Planar/Tube/Manual）替代原有的 2-option combo |
| `commands/commands_cam.h/.cpp` | 修改 | 新增 `CmdSelectMachiningFace`（beginFacePick）、`CmdClearMachiningFaces` |

### 2.5 App 层 (`src/app/`)

| 文件 | 变更 | 说明 |
|------|------|------|
| `main_window.cpp` | 修改 | 连接 `machiningFacesChanged`→`rebuildProjectExplorer`、`extractionStrategyChanged`→`setExtractionStrategy`、facePick 路由、加工面节点右键菜单（删除/清空） |
| `project_explorer_model.h/.cpp` | 修改 | 新增 `MachiningFaceRoot`/`MachiningFace` 节点类型、`machiningFaceId` 字段、`appendMachiningFaceSection` |
| `project_explorer_tree_utils.h/.cpp` | 修改 | 新增 `MachiningFaceId` Role、`MachiningFaceRoot`/`MachiningFace` 图标 |

### 2.6 测试

| 文件 | 变更 | 说明 |
|------|------|------|
| `tests/contour_extraction_test.cpp` | **新增** | 多孔板（3孔）→4 轮廓（1 外+3 孔，孔优先）；Auto 自动选顶面；管材回归测试 |
| `tests/cam_lead_in_test.cpp` | 修改 | `useFaceClassification`→`strategy = ExtractionStrategy::TubeClassification` |

### 2.7 构建

| 文件 | 变更 | 说明 |
|------|------|------|
| `CMakeLists.txt` | 修改 | 注册 `lcnc_contour_extraction_test` target |
| `CLAUDE.md` | 修改 | 新增 CAM 轮廓提取策略章节，更新构建说明 |

---

## 3. 架构设计

### 3.1 策略枚举

```cpp
enum class ExtractionStrategy {
    Auto,                // 机台+姿态驱动：beam 方向→加工面→所有 Wire
    PlanarFaceWires,     // 同上但仅做平面面提取
    TubeClassification,  // 管材：外表面∩横截面
    ManualFaceSelection, // 手动选面
    LegacyOuterWire      // 旧版：每面取 OuterWire
};
```

### 3.2 轮廓类型标记

```cpp
enum class ContourKind {
    OuterBoundary    = 0,  // 加工面外边界
    TubeCrossSection = 1,  // 管材横截面轮廓
    InnerHole        = 2,  // 通孔内边界
    Unknown          = 3   // 未分类
};
```

### 3.3 加工面识别流程

```
机台构型 (MachineKinematics)
    │ nominalBeamDirectionMachine() = gp_Dir(0,0,-1)
    │ computeWpcTransformHome(wpcEntry) — 回转轴归零位姿
    ▼
beamDirectionWpc = nominalBeamDirectionMachine · wpcHome⁻¹
    │
    ▼
selectMachiningFace(workpiece, beamDir)
    │ 1. 找出所有平面 (isPlanarFace)
    │ 2. 计算面外法向与 beam 方向的对齐度 (dot < -0.7)
    │ 3. 按面积降序排列 → 取最大面
    ▼
extractContoursFromFaces(workpiece, {machiningFace})
    │ 1. 取每个面的所有 closed wire
    │ 2. bbox 对角线最大者为 OuterBoundary
    │ 3. 其余为 InnerHole
    │ 4. 按 contourCutOrderRank 排序：孔优先，外边界最后
```

### 3.4 加工面持久化流程

```
── 生成刀路 ──► pushMachiningFaceRecordsToCamData()
                      │ │ computeFaceSignature(face)
                      │ │    = hash(面类型 + 面积 + 质心 + 外Wire顶点数)
                      │ │ ▶ CamDataManager::m_machiningFaceRecords
                      │
                      ▼ saveCamToolpath ──► cam_toolpath.toml [machiningFaces]

── 打开工程 ──► loadCamToolpath ──► m_machiningFaceRecords
                      │
                      ▼ onCamDataLoaded
                      │
                      ▼ rebindMachiningFacesFromRecords()
                      │   1. 遍历工件所有面，计算签名
                      │   2. 与记录签名匹配 → 重绑 TopoDS_Face
                      │   3. 不匹配的面丢弃（WARN 日志）
                      ▼ refreshMachiningFaceDisplay()
```

### 3.5 引刀线验证契约

每个轮廓生成/重算时，引刀线经过两级验证：

1. `setContourStart` / `setAutomaticContourStart` → 选择起点
2. `computeLeadInSolution` → 验证悬空方向（投影到加工外表面 + 实体分类确认材料侧）

若 `leadInSolution.valid == false`，生成失败并弹出错误提示。**此契约在同步/异步生成和单轮廓重算三条路径均已强制执行。**

---

## 4. UI 交互

### 4.1 刀路参数面板

4 选 1 下拉菜单：
- **Auto** (0) — 默认。机台驱动自动识别
- **平面** (1) — 针对平面工件
- **管材** (2) — 管材横截面提取
- **手动** (3) — 手动选面

### 4.2 项目树"加工面"节点

- **加工面** 根节点 → 展开显示已识别的面
- **自动面**（青色 AIS 高亮，透明度 0.6）— 生成刀路后自动捕获
- **手动面**（黄色 AIS 高亮，透明度 0.6）— 通过"选择加工面"命令添加
- 右键菜单：删除面 / 清空全部面
- 视图同步：点击树节点时对应面高亮

### 4.3 命令

| 命令 | 说明 |
|------|------|
| `cam.select_machining_face` | 进入面拾取模式，点击工件面添加到手动加工面 |
| `cam.clear_machining_faces` | 清空所有手动加工面 |

### 4.4 生成按钮行为

| 策略 | 生成前检查 | 生成后 |
|------|-----------|--------|
| Auto / Planar / Tube | 无特殊检查 | 自动捕获加工面到树中 |
| Manual | 需至少选择 1 个面，否则弹窗提示 | 保留手动选择的加工面 |

---

## 5. 验证结果

### 5.1 构建

- **LaserCNC.exe**: ✅ 编译通过，链接成功
- **所有 lib**: ✅ lcnc_core / lcnc_view / lcnc_module_cad / lcnc_module_cam / lcnc_module_process / lcnc_app

### 5.2 测试

| 测试 | 结果 |
|------|------|
| `lcnc_contour_extraction_test` | ✅ 通过 — 3 孔板 → 4 轮廓 (1 外+3 孔，孔优先)；Auto 自动选顶面；管材回归 |
| `lcnc_cam_lead_in_test` | ✅ 通过 — 引刀线测试已适配新策略枚举 |
| `lcnc_project_package_test` | ✅ 通过 — v4 包读写正常 |
| `lcnc_process_runtime_configuration_test` | ✅ 通过 — 运行时配置正常 |

### 5.3 合规检查

| 检查项 | 结果 |
|--------|------|
| 旧 API 使用 (`projectDocument`/`workspaceGuiDocument`/`ensureProjectDocument`/`sourceDocument`) | ✅ 无匹配 |
| Process 模块含 OCC 类型 (`TopoDS\|AIS_\|gp_\|Geom_\|BRep\|XCAF`) | ✅ 无匹配 |

---

## 6. 兼容性说明

- **项目格式**: 仍为 v4，新增 `[machiningFaces]` 段仅在加工面存在时写入。老工程加载时无该段，加工面为空（不报错）。
- **cam.toml**: 新增 `extractionStrategy` 键，默认值 0（Auto），老配置文件可正常加载。
- **cam_toolpath.toml**: `schemaVersion` 仍为 3，新增字段为可选，向后兼容。`[generation]` 新增 `extractionStrategy` / `appliedExtractionStrategy`。
- **API**: `useFaceClassification` getter/setter 保留，内部桥接到 `extractionStrategy`。旧调用方的 `useFaceClassification=true` 等价于 `strategy=TubeClassification`。
- **管材路径**: `TubeClassification` 策略与旧 `useFaceClassification=true` 行为一致。

---

## 7. 已知限制

1. **手动面跨 session 重绑**: 手动选择的加工面在保存/重新加载后，通过签名匹配恢复。若工件几何发生变化导致签名不匹配，面对会被丢弃（WARN 日志）。极端情况下（如面被编辑）需要用户重新选择。
2. **加工面仅针对平面**: `selectMachiningFace` 使用平面面 (planar face) 检测。曲面/自由曲面需要手动面选择。
3. **多工件源**: 每个 `WorkpieceSource` 独立识别加工面，树中合并显示。
4. **签名碰撞**: `computeFaceSignature` 使用确定性哈希（FNV-1a-like），理论上不同面可能产生相同签名，但实际概率极低（area + centroid + surface type + vertex count 四要素）。

---

## 8. 后续建议

1. **加工面编辑**: 允许用户修改自动识别的加工面（转为手动模式），调整面内轮廓排序。
2. **多加工面支持**: 当前仅提取 beam 对齐度最高的一个平面。复杂工件可能需要多个加工面（如侧铣）。
3. **面签名改进**: 可考虑加入更丰富的拓扑特征（如内孔数量/面积分布）以进一步降低碰撞概率。
4. **ASan 构建验证**: 在 ASan 预设下编译并运行测试，确保无内存问题。
