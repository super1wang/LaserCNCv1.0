# CAM 模块目录约定

`CamModule` 是模块的公共外观和生命周期所有者。各项功能的实现文件应放入拥有该能力的目录：

- `contracts/`：跨模块公开接口、事件和不含 OCC 类型的 DTO。
- `integration/`：将 CAM 能力注册到 Kernel 的适配器。
- `internal/`：多个 `CamModule` 实现单元共享的私有辅助代码，不作为扩展接口。
- `pipeline/`：加工面选择和分阶段 CAM 生成流水线。
- `toolpath/`：刀路生成、排序、版本投影和机床坐标求解。
- `collision/`：碰撞几何准备、缓存和验证。
- `machine/`：机床模型导入导出、轴识别、标定和机床工作区编排。
- `display/`：CAM 到视图的投影和显示状态编排。
- `interaction/`：几何拾取等可复用视图交互工具。
- `commands/`：按调用功能分类的命令。
- `ui/`：按功能区域分类的控件和对话框。
- `settings/`：CAM 配置持久化。

只有其他模块需要的稳定能力可以放入 `contracts/`。不依赖模块状态的算法应进入对应功能服务或
`core/algorithms/`，不得继续堆入 `cam_module.cpp`。面向 Process 的契约必须保持不含 OCC 类型，
并在消费边界提供不可变快照。
