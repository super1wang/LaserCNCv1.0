# 自动化测试分层

测试优先级以安全关键算法和可重复的业务流程为主。日常全量验证仍使用：

```powershell
ctest --test-dir build-cmake --build-config Debug --output-on-failure
```

需要快速验证关键门禁时，可按标签运行：

```powershell
ctest --test-dir build-cmake --build-config Debug -L critical --output-on-failure
ctest --test-dir build-cmake --build-config Debug -L algorithm --output-on-failure
ctest --test-dir build-cmake --build-config Debug -L flow --output-on-failure
ctest --test-dir build-cmake --build-config Debug -L safety --output-on-failure
```

## 关键覆盖

- `algorithm`：面分离、轮廓提取、点离散、引刀、轮廓排序、机床运动学、空程规划、碰撞预筛和 OCCT 精确运算并发门禁。
- `flow`：工程包打开/保存、真实 `model/半球.stp` 打开、CAM 五阶段流水线、非机台工件代理碰撞、机台模式安全契约、默认加工工作流。
- `safety`：任务取消、设备命令队列、加工前检查、轮廓边界安全、状态监控、连接与手动运动、快照并发。
- `sdk-integration`：ACS 文本命令和 `SimulatorCMHP`。它证明 SDK 模拟器路径，不等同于实体硬件验证。
- `support`：架构之外的 UI、翻译、显示和启动冒烟等辅助回归。

## 流程测试边界

`lcnc_manufacturing_flow_test` 由多个独立 CTest 用例调用同一程序：

1. 通过 `LcncProjectManager` 打开真实 STEP 文件，并验证 Workpiece/CAM 统一文档。
2. 对同一真实模型执行面分离、轮廓提取、点离散、几何刀路和 XYZ 机床求解，并验证阶段 revision 链完整。
3. 在无机台模型时，用切割嘴代理对真实工件执行空程碰撞规划。
4. 在机台模式时，验证 `FullEnvironment` 模式选择，以及碰撞校验处于 Pending、Safe、Collision 时 Process 的禁止/放行契约。

完整机台多刚体扫描的几何基础由碰撞预筛、精确运算锁和运动计划契约测试覆盖；真实 AC/BC/摆头装配的碰撞对、连续扫掠和实体硬件低速无激光验证仍属于设备验收，不能由上述自动化用例替代。

## 用例维护原则

- 一个测试程序可以承载同一领域的多个紧密相关断言；需要独立定位的流程阶段通过命令参数注册为不同 CTest 用例。
- 小型状态机、策略排序和 DTO 契约优先合并进相邻的安全/算法测试，避免为数行断言启动独立进程。
- 临时几何排障工具不进入默认构建和 CTest；稳定复现的问题应提炼成有明确成功条件的回归用例。
- 不以 GUI 冒烟或 SDK 模拟器结果声称实体机加工安全。
