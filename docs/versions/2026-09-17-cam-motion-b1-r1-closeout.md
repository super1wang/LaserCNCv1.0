# CAM Motion v3.2.1 B1/R1 收口

日期：2026-09-17  
分支：`codex/cam-motion-v3.2.1-b1`

## 范围

- 修复 source/block 结构边界与激光状态、required stop 的语义混用。
- 为 FinalMotionPlan 增加布局、active mask、motion class、source span 和事件顺序校验。
- 为公共 CAM provider 增加配置代际、travel key、当前 authority、异步编译、取消和迟到结果校验。
- 保留 exact planHash 的碰撞 proof 更新，不重新选择运动候选。
- 更新 B1 R1 收口、验证矩阵和状态记录。

## 验证

- Ninja Debug 构建通过。
- ASan 构建通过。
- B1 Gate 定向集合 Debug 12/12 通过；ASan 12/12 通过。
- 补强后的 CAM pipeline 单项 Debug 1/1 通过。
- 架构检查与 `git diff --check` 通过。

## 边界

R1 仅代表软件收口与 B1 Stage Gate 证据。生产 controller qualification 仍为
`Unavailable / revision 0`，默认 policy 为 Off；process Z whitelist、B2 lowering、HIL、
实机和生产碰撞资格不在本次范围内。R1 仍待人工复核，B2 不因本提交自动放行。
