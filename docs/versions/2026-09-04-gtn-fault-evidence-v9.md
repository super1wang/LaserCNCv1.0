# GTN v9：故障位置保留与真实诊断日志

## 已确认的问题

现场日志在 14:45:12.867 记录物理轴 3 的规划位置 -41734.43087768555 pulse、编码器位置 -37656 pulse；14:45:12.981 的 StopMotion 随后调用 SetPrfPosEx，把编码器值写回规划位置。这解释了报错后固高界面两列都显示 -3.765600，而运动指令中没有该目标。

## 本次修复

- 首次反馈故障的轴号、原始规划值、编码器值、换算比例和单位冻结保存；后续 Group 释放与停止不覆盖首次故障。
- 反馈故障锁定期间，StopMotion/StopMotionAxis 执行停止但跳过规划位置同步。使能入口也不允许通过隐式位置同步覆盖证据。普通非故障停止仍保留原有同步行为，并明确记录写前值、反馈来源、请求写入值及 API 返回值。
- Group 状态读取失败必须作为错误传递，不能误报执行完成。
- 界面区分反馈校验失败、控制器停止码和 API 读取错误。反馈提示包含轴号、规划、反馈、偏差、容差。
- 配置加载记录实际路径及 SHA-256。批次启动和故障边界记录原始规划/编码器位置、映射后轴位置、速度、控制器时钟及各 API 返回值。读取是顺序采样，不宣称原子同时快照。
- 故障清理前记录 Group/CommandList 状态、执行段、最后提交段、最后提交目标和 RTCP 状态；最后提交目标不等于发生故障时已执行的段目标。

## 日志检索

`gtn.api:` 下重点筛选：

- `phase=configuration_file`：实际配置来源。
- `phase=position_evidence`：只读边界快照，read_results 顺序为 GetPrfPos、GetEncPos、GetAxisPrfPos、GetAxisEncPos、GetPrfVel、GetEncVel；仅返回 0 的字段可用于判断。
- `phase=feedback_mismatch`：触发校验的原始值。
- `phase=fault_snapshot`：清理前批次状态与提交目标。
- `phase=profile_sync action=skip`：故障停止保留原规划值。
- `phase=profile_write`：真正的坐标寄存器改写，不是运动命令。

## 保留的现场验证门槛

本次没有改变脉冲当量、编码器方向、电子齿轮、反馈容差或 500 ms 稳定窗口，也没有操作真实机台。两次 X 增量的反馈/规划比约 1.04987 仍需使用新快照和现场实测判断来源，不能把软件构建与单元测试当作运动精度合格证明。反馈异常时继续阻止加工；停止或复位不是允许忽略故障继续加工的入口。

## 验证结果

- Ninja Debug 全构建通过；最终补丁后重新构建应用、切割安全测试和翻译测试通过。Debug 的 14 项相关 CTest 全部通过。
- Ninja Release 应用及切割安全、设备队列、翻译测试目标构建通过；连同架构检查的 4 项定向 CTest 全部通过。
- 首次故障值保留、后续样本不覆盖、毫米/角度提示和中文翻译已加入回归。未执行真实 GTN 机台运动或全量 CTest。
- 已核对所用新诊断读取接口存在于当前 gts.dll 导出表；Release 可执行文件包含 `group-fault-evidence-preserved-v9` 标识。
- 本次源文件差异检查通过；供应商头文件中原有空白差异不在本次修改范围。

输出：`F:/wangchao/LaserCNC/x64/ninja/Debug/LaserCNC.exe` 与 `F:/wangchao/LaserCNC/x64/ninja/Release/LaserCNC.exe`。
