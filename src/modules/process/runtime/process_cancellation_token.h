#pragma once

// 历史命名兼容：原 `ProcessCancellationToken` 已升级为统一中断上下文
// `ProcessInterruptContext`，并通过 `using` 别名暴露原名。
// 公共原子字段 `paused / stopRequested / emergencyStop` 仍可直接 .load()，
// 新代码请使用 ProcessInterruptContext::checkpoint() / noteCheckpoint() /
// hasResumePoint() / resumePoint() 等 API。

#include "modules/process/runtime/process_interrupt_context.h"

namespace lcnc::process {

using ProcessCancellationToken = ProcessInterruptContext;

} // namespace lcnc::process
